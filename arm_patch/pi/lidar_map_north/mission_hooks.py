"""整场任务钩子：接在 auto_run.drive 每个停车点上(hooks.task)。

路线(配置里的 mission)： QR -> RAW -> ROUGH -> TEMP -> RAW -> ROUGH -> TEMP -> START
  QR     读任务码(STM32 的 GM65 扫码模块)，解析，显示到串口屏
  RAW    原料盘：按任务码的颜色顺序，用摄像头找到物料，闭环对准，夹起放进车上转盘 1/2/3 号槽
  ROUGH  粗加工区：把转盘里的物料依次放到指定圆环(摄像头对准圆环中心)，再按同样顺序取回转盘
  TEMP   暂存区：第一批平放在指定圆环；第二批码垛在同色物料上
  START  回到启停区：收臂，串口屏显示抓取/放置数量

放圆环的办法(为了准)：
  1 底盘沿车头方向挪到目标圆环旁边(圆环间隔 150mm 的整数倍，位置已知)
  2 手臂摆到观察姿态，爪子是空的，圆环不会被挡住；摄像头闭环对准圆环中心
  3 记下这时 ID1/ID2 的角度，去转盘取物料，再回到记下的角度(舵机回到同一角度的重复精度高)，下降、松手
  (10-12 起默认 align_by=wheels：第 2 步手臂摆到观察姿态就不动，只微调车轮(横移 + 前后)把圆环对到爪子点；原料区也一样)
取回(粗加工区)和码垛(暂存区第二批)时圆环白心被物料盖住了：直接认物料顶面，把它对到 claw_px.PICK
(物料在爪子正下方时顶面在画面里的位置；第一次放下物料后自动量一次，存进 vision_cal.json)；
认不到物料(黑色物料在黑环上、还没量过 PICK)再认圆环；都看不到时取回按放下时记下的位置直接夹。
码垛的下降高度浅一个物料。

出错处理：
  - 机械臂还没标定(ARMOK=0)、升降没回零：整轮不做夹放，路线照走，并在串口屏提示
  - 单个物料对不准或出错：跳过这个物料，收臂，继续下一个
  - 停车点里出了意外的错(程序 bug 之类)：这个停车点不再做，收臂、挪回停车点，路线照走(不结束这一轮)
  - 收到急停(abort)、收臂失败、底盘挪不回停车点(车在哪不知道了)：抛 Abort，路线停止
  - 时间不够(按每个物料的用时 + 回家的时间算)：不再开始新的物料，记下 out_of_time，让导航直接回家(不抛异常)

给导航(auto_run / map_merge_live)用的接口(导航都先 hasattr 再调)：
  prepare(link, log)   一键出发后马上调：机械臂初始化 + 收臂、同步参数、清掉 STM32 里的旧任务码、屏上任务码写 ---、打开摄像头
  start_clock()        比赛计时从现在开始(按下屏上 START 的那一刻)
  time_left()          离 time_limit_s 还有多少秒
  go_home_now(下一个停车点, 开过去要几秒, 从那里回家要几秒) -> True = 去了就来不及回家，从这里直接回家
  set_home_eta(秒)     导航告诉钩子：从现在这个停车点回家要多久(停车点里每个物料开始前按它判断来不来得及)
  keepalive()          车长时间不动时手臂轻轻摆一下(规则：停止运行 15 秒本轮结束)
  park()               收臂、升降停到 60mm(跑完回到启停区、退出程序时；急停以后不自动做)
"""
import json
import math
import os
import time

import numpy as np

from arm_link import ArmLink, ArmActuators, ArmError, ArmAbort, unknown_cmd
from task_plan import parse_code, TaskError, role_of, Stats, COLOR_NAMES
from visual_servo import VisualServo, JacStore
from vision import Vision, VisionError, apply_vision_cal, load_vision_cal

try:
    from auto_run import Abort
except Exception:                       # 单独测试时没有 auto_run
    class Abort(Exception):
        pass


class DroppedError(ArmError):
    """(10-23)放置时已经在环上松了手，抬起来才出的错：物料已经放下了，不能当还夹着放回转盘(放回去程序会以为物料在车上、环是空的)。"""


RAW_CAR_FIXED = True   # (10-11 用户：写死)原料区：不挪车找物料(配置里的 raw_seek / raw_chassis / raw_wheels_first 都不管)。
                       #   10-12 起 align_by.RAW='wheels'(默认)时对准只动车轮(横移限 raw_wheels_s_range、前后限 raw_fix_max_mm)，手臂不动；
                       #   align_by.RAW='arm' 时是 10-11 的做法：车不动，只动手臂

DEFAULTS = dict(
    enabled=True,
    # (10-12 用户："在对每个任务点进行任务的时候要求通过微调轮子进行对准和对齐，不要调控机械臂了，太慢了")
    # 每个任务点怎么对准：'wheels' = 手臂摆到观察姿态就不动，只微调车轮(横移 + 前后，整毫米、慢速)把目标对到爪子下面；
    #   'arm' = 以前的做法(手臂 ID1/ID2 闭环对准)。原料区 / 粗加工区 / 暂存区各自设
    align_by=dict(RAW='wheels', ROUGH='wheels', TEMP='wheels'),
    place_anyway_mm=10.0,               # 只动车轮对准完还差这么多毫米以内：照样放/夹(物料留在车上 = 0 分)。只有看不到目标、
                                        #   差得更多(会放到两个环中间)、或者会砸到别的物料才不放
    raw_wheels_s_range=[-15.0, 15.0],   # 原料区只动车轮时横移累计允许的范围(毫米，相对原料区停车点；负 = 往右 = 原料盘那边)，不压进原料区。
                                        #   前后累计不超过 ±raw_fix_max_mm。原料区做完车挪回停车点(路线从停车点接着走)
    full_course=True,                   # (10-11 用户：车一定要跑完全程)True = 不因为时间、没任务码、没活可干提前回家，每个停车点都去；
                                        #   时间也不限制开始新的物料(每个物料最多等 raw_track_s)。比赛要 3 分钟内回家时改 false
    time_limit_s=150.0,                 # 每轮 3 分钟；超过它就不再做夹放，留出时间让车开回启停区(一次路线最后一段大约 20~30 秒)。
                                        #   每个物料开始前还要来得及在这之前做完(按实测的每个物料用时)
    round_s=180.0,                      # 一轮多长(赛规 3 分钟)：导航告诉了回家要多久(set_home_eta)时，按它算还来不来得及
    home_margin_s=12.0,                 # 算"来不来得及回家"时再多留这么多秒(路线时间估得不准、回家前雷达定位、回家对准)
    stop_finish_s=3.0,                  # 停车点做完物料以后收臂、底盘挪回停车点大约要多久
    home_when_idle=True,                # 后面再也没有夹放可做(没读到任务码、机械臂不能用)：go_home_now 回 True，导航直接回家(少走路、少压线)；
                                        #   false = 照样把路线走完(只测路线时用 enabled=false 更直接)
    work_item_s=dict(grab=12.0, place=10.0, pick=8.0, stack=10.0),   # 每个物料大约要多久(抓/放/取回/码垛；按模拟的提速参数估的)。
                                        #   跑过一次就按实测的(每次都更新)，这里只是还没实测时用
    work_base_s=dict(QR=3.0, RAW=2.0, ROUGH=6.0, TEMP=5.0, START=3.0),   # 每个停车点物料以外的用时(看圆环、收臂、挪回停车点)
    next_leg_s=12.0,                    # 粗加工区开到暂存区大约要多久(粗加工区放完以后判断"取回了还来不来得及去暂存区"用)
    qr_timeout_s=6.0,                   # 到 QR 点最多等多久读码(qr_search=false 时；前后挪着找时每个位置等 qr_dwell_s)
    qr_search=True,                     # 到 QR 点读不到码：车沿车道前后挪，让扫码器依次对着码板可能的位置再读
    qr_first_wait_s=0.6,                # 到 QR 点先原地查这么久(路上可能已经读到了)
    qr_targets_mm=[1200.0, 1260.0, 1140.0, 1320.0, 1080.0],     # 扫码器依次对着码板的这些位置(沿码板方向的场地坐标；码板中心每场随机在 1100~1300)
    qr_dwell_s=0.8,                     # 每个位置停多久查 QR?
    qr_board_axis='y',                  # 码板沿哪个方向摆(在东墙上：y)
    qr_scanner_from_rear_mm=20.0,       # 扫码器装在车的右后方：离车尾多少毫米
    qr_scanner_from_right_mm=40.0,      #   离车右边多少毫米(车 290x260：在车中心后 125、右 90)
    qr_max_move_mm=300.0,               # 找码时离停车点最多前后挪多少
    qr_obstacle_margin_mm=60.0,         # 找码挪车时车身(含雷达)离障碍物至少多远，不够就跳过那个位置
    qr_zone_margin_mm=30.0,             # 找码挪车时车身离黄色区、工位、原料转盘、场地边至少多远
    qr_move_rpm=None,                   # 找码时底盘挪的速度(转/分)；None = 和视觉微调一样(60mm 以内 chassis_fine_rpm，再远用 STM32 默认)
    qr_retry_at_stops=True,             # 到 QR 点也没读到：后面每个停车点再问一次 QR?(STM32 可能在路上读到了)
    code_plus=True,                     # 屏上任务码带上两组之间的 +(t0 = '156+123+'，t7 = '516+231')。只在新 STM32 程序上带
                                        #   (LIFT? 回 BOOT=…：t0 加宽到放得下 8 个大字)；旧程序 t0 只有 7 个字宽，自动不带(不然两头被裁掉)。false = 一律不带
    raw_wait_s=10.0,                    # 等原料盘停稳最多多久(规则：等转盘最多多停 8 秒)
    raw_chassis='off',                  # 原料区对准时车轮能不能动：'off' = 只动手臂，车轮不动(不会压进原料区)；
                                        #   'F' = 只许沿车头方向前后挪，最多 raw_fix_max_mm；'SF' = 前后、横着都能挪(以前的做法，可能压进原料区)
    raw_fix_max_mm=40.0,                # raw_chassis='F' 时一次对准车轮最多前后挪多少毫米；align_by.RAW='wheels' 时前后累计离停车点最多这么多
    raw_wheels_first=False,             # (10-10 用户：原料区只动爪子)True = 原料区对准先动车轮(沿车头方向前后小步慢慢挪，每次最多 wheels_step_mm、wheels_rpm 转/分)，
                                        #   差不到 wheels_min_mm 再动手臂；车轮不横移(不会压进原料区)。要先做过 vcal RAW 颜色号(不加 arm)，
                                        #   存了车轮和画面的对应关系才有用，没有就只动手臂。False = 只动手臂(以前的做法)
    qr_prescan_s=60.0,                  # (10-11 用户：直接扫码)开跑前这么多秒以内手拿码板对着扫码器扫到的码留着用(0 = 开跑时清掉)
    qr_fixed_code=None,                 # (10-11 用户)手动给任务码，例如 "652+312+526+231"：不扫二维码，到 QR 点直接用它(屏上照样显示)；null = 正常扫码(比赛用 null)
    raw_track_s=40.0,                   # (10-11 用户：没夹到就接着做下一个)等要抓的物料转过来、停在爪子下面最多等多久(秒)，到了就跳过做下一个
    raw_stop_s=5.0,                     # 原料盘每次停多久(秒)，没看到它完整停过一次之前先按这个算(保守一点)；看到以后按实测的
    raw_still_px=6.0,                   # 这一帧物料位置和前两帧比变了不到这么多像素 = 没在动(转盘转的时候一帧要走二三十像素)
    raw_still_deg=3.0,                  # (10-21)对准原料盘中心时(车在走)：物料绕圆心转了不到这么多度 = 原料盘没在转(转的时候一帧要转二十度左右)
    raw_motion_frac=0.004,              # 前后两帧画面(爪子以外)变了的地方超过这个比例 = 原料盘在转
    raw_static_s=9.0,                   # 还没做 vcal RAW(没有手臂和画面的对应关系)时：看到原料盘连续这么多秒都不转(关了)，才现场小幅动几下测
    raw_keepalive_s=0.0,                # 等物料转过来时手臂这么多秒没动过，就升降上下动一下(规则：机器人停止运行 15 秒、等转盘时 23 秒本轮结束)；0 = 不动
                                        #   (10-24 用户"把这个每隔 15s 动的这个动作给去掉"：12 -> 0)
    raw_keepalive_deg=1.0,              # 摆多少度(ID1，摆过去马上回来)。(10-21 用户"突然抽搐一下，先向右动一点然后快速回来")默认不再摆 ID1，见 keepalive_lift_mm
    keepalive_lift_mm=4.0,              # (10-21)车不动太久时改成升降往上 4mm 再回来(手臂、爪子不左右动，升降是步进电机，回得准)；0 = 照以前摆 ID1
    raw_grab_fix=False,                 # (10-21 用户"对齐的不够快不够准，原料会夹取不上")最近那格：要的颜色停到这一格、下爪之前，拍两帧看它离爪子点
                                        #   差多少，差 raw_grab_fix_mm 以上就车轮挪一下(手臂不动)，等画面跟上(raw_grab_fix_settle_s)再看。
                                        #   (10-21 夜用户"对准一次之后就应该不动了啊，后面为什么还是乱动")默认关：对准以后车不动，只在日志里记物料偏多少
    raw_grab_fix_mm=1.5,
    raw_grab_fix_max_mm=25.0,           #   离对准原料盘中心的地方一共最多挪多少毫米(差更多多半是认错了，不挪)。10-21 以前 15(差 16mm 的就没补)
    raw_grab_fix_moves=3,               #   最多挪几次
    raw_grab_fix_s=1.5,                 #   挪一次大约要多久(秒)：原料盘这次停下剩的时间不够就不挪，按现在的位置夹
    raw_grab_fix_settle_s=0.3,          #   (10-21 实车：挪了 10mm 再看还差 9mm)挪完先等这么久再拍(停稳、画面跟上)
    raw_grab_fix_trust=0.5,             #   (10-21)挪完画面里物料要跟着动(按车轮和画面的对应关系)至少这么多：动得太少 = 认的位置不对
                                        #   (多半被爪子挡住、和蓝色爪子连在一起)，不再按它挪，车轮挪回去按原来对准的位置夹
    raw_grab_fix_back=False,            #   (10-21 用户"对准的时间太长")夹完车轮不挪回去(下一个照样先看它停在哪再补)；True = 以前的挪回去
    raw_near_any_order=False,           # (10-22 用户"就是要全部按照这个任务码的顺序来放置和夹取")不再用：原料区一律按任务码顺序夹，
                                        #   配置里写着 true 也不用(见 _grab_any_order)。只想这一次不按顺序(测试用)：mtest RAW 1 any
    raw_near_regrab=1,                  #   (10-21 实车"一会儿夹取一会儿不夹取")夹过的颜色还在原料盘上(刚才没夹上)：等它转到这一格再夹几次(0 = 不再夹)
    raw_near_check_last=False,          #   这一批都夹过以后再看 raw_near_check_s 秒原料盘：夹过的还在上面就再夹(最后一个没夹上也看得出来)。
                                        #   (10-26 用户"最后一个夹取完了把爪子缩回来就完事了啊还把爪子转过去干嘛呢")默认不看：最后一个夹完
                                        #   手臂不再转回原料盘上方，直接收臂。True = 以前的再看一眼
    raw_near_check_s=3.0,
    zone_anchor=True,                   # (10-26 用户"后面第二轮运行……还能在同样的地方吗")工位/原料区对准了(2 号环/原料盘中心)，
                                        #   和第一次来对准时车轮挪的差多少，这一次停的位置就和第一次差多少(一路车轮打滑累计的)；
                                        #   做完挪回停车点时多挪这一点，回到第一次停的地方，路线接着按停车点算(误差不再一轮轮累计)
    zone_anchor_min_mm=8.0,             #   差得比这少不多挪(对准本身就有几毫米的误差)
    zone_anchor_max_mm=60.0,            #   差得比这多不信(多半认错了)：照旧挪回这一次停的地方，日志说一句
    raw_frames=2,                       # 原料区对准时每次测量用几帧(一帧最快，停下来的时间有限)
    raw_max_iter=1,                     # (10-11 用户：不要一直校准)原料区对准最多修正几次，以前 3
    raw_gain=1.0,                       # 原料区对准每次修正偏差的多少(用 vcal RAW 测好的 J，一下修到位，不留余量)
    raw_force_grab=True,                # (10-11 用户：一定要夹)修完偏差还大、越对越偏、停的时间到了：只要偏差不超过 raw_force_max_mm 照样下爪
    raw_force_max_mm=60.0,
    raw_grab_max_mm=30.0,               # (10-11 用户：误差 30mm 以内都可以夹)修正完按当时的偏差下爪，只要不超过这个。以前 5
    raw_tol_mm=3.0,                     # 原料区对准到多小就马上下爪(夹爪合上时会把物料夹正，不用像放圆环那么准)
    raw_follow=True,                    # (10-16 用户)原料区"跟随抓取"：手臂停在看得到整个原料盘的高度(ZOBRAW)，挑一个要夹的颜色，原料盘转的时候
                                        #   手臂跟着它转(ID1/ID2，用 vcal RAW 颜色号 arm 测的 J)，原料盘一停马上下爪；(10-22 起)按任务码顺序，只跟下一个要夹的。
                                        #   要有手臂的 J(没有就照旧等物料停到爪子附近)。False = 以前的做法
    raw_follow_limit_deg=dict(id1=25.0, id2=250.0),   # 跟随时手臂相对 OBS RAW 姿态最多转多少度(超出就换一个够得着的物料跟；ID2 还受 STM32 里的限位)
    raw_follow_deadband_mm=3.0,         # 物料离爪子点不到这么多毫米就不动手臂(省得来回抖)
    raw_follow_lead_s=0.35,             # 原料盘在转时往前多跟这么多秒(物料的速度 × 这个；手臂转过去要时间)
    raw_follow_gain=0.9,                # 跟随时每次修正偏差的多少
    raw_center=True,                    # (10-16 用户)原料区：①车轮一小步一小步挪，让爪子(看原料盘的姿态)对准原料盘中心(用画面里三个物料的位置算圆心)；
                                        #   ②认出三个物料停下的位置(停靠位)，算好手臂转到每个位要多少度；③原料盘每停一次，看一眼哪个颜色停在哪个位，
                                        #   手臂直接转过去，拍一帧修一下就下爪，不一直跟着识别。认不出圆心/停靠位时默认不抓(见 raw_center_fallback)
    raw_center_tol_mm=4.0,              # 爪子离原料盘中心不到这么多毫米就算对准了
    raw_center_outer=True,              # (10-16 用户：照抄开源方案)认原料盘中心用原料盘(转盘)的外圆(turntable_detector.py)；认不出外圆时再用物料位置算
    raw_orbit_ratio=0.77,               # 还没看到物料时：物料所在圆的半径 ≈ 外圆半径 × 这个
    raw_center_smooth=True,             # (10-16 用户)车轮连续、顺滑地慢慢走，一边走一边每帧算圆心(STM32 的 V 指令)，不再挪一下停一下；
                                        #   STM32 是旧程序(不认 V)时自动改回一步一步挪
    raw_center_speed=15.0,              # 对准中心时车轮最快多少毫米/秒(边走边看，实车一秒不到 2 帧：再快存的对应关系反了时来不及发现)
    raw_center_search_speed=30.0,       # 找中心(挪着找)、只用 V 挪到一个地方(_vel_goto)时车轮最快多少毫米/秒(10-21：20 -> 30)
    raw_center_align_s=25.0,            # 对准中心最多用多少秒
    raw_center_iters=5,                 # 车轮最多挪几次
    raw_center_step_mm=20.0,            # 车轮每次最多挪多少毫米(慢慢挪，挪一次看一次)
    raw_center_s_mm=40.0,               # 对准中心时车轮横移最多离原料区停车点多远(毫米)
    raw_center_f_mm=60.0,               # 对准中心时车轮前后最多离原料区停车点多远(毫米)
    raw_center_probe_mm=15.0,           # 还没有车轮和画面的对应关系时：先前后、横着各挪这么多测一下(测完存进 servo_cal.json，以后不用再测)
    raw_center_wait_s=12.0,             # 认圆心最多看多久(秒)
    raw_center_search_mm=[[-20, 0], [-40, 0], [-40, 30], [-40, -30], [0, 30], [0, -30]],
                                        # 停车点认不出圆心：车轮依次挪到这些位置(相对原料区停车点 [横移, 前进] mm；横移负 = 往右 = 原料盘那边)
                                        #   每到一处看一会儿，认出圆心就停下开始对准；都认不出才放弃
    raw_center_colors=None,             # 认圆心时看哪些颜色的物料；None = 这一批任务码里的三个颜色(原料盘上放的就是它们)
    raw_center_reach_deg=dict(id1=60.0, id2=700.0),   # 停靠位离看原料盘的姿态超过这么多度就算够不着(另外还不能超出舵机限位)
    raw_grab_fixed=True,                # (10-16 用户，和开源方案的"抓取区"一样)认出圆心和三个停靠位以后，车轮再慢慢把爪子对到一个停靠位(抓取位)上，
                                        #   然后车、手臂都不动：原料盘每停一次，停在抓取位的是要的颜色就直接下爪(不用手臂去够别的位)
    raw_center_min_speed=5.0,           # 还没对准时车轮最慢多少毫米/秒(太慢电机按 0 转/分算会不动)
    raw_center_fallback='none',         # 没对准中心/没认出停靠位时：'none' = 这次不抓(不改用跟随)；'follow' = 改用跟随抓取
    raw_center_force_mm=12.0,           # 停靠位抓取：修完偏差还超过这么多毫米就不硬夹(等下一次停)，免得砸到物料
    a1_range=None,                      # ID1 限位 [最小, 最大](度)；None = STM32 的 S1MIN ~ S1MAX(旧程序 A1_MIN ~ A1_MAX)
    raw_follow_smooth=True,             # (10-16 用户)跟随时手臂一直平滑地动：每帧读一次实际角度、发一个新目标(APN)就接着看，不等转到；
                                        #   False 或 STM32 是旧程序(不认 APN) = 以前的"转一下、停住、看一眼"
    raw_follow_track_ms=250,            # 平滑跟随：每个新目标大约多少毫秒转到(小 = 跟得紧但容易抖，大 = 稳但跟得慢)
    raw_slots=True,                     # (10-16 用户)原料盘的物料每次停在固定的 3 个停靠位：示教过(rteach RAW 停靠位 颜色号)就按停靠位抓——
                                        #   看一次哪个颜色在哪个位、看它转一次知道往哪边挪，推算以后每次停下的颜色；手臂提前到够得着的停靠位上方等，一停就下爪。
                                        #   停偏了照样修一下再夹(跟随抓取的修正)；没示教就用跟随抓取
    raw_slot_match=0.4,                 # 认停靠位：物料离示教的画面位置不到"两个停靠位间距 × 这个"才算停在这个位上
    raw_slot_accept_mm=25.0,            # 手臂在停靠位上方等着：要夹的物料停在离爪子点这么近以内才算停到了这个位(再修一下就夹)
    raw_slot_wait_stops=4,              # 手臂在停靠位上方最多等几次停(推算错了就回去重新看)
    raw_ct=True,                        # (10-18 用户)原料区"对准圆心 + 示教的三个停靠位动作"：示教过(rteach RAW center、rteach RAW 1~3 颜色号)就用这个——
                                        #   手臂摆到看原料盘的姿态就不动，车轮边走边看、把原料盘中心对到示教时的位置(连续慢慢走，不走一下停一下)；
                                        #   对准以后车不再动：按任务码顺序，原料盘每停一次看三个停靠位上是什么颜色，下一个要的颜色停在几号位，
                                        #   就按几号位示教的手臂角度直接转过去下爪(不拿画面再修手臂)。没示教(或 False)就用上面以前的做法
    raw_ct_tol_mm=3.0,                  # 对准圆心：原料盘中心离示教时的位置不到这么多毫米算对准
    raw_ct_skip_stops=4,                # 按顺序等下一个颜色：连着这么多次停下都没在停靠位上看到它(原料盘上没有)，就跳过它做下一个
    raw_ct_check=True,                  # 转到停靠位上方、下爪前拍一帧：看到要夹的物料离爪子点超过 raw_slot_accept_mm(原料盘又转了)就不下爪；看不到照样下爪
                                        #   (10-24 用户"识别到了之后下达抓取命令能不能快速一点")最近那格：刚看着原料盘停下的那一帧不到 raw_seen_fresh_s 秒，
                                        #   就按那一帧里物料的位置查，不再另拍一帧；手臂已经在抓取的角度上就不再发 AP
    raw_seen_fresh_s=0.5,
    raw_need_clear=True,                # (10-24 用户"上升过程中停顿一下，原料盘一转就把物料甩飞")原料盘这次停下剩的时间，要够下爪、夹紧、
                                        #   再把物料直着抬起 OVCLR 离开原料盘(CLWAIT + 升 OVCLR 的时间)，不够就等下一次停，不在它要转的时候夹
    raw_ct_stop_s=6.0,                  # (10-18 用户量的)原料盘每次大约停多久、转多久(秒)：还没量到之前按这个估计(停的时间按九折算，留余量)；
    raw_ct_move_s=3.0,                  #   亲眼看到两次停下以后，按量到的周期推算回到看的姿态时这一次已经停了多久
                                        #   (10-26)最近那格等着的时候看到它转起来、又停下：量出停多久、转多久，以后按量到的算
    raw_ct_period_tol=0.1,              #   (10-26)只看到过一次停下就回来、它已经停着：按周期推下一次停下，周期按这么多打折(按最早停下算，保守)
    raw_backlash_deg=2.0,               # (10-18 实车 ID1 有齿轮间隙)转到看原料盘的姿态时 ID1 先到小这么多度再转过去：每次都从同一边转到，停的地方一样
    raw_teach_verify=True,              # 示教停靠位对上以后按比赛的走法(从看的姿态一下转过去)再试，差了修角度(一点一点对和一下转过去停的地方不一样)
    raw_teach_verify_n=3,
    raw_a1_margin_deg=5.0,              # (10-18 实车)ID1 离限位这么多度以内算够不着：STM32 给 ID1 往前多给 5° 补偿摩擦，靠近限位时多给的被限位截掉，
                                        #   ID1 停在离限位四五度的地方(示教 3 号位时停在 48.3°，限位 44°)
    raw_ct_accept_mm=5.0,               # 比赛时对准圆心：到时间还没对到 raw_ct_tol_mm，但离示教的位置不到这么多毫米也照样抓；再远就这次不抓(会砸到物料)
    raw_pose_follow=True,               # (10-25 用户"每一个地方都要做完")最近那格：看原料盘的姿态和示教时不一样(arm_calib 改过 A1G/A2E，比如重装齿条后 ID2 读数变了)
                                        #   也照样做：车轮按画面对准(摄像头装在爪子上，伸缩差的长度车轮补)，夹的角度跟着挪同样多。False = 以前的：这次不抓
    raw_pose_a1_tol_deg=3.0,            #   ID1 差超过这么多度(画面跟着转了，示教的位置对不上)才不做
    raw_near_teach_s_mm=100.0,          # (10-18 用户)"最近那格"的做法(rteach RAW near 颜色号)：示教时车轮最多挪这么多毫米(横移、前后)——
    raw_near_teach_f_mm=100.0,          #   arm_calib 里爪子摆不到那一格正上方(齿条不够长)时车轮补；比赛时车轮以示教时挪到的地方为中心挪 raw_center_s_mm/f_mm
    raw_near_iters=3,                   #   示教时最多对几轮(每轮按物料离爪子点还差多少再对一次)
    raw_near_tol_mm=2.0,                #   示教和比赛时原料盘中心对到这么多毫米以内(只在一格夹，比三个停靠位的 raw_ct_tol_mm 严一点：
                                        #   仿真 3mm 时爪子离物料最多 7~9mm，2mm 时 6~7mm；再严对得更久也不更准)
    raw_near_recheck=False,             #   True = 比赛时每夹完一个、回到看的姿态：看一眼原料盘中心，车被带偏了超过 raw_near_redo_mm 就车轮再慢慢对回去。
                                        #   (10-24 用户"物料夹取过程中车不要再进行对准了啊太浪费时间了识别过程中车又不会动")默认关：对准一次以后车一直不动
    raw_near_redo_mm=4.0,
    raw_near_seek=True,                 # (10-20 用户"偏差大一点也要一直识别对准；不能写死先左后右挪着看，要按偏差算")最近那格比赛时找原料盘中心：
                                        #   每帧按车可能停偏的地方猜圆心在画面里哪儿，外圈露出一段就认得出(圆心跑出画面也行)，认出来直接算该挪多少；
                                        #   只看到一个物料就按物料所在的圆推圆心往那边挪；都看不到才在下面的范围里从近到远一处一处看；跟丢了接着找
    raw_near_toward_mm=40.0,            #   能挪的范围(以示教时车停的地方为中心)：往原料盘那边最多 40mm(再近会碰到原料盘)、
    raw_near_away_mm=80.0,              #   往外最多 80mm、
    raw_near_along_mm=120.0,            #   前后最多 ±120mm；按地图查过：会出场地、离黄色区/障碍物太近的那边少挪
    raw_near_wait_s=4.0,                #   到了先原地看几秒(外圈、物料都看不到才开始挪着找；看到一个物料马上往那边挪)
    raw_near_find_s=60.0,               #   找 + 对准一共最多多少秒(跟丢了停下接着找；到时间还没对上才这一批不抓)。(10-23 以前 90 秒；
                                        #   说好的 20 秒在仿真里不够：车停偏 6cm、一开始什么都看不到时要 22~30 秒才找得到，20 秒会三个都不夹)
                                        #   (10-25 用户"把摄像头的识别时间改长，每一个地方都要做完")30 -> 60
    raw_near_lost_s=4.0,                #   对准时连着几秒认不出圆心算跟丢了
    raw_near_step_mm=40.0,              #   都看不到时挪着找：每隔多少毫米看一处
    raw_near_max_mm=150.0,              #   认出圆心以后：对准的地方(就是示教时车停的地方)在上面的范围外也挪过去，但最多离示教时停的地方这么远
    raw_center_mats_every=5,            #   对准时认出外圈以后每几帧才认一次物料(只认外圈快得多)
    raw_rim_tol=0.15,                   # (10-23)比赛时认出的原料盘外圈半径和示教时差多少以内才算认对(比例)：差得多 = 认成了别的圆，这一帧不用
                                        #   (10-23 实车认成了半径 102mm、85mm 的圆，示教时约 135mm，车追着它横挪了几十秒，三个都没夹)
    raw_orbit_tol=0.2,                  # (10-23)只靠物料算圆心时：物料所在圆的半径和示教时差多少以内才算认对(比例)
    raw_meas_every_s=0.5,               # (10-23 用户"识别慢")对准时车在走：每隔多久读一次 STM32 的 V?(一次要 0.2 秒左右)，中间按发的速度推算。
                                        #   以前每帧都读，一秒只认 2 帧；测车轮和画面的对应关系时还是每帧都读(要准)
    raw_vel_back_mm=160.0,              # 原料区做完：离停车点这么多毫米以内只用 V 挪回去(不转车头)，再远才用 S/F
    raw_frame_thread=True,              # (10-18)对准圆心车在走时，认一帧放在后台线程，这边每 raw_vel_keep_s 秒给 STM32 续一次速度
    raw_vel_keep_s=0.15,                #   (STM32 0.4 秒收不到新的 V 就停车，实车上认一帧要零点几秒到一秒多：不续的话车走一下停一下)
    raw_center_probe_speed=10.0,        # 测车轮和画面的对应关系时车轮走多快(毫米/秒)
    raw_any_order=False,                # (10-11 晚 用户：原料区要按顺序夹，不能随机夹)(10-22 起配置里写着 true 也不再用，一律按任务码顺序)。只想这一次不按顺序：mtest RAW 1 force any
                                        #   规则按任务码顺序算"正确抓取"的 2 分，不按顺序可能拿不到这 2 分(放置分不受影响)
    raw_seek=False,                     # (10-11 早 用户：到了原料区停车点车就不动，只动爪子对准夹)True = 原料盘停下时要夹的物料都停在手臂够不着的地方：车挪过去
                                        #   (不横移，不压线)，下一次停下再夹；记住这个位置，同一轮第二批到原料区直接挪到这里
    raw_seek_steps_mm=[40, 20, 10, 5],  # (10-11 用户)原料盘停下时扫不到/够不着要夹的物料：车往原料盘那边(车的右边)挪 4cm，再不行挪 2cm、1cm、0.5cm，一次比一次少
    raw_seek_dir='right',               # 往哪边挪：'right' = 车的右边(原料盘那边，横移)；'left'；'forward' / 'back' = 沿车头前后
    raw_keep_pose=True,                 # (10-11 用户)原料区第一次对准夹到以后，后面的物料都从这个姿态开始等、对准(物料每次停在同一个地方)
    raw_claw_close_s=0.15,              # 发出"合上"到夹爪真的夹住大约要多久(秒)
    lift_init='skip',                   # 升降位置：STM32 开机读编码器自己找准高度并走到 60mm，一般不用管。'home'=让驱动器回零；'zero'/'skip'=不自动记零
    lift_park_mm=60.0,                  # 跑完回到启停区后升降停在这里：下次开机时升降必须在 60±20mm 内，编码器才能认出准确高度；None=不停。
                                        #   60 时用 STM32 的 PARK(收臂 + 停 60)，旧程序不认识就 STOW + LIFT 60
    keepalive_deg=3.0,                  # keepalive()：ID1 摆多少度再摆回来(导航在扫描、规划、定位这些车不动的时候调)
    keepalive_min_s=5.0,                #   两次之间至少隔多久
    camera=dict(device='/dev/video0', width=640, height=480, fps=30, flip=None),
    frames=5,                                                    # 每次测量取几帧(多帧取中值；少一点快一点，噪声会大一点)
    detector='circle',                                           # 物料识别：'circle' = matdet.py 抗遮挡圆拟合；'wuliao' = 原来的 wuliao.py
    matdet=dict(),                                               # matdet 的参数(r_px 半径范围、claw_mask 爪子区域文件…)，一般不用改
    claw_px=dict(RAW=[336.8, 282.9], RING=[336.8, 282.9]),      # 爪子轴线在画面里的位置
    px_per_mm=dict(RAW=1.97, RING=2.96),                         # 每毫米多少像素(只用来换算容差)。RAW：现场画面物料半径 49 像素 = 25mm
    material_diam_mm=50.0,                                       # 物料顶面直径(毫米)：有它就用认到的物料半径现算 RAW 的每毫米像素(观察高度变了也准)；0 = 用 px_per_mm.RAW
    # 对准到多小算好。赛规：1 环外径 = 物料底径 + 3mm，物料偏心不到 1.5mm 才是 1 环(15 分)；2 环 4mm 以内(10 分)；3 环 7.5mm 以内(7 分)
    #   RING 1.2 = 冲 1 环(要先做 vclaw RING 把爪子点量准)；时间不够就改 2.0~3.0(多半是 2 环)。码垛只要不掉下来就得分，放宽
    tol_mm=dict(RAW=2.0, RING=1.2, PICK=2.5, STACK=2.5),
    accept_mm=dict(RAW=4.0, RING=3.5, PICK=5.0, STACK=4.0),      # 修正次数用完后，误差不超过这个也照常夹/放，超过就跳过(RAW：爪子每边只有约 5mm 余量；RING 3.5 = 还在 2 环，放了 10 分，不放 0 分)
    place_confirm=True,                 # 放物料对准到容差以内后再拍一次确认(多花零点几秒，防止一次测量的噪声把偏了的当成对准了)
    align_max_iter=6,                   # 粗加工区/暂存区对准(放、码垛、取回)最多修正几次(车停得很偏、第一次测 J 时要多动几下；平时 1~2 次就到容差)
    zone_frames=None,                      # 粗加工区/暂存区对准时每次测量拍几帧(2 帧快；偏差刚好超出一点点时会自动再测一次取平均)
    zone_gain=0.85,                      # 粗加工区/暂存区对准时手臂每次修正掉偏差的多少(J 是 vcal 测好、每次对准都在修正的，一下修到位)
    zone_filter=False,                   # 粗加工区/暂存区对准时把几次测量合起来用(按动作推算 + 这次测的)：不按一次测量的噪声来回微调
    take_first=False,                    # (10-10 用户：B) 平放时先去转盘取物料，再到圆环上方带着物料对准一次就放下(不再"空爪先对准、
                                        #   去取、再回到原位")。看不到圆环/对不准：物料放回转盘，这个环按以前的做法。码垛照旧
    take_first_iter=4,                  # take_first：带着物料最多修正几次(用存好的 J，不现场探测)；修完按当时的位置放下
    take_first_drop_mm=12.0,
    take_first_retry_max_mm=40.0,       # take_first：第一次修完还差这么多以上(可能看成了旁边的环)就不接着追，放回转盘按空爪做法            # take_first：修正次数用完、偏差不超过这个就照样放下(还在 4~5 环里，有分)；物料绝不放回车上
    place_shift_mm=dict(F=0.0, S=0.0),   # 放下的位置整体挪多少(毫米，F=车头方向，正=往前；S=往左)。10-10 实测物料都偏前约 20mm → F=-20。
                                        #   做了 vclaw RING 把爪子点量准以后改回 0
    take_first_clear_mm=15.0,           # take_first：观察高度 ZOBRNG 时手里物料底面离地至少这么多(ZOBRNG-ZPLC)，不够就按以前的做法
    wheels_first=False,                  # (10-10 用户要的：尽量先慢慢动车)True = 粗加工区/暂存区对准时沿圆环那一排先让车轮前后小步慢慢挪(每次最多 wheels_step_mm)，
                                        #   差不到 wheels_min_mm 再动手臂；离圆环的远近还是手臂伸缩(车轮不横着往圆环那边挪，不压线)。
                                        #   模拟里比默认慢、失败多(车轮只能走整毫米，小步不准)，所以默认关；上车对比：mtest TEMP 1 force wheels
    wheels_step_mm=15.0,                # wheels_first：车轮每次最多挪多少毫米
    wheels_min_mm=4.0,                  # wheels_first：沿那一排的偏差比这个小就交给手臂(1.5 时模拟里失败多，4 好一些)
    wheels_rpm=60,                      # wheels_first：车轮小步挪的速度(转/分，慢一点准一点)
    ring_precorrect=False,               # 同一个工位里对准过一个环以后，去下一个环时手臂伸缩直接伸到同样的远近(不先缩回观察姿态再伸出来，省一次大的修正)
    tilt_precorrect=False,               # 车身和圆环那一排不平行(车停斜了)：看三个环时算出斜了几度，挪到每个环以后离圆环远/近多少，
                                        #   第一次测量之前手臂伸缩(必要时 ID1)就先补上
    tilt_warn_deg=3.0,                  # 斜到这么多度就在终端打 ★ 提醒"把车摆正"
    survey_frames=None,                    # 到工位看三个圆环时拍几帧
    servo=dict(),                                                # 覆盖 visual_servo.DEFAULTS
    servo_cal_file='servo_cal.json',
    vision_cal_file='vision_cal.json',
    zone_reach='arm',                   # (10-14 用户)粗加工区/暂存区：车只在 2 号环前对准一次，1、3 号环按示教(rteach)的手臂角度直接转过去放/取，不再一个一个对准；
                                        #   没示教过(ring_teach.json 里没有这个区的 1、3 号)就照旧一个环一个环对准。'wheels' = 照旧
    ring_teach_file='ring_teach.json',  # 1、3 号环手臂姿态(相对 2 号环对准时的 ID1/ID2 角度差)：(10-20)arm_calib 的 R1、R3 两步写的(src=arm_calib)，
                                        #   以前 rteach 示教的也在这里
    zone_arm_geom=False,                # (10-19 用户：三个环的距离是知道的)没示教 1、3 号环：车对准 2 号环以后，按环距 ring_spacing_mm 和手臂的
                                        #   转轴几何(vcal RING 量的手臂 J、底盘 J)算出转到 1、3 号环的 ID1/ID2，只动手臂、车不动。False = 照旧一个一个对准
                                        #   (10-20 用户：1、3 号环在 arm_calib 里自己调，默认不再按环距算)
    zone_arm_model='polar',             #   'polar' = 手臂绕转轴转(ID1) + 齿条伸缩(ID2)；'linear' = 按手臂 J 直线算(转得少时两种差不多)
    zone_a2_out=1,                      #   ID2 正转是往外伸(1)还是往里缩(-1)
    zone_arm_fine=True,                 # (10-19)1、3 号环：手臂按示教/算的角度转过去以后，空爪只动手臂(车不动)用摄像头对准这个环再去取物料放下
    zone_a1_margin_deg=5.0,             #   转过去的角度离 ID1 限位至少这么多度(补偿摩擦多给的角度被限位截掉)，不然这个环照旧挪车对准
    zone_a2_margin_deg=15.0,            #   离 ID2 限位至少这么多度
    # (10-20 用户)arm_calib 里调过 1、3 号环(ring_teach.json 里 src=arm_calib)的粗加工区/暂存区：车只把 2 号环对准一次，之后车一下都不动；
    #   1、3 号环按调的角度(比 2 号环多转多少)直接转过去放、取，不再挪车。没调过：大声提醒，照旧一个环一个环挪车对准
    zone_cal_fine=True,                 # (10-26 用户"到指定位置之后再通过摄像头微调机械臂的位置……车的位置不动，对准之间必须控制在两秒左右")
                                        #   调过的 1、3 号环：空爪先转到调的角度上方(看圆环的高度)，摄像头只动手臂把这个环的中心对到它自己的十字
                                        #   (claw_px.RING1 / RING3，没调过就是红十字)上，再去车上转盘夹物料、回到对准的角度放。记下来，
                                        #   这个区第二轮(码垛也是)直接用，不再对。False = 以前的直接按调的角度放
    zone_cal_fine_s=2.0,                #   对准最多这么多秒(不算转过去的时间)；到时间没对准：比调的角度更准就按现在的放，不然按调的角度放
    zone_cal_fine_max_mm=30.0,          #   一开始就差这么多以上：多半认错了环(或者车停偏太多)，不信它，按调的角度放
    zone_smooth=True,                   # 对准 2 号环(align_by 是 wheels 时)：像原料区一样车轮连续慢慢走(只发 V 速度，不转车头)，边走边每帧认 2 号环，
                                        #   越近越慢，停着看几帧确认对准；跟丢了停下重新找。要 STM32 认 V、vcal RING 量过车轮和画面的对应关系
                                        #   (横移、前进两列都实测过)；不然照旧一步一步挪(servo.run)
    zone_f_mm=60.0,                     #   沿圆环那一排最多挪多少毫米(以看三个环时算出的 2 号环位置为中心；没看清就以停车点为中心)
    zone_s_max_mm=70.0,                 #   (10-20)横移：挪到"对上 2 号环时车该在的远近"(= arm_calib 调 2 号环时车离圆环那一排的远近)，最多比那里再靠近
                                        #   工位 ring_strafe_max_mm；离停车点最多这么多毫米；按地图查过不压工位。还差的手臂伸缩补
    zone_seek_mm=200.0,                 #   (10-20 用户"找不到环不要后退，往前慢慢去找")停着找不到 2 号环：沿车头往前最多挪这么多毫米找(按地图查过)
    zone_seek_speed=10.0,               #   往前找的速度(毫米/秒)
    zone_vel_speed=15.0,                #   车轮最快多少毫米/秒(偏多少毫米就每秒走多少，越近越慢)
    zone_vel_min_speed=5.0,             #   最慢多少毫米/秒(太慢电机按 0 转/分算会不动)
    zone_vel_tol_mm=None,               #   车轮对到多少毫米以内算对准(停着拍 3 帧取平均)；None = tol_mm.RING
    zone_find_s=60.0,                   #   找 + 对准一共最多多少秒((10-25 用户"识别时间改长，每一个地方都要做完")40 -> 60)
    zone_lost_s=3.0,                    #   对准时连着几秒认不出 2 号环算跟丢了：停下重新找(停着找 2 倍这么久)
    zone_look_s=15.0,                   #   一开始停着找 2 号环最多多少秒(车没动，找不到多半是认不出：暂存区第二批 2 号环被物料盖住时改认物料)。(10-25)10 -> 15
    zone_settle_s=0.15,                 #   停车以后过这么久拍的才算"停着拍的"(刚停下车可能还在晃)
    zone_stall_s=6.0,                   #   车轮挪了这么多秒偏差都没小下去(打滑、没走起来、在容差边上来回)：不再磨，剩下的手臂补
    zone_calc=True,                     # (10-21 用户"不要一开始就往前运动，要根据实际情况来计算位置然后修正")找到 2 号环以后：车停着拍几帧
                                        #   算好要挪多少，S/F 一次挪过去，停稳再拍，按这一下实际走的修正车轮和画面的对应关系，还差再算再挪。
                                        #   False = 以前的边走边对(V 连续)
    zone_calc_moves=4,                  #   最多挪几次(还差的手臂补)
    zone_calc_frames=3,                 #   每次停着拍几帧取中间值
    zone_calc_settle_s=0.3,             #   挪完等这么久再拍(车停稳)
    zone_calc_arm_mm=2.5,               #   只差这么多以内：车轮不再挪(小挪动车轮本身就差 1mm 左右)，剩下的手臂补
    zone_calc_smooth=True,              #   (10-21 用户"改成缓慢移动对准，丝滑一点")算好的那一下用 V 连续慢慢走过去(斜着一起走，不是先横移再前进)；
                                        #   STM32 不认 V 才用 S/F
    zone_calc_speed=30.0,               #   慢慢走过去的最快速度(毫米/秒，快到了自动减速)。(10-21 用户"运动太慢"：20 -> 30)
    # (10-24 用户"故意对偏的那个红色十字去掉，把粗加工区域和暂存区域的十字中心做成可调节的")r2off(ring2_off_mm 故意对偏)去掉了：
    #   2 号环对到圆环这边的爪子点 claw_px.RING(vlive 里的红十字)，在 arm_calib 看圆环的那几步 g 左右 上下 挪它。配置里还写着 ring2_off_mm 也不用
    zone_wheels_only=True,              # (10-21 用户"对准的时候不要用机械臂修正，就用车轮修正，修正距离可以放开")2 号环只用车轮对准：手臂停在
                                        #   观察姿态(A1P/A2P)一下都不动；只差一点、打滑、到能挪的边上、次数用完都不交给手臂补，车轮接着修
                                        #   (最多 zone_wheels_moves 次)，最后还差不到 accept_mm 照样放，不然这个区这次不放。False = 以前(剩下的手臂补)
    zone_wheels_moves=8,                #   只用车轮时最多挪几次
    zone_wheels_s_mm=150.0,             #   只用车轮时横移最多离停车点多少毫米(按地图查过：车身不压工位/黄色区、不出场地、不碰障碍物，碰就少挪)
    zone_wheels_f_mm=150.0,             #   只用车轮时沿圆环那一排最多挪多少毫米(以看三个环时算出的 2 号环位置为中心，同样按地图查)
    zone_map_free_mm=60.0,              #   (10-21 夜实车：2 号环偏 46mm，按地图查把后退限成 30mm，还差 16mm 就不放了)沿圆环那一排离停车点这么近以内不按地图查：
                                        #   地图查是按"车正好停在规划的停车点"算的，车本来就停偏了才要挪；对准的地方就是 arm_calib 调 2 号环时车停的地方
    zone_map_free_s_mm=10.0,            #   横着(往工位那边/往外)只有这么近以内不查：横着挪压进工位、别的颜色区域这一轮就结束了，照样按地图查
    zone_obs_fine=True,                 #   对准前手臂精确回到 A1P/A2P(OBS 大动作只到 1° 以内；arm_calib 调 1、3 号环时手臂就在 A1P/A2P，差 1° 到 1、3 号环会偏两三毫米)
    zone_cal_over_deg=dict(id1=2.0, id2=40.0),   # 1、3 号环按调的角度(加上 2 号环对准时手臂补的)超出限位不到这么多度：按限位放；再多就这个环不放(车不挪)
    zone_cal_min_deg=5.0,               # 1、3 号环存的 ID1 比 2 号环转得不到这么多度：多半是没调就回车了(存成 0)，当没调
    stack_carry=True,                   # (10-23 用户"先放好三环我再去放1环的时候容易把三环放好的东西给撞掉")码垛时夹着物料从车上转盘去圆环，
                                        #   路上要经过已经叠了两层的环才躲；路上没有两层的环照旧走。False = 不躲(按目标环的长度一边转一边扫过去)
    stack_carry_mode='in',              # (10-24 用户"往里面缩去躲避然后到位置了再伸过去不要一会缩一会伸")怎么躲：
                                        #   'in' = 绕过电池盒以后在 stack_carry_a1 停下、往里缩到 stack_carry_a2 -> 缩着转到目标环的角度 -> 只伸出去(只缩一次、伸一次)；
                                        #   'plan' = 10-23 的走法(按三个环的几何，缩着过、太近的从外侧绕，在两个环中间改长度)
    stack_carry_always=True,            #   (10-25 用户"不管是先放哪个环……在经过环3之前就缩回到指定位置然后到指定的环的位置然后再伸长出来")
                                        #   'in'：第二层(码垛)每一个都这样走，路上有没有两层的垛都一样，在路上第一个环(一般是 3 号环)前面缩；
                                        #   False = 以前的：只有路上有两层的垛才躲。放完空爪回车上转盘、第一层、粗加工区都不变
    stack_carry_a1=None,                #   'in'：在哪里往里缩(ID1 度数，arm_calib 的 STKA1 调)；None = 刚绕过电池盒的 R1T1(圆环那组没调 = 在车上转盘上方就缩)
    stack_carry_a2=None,                #   往里缩到多短(ID2 度数，arm_calib 的 STKA2 调；'plan' 也用它)；None = 车上转盘上方的长度 A2R
    stack_carry_clear_mm=50.0,          #   夹着的物料中心离两层垛的中心至少这么远(物料底半径 25 + 垛顶半径 15 + 余量 10)：'in' 只在日志里提醒，'plan' 按它绕
    stack_material_mm=60.0,             #   一个物料多高：日志里按 ZHI - ZSTK - 这个 算夹着的物料底比两层垛顶高多少(ZSTK 时物料底正好在第一层顶上)
    stack_carry_outer=True,             #   'plan'：缩到最短也离得不够远的环(一般是离转轴最近的 2 号环)从外侧绕过；False = 只缩着转
    stack_carry_pre_deg=20.0,           #   'plan'：从车上转盘出来先转到第一个要躲的环前面这么多度(伸缩已经改好)，再转过它
    place_adj_file='place_adj.json',    # pcal 1 / pcal 2 实测的放置补偿(第一层平放、第二层码垛各一组 ID1/ID2 度数)存在这里  # vclaw 实测的爪子像素(claw_px)存在这里，覆盖上面的 claw_px
    place_comp_on=False,                # (10-20 用户"把 28、29 步的补偿去掉")放置补偿(place_adj.json 里的 PL1/PL2、pcal 量的，还有 place_shift_mm)都不用：
                                        #   按摄像头对准的位置(1、3 号环按 arm_calib 调的角度)直接放。True = 照以前加补偿
    chassis_fine_rpm=100,               # 视觉微调时底盘前后挪的速度(转/分)
    # 粗加工区 / 暂存区：
    ring_survey=True,                   # 到工位先用摄像头一次看清三个圆环，算出到 1、2、3 号环底盘各要前后挪多少，直接开过去(不靠估计的 150mm)
    ring_order=dict(ROUGH='lr', TEMP='lr'),   # 摄像头画面里 1、2、3 号环的排列：'lr' = 从左到右是 1 2 3；'rl' = 从左到右是 3 2 1
    ring_spacing_mm=150.0,              # 相邻两个圆环中心的距离(毫米)：用来从画面里算每毫米多少像素
    chassis_strafe=False,               # True = 工位里对准时底盘随便横移(不推荐，会压进工位)。False = 沿圆环那一排前后挪；离圆环远近先靠手臂伸缩补，
                                        #   手臂伸缩到头还够不着，车轮才横着挪(靠近/远离圆环)，只挪够不着的那一段、累计不超过 ring_strafe_max_mm
    ring_strafe_max_mm=20.0,            # 工位里车轮横着挪(靠近/远离圆环)累计最多多少毫米(相对停车点)。0 = 绝不横移，全靠手臂伸缩。
                                        #   10-10 从 40 改成 20：停车点离工位白区只有 60mm，挪满 40 再加上定位误差就只剩几毫米
    ring_fix_max_mm=60.0,               # 对准一个圆环时，底盘最多再为对准前后挪这么多毫米(超过就停下：多半认错了环)
    ring_move_min_mm=15.0,              # 到下一个圆环要挪的距离小于这个就不动底盘(手臂够得着)
    drop_retract=False,                 # 放下物料后要不要缩回伸缩舵机。False = 只抬起来，接着去下一个环，这个工位做完再收臂(省时间)
    mtest_hold_heading=True,            # mtest 开始时把"现在的车头方向"记为要保持的方向(车是手搬过来的，不然底盘一动就往开机时的方向转)
    # 圆环相对停车点、沿车头方向的位置(毫米，正=在车头前方)。车头朝向见 map_config 的 stops：
    #   ROUGH 车头朝东，圆环板 3-2-1 从西到东 -> 1 号在前方 +150
    #   TEMP  车头朝南                       -> (10-20 用户的照片)也是 1 号在前方 +150(和粗加工区一样：arm_calib 的 1、3 号环两个区共用)
    #   (有 vcal RING 量过的车轮和画面的对应关系时不用这个，按 ring_order 推)
    ring_offset_mm=dict(ROUGH={'1': 150.0, '2': 0.0, '3': -150.0}, TEMP={'1': 150.0, '2': 0.0, '3': -150.0}),
    pickback_fast=True,                 # 粗加工区取回时直接回到放下时记下的底盘位置和手臂角度，只测一次确认，容差内就不重新对准(省 4~5 秒/个)
    # (10-22 用户"这个也要按顺序来")粗加工区/暂存区(含码垛)一律按任务码的顺序放(规则：必须按照任务码的顺序和位置放置)。
    #   以前的 place_order='sweep'(顺路 3→2→1)去掉了：车上配置里还写着也不再用
    zone_timeout_s=None,                # 工位里一次对准最多用几秒(到了还差一点点也按 accept_mm 放，不一直磨)
    pickback_order='code',              # 粗加工区取回的顺序：'code'=按任务码顺序(最符合规则)；'reverse'=倒序；'near'=就近(底盘走得最少，省时间)
    learn_pick=False,                   # True = 还没量过 claw_px.PICK 时，放下第一个物料后回去再拍一张量一次(要多花几秒)；默认不量，取回按圆环对准
    stow_at_start=True,                 # go 开始时先把手臂收到待机姿态(STOW)
    # (10-23 用户"到原料区、粗加工区、暂存区之前机械臂提前转到需要的姿态，节省时间")车开往这三个区的最后一段直线(后面不再转弯)
    #   之前发 PRE RAW / PRE RING：手臂在最高处(ZHI)边开车边转向观察姿态，到了 OBS 只补最后一点。要新固件(旧的不认识就自动不用)
    pre_arm=True,
    return_tol_deg=0.4,                 # 回到记下的姿态后回读，差得比这个多就再转一次
    return_bias=False,                   # 回到记下的姿态时，按前几次"转回来总是多转/少转几度"提前补上(省掉"再转一次")
    return_preload_deg=[0.0, 0.0],      # 回到记下的姿态前，先从反方向多转这些度(ID1, ID2)再回来，消除齿轮间隙；0=不用
    stop_aliases=None,                  # 停车点名字不叫 QR/RAW/ROUGH/TEMP/START 时，例如 {'ROUGH': ['PROC']}
    screen=dict(code='t0', code2='t7', stage='t1', grab='t2', place='t3', msg='t4', b1='t5', b2='t6'),
    screen_show=('code', 'code2', 'grab', 'place'),   # (10-23 用户"按要求显示必要的字")屏上只写这几样：任务码两行、抓取数、放置数
    screen_stats_live=False,            # (10-23 晚)抓取数、放置数只在回到启停区以后写(新固件画成中文"抓取3  放置5")；true = 跑的时候也写
                                        #   (赛规：一直显示任务码，回家后显示正确抓取数、正确放置数)；阶段、提示、B1/B2 只在终端日志里   # 任务码分两行(字高 ≥12mm 一行放不下)：t0=前两组，t7=后两组
)


A2_MIN, A2_MAX = -1210.0, -300.0       # 旧程序(没有 S2MIN/S2MAX 参数)ID2 的限位(10-16 用户：-1210 ~ -300)
A1_MIN, A1_MAX = 44.0, 244.0           # STM32 里 ID1 的限位(main.c SERVO1_MIN_DEG / SERVO1_MAX_DEG)；配置 a1_range 可以改
RAW_COLORS = (1, 2, 3, 4)              # (10-19 用户：省赛没有黑、浅蓝)没有任务码时原料区认这几种颜色：红、黄、蓝、绿

BOOT_TEXT = {'ENC': '按编码器认的高度', 'NOCAL': '没标定，当成 60mm', 'NOENC': '读不到编码器，当成上次停下的高度(旧程序当成 60mm)',
             'POS': '按上次停的位置认的高度', 'SYNC': '开机没读到编码器，后来按编码器认准了'}

PICK_COLORS = (1, 2, 3, 4, 6)          # 能按物料顶面对准的颜色。黑色(5)物料放在黑环上和黑环连成一片，分不出来：认圆环


def deep_merge(base, extra):
    out = {}
    for k, v in base.items():
        out[k] = deep_merge(v, None) if isinstance(v, dict) else (list(v) if isinstance(v, list) else v)
    for k, v in (extra or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


def vision_cfg(cfg):
    """mission_cfg -> 给 vision.Vision 的配置：Vision.DEFAULT 里有的键都带过去(圆环识别的设置也在里面)。"""
    vc = {k: cfg[k] for k in Vision.DEFAULT if cfg.get(k) is not None}
    vc.setdefault('detector', 'circle')
    vc['matdet'] = cfg.get('matdet') or {}
    return vc


def load_place_adj(path):
    """pcal 存的放置补偿 {'RING': [d1, d2], 'STACK': [d1, d2]}；没有文件返回 {}。"""
    if not path or not os.path.exists(path):
        return {}
    try:
        with open(path, 'r', encoding='utf-8') as f:
            d = json.load(f)
        return d if isinstance(d, dict) else {}
    except Exception:
        return {}


def save_json(path, d, indent=1):
    """(10-23)一键启动就是整车电闸，示教完可能马上就关电：先写临时文件、fsync，再改名替换。
    断电时文件要么是旧的、要么是新的，不会只写了一半。"""
    tmp = str(path) + '.tmp'
    with open(tmp, 'w', encoding='utf-8') as f:
        json.dump(d, f, ensure_ascii=False, indent=indent)
        f.flush()
        os.fsync(f.fileno())
    os.replace(tmp, path)
    try:
        fd = os.open(os.path.dirname(os.path.abspath(str(path))), os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError:
        pass


def save_place_adj(path, key, d1, d2):
    d = load_place_adj(path)
    d[key] = [round(float(d1), 2), round(float(d2), 2)]
    save_json(path, d)
    return d


def same_item(a, b):
    """两个 Item 是不是同一个物料(重新 mcode / 扫码以后 Item 对象换了新的，按批次、序号、颜色、圆环比)。"""
    if a is None or b is None:
        return False
    return a is b or (a.batch, a.index, a.color, a.ring) == (b.batch, b.index, b.color, b.ring)


def fit_arm_polar(poses, spacing):
    """(10-23)按爪子在三个圆环正上方时的手臂角度 poses = {环号: (ID1, ID2)} 算手臂的几何(三个环在一条直线上、相邻隔 spacing 毫米)：
    爪子离转轴 r = a + k * ID2 毫米，ID1 的度数就是手臂转过的角度。返回 (a, k, 残差 mm)；角度不全、算不出返回 None。"""
    try:
        th = {i: math.radians(float(poses[i][0])) for i in (1, 2, 3)}
        x2 = {i: float(poses[i][1]) for i in (1, 2, 3)}
        s = float(spacing)
    except (KeyError, TypeError, ValueError, IndexError):
        return None
    want = ((1, 2, s), (2, 3, s), (1, 3, 2.0 * s))

    def cost(a, k):
        tot = 0.0
        for i, j, d in want:
            ri, rj = a + k * x2[i], a + k * x2[j]
            dd = np.sqrt(np.maximum(ri * ri + rj * rj - 2.0 * ri * rj * math.cos(th[i] - th[j]), 0.0))
            tot = tot + (dd - d) ** 2
        ok = np.ones_like(tot, dtype=bool)
        for i in (1, 2, 3):
            ok &= (a + k * x2[i]) > 20.0                        # 爪子总在转轴外面
        return np.where(ok & (np.abs(k) > 0.005), tot, np.inf)

    a, k = np.meshgrid(np.linspace(-400.0, 1200.0, 801), np.linspace(-0.5, 0.5, 501))
    c = cost(a, k)
    i = np.unravel_index(np.argmin(c), c.shape)
    a0, k0, da, dk = float(a[i]), float(k[i]), 2.0, 0.002
    for _ in range(4):                                      # 在找到的地方附近再细找
        a, k = np.meshgrid(np.linspace(a0 - 2 * da, a0 + 2 * da, 41), np.linspace(k0 - 2 * dk, k0 + 2 * dk, 41))
        c = cost(a, k)
        i = np.unravel_index(np.argmin(c), c.shape)
        a0, k0, da, dk = float(a[i]), float(k[i]), da / 10.0, dk / 10.0
    best = float(c[i])
    if not math.isfinite(best):
        return None
    return a0, k0, math.sqrt(best / 3.0)


def stack_carry_plan(start_a1, target, poses, tall, spacing, clear_mm, a2_in, a2_lim=(None, None), pre_deg=20.0,
                     outer=True, fit_max_mm=6.0, avoid_a1=None):
    """(10-23)码垛：夹着物料从车上转盘(手臂在 start_a1)转去 target = (ID1, ID2)，路上要经过 tall 里这几个已经叠了两层的环。
    poses = {环号: (ID1, ID2)}：爪子在 1、2、3 号环正上方时的角度(算手臂几何、看路上经过哪几个环)。
    返回 dict(wps=[(ID1, ID2)...] 路点, dodge=[(环号, '缩着'|'外侧', 离它中心多少 mm 或 None)], fit=(a, k, 残差) 或 None)；
    路上一个两层的环都不经过返回 None(照旧走)。
    走法：先转到第一个要躲的环前面 pre_deg 度(伸缩已经改成躲它的长度)，再转过它；在两个环中间(角度的一半)改伸缩长度躲下一个；
    过了最后一个要躲的环，在它和目标中间伸到目标的长度，最后(调用的人)转到目标。每一段只动一个舵机：转的时候伸缩不变，伸缩的时候不转。
    avoid_a1：绕电池盒开始缩回的角度(R1T1)：第一个路点不早于它(过了电池盒才改伸缩长度)。"""
    a1t, a2t = float(target[0]), float(target[1])
    start_a1 = float(start_a1)
    lo, hi = sorted((start_a1, a1t))
    cross = [r for r in tall if r in poses and lo + 1.0 < float(poses[r][0]) < hi - 1.0]
    if not cross:
        return None
    cross.sort(key=lambda r: abs(float(poses[r][0]) - start_a1))
    fit = fit_arm_polar(poses, spacing)
    if fit is not None and fit[2] > fit_max_mm:
        fit = None                                          # 三个环的角度对不上一条直线：几何不可信，只缩着转
    a2_min, a2_max = a2_lim

    def clamp(v):
        if a2_min is not None:
            v = max(v, float(a2_min))
        if a2_max is not None:
            v = min(v, float(a2_max))
        return v
    a2_in = clamp(float(a2_in))
    lens, dodge = [], []
    for r in cross:
        L, side, gap = a2_in, '缩着', None
        if fit is not None:
            a, k, _ = fit
            R = a + k * float(poses[r][1])                     # 这个环离转轴多远
            g_in = abs(R - (a + k * a2_in))
            gap = g_in
            if g_in < clear_mm and outer and (a + k * a2_in) < R:
                L_out = clamp((R + clear_mm - a) / k)
                g_out = (a + k * L_out) - R
                if g_out > g_in:
                    L, side, gap = L_out, '外侧', g_out
        lens.append(L)
        dodge.append((r, side, gap))
    sgn = 1.0 if a1t > start_a1 else -1.0
    first = float(poses[cross[0]][0])
    pre = first - sgn * float(pre_deg)
    if avoid_a1 is not None and (float(avoid_a1) - start_a1) * sgn > 0 and (first - float(avoid_a1)) * sgn > 0:
        if (pre - float(avoid_a1)) * sgn < 0:
            pre = float(avoid_a1) + sgn * 1.0                  # 还没过电池盒：转过 R1T1 再改伸缩长度
    if (pre - start_a1) * sgn <= 0 or (first - pre) * sgn <= 0:
        pre = (start_a1 + first) / 2.0
    wps = [(pre, lens[0])]
    marks = [float(p[0]) for p in poses.values()] + [a1t]
    for n, r in enumerate(cross):
        here = float(poses[r][0])
        nxt = min((m for m in marks if (m - here) * sgn > 1.0), key=lambda m: abs(m - here), default=a1t)
        mid = (here + nxt) / 2.0                            # 和下一个环(不管叠没叠)的中间改伸缩长度：改的时候爪子不在任何一个环上方
        wps.append((mid, lens[n]))
        nxt_len = lens[n + 1] if n + 1 < len(cross) else a2t
        if abs(nxt_len - lens[n]) > 0.5:
            wps.append((mid, nxt_len))
    return dict(wps=wps, dodge=dodge, fit=fit)


def stack_in_plan(start_a1, target, poses, tall, a2_in, a1_in=None, after=None, hold=None, fit=None, a2_lim=(None, None),
                  margin_deg=2.0, a2_start=None, always=False):
    """(10-24 用户"往里面缩去躲避然后到位置了再伸过去不要一会缩一会伸")码垛：夹着物料从车上转盘(手臂在 start_a1)去 target = (ID1, ID2)，
    路上要经过 tall 里这几个已经叠了两层的环。只往里缩一次、往外伸一次：
      ① 从车上转盘出来(STM32 照旧绕电池盒)只转到 a1_in，伸缩停在 hold(绕电池盒伸的长度 R2T；不绕就是车上转盘上方的 A2R)；
      ② 在 a1_in 只往里缩到 a2_in；③ 缩着只转到目标的 ID1；④ (调用的人)只伸到目标的 ID2。
    poses = {环号: (ID1, ID2)}：爪子在 1、2、3 号环正上方时的角度(看路上经过哪几个环)。after = 绕电池盒转过 R1T1 的角度(不知道 = None)。
    a1_in 要在绕过电池盒(after，不知道就是车上转盘)和第一个要经过的两层垛中间；没给、调错了：after 在中间就用 after，
    不然用车上转盘和那个垛的正中间。a1_in 就在车上转盘上方(没出车上转盘)：① 不动，伸缩还是 a2_start(A2R)。
    (10-25 用户"不管是先放哪个环……在经过环3之前就缩回到指定位置然后到指定的环的位置然后再伸长出来")always=True：路上有没有两层的垛
    都这样走，往里缩的地方在路上第一个环(从车上转盘出来先经过的，一般是 3 号环；去的就是它就是目标环)前面。
    返回 dict(wps=[①②③ 的 (ID1, ID2)], cross=[经过的两层垛(先经过的在前)], a1_in=, a2_in=, moved=a1_in 换过没有,
    first_ring=往里缩要赶在它前面的那个环(None = 目标环本身), gaps={环号: 缩着转过去离它中心多少 mm}(给了手臂几何 fit = (a, k, 残差) 才有))；
    always=False 时路上一个两层的环都不经过返回 None(照旧走)。"""
    a1t, a2t = float(target[0]), float(target[1])
    start_a1 = float(start_a1)
    lo, hi = sorted((start_a1, a1t))
    cross = [r for r in tall if r in poses and lo + 1.0 < float(poses[r][0]) < hi - 1.0]
    if not cross and not always:
        return None
    cross.sort(key=lambda r: abs(float(poses[r][0]) - start_a1))
    sgn = 1.0 if a1t > start_a1 else -1.0
    on_way = sorted((r for r in poses if lo + 1.0 < float(poses[r][0]) < hi - 1.0), key=lambda r: abs(float(poses[r][0]) - start_a1))
    first_ring = (cross or on_way or [None])[0] if not always else (on_way or [None])[0]
    first = float(poses[first_ring][0]) if first_ring is not None else a1t

    def between(v, base):                               # 在 base(含)和第一个两层垛(离它 margin_deg 以上)中间
        return v is not None and (float(v) - base) * sgn >= 0.0 and (first - float(v)) * sgn > margin_deg
    base = float(after) if between(after, start_a1) else start_a1      # 绕电池盒的那一段里不能缩(会碰电池盒)
    want = None if a1_in is None else float(a1_in)
    if between(want, base):
        a1 = want
    elif base != start_a1:
        a1 = base
    else:
        a1 = (start_a1 + first) / 2.0
    moved = want is not None and abs(a1 - want) > 1e-6
    lo2, hi2 = a2_lim
    a2 = float(a2_in)
    if lo2 is not None:
        a2 = max(a2, float(lo2))
    if hi2 is not None:
        a2 = min(a2, float(hi2))
    h = a2 if hold is None else float(hold)
    if abs(a1 - start_a1) <= 1.0 and a2_start is not None:
        h = float(a2_start)                             # 就在车上转盘上方缩：不用先伸出去绕电池盒
    wps = [(a1, h)]
    if abs(h - a2) > 0.5:
        wps.append((a1, a2))
    wps.append((a1t, a2))
    gaps = {}
    if fit is not None:
        a, k, _res = fit
        r_in = a + k * a2
        gaps = {r: abs(a + k * float(poses[r][1]) - r_in) for r in cross}
    return dict(wps=wps, cross=cross, a1_in=a1, a2_in=a2, moved=moved, gaps=gaps, first_ring=first_ring)


def _first(p, measure):
    """第一次返回已经测好的 p(刚测过、还没动，不用再拍一遍)，以后每次调 measure()。"""
    box = [p]

    def m():
        if box[0] is not None:
            q, box[0] = box[0], None
            return q
        return measure()
    return m


class _VelDrive:
    """(10-20)车轮按 V 速度连续走(工位对准 2 号环用；原料区那套写在 _raw_center_align_smooth 里)。
    p = 车走了多少(相对开始时，[横移, 前进] 毫米)，按 STM32 自己算的 V 走过的距离(V?)，读不到才按发的速度推；
    lo / hi = 底盘位移(act.disp，相对停车点)能到的范围：到了边上，往外的那个方向不再走(clamped 记下)。
    认一帧要零点几秒到一秒多，STM32 0.4 秒收不到新的 V 就停车：frame(fn) 把识别放后台线程，这边每 keep_s 秒续一次速度。"""

    def __init__(self, hooks, lo, hi, keep_s=0.15, threaded=True):
        self.h, self.act = hooks, hooks.act
        self.base = np.array([float(self.act.disp['S']), float(self.act.disp['F'])])
        self.lo, self.hi = np.asarray(lo, float), np.asarray(hi, float)
        self.keep_s, self.threaded = float(keep_s), bool(threaded)
        self.p, self.tp = np.zeros(2), hooks.now()
        self.cmd, self.stop_at, self.clamped = np.zeros(2), None, False
        self.t_stop = hooks.now()                              # 最近一次停下的时刻(停稳以后拍的才算停着拍的)
        self.v0 = None
        self.t_ser, self.n_ser = 0.0, 0

    def start(self):
        """先停车、记下 V? 的起点。STM32 不认 V 返回 False。"""
        try:
            v0 = self.act.vel_read()
        except ArmAbort:
            raise
        except ArmError:
            v0 = None
        if not self.act.vel(0.0, 0.0):
            return False
        self.v0 = None if v0 is None else np.array(v0, float)
        return True

    def est(self):
        """车现在大概走了多少(上次量的 + 按发的速度推)。"""
        return self.p + self.cmd * (self.h.now() - self.tp)

    def pos(self):
        """底盘现在的位移(相对停车点)。"""
        return self.base + self.est()

    def meas(self):
        p = None
        t0 = time.perf_counter()
        if self.v0 is not None:
            try:
                v1 = self.act.vel_read()
            except ArmAbort:
                raise
            except ArmError:
                v1 = None
            if v1 is not None:
                p = np.array(v1, float) - self.v0
        self.t_ser += time.perf_counter() - t0
        self.n_ser += 1
        if p is None:
            p = self.est()
        self.p, self.tp = p, self.h.now()
        return p

    def send(self, vs, vf, run_s=None):
        """发车轮速度(到范围边上往外的方向不走)。run_s：最多走这么久就停(下一帧认得慢也不会走过头)。"""
        p = self.est()
        self.p, self.tp = p, self.h.now()
        a = self.base + p
        clamped = False
        if (a[0] >= self.hi[0] and vs > 0) or (a[0] <= self.lo[0] and vs < 0):
            vs, clamped = 0.0, True
        if (a[1] >= self.hi[1] and vf > 0) or (a[1] <= self.lo[1] and vf < 0):
            vf, clamped = 0.0, True
        self.act.vel(vs, vf)
        was = float(np.linalg.norm(self.cmd)) > 1e-9
        self.cmd, self.clamped = np.array([vs, vf], float), clamped
        moving = abs(vs) > 1e-9 or abs(vf) > 1e-9
        if was and not moving:
            self.t_stop = self.h.now()
        self.stop_at = (self.h.now() + float(run_s)) if (moving and run_s is not None) else None

    def keep(self):
        """识别一帧的时候：到点了就停，不然续一次速度。"""
        if float(np.linalg.norm(self.cmd)) < 1e-9:
            return
        if self.stop_at is not None and self.h.now() >= self.stop_at:
            self.send(0.0, 0.0)
        else:
            stop_at = self.stop_at
            self.send(float(self.cmd[0]), float(self.cmd[1]))
            self.stop_at = stop_at

    def finish(self):
        """停车，V 走过的距离记进 act.disp(挪回停车点要用)。"""
        self.act.vel(0.0, 0.0)
        self.cmd, self.stop_at = np.zeros(2), None
        p = self.meas()
        self.act.disp['S'] = float(self.base[0] + p[0])
        self.act.disp['F'] = float(self.base[1] + p[1])

    def frame(self, fn):
        """认一帧(fn())：返回 (fn 的结果, 拍这一帧时车走了多少)。车在走时 fn 放后台线程，这边每 keep_s 秒续一次速度。"""
        p_cap = self.meas()
        if not self.threaded or float(np.linalg.norm(self.cmd)) < 1e-9:
            return fn(), p_cap
        import threading
        box = {}

        def work():
            try:
                box['r'] = fn()
            except BaseException as ex:                        # 识别出错：交给这边抛出
                box['e'] = ex
        th = threading.Thread(target=work, daemon=True)
        th.start()
        try:
            while th.is_alive():
                th.join(self.keep_s)
                if th.is_alive():
                    self.h._check_abort()
                    self.keep()
        except BaseException:
            th.join(3.0)                                       # 等识别线程停下再往外抛(别和下一次识别一起用摄像头)
            raise
        if 'e' in box:
            raise box['e']
        return box['r'], p_cap

    def timing(self):
        n = max(1, self.n_ser)
        out = f'(串口读一次走了多少 {1000.0 * self.t_ser / n:.0f}ms)'
        self.t_ser, self.n_ser = 0.0, 0
        return out


class MissionHooks:
    def __init__(self, raw_cfg=None, log=print, vision=None, store=None, now=time.monotonic, sleep=time.sleep, ctx=None):
        self.raw_cfg = raw_cfg or {}
        self.cfg = deep_merge(DEFAULTS, self.raw_cfg.get('mission_cfg'))
        self.log = log
        self.now = now
        self.sleep = sleep
        self.ctx = ctx                      # auto_run.drive 会把 ctx 填进来，用来检查 abort
        self.vision = vision
        self.store = store
        self.t0 = now()                     # 比赛计时的起点：start_clock() 时改成那一刻(按下 START)；导航不调 start_clock 时就从创建算(偏保守)
        self.clock_started = False          # start_clock() 调过了
        self.out_of_time = False            # 时间不够了：不再开始新的物料，go_home_now 一律回 True
        self.home_eta_s = None              # 导航说的"从现在这个停车点回家要多久"(秒)；None = 不知道(只按 time_limit_s)
        self.work_t = {}                    # 实测的每个物料用时(秒)：'grab' / 'place' / 'pick' / 'stack'
        self.base_t = {}                    # 实测的每个停车点物料以外的用时(秒)：角色 -> 秒
        self.stop_times = []                # 每个停车点用了多久：(停车点, 角色, 第几次, 秒)
        self._items_t = 0.0                 # 这个停车点里做物料一共用了多久
        self._stop_name = None              # 现在在哪个停车点(配置 stops 里的名字)
        self._stop_shift = {}               # (10-20)导航说的：下次到这个停车点时停的位置比规定的点挪了 (往前mm, 往左mm)
        self._cur_shift = (0.0, 0.0)        # 这一次停的位置比规定的点挪了多少
        self._prep = {}                     # prepare() 做成了哪几步：'arm' / 'qr' / 'cam'
        self._t_alive = -1e9                # 上一次 keepalive 摆手臂的时刻
        self.holding = None                 # 爪子里现在夹着的物料(从转盘取出来、还没放下)
        self.maybe_holding = None           # 不为 None = 爪子里可能还夹着东西(原因)：不降升降、不张爪子
        self._pre_pending = None            # (10-23)提前摆了臂(PRE)、还没收臂的停车点：到了没做任务就收臂
        self._pre_off = False               # (10-23)STM32 不认识 PRE(旧程序)：这次不再提前摆臂
        self._dodge = None                  # (10-24)码垛正在躲两层垛：(往里缩的 ID1, 缩到的 ID2)，路上出错放回转盘时先缩着转回来
        self._aborted = False               # 急停过：park() 不自动做(人手可能在附近)
        self._boot_asked = False            # 为了看 STM32 是不是新程序(屏上任务码带不带 +)问过 LIFT? 了
        self._plus_warned = False
        self._tilt = None                   # 看三个圆环时量的车身斜度和远近(提前补手臂伸缩用)，见 _survey
        self._a2_hint = None                # 刚 OBS RING 过：ID2 在 A2P、ID1 在 A1P(OBS 不回报角度)
        self._a1_hint = None
        self.arm = None
        self.act = None
        self.servo = None
        self.plan = None
        self.stats = Stats()
        self.visits = {}
        self.in_tray = {}                   # 槽位 -> Item(现在转盘里有的物料)
        self.on_ring = {}                   # (区, 圆环号) -> [Item, ...](从下到上)
        self._teach_fix = {}                # (10-14)区 -> {环号: (dID1, dID2)} 拍照补的
        self._p2 = {}                       # (10-14)区 -> 2 号环对准时的 ID1/ID2 角度(zone_reach='arm')
        self._zone_learn = {}               # (10-19)区 -> {环号: (dID1, dID2)}：只动手臂对准 1、3 号环时量到的(相对 2 号环)，第二批直接用
        self._zone_geom_why = {}            # (10-19)区 -> 这一趟为什么不能按环距算(None = 能)：到工位时定下，中途量到对应关系也不换做法
        self._p2_at = {}                    # (10-19)区 -> 对准 2 号环时车在哪(底盘位移 S、F)：车挪过就要重新对准
        self._anchor0 = {}                  # (10-26)区 -> 第一次来对准(2 号环/原料盘中心)时车在哪(相对规定的停车点：S、F)
        self._anchor_now = None             #   这一趟对准时的 (区, (S, F))；task() 开头清掉
        self._p2_failed = set()             # (10-20)这一趟 2 号环没对准的区：后面的物料不再试(省时间)，直接照旧一个环一个环对准
        self._zone_v_only = False           # (10-20)这一趟工位里车只用 V 动过(没发 S/F)：做完挪回停车点也只用 V(不转车头)
        self.ring_uv = {}                   # (10-20)(区, 圆环号) -> (画面里 u, v, 那时底盘 S, F)：看三个环时记下的，对准 2 号环时按它找
        self.pose_at = {}                   # (区, 转盘槽位) -> 放下那一刻的 底盘位移 S/F 和手臂角度 a1/a2，取回时直接回到这里
        self.learn = {'S': 0.0, 'F': 0.0}   # 这个工位里前一个圆环对准时发现的停车误差(底盘位移里扣掉圆环间距后剩下的)，带到下一个圆环
        self.disabled = None                # 不为 None = 整轮不做夹放，原因在里面
        self._inited = False
        self.placements = []                # 记录每次放置：(区, 环, 是否码垛, 对准误差 mm)
        self._warned_vmask = False          # 蓝色物料没做 vmask 的提醒只说一次
        self.nogo = False                   # mtest ... nogo：只认圆环、对准，不取物料、不放、不夹回
        self.ring_rev = False               # mtest ... rev：这一次圆环的前后方向反过来(车头朝向和配置的相反)
        self._pick_tries = 0                # 量 claw_px.PICK 试了几次(认不到就下一个物料再试，最多 3 次)
        self.raw_stops = []                 # 看到的原料盘每次停了多久(秒)，用来估计这一次停下还剩多少时间
        self.raw_moves = []                 # 每次转了多久(秒)
        self.raw_cycle = {}                 # 最近一次看到原料盘停下的时刻 't_stop'：按"停多久+转多久"的周期推算以后每次停下的时刻
        self.ring_f = {}                    # (区, 圆环号) -> 这个环正对爪子时底盘的前后位移(毫米)，到工位时用摄像头看出来的(_survey)
        self.ring_px = {}                   # (区, 圆环号) -> (开到这个环以后垂直于圆环那一排还差多少像素(带正负), 那个方向(画面里), 看圆环时底盘 S)：
                                            #   只动车轮时算到每个环顺便横移多少(看到两个以上圆环才有)
        self._zone_s_ref = None             # 只动车轮：这个工位里对准好的一个环 (区, 环号, 底盘 S)，去别的环按它横移(只差车身斜的那一点)
        self.ring_gate_px = None            # 对准时只认离爪子点这么多像素以内的圆环(不去追旁边那个)
        self._ring_ready = False            # 手臂现在就在这个工位的圆环上方、爪子张开(刚放完/刚看完圆环)：去下一个环不用整套 OBS
        self._at_obs = False                # 手臂就在观察姿态(刚 OBS 过、还没动过)：去第一个环不用再 OBS 一遍
        self._zone_d2 = []                  # 这个工位里每次对准好时 (底盘前后位移, ID2 比 A2P 多转的度数)：车离圆环那一排的远近
        self._ret_bias = [0.0, 0.0]
        self._raw_seek_F = 0.0              # 原料区：车离停车点挪了多少(前后 F、横移 S)时够得着物料(挪着找到的，第二批直接用)
        self._raw_seek_S = 0.0
        self._raw_pose = None               # 原料区第一次夹到时对准好的 (ID1, ID2)：后面的物料直接从这里开始
        self._seek = None         # 从转盘回到记下的姿态时 ID1、ID2 总是多转了多少度(学出来的，下次提前补上)

    # ------------------------------------------------------------------ 给 auto_run 的接口
    def adjust(self, stop, link, log):
        """到停车点后调姿态：目前什么也不做(车位已经由雷达定位好；精确对准在 task 里用摄像头做)。"""
        return None

    def stop_offset(self, stop, fwd, left):
        """(10-12/10-20)导航说：这次停的位置比规定的停车点往前 fwd、往左 left 毫米(规定的点被障碍物挡住、开不过去、压到工位，规划时挪的)。
        到了做任务时：地图检查按真正停的位置算；工位对 2 号环横移、往前找，原料区找圆心能挪的范围都放宽这么多。"""
        self._stop_shift[str(stop)] = (float(fwd), float(left))

    def task(self, stop, link, log):
        self.log = log or self.log
        role = role_of(stop, self.cfg.get('stop_aliases'))
        if role is None:
            self.log(f'    (停车点 {stop} 没有对应任务，跳过)')
            return
        self._stop_name = stop
        self._cur_shift = self._stop_shift.pop(str(stop), (0.0, 0.0))
        if math.hypot(*self._cur_shift) >= 1.0:
            self.log(f'    (这次停的位置比规定的停车点往前 {self._cur_shift[0]:+.0f}mm、往左 {self._cur_shift[1]:+.0f}mm：'
                     '对准时车能挪的范围放宽这么多，地图检查按真正停的位置算)')
        self._ensure(link)
        self.act.reset_disp()                       # 路线把车开到了新的停车点，位移从 0 重新算
        self.learn = {'S': 0.0, 'F': 0.0}
        self._anchor_now = None
        self.visits[role] = self.visits.get(role, 0) + 1
        t_start = self.now()
        self._items_t = 0.0
        try:
            if role in ('RAW', 'ROUGH', 'TEMP') and self.plan is None and str(self.cfg.get('qr_fixed_code') or '').strip():
                self.log(f'    用配置里手动给的任务码 {self.cfg.get("qr_fixed_code")}(路线里没有 QR 点也行)')
                self._use_code(str(self.cfg.get('qr_fixed_code')).strip())
            if (role in ('RAW', 'ROUGH', 'TEMP') and self.plan is None and not self.disabled
                    and self.cfg.get('qr_retry_at_stops', True)):
                self._qr_again()                    # QR 点没读到码：STM32 可能在路上读到了，再问一次
            self.run_role(role, self.visits[role])
            if self._pre_pending:
                self.pre_cancel(stop)               # (10-23)提前摆了臂，这一站却没用手臂(没码、时间到…)：收臂再开走
        except Abort:
            self._aborted = True
            raise
        except ArmAbort as ex:
            self._aborted = True
            raise Abort(str(ex))
        except Exception as ex:                     # 程序里意外的错：不结束这一轮，这个停车点不做了，收臂、挪回停车点
            self.log(f'  ★ {stop} 出了意外的错({ex!r})：这个停车点不再做，收臂、挪回停车点，路线照走')
            self._bail_out(role)
        dt = self.now() - t_start
        self.stop_times.append((stop, role, self.visits[role], dt))
        if role in ('RAW', 'ROUGH', 'TEMP') and self._items_t > 0.5:
            self._learn_t(self.base_t, role, max(0.0, dt - self._items_t))
        if role != 'START':
            self.log(f'    ({stop} 用了 {dt:.0f} 秒；从出发算已用 {self.elapsed():.0f} 秒)')

    # ------------------------------------------------------------------ (10-23)到站前提前摆臂
    def pre_arrive(self, stop, link=None, log=None):
        """(10-23 用户"到原料区、粗加工区、暂存区之前让机械臂提前转到需要的姿态，节省时间")导航在车开往这个停车点的最后一段直线
        (到停车点之前不再转弯)之前调用：STM32 的 PRE RAW / PRE RING 让手臂在最高处(ZHI)边开车边转向观察姿态，不降到观察高度；
        到了照常 OBS，已经在姿态上就只补最后一点、降到观察高度。这一站用不上手臂(没任务码、转盘里没有要放的…)就不提前转。
        到了却没做任务(分段调试跳过)：导航调 pre_cancel 收臂。返回是否提前摆了。"""
        if log is not None:
            self.log = log
        if not self.cfg.get('pre_arm', True) or self.arm is None or self._pre_off or self._pre_pending:
            return False
        role = role_of(stop, self.cfg.get('stop_aliases'))
        if role not in ('RAW', 'ROUGH', 'TEMP') or not self._pre_useful(role):
            return False
        kind = 'RAW' if role == 'RAW' else 'RING'
        try:
            ok, reply, _ = self.arm.request(f'PRE {kind}')
        except ArmAbort as ex:
            self._aborted = True
            raise Abort(str(ex))
        except ArmError as ex:
            self.log(f'    (提前摆臂出错：{ex}；到了再摆)')
            return False
        if not ok:
            if unknown_cmd(reply):
                self._pre_off = True
                self.log('    (STM32 还是旧程序，不认识 PRE：到了再摆臂。烧了新固件才能边开车边摆臂)')
            else:
                self.log(f'    (提前摆臂没做：{reply}；到了再摆)')
            return False
        self._pre_pending = str(stop)
        self.log(f'    手臂提前转向看{"原料盘" if kind == "RAW" else "圆环"}的姿态(在最高处转)，车同时开过去')
        return True

    def pre_cancel(self, stop=None):
        """提前摆了臂、到了却没用手臂(分段调试跳过这一站、没任务码…)：收臂，带着伸出的手臂开往下一站不安全。"""
        if not self._pre_pending:
            return
        self.log(f'    (提前摆了臂，{self._pre_pending} 没用上：收臂再开走)')
        self._pre_pending = None
        if self.arm is not None:
            self._stow_quiet()

    def _pre_useful(self, role):
        """到这个工位会用手臂吗：有任务码、没关掉夹放、爪子里没东西；原料区这一批有要抓的，粗加工区/暂存区转盘里有物料。"""
        if self.disabled or self.plan is None or self.maybe_holding or self.holding is not None:
            return False
        if role == 'RAW':
            visit = self.visits.get('RAW', 0) + 1
            return visit <= 2 and bool(self.plan.items(1 if visit <= 1 else 2))
        return any(v is not None for v in self.in_tray.values())

    # ------------------------------------------------------------------ 给导航的接口：准备、计时、时间判断、保活、收臂停 60
    def prepare(self, link, log=None):
        """一键出发以后马上调(第二站动作之前)：把开跑前能做的先做掉，到了 QR 点就不用再等。
          机械臂初始化 + 收臂(STOW)、同步参数(和第一次 task 一样)；
          QR CLR：清掉 STM32 里存的旧任务码(上一轮/调试时扫过的)；(10-23)清屏(ZONE LOCK)，屏上先空着(读到码写任务码，回到启停区写抓取数、放置数)；
          打开摄像头。
        调多次没关系(做成过的不再做)；出错只记日志、不抛异常(导航照常走；到 QR 点 task 会再初始化)。返回机械臂是否就绪。
        比赛(race)时在 ZONE LOCK 之后调；这里自己也清一次屏(go 的时候屏上还留着上一次的字)。"""
        if log is not None:
            self.log = log
        if self._aborted:
            self.log('  ★ 急停过：出发前准备不做(手臂不自动动)')
            return False
        arm_ok = True
        try:
            if not self._prep.get('arm'):
                self._ensure(link)
                self._prep['arm'] = True
        except Exception as ex:                     # 包括急停(Abort)：导航自己会看 ctx.aborted()
            self.log(f'  ★ 出发前准备机械臂出错：{ex!r}(到 QR 点再试)')
            if isinstance(ex, (Abort, ArmAbort)):
                self._aborted = True
                return False
            arm_ok = False
            if self.arm is None:
                try:
                    self._ensure_arm_only(link)     # 别的出错(摄像头模块之类)：旧任务码照样要清
                except Exception:
                    return False
        if not self._prep.get('qr'):
            try:
                keep = float(self.cfg.get('qr_prescan_s', 60.0) or 0.0)
                ok = self.arm.qr_clear(keep_recent_s=keep) if keep > 0 else self.arm.qr_clear()
                if ok is False:
                    self.sleep(0.1)
                    ok = self.arm.qr_clear(keep_recent_s=keep) if keep > 0 else self.arm.qr_clear()   # 串口偶尔一次没回话：再清一次
                u = getattr(self.arm, 'usb_qr', None)
                if keep > 0 and u is not None and u.get():
                    self.log(f'  开跑前扫码器刚读到任务码 {u.get()}：到 QR 点直接用它')
                if ok is False:
                    raise ArmError('QR CLR 没有回 DONE')
                self._screen_clear()                # (10-23)清屏：上一次的任务码擦掉，读到码再写
                self._show_stats()                  # screen_stats_live=true 才写 GRAB 0、PLACE 0(默认回家才写)
                self._prep['qr'] = True
                self.log('  已清掉 STM32 里存的旧任务码(QR CLR)，屏上的字也清了(读到任务码再显示)')
            except Exception as ex:
                if isinstance(ex, ArmAbort):
                    self._aborted = True
                self.log(f'  ★ 清旧任务码出错：{ex!r}(STM32 里可能还存着上一轮/调试时的码；下次 prepare 再清)')
        if not self._prep.get('cam'):
            try:
                self._ensure_vision()
                if hasattr(self.vision, 'open'):
                    self.vision.open()
                self._prep['cam'] = True
            except Exception as ex:
                self.log(f'  ★ 打开摄像头出错：{ex!r}(到用摄像头时再试)')
        return arm_ok and not self.disabled

    def start_clock(self):
        """比赛计时从现在开始(按下屏上 START / 终端确认出发的那一刻)。之前算的"时间不够"也清掉。"""
        self.t0 = self.now()
        self.clock_started = True
        self.out_of_time = False

    def set_home_eta(self, seconds):
        """导航告诉钩子：从现在这个停车点开回启停区要多久(秒，路线估算)。None = 不知道。
        停车点里每个物料开始前按"这个物料的用时 + 收臂挪回 + 回家 + 余量"算来不来得及(见 _no_time)。"""
        try:
            self.home_eta_s = None if seconds is None else max(0.0, float(seconds))
        except (TypeError, ValueError):
            self.home_eta_s = None

    def go_home_now(self, next_role, est_leg_s, est_home_from_next_s):
        """导航开往下一个停车点之前问：去 next_role(停车点名或角色)做事，还来得及回家吗？来不及返回 True(导航从这里直接回家)。
        est_leg_s = 开过去要几秒，est_home_from_next_s = 从那里回家要几秒(导航按路线估的)。
        "做事"按最少有用的算：那里至少能做完一个物料(放一个就有分；到了以后每个物料开始前还会再按时间判断)；
        那里没活可干(没有任务码、转盘里没东西)就只看开过去再回家来不来得及。用时按实测的每个物料用时(没实测过按 work_item_s)。"""
        role = role_of(next_role, self.cfg.get('stop_aliases')) if next_role else None
        if role == 'START':
            return False
        if self.cfg.get('full_course', True):
            return False                                   # (10-11 用户)跑完全程：每个停车点都去
        if self.out_of_time:
            self.log(f'  ★ 时间不够了(前面已经停止开始新的物料)：不去 {next_role}，直接回家')
            return True
        idle = self._idle_for_good() if self.cfg.get('home_when_idle', True) else None
        if idle:
            self.log(f'  ★ {idle}：后面的停车点都没活可干，不去 {next_role}，直接回家(少走路、少压线的风险)')
            return True
        try:
            leg = max(0.0, float(est_leg_s or 0.0))
            home = max(0.0, float(est_home_from_next_s or 0.0))
        except (TypeError, ValueError):
            return False
        cfg = self.cfg
        el = self.elapsed()
        work = self._work_s(role, first_only=True)
        if (work <= 0 and role in ('RAW', 'ROUGH', 'TEMP') and self.visits.get(role, 0) >= 1
                and cfg.get('home_when_idle', True)):
            # 第二批的停车点(路线最后几个)没活可干：后面也不会再有(第二批的物料只能从这里来)，直接回家
            self.log(f'  ★ {next_role} 第二批没活可干(转盘里没有要放的/槽都占着)：后面也没有了，不去，直接回家')
            return True
        finish = float(cfg.get('stop_finish_s', 3.0)) if work > 0 else 0.0
        end = el + leg + work + finish + home + float(cfg.get('home_margin_s', 12.0))
        late = end > float(cfg.get('round_s', 180.0))
        if work > 0 and el + leg + work > float(cfg['time_limit_s']):
            late = True                                       # 到那里时已经过了 time_limit_s，一个物料也做不了
        what = f'做一个物料约 {work:.0f} 秒 + ' if work > 0 else '(那里没活可干) '
        if late:
            self.out_of_time = True
            self.log(f'  ★ 时间不够去 {next_role}：已用 {el:.0f} 秒，开过去约 {leg:.0f} 秒 + {what}回家约 {home:.0f} 秒'
                     f' + 余量 {float(cfg.get("home_margin_s", 12.0)):.0f} 秒 > {float(cfg.get("round_s", 180.0)):.0f} 秒：从这里直接回家')
        else:
            self.log(f'    时间：已用 {el:.0f} 秒，去 {next_role} {what}再回家，预计 {end:.0f} 秒(含余量)，来得及')
        return late

    def keepalive(self):
        """车长时间不动(扫描、规划、雷达定位)时手臂轻轻摆一下：ID1 转 keepalive_deg 度马上转回来(不到 1 秒)，
        让裁判看得出车还在运行(规则：停止运行 15 秒本轮结束)。只在任务线程里调(和 task 同一个线程，不会和别的机械臂指令撞上)。
        机械臂没就绪、本轮不夹放、爪子里夹着东西、离上次摆不到 keepalive_min_s 秒：什么也不做。不抛异常。返回这次摆了没有。"""
        if not self._inited or self.arm is None or self.disabled or self.holding is not None or self.maybe_holding:
            return False
        if self._aborted:
            return False                                  # 急停过：手臂一律不自动动(人手可能在附近)
        try:
            if self.ctx is not None and hasattr(self.ctx, 'aborted') and self.ctx.aborted():
                self._aborted = True
                return False
        except Exception:
            pass
        now = self.now()
        if now - self._t_alive < float(self.cfg.get('keepalive_min_s', 5.0)):
            return False
        self._t_alive = now
        d = float(self.cfg.get('keepalive_deg', 3.0) or 0.0)
        if d <= 0:
            return False
        try:
            if self._lift_bob('车不动'):                       # (10-21 用户"突然抽搐一下")升降上下，手臂不左右摆
                return True
            self.arm.do(f'AD 1 {d:g}')
            self.arm.do(f'AD 1 {-d:g}')
        except ArmAbort as ex:
            self._aborted = True
            self.log(f'    (keepalive：{ex})')
            return False
        except Exception as ex:
            self.log(f'    (keepalive 摆手臂出错：{ex!r})')
            return False
        return True

    def park(self, link=None, force=False):
        """收臂，升降停到开机高度(lift_park_mm，60mm：下次开机编码器才认得准)。STM32 认识 PARK 就用 PARK(收臂 + 停 60，
        没标定时只降升降)，旧程序不认识就 STOW + LIFT 60(STOW 失败就不降)。跑完回到启停区、退出程序时用。
        不做的情况：爪子里可能夹着物料(60 正好是放进车上转盘的高度，会压上去)；急停过(人手可能在附近)，除非 force=True(终端 park 命令)。
        link：机械臂还没连上时用它连(导航退出时可以新建一个钩子来调)。不抛异常，返回是否停好了。"""
        if self.arm is None:
            if link is None:
                self.log('  (机械臂还没连上：不收臂、不停 60)')
                return False
            self._ensure_arm_only(link)
        if self._aborted and not force:
            self.log('  ★ 急停过：不自动收臂、不降升降(人手可能在附近)。确认安全后输入 park')
            return False
        why = self._may_hold()
        if why:
            self.log(f'  ★ 不收臂停 60：{why}。先把物料拿出来(arm CLAW O)再输入 park')
            return False
        mm = self.cfg.get('lift_park_mm')
        try:
            if mm is None:
                self.arm.stow()
                self.log('  已收臂(lift_park_mm=null：升降不动)')
                return True
            mm = float(mm)
            if abs(mm - 60.0) < 0.5:
                ok, reply = self.arm.park()
                if ok:
                    self.log('  已收臂，升降停在 60mm(PARK)')
                    return True
                if ok is False:
                    self.log(f'  ★ PARK 没做成：{reply}' + ('(升降位置不知道了：把升降放到最低点，arm LIFT ZERO)' if 'NOZERO' in reply else ''))
                    return False
            P = self.arm.params() or {}                       # 旧程序：STOW + LIFT 60(没标定时只降升降，和 PARK 一样)
            if float(P.get('ARMOK', 1.0)) >= 0.5:
                self.arm.stow()
            self.arm.lift(mm)
            self.log(f'  已收臂，升降停在 {mm:g}mm')
            return True
        except ArmAbort as ex:
            self._aborted = True
            self.log(f'  ★ 收臂停 60 被急停打断：{ex}')
        except Exception as ex:
            self.log(f'  ★ 收臂/停 60 失败：{ex}')
        return False

    def _may_hold(self):
        if self.holding is not None:
            return f'爪子里夹着{self.holding.color_name}'
        return self.maybe_holding

    # ------------------------------------------------------------------ 时间：每个物料来不来得及、下一个停车点值不值得去
    def _learn_t(self, d, key, dt):
        """实测的用时：和以前的平均(新的占一半)，太离谱的不要。"""
        if not 0.3 < dt < 150.0:
            return
        old = d.get(key)
        d[key] = dt if old is None else 0.5 * old + 0.5 * dt

    def _note_item(self, kind, t0):
        dt = self.now() - t0
        self._items_t += dt
        self._learn_t(self.work_t, kind, dt)

    def _item_s(self, kind):
        """一个物料大约要多久(秒)：实测过按实测的，没有按 work_item_s。"""
        if kind in self.work_t:
            return float(self.work_t[kind])
        return float((self.cfg.get('work_item_s') or {}).get(kind, 10.0))

    def _base_s(self, role):
        if role in self.base_t:
            return float(self.base_t[role])
        return float((self.cfg.get('work_base_s') or {}).get(role, 3.0))

    def _work_deadline(self):
        """物料最晚什么时候必须做完(绝对时刻)：time_limit_s；知道回家要多久时还要 留出收臂 + 回家 + 余量。"""
        cfg = self.cfg
        t = self.t0 + float(cfg['time_limit_s'])
        if self.home_eta_s is not None:
            t = min(t, self.t0 + float(cfg.get('round_s', 180.0)) - float(cfg.get('home_margin_s', 12.0))
                    - float(cfg.get('stop_finish_s', 3.0)) - float(self.home_eta_s))
        return t

    def _no_time(self, kind, what, quiet=False):
        """再开始一个 kind 的物料(grab/place/pick/stack)来不来得及：来不及返回原因(并记下 out_of_time，让车回家)，来得及返回 None。"""
        if self.cfg.get('full_course', True):
            return None                                    # (10-11 用户)跑完全程：不按时间停
        need = self._item_s(kind)
        left = self._work_deadline() - self.now()
        if left >= need:
            return None
        if self.home_eta_s is not None:
            why = f'已用 {self.elapsed():.0f} 秒，{what}一个约 {need:.0f} 秒，再收臂、回家(约 {self.home_eta_s:.0f} 秒)就超时'
        else:
            why = f'已用 {self.elapsed():.0f} 秒，{what}一个约 {need:.0f} 秒，做完会超过 time_limit_s={float(self.cfg["time_limit_s"]):g} 秒'
        if not quiet:
            self.out_of_time = True
            self.log(f'    ★ 时间不够：{why}。不再{what}，让车回家')
            self._ui('msg', 'TIME UP')
        return why

    def _pickback_budget(self, n):
        """粗加工区放完 n 个：取回几个以后还来得及开到暂存区把它们放下再回家(取回 k 个 + 暂存区放 k 个 + 路上 + 回家 + 余量)。
        来不及的就不取回：取回要时间，物料放在粗加工区已经算了分，留在那儿不吃亏，车早点回家。
        不知道回家要多久(导航没给 set_home_eta)时不判断，返回 n。"""
        if self.home_eta_s is None or n <= 0 or self.cfg.get('full_course', True):
            return n
        cfg = self.cfg
        # 偏保守：取回了却去不成暂存区，白花时间，物料还从算了分的地方拿走了
        fixed = (2 * float(cfg.get('stop_finish_s', 3.0)) + float(cfg.get('next_leg_s', 12.0)) + self._base_s('TEMP')
                 + float(self.home_eta_s) + float(cfg.get('home_margin_s', 12.0)) + 4.0)
        per = 1.15 * (self._item_s('pick') + self._item_s('place'))
        left = float(cfg.get('round_s', 180.0)) - self.elapsed()
        k = int(max(0.0, min(float(n), (left - fixed) / max(per, 1.0))))
        if k < n:
            self.log(f'    ★ 还剩 {left:.0f} 秒：取回、再去暂存区放，每个约 {per:.0f} 秒，加上路上和回家约 {fixed:.0f} 秒，'
                     + (f'只来得及 {k} 个：其余的留在粗加工区(放下时已经算分)' if k else '一个也来不及：不取回了，物料留在粗加工区(放下时已经算分)，车直接回家'))
            if k == 0:
                self.out_of_time = True
        return k

    def _idle_for_good(self):
        """这一轮后面再也没有夹放可做了(返回原因)：到过 QR 点以后，机械臂不能用、或者还是没有任务码。否则 None。
        QR 点还没去过不算(读到任务码本身就有分，机械臂不能用也照样去读)。"""
        if not self.visits.get('QR'):
            return None
        if self.disabled:
            return f'本轮不做夹放({self.disabled})'
        if self.plan is None and self.arm is not None and self.cfg.get('qr_retry_at_stops', True):
            # 回家之前再问一次 QR?(STM32 在路上/找码挪回来时可能读到了)：不然车直接回家，后面停车点的"再问一次"永远轮不到
            try:
                self._qr_again()
            except ArmAbort as ex:
                self._aborted = True
                self.log(f'    (再问 QR? 时收到急停：{ex})')
            except Exception as ex:
                self.log(f'    (再问 QR? 出错：{ex!r})')
        if self.plan is None:
            return '没有读到任务码'
        return None

    def _work_s(self, role, first_only=False):
        """到停车点 role 要干多久的活(秒)；那里没活可干返回 0。first_only = 只算第一个物料(最少有用的活)。"""
        if role in (None, 'START'):
            return 0.0
        if role == 'QR':
            if self.plan is not None or self.disabled:
                return 0.0
            return self._base_s('QR') + float(self.cfg.get('qr_dwell_s', 0.8)) * 6
        if self.disabled or self.plan is None:
            return 0.0
        batch = 1 if self.visits.get(role, 0) + 1 <= 1 else 2
        items = self.plan.items(batch)
        if role == 'RAW':
            kinds = ['grab' for it in items if self.in_tray.get(it.slot) is None or same_item(self.in_tray.get(it.slot), it)]
        else:
            have = [it for it in items if same_item(self.in_tray.get(it.slot), it)]
            if role == 'ROUGH':
                kinds = ['place'] * len(have) + ['pick'] * len(have)
            elif batch == 1:
                kinds = ['place'] * len(have)
            else:
                kinds = ['stack' for it in have if self._stack_target(it) is not None]
        if not kinds:
            return 0.0
        if first_only:
            kinds = kinds[:1]
        return self._base_s(role) + sum(self._item_s(k) for k in kinds)

    # ------------------------------------------------------------------ 出错以后
    def _bail_out(self, role):
        """停车点里出了意外的错：把手臂收好(夹着物料就先放回转盘)、底盘挪回停车点，路线照走。收不好/挪不回去就停下(Abort)。"""
        self._pre_pending = None                    # (10-23)手臂在这里收；不清掉的话后面每一站都不再提前摆臂
        try:
            if self.holding is not None:
                self._put_back(self.holding, '出错时爪子里夹着物料')
            elif self.arm is not None and not self.maybe_holding:
                self._stow_quiet()
            if self.act is not None:
                self._return_to_stop(role)
        except Abort:
            self._aborted = True
            raise
        except Exception as ex:
            raise Abort(f'出错以后收臂/挪回停车点也失败了({ex!r})，停止路线')

    # ------------------------------------------------------------------ 初始化
    def _ensure_arm_only(self, link):
        """只建立和 STM32 的指令通道(不初始化升降、不检查 ARMOK)：给 arm / qr 这类简单测试命令用。"""
        if self.arm is None:
            self.arm = ArmLink(link, log=self.log)

    def _ensure_vision(self):
        if self.vision is None:
            cfg = self.cfg
            vc = vision_cfg(cfg)
            if cfg.get('vision_cal_file'):
                apply_vision_cal(vc, cfg['vision_cal_file'])        # vclaw 实测的爪子像素优先
            self.vision = Vision(vc, log=self.log)

    def _ensure(self, link):
        if self._inited:
            if (self.disabled and self.disabled == getattr(self, '_init_fail', None)
                    and not getattr(self, '_init_again', False)):
                # (10-23)开头初始化时 GET / LIFT? 连着没回话(串口偶尔不稳)：到下一站再初始化一次，别整轮都不夹不放
                self._init_again = True
                self.log(f'  (开头初始化机械臂没成：{self.disabled}；到这一站再试一次)')
                self.disabled = None
                self._init_arm()
                self.sync_params()
            return
        cfg = self.cfg
        self._ensure_arm_only(link)
        self._ensure_vision()
        if self.store is None:
            self.store = JacStore(cfg.get('servo_cal_file'))
        self.act = ArmActuators(self.arm, link, cfg['chassis_fine_rpm'], log=self.log)
        self.servo = VisualServo(self.act, self.store, cfg=cfg['servo'], log=self.log, sleep=self.sleep, clock=self.now)
        self._inited = True
        self._init_arm()
        self.sync_params()

    def sync_params(self):
        """按 STM32 现在的参数调整视觉这边：
        - 视觉闭环的最小一步要比舵机的到位误差 ATOL 大(不然那一步舵机根本不动)；
        - vclaw 时的观察高度(ZOBRAW/ZOBRNG)和现在不一样：提示重做 vclaw，圆环大小不再按旧的过滤。"""
        try:
            P = self.arm.params() or {}
        except ArmAbort:
            raise
        except Exception:
            return
        if self.servo is not None and 'ATOL' in P:
            ms = self.servo.cfg.setdefault('min_step_deg', {})
            for k in ('id1', 'id2'):
                ms[k] = max(float(ms.get(k, 0.0)), float(P['ATOL']) + 0.05)
        cal = load_vision_cal(self.cfg.get('vision_cal_file'))
        for key, kind in (('ZOBRAW', 'RAW'), ('ZOBRNG', 'RING')):
            if key in cal and key in P and abs(float(cal[key]) - float(P[key])) > 0.5:
                self.log(f'★ {key} 现在是 {float(P[key]):g}，vclaw {kind} 时是 {float(cal[key]):g}：爪子点已经不准，重做 vclaw {kind}')
                if kind == 'RING' and self.vision is not None and hasattr(self.vision, 'cfg'):
                    self.vision.cfg.pop('ring_rmax_cal', None)     # 圆环大小变了：别按旧的大小过滤掉
        v = self.vision
        if ('pick_ZOBRNG' in cal and 'ZOBRNG' in P and abs(float(cal['pick_ZOBRNG']) - float(P['ZOBRNG'])) > 0.5
                and v is not None and hasattr(v, 'set_pick') and v.has_pick()):
            v.set_pick(None)
            self.log(f'  ZOBRNG 现在是 {float(P["ZOBRNG"]):g}，量 PICK 时是 {float(cal["pick_ZOBRNG"]):g}：物料顶面的爪子点作废，'
                     '下次放下物料时自动重新量(或者 vclaw PICK 颜色号)')

    def _init_arm(self):
        try:
            params = self.arm.params()
            if not params or 'ARMOK' not in params:
                self.sleep(0.3)                     # (10-23)串口偶尔一次没回话：再问一次(以前一次没回话整轮都不夹不放)
                params = self.arm.params(refresh=True)
            if not params or 'ARMOK' not in params:
                self.disabled = 'STM32 里没有机械臂参数(GET 里找不到 ARMOK)：STM32 程序是不是新版(arm.c)'
            elif params['ARMOK'] < 0.5:
                self.disabled = '机械臂参数还没标定(ARMOK=0)：标定好各姿态后 set ARMOK 1'
            else:
                known, mm = self.arm.lift_state()
                if not known:
                    self.sleep(0.3)                 # (10-23)LIFT? 偶尔一次没回话(回 0 和没回话分不出来)：再问一次
                    known, mm = self.arm.lift_state()
                boot = getattr(self.arm, 'lift_boot', None)
                if known and boot == 'NOENC':
                    # (10-21 用户：原料区放下去了不夹)开机没读到升降编码器，高度是猜的(实车差过 17mm，夹的高度全偏)：现在读一次编码器认准
                    ok_s, rep_s, info_s = self.arm.request('LIFT SYNC', 5.0)
                    if ok_s:
                        known, mm = self.arm.lift_state()
                        boot = getattr(self.arm, 'lift_boot', None) or 'SYNC'
                        self.log('  升降：开机时没读到编码器，现在读到了，按编码器认准高度 ' +
                                 ' '.join(l for l in info_s if l.startswith('LIFTSYNC')))
                    elif 'NOENC' in str(rep_s):
                        self.disabled = ('升降高度不知道准不准：开机时、现在都读不到升降电机(5 号驱动器)的编码器，夹、放的高度可能偏一两厘米(夹不到、'
                                         '半空松开)。本轮不夹不放。查 5 号驱动器的插头/线，电机电源开着按一下 STM32 复位，arm LIFT? 回 BOOT=POS 再跑')
                if known:
                    self.log(f'  升降已回零，现在 {mm:.1f}mm' + (f'(开机时{BOOT_TEXT.get(boot, boot)})' if boot else ''))
                    if boot in ('NOCAL', 'NOENC'):
                        self.log(f'  ★ 开机时没用编码器认高度，当成了 {mm:.0f}mm(不一定对)。为了不顶坏升降，这次不自动收臂(不往上升)。\n'
                                 f'    先看实际高度是不是 {mm:.0f}mm：是就照常用；不是就关电，把升降手动放到最低点，开机后输入 arm LIFT ZERO。'
                                 '编码器读不到多半是升降驱动器(ID5)的插头松了')
                elif self.cfg['lift_init'] == 'home':
                    self.arm.do('LIFT HOME')
                    self.log('  升降：驱动器回零完成')
                else:
                    self.disabled = '升降位置不知道了(急停打断过升降？)：把升降放到最低点，输入 arm LIFT ZERO'
                if not self.disabled and self.cfg.get('stow_at_start', True) and \
                        not (known and boot in ('NOCAL', 'NOENC') and not self.cfg.get('stow_when_lift_guessed', False)):
                    self._stow_retry()              # 开机时两个舵机是松的：出发前先收到待机姿态(存了 STBY 就是它，否则 ID1=A1H、ID2=A2R，升到 ZHI)
                    self.log('  手臂已收到待机姿态')
        except ArmAbort as ex:
            raise Abort(str(ex))
        except (ArmError, Exception) as ex:
            self.disabled = f'初始化机械臂出错：{ex}'
        # (10-23)这几种可能只是串口偶尔没回话：记下来，到下一站再初始化一次(_ensure)
        retry = self.disabled and (self.disabled.startswith(('STM32 里没有机械臂参数', '升降位置不知道了', '初始化机械臂出错')))
        self._init_fail = self.disabled if retry else None
        if self.disabled:
            self.log(f'★ 本轮不做夹取/放置：{self.disabled}' + ('(到下一站再试一次初始化)' if retry and not getattr(self, '_init_again', False) else ''))
            self._ui('msg', 'ARM OFF')
        else:
            self.log('  机械臂就绪。')

    def close(self):
        """释放摄像头。每次 go 结束要调用，否则下一次 go 打不开 /dev/video0。"""
        v = self.vision
        if v is not None and hasattr(v, 'close'):
            try:
                v.close()
            except Exception:
                pass

    # ------------------------------------------------------------------ 小工具
    def _ui(self, key, text):
        if self.arm is None:
            return
        if key not in (self.cfg.get('screen_show') or ()):   # (10-23)屏上只写赛规要的：任务码、抓取数、放置数
            return
        obj = self.cfg['screen'].get(key)
        if obj:
            self.arm.screen(obj, text)

    def _screen_clear(self):
        """(10-23)清屏回到比赛布局(STM32 的 ZONE LOCK)：上一次跑留下的任务码、旧的字都擦掉。旧固件不认 ZONE 就算了。"""
        try:
            self.arm.request('ZONE LOCK', 2.0)
        except ArmAbort:
            raise
        except Exception:
            pass
        self.arm.screen_reset()                     # 清过屏：每个控件都重新写

    def _check_abort(self):
        if self.ctx is not None and hasattr(self.ctx, 'aborted') and self.ctx.aborted():
            self._aborted = True
            raise Abort('收到 abort，已停止')

    def elapsed(self):
        return self.now() - self.t0

    def time_left(self):
        return self.cfg['time_limit_s'] - self.elapsed()

    def _can_work(self):
        if self.disabled:
            self.log(f'    ★ 本轮不做夹放：{self.disabled}')           # (10-24)以前一声不吭就跳过这一站
            return False
        if self.plan is None:
            self.log('    没有任务码，不能做夹取/放置')
            return False
        return True

    def _show_stats(self):
        # (10-23)屏上只写做成的个数(GRAB 3、PLACE 5)，和回家以后一样：赛规要显示"正确抓取数量和正确放置数量"，3/6 这种裁判看不懂。
        # (10-23 晚用户"回启停区后显示中文抓取 n 放置 n")跑的时候不写(屏上只有任务码)，回到启停区(_start)才写；
        #   新固件把 GRAB 3 画成"抓取3"。screen_stats_live=true 跑的时候也写
        if not self.cfg.get('screen_stats_live', False):
            return
        self._ui('grab', self.stats.final_grab_text())
        self._ui('place', self.stats.final_place_text())

    def _recover(self, why):
        """单个物料出错后：尽量把手臂收好。收不好就停下整个路线(带着伸出的手臂乱走不安全)。"""
        self.log(f'    ★ {why}；收臂')
        self._pre_pending = None
        if 'ERR HOLD' in str(why):
            # (10-21)STM32 记着爪子里还夹着物料(上一个没放下)，不让 TAKE/GRAB/PICK 先张开爪子：本轮不再夹放，免得它掉在半路
            self.maybe_holding = 'STM32 说爪子里还夹着物料(上一个没放下)'
            if not self.disabled:
                self.disabled = 'STM32 说爪子里还夹着物料(上一个没放下)：本轮不再夹放，免得张开爪子把它掉在半路。拿出来：arm CLAW O'
                self.log(f'    ★ {self.disabled}')
                self._ui('msg', 'ARM OFF')
        self._ring_ready = self._at_obs = False
        try:
            self._stow_retry()
        except ArmAbort as ex:
            raise Abort(str(ex))
        except ArmError as ex:
            raise Abort(f'收臂也失败了({ex})，停止路线')

    def _stow_retry(self):
        """收臂；STM32 偶尔一次出错(总线舵机没回话之类)就歇一下再收一次，还不行抛 ArmError。"""
        try:
            self.arm.stow()
        except ArmAbort:
            raise
        except ArmError as ex:
            self.log(f'    收臂出错({ex})，再收一次')
            self.sleep(0.3)
            self.arm.stow()

    def _grab_failed(self, item, ex, what='抓'):
        """(10-21)GRAB 出错以后：按这条指令里 STM32 回的夹爪动作(CLAWSET，最后一次)看物料在哪：
        - 还没夹紧(夹爪没动过/只张开过)：物料还在原料盘上，收臂，当没夹到；
        - 夹紧了(C，夹紧以后抬起、转去车上转盘时出的错)：物料可能在爪子里，STM32 也记着"夹着物料"(后面 GRAB 都回 ERR HOLD)：
          放回它的转盘槽(在转盘上方张开，STM32 就不再当夹着；爪子是空的也没坏处)，当夹到了(其实没夹上的话，最近那格夹完再看一眼原料盘认得出来)；
        - 已经在车上转盘上方松开了(OT)：物料已经在它的转盘槽里，收臂，当夹到了。
        ERR HOLD：上一个没放下，本轮不再夹放。ERR LIFT：升降的高度认不准了，不再按它动升降，本轮不再夹放，叫重新 LIFT ZERO。
        返回 True = 物料在它的转盘槽里了(算夹到)。"""
        msg = str(ex)
        why = f'{what} {item.color_name} 出错：{ex}'
        claw = getattr(self.arm, 'claw_last', None)
        if 'ERR HOLD' in msg:
            self._recover(why)
            return False
        if 'ERR LIFT' in msg:
            self.disabled = ('升降的高度认不准了(编码器读的和程序记的差太多，可能是升降没走/被卡住)：本轮不再夹放。'
                             '车停好以后看一下升降，放到最低点，输入 arm LIFT ZERO')
            if claw == 'C':
                self.maybe_holding = f'{item.color_name}可能夹在爪子里'
            self.log(f'    ★ {why}；{self.disabled}')
            self._ui('msg', 'ARM OFF')
            if claw == 'OT':
                self.in_tray[item.slot] = item
            self._recover(why)
            return claw == 'OT'
        if claw == 'C':
            self._put_back(item, why)
            return (not self.disabled) and self.in_tray.get(item.slot) is item
        if claw == 'OT':
            self.log(f'    {item.color_name} 已经松进转盘 {item.slot} 号槽了(出错在松开以后)')
            self.in_tray[item.slot] = item
            self._recover(why)
            return True
        self._recover(why)                                     # 还没夹紧：爪子是空的
        return False

    def _put_back(self, item, why):
        """从转盘取出物料以后出错(回不到对准姿态、DROP 失败…)：把物料放回它的转盘槽，再收臂。
        不能夹着物料直接收臂：后面任何一步一张开爪子(OBS … O、TAKE、GRAB)，物料就掉在半路上。
        DROP 中途出错时爪子里可能已经空了，照样走一遍也没坏处(空爪在转盘上方张开)。
        放回也失败：爪子保持夹紧收臂，本轮不再做夹放(不再张开爪子)。"""
        self.log(f'    ★ {why}；爪子里夹着{item.color_name}，先放回转盘 {item.slot} 号槽')
        dodge, self._dodge = self._dodge, None
        try:
            P = self.arm.params() or {}
            self.arm.lift(float(P['ZHI']))
            self._to_tray_detour(P, dodge=dodge)
            self.arm.tt(item.slot)
            self.arm.lift(float(P['ZDROP']))
            self.arm.claw(True, where='TRAY')                  # (10-20)车上转盘上方张开用 CLWOT
            self.arm.lift(float(P['ZHI']))
        except ArmAbort as ex:
            raise Abort(str(ex))
        except (ArmError, KeyError, TypeError, ValueError) as ex:
            self.disabled = f'{item.color_name}可能还夹在爪子里(放回转盘失败：{ex})，本轮不再夹放，免得张开爪子把它掉在半路'
            self.maybe_holding = f'{item.color_name}可能还夹在爪子里'
            self.holding = None
            self.log(f'    ★ {self.disabled}')
            self._ui('msg', 'ARM OFF')
            self._recover('放回转盘失败')
            return
        self.holding = None
        self.in_tray[item.slot] = item
        self._recover(f'{item.color_name}已放回转盘 {item.slot} 号槽')

    def _to_tray_detour(self, P, dodge=None):
        """(10-20 用户：夹着物料转去车上转盘怕刮到电池盒)从圆环那边转到车上转盘上方(A1D/A2R)：绕电池盒的参数调过(圆环那组 R2T，
        没有就原料区那组 A2T)就照 STM32 那样分段走：转到开始伸长的角度 -> 伸长 -> 伸着转过电池盒 -> 缩回 -> 转到转盘上方；
        没调过(或 ID1 管伸缩)照旧直接转过去。
        (10-24)dodge = (ID1, ID2)：码垛躲两层垛的路上出了错(_stack_in)，手臂已经过了往里缩的地方：先在原地只往里缩、缩着只转回那里，再照上面走。"""
        def g(k):
            try:
                return float(P.get(k) or 0.0)
            except (TypeError, ValueError):
                return 0.0
        a1d, a2r = float(P['A1D']), float(P['A2R'])
        if dodge is not None:
            try:
                b1, _b2 = self.arm.read_angles()
            except ArmAbort:
                raise
            except ArmError:
                b1 = None
            d1, d2 = float(dodge[0]), float(dodge[1])
            sgn = 1.0 if (b1 or 0.0) > a1d else -1.0                # 手臂在车上转盘的哪一边
            if b1 is not None and (b1 - d1) * sgn > 1.0 and (d1 - a1d) * sgn >= -0.5:
                self.log('    (还在码垛躲避的那一段：先往里缩、缩着转回往里缩的地方，再回车上转盘)')
                self.arm.ap(b1, d2)
                self.arm.ap(d1, d2)
        ring_set = abs(g('R2T')) > 0.5
        ext, t1, t2 = (g('R2T'), g('R1T1'), g('R1T2')) if ring_set else (g('A2T'), g('A1T1'), g('A1T2'))
        if abs(ext) <= 0.5 or float(P.get('AEXT', 2.0) or 2.0) != 2.0:
            self.arm.ap(a1d, a2r)
            return
        try:
            a1, a2 = self.arm.read_angles()
        except ArmAbort:
            raise
        except ArmError:
            a1 = a2 = None
        if a1 is None or a2 is None:
            self.arm.ap(a1d, a2r)
            return
        t1 = t1 if abs(t1) > 0.5 else a1
        t2 = t2 if abs(t2) > 0.5 else a1d
        self.log('    按' + ('圆环那组' if ring_set else '原料区那组') + '绕电池盒的角度分段转回转盘(夹着物料)')
        self.arm.ap(t1, a2)                                       # 伸缩不动，转到开始伸长的角度
        self.arm.ap(t1, ext)                                      # 伸长
        self.arm.ap(t2, ext)                                      # 伸着转过电池盒
        self.arm.ap(t2, a2r)                                      # 缩回
        self.arm.ap(a1d, a2r)                                     # 转到转盘上方

    # ------------------------------------------------------------------ 分发
    def run_role(self, role, visit):
        self._check_abort()
        h = {'QR': self._qr, 'RAW': self._raw, 'ROUGH': self._rough, 'TEMP': self._temp, 'START': self._start}[role]
        try:
            h(visit)
        except ArmAbort as ex:
            self._aborted = True
            raise Abort(str(ex))

    # ------------------------------------------------------------------ QR
    def _qr(self, visit):
        """读任务码：先原地查 qr_first_wait_s 秒(路上可能已经读到了)；没读到就沿车道前后挪着找(_qr_search)，最后回到停车点。
        qr_search=false：原地最多等 qr_timeout_s 秒(以前的做法)。"""
        cfg = self.cfg
        fixed = str(cfg.get('qr_fixed_code') or '').strip()
        if fixed:
            self.log(f'    不扫二维码：用配置里手动给的任务码 qr_fixed_code = {fixed}')
            if self._use_code(fixed):
                return
            self.log('    ★ 手动给的任务码格式不对(要像 652+312+526+231)：改成正常扫码')
        self._ui('stage', 'QR SCAN')
        search = bool(cfg.get('qr_search', True))
        code = self._qr_poll(float(cfg.get('qr_first_wait_s', 0.6)) if search else float(cfg['qr_timeout_s']))
        if not code and search:
            code = self._qr_search()
        if not code:
            code = self._qr_poll(0.0)                   # 找码挪回停车点的路上可能刚读到：用备用码之前再问一次
        if not code:
            self.log('★ 没有读到二维码：先不做夹取/放置(路线照走，后面每个停车点再问一次 QR?)')
            self._ui('msg', 'QR FAIL')
            return
        self._use_code(code)

    def _qr_poll(self, dwell_s):
        """每 0.2 秒问一次 QR?，最多 dwell_s 秒(至少问一次)。读到返回任务码，没读到返回 None。偶尔一次出错不放弃。"""
        t_end = self.now() + max(0.0, float(dwell_s))
        while True:
            try:
                code = self.arm.qr()
            except ArmAbort:
                raise
            except ArmError as ex:
                self.log(f'    读二维码出错：{ex}')
                code = None
            if code or self.now() >= t_end:
                return code
            self._check_abort()
            self.sleep(0.2)

    def _qr_again(self):
        """QR 点没读到码：到后面的停车点再问一次(STM32 / USB 扫码器在路上读到的码也会存着)。"""
        try:
            code = self.arm.qr()
        except ArmAbort:
            raise
        except ArmError:
            return
        if code:
            self.log(f'  ★ 到 {self._stop_name} 时读到任务码了(路上读到的)：{code}')
            self._use_code(code)

    def _use_code(self, code):
        try:
            self.plan = parse_code(code)
        except TaskError as ex:
            self.log(f'★ 二维码内容不对：{ex}')
            self._ui('msg', 'QR BAD')
            return False
        self.log(f'    任务码 {self.plan.code}：{self.plan.describe()}')
        for w in self.plan.warnings:
            self.log(f'    注意：{w}')
        self._show_code()
        return True

    def _code_plus(self):
        """屏上任务码带不带两组之间的 +：配置要带、而且 STM32 是新程序(t0 放得下 8 个大字)。
        新程序的 LIFT? 回复带 BOOT=…(和加宽 t0 是同一版)；还没问过就问一次。旧程序：不带(7 个字，和以前一样)。"""
        if not self.cfg.get('code_plus', True) or self.arm is None:
            return False
        if getattr(self.arm, 'lift_boot', None) is None and not self._boot_asked:
            self._boot_asked = True
            try:
                self.arm.lift_state()
            except ArmAbort:
                raise
            except Exception:
                pass
        if getattr(self.arm, 'lift_boot', None) is not None:
            return True
        if not self._plus_warned:
            self._plus_warned = True
            self.log('    (STM32 是旧程序：屏上 t0 只放得下 7 个大字，任务码中间的 + 不显示；烧录新程序以后自动带上)')
        return False

    def _show_code(self):
        l0, l1 = self.plan.screen_code(self._code_plus())
        self._ui('code', l0)
        self._ui('code2', l1)
        b1, b2 = self.plan.screen_lines()
        self._ui('b1', b1)
        self._ui('b2', b2)
        self._ui('stage', 'QR OK')
        self._ui('msg', 'QR OK')
        self._show_stats()

    # ---- 读不到码：沿车道前后挪着找
    def _qr_pose(self):
        """QR 停车点的位姿 (x, y, 车头°)(配置 stops 里)；没有返回 None。"""
        stops = self.raw_cfg.get('stops') or {}
        pose = stops.get(self._stop_name) if self._stop_name else None
        if pose is None:
            pose = next((v for k, v in stops.items() if role_of(k, self.cfg.get('stop_aliases')) == 'QR'), None)
        try:
            return float(pose[0]), float(pose[1]), float(pose[2])
        except (TypeError, ValueError, IndexError):
            return None

    def _qr_offsets(self, pose):
        """读不到码时底盘依次前后挪到哪儿：[(扫码器对着的码板位置, 底盘前进多少 mm)]。
        扫码器在车的右后方(离车尾 qr_scanner_from_rear_mm、离右边 qr_scanner_from_right_mm)，底盘沿车头方向挪 F，
        扫码器沿码板方向移动 F × (车头方向在码板方向上的分量)。车头和码板垂直(挪了也对不上)返回 []。"""
        cfg = self.cfg
        x, y, h = pose
        L = float(self.raw_cfg.get('car_length_mm', 290.0))
        W = float(self.raw_cfg.get('car_width_mm', 260.0))
        sf = -L / 2.0 + float(cfg.get('qr_scanner_from_rear_mm', 20.0))     # 车身坐标：前 / 左
        sl = -W / 2.0 + float(cfg.get('qr_scanner_from_right_mm', 40.0))
        a = math.radians(h)
        fx, fy = math.cos(a), math.sin(a)
        lx, ly = -fy, fx
        scan = (x + sf * fx + sl * lx, y + sf * fy + sl * ly)
        i = 0 if str(cfg.get('qr_board_axis', 'y')).lower() == 'x' else 1
        along, k = scan[i], (fx, fy)[i]
        if abs(k) < 0.5:
            self.log(f'    (QR 停车点车头 {h:g}°，和码板方向差太多：前后挪也对不上码，不挪)')
            return []
        lim = float(cfg.get('qr_max_move_mm', 300.0))
        out, done = [], [0.0]
        for t in cfg.get('qr_targets_mm') or []:
            f = (float(t) - along) / k
            if abs(f) > lim or any(abs(f - d) < 25.0 for d in done):
                continue                                    # 太远 / 和已经看过的位置差不多(扫码器在停车点时就对着它)
            out.append((float(t), float(round(f))))
            done.append(f)
        return out

    def _qr_obstacles(self):
        """现在地图上的障碍物 [(x, y, 半径)]：导航的 ctx 有 obstacles() 才有，没有返回 None。"""
        if not (self.raw_cfg or {}).get('exec_obstacles', False):
            return []                                     # (10-11 用户)障碍物只用来规划路线，跑的时候不管
        f = getattr(self.ctx, 'obstacles', None) if self.ctx is not None else None
        if f is None:
            return None
        try:
            return [(float(o[0]), float(o[1]), float(o[2])) for o in (f() or [])]
        except Exception as ex:
            self.log(f'    (读地图障碍物出错：{ex!r})')
            return None

    def _qr_move_ok(self, pose, f0, f1, obstacles):
        """底盘从停车点前进 f0 挪到 f1(只沿车头方向)，车身(含雷达)扫过的长方形离障碍物、黄色区、工位、原料转盘、场地边够不够远。
        返回 (能不能挪, 原因)。route_plan 有 moves_clear 时也让它查一遍(用它的车身和余量)。"""
        cfg, rc = self.cfg, self.raw_cfg
        x, y, h = pose
        a = math.radians(h)
        fx, fy = math.cos(a), math.sin(a)
        lx, ly = -fy, fx
        L = float(rc.get('car_length_mm', 290.0))
        W = float(rc.get('car_width_mm', 260.0))
        ov = float(rc.get('lidar_overhang_mm', 45.0))
        u0, u1 = -L / 2.0 + min(f0, f1), L / 2.0 + max(f0, f1)          # 车身坐标：前后、左右(雷达在左边)
        v0, v1 = -W / 2.0, W / 2.0 + ov

        def dist_pt(px, py):                                # 点到扫过的长方形的距离(在里面 = 0)
            dx, dy = px - x, py - y
            u, v = dx * fx + dy * fy, dx * lx + dy * ly
            return math.hypot(max(u0 - u, 0.0, u - u1), max(v0 - v, 0.0, v - v1))

        corners = [(x + u * fx + v * lx, y + u * fy + v * ly) for u in (u0, u1) for v in (v0, v1)]
        zm = float(cfg.get('qr_zone_margin_mm', 30.0))
        om = float(cfg.get('qr_obstacle_margin_mm', 60.0))
        try:
            import route_plan as rp
            field = float(getattr(rp, 'FIELD', 2400))
            rects = list(getattr(rp, 'YELLOW', [])) + [getattr(rp, 'TEMP_ZONE', (0, 910, 150, 1490)),
                                                        getattr(rp, 'ROUGH_ZONE', (910, 0, 1490, 150))]
        except Exception:
            rp, field = None, 2400.0
            rects = [(550, 1400, 1000, 1850), (1400, 1400, 1850, 1850), (550, 550, 1000, 1000), (1400, 550, 1850, 1000),
                     (0, 910, 150, 1490), (910, 0, 1490, 150)]
        rects += [tuple(r) for r in (rc.get('extra_blocked_rects') or [])]
        if any(cx < zm or cx > field - zm or cy < zm or cy > field - zm for cx, cy in corners):
            return False, '车身会出场地(或离场地边太近)'
        for r in rects:
            x0, y0, x1, y1 = (float(v) for v in r[:4])
            # 两个长方形(车身那个是斜的)离得够不够远：按分离轴，四个方向里有一个方向上投影隔开 zm 以上就够远
            sep = False
            for ax in ((1.0, 0.0), (0.0, 1.0), (fx, fy), (lx, ly)):
                pa = [cx * ax[0] + cy * ax[1] for cx, cy in corners]
                pb = [px * ax[0] + py * ax[1] for px in (x0, x1) for py in (y0, y1)]
                if min(pa) >= max(pb) + zm or min(pb) >= max(pa) + zm:
                    sep = True
                    break
            if not sep:
                return False, f'车身会离黄色区/工位 {tuple(int(v) for v in (x0, y0, x1, y1))} 太近'
        rcx, rcy = (float(v) for v in (rc.get('raw_center_mm') or [1200, 2480])[:2])
        if dist_pt(rcx, rcy) < float(rc.get('raw_radius_mm', 150)) + zm:
            return False, '车身会离原料转盘太近'
        for ox, oy, orr in obstacles or []:
            d = dist_pt(ox, oy) - orr
            if d < om:
                return False, f'离障碍物({ox:.0f},{oy:.0f})只有 {max(0.0, d):.0f}mm'
        mc = getattr(rp, 'moves_clear', None) if rp is not None else None
        if mc is not None:
            try:
                res = mc(rc, obstacles or [], (x + f0 * fx, y + f0 * fy, h), [('F', float(f1 - f0))])
                if not res[0]:
                    return False, f'路线规划的检查不通过({res[3]}，最近 {float(res[1]):.0f}mm)'
            except Exception as ex:
                self.log(f'    (route_plan.moves_clear 出错：{ex!r}，只按自己的检查)')
        return True, ''

    def _qr_search(self):
        """到 QR 点没读到码：车沿车道前后挪(只挪车头方向 F，不横移)，让扫码器依次对着码板可能的位置(qr_targets_mm)，
        每个位置停 qr_dwell_s 秒查 QR?。挪之前检查车身扫过的地方离障碍物、黄色区、工位、场地边够不够远，不够就跳过那个位置。
        不管读没读到，最后都挪回停车点(后面的路线是从停车点算的)。急停时不再动。返回任务码或 None。"""
        cfg = self.cfg
        rest = max(0.0, float(cfg['qr_timeout_s']) - float(cfg.get('qr_first_wait_s', 0.6)))
        pose = self._qr_pose()
        if pose is None:
            self.log('    (配置 stops 里没有 QR 停车点的位置：不前后挪着找码，原地再等)')
            return self._qr_poll(rest)
        plan = self._qr_offsets(pose)
        if not plan:
            return self._qr_poll(rest)
        obstacles = self._qr_obstacles()
        if obstacles is None:
            self.log('    (没有地图障碍物信息：只按黄色区、工位、场地边检查)')
        self.log('    没读到码：沿车道前后挪，让扫码器依次对着码板 ' + '、'.join(f'{t:.0f}' for t, _f in plan) + ' 再读')
        dwell = float(cfg.get('qr_dwell_s', 0.8))
        rpm = cfg.get('qr_move_rpm')
        code = None
        aborted = False
        try:
            for target, f in plan:
                self._check_abort()
                cur = float(self.act.disp['F'])
                ok, why = self._qr_move_ok(pose, cur, f, obstacles)
                if not ok:
                    self.log(f'    扫码器对着 {target:.0f}：要{"前进" if f > cur else "后退"} {abs(f - cur):.0f}mm，{why}，跳过')
                    continue
                self.log(f'    扫码器对着 {target:.0f}：底盘{"前进" if f > cur else "后退"} {abs(f - cur):.0f}mm')
                self._ui('stage', 'QR SEARCH')
                if rpm:
                    self.act.chassis_move(0, f - cur, speed=int(rpm))
                else:
                    self.act.chassis_move(0, f - cur)
                code = self._qr_poll(dwell)
                if code:
                    self.log(f'    读到了(扫码器对着 {target:.0f} 左右)')
                    break
        except (Abort, ArmAbort):
            aborted = True
            self._aborted = True
            raise
        except ArmError as ex:
            self.log(f'    ★ 找码时底盘出错：{ex}，不再挪')
        finally:
            if not aborted:
                self._return_to_stop('QR')
        return code

    # ------------------------------------------------------------------ RAW：从原料盘抓进转盘
    def _raw(self, visit):
        if not self._can_work():
            return
        self._arm_side('RAW')                                     # (10-20)去车上转盘按原料区那组绕电池盒(A1T1~A2SPD)
        self._raw_seek_begin()
        try:
            self._raw_body(visit)
        finally:
            self._raw_slots_rt = None                             # 这次认出的停靠位、对准中心的位置只在这次原料区用
            self._raw_grabpos = None
            self._raw_center_disp = None
            self._raw_force_cap = None
        self._raw_seek_end()

    def _raw_body(self, visit):
        batch = 1 if visit <= 1 else 2
        try:
            self.arm.params(refresh=True)                         # 升降速度可能刚 set 过：算下爪提前量要用最新的
        except ArmAbort:
            raise
        except ArmError:
            pass
        # 10-10 用户定的(写死，配置改不了)：原料区来一个夹一个——爪子下面停的是要抓的哪个颜色就夹哪个，放进它自己的槽
        # (_raw_in_order 只给以前按顺序写的模拟测试用)
        self._raw_center_cols = sorted({int(it.color) for it in self.plan.items(batch)})
        if self._raw_ct() is not None and self.act is not None:
            if self._raw_ct_body(self.plan.items(batch)):
                return                                            # (10-18 用户)对准圆心 + 示教的三个停靠位动作
        if self._raw_center_on():
            if self._raw_center_setup():
                if getattr(self, '_raw_grabpos', None):
                    self._raw_grabpos_loop(self.plan.items(batch))    # 爪子对准了抓取位：车不动，要的颜色停过来就夹
                else:
                    self._raw_any(self.plan.items(batch), follow=True)    # 停靠位认出来了：_raw_any 里按停靠位抓(_raw_slot_pick)
                return
            if str(self.cfg.get('raw_center_fallback') or 'none') != 'follow':
                self.log('  ★ 原料区这次没对准原料盘中心/没认出停靠位：不抓(不改用跟随抓取；要跟随就 set raw_center_fallback follow)')
                for it in self.plan.items(batch):
                    self.log(f'    ★ {it.color_name} 没夹到')
                    self.stats.grab(False)
                self._show_stats()
                self._stow_quiet()
                return
            self.log('  改用跟随抓取(raw_center_fallback=follow)')
        if self._raw_follow_on():
            self._raw_any(self.plan.items(batch), follow=True)
            return
        if self._grab_any_order() and hasattr(self.vision, 'material_stream'):
            self._raw_any(self.plan.items(batch))               # 只有 mtest RAW n any 才走这里(不按顺序)
            return
        for item in self.plan.items(batch):
            self._check_abort()
            if self.disabled:                                     # 中途出了不能继续夹放的错(见 _put_back)
                break
            if not self.cfg.get('full_course', True) and self.time_left() < 0:    # full_course(默认)：时间到了也接着抓
                self.log('    ★ 时间到，不再抓取')
                break
            left = self.in_tray.get(item.slot)
            if left is not None and not same_item(left, item):    # 前面没放出去的物料还在这个槽里：再放进去会砸在它上面
                self.log(f'    ★ 转盘 {item.slot} 号槽里还有没放出去的 {left.color_name}，{item.color_name} 不夹')
                self.stats.grab(False)
                self._show_stats()
                continue
            ok = False
            for again in (False, True):
                try:
                    ok = self._grab_item(item)
                except (ArmError, VisionError) as ex:
                    if isinstance(ex, ArmAbort):
                        raise
                    claw = getattr(self.arm, 'claw_last', None)
                    ok = self._grab_failed(item, ex) if isinstance(ex, ArmError) else False
                    if not ok and isinstance(ex, VisionError):
                        self._recover(f'抓 {item.color_name} 出错：{ex}')
                    if (not ok and not again and isinstance(ex, ArmError) and not self.disabled and claw not in ('C', 'OT')
                            and not any(k in str(ex) for k in ('ERR HOLD', 'ERR LIFT', 'ERR NOZERO'))):
                        self.log(f'    {item.color_name} 还在原料盘上(夹爪没夹)：再抓一次')   # (10-23)串口偶尔一次没回话
                        continue
                break
            self.stats.grab(ok)
            if ok:
                self.in_tray[item.slot] = item
            self._show_stats()
        self._stow_quiet()

    def _raw_follow_on(self):
        """原料区这次用"跟随抓取"吗：raw_follow 开着、有手臂和画面的对应关系(vcal RAW 颜色号 arm)、看得到每帧几个颜色。"""
        if not self.cfg.get('raw_follow') or not hasattr(self.vision, 'material_errors'):
            return False
        if self.store is None or self.store.get('RAW', 'arm') is None:
            if not getattr(self, '_raw_follow_said', False):
                self._raw_follow_said = True
                self.log('    ★ 原料区要跟随抓取(raw_follow)，但还没有手臂和画面的对应关系：这次照旧等物料停到爪子附近。'
                         '原料盘停转、物料放在爪子下面，做一次 vcal RAW 颜色号 arm 就能跟随了')
            return False
        return True

    def _grab_any_order(self):
        """(10-22 用户"就是要全部按照这个任务码的顺序来放置和夹取")原料区一律按任务码顺序夹：配置里的 raw_any_order /
        raw_near_any_order 写着 true 也不用(日志说一次)。只有 mtest RAW n any(只这一次，测试用)才不按顺序。"""
        if getattr(self, 'raw_any_once', False):
            return True
        if (self.cfg.get('raw_any_order') or self.cfg.get('raw_near_any_order')) and not getattr(self, '_any_cfg_said', False):
            self._any_cfg_said = True
            self.log('    (配置里的 raw_any_order / raw_near_any_order(不按顺序夹)不再用：原料区按任务码顺序夹)')
        return False

    def _raw_any(self, items, follow=False):
        """跟随抓取 / 按停靠位抓 / 等停在爪子附近再夹(follow=False)。(10-22)默认按任务码顺序：每次只等下一个要夹的颜色，
        等了 raw_track_s 秒没夹到就记"没夹到"、接着夹下一个；mtest RAW n any 时才是"要抓的几个颜色里哪个停在爪子附近、
        够得着就先夹哪个"。follow=True：原料盘转的时候手臂就跟着要夹的物料走，停下马上夹(_raw_follow_pick)。"""
        cfg = self.cfg
        in_order = not self._grab_any_order()
        todo = []
        for item in items:
            left = self.in_tray.get(item.slot)
            if left is not None and not same_item(left, item):
                self.log(f'    ★ 转盘 {item.slot} 号槽里还有没放出去的 {left.color_name}，{item.color_name} 不夹')
                self.stats.grab(False)
                continue
            todo.append(item)
        if in_order:
            self.log(('  原料区(按停靠位抓)：按任务码顺序，原料盘每停一次看一眼，下一个要的颜色停在够得着的位上就直接转过去夹：'
                      if follow and getattr(self, '_raw_slots_rt', None) else
                      '  原料区(跟随抓取)：按任务码顺序，手臂跟着下一个要夹的物料转，原料盘一停就夹：' if follow else
                      '  原料区：按任务码顺序一个一个夹(停在爪子附近就对准下爪)：')
                     + '  '.join(f'{it.color_name}->{it.slot}号槽' for it in todo))
        else:
            self.log(('  原料区(按停靠位抓)：原料盘每停一次看一眼，要夹的颜色停在够得着的位上就直接转过去夹(放进它自己的槽)：'
                      if follow and getattr(self, '_raw_slots_rt', None) else
                      '  原料区(跟随抓取)：手臂跟着要夹的物料转，原料盘一停就夹，哪个先跟上先夹哪个(放进它自己的槽)：' if follow else
                      '  原料区：不按任务码顺序，哪个颜色停在爪子附近就先夹哪个(放进它自己的槽)：')
                     + '  '.join(f'{it.color_name}->{it.slot}号槽' for it in todo))
        md = self.vision.material_detector_obj() if hasattr(self.vision, 'material_detector_obj') else None
        if md is not None and hasattr(md, 'need_vmask') and any(md.need_vmask(it.color) for it in todo) and not self._warned_vmask:
            self._warned_vmask = True
            self.log('    ★ 没标定爪子区域(claw_mask.png)：蓝色/浅蓝物料挨着蓝色爪子时可能认错。赛前在 map_merge_live 里做一次 vmask')
        while todo:
            self._check_abort()
            if self.disabled:
                break
            if not self.cfg.get('full_course', True) and self.time_left() < 0:
                self.log('    ★ 时间到，不再抓取')
                break
            self._ui('stage', f'RAW GRAB {len(items) - len(todo) + 1}/3' + ('' if in_order else ' ANY'))
            wheels = (not follow) and self._raw_wheels_on()
            if not wheels:
                self._raw_recenter()                        # 只动车轮时不挪回：物料每次停在同一个地方，留在上次对准的位置(横移/前后都限着范围)
            got = None
            try:
                self.arm.obs('RAW', open_claw=True)
                if not wheels and not follow:
                    self._raw_goto_pose()
                t_end = min(self.now() + float(cfg.get('raw_track_s', 30.0)), self.t0 + (1e9 if cfg.get('full_course', True) else float(cfg['time_limit_s'])))
                cands = [(it.color, it.slot, it) for it in (todo[:1] if in_order else todo)]
                if follow:
                    self._raw_follow_now = True
                    try:
                        got = (self._raw_slot_pick(cands, 'grab', t_end, '抓') if self._raw_slots() else
                               self._raw_follow_pick(cands, 'grab', t_end, '抓'))
                    finally:
                        self._raw_follow_now = False
                else:
                    got = self._raw_stop_pick(cands, 'grab', t_end, '抓')
            except (ArmError, VisionError) as ex:
                if isinstance(ex, ArmAbort):
                    raise
                self._recover(f'抓取出错：{ex}')
            if got is None:
                if in_order and not self.disabled:
                    it0 = todo.pop(0)                       # 按顺序：这一个等不到/没夹成，记没夹到，接着夹下一个(不跳着夹)
                    self.log(f'    ★ {it0.color_name} 没夹到：接着夹下一个')
                    self.stats.grab(False)
                    self._show_stats()
                    continue
                break
            it = got[2]
            self.stats.grab(True)
            self.in_tray[it.slot] = it
            todo.remove(it)
            self._show_stats()
        for it in todo:
            self.log(f'    ★ {it.color_name} 没夹到')
            self.stats.grab(False)
        self._show_stats()
        self._stow_quiet()

    # ------------------------------------------------------------------ 原料区：车轮前后挪到够得着物料的地方
    def _raw_remember_pose(self):
        """原料区对准好、马上下爪：记下这时的 ID1、ID2(只记第一次)。后面的物料都停在差不多同一个地方，直接从这个姿态开始。"""
        if not self.cfg.get('raw_keep_pose', True) or self._raw_pose is not None or self._raw_wheels_on():
            return
        ang = getattr(self.arm, 'angle', None) or {}
        a1, a2 = ang.get(1), ang.get(2)                          # 对准最后一下 AP 回报的角度(不再发 A? 去读：下爪要快，原料盘停的时间有限)
        if a1 is not None and a2 is not None:
            self._raw_pose = (float(a1), float(a2))
            self.log(f'    记下这次夹的位置(ID1 {a1:.1f}°、ID2 {a2:.1f}°)：后面的物料都从这里开始等')

    def _raw_goto_pose(self):
        """OBS RAW 以后：第一次夹到的姿态记下了就转过去(在看原料盘的高度上小幅转)。"""
        if not self.cfg.get('raw_keep_pose', True) or self._raw_pose is None:
            return
        self.arm.ap(self._raw_pose[0], self._raw_pose[1])

    def _raw_seek_begin(self):
        self._seek = {'last': None, 'jf': None, 'stop': None, 'told': False}
        J = self.store.get('RAW', 'ch') if self.store is not None else None
        if J is not None and float(np.hypot(*np.asarray(J, float)[:, 1])) > 0.2:
            self._seek['jf'] = np.asarray(J, float)[:, 1]
        f, sv = float(self._raw_seek_F), float(self._raw_seek_S)
        if not RAW_CAR_FIXED and self.cfg.get('raw_seek', True) and self.act is not None and (abs(f) >= 3 or abs(sv) >= 3):
            self.log(f'    原料区：上次挪到 前后 {f:+.0f}mm、横移 {sv:+.0f}mm 够得着物料，直接挪过去')
            self.act.chassis_move(int(round(sv)), int(round(f)))

    def _raw_seek_end(self):
        """原料区做完：车挪回停车点(路线下一段从停车点算)。离得不远(对准圆心挪的)就只用 V 挪回去：V 不纠车头，
        S/F 走完会把车头转回 STM32 记着的方向(STM32 复位过、单站测试时那个方向可能差很多)。STM32 不认 V 才用 S/F。"""
        self._seek = None
        if self.act is None:
            return
        d = self.act.disp
        ts, tf = self._zone_back_target('RAW')
        if abs(d['S'] - ts) >= 1 or abs(d['F'] - tf) >= 1:
            if math.hypot(float(d['S']) - ts, float(d['F']) - tf) <= float(self.cfg.get('raw_vel_back_mm', 160.0)) \
                    and self._vel_goto(ts, tf, tol_mm=0.6):
                self.log('    车轮挪回停车点(只用 V，不转车头)')
            d = self.act.disp
            if abs(d['S'] - ts) >= 1 or abs(d['F'] - tf) >= 1:
                self._return_to_stop('RAW', (ts, tf))

    def _raw_recenter(self):
        """挪回原料区的工作位置：停车点，或者挪着找到的够得着物料的地方(raw_seek)。"""
        d = self.act.disp
        on = self.cfg.get('raw_seek', True) and not RAW_CAR_FIXED
        s = (float(self._raw_seek_S) if on else 0.0) - float(d['S'])
        f = (float(self._raw_seek_F) if on else 0.0) - float(d['F'])
        cd = getattr(self, '_raw_center_disp', None)
        if cd is not None:                                     # 对准过原料盘中心：工作位置就是那里
            s, f = float(cd[0]) - float(d['S']), float(cd[1]) - float(d['F'])
            if math.hypot(s, f) < 2.0:
                return
        elif math.hypot(s, f) < 20.0:
            return
        self.log(f'    底盘挪回原料区的工作位置：前进 {f:+.0f}mm、横移 {s:+.0f}mm')
        self.act.chassis_move(int(round(s)), int(round(f)))

    def _raw_seek(self, e, tag, stop_id):
        """原料盘停下了，要夹的物料扫不到/手臂够不着：车往原料盘那边(默认车的右边)挪一步(用户 10-11 定的：先 4cm，再 2cm、1cm、0.5cm，
        一次比一次少)，下一次停下再看。每次停下最多挪一次。挪了返回 True。"""
        cfg, sk = self.cfg, self._seek
        if RAW_CAR_FIXED or not cfg.get('raw_seek', True) or sk is None or self.act is None or sk.get('stop') == stop_id:
            return False
        sk['stop'] = stop_id
        steps = list(cfg.get('raw_seek_steps_mm') or [])
        k = sk.setdefault('k', 0)
        if k >= len(steps):
            if not sk['told']:
                sk['told'] = True
                d = self.act.disp
                self.log(f'  {tag} 已经挪了 前后 {d["F"]:+.0f}mm、横移 {d["S"]:+.0f}mm 还是扫不到/够不着：不再挪，原料区停车点要调')
            return False
        step = float(steps[k])
        sk['k'] = k + 1
        way = str(cfg.get('raw_seek_dir') or 'right').lower()
        sv, f = {'right': (-step, 0.0), 'left': (step, 0.0), 'forward': (0.0, step), 'back': (0.0, -step)}.get(way, (-step, 0.0))
        name = {'right': '往右(原料盘那边)', 'left': '往左', 'forward': '往前', 'back': '往后'}.get(way, '往右(原料盘那边)')
        self.log(f'  {tag} 扫不到/够不着要夹的物料：车{name}挪 {step:g}mm(第 {k + 1} 次)，下一次停下再看')
        self.act.chassis_move(int(round(sv)), int(round(f)))
        self._raw_seek_F = float(self.act.disp['F'])
        self._raw_seek_S = float(self.act.disp['S'])
        return True

    def _grab_item(self, item):
        cfg = self.cfg
        self.log(f'  ▶ 抓第{item.batch}批 #{item.index + 1}：{item.color_name} -> 转盘 {item.slot} 号槽')
        self._ui('stage', f'RAW GRAB {item.index + 1}/3 {item.color_short}')
        wheels = self._raw_wheels_on()
        if not wheels:
            self._raw_recenter()                    # 上一个物料对准时底盘挪过的话先挪回(够得着物料的)位置，免得这个物料出了视野
        self.arm.obs('RAW', open_claw=True)         # 只动车轮时手臂就停在这个姿态，不再动
        if not wheels:
            self._raw_goto_pose()
        md = self.vision.material_detector_obj() if hasattr(self.vision, 'material_detector_obj') else None
        if md is not None and hasattr(md, 'need_vmask') and md.need_vmask(item.color) and not self._warned_vmask:
            self._warned_vmask = True
            self.log(f'    ★ 没标定爪子区域(claw_mask.png)：{item.color_name}物料挨着蓝色爪子时可能认错。赛前在 map_merge_live 里做一次 vmask')
        if hasattr(self.vision, 'material_stream'):
            t_end = min(self.now() + max(float(cfg['raw_wait_s']), float(cfg.get('raw_track_s', 30.0))),
                        self.t0 + (1e9 if cfg.get('full_course', True) else float(cfg['time_limit_s'])))
            return self._raw_stop_grab(item.color, item.slot, 'grab', t_end, f'抓{item.color_short}')
        # 旧的视觉模块(没有一帧一帧认的功能)：等停稳，再闭环对准
        still, last = self.vision.wait_still(item.color, timeout_s=cfg['raw_wait_s'])
        if last is None and self._recenter(min_mm=3.0):
            self.log('    看不到，底盘挪回停车点再找一次')
            still, last = self.vision.wait_still(item.color, timeout_s=cfg['raw_wait_s'])
        if last is None:
            self.log(f'    ★ 看不到 {item.color_name} 物料，跳过')
            self._ui('msg', f'NO {item.color_short}')
            return False
        if not still:
            self.log('    原料盘还没停稳，按现在的位置先试(只动手臂，不用车轮去追)')
        allow, kw = False, {}                       # (10-11 用户：写死)原料区车不动，只动手臂
        if wheels:
            kw = self._raw_wheels_kw()              # (10-12 用户)只动车轮，手臂不动
        res = self.servo.run('RAW', lambda: self.vision.material_error(item.color), self.vision.scale('RAW'),
                             cfg['tol_mm']['RAW'], allow_chassis=allow, label=f'抓{item.color_short}', bounds=self.vision.bounds('RAW'),
                             confirm=False, **kw)
        self.log(f'    对准结果：{res}')
        acc = self._wheels_accept('RAW') if wheels else cfg['accept_mm']['RAW']
        if not res.ok and not res.err_mm <= acc:
            if not allow and not wheels:
                self.log('    (原料区对准只动手臂、车轮不动：物料停在手臂够不着的地方就跳过。车要停得让物料停下时在爪子附近)')
            self._recover(f'没对准({res.reason})，不夹，免得夹偏')
            return False
        self._raw_remember_pose()
        self.arm.grab_here(item.slot)
        return True

    # ------------------------------------------------------------------ 原料盘转一会儿、停一会儿：物料一停下就对准、下爪
    def _raw_stream(self, color):
        """一帧一帧认物料的数据流(Vision.material_stream)；没有这个功能(旧的视觉模块)返回 None。"""
        f = getattr(self.vision, 'material_stream', None)
        return f(None if color is None else int(color)) if f is not None else None

    def _raw_close_delay(self, mode):
        """从发出下爪指令到夹爪合上要多久(秒)：升降从 ZOBRAW 降到 ZGRAB(和 STM32 算等待时间的办法一样，含 LFMRG) + 夹爪合上。
        (10-21)STM32 下爪时车上转盘同时换槽，转盘转完(TTWAIT)才夹紧：两个里慢的那个(按会换槽算，保守一点)。"""
        P = self.arm.params() or {}
        try:
            sps = float(P['LFRPM']) / 60.0 * float(P['LFSPR'])
            lift = abs(float(P['ZOBRAW']) - float(P['ZGRAB'])) * float(P['LFPPM']) / sps + float(P.get('LFMRG', 0.0)) / 1000.0
        except (KeyError, TypeError, ValueError, ZeroDivisionError):
            lift = 0.45
        try:
            tt = float(P.get('TTWAIT', 800.0)) / 1000.0
        except (TypeError, ValueError):
            tt = 0.8
        return max(lift, tt) + float(self.cfg.get('raw_claw_close_s', 0.15)) + (0.04 if mode == 'lift' else 0.02)

    def _raw_clear_s(self):
        """(10-24)夹爪合上以后，还要多久物料才被抬离原料盘：STM32 夹紧后等 CLWAIT(合上那一下 _raw_close_delay 里已经算了 raw_claw_close_s)，
        再升 OVCLR(和固件 Lift_Rise_Ms 一样按 LFRPM 算，加 LFMRG 的一半)。raw_need_clear=False 返回 0。"""
        if not self.cfg.get('raw_need_clear', True):
            return 0.0
        P = self.arm.params() or {}
        try:
            sps = float(P['LFRPM']) / 60.0 * float(P['LFSPR'])
            rise = float(P.get('OVCLR', 20.0)) * float(P['LFPPM']) / sps + 0.5 * float(P.get('LFMRG', 0.0)) / 1000.0
        except (KeyError, TypeError, ValueError, ZeroDivisionError):
            rise = 0.5
        try:
            wait = float(P.get('CLWAIT', 400.0)) / 1000.0
        except (TypeError, ValueError):
            wait = 0.4
        return max(0.0, wait - float(self.cfg.get('raw_claw_close_s', 0.15))) + rise

    def _raw_stop_len(self):
        """原料盘每次停多久：看到过完整的停(从停下到开始转)就按实测的中值打九折，没看到过按配置的 raw_stop_s。"""
        seen = sorted(self.raw_stops[-5:])
        if seen:
            return 0.9 * seen[len(seen) // 2]
        return float(self.cfg.get('raw_stop_s', 5.0))

    def _raw_predict_stop_start(self, t):
        """一开始就看到原料盘停着(没看到它停下的那一刻)：按前面量到的周期(停多久 + 转多久)推算这一次是什么时候停下的。
        (10-22)还没量到完整的周期：离上次看到它停下还不到"停一次 + 转一次"的时间，就还是那一次停下(刚才在这次停下里夹了上一个，
        回来它还停着)，按那一次停下算——不能当它刚停下，不然对准完、下爪时原料盘已经转起来了。
        推算不了(没看到过它停下、太久以前量的、推算出来这会儿应该在转)返回 None。"""
        c = self.raw_cycle
        if not self.raw_stops or not self.raw_moves:
            mv = sorted(self.raw_moves[-5:])[len(self.raw_moves[-5:]) // 2] if self.raw_moves else 3.0
            win = self._raw_stop_len() + mv
            seen = [float(x) for x in (c.get('t_stop'), c.get('grab_stop')) if x is not None and 0.0 <= t - float(x) < win]
            return min(seen) if seen else None                 # 拿不准就按早的算(保守)
        if not c.get('t_stop'):
            return None
        st = sorted(self.raw_stops[-5:])[len(self.raw_stops[-5:]) // 2]
        mv = sorted(self.raw_moves[-5:])[len(self.raw_moves[-5:]) // 2]
        per = st + mv
        if per < 0.5 or t - c['t_stop'] > 90.0:
            return None
        el = (t - c['t_stop']) % per
        if el > st + 0.3:
            return None
        return t - el

    def _raw_stop_grab(self, color, slot, mode, t_end, label):
        """只抓这一个颜色(按任务码顺序时、gtest)。返回是否成功。"""
        return self._raw_stop_pick([(int(color), slot, None)], mode, t_end, label) is not None

    def _raw_stop_pick(self, cands, mode, t_end, label):
        """cands = [(颜色, 槽位, 物料), ...]。只有一个：每帧都认这个颜色(跟踪最准)；
        好几个(不按顺序抓)：每帧只看画面动没动(快)，停下以后把这几个颜色各认一次，哪个在爪子附近、够得着就对准夹哪个。
        原料盘转一会儿、停一会儿(大约转 3 秒停 6 秒，不一定准)：
          - 从画面判断盘在转还是停着(整个画面变了多少 + 物料位置变没变)，每次停、转多久都实测记下来
          - 要抓的物料停在爪子附近、手臂够得着：马上对准(只用存好的 J，最多修正几次)，下爪前再拍一帧确认没动、位置准，马上下爪
          - 这一次停下剩的时间不够、停在够不着的地方、对准中途转起来了：回到 OBS RAW 姿态，等下一次停
        mode：'grab' = GRAB slot H(夹进车上转盘)；'lift' = gtest：下降、夹紧、抬起；'nogo' = 只对准不夹。
        返回夹到(nogo：对准到)的那一项，没有返回 None。"""
        cfg, v = self.cfg, self.vision
        single = len(cands) == 1
        color = cands[0][0] if single else None
        tag = f'[{label}]'
        wheels = self._raw_wheels_on()
        J = self._raw_J()
        static_s = float(cfg.get('raw_static_s', 9.0))
        if J is None and wheels:
            self.log('    ★ 还没有原料区车轮和画面的对应关系(没做 vcal RAW 颜色号)：原料盘一停一转时现测会测错，'
                     f'要看到它连续 {static_s:.0f} 秒都不转(转盘关了)才现测。赛前先让原料盘停转，做一次 vcal RAW 颜色号(不加 arm)')
        elif J is None:
            self.log('    ★ 还没有原料区手臂和画面的对应关系(没做 vcal RAW)：原料盘一停一转时现测会测错，'
                     f'要看到它连续 {static_s:.0f} 秒都不转(转盘关了)才现测。赛前先让原料盘停转，做一次 vcal RAW 颜色号 arm')
        T = self._raw_close_delay(mode)
        still_px = float(cfg.get('raw_still_px', 6.0))
        thr = float(cfg.get('raw_motion_frac', 0.004))
        try:
            home = self.arm.read_angles()                     # OBS RAW 的姿态：没夹成就回到这里等下一次停
        except ArmAbort:
            raise
        except ArmError:
            home = (None, None)
        state, cnt, t_cand = None, 0, None                    # 原料盘：'move' / 'stop' / None(还不知道)；连着几帧和现在的状态不一样
        t_stop0 = None                                        # 这一次停下的时刻(亲眼看到它从转到停才有)
        t_move0 = None
        stop_id, tries = 0, 0                                 # 第几次停；这一次停下试了几次
        t_still = None                                        # 从什么时候起一直停着(没做 vcal 时用来判断转盘是不是关了)
        stop_seen = False                                     # 这一次停下是亲眼看到的(不是推算的)
        hist = []
        said = set()
        self.log(f'    盯着{label[1:] if (single and label.startswith("抓")) else ""}物料：原料盘一停、物料在爪子附近就马上对准下爪'
                 f'(下爪到夹住约 {T:.2f} 秒；每次停先按 {self._raw_stop_len():.1f} 秒算，看到实际的就按实际的)')
        keep_s = float(cfg.get('raw_keepalive_s', 12.0) or 0.0)
        t_arm = self.now()                                    # 手臂最后一次动的时刻
        stream = self._raw_stream(color)
        fresh = 0                                             # 这个画面流(手臂上次动完以后)看了几帧
        try:
            while True:
                t, p, moved = next(stream)
                fresh += 1
                if not single:
                    p = None                                  # 好几个颜色：只看画面动没动
                self._check_abort()
                now = self.now()
                if now > t_end:
                    self.log(f'  {tag} ★ 等了 {float(cfg.get("raw_track_s", 30.0)):.0f} 秒没等到它停在爪子附近，跳过')
                    return None
                if (keep_s > 0 and now - t_arm > keep_s and home[0] is not None and home[1] is not None
                        and not (state == 'stop' and p is not None)):
                    self.log(f'  {tag} 手臂 {now - t_arm:.0f} 秒没动了：轻轻摆一下(规则：机器人停止运行 15 秒、等转盘时 23 秒，本轮结束)')
                    d = float(cfg.get('raw_keepalive_deg', 1.0))
                    self.arm.ap(home[0] + d, home[1])
                    self.arm.ap(home[0], home[1])
                    t_arm = self.now()
                    stream.close()
                    stream = self._raw_stream(color)          # 手臂动过：重新比画面
                    hist, cnt, fresh = [], 0, 0
                    continue
                if p is not None:
                    hist = ([q for q in hist if t - q[0] <= 0.8] + [(t, p)])[-6:]
                elif hist and t - hist[-1][0] > 0.6:
                    hist = []                                 # 偶尔一帧没认到(物料挨着爪子被挡住一部分)不要紧，连着认不到才清掉
                # ---- 原料盘在转还是停着：看得到物料时看物料位置变没变(最准)；看不到时看整个画面变了多少
                now_moving = None
                if p is not None and len(hist) >= 2 and t - hist[-2][0] < 0.6:
                    prev = [q[1] for q in hist[-3:-1] if t - q[0] < 0.8]          # 和前一两帧(的中值)比，少受识别抖动影响
                    ref = np.median(np.array(prev, float), axis=0)
                    now_moving = math.hypot(hist[-1][1][0] - ref[0], hist[-1][1][1] - ref[1]) > still_px
                elif moved is not None:
                    now_moving = moved > thr
                if now_moving is not None:
                    want = 'move' if now_moving else 'stop'
                    if want == state:
                        cnt = 0
                    else:
                        cnt += 1
                        if cnt == 1:
                            t_cand = t                        # 第一帧和现在的状态不一样
                    if state is None or cnt >= 2:
                        if state == 'stop' and want == 'move':
                            t_move0 = t_cand
                            if t_stop0 is not None and stop_seen:
                                self.raw_stops.append(t_move0 - t_stop0)
                                self.log(f'  {tag} 原料盘开始转(这次停了 {t_move0 - t_stop0:.1f} 秒)')
                        elif state == 'move' and want == 'stop':
                            t_stop0 = t_cand - 0.07           # 停下大概在第一帧停着的帧之前一点点
                            stop_seen = True
                            stop_id, tries = stop_id + 1, 0
                            self.raw_cycle['t_stop'] = t_stop0
                            if t_move0 is not None:
                                self.raw_moves.append(t_stop0 - t_move0)
                            self.log(f'  {tag} 原料盘停了' + (f'(这次转了 {t_stop0 - t_move0:.1f} 秒)' if t_move0 is not None else ''))
                        elif want == 'stop':
                            # 一开始就停着：不知道已经停了多久。量到过周期就按周期推算，否则按刚停下算(果断先试)
                            t_stop0, stop_seen = self._raw_predict_stop_start(t), False
                            if t_stop0 is not None:
                                self.log(f'  {tag} 原料盘停着，按前面量到的周期算已经停了 {t - t_stop0:.1f} 秒')
                        if want == 'stop' and state != 'stop':
                            t_still = t if state is None else t_cand
                        elif want == 'move':
                            t_still = None
                        state, cnt = want, 0
                if single and (state != 'stop' or len(hist) < 2 or t - hist[-1][0] > 0.3):
                    if not hist and state == 'stop' and ('none', stop_id) not in said:
                        said.add(('none', stop_id))
                        self.log(f'  {tag} 停下了，画面里没有这个颜色的物料：等它转过来')
                    continue
                if not single and (state != 'stop' or fresh < 3):
                    continue                                  # 手臂刚动过：先看两三帧确认转盘真的还停着
                static = t_still is not None and now - t_still >= static_s     # 一直没转过：转盘是关着的
                if tries >= (5 if static else 2):
                    continue                                  # 这一次停下已经试了两次：等下一次
                # ---- 停着、看得到物料：够不够时间、够不够得着
                stop_len = self._raw_stop_len()
                left = None if (static or t_stop0 is None) else t_stop0 + stop_len - now
                if left is not None and left < T + 0.15:
                    if ('late', stop_id) not in said:
                        said.add(('late', stop_id))
                        self.log(f'  {tag} 这一次停下只剩 {max(0.0, left):.1f} 秒，来不及：等下一次停')
                    tries = 2
                    continue
                cand, lab = cands[0], label
                if single:
                    e = np.array(hist[-1][1], float) - np.array(v.claw('RAW'), float)
                else:
                    # 停下了：要抓的几个颜色各认一次(同一帧)，挑离爪子最近、手臂够得着的
                    errs = v.material_errors([c[0] for c in cands]) if hasattr(v, 'material_errors') else \
                        {c[0]: v.material_error(c[0], n=1) for c in cands}
                    seen = []
                    for c in cands:
                        q = errs.get(c[0])
                        if q is not None:
                            d = math.hypot(q[0], q[1]) / v.scale('RAW')
                            far_c = self._raw_reachable(J, np.array(q, float)) if (J is not None) else ''
                            if far_c and cfg.get('raw_force_grab', True) and d <= float(cfg.get('raw_force_max_mm', 60.0)):
                                far_c = ''
                            seen.append((d, far_c, c, np.array(q, float)))
                    near = [x for x in seen if not x[1]]
                    if not near:
                        if ('none', stop_id) not in said:
                            said.add(('none', stop_id))
                            what = '、'.join(f'{x[2][2].color_name if x[2][2] is not None else x[2][0]} {x[0]:.0f}mm' for x in seen) or '一个也没看到'
                            self.log(f'  {tag} 这次停下，爪子附近没有要夹的颜色({what})：等下一次停')
                        tries = 2
                        if self._raw_seek(None, tag, stop_id):
                            t_arm = self.now()
                            stream.close()
                            stream = self._raw_stream(color)  # 车挪过：重新比画面
                            hist, cnt, fresh = [], 0, 0
                        continue
                    d, _f, cand, e = min(near, key=lambda x: x[0])
                    if cand[2] is not None:
                        lab = f'抓{cand[2].color_short}'
                        self.log(f'  {tag} 这次停在爪子附近的是{cand[2].color_name}：先夹它(放 {cand[1]} 号槽)')
                far = self._raw_reachable(J, e) if (J is not None) else ''
                if far and cfg.get('raw_force_grab', True) and np.linalg.norm(e) / v.scale('RAW') <= float(cfg.get('raw_force_max_mm', 60.0)):
                    far = ''                                  # (10-11 用户：一定要夹)手臂转到头也去夹
                if far:
                    if ('far', stop_id) not in said:
                        said.add(('far', stop_id))
                        self.log(f'  {tag} 这次物料停在别的位置(离爪子点约 {np.linalg.norm(e) / v.scale("RAW"):.0f}mm，手臂够不着)：等它转到爪子下面')
                    tries = 2
                    if self._raw_seek(e, tag, stop_id):
                        t_arm = self.now()
                        stream.close()
                        stream = self._raw_stream(color)      # 车挪过：重新比画面
                        hist, cnt, fresh = [], 0, 0
                    continue
                probe = False
                if J is None:
                    if not static:
                        continue                              # 没有对应关系：等确定转盘是一直停着的再说
                    probe = True
                    self.log(f'  {tag} 原料盘 {now - t_still:.0f} 秒都没转，当它是停着的：先小幅动几下测出' + ('车轮' if wheels else '手臂') + '和画面的对应关系(只测这一次)')
                tries += 1
                if static:
                    deadline = now + 20.0
                elif left is not None:
                    deadline = now + left - T - 0.15                # 到这个时刻必须下爪(下爪到合上要 T 秒，再留一点余量)
                else:                                         # 一开始就停着，不知道已经停了多久：按刚看到它时才停下算(果断先试)
                    # 只动车轮(一条 S/F 指令就要一秒多)：按已经停了一半算，不然对准完原料盘可能已经转起来了
                    sl = 0.5 * stop_len if wheels else stop_len
                    deadline = now + max(0.8, (t_still if t_still is not None else now) + sl - now - T - 0.15)
                self.log(f'  {tag} 停着，离爪子点 {np.linalg.norm(e) / v.scale("RAW"):.1f}mm：马上对准'
                         + ('' if left is None else f'(这次停下大约还剩 {left:.1f} 秒)'))
                if self._raw_align_go(cand[0], cand[1], mode, e, deadline, lab, probe=probe):
                    # (10-22)这一个是在哪次停下里夹的(没看到它停下的那一刻：至少从 t_still 起就停着)：下一个回来一看还停着，就按它算还剩多久
                    self.raw_cycle['grab_stop'] = t_stop0 if t_stop0 is not None else t_still
                    return cand
                J = self._raw_J()
                if home[0] is not None and home[1] is not None and not wheels:
                    self.arm.ap(home[0], home[1])             # 回到 OBS RAW 姿态等(只动车轮时手臂没动过)
                t_arm = self.now()
                stream.close()
                stream = self._raw_stream(color)              # 手臂动过：重新比画面(不然会把手臂动当成转盘在转)；好几个颜色时 color=None 只看画面
                hist, cnt, fresh = [], 0, 0
        finally:
            stream.close()

    def _raw_follow_pick(self, cands, mode, t_end, label):
        """(10-16 用户)跟随抓取。cands = [(颜色, 槽位, 物料), ...]。手臂在 OBS RAW 的高度(看得到整个原料盘)：
          - 先把要夹的几个颜色都认一遍，挑一个手臂够得着(相对 OBS RAW 姿态不超过 raw_follow_limit_deg)、离爪子最近的跟着；
            跟上以后每帧只认这一个颜色(快)
          - 每帧算物料自己动了多少：画面里的位置变化减掉手臂这一下转动造成的(按 J 算)，所以手臂边跟边看也分得清原料盘转没转
          - 在转：手臂转到物料上方，往前多跟一点(物料速度 × raw_follow_lead_s)
          - 停了(手臂没动的两帧之间物料也没动)：手臂已经在它上方，修一两下就下爪(_raw_follow_grab)
          - 这一次停下剩的时间不够、没夹成：接着跟，等下一次停
        返回夹到的那一项，没有返回 None。"""
        cfg, v = self.cfg, self.vision
        tag = f'[{label}]'
        J = np.asarray(self.store.get('RAW', 'arm'), float)
        try:
            Jinv = np.linalg.inv(J)
        except np.linalg.LinAlgError:
            return self._raw_stop_pick(cands, mode, t_end, label)
        try:
            home = self._raw_home()                           # OBS RAW 的姿态：跟随的范围以它为中心(读不到就用 A1G、A2E，不再改成"等停到爪子附近")
        except (KeyError, TypeError, ValueError):
            return self._raw_stop_pick(cands, mode, t_end, label)
        cur = [home[0], home[1]]                              # 现在的 ID1、ID2(最后一次发的)
        pos = [home[0], home[1]]                              # 拍这一帧时手臂实际在哪(平滑跟随每帧读一次；不平滑 = 最后一次 AP 的目标)
        smooth = [bool(cfg.get('raw_follow_smooth', True)) and hasattr(self.arm, 'apn') and not getattr(self.arm, '_no_apn', False)]
        track_ms = float(cfg.get('raw_follow_track_ms', 250.0))
        sc = float(v.scale('RAW'))
        lim = dict(id1=25.0, id2=250.0)
        lim.update(cfg.get('raw_follow_limit_deg') or {})
        still_px = float(cfg.get('raw_still_px', 6.0))
        dead_px = float(cfg.get('raw_follow_deadband_mm', 3.0)) * sc
        lead_s = float(cfg.get('raw_follow_lead_s', 0.35))
        gain = float(cfg.get('raw_follow_gain', 0.9))
        keep_s = float(cfg.get('raw_keepalive_s', 12.0) or 0.0)
        T = self._raw_close_delay(mode)
        colors = [c[0] for c in cands]
        by_color = {c[0]: c for c in cands}

        def need(e):
            """偏差 e(像素)要转到的绝对角度 (ID1, ID2)，和超出跟随范围多少(0 = 够得着)。"""
            d = -Jinv @ np.asarray(e, float)                  # [ID2, ID1] 度
            a1, a2 = pos[0] + float(d[1]), pos[1] + float(d[0])
            over = max(0.0, abs(a1 - home[0]) - float(lim['id1'])) + max(0.0, abs(a2 - home[1]) - float(lim['id2'])) / 10.0
            return a1, a2, over

        def clampa(a1, a2):
            a1 = min(home[0] + float(lim['id1']), max(home[0] - float(lim['id1']), a1))
            a2 = min(home[1] + float(lim['id2']), max(home[1] - float(lim['id2']), a2))
            return a1, a2

        def go(a1, a2):
            if os.environ.get('FOLLOW_DBG'):
                self.log(f'      dbg AP {cur[0]:.1f},{cur[1]:.1f} -> {a1:.1f},{a2:.1f}')
            self.arm.ap(a1, a2)
            cur[0], cur[1] = a1, a2
            pos[0], pos[1] = a1, a2

        def track(a1, a2):
            """平滑跟随：只发目标(APN)不等，马上接着看；旧程序不认 APN 就改回一步一步的 AP。"""
            if smooth[0]:
                if self.arm.apn(a1, a2, track_ms) is not False:
                    cur[0], cur[1] = a1, a2
                    return
                smooth[0] = False
                self.log(f'  {tag} STM32 是旧程序(不认 APN)：改回"转一下、停住、看一眼"地跟。刷新的 arm.c 才能平滑跟随')
            go(a1, a2)

        def read_pos():
            """平滑跟随：读拍这一帧时手臂实际的角度。读不到返回 False(这一帧不算原料盘动没动)。"""
            try:
                b = self.arm.read_angles(tries=1)
            except ArmAbort:
                raise
            except ArmError:
                return False
            if b[0] is None or b[1] is None:
                return False
            pos[0], pos[1] = float(b[0]), float(b[1])
            return True

        if smooth[0] and not getattr(self, '_smooth_said', False):
            self._smooth_said = True
            self.log('    平滑跟随：每帧发一个新目标就接着看(手臂一直在动，不再转一下停一下)')
        target = None                                         # 正在跟的颜色
        prev = None                                           # 上一帧 (时刻, 偏差, 那时的 ID1/ID2)
        state = None                                          # 原料盘 'move' / 'stop'
        t_move0 = t_stop0 = None
        stop_seen = False
        t_still = None                                        # 这次看到它停着的第一帧(没看到它停下的那一刻时，按这时才停下算)
        skip_stop = False                                     # 这一次停下已经试过/来不及：等它再转起来
        t_arm = self.now()
        said = set()
        self.log(f'    跟随抓取：手臂跟着要夹的物料转(相对现在的姿态 ID1 ±{float(lim["id1"]):g}°、ID2 ±{float(lim["id2"]):g}°)，'
                 f'原料盘一停马上下爪(下爪到夹住约 {T:.2f} 秒)')
        while True:
            self._check_abort()
            now = self.now()
            if now > t_end:
                self.log(f'  {tag} ★ 跟了 {float(cfg.get("raw_track_s", 30.0)):.0f} 秒没夹到，跳过')
                return None
            if keep_s > 0 and now - t_arm > keep_s:
                d = float(cfg.get('raw_keepalive_deg', 1.0))
                self.log(f'  {tag} 手臂 {now - t_arm:.0f} 秒没动了：轻轻摆一下(规则：机器人停止运行 15 秒本轮结束)')
                self.arm.ap(pos[0] + d, pos[1])
                go(pos[0], pos[1])
                t_arm = self.now()
                prev = None
            if smooth[0] and not read_pos():                  # 拍这一帧前读手臂实际在哪(手臂一直在动：拍完再读就对不上了)
                prev = None
            t = self.now()
            errs = v.material_errors([target] if target is not None else colors)
            vis = {c: np.asarray(q, float) for c, q in errs.items() if q is not None}
            if target is not None and target not in vis:
                if ('lost', target) not in said:
                    said.add(('lost', target))
                    self.log(f'  {tag} 跟着的物料看不到了：换一个')
                target, prev = None, None
                continue
            if target is None:
                best = None
                for c, e in vis.items():
                    _a1, _a2, over = need(e)
                    k = (over > 0.0, over, float(np.linalg.norm(e)))
                    if best is None or k < best[0]:
                        best = (k, c)
                if best is None or best[0][0]:
                    if 'none' not in said:
                        said.add('none')
                        self.log(f'  {tag} 画面里还没有够得着的要夹的物料：等它转过来')
                    if best is not None:                      # 够不着的：手臂先往它那边转到范围边上等着
                        a1, a2 = clampa(*need(vis[best[1]])[:2])
                        if abs(a1 - cur[0]) > 0.5 or abs(a2 - cur[1]) > 2.0:
                            track(a1, a2)
                            t_arm = self.now()
                    prev = None
                    continue
                target, prev = best[1], None
                it = by_color[target][2]
                self.log(f'  {tag} 跟着{it.color_name if it is not None else target}(离爪子点 {np.linalg.norm(vis[target]) / sc:.0f}mm)')
            e = vis[target]
            moving, vel, arm_moved = None, np.zeros(2), False
            if prev is not None:
                dt = max(1e-3, t - prev[0])
                dA = np.array([pos[1] - prev[2][1], pos[0] - prev[2][0]])    # 两帧之间手臂转了多少 [ID2, ID1]
                if smooth[0]:                                 # 读回来的角度有一点抖：画面里动不到半个"静止"阈值算没动
                    arm_moved = float(np.linalg.norm(J @ dA)) > 0.5 * still_px
                else:
                    arm_moved = float(np.abs(dA).max()) > 1e-6
                motion = (e - prev[1]) - J @ dA               # 物料自己(原料盘)动了多少像素
                thr = still_px * (2.0 if arm_moved else 1.0)
                moving = float(np.linalg.norm(motion)) > thr
                vel = motion / dt
            prev = (t, e, (pos[0], pos[1]))
            if moving is not None and not (moving is False and arm_moved):
                want = 'move' if moving else 'stop'
                if want != state:
                    if state == 'stop' and want == 'move':
                        t_move0 = t
                        if t_stop0 is not None and stop_seen:
                            self.raw_stops.append(t_move0 - t_stop0)
                        skip_stop = False
                    elif want == 'stop':
                        if state == 'move':
                            t_stop0, stop_seen = t - 0.15, True
                            self.raw_cycle['t_stop'] = t_stop0
                            if t_move0 is not None:
                                self.raw_moves.append(t_stop0 - t_move0)
                            self.log(f'  {tag} 原料盘停了，物料离爪子点 {np.linalg.norm(e) / sc:.1f}mm')
                        else:
                            t_stop0, stop_seen = self._raw_predict_stop_start(t), False
                        t_still = t
                    state = want
            if moving is False and arm_moved:
                continue                                      # 好像停了，但手臂刚动过：手臂不动再拍一帧确认
            if moving is None or moving:
                ee = e + (lead_s * vel if moving else 0.0)
                a1, a2, over = need(ee)
                if over > 0.0:
                    a1, a2 = clampa(a1, a2)                   # 跟到范围边上(转出范围以后再换一个)
                a1 = cur[0] + gain * (a1 - cur[0])
                a2 = cur[1] + gain * (a2 - cur[1])
                if float(np.linalg.norm(ee)) > dead_px and (abs(a1 - cur[0]) > 0.15 or abs(a2 - cur[1]) > 0.5):
                    track(a1, a2)
                    t_arm = self.now()
                if need(e)[2] > 0.0 and moving and float(np.linalg.norm(e)) / sc > 30.0 and need(e + vel)[2] > need(e)[2]:
                    target, prev = None, None                 # 正在转出手臂够得着的范围：换一个
                continue
            # ---- 停着(手臂没动的两帧之间物料也没动)
            if skip_stop:
                continue
            if need(e)[2] > 0.0 and float(np.linalg.norm(e)) / sc > float(cfg.get('raw_grab_max_mm', 30.0)):
                self.log(f'  {tag} 停在手臂够不着的地方(离爪子点 {np.linalg.norm(e) / sc:.0f}mm)：等下一次停')
                target, skip_stop = None, True
                continue
            stop_len = self._raw_stop_len()
            left = None if t_stop0 is None else t_stop0 + stop_len - self.now()
            if left is not None and left < T + 0.15:
                self.log(f'  {tag} 这一次停下只剩 {max(0.0, left):.1f} 秒，来不及：接着跟，等下一次停')
                skip_stop = True
                continue
            guess = left is None
            if guess:
                # 没看到它停下的那一刻、也推算不出来：可能快停完了，下爪时转起来就夹空——跟着它等下一次停(看得到它停下)；
                # 一直不转(原料盘关着，raw_static_s 秒都没动)才直接夹
                if t_still is None or self.now() - t_still < float(cfg.get('raw_static_s', 9.0)):
                    if ('guess', t_still) not in said:
                        said.add(('guess', t_still))
                        self.log(f'  {tag} 物料停着，但没看到它什么时候停下的(下爪时可能转起来)：跟着它，等下一次停')
                    continue
                left = 20.0
            deadline = self.now() + left - T - 0.15
            cand = by_color[target]
            self.log(f'  {tag} 手臂已经在物料上方(离爪子点 {np.linalg.norm(e) / sc:.1f}mm)：修一下就下爪'
                     + ('(没看到它什么时候停下的，按刚停下算)' if guess else f'(这次停下大约还剩 {left:.1f} 秒)'))
            if smooth[0]:
                cur[0], cur[1] = pos[0], pos[1]               # 手臂已经停在这里了：下面一步一步修(AP)从这里算
            r = self._raw_follow_grab(cand, mode, e, deadline, J, need, clampa, go, cur)
            if r:
                return cand
            skip_stop = True
            t_arm = self.now()
            prev = None
            if r is None:
                target = None                                 # 看不到了：重新挑

    # ------------------------------------------------------------------ 原料区：车轮对准原料盘中心 -> 认停靠位 -> 停下直接抓(10-16 用户)
    def _raw_center_on(self):
        """原料区这次先对准原料盘中心再按停靠位抓吗(raw_center)：要有手臂和画面的对应关系(vcal RAW 颜色号 arm)、能动车轮。"""
        if not self.cfg.get('raw_center') or self.act is None or not hasattr(self.vision, 'material_errors'):
            return False
        return self.store is not None and self.store.get('RAW', 'arm') is not None

    def _raw_center_colors(self):
        c = self.cfg.get('raw_center_colors') or getattr(self, '_raw_center_cols', None) or [1, 2, 3, 4]
        return [int(x) for x in c]

    @staticmethod
    def _fit_circle(pts):
        """一组画面上的点拟合圆：返回 (圆心, 半径)，点太少/排成一条线返回 None。"""
        P = np.asarray(pts, float)
        if len(P) < 3:
            return None
        A = np.c_[2.0 * P, np.ones(len(P))]
        b = (P ** 2).sum(axis=1)
        try:
            sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        except np.linalg.LinAlgError:
            return None
        c = sol[:2]
        r2 = float(sol[2] + c @ c)
        if not np.all(np.isfinite(c)) or r2 <= 0:
            return None
        r = math.sqrt(r2)
        span = float(np.ptp(P, axis=0).max())
        if span < 0.5 * r:                                     # 点挤在一小段弧上：算出来的圆心不可信
            return None
        return c, r

    @staticmethod
    def _center_from_pair(p1, p2, near):
        """原料盘上三个物料隔 120°：画面里只看到相邻两个时，圆心在它们连线的垂直平分线上、离连线中点 半径/2 的地方
        (半径 = 两点距离/√3)。两边各有一个解，取离 near(爪子点，标定时就在原料盘中心上方)近的。返回 (圆心, 半径)。"""
        p1, p2 = np.asarray(p1, float), np.asarray(p2, float)
        d = p2 - p1
        L = float(np.linalg.norm(d))
        if L < 1e-6:
            return None
        r = L / math.sqrt(3.0)
        n = np.array([-d[1], d[0]]) / L
        mid = (p1 + p2) / 2.0
        c1, c2 = mid + n * r / 2.0, mid - n * r / 2.0
        c = c1 if float(np.linalg.norm(c1 - near)) <= float(np.linalg.norm(c2 - near)) else c2
        return c, r

    def _raw_plate_center(self, max_s):
        """看原料盘上的物料(这一批的三个颜色，转着也行)，算原料盘中心在画面里的位置：返回 (圆心像素, 半径像素)，认不出返回 None。
        一帧里看到 3 个物料：直接算外接圆；只看到相邻 2 个：按"三个物料隔 120°"算；只看到 1 个：攒它转过的一段弧拟合。取几次的中位数。"""
        v, cfg = self.vision, self.cfg
        colors = self._raw_center_colors()
        claw = np.array(v.claw('RAW'), float)
        sc = float(v.scale('RAW'))
        cs3, cs2, pts = [], [], []
        most, seen_cols, frames = 0, set(), 0
        ok_r = (10.0 * sc, 250.0 * sc)
        t_lim = self.now() + float(max_s)
        while self.now() < t_lim:
            self._check_abort()
            errs = v.material_errors(colors)
            frames += 1
            P = [claw + np.asarray(q, float) for q in errs.values() if q is not None]
            seen_cols.update(k for k, q in errs.items() if q is not None)
            most = max(most, len(P))
            pts.extend(P)
            got = None
            if len(P) >= 3:
                got = self._fit_circle(P)
                if got is not None and ok_r[0] <= got[1] <= ok_r[1]:
                    cs3.append(got)
            elif len(P) == 2:
                got = self._center_from_pair(P[0], P[1], claw)
                if got is not None and ok_r[0] <= got[1] <= ok_r[1]:
                    cs2.append(got)
            if len(cs3) >= 3 or (not cs3 and len(cs2) >= 5):
                break
        for cs in (cs3, cs2):
            if len(cs) >= 2:
                return (np.median(np.asarray([c for c, _r in cs]), axis=0), float(np.median([r for _c, r in cs])))
        got = self._fit_circle(pts[-40:]) if len(pts) >= 6 else None
        if got is not None and ok_r[0] <= got[1] <= ok_r[1]:
            return got
        names = '、'.join(COLOR_NAMES.get(int(c), str(c)) for c in sorted(seen_cols)) or '一个也没有'
        self.log(f'    看了 {frames} 帧：一帧里最多看到 {most} 个物料(认到过的颜色：{names}；要认的：'
                 + '、'.join(COLOR_NAMES.get(int(c), str(c)) for c in colors) + ')')
        return None

    def _raw_chassis_J(self, c0):
        """车轮和画面的对应关系(横移 S、前进 F 各 1mm，原料盘中心在画面里动多少像素)。存过(vcal RAW 颜色号)就用存的；
        没有：前后、横着各挪 raw_center_probe_mm 测一次(看的是原料盘中心，原料盘转着也能测)，存起来。返回 (J, 测完以后的圆心) 或 (None, c0)。"""
        st = self.store
        J = st.get('RAW', 'ch') if st is not None else None
        if J is not None:
            return np.asarray(J, float), c0
        p = float(self.cfg.get('raw_center_probe_mm', 15.0))
        wait = float(self.cfg.get('raw_center_wait_s', 12.0))
        self.log(f'    还没有车轮和画面的对应关系：先前后、横着各挪 {p:g}mm 测一下(测完存起来，以后不用再测)')
        self.act.chassis_move(0, int(round(p)))
        m1 = self._raw_plate_center(wait)
        if m1 is None:
            return None, c0
        self.act.chassis_move(int(round(p)), 0)
        m2 = self._raw_plate_center(wait)
        if m2 is None:
            return None, m1[0]
        J = np.c_[(m2[0] - m1[0]) / p, (m1[0] - c0) / p]       # 列：[横移 S, 前进 F]
        sc = float(self.vision.scale('RAW'))
        if abs(float(np.linalg.det(J))) < (0.3 * sc) ** 2:
            self.log('    ★ 车轮挪了，画面里原料盘中心几乎没动(或者两个方向分不开)：不对准中心')
            return None, m2[0]
        if st is not None:
            st.put('RAW', 'ch', J)
            if hasattr(st, 'set_synth'):
                st.set_synth('RAW', False)
        self.log(f'    车轮：横移 1mm 画面动 {np.linalg.norm(J[:, 0]):.2f} 像素，前进 1mm 动 {np.linalg.norm(J[:, 1]):.2f} 像素(已存)')
        return J, m2[0]

    def _raw_center_frame(self, colors, claw, near, hints=None, mats=True, ref=None):
        """这一帧画面里认圆心(车在走也行：同一帧里的物料一起算)：看到 3 个以上拟合外接圆，相邻 2 个按隔 120° 算(两个解取离 near 近的)。
        返回 (圆心, 半径, 看到几个)；认不出圆心返回 (None, None, 看到几个)。
        hints：(10-20)猜的圆心 [(x, y, 外圈半径), ...]，外圈照原来的办法认不出时按露出来的那段投票找、按它们找(圆心在画面外也行，
        Vision.plate_scan)；按猜的找到的外圈，画面里看到的物料要在物料所在的圆上(差 30% 以内)才算(防认错)；外圈只露出一小段、
        只算得出大概的圆心：不当认出来，放在 self._rc_coarse(找的时候往那边挪)。mats=False：这一帧不认物料(只认外圈，快)。
        ref(10-23)：比赛时示教存的 (外圈半径, 物料所在圆半径) 像素：认出的外圈半径和它差 raw_rim_tol 以上、只靠物料算的圆半径差
        raw_orbit_tol 以上，都当认错(认成了别的圆)，这一帧不用；self._rc_wide_ok = 这一帧认得够确定，可以放大挪车范围。"""
        sc = float(self.vision.scale('RAW'))
        plate = None
        cols = list(colors) if mats else []
        t_cap = self.now()
        if self.cfg.get('raw_center_outer', True) and hasattr(self.vision, 'plate_scan'):
            if hints and self._plate_hints_ok():
                plate, errs = self.vision.plate_scan(cols, hints=hints)
            else:
                plate, errs = self.vision.plate_scan(cols)      # 开源方案的做法：认原料盘(转盘)的外圆，物料被挡住也不影响
        else:
            errs = self.vision.material_errors(cols)
        P = [claw + np.asarray(q, float) for q in errs.values() if q is not None]
        self._rc_coarse = None
        self._rc_wide_ok = False
        rim0, orb0 = (ref or (None, None))[:2]
        if plate is not None and rim0:
            r_now = float(plate.get('r_now', plate['r']))
            if abs(r_now - float(rim0)) > float(self.cfg.get('raw_rim_tol', 0.15)) * float(rim0):
                self._rc_rej_rim = getattr(self, '_rc_rej_rim', 0) + 1    # 外圈大小和示教时对不上：认成了别的圆，不用(也不往那边挪)
                self._rc_rej_rim_px = r_now
                plate = None
        if plate is not None and plate.get('coarse'):
            self._rc_coarse = np.asarray(plate.get('c_now', plate['c']), float)   # 外圈只露出一小段：大概的圆心(只够知道往哪挪)
            plate = None
        orb = getattr(self, '_rc_orbit', None)
        if plate is None or plate.get('seek_new'):
            self._rc_conf = False                              # 新找到的(按猜的圆心)外圈：还没拿物料确认过
        if plate is not None and plate.get('seek') and P and orb:
            c_ = np.asarray(plate.get('c_now', plate['c']), float)
            if min(abs(float(np.linalg.norm(p - c_)) - orb) for p in P) > 0.3 * orb:
                plate = None                                   # 看到的物料都不在物料所在的圆上：按猜的圆心找到的外圈是错的
                self._rc_conf = False
            else:
                self._rc_conf = True                           # 看到的物料在物料圆上：确认没认错(之后跟着它的帧都算)
        # 照原来的办法认出的外圈算数；按猜的圆心找到的要物料确认过才算(往原料盘那边多挪要它)
        self._rc_sure = plate is not None and (not plate.get('seek') or bool(getattr(self, '_rc_conf', False)))
        if plate is not None:
            self._rc_wide_ok = bool(self._rc_sure)
            self._rc_rim = float(plate.get('r_now', plate['r']))
            # 10-18：用这一帧的外圈(c_now)。plate['c'] 是最近 3 帧的中值，车在走时落后一两帧，测车轮和画面的对应关系就测小了
            c = np.asarray(plate.get('c_now', plate['c']), float)
            if P:                                              # 物料所在圆的半径(推停靠位、只看到一个物料时用)：按看到的物料现量
                rr = float(np.median([np.linalg.norm(p - c) for p in P]))
                old = getattr(self, '_rc_orbit', None)
                self._rc_orbit = rr if old is None else 0.7 * old + 0.3 * rr
            r = getattr(self, '_rc_orbit', None) or float(self.cfg.get('raw_orbit_ratio', 0.77)) * float(plate.get('r_now', plate['r']))
            self._rc_pts = P
            self._rc_note(t_cap, c, claw, errs)
            return c, r, len(P)
        got = None
        if len(P) >= 3:
            got = self._fit_circle(P)
        elif len(P) == 2:
            got = self._center_from_pair(P[0], P[1], near)
        self._rc_pts = P                                       # 这一帧看到的物料(只看到一个时对准用它 + 已知半径推圆心)
        if got is None or not (10.0 * sc <= got[1] <= 250.0 * sc):
            return None, None, len(P)
        if orb0 and abs(float(got[1]) - float(orb0)) > float(self.cfg.get('raw_orbit_tol', 0.2)) * float(orb0):
            self._rc_rej_orb = getattr(self, '_rc_rej_orb', 0) + 1        # 物料所在圆的大小和示教时对不上：认错了物料，不用
            return None, None, len(P)
        self._rc_wide_ok = bool(orb0) and len(P) >= 3             # 三个物料都看到、圆的大小也对：够确定
        self._rc_note(t_cap, got[0], claw, errs)
        return got[0], got[1], len(P)

    def _rc_note(self, t, c, claw, errs):
        """(10-21)认出圆心的这一帧：记下每个物料绕原料盘中心转到了多少度(_rc_plate_stop 看原料盘什么时候停下的)。"""
        ang = {}
        for col, q in errs.items():
            if q is not None:
                u = np.asarray(claw, float) + np.asarray(q, float) - np.asarray(c, float)
                ang[int(col)] = math.degrees(math.atan2(float(u[1]), float(u[0])))
        if ang:
            h = self.__dict__.setdefault('_rc_hist', [])
            h.append((float(t), ang))
            del h[:-16]

    def _rc_plate_stop(self):
        """(10-21 用户"对准了之后就应该立刻识别颜色然后夹取啊为什么要等待那么久")对准原料盘中心时每帧顺便记下了物料绕原料盘中心的角度
        (车在走也看得出原料盘转没转：看的是物料相对圆心转了没有)。对准完原料盘停着、对准的时候看着它停下的：返回停下的时刻
        (保守一点，按看到它还在转的最后一帧算，多算一点已经停了多久)；最后两帧还在转、看不清、一直停着(不知道是什么时候停下的)返回 None。"""
        H = list(getattr(self, '_rc_hist', None) or [])
        thr = float(self.cfg.get('raw_still_deg', 3.0))
        still = 0
        for k in range(len(H) - 1, 0, -1):
            (_t1, a1), (t0, a0) = H[k], H[k - 1]
            common = [col for col in a1 if col in a0]
            if not common:
                return None
            dd = [(a1[col] - a0[col] + 180.0) % 360.0 - 180.0 for col in common]
            if len(dd) >= 2:                                   # 原料盘转 = 看到的物料都往同一边转(车在走时圆心认偏一点不算)
                turned = all(abs(x) > thr for x in dd) and (all(x > 0 for x in dd) or all(x < 0 for x in dd))
                jitter = max(abs(x) for x in dd) > thr
            else:                                              # 只看到一个：要转得更多才算
                turned = abs(dd[0]) > 2.0 * thr
                jitter = abs(dd[0]) > thr
            if turned:
                return t0 if still >= 1 else None
            if jitter:
                return None                                    # 说不清转没转：不当知道它是什么时候停下的
            still += 1
        return None

    def _plate_hints_ok(self):
        """Vision.plate_scan 认不认 hints(老的/测试用的假摄像头不认)。"""
        got = getattr(self, '_plate_hints', None)
        if got is not None and got[0] is self.vision:
            return got[1]
        import inspect
        try:
            ok = 'hints' in inspect.signature(self.vision.plate_scan).parameters
        except (TypeError, ValueError):
            ok = False
        self._plate_hints = (self.vision, ok)
        return ok

    def _raw_center_align(self, goal=None, tol=None, fresh=False, rng_c=None, lim=None, search=None):
        """让爪子点(看原料盘的姿态)对准原料盘中心。返回 (圆心像素, 半径像素) 或 None(认不出圆心、测不出车轮和画面的对应关系)；
        对准了没有看 self._rc_last['ok']。goal = 画面里的一个点(10-18：示教时圆心的位置)：把圆心对到这里；
        tol = 对准到多少毫米(None = raw_center_tol_mm)；fresh = 车轮和画面的对应关系现测(示教时)；
        lim = (横移, 前后) 车轮最多挪多少毫米(None = raw_center_s_mm、raw_center_f_mm)。
        STM32 认识 V：车轮连续慢慢走、每帧看(_raw_center_align_smooth，测不出来就停下，不再改成一步一步挪)；
        只有 STM32 是旧程序(不认 V)才一步一步挪(下面)。"""
        self._rc_last = dict(ok=False, err_mm=None, why='')
        if self.cfg.get('raw_center_smooth', True) and hasattr(self.act, 'vel') and not getattr(self.act, '_no_vel', False):
            r = self._raw_center_align_smooth(goal=goal, tol=tol, fresh=fresh, rng_c=rng_c, lim=lim, search=search)
            if r != 'nofw':
                return r
            self.log('    STM32 是旧程序(不认 V 指令)：改成一步一步挪。刷新的 main.c / chassis.c 才能连续顺滑地挪')
        cfg, v = self.cfg, self.vision
        sc = float(v.scale('RAW'))
        claw = np.array(v.claw('RAW'), float) if goal is None else np.asarray(goal, float)   # 下面"爪子点"都换成要对到的点
        where = '爪子点' if goal is None else '示教时的位置'
        wait = float(cfg.get('raw_center_wait_s', 12.0))
        tol = float(cfg.get('raw_center_tol_mm', 4.0)) if tol is None else float(tol)
        step = float(cfg.get('raw_center_step_mm', 20.0))
        smax, fmax = (float(x) for x in lim) if lim else (float(cfg.get('raw_center_s_mm', 40.0)), float(cfg.get('raw_center_f_mm', 60.0)))
        m = self._raw_plate_center(wait)
        if m is None:
            m = self._raw_center_search(wait)
        if m is None:
            self.log('    ★ 挪了几个地方都认不出原料盘中心：不对准中心')
            return None
        c, r = m
        self.log(f'    原料盘中心离{where} {np.linalg.norm(c - claw) / sc:.1f}mm(原料盘半径约 {r / sc:.0f}mm)')
        if float(np.linalg.norm(c - claw)) / sc <= tol:
            self._rc_last = dict(ok=True, err_mm=float(np.linalg.norm(c - claw)) / sc, why='')
            return c, r
        J, c = self._raw_chassis_J(c)
        if J is None:
            self._rc_last = dict(ok=False, err_mm=float(np.linalg.norm(c - claw)) / sc, why='测不出车轮和画面的对应关系')
            return c, r
        for _k in range(int(cfg.get('raw_center_iters', 5))):
            e = c - claw
            if float(np.linalg.norm(e)) / sc <= tol:
                break
            ds, df = (float(x) for x in -np.linalg.solve(J, e))
            d = self.act.disp
            ds = max(-smax - float(d['S']), min(smax - float(d['S']), max(-step, min(step, ds))))
            df = max(-fmax - float(d['F']), min(fmax - float(d['F']), max(-step, min(step, df))))
            if abs(ds) < 1.0 and abs(df) < 1.0:
                self.log(f'    ★ 车轮挪到范围边上了(横移 ±{smax:g}mm、前后 ±{fmax:g}mm)：停在这里')
                break
            self.log(f'    车轮慢慢挪：横移 {ds:+.0f}mm、前进 {df:+.0f}mm')
            self.act.chassis_move(int(round(ds)), int(round(df)))
            m = self._raw_plate_center(wait)
            if m is None:
                self.log('    ★ 挪了以后认不出原料盘中心：停在这里')
                break
            c, r = m
            self.log(f'    原料盘中心离{where} {np.linalg.norm(c - claw) / sc:.1f}mm')
        em = float(np.linalg.norm(c - claw)) / sc
        self._rc_last = dict(ok=em <= tol, err_mm=em, why='' if em <= tol else f'还差 {em:.1f}mm')
        return c, r

    @staticmethod
    def _j_rescaled(J, sc):
        """(10-26)车轮和画面的对应关系只是大小不对(横移、前进两列在画面里还是差不多垂直)：方向照用，每列大小改成 sc。
        方向也不对(或者数据坏了)返回 None。"""
        J = np.asarray(J, float)
        if J.shape != (2, 2) or not np.all(np.isfinite(J)) or not float(sc) > 0:
            return None
        n = np.linalg.norm(J, axis=0)
        if float(n.min()) < 1e-6 or abs(float(J[:, 0] @ J[:, 1])) / float(n[0] * n[1]) > 0.5:
            return None
        return J / n * float(sc)

    @staticmethod
    def _raw_J_bad(J, sc, rms=None, moved=None, pmm=None):
        """(10-18)车轮和画面的对应关系 J(列：横移 1mm、前进 1mm，原料盘中心在画面里动多少像素)合不合理：合理返回 ''，不合理返回原因。
        摄像头装在手臂上、看原料盘的姿态固定：车走 1mm，画面里该动约 px_per_mm.RAW 像素，横移和前进在画面里差不多垂直。"""
        J = np.asarray(J, float)
        if J.shape != (2, 2) or not np.all(np.isfinite(J)):
            return '数据不对'
        n1, n2 = float(np.linalg.norm(J[:, 0])), float(np.linalg.norm(J[:, 1]))
        for nm, n in (('横移', n1), ('前进', n2)):
            if not 0.5 * sc <= n <= 2.0 * sc:
                return (f'{nm} 1mm 画面里动 {n:.2f} 像素，应该约 {sc:.2f}'
                        + ('(画面里圆心几乎没跟着动：车轮真的走了吗？)' if n < 0.5 * sc else ''))
        cosang = abs(float(J[:, 0] @ J[:, 1])) / (n1 * n2)
        if cosang > 0.5:
            return f'横移和前进在画面里几乎是同一个方向(夹角 {math.degrees(math.acos(min(1.0, cosang))):.0f}°，应该约 90°)'
        if moved is not None and pmm and min(moved) < 0.5 * float(pmm):
            return f'车只走了 {min(moved):.0f}mm(要走 {float(pmm):g}mm)：车轮没走起来？'
        if rms is not None and rms > 2.0 * sc:
            return f'圆心一帧一帧跳得厉害(拟合误差 {rms / sc:.1f}mm)'
        return ''

    def _vel_goto(self, s_t, f_t, tol_mm=0.8, max_s=None, vmax=None):
        """(10-18)车轮只用 V 速度指令挪到 disp = (s_t, f_t)(相对停车点，毫米)。V 不纠车头(S/F 走完会把车头转回 STM32 记着的方向，
        STM32 复位过以后那个方向可能差很多)，车不会转；走了多少按 STM32 自己算的(V?)。到了返回 True；
        STM32 不认 V、读不到 V?、到时间还没到返回 False(原因在 self._vel_why，调用的地方改用 S/F 或者报错)。
        max_s = 最多挪多久(None = 按距离算)；V? 偶尔读不到就再读，连着 3 次读不到才算。
        (10-21 实车"V 慢慢走不了：9 秒还没挪到(还差 0.7mm)")串口来回一次 0.1~0.2 秒、电机按整数 转/分 走(1 转/分约 4mm/s)：
        以前按"还差多少走多快"一直修，快到了总冲过头、来回摆，到了也是停车前量的(停下来又多走一毫米)。现在：
        ① 按读到的位置 + 新指令生效之前车还会走的那一段(现在的速度 × 串口来回的时间)预判，够了就先停；
        ② 只差一两毫米(按 1 转/分 走一次都嫌多)：一个方向一个方向按 1 转/分 走算好的一小段时间就停；
        ③ 车停下以后再读一次，在 tol_mm 以内才算到了。"""
        act = self.act
        self._vel_why = ''
        if act is None or not hasattr(act, 'vel') or getattr(act, '_no_vel', False):
            self._vel_why = 'STM32 不认 V(旧程序)'
            return False
        lat = [float(self.cfg.get('vel_latency_s', 0.1))]    # 串口来回一次多久(秒)：每次读 V? 时量

        def read():
            for k in range(3):
                t0 = self.now()
                try:
                    v1 = act.vel_read()
                except ArmAbort:
                    raise
                except ArmError:
                    v1 = None
                if v1 is not None:
                    lat[0] = 0.6 * lat[0] + 0.4 * min(1.0, max(0.0, self.now() - t0))
                    return np.array(v1, float)
                self.sleep(0.05 * (k + 1))
            return None
        v0 = read()
        if v0 is None:
            self._vel_why = '读不到 STM32 走了多少(V?)'
            return False
        d0 = np.array([float(act.disp['S']), float(act.disp['F'])])
        goal = np.array([float(s_t), float(f_t)])
        vmax = float(self.cfg.get('raw_center_search_speed', 30.0) if vmax is None else vmax)
        vmin = float(self.cfg.get('raw_center_min_speed', 5.0))
        v1rpm = float(self.cfg.get('vel_one_rpm_mmps', 4.0))     # 1 转/分 车走多快(前进约 4.1、横移约 3.9mm/s)
        dist0 = float(np.linalg.norm(goal - d0))
        if max_s is None:
            max_s = 6.0 + 2.5 * dist0 / max(vmin, 1.0)
        ok, cur, vcmd, pulses = False, d0, np.zeros(2), 0
        t_lim = self.now() + float(max_s)
        try:
            while True:
                self._check_abort()
                v1 = read()
                if v1 is None:
                    self._vel_why = '读不到 STM32 走了多少(V?)'
                    break
                cur = d0 + (v1 - v0)
                d = goal - cur
                dn = float(np.linalg.norm(d))
                moving = float(np.linalg.norm(vcmd)) > 1e-9
                tol_ = tol_mm if pulses == 0 else max(tol_mm, 0.55 * v1rpm * lat[0])   # 1 转/分 最短也要走"串口来回"那么久
                if dn <= tol_ and not moving:
                    ok = True                                  # 停下来以后量的：真的到了
                    break
                if self.now() >= t_lim:
                    self._vel_why = f'{max_s:.0f} 秒还没挪到(还差 {dn:.1f}mm)'
                    break
                pred = d - vcmd * lat[0]                       # 新指令生效之前车还按现在的速度走 lat 秒
                pn = float(np.linalg.norm(pred))
                if moving and (pn <= tol_mm or float(pred @ vcmd) <= 0.0):
                    act.vel(0.0, 0.0)                          # 够了(再走就过头)：先停，停下再量
                    vcmd = np.zeros(2)
                    self.sleep(0.05)
                    continue
                per = 2.0 * lat[0] + 0.03                      # 这条速度大约管多久(到下一条生效)
                sp = min(vmax, pn / max(per, 0.05))
                if sp < max(vmin, 1.05 * v1rpm):               # 只差一点：按 1 转/分 走算好的时间(一个方向一个方向走)
                    ax = 0 if abs(float(pred[0])) >= abs(float(pred[1])) else 1
                    if abs(float(pred[ax])) <= 0.5 * tol_mm:
                        ax = 1 - ax
                    vv = np.zeros(2)
                    vv[ax] = math.copysign(1.05 * v1rpm, float(pred[ax]))
                    if not act.vel(float(vv[0]), float(vv[1])):
                        self._vel_why = 'STM32 不认 V(旧程序)'
                        break
                    self.sleep(max(0.0, abs(float(pred[ax])) / v1rpm - lat[0]))   # 停车指令发过去那一下还要走一点
                    act.vel(0.0, 0.0)
                    vcmd = np.zeros(2)
                    pulses += 1
                    self.sleep(0.05)
                    continue
                vv = pred / pn * sp
                if not act.vel(float(vv[0]), float(vv[1])):
                    self._vel_why = 'STM32 不认 V(旧程序)'
                    break
                vcmd = vv
                self.sleep(0.03)
        finally:
            try:
                act.vel(0.0, 0.0)
                v1 = read()
                if v1 is not None:
                    cur = d0 + (v1 - v0)
            except ArmError:
                pass
            act.disp['S'], act.disp['F'] = float(cur[0]), float(cur[1])
        return ok

    def _raw_center_align_smooth(self, start=None, target_dir=None, goal=None, tol=None, fresh=False, rng_c=None, lim=None,
                                 search=None):
        """(10-16 用户；10-18 按实车重写)车轮连续、顺滑地慢慢走，一边走一边每帧算圆心。只用 V 速度指令，不发 S/F：
        V 不纠车头(S/F 走完会把车头转回 STM32 记着的方向，STM32 复位过以后车会突然转一大圈)。
          ① 停车点看不到圆心：一边按 raw_center_search_mm 的路线慢慢走一边看，连着几帧认出圆心就停；
          ② 车轮和画面的对应关系 J：没有、只测过前后、存的不合理，或者 fresh=True(示教时每次现测)：慢慢前进、横移各一小段，
             边走边记圆心怎么动，算出来先检查(_raw_J_bad)，合理才用、才存；测不出来/不合理就挪回原处停下(不再一步一步乱挪)；
          ③ 按圆心偏差给车轮速度(越近越慢)，一直到对准(停着连 3 帧在 tol 以内)。
        实车上认一帧要零点几秒到一秒多，STM32 0.4 秒收不到新的 V 就停车：识别放在后台线程，这边每 raw_vel_keep_s 秒续一次速度，
        车一直在走；每帧记下拍照那一刻车走了多少(STM32 自己算的 V?，不按时间估)；圆心用这一帧的(几帧的中值车在走时会落后)。
        start = (圆心, 半径)：已经认出圆心了，跳过 ①；target_dir = 单位向量(画面里)：对准的不是圆心，是"圆心 + 半径 × target_dir"
        (原料盘上的抓取位)；goal = 画面里的一个点(示教时圆心的位置)：把圆心对到这里，不是对到爪子点；tol = 对准到多少毫米；
        rng_c = (横移, 前进)：车轮能挪的范围(±raw_center_s_mm、±raw_center_f_mm)以这里为中心(默认停车点；10-18：rteach RAW shift
        挪过车，比赛时以挪过的地方为中心)；lim = (横移, 前后) 能挪的范围(None = raw_center_s_mm、raw_center_f_mm；
        10-18 rteach RAW near 示教时车可能要挪得远一些)。
        search(10-20 用户"偏差大一点也要一直识别对准；不能写死先向左再向右挪着看，要按偏差算")：最近那格比赛时用(_raw_near_search 给)：
          ① 不按写死的路线找：每帧按"车对准时该在能挪的范围里哪儿"猜圆心在画面里的位置(按 J 算)，外圈露出一段就认得出(圆心在画面外也行)，
             认出来直接算车该挪多少；外圈没认出、只看到一个物料：按物料所在圆的半径推圆心大概在哪，直接往那边挪；
             外圈、物料都看不到(偏得太多)才在能挪的范围里从近到远一处一处看，一圈没找到再找一圈；
             能挪的范围：往原料盘那边 toward、往外 away、前后 along 毫米(_raw_near_box：按地图查过不出场地、不碰障碍物)；
          ② 存过车轮和画面的对应关系就直接用(比赛时不再挪着测)；
          ③ 对准时连着 lost_s 秒认不出圆心：停下回到 ① 接着找；挪到范围边上了还差就停下(不干等)；①②③ 一共最多 find_s 秒；
             认出外圈以后每 raw_center_mats_every 帧才认一次物料(只认外圈快得多)。
        结果另外放在 self._rc_last = dict(ok=对准了没有, err_mm=最后还偏多少毫米, why=没对准的原因)。
        返回 (圆心, 半径) / None(认不出圆心、测不出对应关系) / 'nofw'(STM32 不认 V)。"""
        import threading
        cfg, v, act = self.cfg, self.vision, self.act
        tdir = None if target_dir is None else np.asarray(target_dir, float)
        what = '原料盘中心' if tdir is None else '抓取位'

        def tgt(c_, r_):
            return c_ if tdir is None else c_ + r_ * tdir
        sc = float(v.scale('RAW'))
        claw = np.array(v.claw('RAW'), float)
        goalp = claw if goal is None else np.asarray(goal, float)
        where = '爪子点' if goal is None else '示教时的位置'
        colors = self._raw_center_colors()
        tol = float(cfg.get('raw_center_tol_mm', 4.0)) if tol is None else float(tol)
        vmax = float(cfg.get('raw_center_speed', 15.0))
        vmin = float(cfg.get('raw_center_min_speed', 5.0))
        vsearch = float(cfg.get('raw_center_search_speed', 30.0))
        smax, fmax = (float(x) for x in lim) if lim else (float(cfg.get('raw_center_s_mm', 40.0)), float(cfg.get('raw_center_f_mm', 60.0)))
        wait = float(cfg.get('raw_center_wait_s', 12.0))
        keep_s = float(cfg.get('raw_vel_keep_s', 0.15))
        threaded = bool(cfg.get('raw_frame_thread', True))
        self._rc_last = dict(ok=False, err_mm=None, why='')
        ref_px = None
        if search:                                             # (10-23)比赛时：认出的外圈、物料圆要和示教时一样大才用
            ref_px = (float(search['rim_px']) if search.get('rim_px') else None,
                      float(search['orbit_px']) if search.get('orbit_px') else None)
            if ref_px[0]:
                self._rc_rim = ref_px[0]                       # 按示教时的外圈大小猜、找(不用上一次认出的：上一次可能认错过)
            if ref_px[1]:
                self._rc_orbit = ref_px[1]
        self._rc_rej_rim = self._rc_rej_orb = 0
        self._rc_ref_last = ref_px                             # 对准以后等原料盘时顺便看车有没有被带偏(_raw_ct_watch)：也按它查大小
        meas_every = float(cfg.get('raw_meas_every_s', 0.5))
        base = np.array([float(act.disp['S']), float(act.disp['F'])])
        rc = np.zeros(2) if rng_c is None else np.asarray(rng_c, float)[:2]
        lo, hi = -np.array([smax, fmax]), np.array([smax, fmax])        # 车轮能挪的范围(相对 rc)：[横移, 前进] 的下限、上限
        J0, t_dead = None, None
        if search:
            J0 = self._raw_J_stored(fresh)
            lo, hi = self._raw_near_box(search, J0, claw, goalp, rc)
            t_dead = self.now() + float(search.get('find_s', 90.0))
        try:
            v0 = act.vel_read()
        except ArmAbort:
            raise
        except ArmError:
            v0 = None
        if not act.vel(0.0, 0.0):
            return 'nofw'
        v0 = None if v0 is None else np.array(v0, float)
        st = dict(p=np.zeros(2), tp=self.now(), cmd=np.zeros(2), stop_at=None, clamped=False,   # p：车走了多少(相对开始时)，tp：那时的时刻
                  t_meas=-1e9, t_cmd=self.now(), wide_told=False)  # t_meas：上次真读 V? 的时刻；t_cmd：速度上次变的时刻
        tm = dict(ser=0.0, mat=0.0, rim=0.0, n=0)                     # 每秒那行日志里报：串口读车走了多少、认物料、认外圈各花多少
        try:
            v.live_goal = (float(tgt(goalp, 0.0)[0]), float(tgt(goalp, 0.0)[1])) if tdir is None else None
        except Exception:
            pass

        def pos_est():
            return st['p'] + st['cmd'] * (self.now() - st['tp'])

        def pos_meas():
            """车走了多少：STM32 自己算的 V 走过的距离(V?)；读不到才按发的速度推。"""
            p = None
            t0 = time.perf_counter()
            if v0 is not None:
                try:
                    v1 = act.vel_read()
                except ArmAbort:
                    raise
                except ArmError:
                    v1 = None
                if v1 is not None:
                    p = np.array(v1, float) - v0
            tm['ser'] += time.perf_counter() - t0
            if p is None:
                p = pos_est()
            st['p'], st['tp'] = p, self.now()
            st['t_meas'] = self.now()
            return p

        def pos_cap(exact=False):
            """(10-23 用户"识别慢")拍这一帧时车走了多少：测车轮和画面的对应关系(exact)、刚停下还没读过、离上次读超过 raw_meas_every_s
            才真读 V?(一次 0.2 秒左右)，其余按发的速度推算。以前每帧都读，一秒只认 2 帧。"""
            stopped = float(np.linalg.norm(st['cmd'])) < 1e-9
            if (exact or meas_every <= 0.0 or self.now() - st['t_meas'] >= meas_every
                    or (stopped and st['t_meas'] < st['t_cmd'])):
                return pos_meas()
            return pos_est()

        def send(vs, vf, run_s=None):
            """发车轮速度(到范围边上的方向不再走，st['clamped'] 记下)。run_s：最多走这么久就停(下一帧认得慢也不会走过头)。"""
            p = pos_est()
            st['p'], st['tp'] = p, self.now()
            ps, pf = base[0] + float(p[0]) - rc[0], base[1] + float(p[1]) - rc[1]
            clamped = False
            if (ps >= hi[0] and vs > 0) or (ps <= lo[0] and vs < 0):
                vs, clamped = 0.0, True
            if (pf >= hi[1] and vf > 0) or (pf <= lo[1] and vf < 0):
                vf, clamped = 0.0, True
            act.vel(vs, vf)
            if float(np.linalg.norm(np.array([vs, vf], float) - st['cmd'])) > 1e-9:
                st['t_cmd'] = self.now()
            st['cmd'], st['clamped'] = np.array([vs, vf], float), clamped
            moving = abs(vs) > 1e-9 or abs(vf) > 1e-9
            st['stop_at'] = (self.now() + float(run_s)) if (moving and run_s is not None) else None

        def keep():
            """识别一帧的时候：到点了就停，不然续一次速度(STM32 0.4 秒收不到新的 V 就停车)。"""
            if float(np.linalg.norm(st['cmd'])) < 1e-9:
                return
            if st['stop_at'] is not None and self.now() >= st['stop_at']:
                send(0.0, 0.0)
            else:
                stop_at = st['stop_at']
                send(float(st['cmd'][0]), float(st['cmd'][1]))
                st['stop_at'] = stop_at

        def finish():
            """停车，V 走过的距离记进 disp(挪回停车点、路线下一段按它算)。"""
            act.vel(0.0, 0.0)
            st['cmd'], st['stop_at'] = np.zeros(2), None
            p = pos_meas()
            act.disp['S'] = float(base[0] + p[0])
            act.disp['F'] = float(base[1] + p[1])

        def go_back():
            """测不出/认不出：车挪回开始的地方(只用 V)。"""
            finish()
            if float(np.linalg.norm(np.array([act.disp['S'], act.disp['F']], float) - base)) >= 1.0:
                self._vel_goto(float(base[0]), float(base[1]))

        def recog(near, hints, mats):
            if hints is None and mats:
                return self._raw_center_frame(colors, claw, near)            # 和以前一样(测试里有换掉它的)
            if ref_px is None:
                return self._raw_center_frame(colors, claw, near, hints, mats)
            return self._raw_center_frame(colors, claw, near, hints, mats, ref_px)

        def frame(near, hints=None, mats=True, exact=False):
            """认一帧圆心，返回 (圆心, 半径, 看到几个物料, 拍这一帧时车走了多少)。车在走：识别放后台线程，这边每 keep_s 秒续一次速度。
            exact：拍这一帧时车走了多少要真读 STM32(测车轮和画面的对应关系时)。"""
            p_cap = pos_cap(exact)
            if not threaded or float(np.linalg.norm(st['cmd'])) < 1e-9:
                cc, rr, n = recog(near, hints, mats)
            else:
                box = {}

                def work():
                    try:
                        box['r'] = recog(near, hints, mats)
                    except BaseException as ex:                # 识别出错：交给这边抛出
                        box['e'] = ex
                th = threading.Thread(target=work, daemon=True)
                th.start()
                try:
                    while th.is_alive():
                        th.join(keep_s)
                        if th.is_alive():
                            self._check_abort()
                            keep()
                except BaseException:
                    th.join(3.0)                               # 等识别线程停下再往外抛(别让它和下一次识别一起用摄像头)
                    raise
                if 'e' in box:
                    raise box['e']
                cc, rr, n = box['r']
            ms = getattr(v, 'last_scan_ms', None)
            if isinstance(ms, dict):
                tm['mat'] += float(ms.get('mat', 0.0))
                tm['rim'] += float(ms.get('rim', 0.0))
            tm['n'] += 1
            return cc, rr, n, p_cap

        def timing():
            n = max(1, tm['n'])
            out = f'(一帧：物料 {tm["mat"] / n:.0f}ms、外圈 {tm["rim"] / n:.0f}ms、串口 {1000.0 * tm["ser"] / n:.0f}ms)'
            tm.update(ser=0.0, mat=0.0, rim=0.0, n=0)
            return out

        rej_seen = [0, 0]

        def rej_text():
            """(10-23)上一行日志以后，因为大小和示教时对不上扔掉了几帧(认成了别的圆)。"""
            a, b = int(getattr(self, '_rc_rej_rim', 0)), int(getattr(self, '_rc_rej_orb', 0))
            da, db = a - rej_seen[0], b - rej_seen[1]
            rej_seen[:] = [a, b]
            out = []
            if da and ref_px and ref_px[0]:
                out.append(f'外圈大小不对 {da} 帧(认成半径 {float(getattr(self, "_rc_rej_rim_px", 0.0)) / sc:.0f}mm 的圆，'
                           f'示教时 {ref_px[0] / sc:.0f}mm)')
            if db:
                out.append(f'物料所在圆大小不对 {db} 帧')
            return ('  扔掉：' + '、'.join(out)) if out else ''

        def clampv(d, lim):
            n = float(np.linalg.norm(d))
            return d if n <= lim else d * (lim / n)

        def probe(c, p_c):
            """慢慢前进、横移各 raw_center_probe_mm，边走边认圆心(记下拍每一帧时车走了多少)，拟合 J 并检查。
            返回 (J 或 None, 最后认出的圆心, 拍它时车走了多少)。"""
            pmm = float(cfg.get('raw_center_probe_mm', 15.0))
            spd = float(cfg.get('raw_center_probe_speed', 10.0))
            X, Y, moved = [np.asarray(p_c, float)], [np.asarray(c, float)], []
            for dirv in (np.array([0.0, 1.0]), np.array([1.0, 0.0])):
                p0 = pos_meas()
                t_lim = self.now() + 2.5 * pmm / spd + 3.0
                while True:
                    self._check_abort()
                    cc, _r, _n, pc = frame(c, exact=True)
                    if cc is not None and float(np.linalg.norm(cc - c)) < 40.0 * sc:
                        X.append(pc)
                        Y.append(np.asarray(cc, float))
                        c, p_c = np.asarray(cc, float), pc
                    left = pmm - float(np.linalg.norm(pos_meas() - p0))   # 按现在走到哪算(拍这一帧以后车又走了一点)
                    if left <= 0.3 or self.now() > t_lim:
                        break
                    send(float(dirv[0] * spd), float(dirv[1] * spd), run_s=max(0.1, left / spd))   # 走够了就停(认得慢也不走过头)
                    if float(np.linalg.norm(st['cmd'])) < 0.1:
                        break                                  # 到范围边上走不动了
                send(0.0, 0.0)
                moved.append(float(np.linalg.norm(pos_meas() - p0)))
            cc, _r, _n, pc = frame(c, exact=True)              # 停稳以后再看一帧
            if cc is not None and float(np.linalg.norm(cc - c)) < 40.0 * sc:
                X.append(pc)
                Y.append(np.asarray(cc, float))
                c, p_c = np.asarray(cc, float), pc
            if len(X) < 5:
                self.log(f'    ★ 边走边看只有 {len(X)} 帧认出了圆心(至少要 5 帧)：看实时画面(python3 vview.py)，圆盘外圈认出来没有')
                return None, c, p_c
            A = np.c_[np.asarray(X), np.ones(len(X))]
            Yv = np.asarray(Y)
            sol, *_ = np.linalg.lstsq(A, Yv, rcond=None)
            J = sol[:2].T                                      # 列：[横移 S, 前进 F]
            rms = float(np.sqrt(((Yv - A @ sol) ** 2).sum(axis=1).mean()))
            bad = self._raw_J_bad(J, sc, rms=rms, moved=moved, pmm=pmm)
            self.log(f'    车轮：横移 1mm 画面动 {np.linalg.norm(J[:, 0]):.2f} 像素，前进 1mm 动 {np.linalg.norm(J[:, 1]):.2f} 像素'
                     f'(应该各约 {sc:.2f})；车走了 前进 {moved[0]:.0f}mm、横移 {moved[1]:.0f}mm，{len(X)} 帧，拟合误差 {rms / sc:.1f}mm'
                     + (f'  ★ 不合理：{bad}' if bad else '(已存)'))
            if bad:
                return None, c, p_c
            if self.store is not None:
                self.store.put('RAW', 'ch', J)
                if hasattr(self.store, 'set_synth'):
                    self.store.set_synth('RAW', False)
            return J, c, p_c

        # ---- (search)按偏差直接算：猜圆心、只看到一个物料往哪挪、都看不到时从近到远看
        sk = dict(search or {})
        sk_J = [J0]                                            # ①里用的 J(存的；② 现测了以后换成测的)
        last = dict(c=None, p=None)                            # 上次认出的圆心、拍它时车走了多少(跟丢了再找时先猜它)

        def rim_px():
            orb = getattr(self, '_rc_orbit', None) or sk.get('orbit_px')
            return (getattr(self, '_rc_rim', None) or sk.get('rim_px')
                    or (float(orb) / max(0.3, float(cfg.get('raw_orbit_ratio', 0.77))) if orb else None))

        def prior(cur):
            """车对准时该在 rc(示教时停的地方)：圆心现在最可能在画面里哪儿。"""
            J_ = sk_J[0]
            return goalp if J_ is None else goalp + J_ @ (cur - rc)

        def hyps(cur):
            """给 plate_scan 的猜的圆心(外圈认不出时，它还会按露出来的那段外圈投票找)：最可能的(车对准时在 rc)、上次认出的(跟丢了时)。"""
            R = rim_px()
            out = [prior(cur)]
            J_ = sk_J[0]
            if last['c'] is not None and J_ is not None:
                out.insert(0, last['c'] + J_ @ (cur - (base + last['p'])))
            return [(float(q[0]), float(q[1]), R) for q in out]

        def clip_abs(p_abs):
            return rc + np.minimum(np.maximum(np.asarray(p_abs, float) - rc, lo), hi)

        def widen(c_, r_, p_):
            """认出圆心了：对准时车该停的地方就是示教时车停的地方(相对原料盘一样，不会碰到原料盘)。它在能挪的范围外(导航停偏了)
            就把范围放大到包住它(多 10mm)，最多离示教时停的地方 max_mm；再远说明导航停车点偏太多了，挪到边上为止。
            往原料盘那边超出原来的范围：外圈要是照原来的办法认出的、或者看到的物料在物料圆上(确认没认错)才放大(认错了会撞原料盘)。"""
            if not getattr(self, '_rc_wide_ok', True):
                if not st['wide_told']:                        # (10-23)认得不确定(外圈是按露出来的一段找的、物料也没对上)：不放大，挪近了认清楚再说
                    st['wide_told'] = True
                    self.log('    认出的圆心还不确定(外圈只露出一段、画面里的物料也没对上)：先不放大挪车范围，挪近了认清楚再说')
                return
            need = (base + p_) - np.linalg.solve(sk_J[0], tgt(c_, r_) - goalp) - rc      # 对准时车该在哪(相对 rc)
            cap = float(sk.get('max_mm', 150.0))
            want = np.clip(need, -cap, cap)
            k_t = int(sk.get('toward_axis', 0))
            tv = (sk.get('toward_vec') or (0.0, 0.0))[k_t]
            if tv and not getattr(self, '_rc_sure', True):
                lim_t = float(sk.get('toward', 40.0))
                if tv * want[k_t] > lim_t:
                    want[k_t] = tv * lim_t
                    self.log(f'    外圈是按露出来的一段找到的、画面里没有物料确认：往原料盘那边先只挪原来的 {lim_t:.0f}mm，挪近了认清楚再说')
            lo2, hi2 = np.minimum(lo, want - 10.0), np.maximum(hi, want + 10.0)
            if np.any(lo2 < lo - 1e-6) or np.any(hi2 > hi + 1e-6):
                lo[:], hi[:] = lo2, hi2
                self.log(f'    对准时车要停在(相对示教时停的地方)横移 {need[0]:+.0f}mm、前进 {need[1]:+.0f}mm：车停偏了，'
                         f'能挪的范围放大到{self._raw_box_text(lo, hi, sk)}(对准的地方就是示教时车停的地方，不会碰到原料盘)')
            if np.any(np.abs(need) > cap + 1.0):
                self.log(f'    ★ 要挪超过 {cap:.0f}mm 才对得上：导航停车点偏太多了，先挪到 {cap:.0f}mm 为止。把这几行发给我')

        def spots(cur, done):
            """外圈、物料都看不到：能挪的范围里每隔 step_mm 一处，从近到远(往原料盘那边的先看)；看过的不再看(都看过了再来一圈)。"""
            step = float(sk.get('step_mm', 40.0))
            ss = np.linspace(lo[0], hi[0], max(2, int(math.ceil((hi[0] - lo[0]) / step)) + 1))
            ff = np.linspace(lo[1], hi[1], max(2, int(math.ceil((hi[1] - lo[1]) / step)) + 1))
            tv = np.asarray(sk.get('toward_vec') or (0.0, 0.0), float)
            out = [rc + np.array([s_, f_]) for s_ in ss for f_ in ff]
            left = [p for p in out if all(float(np.linalg.norm(p - q)) > 0.5 * step for q in done)] or out
            return sorted(left, key=lambda p: float(np.linalg.norm(p - cur)) - 0.3 * float((p - rc) @ tv))

        def seek(first):
            """search 模式的 ①：边看边按看到的东西直接算往哪挪，认出圆心(停着连 3 帧差不多)就返回 (圆心, 半径, 拍那帧时车走了多少)；
            到 t_dead 还没认出返回 None。first：刚到(先原地看 wait_s 秒)；不然是对准时跟丢了(只原地看 1 秒)。"""
            hist, done = [], []
            t_look = self.now() + (float(sk.get('wait_s', 4.0)) if first else 1.0)
            plan, k, t_at, mode, most, t_hold = None, 0, None, '', 0, -1e9
            one = dict(p=None, pc=None)                        # 上一帧只看到的那一个物料(看原料盘停没停)
            while self.now() < t_dead:
                self._check_abort()
                cur = base + pos_est()
                c, r, n, pc = frame(prior(cur), hyps(cur), True)
                most = max(most, n)
                if c is not None:
                    send(0.0, 0.0)                             # 认出来了：停下再看两帧，差不多才算
                    t_hold = self.now()
                    hist.append((np.asarray(c, float), float(r), pc))
                    last3 = hist[-3:]
                    if len(last3) == 3 and max(float(np.linalg.norm(a[0] - last3[-1][0])) for a in last3) <= 8.0 * sc:
                        return (np.median([a[0] for a in last3], axis=0), float(np.median([a[1] for a in last3])),
                                np.median([a[2] for a in last3], axis=0))
                    continue
                if self.now() - t_hold < 2.0:
                    send(0.0, 0.0)                             # 刚认出过：停着多看两帧
                    continue
                J_ = sk_J[0]
                est, how = None, ''
                p1 = np.asarray(self._rc_pts[0], float) if (n == 1 and getattr(self, '_rc_pts', None)) else None
                if getattr(self, '_rc_coarse', None) is not None:
                    est, how = np.asarray(self._rc_coarse, float), 'coarse'      # 外圈只露出一小段：大概的圆心
                elif p1 is not None and J_ is not None:
                    still = one['p'] is not None and float(np.linalg.norm(p1 - (one['p'] + J_ @ (pc - one['pc'])))) <= 8.0
                    one['p'], one['pc'] = p1, pc
                    if still:                                  # 只看到一个物料、原料盘停着：车停远了只看得到最近那格上的(示教时它在爪子下)
                        vn = sk.get('near_vec')
                        if vn is not None:
                            est = p1 + np.asarray(vn, float)
                        else:
                            orb = getattr(self, '_rc_orbit', None) or sk.get('orbit_px')
                            u = prior(cur) - p1
                            if orb and float(np.linalg.norm(u)) > 1e-6:
                                est = p1 + u / float(np.linalg.norm(u)) * float(orb)
                        how = 'one'
                if p1 is None:
                    one['p'] = None
                if est is not None and J_ is not None:
                    to = clip_abs(cur - np.linalg.solve(J_, est - goalp))
                    if mode != how:
                        d_ = to - cur
                        self.log('    ' + ('外圈只露出一小段，算出圆心大概在哪' if how == 'coarse' else
                                           '外圈没认出来，只看到一个物料(原料盘停着)：按示教时那一格到圆心的位置推圆心大概在哪')
                                 + f'：车轮直接往那边挪(横移 {d_[0]:+.0f}mm、前进 {d_[1]:+.0f}mm)')
                    mode, plan = how, None
                    d = to - cur
                    if float(np.linalg.norm(d)) >= 2.0:
                        vv = clampv(d * 1.5, vsearch)
                        send(float(vv[0]), float(vv[1]), run_s=float(np.linalg.norm(d)) / max(1e-6, float(np.linalg.norm(vv))))
                    else:
                        send(0.0, 0.0)
                    t_look = max(t_look, self.now() + 1.0)     # 挪到了还认不出：原地多看一秒再去别处找
                    continue
                if mode in ('coarse', 'one') and self.now() < t_look:
                    send(0.0, 0.0)
                    continue
                if self.now() < t_look:
                    send(0.0, 0.0)
                    continue
                if plan is None or k >= len(plan):
                    if plan is not None and k >= len(plan):
                        done.clear()
                    plan, k = spots(cur, done), 0
                    if mode != 'blind':
                        self.log(f'    外圈、物料都没看到(一帧里最多看到过 {most} 个物料)：在能挪的范围里从近到远一处一处看'
                                 f'({self._raw_box_text(lo, hi, sk)})')
                    mode = 'blind'
                d = plan[k] - cur
                if float(np.linalg.norm(d)) < 2.0:
                    send(0.0, 0.0)
                    t_at = t_at or self.now()
                    if self.now() - t_at > float(sk.get('dwell_s', 1.0)):
                        done.append(plan[k])
                        k, t_at = k + 1, None
                else:
                    t_at = None
                    vv = clampv(d * 1.5, vsearch)
                    send(float(vv[0]), float(vv[1]), run_s=float(np.linalg.norm(d)) / max(1e-6, float(np.linalg.norm(vv))))
            send(0.0, 0.0)
            ls = getattr(v, 'last_seek', None) or {}
            nr = ls.get('near')
            self.log(f'    ★ 找了 {float(sk.get("find_s", 90.0)):.0f} 秒还是认不出原料盘中心(一帧里最多看到 {most} 个物料；外圈也没认出来'
                     + (f'；最像外圈的一段：露出 {nr[0]:.0f}°(要 90°)、残差 {nr[1]:.1f} 像素(要 4 以内)、边缘强度 {nr[2]:.0f}(要 12 以上)'
                        if nr else '') + ')')
            return None

        hist, most = [], 0
        try:
            # ---- ① 找圆心(找不到就边走边找)
            found = None
            if start is not None:
                found = (np.asarray(start[0], float), float(start[1]), pos_meas())
            elif search:
                found = seek(True)
                if found is None:
                    self._rc_last['why'] = f'找了 {float(sk.get("find_s", 90.0)):.0f} 秒认不出原料盘中心'
                    go_back()
                    return None
            spots0 = [] if (start is not None or search) else [(0.0, 0.0)] + [
                (max(-smax, min(smax, float(p[0]))), max(-fmax, min(fmax, float(p[1])))) for p in (cfg.get('raw_center_search_mm') or [])]
            for k, spot in enumerate(spots0):
                if k == 1:
                    self.log(f'    停车点认不出原料盘中心：车轮边走边找(一帧里最多看到过 {most} 个物料)')
                t_end = self.now() + (wait if k == 0 else 60.0)
                t_at = None
                while self.now() < t_end and found is None:
                    self._check_abort()
                    c, r, n, pc = frame(hist[-1][0] if hist else claw)
                    most = max(most, n)
                    if c is not None:
                        hist.append((np.asarray(c, float), float(r), pc))
                        last3 = hist[-3:]
                        if len(last3) == 3 and max(float(np.linalg.norm(a[0] - last3[-1][0])) for a in last3) <= 8.0 * sc:
                            found = (np.median([a[0] for a in last3], axis=0), float(np.median([a[1] for a in last3])),
                                     np.median([a[2] for a in last3], axis=0))
                            break
                    d = np.array(spot, float) + rc - (base + pos_est())
                    if k == 0 or float(np.linalg.norm(d)) < 2.0:
                        send(0.0, 0.0)
                        t_at = t_at or self.now()
                        if k > 0 and self.now() - t_at > 1.5:
                            break                              # 到了这一处、看了一会儿：去下一处
                    else:
                        vv = clampv(d * 1.5, vsearch)
                        send(float(vv[0]), float(vv[1]), run_s=float(np.linalg.norm(d)) / max(1e-6, float(np.linalg.norm(vv))))
                if found is not None:
                    break
            if found is None:
                self.log(f'    ★ 边走边找也认不出原料盘中心(一帧里最多看到 {most} 个物料；外圈也没认出来)')
                self._rc_last['why'] = '认不出原料盘中心'
                go_back()
                return None
            send(0.0, 0.0)
            c, r, p_c = np.asarray(found[0], float), float(found[1]), np.asarray(found[2], float)
            if start is None:
                self.log(f'    原料盘中心离{where} {np.linalg.norm(c - goalp) / sc:.1f}mm(原料盘半径约 {r / sc:.0f}mm)')
            # ---- ② 车轮和画面的对应关系
            if search and J0 is not None:
                J = J0                                         # 比赛时：存的就用(不再挪着测)
            else:
                J = None if fresh else (self.store.get('RAW', 'ch') if self.store is not None else None)
                if J is not None and hasattr(self.store, 'synth') and self.store.synth('RAW'):
                    J = None                                   # 存的只测过前后(横移是补出来的)：重新测
                if J is not None:
                    bad = self._raw_J_bad(J, sc)
                    if bad:
                        self.log(f'    存的车轮和画面的对应关系不合理({bad})：重新测')
                        J = None
            if J is None:
                self.log(f'    车轮慢慢前进、横移各 {float(cfg.get("raw_center_probe_mm", 15.0)):g}mm，边走边看圆心怎么动(测车轮和画面的对应关系)')
                J, c, p_c = probe(c, p_c)
                if J is None:
                    self._rc_last['why'] = '测不出车轮和画面的对应关系'
                    self.log('    ★ 测不出车轮和画面的对应关系：车挪回原处停下，不对准(不乱挪)')
                    go_back()
                    return None
            J = np.asarray(J, float)
            sk_J[0] = J
            if search:
                widen(c, r, p_c)
            # ---- ③ 边走边对准
            self.log(f'    车轮边走边对准{what}(最快 {vmax:g}mm/s，越近越慢)')
            t_end = t_dead if search else self.now() + float(cfg.get('raw_center_align_s', 25.0))
            lost_s = float(sk.get('lost_s', 4.0))
            mats_every = max(1, int(cfg.get('raw_center_mats_every', 5))) if search else 1
            good, t_meas = 0, self.now()
            t_log, frames, src = self.now(), 0, {'2+': 0, '1': 0, '推算': 0}
            e_start, t_start, reprobed = float(np.linalg.norm(tgt(c, r) - goalp)), self.now(), False
            em = e_start / sc
            k_fr, rim_ok, t_stuck = 0, False, None
            while self.now() < t_end:
                self._check_abort()
                pred = c + J @ (pos_est() - p_c)
                k_fr += 1
                if search:
                    mats = (not rim_ok) or k_fr % mats_every == 0
                    cc, rr, n, pc = frame(pred, [(float(pred[0]), float(pred[1]), rim_px())], mats)
                else:
                    cc, rr, n, pc = frame(pred)
                frames += 1
                cp = c + J @ (pc - p_c)                        # 拍这一帧时，圆心按车走了多少应该在这里
                rim_ok = cc is not None
                seen = False
                if cc is not None and float(np.linalg.norm(cc - cp)) < 30.0 * sc:
                    c = 0.3 * cp + 0.7 * cc                    # 认出来了：以它为主，和推算的平均一下(去掉一帧一帧的抖动)
                    r = 0.8 * r + 0.2 * rr
                    t_meas, seen = self.now(), True
                    src['2+'] += 1
                elif n == 1 and r > 0:
                    p1 = np.asarray(self._rc_pts[0], float)    # 只看到一个物料：圆心在它外面一个半径、靠近推算圆心的方向上
                    u = cp - p1
                    if float(np.linalg.norm(u)) > 1e-6:
                        c = 0.5 * cp + 0.5 * (p1 + u / float(np.linalg.norm(u)) * r)
                        t_meas, seen = self.now(), True
                        src['1'] += 1
                    else:
                        c = cp
                else:
                    c = cp                                     # 这一帧什么都没认出：按车走的距离推算
                    src['推算'] += 1
                p_c = pc
                last['c'], last['p'] = c, p_c
                e = tgt(c + J @ (pos_est() - p_c), r) - goalp  # 认这一帧的时候车又走了一点：按现在的位置算偏差
                em = float(np.linalg.norm(e)) / sc
                if self.now() - t_meas > lost_s and search:
                    send(0.0, 0.0)                             # 跟丢了：不干等，停下接着找
                    self.log(f'    ★ 连着 {lost_s:g} 秒认不出圆心(跟丢了)：停下重新找')
                    f2 = seek(False)
                    if f2 is None:
                        break
                    c, r, p_c = np.asarray(f2[0], float), float(f2[1]), np.asarray(f2[2], float)
                    widen(c, r, p_c)
                    t_meas, good, t_stuck = self.now(), 0, None
                    e_start, t_start = float(np.linalg.norm(tgt(c, r) - goalp)), self.now()
                    continue
                if self.now() - t_meas > 4.0:
                    send(0.0, 0.0)                             # 4 秒都只能推算：先停下，等认出来
                elif em <= tol:
                    good += 1 if seen else 0                   # 只有这一帧真认出来了才算(光靠推算不算对准了)
                    t_stuck = None
                    send(0.0, 0.0)
                    if good >= 3:
                        break
                else:
                    good = 0
                    d = -np.linalg.solve(J, e)                 # 还要走多少 mm [横移, 前进]
                    vv = clampv(d * 1.0, vmax)                 # 偏多少毫米就按每秒走多少(越近越慢)
                    nv = float(np.linalg.norm(vv))
                    if 1e-6 < nv < vmin:
                        vv = vv * (vmin / nv)                  # 太慢电机按 0 转/分算会不动：至少 vmin
                        nv = vmin
                    send(float(vv[0]), float(vv[1]), run_s=max(0.1, float(np.linalg.norm(d)) / max(nv, 1e-6)))
                    if search and st['clamped'] and float(np.linalg.norm(st['cmd'])) < 0.5 * vmin:
                        t_stuck = t_stuck or self.now()
                        if self.now() - t_stuck > 3.0:
                            lo0, hi0 = lo.copy(), hi.copy()
                            if getattr(self, '_rc_sure', False):
                                widen(c, r, p_c)               # 挪近了外圈认清楚了(物料确认了)：范围按它再放大
                            if np.any(lo != lo0) or np.any(hi != hi0):
                                t_stuck = None
                                continue
                            self.log(f'    ★ 车轮挪到能挪的范围边上了({self._raw_box_text(lo, hi, sk)})，还差 {em:.1f}mm：停在这里')
                            break
                    else:
                        t_stuck = None
                if not reprobed and self.now() - t_start > 3.0 and em * sc > e_start + 6.0 * sc:
                    reprobed = True                            # 越走越偏：对应关系方向不对，重新测
                    self.log(f'    ★ 越走越偏({e_start / sc:.0f}mm → {em:.0f}mm)：车轮和画面的对应关系不对，重新测')
                    send(0.0, 0.0)
                    J2, c, p_c = probe(c, p_c)
                    if J2 is None:
                        self._rc_last['why'] = '车轮和画面的对应关系不对、重新测也测不出'
                        go_back()
                        return None
                    J = np.asarray(J2, float)
                    sk_J[0] = J
                    e_start, t_start = float(np.linalg.norm(tgt(c, r) - goalp)), self.now()
                if self.now() - t_log >= 1.0:
                    dt = self.now() - t_log
                    self.log(f'      {frames / dt:.1f} 帧/秒(认出圆心 {src["2+"]}、一个物料修正 {src["1"]}、推算 {src["推算"]})  '
                             f'{what}还偏 {em:.1f}mm  车轮 横移 {st["cmd"][0]:+.0f}、前进 {st["cmd"][1]:+.0f}mm/s'
                             + (f'  {timing()}' if search else '') + rej_text())
                    t_log, frames, src = self.now(), 0, {'2+': 0, '1': 0, '推算': 0}
            finish()
            c = c + J @ (st['p'] - p_c)                        # 停下的地方
            em = float(np.linalg.norm(tgt(c, r) - goalp)) / sc
            ok = good >= 3
            self._rc_last = dict(ok=ok, err_mm=em, why='' if ok else f'到时间了还差 {em:.1f}mm(要 {tol:g}mm 以内)')
            self.log(f'    {what}离{where} {em:.1f}mm' + ('' if ok else '(没完全对准)'))
            return c, r
        except BaseException:
            try:
                finish()
            except Exception:
                pass
            raise
        finally:
            try:
                v.live_goal = None
            except Exception:
                pass

    def _raw_J_stored(self, fresh=False):
        """(10-20)存的原料区车轮和画面的对应关系(vcal / 示教时测的)：能用(大小、方向合理)就返回，比赛时直接用；
        没存、不合理、fresh=True(示教时要现测)返回 None。只测过前后、横移是补出来的也用(差得不多，越走越偏会重新测)。"""
        if fresh or self.store is None:
            return None
        J = self.store.get('RAW', 'ch')
        if J is None:
            return None
        J = np.asarray(J, float)
        if self._raw_J_bad(J, float(self.vision.scale('RAW'))):
            return None
        return J

    def _raw_near_search(self, D):
        """(10-20)最近那格比赛时找/对准原料盘中心的做法(_raw_center_align_smooth 的 search)；raw_near_seek 关掉返回 None(照旧)。"""
        cfg = self.cfg
        if not cfg.get('raw_near_seek', True):
            return None
        near_vec = None
        sl = (D.get('slots') or {}).get(str(D.get('near') or ''))
        if sl and sl.get('px') and D.get('c'):
            near_vec = [float(D['c'][0]) - float(sl['px'][0]), float(D['c'][1]) - float(sl['px'][1])]   # 最近那格 -> 圆心(画面里)
        return dict(toward=float(cfg.get('raw_near_toward_mm', 40.0)), away=float(cfg.get('raw_near_away_mm', 80.0)),
                    along=float(cfg.get('raw_near_along_mm', 120.0)), wait_s=float(cfg.get('raw_near_wait_s', 4.0)),
                    find_s=float(cfg.get('raw_near_find_s', 90.0)), lost_s=float(cfg.get('raw_near_lost_s', 4.0)),
                    step_mm=float(cfg.get('raw_near_step_mm', 40.0)),
                    max_mm=float(cfg.get('raw_near_max_mm', 150.0)),
                    orbit_px=float(D.get('r') or 0.0) or None, rim_px=float(D.get('rim') or 0.0) or None,
                    near_vec=near_vec)

    @staticmethod
    def _raw_box_text(lo, hi, sk):
        k = int(sk.get('toward_axis', 0))
        names = ('横移', '前后')
        o = 1 - k
        tv = (sk.get('toward_vec') or (0.0, 0.0))[k]
        if tv:
            tw, aw = (hi[k], -lo[k]) if tv > 0 else (-lo[k], hi[k])
            head = f'往原料盘那边 {tw:.0f}mm、往外 {aw:.0f}mm'
        else:
            head = f'{names[k]} {lo[k]:+.0f}~{hi[k]:+.0f}mm'
        return head + f'、{names[o]} {lo[o]:+.0f}~{hi[o]:+.0f}mm'

    def _stop_pose(self, role):
        """现在这个停车点(或者配置 stops 里 role 那个停车点)的位姿 (x, y, 车头°)；没有返回 None。"""
        stops = self.raw_cfg.get('stops') or {}
        al = self.cfg.get('stop_aliases')
        pose = stops.get(self._stop_name) if (self._stop_name and role_of(self._stop_name, al) == role) else None
        cur = pose is not None
        if pose is None:
            pose = next((v for k, v in stops.items() if role_of(k, al) == role), None)
        try:
            x, y, h = float(pose[0]), float(pose[1]), float(pose[2])
        except (TypeError, ValueError, IndexError):
            return None
        f, l = getattr(self, '_cur_shift', (0.0, 0.0)) if cur else (0.0, 0.0)
        if f or l:                                           # (10-20)这次规划时把停车点挪了：按真正停的位置算
            a = math.radians(h)
            x, y = x + f * math.cos(a) - l * math.sin(a), y + f * math.sin(a) + l * math.cos(a)
        return x, y, h

    def _raw_near_box(self, search, J, claw, goalp, rc=(0.0, 0.0)):
        """(10-20)最近那格找/对准原料盘中心时车轮能挪的范围(相对示教时车停的地方)：往原料盘那边 toward、往外 away、前后 along 毫米
        (用户：往原料盘那边再近会碰到原料盘)。哪边是原料盘：按存的车轮和画面的对应关系(车往原料盘走，圆心在画面里往爪子点那边挪)；
        没有就按地图(停车点 -> 原料转盘)；都没有两边都按 toward。再按地图查：车挪到范围边上会出场地、离黄色区/工位/地图上的障碍物太近，
        那边就少挪一点。返回 (lo, hi)：[横移, 前进] 的下限、上限；往原料盘的方向记在 search['toward_vec']、search['toward_axis']。"""
        tw, aw, al = float(search.get('toward', 40.0)), float(search.get('away', 80.0)), float(search.get('along', 120.0))
        sf, sl = (abs(float(v)) for v in getattr(self, '_cur_shift', (0.0, 0.0)))
        if sf or sl:                                         # (10-20)规划时停车点挪过：能挪的范围放宽这么多(下面再按地图收)
            tw, aw, al = tw + sl, aw + sl, al + sf
        u, how = None, ''
        if J is not None and float(np.linalg.norm(claw - goalp)) > 1.0:
            try:
                u, how = np.linalg.solve(np.asarray(J, float), claw - goalp), '按画面'
            except np.linalg.LinAlgError:
                u = None
        pose = self._stop_pose('RAW')
        if u is None and pose is not None:
            x, y, h = pose
            pc = (self.raw_cfg or {}).get('raw_center_mm') or [1200, 2480]
            dx, dy = float(pc[0]) - x, float(pc[1]) - y
            a = math.radians(h)
            u, how = np.array([-dx * math.sin(a) + dy * math.cos(a), dx * math.cos(a) + dy * math.sin(a)]), '按地图'   # [左(横移+), 前]
        lo, hi = np.array([-tw, -al]), np.array([tw, al])
        if u is not None and float(np.linalg.norm(u)) > 1e-9:
            k = int(np.argmax(np.abs(u)))
            sg = 1.0 if u[k] > 0 else -1.0
            lo[k], hi[k] = min(sg * tw, -sg * aw), max(sg * tw, -sg * aw)
            lo[1 - k], hi[1 - k] = -al, al
            tv = [0.0, 0.0]
            tv[k] = sg
            search['toward_vec'], search['toward_axis'] = tv, k
        else:
            search['toward_vec'], search['toward_axis'] = None, 0
            how = '不知道哪边是原料盘：两边都按往原料盘那边的限'
        why = []
        if pose is not None:
            obst = self._map_obstacles()
            for k in (0, 1):
                for side in (0, 1):
                    lim0 = hi[k] if side else lo[k]
                    lim, first = lim0, ''
                    while abs(lim) > 1e-6:
                        p = np.zeros(2)
                        p[k] = lim
                        bad = ''
                        for t in np.linspace(lo[1 - k], hi[1 - k], 5):
                            p[1 - k] = t
                            ok, why1 = self._body_clear(pose, np.asarray(rc, float)[:2] + p, obst)
                            if not ok:
                                bad = why1
                                break
                        if not bad:
                            break
                        first = first or bad
                        lim = (lim - 10.0) if lim > 0 else (lim + 10.0)
                        if lim * lim0 <= 0:
                            lim = 0.0
                    if abs(lim - lim0) > 1e-6:
                        why.append(f'{("横移", "前后")[k]}{"+" if side else "-"} 只能挪 {abs(lim):.0f}mm({first})')
                        if side:
                            hi[k] = lim
                        else:
                            lo[k] = lim
        self.log(f'    能挪的范围：{self._raw_box_text(lo, hi, search)}(以示教时停的地方为中心；{how}定的哪边是原料盘)'
                 + (('；按地图少挪：' + '；'.join(why)) if why else ''))
        return lo, hi

    def _map_obstacles(self):
        """地图上的障碍物 [(x, y, 半径)](导航的 ctx 有 obstacles() 才有；10-20：原料区找圆心时挪车也查，不管 exec_obstacles)。"""
        f = getattr(self.ctx, 'obstacles', None) if self.ctx is not None else None
        if f is None:
            return []
        try:
            return [(float(o[0]), float(o[1]), float(o[2])) for o in (f() or [])]
        except Exception as ex:
            self.log(f'    (读地图障碍物出错：{ex!r})')
            return []

    def _body_clear(self, pose, d, obstacles):
        """车从停车点 pose 挪 d = [横移(左+), 前进] 毫米以后，车身(含雷达)离场地边、黄色区/工位、障碍物够不够远(原料转盘不查：
        往原料盘那边能挪多少另外限着)。返回 (行不行, 原因)。"""
        cfg, rc = self.cfg, self.raw_cfg
        x0, y0, h = pose
        a = math.radians(h)
        fx, fy = math.cos(a), math.sin(a)
        lx, ly = -fy, fx
        x, y = x0 + float(d[1]) * fx + float(d[0]) * lx, y0 + float(d[1]) * fy + float(d[0]) * ly
        L = float(rc.get('car_length_mm', 290.0))
        W = float(rc.get('car_width_mm', 260.0))
        ov = float(rc.get('lidar_overhang_mm', 45.0))
        u0, u1, v0, v1 = -L / 2.0, L / 2.0, -W / 2.0, W / 2.0 + ov
        corners = [(x + u * fx + v * lx, y + u * fy + v * ly) for u in (u0, u1) for v in (v0, v1)]
        zm = float(cfg.get('qr_zone_margin_mm', 30.0))
        om = float(cfg.get('qr_obstacle_margin_mm', 60.0))
        try:
            import route_plan as rp
            field = float(getattr(rp, 'FIELD', 2400))
            rects = list(getattr(rp, 'YELLOW', [])) + [getattr(rp, 'TEMP_ZONE', (0, 910, 150, 1490)),
                                                        getattr(rp, 'ROUGH_ZONE', (910, 0, 1490, 150))]
        except Exception:
            field = 2400.0
            rects = [(550, 1400, 1000, 1850), (1400, 1400, 1850, 1850), (550, 550, 1000, 1000), (1400, 550, 1850, 1000),
                     (0, 910, 150, 1490), (910, 0, 1490, 150)]
        rects += [tuple(r) for r in (rc.get('extra_blocked_rects') or [])]
        if any(cx < zm or cx > field - zm or cy < zm or cy > field - zm for cx, cy in corners):
            return False, '会出场地'
        for r in rects:
            x0_, y0_, x1_, y1_ = (float(t) for t in r[:4])
            sep = False
            for ax in ((1.0, 0.0), (0.0, 1.0), (fx, fy), (lx, ly)):
                pa = [cx * ax[0] + cy * ax[1] for cx, cy in corners]
                pb = [px * ax[0] + py * ax[1] for px in (x0_, x1_) for py in (y0_, y1_)]
                if min(pa) >= max(pb) + zm or min(pb) >= max(pa) + zm:
                    sep = True
                    break
            if not sep:
                return False, '离黄色区/工位太近'
        for ox, oy, orr in obstacles or []:
            dx, dy = ox - x, oy - y
            uu, vv = dx * fx + dy * fy, dx * lx + dy * ly
            dist = math.hypot(max(u0 - uu, 0.0, uu - u1), max(v0 - vv, 0.0, vv - v1)) - orr
            if dist < om:
                return False, f'离障碍物({ox:.0f},{oy:.0f})太近'
        return True, ''

    def _raw_center_search(self, wait):
        """停车点认不出原料盘中心：车轮依次挪到 raw_center_search_mm 里的位置去找，认出来就停在那里返回 (圆心, 半径)；
        都认不出就挪回停车点、返回 None。"""
        spots = self.cfg.get('raw_center_search_mm') or []
        smax = float(self.cfg.get('raw_center_s_mm', 40.0))
        fmax = float(self.cfg.get('raw_center_f_mm', 60.0))
        for k, spot in enumerate(spots, 1):
            ts = max(-smax, min(smax, float(spot[0])))
            tf = max(-fmax, min(fmax, float(spot[1])))
            d = self.act.disp
            ds, df = ts - float(d['S']), tf - float(d['F'])
            if abs(ds) < 1.0 and abs(df) < 1.0:
                continue
            self.log(f'    认不出原料盘中心：车轮挪着找(第 {k} 处，横移 {ts:+.0f}mm、前进 {tf:+.0f}mm)')
            self.act.chassis_move(int(round(ds)), int(round(df)))
            m = self._raw_plate_center(wait)
            if m is not None:
                return m
        d = self.act.disp
        if abs(float(d['S'])) >= 1.0 or abs(float(d['F'])) >= 1.0:
            self.act.chassis_move(-int(round(float(d['S']))), -int(round(float(d['F']))))
        return None

    def _raw_home(self):
        """看原料盘的姿态(OBS RAW 以后)的 ID1、ID2：读不到就用参数 A1G、A2E。"""
        try:
            a = self.arm.read_angles()
            if a[0] is not None and a[1] is not None:
                return float(a[0]), float(a[1])
        except ArmAbort:
            raise
        except ArmError:
            pass
        P = self.arm.params() or {}
        return float(P['A1G']), float(P['A2E'])

    def _raw_center_setup(self):
        """对准原料盘中心、认出三个停靠位(存进 _raw_slots_rt，之后 _raw_slot_pick 按它抓)。做成了返回 True；
        认不出圆心/停靠位、手臂一个位都够不着返回 False(默认这次不抓；raw_center_fallback=follow 时改用跟随抓取)。"""
        cfg, v = self.cfg, self.vision
        self.log('  原料区：先让车轮慢慢挪，爪子对准原料盘中心；再认出三个物料停下的位置，以后每次停下直接转过去抓')
        try:
            self.arm.obs('RAW', open_claw=True)
            m = self._raw_center_align()
        except (ArmError, VisionError) as ex:
            if isinstance(ex, ArmAbort):
                raise
            self.log(f'    ★ 对准原料盘中心出错：{ex}')
            return False
        if m is None:
            return False
        c, r = m
        self._raw_center_disp = (float(self.act.disp['S']), float(self.act.disp['F']))
        self._anchor_mark('RAW')
        home = self._raw_home()
        J = np.asarray(self.store.get('RAW', 'arm'), float)
        Jinv = np.linalg.inv(J)
        claw = np.array(v.claw('RAW'), float)
        sc = float(v.scale('RAW'))
        # 原料盘停着的时候看一帧：物料在哪(停靠位)。看到几个算几个，没看到的按"三个位隔 120°"补上
        colors = self._raw_center_colors()
        still_px = float(cfg.get('raw_still_px', 6.0))
        prev, seen = None, None
        t_lim = self.now() + float(cfg.get('raw_center_wait_s', 12.0)) + 10.0
        while self.now() < t_lim:
            self._check_abort()
            errs = v.material_errors(colors)
            px = {k: claw + np.asarray(q, float) for k, q in errs.items() if q is not None}
            if prev is not None and px and all(k in prev and float(np.linalg.norm(px[k] - prev[k])) <= still_px for k in px):
                seen = px
                break
            prev = px
        if not seen:
            self.log('    ★ 等不到原料盘停稳(或者看不到物料)：认不出停靠位')
            return False
        angs = sorted(math.atan2(p[1] - c[1], p[0] - c[0]) for p in seen.values())
        base = angs[0]
        pos = []
        for k in range(3):
            a = base + k * 2.0 * math.pi / 3.0
            near = [x for x in angs if abs(math.remainder(x - a, 2.0 * math.pi)) < math.pi / 6.0]
            a = near[0] if near else a                          # 看到了就用看到的位置，没看到的按 120° 补
            pos.append(c + r * np.array([math.cos(a), math.sin(a)]))
        self._raw_grabpos = None
        if self._raw_grabpos_setup(c, r, pos, claw, sc):
            return True                                         # 车轮把爪子对到了一个抓取位：原地等要的颜色转过来
        lim = dict(id1=60.0, id2=700.0)
        lim.update(cfg.get('raw_center_reach_deg') or {})
        a1r = cfg.get('a1_range') or self._arm_limits()[0]
        slots = {}
        for k, p in enumerate(pos, 1):
            d = -Jinv @ (p - claw)                              # [ID2, ID1] 度
            a1, a2 = home[0] + float(d[1]), home[1] + float(d[0])
            ok = (abs(a1 - home[0]) <= float(lim['id1']) and abs(a2 - home[1]) <= float(lim['id2'])
                  and float(a1r[0]) <= a1 <= float(a1r[1]) and self._a2_limits()[0] <= a2 <= self._a2_limits()[1])
            slots[str(k)] = dict(px=[round(float(p[0]), 1), round(float(p[1]), 1)], a=[round(a1, 2), round(a2, 2)] if ok else None)
            self.log(f'    {k} 号位：离爪子点 {np.linalg.norm(p - claw) / sc:.0f}mm，'
                     + (f'手臂转到 ID1 {a1:.1f}°、ID2 {a2:.1f}°' if ok else f'手臂够不着(要 ID1 {a1:.1f}°、ID2 {a2:.1f}°)'))
        if not any(sv['a'] for sv in slots.values()):
            self.log('    ★ 三个停靠位手臂都够不着(看原料盘的姿态 A1G/A2E 或者停车点要调)')
            return False
        self._raw_slots_rt = dict(home=[home[0], home[1]], slots=slots, rt=True)
        self.__dict__.pop('_raw_perm', None)
        self._raw_force_cap = float(cfg.get('raw_center_force_mm', 12.0))
        return True

    # ------------------------------------------------------------------ 原料区：固定抓取位(10-16 用户，和开源方案一样)
    def _raw_grabpos_setup(self, c, r, pos, claw, sc):
        """三个停靠位里挑一个车轮挪得最少、不出范围的当抓取位，车轮连续慢慢把爪子对过去。做成了返回 True。"""
        cfg, act = self.cfg, self.act
        if not cfg.get('raw_grab_fixed', True) or not cfg.get('raw_center_smooth', True) \
                or not hasattr(act, 'vel') or getattr(act, '_no_vel', False):
            return False
        Jc = self.store.get('RAW', 'ch') if self.store is not None else None
        if Jc is None:
            return False
        Jc = np.asarray(Jc, float)
        smax, fmax = float(cfg.get('raw_center_s_mm', 40.0)), float(cfg.get('raw_center_f_mm', 60.0))
        d0 = (float(act.disp['S']), float(act.disp['F']))
        best = None
        for k, p in enumerate(pos, 1):
            d = -np.linalg.solve(Jc, np.asarray(p, float) - claw)      # 车轮要走多少 mm [横移, 前进]
            ns, nf = d0[0] + float(d[0]), d0[1] + float(d[1])
            over = max(0.0, abs(ns) - smax) + max(0.0, abs(nf) - fmax)
            key = (over, float(np.linalg.norm(d)))
            if best is None or key < best[0]:
                best = (key, k, np.asarray(p, float), d)
        key, k, p, d = best
        if key[0] > 1.0:
            self.log(f'    ★ 三个停靠位车轮都要挪出范围(横移 ±{smax:g}mm、前后 ±{fmax:g}mm)才对得上：不用固定抓取位')
            return False
        u = (p - np.asarray(c, float)) / max(1e-9, float(np.linalg.norm(p - np.asarray(c, float))))
        self.log(f'    抓取位定在 {k} 号位(车轮还要走 横移 {float(d[0]):+.0f}mm、前进 {float(d[1]):+.0f}mm)：车轮慢慢把爪子对过去')
        m = self._raw_center_align_smooth(start=(c, r), target_dir=u)
        if m is None or m == 'nofw':
            return False
        self._raw_center_disp = (float(act.disp['S']), float(act.disp['F']))
        self._raw_grabpos = dict(slot=k, r=float(m[1]))
        self._raw_force_cap = float(cfg.get('raw_center_force_mm', 12.0))
        return True

    def _raw_wait_still(self, colors, max_s, after_move):
        """车、手臂不动，等原料盘停稳(连着两帧看得到的物料都没动)：返回 (停稳时刻, 停下大约的时刻或 None, {颜色: 像素})；等不到返回 None。
        after_move：要先看到它转过(这一次停下已经看过了，等下一次)。"""
        v = self.vision
        claw = np.array(v.claw('RAW'), float)
        still_px = float(self.cfg.get('raw_still_px', 6.0))
        prev, t_move, t_stop = None, None, None
        t_lim = self.now() + max_s
        while self.now() < t_lim:
            self._check_abort()
            t = self.now()
            errs = v.material_errors(colors)
            px = {c: claw + np.asarray(q, float) for c, q in errs.items() if q is not None}
            if prev is not None and px:
                common = [c for c in px if c in prev[1]]
                moved = any(float(np.linalg.norm(px[c] - prev[1][c])) > still_px for c in common) or len(common) < len(px)
                if moved:
                    if t_move is None and t_stop is not None and t_stop != prev[0]:
                        pass
                    t_move, t_stop = t, None
                elif after_move and t_move is None:
                    pass
                else:
                    if t_stop is None:
                        t_stop = prev[0]
                    return t, (t_stop if t_move is not None else None), px
            prev = (t, px)
        return None

    def _raw_grabpos_loop(self, items):
        """爪子已经对准抓取位(车、手臂都不动)：原料盘每停一次看一眼，停在抓取位的是要的颜色就直接下爪(拍一帧修一下)，
        放进它自己的转盘槽，再回到看原料盘的姿态等下一次停。"""
        cfg, v = self.cfg, self.vision
        sc = float(v.scale('RAW'))
        claw = np.array(v.claw('RAW'), float)
        gp = self._raw_grabpos
        match = 0.5 * math.sqrt(3.0) * float(gp['r'])               # 半个停靠位间距以内才算停在抓取位上
        J = np.asarray(self.store.get('RAW', 'arm'), float)
        Jinv = np.linalg.inv(J)
        T = self._raw_close_delay('grab')
        todo = []
        for item in items:
            left = self.in_tray.get(item.slot)
            if left is not None and not same_item(left, item):
                self.log(f'    ★ 转盘 {item.slot} 号槽里还有没放出去的 {left.color_name}，{item.color_name} 不夹')
                self.stats.grab(False)
                continue
            todo.append(item)
        self.log(f'  原料区(固定抓取位 {gp["slot"]} 号位)：车和手臂不动，原料盘每停一次，停在抓取位的是要的颜色就直接夹：'
                 + '  '.join(f'{it.color_name}->{it.slot}号槽' for it in todo))
        watch = sorted(set(self._raw_center_colors()) | {it.color for it in todo})
        t_end = self.now() + float(cfg.get('raw_track_s', 30.0)) * max(1, len(todo)) + 10.0
        in_order = not self._grab_any_order()                  # (10-22)按任务码顺序：停在抓取位的要是下一个要夹的颜色才夹
        first = True
        home = self._raw_home()
        head, t_head = None, 0.0
        while todo and self.now() < t_end:
            self._check_abort()
            if self.disabled:
                break
            if in_order and head is not todo[0]:
                head, t_head = todo[0], self.now() + float(cfg.get('raw_track_s', 30.0))
            if in_order and self.now() > t_head:               # 按顺序：这一个等太久没转到抓取位，记没夹到，接着夹下一个(不跳着夹)
                self.log(f'    ★ {head.color_name} 等了 {float(cfg.get("raw_track_s", 30.0)):.0f} 秒没转到抓取位：没夹到，接着夹下一个')
                self.stats.grab(False)
                todo.pop(0)
                self._show_stats()
                continue
            ob = self._raw_wait_still(watch, 20.0, after_move=not first)
            first = False
            if ob is None:
                continue
            t_seen, t_stop, px = ob
            if not px:
                continue
            k = min(px, key=lambda c_: float(np.linalg.norm(px[c_] - claw)))
            dist = float(np.linalg.norm(px[k] - claw))
            name = COLOR_NAMES.get(int(k), str(k))
            if dist > match:
                self.log(f'  [抓] 原料盘停了，抓取位上没有物料(最近的 {name} 离爪子点 {dist / sc:.0f}mm)：等下一次停')
                continue
            it = (todo[0] if todo[0].color == k else None) if in_order else next((x for x in todo if x.color == k), None)
            if it is None:
                self.log(f'  [抓] 停在抓取位的是 {name}，这次不要' + (f'(下一个要 {todo[0].color_name})' if in_order else '') + '：等下一次停')
                continue
            if t_stop is None:
                self.log(f'  [抓] 停在抓取位的是 {name}，但没看到它什么时候停下的(下爪时可能转起来)：等下一次停')
                continue
            left = t_stop + self._raw_stop_len() - self.now()
            if left < T + 0.4:
                self.log(f'  [抓] 停在抓取位的是 {name}，这次停下只剩 {max(0.0, left):.1f} 秒，来不及：等下一次停')
                continue
            self.log(f'  [抓] 停在抓取位的是 {name}(离爪子点 {dist / sc:.1f}mm)：直接下爪，放 {it.slot} 号槽')
            cur = [home[0], home[1]]

            def go(a1, a2):
                self.arm.ap(a1, a2)
                cur[0], cur[1] = a1, a2

            def need(e):
                d = -Jinv @ np.asarray(e, float)
                return cur[0] + float(d[1]), cur[1] + float(d[0]), 0.0

            ok = False
            try:
                ok = self._raw_follow_grab((it.color, it.slot, it), 'grab', px[k] - claw, self.now() + left - T - 0.15,
                                           J, need, lambda a1, a2: (a1, a2), go, cur)
            except (ArmError, VisionError) as ex:
                if isinstance(ex, ArmAbort):
                    raise
                self._recover(f'抓取出错：{ex}')
            if ok:
                self.stats.grab(True)
                self.in_tray[it.slot] = it
                todo.remove(it)
                self._show_stats()
                if not todo:
                    # (10-27 用户"夹取最后一个物料完成之后机械臂不应该再转到原料区域那边去了，应该直接回到待机状态走")
                    self.log(f'    ({it.color_name} 是这一批最后一个：夹完直接收臂，不再转回原料盘上方)')
                    break
            try:
                self.arm.obs('RAW', open_claw=True)                    # 回到看原料盘的姿态(车没动，还对着抓取位)
            except ArmAbort:
                raise
            except ArmError as ex:
                self._recover(f'回到看原料盘的姿态出错：{ex}')
        for it in todo:
            self.log(f'    ★ {it.color_name} 没夹到')
            self.stats.grab(False)
        self._show_stats()
        self._stow_quiet()

    # ------------------------------------------------------------------ 原料区：固定停靠位(示教) + 预判
    def _raw_slots(self):
        """示教过的原料区停靠位：{'home': [ID1, ID2], 'slots': {'1': {'px': [u, v], 'a': [ID1, ID2] 或 None}, ...}}；
        至少要有 2 个停靠位的画面位置、1 个手臂够得着(有角度)才用，否则 None。"""
        rt = getattr(self, '_raw_slots_rt', None)
        if rt is not None:
            return rt                                          # 这次对准原料盘中心以后现认的停靠位(raw_center)
        if not self.cfg.get('raw_slots', True):
            return None
        if not hasattr(self, '_raw_slots_cache'):
            self._raw_slots_cache = (load_place_adj(self.cfg.get('ring_teach_file')) or {}).get('RAW')
        d = self._raw_slots_cache
        if not isinstance(d, dict):
            return None
        sl = {k: v for k, v in (d.get('slots') or {}).items() if isinstance(v, dict) and v.get('px')}
        if len(sl) < 2 or not any(v.get('a') for v in sl.values()) or not d.get('home'):
            return None
        return dict(home=[float(x) for x in d['home']], slots=sl)

    def raw_teach_slot(self, n, color, path):
        """示教原料区停靠位 n(1~3)：车停在原料区停车点、原料盘停着、颜色 color 的物料停在这个位上。
        手臂到 OBS RAW 记下物料在画面里的位置；再只动手臂对准它，对得上就记下角度(够得着)，对不上只记画面位置(够不着也要记，用来认颜色在哪个位)。"""
        v = self.vision
        self.arm.obs('RAW', open_claw=True)
        home = self.arm.read_angles()
        if home[0] is None or home[1] is None:
            raise ArmError('读不到手臂角度，没保存')
        e = v.material_error(int(color), n=3)
        if e is None:
            raise VisionError('画面里没看到这个颜色的物料：原料盘停着、物料在画面里再试')
        cu, cv_ = v.claw('RAW')
        px = [round(cu + float(e[0]), 1), round(cv_ + float(e[1]), 1)]
        a = None
        res = None
        J = self.store.get('RAW', 'arm') if self.store is not None else None
        far = False
        if J is not None:
            dd = -np.linalg.solve(np.asarray(J, float), np.asarray(e, float))      # [ID2, ID1] 度
            lim = dict(id1=25.0, id2=250.0)
            lim.update(self.cfg.get('raw_follow_limit_deg') or {})
            far = abs(float(dd[1])) > float(lim['id1']) or abs(float(dd[0])) > float(lim['id2'])
        if J is not None and not far:
            res = self.servo.run('RAW', lambda: v.material_error(int(color), n=2), v.scale('RAW'),
                                 float(self.cfg.get('raw_tol_mm') or self.cfg['tol_mm']['RAW']), allow_chassis=False,
                                 label=f'示教停靠位{n}', bounds=v.bounds('RAW'))
            self.log(f'   对准结果：{res}')
            if res.ok or res.err_mm <= float(self.cfg.get('raw_tol_mm', 3.0)) + 2.0:
                b = self.arm.read_angles()
                if b[0] is not None and b[1] is not None:
                    a = [round(float(b[0]), 2), round(float(b[1]), 2)]
        self.arm.ap(home[0], home[1])
        d = load_place_adj(path)
        r = d.setdefault('RAW', {})
        if r.get('home') and math.hypot(float(r['home'][0]) - home[0], (float(r['home'][1]) - home[1]) / 10.0) > 5.0:
            self.log('   ★ 这次 OBS RAW 的姿态和以前示教时不一样(改过 A1G/A2E？)：以前示教的停靠位清掉，重新示教')
            r.clear()
        r['home'] = [round(home[0], 2), round(home[1], 2)]
        r.setdefault('slots', {})[str(int(n))] = dict(px=px, a=a)
        save_json(path, d)
        self.__dict__.pop('_raw_slots_cache', None)
        self.log(f'   停靠位 {n}：画面位置 {px}' + (f'，手臂 ID1 {a[0]:.2f}°、ID2 {a[1]:.2f}°(够得着)' if a else '，手臂够不着(只记位置)'))
        return a is not None

    def _raw_slot_pick(self, cands, mode, t_end, label):
        """按示教的停靠位抓(见 raw_slots)。返回夹到的那一项，没有返回 None；示教数据用不了就改用跟随抓取。"""
        cfg, v = self.cfg, self.vision
        tag = f'[{label}]'
        S = self._raw_slots()
        J = np.asarray(self.store.get('RAW', 'arm'), float) if self.store is not None and self.store.get('RAW', 'arm') is not None else None
        if S is None or J is None:
            return self._raw_follow_pick(cands, mode, t_end, label)
        Jinv = np.linalg.inv(J)
        slots = S['slots']
        keys = sorted(slots)
        P = {k: np.asarray(slots[k]['px'], float) for k in keys}
        reach = {k: slots[k]['a'] for k in keys if slots[k].get('a')}
        sp = min(float(np.linalg.norm(P[a] - P[b])) for a in keys for b in keys if a < b)
        match = float(cfg.get('raw_slot_match', 0.4)) * sp
        sc = float(v.scale('RAW'))
        still_px = float(cfg.get('raw_still_px', 6.0))
        T = self._raw_close_delay(mode)
        colors = [c[0] for c in cands]
        by_color = {c[0]: c for c in cands}
        cu, cv_ = v.claw('RAW')
        claw = np.array([cu, cv_], float)
        home = S['home']
        cur = [home[0], home[1]]

        def go(a1, a2):
            self.arm.ap(a1, a2)
            cur[0], cur[1] = a1, a2

        def need(e):
            d = -Jinv @ np.asarray(e, float)
            return cur[0] + float(d[1]), cur[1] + float(d[0]), 0.0

        def clampa(a1, a2):
            return a1, a2

        def observe(watch, max_s, after_move=False):
            """手臂不动，等原料盘停稳(连着两帧看得到的物料都没动)：返回 (停稳时刻, 停下大约的时刻或 None, {颜色: 像素})；等不到返回 None。
            after_move：要先看到它转过(这一次停下已经看过了，等下一次)。"""
            prev, t_move, t_stop = None, None, None
            t_lim = min(t_end, self.now() + max_s)
            while self.now() < t_lim:
                self._check_abort()
                t = self.now()
                errs = v.material_errors(watch)
                px = {c: claw + np.asarray(q, float) for c, q in errs.items() if q is not None}
                if prev is not None and px:
                    common = [c for c in px if c in prev[1]]
                    moved = any(float(np.linalg.norm(px[c] - prev[1][c])) > still_px for c in common) or len(common) < len(px)
                    if moved:
                        t_move, t_stop = t, None
                    elif after_move and t_move is None:
                        pass
                    else:
                        if t_stop is None:
                            t_stop = prev[0]
                        return t, (t_stop if t_move is not None else None), px
                prev = (t, px)
            return None

        def occupancy(px):
            """{停靠位: 颜色}：先按整体平移对齐(车停得偏一点，示教的画面位置一起挪)，再每个物料归到最近的停靠位。"""
            pts = list(px.items())
            best = None
            for c0, p0 in pts:                         # 试着把每个物料当成停在每个停靠位上，算出整体平移，取对得最好的
                for k0 in keys:
                    off = p0 - P[k0]
                    if float(np.linalg.norm(off)) > match:
                        continue                               # 车每次停得差不多：整体平移不会超过这么多
                    occ, cost = {}, float(np.linalg.norm(off))
                    for c, p in pts:
                        k = min(keys, key=lambda k: float(np.linalg.norm(p - off - P[k])))
                        dd = float(np.linalg.norm(p - off - P[k]))
                        if dd > match or k in occ:
                            cost += match                      # 对不上的物料(被爪子挡住认偏了、认错颜色)不管它，只罚一点
                            continue
                        occ[k] = c
                        cost += dd
                    if best is None or cost < best[0]:
                        best = (cost, occ, off)
            if best is None or not best[1]:
                return None, None
            return best[1], best[2]

        perm = getattr(self, '_raw_perm', None)        # 停靠位 -> 下一次停下它上面的物料去哪个停靠位
        prev_occ = None
        self.log(f'    按停靠位抓：{"认出了" if S.get("rt") else "示教了"} {len(keys)} 个停靠位，手臂够得着 {"、".join(sorted(reach))} 号位'
                 + ('' if perm is None else '(转向已知)'))
        while self.now() < t_end:
            if abs(cur[0] - home[0]) > 0.05 or abs(cur[1] - home[1]) > 0.5:
                go(home[0], home[1])
            ob = observe(colors, 15.0, after_move=prev_occ is not None)
            if ob is None:
                continue
            t_seen, t_stop, px = ob
            occ, off = occupancy(px)
            if occ is None:
                if S.get('rt'):
                    self.log(f'  {tag} 这次看到的物料对不上停靠位：等下一次停')
                    continue
                self.log(f'  {tag} 看到的物料对不上示教的停靠位(车停偏太多？)：这次改用跟随抓取')
                return self._raw_follow_pick(cands, mode, t_end, label)
            names = {k: (by_color[c][2].color_short if by_color[c][2] is not None else c) for k, c in occ.items()}
            self.log(f'  {tag} 原料盘停着：' + '  '.join(f'{k}号位 {names[k]}' for k in sorted(occ)))
            if prev_occ is not None and perm is None:
                for cand_perm in ({keys[i]: keys[(i + 1) % len(keys)] for i in range(len(keys))},
                                  {keys[i]: keys[(i - 1) % len(keys)] for i in range(len(keys))}):
                    pairs = [(k, c) for k, c in prev_occ.items() if c in occ.values()]
                    if pairs and all(occ.get(cand_perm[k]) == c for k, c in pairs):
                        perm = cand_perm
                        self._raw_perm = perm
                        self.log(f'  {tag} 看出转向了：' + '、'.join(f'{a}号位->{b}号位' for a, b in sorted(perm.items())))
                        break
            prev_occ = occ
            # ---- 现在就能夹：要夹的颜色停在够得着的停靠位上
            stop_len = self._raw_stop_len()
            if t_stop is None:
                t_stop = self._raw_predict_stop_start(t_seen)  # 没看到它停下的那一刻：按量到的周期推算
            left = (t_stop + stop_len - self.now()) if t_stop is not None else None
            if t_stop is not None and t_stop == t_stop:
                self.raw_cycle['t_stop'] = t_stop
            now_ok = [(k, occ[k]) for k in sorted(reach) if k in occ and occ[k] in by_color]
            if now_ok and left is None and perm is None:
                self.log(f'  {tag} {now_ok[0][0]} 号位就是要夹的颜色，但不知道它停了多久(下爪时可能转起来)：等下一次停')
            if now_ok and left is not None and perm is not None:
                # 转向已知：手臂转过去来不及(伸缩舵机一次要转几百度)的就不现在追，下面推算下一次停到哪个位、提前过去等
                P_ = self.arm.params() or {}
                spd = max(10.0, float(P_.get('ASPD', 90.0)))
                def t_mv(k_):
                    a_ = reach[k_]
                    return max(abs(float(a_[0]) - cur[0]), abs(float(a_[1]) - cur[1])) / spd + 0.4
                now_ok = [(k_, c_) for k_, c_ in now_ok if left > T + t_mv(k_) + 0.5]
            if now_ok and left is not None and left > T + 1.0:
                k, c = now_ok[0]
                r = self._raw_slot_grab(by_color[c], mode, k, reach[k], off, J, need, clampa, go, cur, left, T, tag)
                if r:
                    return by_color[c]
                continue
            if perm is None:
                continue                                   # 还不知道往哪边转：等下一次停
            # ---- 推算：第几次停下，哪个要夹的颜色会停到够得着的停靠位
            plan, o = None, dict(occ)
            for j in range(1, len(keys) + 1):
                o = {perm[k]: c for k, c in o.items()}
                hit = [(k, o[k]) for k in sorted(reach) if k in o and o[k] in by_color]
                if hit:
                    plan = (j, hit[0][0], hit[0][1])
                    break
            if plan is None:
                if S.get('rt'):
                    self.log(f'  {tag} ★ 推算下来要夹的颜色都不会停到手臂够得着的停靠位：跳过')
                    return None
                self.log(f'  {tag} 推算下来要夹的颜色都不会停到手臂够得着的停靠位：改用跟随抓取')
                return self._raw_follow_pick(cands, mode, t_end, label)
            j, k, c = plan
            it = by_color[c][2]
            self.log(f'  {tag} 推算：再停 {j} 次，{it.color_name if it is not None else c} 会停到 {k} 号位：手臂先过去等')
            a1, a2 = reach[k]
            d = -Jinv @ np.asarray(off, float)             # 车停偏了：画面整体平移 off 按 J 换成角度补上
            go(float(a1) + float(d[1]), float(a2) + float(d[0]))
            cyc = stop_len / 0.9 + (sorted(self.raw_moves[-5:])[len(self.raw_moves[-5:]) // 2] if self.raw_moves else 3.0)
            waited = 0
            while waited < max(j, 1) + int(cfg.get('raw_slot_wait_stops', 4)) - 1 and self.now() < t_end:
                ob2 = observe([c], cyc * (j + 1) + 3.0, after_move=True)
                if ob2 is None:
                    break
                waited += 1
                t2, ts2, px2 = ob2
                e = px2.get(c)
                if e is None:
                    continue
                e = e - claw
                if float(np.linalg.norm(e)) / sc > float(cfg.get('raw_slot_accept_mm', 25.0)):
                    continue                               # 还没到这个位：等下一次停
                left2 = (ts2 + stop_len - self.now()) if ts2 is not None else stop_len - 1.0
                if left2 < T + 0.3:
                    continue
                self.log(f'  {tag} {k} 号位停着要夹的物料(离爪子点 {np.linalg.norm(e) / sc:.1f}mm)：下爪')
                r = self._raw_follow_grab(by_color[c], mode, e, self.now() + left2 - T - 0.15, J, need, clampa, go, cur)
                if r:
                    return by_color[c]
                break
            else:
                self.log(f'  {tag} 等了 {waited} 次停，要夹的物料没停到 {k} 号位：推算可能不对，回去重新看')
                self._raw_perm = perm = None
        self.log(f'  {tag} ★ 等了 {float(cfg.get("raw_track_s", 30.0)):.0f} 秒没夹到，跳过')
        return None

    def _raw_slot_grab(self, cand, mode, k, a, off, J, need, clampa, go, cur, left, T, tag):
        """要夹的物料现在就停在够得着的停靠位 k：手臂转到示教的角度(车停偏的整体平移按 J 补一下)，拍一帧修正后下爪。"""
        Jinv = np.linalg.inv(J)
        d = -Jinv @ np.asarray(off, float)                 # 整体平移 off(像素)要多转多少度 [ID2, ID1]
        a1, a2 = float(a[0]) + float(d[1]), float(a[1]) + float(d[0])
        t0 = self.now()
        go(a1, a2)
        q = self.vision.material_errors([cand[0]]).get(cand[0])
        if q is None:
            self.log(f'  {tag} 转到 {k} 号位上方看不到物料：不夹')
            return None
        self.log(f'  {tag} {k} 号位就是要夹的颜色：手臂直接转过去，离爪子点 {np.linalg.norm(q) / float(self.vision.scale("RAW")):.1f}mm')
        r = self._raw_follow_grab(cand, mode, np.asarray(q, float), self.now() + left - (self.now() - t0) - T - 0.15,
                                  J, need, clampa, go, cur)
        rt = getattr(self, '_raw_slots_rt', None)
        if r and rt is not None and str(k) in rt['slots']:
            # 现认的停靠位：记下这次修好以后实际的角度(扣掉整体平移)，下次这个位直接转到这里，不用再修
            rt['slots'][str(k)]['a'] = [round(cur[0] - float(d[1]), 2), round(cur[1] - float(d[0]), 2)]
        return r

    def _raw_follow_grab(self, cand, mode, e, deadline, J, need, clampa, go, cur):
        """跟随抓取：原料盘停了、手臂已经在物料上方。按 J 一下修到位(最多 raw_follow_fix 次，每次修完拍一帧看看)，
        到容差(raw_tol_mm)或到必须下爪的时刻就下爪(偏差不超过 raw_grab_max_mm；raw_force_grab 时不超过 raw_force_max_mm 也夹)。
        修的时候发现物料自己在动(原料盘又转了)就不夹。返回 True 夹了 / False 没夹 / None 看不到了。"""
        cfg, v = self.cfg, self.vision
        sc = float(v.scale('RAW'))
        color, slot = cand[0], cand[1]
        tol_px = float(cfg.get('raw_tol_mm') or cfg['tol_mm']['RAW']) * sc
        still_px = float(cfg.get('raw_still_px', 6.0))
        for _i in range(int(cfg.get('raw_follow_fix', 3))):
            if float(np.linalg.norm(e)) <= tol_px or self.now() > deadline:
                break
            a1, a2, _over = need(e)
            a1, a2 = clampa(a1, a2)
            before = np.asarray(e, float)
            c0 = (cur[0], cur[1])
            go(a1, a2)
            q = v.material_errors([color]).get(color)
            if q is None:
                self.log('    ★ 修的时候看不到物料了：不下爪')
                return None
            e = np.asarray(q, float)
            dA = np.array([cur[1] - c0[1], cur[0] - c0[0]])
            if True:
                if float(np.linalg.norm((e - before) - J @ dA)) > 3.0 * still_px + 0.15 * float(np.linalg.norm(J @ dA)):
                    self.log('    ★ 修的时候物料自己在动(原料盘又转了)：不下爪，接着跟')
                    return False
        err = float(np.linalg.norm(e)) / sc
        grab_max = float(cfg.get('raw_grab_max_mm', 5.0))
        fmax = float(cfg.get('raw_force_max_mm', 60.0))
        cap = getattr(self, '_raw_force_cap', None)
        if cap is not None:
            fmax = min(fmax, float(cap))                          # 按停靠位抓：偏太多不硬夹
        force = bool(cfg.get('raw_force_grab', True)) and err <= fmax
        if err > grab_max and not force:
            self.log(f'    ★ 偏差 {err:.1f}mm，超过 {grab_max:g}mm，下爪会砸到物料：不下爪，等下一次停')
            return False
        if self.now() > deadline + 0.25 and not force:
            self.log('    ★ 修得太久，原料盘马上要转了：不下爪，等下一次停')
            return False
        if mode == 'nogo':
            self.log(f'    nogo：对准了(偏差 {err:.1f}mm)，不夹')
            return True
        self.log(f'    下爪(偏差 {err:.1f}mm)')
        if mode == 'lift':
            P = self.arm.params() or {}
            self.arm.do(f'LIFT {float(P["ZGRAB"]):g}')
            self.arm.do('CLAW C')
            self.arm.do(f'LIFT {float(P["ZHI"]):g}')
            return True
        self.arm.grab_here(slot)
        return True

    # ------------------------------------------------------------------ 原料区：对准圆心 + 示教的三个停靠位动作(10-18 用户)
    def _raw_ct(self):
        """rteach RAW center / rteach RAW 1~3 示教的数据：{'home': [ID1, ID2], 'c': [u, v], 'r': 半径像素,
        'slots': {'1': {'px': [u, v], 'a': [ID1, ID2] 或 None, 'color': 示教时的颜色}, ...}}。
        要示教过圆心、三个停靠位都有画面位置、至少一个有角度才用；raw_ct 关掉或没示教完返回 None。"""
        if not self.cfg.get('raw_ct', True):
            return None
        d = (load_place_adj(self.cfg.get('ring_teach_file')) or {}).get('RAW_CT')
        if not isinstance(d, dict) or not d.get('c') or not d.get('home'):
            return None
        sl = {str(k): s for k, s in (d.get('slots') or {}).items() if isinstance(s, dict) and s.get('px')}
        if len(sl) == 2:                                       # 有一个位的物料被爪子挡住、没示教画面位置：按另外两个推出来
            k = next(x for x in ('1', '2', '3') if x not in sl)
            g = self._raw_ct_guess_px(d, k)
            if g is not None:
                sl[k] = dict((d.get('slots') or {}).get(k) or {}, px=[float(g[0]), float(g[1])], guessed=True)
        if len(sl) < 3 or not any(s.get('a') for s in sl.values()):
            return None
        return dict(home=[float(x) for x in d['home'][:2]], c=[float(x) for x in d['c'][:2]],
                    r=float(d.get('r') or 0.0), slots=sl, shift=[float(x) for x in (d.get('shift_mm') or [0.0, 0.0])[:2]],
                    near=(str(d['near']) if d.get('near') and str(d['near']) in sl else None),   # (10-18)"最近那格"的做法：只在这一格夹
                    rim=float(d.get('rim') or 0.0))                    # (10-20)外圈半径(像素)，没记过是 0

    @staticmethod
    def _raw_ct_guess_px(D, n):
        """(10-18)n 号停靠位的物料被爪子挡住、在看原料盘的姿态下看不到：三个停靠位在原料盘上隔 120°，
        按圆心和另外两个(看得到、示教过的)位推出它在画面里的位置(三个位相对圆心的向量加起来是 0)。推不出来返回 None。"""
        c = D.get('c') if isinstance(D, dict) else None
        sl = (D.get('slots') or {}) if isinstance(D, dict) else {}
        others = [np.asarray(s['px'], float)[:2] for k, s in sl.items()
                  if str(k) != str(int(n)) and isinstance(s, dict) and s.get('px') and not s.get('guessed')]
        if not c or len(others) != 2:
            return None
        return 3.0 * np.asarray(c, float)[:2] - others[0] - others[1]

    def _raw_ct_pose(self, D):
        """(10-18 实车)手臂摆到看原料盘的姿态(OBS RAW 到位误差约 1°；ID1 每度 7mm，差 1° 原料盘在画面里就挪 7mm)，
        再用 AP(到位误差 0.3°)转到示教时的角度。返回 (现在的 [ID1, ID2], 和示教时还差的那一点角度让画面挪了多少像素)。
        和示教时差太多(改过 A1G/A2E)就不 AP，原样返回(调用的地方判断)。"""
        self.arm.obs('RAW', open_claw=True)
        h0 = [float(x) for x in D['home'][:2]]
        a = self.arm.read_angles()
        if a[0] is None or a[1] is None:
            raise ArmError('读不到手臂角度')
        if math.hypot(a[0] - h0[0], (a[1] - h0[1]) / 10.0) <= 5.0:
            self._raw_ct_approach(h0[0], h0[1])                # 从同一边转过去(齿轮间隙)，AP 到位误差 0.3°
            got = getattr(self.arm, 'angle', None) or {}
            a = (got.get(1), got.get(2))                       # (10-26)AP 转完回报了两个角度：直接用，不再另读一遍(省两次串口来回)
            if a[0] is None or a[1] is None:
                a = self.arm.read_angles()
            if a[0] is None or a[1] is None:
                raise ArmError('读不到手臂角度')
        a = [float(a[0]), float(a[1])]
        comp = np.zeros(2)
        Ja = self.store.get('RAW', 'arm') if self.store is not None else None
        if Ja is not None:
            comp = np.asarray(Ja, float) @ np.array([a[1] - h0[1], a[0] - h0[0]])
        return a, comp

    def _arm_limits(self):
        """ID1、ID2 的限位 ((ID1 下, 上), (ID2 下, 上))：STM32 的参数 S1MIN/S1MAX、S2MIN/S2MAX(10-18 可以 set 在线改)；
        旧程序没有这几个参数就用 A1_MIN/A1_MAX、A2_MIN/A2_MAX。"""
        try:
            P = (self.arm.params() if self.arm is not None else None) or {}
        except ArmAbort:
            raise
        except ArmError:
            P = {}

        def pair(a, b, dflt):
            lo, hi = P.get(a), P.get(b)
            return (float(lo), float(hi)) if lo is not None and hi is not None and float(lo) < float(hi) else dflt
        return pair('S1MIN', 'S1MAX', (A1_MIN, A1_MAX)), pair('S2MIN', 'S2MAX', (A2_MIN, A2_MAX))

    def _a2_limits(self):
        return self._arm_limits()[1]

    def _raw_a1_range(self):
        """ID1 真正能转到的范围：限位往里收 raw_a1_margin_deg(补偿摩擦多给的角度被限位截掉，靠近限位转不到)。"""
        a1r = self.cfg.get('a1_range') or self._arm_limits()[0]
        mg = float(self.cfg.get('raw_a1_margin_deg', 5.0))
        return float(a1r[0]) + mg, float(a1r[1]) - mg

    def _raw_ct_approach(self, a1, a2):
        """(10-18 实车：ID1 有齿轮间隙，vcal 前后同一个角度、画面里原料盘差了 8mm)转到 (a1, a2)：
        ID1 总是从小往大转过去(先到 a1 - raw_backlash_deg)，每次停的地方都一样。"""
        bl = float(self.cfg.get('raw_backlash_deg', 2.0))
        if bl > 0.0:
            self.arm.ap(float(a1) - bl, float(a2))
        self.arm.ap(float(a1), float(a2))

    def _raw_ct_keepalive(self):
        """等原料盘时手臂太久没动(规则：机器人停止运行 15 秒、等转盘时 23 秒，本轮结束)：(10-21 用户"突然抽搐一下")升降往上
        keepalive_lift_mm 再回来，手臂、爪子不左右动；keepalive_lift_mm=0 照以前 ID1 轻轻摆一下再回来。"""
        h = self._ct_home
        if self._lift_bob('等原料盘'):
            self._ct_t_arm = self.now()
            return
        d = float(self.cfg.get('raw_keepalive_deg', 1.0))
        self.log(f'    手臂 {self.now() - self._ct_t_arm:.0f} 秒没动了：ID1 轻轻摆一下')
        self.arm.ap(h[0] - d, h[1])                           # 往小摆、再往大回来：还是从同一边转到看的姿态(齿轮间隙)
        self.arm.ap(h[0], h[1])
        self._ct_t_arm = self.now()

    def _lift_bob(self, why):
        """(10-21)车不动太久：升降往上 keepalive_lift_mm 再回到原来的高度(手臂不左右动)。做了返回 True；没设/不知道高度返回 False。"""
        mm = float(self.cfg.get('keepalive_lift_mm', 4.0) or 0.0)
        if mm <= 0 or self.arm is None:
            return False
        known, z = self.arm.lift_state()
        if not known:
            return False
        try:
            top = float((self.arm.params() or {}).get('LFMAX', 1e9))
        except (TypeError, ValueError):
            top = 1e9
        up = z + mm <= top - 0.5                              # 已经在最高点附近：往下 mm 再回来
        self.log(f'    {why}：太久没动，升降往{"上" if up else "下"} {mm:g}mm 再回来(手臂不左右动)')
        self.arm.lift(z + mm if up else z - mm)
        self.arm.lift(z)
        return True

    def _raw_ct_stop_len(self):
        """这一次停下大约能停多久(秒)：量到过完整的停就按量到的(九折)，没量到按 raw_ct_stop_s 的九折。"""
        if self.raw_stops:
            return self._raw_stop_len()
        return 0.9 * float(self.cfg.get('raw_ct_stop_s', 6.0))

    def _raw_ct_period(self):
        """原料盘转一圈(停一次 + 转一次)要多久(秒)：亲眼量到过停多久、转多久就按量到的(中值)，没量到按 raw_ct_stop_s + raw_ct_move_s。"""
        st, mv = sorted(self.raw_stops[-5:]), sorted(self.raw_moves[-5:])
        s = st[len(st) // 2] if st else float(self.cfg.get('raw_ct_stop_s', 6.0))
        m = mv[len(mv) // 2] if mv else float(self.cfg.get('raw_ct_move_s', 3.0))
        return s + m

    def _raw_ct_predict(self, t):
        """手臂刚回到看的姿态、原料盘正停着：按亲眼看到的几次停下的时刻和周期，推算这一次是什么时候停下的。
        看到过两次以上停下：按量到的周期推；推算出来这会儿应该在转、或者马上要转，返回 None。
        (10-26 用户"有时候下一个已经在摄像头下面了但是还是一直不抓取，错过了又要再等一轮")只看到过一次停下(多半就是刚才下爪的那一次)：
        以前推不了，一直等它转一下再停，错过这一次停。现在按周期(量到的，没量到按配置的)推下一次：只推离那一次不到 1.6 个周期的，
        而且按最早可能停下的时刻算(周期按 raw_ct_period_tol 打折：多算一点已经停了多久，剩的时间算少一点)。"""
        ts = getattr(self, '_ct_stops', None) or []
        if not ts:
            return None
        if len(ts) < 2:
            per = self._raw_ct_period()
            lo = (1.0 - float(self.cfg.get('raw_ct_period_tol', 0.1))) * per   # 下一次最早什么时候停下
            dt = t - ts[-1]
            if per < 1.0 or not lo <= dt < 1.6 * per:
                return None
            if dt - lo > self._raw_ct_stop_len() - 0.2:
                return None                                    # 按最早停下的算，这会儿快停完了(要转了)
            return ts[-1] + lo
        nominal = float(self.cfg.get('raw_ct_stop_s', 6.0)) + float(self.cfg.get('raw_ct_move_s', 3.0))
        pers = []
        for a, b in zip(ts, ts[1:]):
            n = max(1, int(round((b - a) / nominal)))
            pers.append((b - a) / n)
        per = float(np.median(pers))
        if per < 1.0:
            return None
        el = (t - ts[-1]) % per
        if el > self._raw_ct_stop_len() - 0.2:
            return None
        return t - el

    def _raw_ct_watch(self, watch, max_s, after_move, focus=None, centers=None, known_stop=None, known_why='在对准的时候',
                      known_color=None, prev_stop=None, say_unknown=False):
        """手臂在看原料盘的姿态不动，等原料盘停稳(连着两帧看得到的物料都没动)：返回 (停稳时刻, 停下的时刻或 None, {颜色: 像素})；
        max_s 秒内等不到返回 None。after_move = 要先看到它转起来(手臂刚动过、上一次停下已经用过：等下一次停，这样知道它是什么时候停下的)。
        focus = (画面位置, 半径像素)：只按这一块里的物料判断转没转、停没停(10-18 最近那格的做法：只管抓取位；
        画面边上的物料一会儿认得出一会儿认不出，会被当成原料盘转了一下又停下)。返回的还是看到的所有物料。
        centers = 列表：(10-20)同一帧顺便认原料盘外圈(Vision.plate_scan)，认出的圆心加进这个列表(看车有没有被带偏，不用另外停下来看)。
        known_stop = (10-21)对准原料盘中心的时候看着它停下的时刻(_rc_plate_stop)、刚才下爪的那一次停下：一看还停着就按它算，
        不用等它转一下再停(known_why = 日志里说是什么时候停下的，None 不说)；known_color：抓取位那一块里要还是这个颜色才算还是那一次
        (别的颜色 = 走开的时候原料盘已经转过、又停下了)；按量到的停多久这一次早该停完了也不算。
        prev_stop = (10-26)上一次停下的时刻(亲眼看到的，之后手臂、车都没动过)：看到它转起来就量出那一次停了多久，再看到停下量出转了多久
        (self.raw_stops / raw_moves，以后按量到的算)。self._ct_seen = 返回的停下时刻是不是亲眼看到的(不是推算/对准时记的)。
        say_unknown = 刚夹完回来/刚对准完：原料盘停着、推不出这一次停了多久时说一句为什么在等(接着等同一次停下时不说)。"""
        cfg, v = self.cfg, self.vision
        scan = centers is not None and hasattr(v, 'plate_scan')
        claw = np.array(v.claw('RAW'), float)
        still_px = float(cfg.get('raw_still_px', 6.0))
        keep_s = float(cfg.get('raw_keepalive_s', 12.0) or 0.0)
        prev, t_move, gap_t = None, None, None
        said = False
        self._ct_seen = False
        t_lim = self.now() + float(max_s)
        while self.now() < t_lim:
            self._check_abort()
            if keep_s > 0 and self.now() - self._ct_t_arm > keep_s:
                if t_move is None:
                    prev_stop = None                           # 动升降的时候它可能转起来了(没看到)：这一次停了多久量不准
                if t_move is not None and gap_t is None:
                    gap_t = self.now()                         # (10-21)原料盘在转的时候去动升降：这段时间里停下的，按动之前算(保守)
                try:
                    self._raw_ct_keepalive()
                except ArmAbort:
                    raise
                except ArmError as ex:
                    self.log(f'    (等原料盘时动一下升降出错：{ex}；接着等)')
                    self._ct_t_arm = self.now()
                prev = None                                    # 手臂动过：重新比画面
                continue
            t = self.now()
            if scan:
                plate, errs = v.plate_scan(watch)
                if plate is not None and not plate.get('coarse'):
                    ref = getattr(self, '_rc_ref_last', None)
                    r_now = float(plate.get('r_now', plate['r']))
                    if ref and ref[0] and abs(r_now - float(ref[0])) > float(cfg.get('raw_rim_tol', 0.15)) * float(ref[0]):
                        self._rc_rej_rim = getattr(self, '_rc_rej_rim', 0) + 1   # (10-23)外圈大小和示教时对不上：认错了，不拿它判断车有没有被带偏
                        self._rc_rej_rim_px = r_now
                    else:
                        centers.append(np.asarray(plate.get('c_now', plate['c']), float))
            else:
                errs = v.material_errors(watch)
            px = {c: claw + np.asarray(q, float) for c, q in errs.items() if q is not None}
            pf = px if focus is None else {c: p for c, p in px.items()
                                           if float(np.linalg.norm(p - np.asarray(focus[0], float))) <= float(focus[1])}
            if prev is not None and pf:
                common = [c for c in pf if c in prev[1]]
                moved = any(float(np.linalg.norm(pf[c] - prev[1][c])) > still_px for c in common) or len(common) < len(pf)
                if moved:
                    if t_move is None:
                        t_move = prev[0]
                        s_nom = float(cfg.get('raw_ct_stop_s', 6.0))
                        if prev_stop is not None and gap_t is None and 0.6 * s_nom <= t_move - float(prev_stop) <= 1.6 * s_nom:
                            self.raw_stops = self.raw_stops[-9:] + [t_move - float(prev_stop)]   # (10-26)量到这一次停了多久
                    gap_t = None                               # 动过升降以后又看着它在转：停下是在这以后，看得到
                elif t_move is not None:
                    t_stop = prev[0] - 0.07                    # 看着它停下：停下大概在第一帧停着的帧之前一点点
                    if gap_t is not None:
                        t_stop = min(t_stop, gap_t)            # 中间动过升降没看：可能早就停了，按动之前算(保守)；这个时刻不准，不记进周期
                    else:
                        self.raw_cycle['t_stop'] = t_stop
                        self._ct_stops = (getattr(self, '_ct_stops', None) or [])[-5:] + [t_stop]
                        self._ct_seen = True
                        m_nom = float(cfg.get('raw_ct_move_s', 3.0))
                        if 0.5 * m_nom <= t_stop - t_move <= 2.0 * m_nom:
                            self.raw_moves = self.raw_moves[-9:] + [t_stop - t_move]          # (10-26)量到转了多久
                    return t, t_stop, px
                elif not after_move:
                    return t, None, px
                elif known_stop is not None and (known_color is None or int(known_color) in pf) \
                        and t - float(known_stop) < self._raw_ct_stop_len() / 0.9 + 0.5:   # 对准的时候就看着它停下了：这一次停下马上能用
                    ts_ = getattr(self, '_ct_stops', None) or []
                    if not ts_ or abs(ts_[-1] - float(known_stop)) > 0.5:
                        self.raw_cycle['t_stop'] = float(known_stop)
                        self._ct_stops = ts_[-5:] + [float(known_stop)]
                    if known_why:
                        self.log(f'    原料盘{known_why}就停下了(已经停了 {t - float(known_stop):.1f} 秒)')
                    return t, float(known_stop), px
                else:
                    tp = self._raw_ct_predict(t)               # 一回来就停着：按周期推算它停了多久，够的话这一次就能用
                    used = getattr(self, '_ct_used', None)
                    if tp is not None and (used is None or abs(tp - used) > 1.0):     # 这一次停下已经看过、没用上：等下一次
                        self.log(f'    原料盘停着：按量到的周期推算已经停了 {t - tp:.1f} 秒')
                        return t, tp, px
                    if tp is None and say_unknown and not said:    # (10-26)以前这里不说话，看着像"物料在下面却不抓"
                        said = True
                        here = '、'.join(COLOR_NAMES.get(int(c_), str(c_)) for c_ in sorted(pf))
                        self.log(f'    原料盘停着(这一格上是 {here})，不知道这一次已经停了多久(推算这会儿快停完了、或者推不出来)：'
                                 '等它转一下再停，免得下爪时它转起来')
            prev = (t, pf)
        return None

    def _raw_plate_still(self, watch_px, color):
        """(10-21)最近那格下爪前：原料盘上别的物料是不是都还在等原料盘时看到的地方(手臂没动、画面没动 = 原料盘没转)。
        看得到别的物料：都在原地返回 True，有一个挪了返回 False；一个都看不到：再看一帧要夹的，这次在爪子附近就算 True。"""
        v = self.vision
        claw = np.array(v.claw('RAW'), float)
        tol = 2.0 * float(self.cfg.get('raw_still_px', 6.0))
        others = [int(c_) for c_ in (watch_px or {}) if int(c_) != int(color)]
        if others:
            errs = v.material_errors(others)
            seen = [(c_, claw + np.asarray(errs[c_], float)) for c_ in others if errs.get(c_) is not None]
            if seen:
                return all(float(np.linalg.norm(p_ - np.asarray(watch_px[c_], float))) <= tol for c_, p_ in seen)
        q = v.material_errors([color]).get(color)
        return q is not None and float(np.linalg.norm(np.asarray(q, float))) / float(v.scale('RAW')) \
            <= float(self.cfg.get('raw_slot_accept_mm', 25.0))

    def _raw_ct_grab(self, item, k, a, tag, t_end=None, near=False, watch_px=None, at=None, seen=None):
        """按 k 号停靠位示教的角度 a 转过去(在看原料盘的高度)，直接下爪夹进 item 自己的转盘槽。返回 True 夹了 / False 没夹。
        (10-21)near(最近那格)：下爪前车轮按物料实际停的位置补一下(_raw_grab_fix，t_end 之前要下爪)；夹完不挪回去
        (raw_grab_fix_back=False；等原料盘时按车轮挪了多少算停靠位在画面里的位置)。
        (10-24 用户"识别到了之后下达抓取命令能不能快速一点")at = 手臂现在的 [ID1, ID2]：已经在 a 上(最近那格就是看的姿态)不再发 AP；
        seen = 刚看着原料盘停下的那一帧里这个物料离爪子点多少像素：按它查，不再另拍一帧。"""
        cfg, v = self.cfg, self.vision
        if at is None or max(abs(float(at[0]) - float(a[0])), abs(float(at[1]) - float(a[1]))) > 0.5:
            self.arm.ap(float(a[0]), float(a[1]))
        self._ct_t_arm = self.now()
        if cfg.get('raw_ct_check', True):
            q = seen if seen is not None else v.material_errors([item.color]).get(item.color)
            if q is not None:
                d = float(np.linalg.norm(np.asarray(q, float))) / float(v.scale('RAW'))
                if d > float(cfg.get('raw_slot_accept_mm', 25.0)):
                    if watch_px is not None and self._raw_plate_still(watch_px, item.color):
                        # (10-21 实车 BLUE 一次认成离爪子点 122mm、一次 60mm，每次停下都不夹)：手臂没动，别的物料都还在原地 = 原料盘没转
                        self.log(f'  {tag} 下爪前一看 {item.color_name} 离爪子点 {d:.0f}mm，但原料盘上别的物料都还在原地(没转)：'
                                 '这一帧多半认错了(和蓝色爪子连在一起？)，照样下爪')
                        q = None
                    else:
                        self.log(f'  {tag} ★ 转过去一看 {item.color_name} 离爪子点 {d:.0f}mm(原料盘又转了？)：不下爪，等下一次停')
                        return False
            if q is not None:
                J = self.store.get('RAW', 'ch') if ((near or watch_px is not None) and self.store is not None) else None
                if J is not None and not cfg.get('raw_grab_fix', False):     # 车不动：只记下物料偏在哪边(看爪子点是不是整体偏了)
                    try:
                        dd = -np.linalg.solve(np.asarray(J, float), np.asarray(q, float))
                        self.log(f'  {tag} 物料离爪子点 {d:.1f}mm({self._car_dir_text(dd)})：车不动，直接下爪')
                    except np.linalg.LinAlgError:
                        pass
        moved = self._raw_grab_fix(item, tag, t_end) if (near and cfg.get('raw_grab_fix', False)) else None
        if isinstance(moved, str):
            return False                                       # 补位出了问题、挪回去以后这次停下来不及了：等下一次
        back = moved is not None and float(np.linalg.norm(moved)) >= 0.5 and cfg.get('raw_grab_fix_back', False)
        try:
            self.arm.grab_here(item.slot)
        finally:
            if back:
                self._raw_wheels_back(moved)
        self._ct_t_arm = self.now()
        return True

    @staticmethod
    def _car_dir_text(d):
        """车轮要挪 d = [横移(左正), 前进] 毫米才对上 -> "物料在爪子前 3.2、左 1.0mm"(看物料偏在哪边，查爪子点是不是整体偏了)。"""
        return (f'物料在爪子{"前" if d[1] >= 0 else "后"} {abs(float(d[1])):.1f}、'
                f'{"左" if d[0] >= 0 else "右"} {abs(float(d[0])):.1f}mm')

    def _raw_fix_go(self, s_t, f_t, t_move):
        """补位用：车轮只用 V 挪到 disp = (s_t, f_t)。STM32 不认 V(一点都没动)才改用 S/F(S/F 会纠车头)。"""
        act = self.act
        s0, f0 = float(act.disp['S']), float(act.disp['F'])
        if self._vel_goto(float(s_t), float(f_t), tol_mm=0.6, vmax=float(self.cfg.get('raw_grab_fix_vmax', 25.0)),
                          max_s=float(t_move) + 1.0):
            return
        if math.hypot(float(act.disp['S']) - s0, float(act.disp['F']) - f0) < 0.3:
            act.chassis_move(int(round(float(s_t) - s0)), int(round(float(f_t) - f0)))

    def _raw_grab_fix(self, item, tag, t_end):
        """(10-21 用户"对齐的不够快不够准，原料会夹取不上")最近那格：原料盘每次停的角度差一点，物料停的位置就差几毫米(爪子每边只有约 5mm 余量)。
        下爪前拍两帧看它离爪子点差多少，差 raw_grab_fix_mm 以上就车轮挪一下(手臂不动)，等 raw_grab_fix_settle_s 秒(停稳、画面跟上)再看；
        原料盘这次停下剩的时间不够就按现在的位置夹。
        (10-21 实车：挪了 10mm 再看还差 9mm、夹空了)挪完画面里的物料要跟着动(按车轮和画面的对应关系，至少 raw_grab_fix_trust)：
        动得太少说明认的物料位置不对(多半被爪子挡住、和蓝色爪子连在一起)，不再按它挪，车轮挪回去按原来对准的位置夹。
        离对准原料盘中心的地方一共最多挪 raw_grab_fix_max_mm。
        返回这次车轮一共挪了多少 [横移, 前进]；没挪返回 None；挪回去以后这次停下来不及下爪返回 'late'(这次不夹)。"""
        cfg, v, act = self.cfg, self.vision, self.act
        J = self.store.get('RAW', 'ch') if self.store is not None else None
        if J is None or act is None:
            if not getattr(self, '_grab_fix_said', False):
                self._grab_fix_said = True
                self.log('    (还没有原料区车轮和画面的对应关系：下爪前不按物料的位置补，按示教的位置直接夹)')
            return None
        J = np.asarray(J, float)
        sc = float(v.scale('RAW'))
        tol = float(cfg.get('raw_grab_fix_mm', 1.5))
        dmax = float(cfg.get('raw_grab_fix_max_mm', 25.0))
        t_move = float(cfg.get('raw_grab_fix_s', 1.5))
        settle = float(cfg.get('raw_grab_fix_settle_s', 0.3))
        trust = float(cfg.get('raw_grab_fix_trust', 0.5))
        s_a, f_a = float(act.disp['S']), float(act.disp['F'])
        base = np.asarray(getattr(self, '_raw_center_disp', None) or (s_a, f_a), float)   # 对准原料盘中心时车在哪

        def look():
            es = []
            for _f in range(2):
                q = v.material_errors([item.color]).get(item.color)
                if q is not None:
                    es.append(np.asarray(q, float))
            return np.median(np.asarray(es), axis=0) if es else None

        def total():
            return np.array([float(act.disp['S']) - s_a, float(act.disp['F']) - f_a])

        e = look()
        for _k in range(int(cfg.get('raw_grab_fix_moves', 3))):
            if e is None:
                self.log(f'  {tag} 下爪前看不到 {item.color_name}：按现在的位置夹')
                break
            em = float(np.linalg.norm(e)) / sc
            try:
                d = -np.linalg.solve(J, e)                     # 车轮要挪多少 [横移, 前进] 毫米
            except np.linalg.LinAlgError:
                break
            where = self._car_dir_text(d)
            if em <= tol:
                self.log(f'  {tag} 物料离爪子点 {em:.1f}mm({where})：直接夹')
                break
            cur = np.array([float(act.disp['S']), float(act.disp['F'])])
            far = float(np.linalg.norm(cur + d - base))
            if far > dmax:
                self.log(f'  {tag} 物料离爪子点 {em:.1f}mm({where})，要挪到离对准原料盘中心的地方 {far:.0f}mm，太多(认错了？)：'
                         '不挪，按现在的位置夹')
                break
            if t_end is not None and self.now() + t_move + settle > t_end:
                self.log(f'  {tag} 物料离爪子点 {em:.1f}mm({where})，原料盘这次停下剩的时间不够车轮再挪：按现在的位置夹')
                break
            self.log(f'  {tag} 物料离爪子点 {em:.1f}mm({where})：车轮挪 横移 {d[0]:+.1f}、前进 {d[1]:+.1f}mm 再夹(手臂不动)')
            self._raw_fix_go(cur[0] + float(d[0]), cur[1] + float(d[1]), t_move)
            dd = np.array([float(act.disp['S']), float(act.disp['F'])]) - cur
            self.sleep(settle)
            e2 = look()
            exp = J @ dd                                       # 车轮挪了 dd：画面里物料该跟着动这么多
            if e2 is not None and float(np.linalg.norm(exp)) >= 2.0 * sc:
                rho = float((e2 - e) @ exp) / float(exp @ exp)
                if rho < trust:
                    self.log(f'  {tag} ★ 车轮挪了 {float(np.linalg.norm(dd)):.1f}mm，画面里 {item.color_name} 只跟着动了'
                             f' {max(0.0, rho) * 100:.0f}%(再看还差 {float(np.linalg.norm(e2)) / sc:.1f}mm)：认的物料位置不对'
                             '(多半被爪子挡住、和蓝色爪子连在一起)，不按它挪：车轮挪回去，按原来对准的位置夹')
                    self._raw_fix_go(cur[0], cur[1], t_move)
                    if t_end is not None and self.now() > t_end:
                        self.log(f'  {tag} 挪回去以后这次停下来不及下爪了：等下一次停')
                        return 'late'
                    break
            e = e2
        t_ = total()
        return t_ if float(np.linalg.norm(t_)) > 0 else None

    def _raw_wheels_back(self, moved):
        """夹完：车轮挪回下爪前补之前的位置(还对着原料盘中心，下一个照样在这一格夹)。raw_grab_fix_back=True 才用。"""
        act = self.act
        try:
            s_t, f_t = float(act.disp['S']) - float(moved[0]), float(act.disp['F']) - float(moved[1])
            self._raw_fix_go(s_t, f_t, float(self.cfg.get('raw_grab_fix_s', 1.5)))
        except ArmAbort:
            raise
        except (ArmError, Exception) as ex:
            self.log(f'    (夹完车轮挪回去出错：{ex!r}；等原料盘时会按原料盘中心再对)')

    def _raw_ct_body(self, items):
        """(10-18 用户)原料区：①手臂摆到看原料盘的姿态(OBS RAW)就不动，车轮边走边看，把原料盘中心对到示教时的位置；
        ②对准以后车不再动，手臂和三个停靠位的相对位置跟示教时一样；③按任务码顺序：原料盘每停一次，看三个停靠位上是什么颜色，
        下一个要的颜色停在几号位，就按几号位示教的手臂角度直接转过去下爪(不拿画面再修手臂)，夹进它自己的转盘槽，回到看的姿态等下一个。
        (10-18 用户)"最近那格"的做法(rteach RAW near 示教的)：对准以后爪子就在离车最近那一格上方，只在这一格夹——
        要的颜色转到这一格就直接下爪，不是就等它转过来；每夹完一个看一眼车有没有被带偏，偏了车轮再慢慢对回去。
        做了返回 True(夹没夹到都算)；示教的姿态和现在对不上返回 False(这次照旧用以前的做法)。"""
        D = self._raw_ct()
        near_k = D.get('near')
        cfg, v = self.cfg, self.vision
        sc = float(v.scale('RAW'))
        todo = []
        for item in items:
            left = self.in_tray.get(item.slot)
            if left is not None and not same_item(left, item):
                self.log(f'    ★ 转盘 {item.slot} 号槽里还有没放出去的 {left.color_name}，{item.color_name} 不夹')
                self.stats.grab(False)
                self._show_stats()
                continue
            todo.append(item)
        if not todo:
            return True
        any_order = bool(near_k) and self._grab_any_order()     # (10-22)一律按任务码顺序；只有 mtest RAW n any 才不按顺序
        if any_order:
            self.log('  原料区(最近那格)：手臂摆到看原料盘的姿态不动，车轮把原料盘中心对到示教时的位置(爪子就在离车最近那一格上方)；'
                     '对准以后车不动，原料盘每停一次，这一格上是这一批要的哪个颜色就马上夹哪个(不按任务码顺序等，'
                     '放进它自己的槽)：' + '  '.join(f'{it.color_name}->{it.slot}号槽' for it in todo))
        elif near_k:
            self.log('  原料区(最近那格)：手臂摆到看原料盘的姿态不动，车轮把原料盘中心对到示教时的位置(爪子就在离车最近那一格上方)；'
                     '对准以后车不动，按任务码顺序，要的颜色转到这一格就直接下爪：'
                     + '  '.join(f'{it.color_name}->{it.slot}号槽' for it in todo))
        else:
            self.log('  原料区(对准圆心 + 示教的三个停靠位)：手臂摆到看原料盘的姿态不动，车轮把原料盘中心对到示教时的位置；'
                     '对准以后车不动，按任务码顺序，要的颜色停在几号位就按几号位示教的角度直接下爪：'
                     + '  '.join(f'{it.color_name}->{it.slot}号槽' for it in todo))

        def give_up(why):
            self.log(f'    ★ {why}')
            for it in todo:
                self.log(f'    ★ {it.color_name} 没夹到')
                self.stats.grab(False)
            self._show_stats()
            self._stow_quiet()
            return True
        try:
            home, comp = self._raw_ct_pose(D)                 # 精确转到示教时看原料盘的角度(还差的一点按画面补)
        except ArmAbort:
            raise
        except ArmError as ex:
            self._recover(f'摆到看原料盘的姿态出错：{ex}')
            return give_up('摆不到看原料盘的姿态：这次不抓')
        if math.hypot(home[0] - D['home'][0], (home[1] - D['home'][1]) / 10.0) > 5.0:
            d1, d2 = home[0] - D['home'][0], home[1] - D['home'][1]
            if near_k and cfg.get('raw_pose_follow', True) and abs(d1) <= float(cfg.get('raw_pose_a1_tol_deg', 3.0)):
                # (10-25 用户"每一个地方都要做完啊不然你搞个空车往回走也没用")重装齿条以后 arm_calib 重调了 A2E：同一个伸出长度读数变了。
                #   摄像头装在爪子上，车轮按画面把原料盘中心对到示教的位置，爪子照样在最近那一格上方(伸缩差的长度车轮补)；
                #   示教的抓取角度跟着挪同样多。ID1 差得多(画面跟着转了)才不做。下爪前照样看一眼物料在不在爪子下面(raw_ct_check)
                D = dict(D, home=[float(home[0]), float(home[1])],
                         slots={k: (dict(s, a=[float(s['a'][0]) + d1, float(s['a'][1]) + d2]) if s.get('a') else s)
                                for k, s in D['slots'].items()})
                comp = np.zeros(2)
                self.log(f'    ★ 看原料盘的姿态(ID1 {home[0]:.1f}°、ID2 {home[1]:.1f}°)和示教时不一样(ID1 差 {d1:+.1f}°、ID2 差 {d2:+.0f}°，'
                         'arm_calib 改过 A1G/A2E，比如重装了齿条)：照样做——车轮按画面把原料盘中心对到示教的位置(摄像头装在爪子上，'
                         '对准了爪子就在最近那一格上方)，夹的角度跟着挪同样多。有空重新 rteach RAW near 颜色号')
            elif near_k:                                       # (10-21 夜用户"后面为什么还是乱动")最近那格：不改用以前的做法(会开着车到处找圆心)
                return give_up(f'看原料盘的姿态(ID1 {home[0]:.1f}°、ID2 {home[1]:.1f}°)和示教时(ID1 {D["home"][0]:.1f}°、'
                               f'ID2 {D["home"][1]:.1f}°)不一样' + (f'(ID1 差 {d1:+.1f}°，画面跟着转了)' if abs(d1) > 0.05 else '') +
                               '：这次不抓，车不动。原料盘停着、最近那格上放一个物料，重新 rteach RAW near 颜色号')
            else:
                self.log('    ★ 看原料盘的姿态(ID1/ID2)和示教时不一样(改过 A1G/A2E？)：示教的停靠位用不了，这次照旧用以前的做法。'
                         '要重新示教：rteach RAW center，再 rteach RAW 1/2/3 颜色号')
                return False
        self._ct_home, self._ct_t_arm = home, self.now()
        self._ct_stops, self._ct_used = [], None
        self._rc_hist = []                                     # (10-21)对准时每帧记下物料绕圆心的角度：看原料盘什么时候停下的
        # ---- ① 车轮对准原料盘中心(手臂不动)
        try:
            ct_tol = float(cfg.get('raw_near_tol_mm', 2.0)) if near_k else float(cfg.get('raw_ct_tol_mm', 3.0))
            m = self._raw_center_align(goal=np.asarray(D['c'], float)[:2] + comp, tol=ct_tol,
                                       rng_c=D.get('shift'),       # rteach RAW shift 挪过车：车轮能挪的范围以挪过的地方为中心
                                       search=self._raw_near_search(D) if near_k else None)   # (10-20)最近那格：按偏差直接算着找
        except (ArmError, VisionError) as ex:
            if isinstance(ex, ArmAbort):
                raise
            m = None
            self.log(f'    ★ 对准原料盘中心出错：{ex}')
        nrej = int(getattr(self, '_rc_rej_rim', 0)) + int(getattr(self, '_rc_rej_orb', 0))
        if nrej >= 5:                                          # (10-23)认出的大小老是和示教时对不上：多半是示教过时了
            self.log(f'    ★ 这次有 {nrej} 帧认出的原料盘外圈或物料圆大小和示教时对不上(当认错扔掉了)。'
                     '手臂、摄像头、看原料盘的高度(ZOBRAW)、A1G/A2E 改过的话，要重新 rteach RAW near')
        if m is None:
            return give_up('认不出原料盘中心、车轮对不准：这次不抓(示教的角度是按对准以后的位置记的，没对准就下爪会砸到物料)')
        info = getattr(self, '_rc_last', None) or {}
        acc = float(cfg.get('raw_ct_accept_mm', 5.0))
        if not info.get('ok') and (info.get('err_mm') is None or float(info['err_mm']) > acc):
            em_ = info.get('err_mm')
            return give_up('车轮没对准' + (f'(还差 {float(em_):.1f}mm，超过 {acc:g}mm)' if em_ is not None else '')
                           + '：这次不抓(示教的角度是按对准以后的位置记的，没对准就下爪会砸到物料)')
        c = np.asarray(m[0], float)
        self._raw_center_disp = (float(self.act.disp['S']), float(self.act.disp['F']))
        self._anchor_mark('RAW')
        self._ct_t_arm = self.now()
        off = c - np.asarray(D['c'], float)                   # 对准以后还剩的一点偏差：认停靠位时示教的画面位置跟着挪
        keys = sorted(D['slots'])
        P = {k: np.asarray(D['slots'][k]['px'], float) + off for k in keys}
        reach = {k: [float(x) for x in D['slots'][k]['a'][:2]] for k in keys if D['slots'][k].get('a')}
        sp = min(float(np.linalg.norm(P[a] - P[b])) for a in keys for b in keys if a < b)
        match = float(cfg.get('raw_slot_match', 0.4)) * sp
        T = self._raw_close_delay('grab')
        batch_cols = sorted({int(it.color) for it in items})
        watch = sorted(set(self._raw_center_colors()) | {it.color for it in todo})
        if near_k:
            self.log('    对准了(还差 ' + f'{float(np.linalg.norm(off - comp)) / sc:.1f}mm)，车不再动：爪子在最近那一格上方，'
                     '要的颜色转过来就夹')
        else:
            self.log('    对准了(还差 ' + f'{float(np.linalg.norm(off - comp)) / sc:.1f}mm)，车不再动：'
                     + '  '.join(f'{k}号位' + (f' ID1 {reach[k][0]:.1f}°、ID2 {reach[k][1]:.1f}°' if k in reach else ' 手臂够不着') for k in keys))
        Pp = self.arm.params() or {}
        spd = max(10.0, float(Pp.get('ASPD', 90.0)))
        margin = float(cfg.get('raw_ct_margin_s', 0.3))

        def t_go(k):
            a = reach[k]
            return max(abs(a[0] - home[0]), abs(a[1] - home[1])) / spd + 0.4

        Jc = self.store.get('RAW', 'ch') if self.store is not None else None
        Jc = None if Jc is None else np.asarray(Jc, float)

        def fix_shift():
            """(10-21)下爪前补位挪过车、夹完不挪回去：原料盘(停靠位)在画面里跟着挪了多少像素。"""
            c0 = getattr(self, '_raw_center_disp', None)
            if Jc is None or c0 is None or self.act is None:
                return np.zeros(2)
            return Jc @ (np.array([float(self.act.disp['S']), float(self.act.disp['F'])]) - np.asarray(c0, float))

        def occupancy(px):
            sh = fix_shift()
            Pn = {k_: P[k_] + sh for k_ in keys}
            occ = {}
            for c_, p_ in sorted(px.items(), key=lambda kv: min(float(np.linalg.norm(kv[1] - Pn[k_])) for k_ in keys)):
                k = min(keys, key=lambda k_: float(np.linalg.norm(p_ - Pn[k_])))
                if float(np.linalg.norm(p_ - Pn[k])) <= match and k not in occ:
                    occ[k] = int(c_)
            return occ

        known = self._rc_plate_stop() if near_k else None     # (10-21)对准的时候看着原料盘停下的：一对准完、还停着就能夹
        if near_k:
            # (10-22)最近那格一律走这里(没夹上的认得出来再夹)：按任务码顺序只等下一个要的颜色；mtest RAW n any 时哪个先来夹哪个
            self._raw_near_any(D, todo, dict(P=P, keys=keys, reach=reach, sp=sp, match=match, T=T, margin=margin, spd=spd,
                                             home=home, comp=comp, watch=watch, known=known, in_order=not any_order,
                                             cyc=float(cfg.get('raw_ct_stop_s', 6.0)) + float(cfg.get('raw_ct_move_s', 3.0))))
            self._stow_quiet()
            return True

        seen = set()
        recheck = False                                        # 最近那格：夹完一个，等原料盘时顺便看车有没有被带偏
        after = True                                           # 车轮刚动过：等原料盘重新停一次(知道它是什么时候停下的)
        cyc = float(cfg.get('raw_ct_stop_s', 6.0)) + float(cfg.get('raw_ct_move_s', 3.0))
        last_stop = None
        quit_ = False
        for i, item in enumerate(todo, 1):
            self._check_abort()
            if self.disabled or quit_:
                if quit_:
                    self.log(f'    ★ {item.color_name} 没夹到')
                    self.stats.grab(False)
                    continue
                break
            if not cfg.get('full_course', True) and self.time_left() < 0:
                self.log('    ★ 时间到，不再抓取')
                break
            self._ui('stage', f'RAW GRAB {i}/{len(todo)}')
            tag = f'[抓 {item.color_name}]'
            t_end = self.now() + float(cfg.get('raw_track_s', 40.0))
            misses, got = 0, False
            while not got and self.now() < t_end:
                sh = fix_shift()
                focus = (P[near_k] + sh, 0.5 * sp) if near_k else None   # 最近那格：只按抓取位那一块判断原料盘转没转
                cen = [] if recheck else None
                ob = self._raw_ct_watch(watch, min(20.0, max(1.0, t_end - self.now())), after_move=after, focus=focus, centers=cen,
                                        known_stop=known)
                known, after = None, True
                if recheck and cen:
                    recheck = False
                    r_ = self._raw_near_redo(D, comp, np.median(np.asarray(cen[-3:]), axis=0), shift=sh)
                    if r_ is not None:                         # 车被带偏了、对回去了：停靠位跟着挪，重新等原料盘停
                        off, P = r_
                        continue
                if ob is None:
                    continue
                _t, t_stop, px = ob
                if t_stop is not None and last_stop is not None and abs(t_stop - last_stop) < 0.75 * cyc:
                    continue                                   # 还是刚才那一次停下(看过了)：不重复算
                if t_stop is not None:
                    last_stop = t_stop
                self._ct_used = t_stop                         # 这一次停下看过了(没用上也不再按推算回头用它)
                occ = occupancy(px)
                seen.update(occ.values())
                how = ''
                if near_k:
                    at = occ.get(near_k)                       # 最近那一格上停的是什么(别的格不管)
                    names = f'这一格上是 {COLOR_NAMES.get(int(at), at)}' if at is not None else '这一格上没认出物料'
                    k = near_k if at == int(item.color) else None
                else:
                    names = '  '.join(f'{k}号位 {COLOR_NAMES.get(int(c_), c_)}' for k, c_ in sorted(occ.items())) or '停靠位上没认出物料'
                    k = next((k_ for k_, c_ in occ.items() if c_ == int(item.color)), None)
                    if k is None:
                        free = [k_ for k_ in keys if k_ not in occ]
                        others = sorted(c_ for c_ in batch_cols if c_ != int(item.color))
                        if int(item.color) in seen and len(free) == 1 and sorted(occ.values()) == others:
                            k, how = free[0], '(没看清，另外两个位是另外两个颜色)'
                if k is None:
                    misses += 1
                    self.log(f'  {tag} 原料盘停了：{names}；' + (f'要的是 {item.color_name}，等它转过来' if near_k
                                                                   else f'没看到 {item.color_name}，等下一次停'))
                    if misses >= int(cfg.get('raw_ct_skip_stops', 4)):
                        self.log(f'    ★ 连着 {misses} 次停下都没看到 {item.color_name}' + ('转到这一格' if near_k else '')
                                 + '(原料盘上没有？)：跳过它')
                        break
                    continue
                misses = 0
                if k not in reach:
                    self.log(f'  {tag} 原料盘停了：{names}；{item.color_name} 在 {k} 号位，这个位没示教角度(够不着)：等下一次停')
                    continue
                if t_stop is None:
                    self.log(f'  {tag} 原料盘停了：{names}；{item.color_name} 在 {k} 号位，但不知道它停了多久：等下一次停')
                    continue
                left = t_stop + self._raw_ct_stop_len() - self.now()
                need = t_go(k) + T + margin
                fix_s = float(cfg.get('raw_grab_fix_s', 1.5)) if (near_k and cfg.get('raw_grab_fix', False)) else 0.0
                where = '这一格' if near_k else f'{k} 号位'
                if left < need:
                    self.log(f'  {tag} 原料盘停了：{names}；{item.color_name} 在{where}，这次停下只剩 {max(0.0, left):.1f} 秒'
                             f'(下爪要 {need:.1f} 秒)：等下一次停')
                    continue
                if near_k:
                    self.log(f'  {tag} 原料盘停了：{names}；直接下爪，放 {item.slot} 号槽')
                else:
                    self.log(f'  {tag} 原料盘停了：{names}{how}；{item.color_name} 在 {k} 号位：按示教的角度转过去下爪，放 {item.slot} 号槽')
                try:
                    got = self._raw_ct_grab(item, k, reach[k], tag, t_end=t_stop + self._raw_ct_stop_len() - T - margin,
                                            near=bool(near_k) and left >= need + fix_s, watch_px=px if near_k else None)
                except (ArmError, VisionError) as ex:
                    if isinstance(ex, ArmAbort):
                        raise
                    got = self._grab_failed(item, ex) if isinstance(ex, ArmError) else False
                    if not got and isinstance(ex, VisionError):
                        self._recover(f'抓 {item.color_name} 出错：{ex}')
                if got:
                    self.stats.grab(True)
                    self.in_tray[item.slot] = item
                    self._show_stats()
                    if i == len(todo):
                        self.log(f'    ({tag} 是这一批最后一个：夹完直接收臂，不再转回原料盘上方)')   # (10-26 用户)
                        break
                try:
                    if near_k:
                        home, comp = self._raw_ct_pose(D)      # 回到看原料盘的姿态(同一边转过去)，顺便算手臂角度还差的一点让画面挪了多少
                        self._ct_home = home
                    else:
                        self.arm.obs('RAW', open_claw=True)    # 回到看原料盘的姿态(车没动，还对着圆心)
                        self._raw_ct_approach(home[0], home[1])    # 和示教时一样从同一边转到看的姿态(齿轮间隙)
                except ArmAbort:
                    raise
                except ArmError as ex:
                    self._recover(f'回到看原料盘的姿态出错：{ex}')
                    self.log('    ★ 回不到看原料盘的姿态：这个原料区不再接着夹(收着的手臂直接转过去会刮到电池盒)')
                    quit_ = True
                    break
                self._ct_t_arm = self.now()
                if near_k and cfg.get('raw_near_recheck', True) and not self.disabled and (not got or i < len(todo)):
                    recheck = True                             # 车被带偏了就车轮慢慢对回去(还有要夹的才看；等原料盘时顺便看，不另外停)
            if not got:
                self.log(f'    ★ {item.color_name} 没夹到')
                self.stats.grab(False)
                self._show_stats()
        self._stow_quiet()
        return True

    def _raw_near_any(self, D, todo, ctx):
        """(10-21 用户"对准了之后就应该立刻识别颜色然后夹取啊为什么要等待那么久")最近那格、车已经对准：原料盘每停一次，
        这一格上是这一批还没夹的哪个颜色就马上夹哪个，放进它自己的转盘槽(不按任务码顺序等)。
        (10-22 用户"就是要全部按照这个任务码的顺序来放置和夹取")ctx['in_order']=True(默认的做法)：只等任务码里下一个要夹的颜色
        (还没夹的里面最靠前的；没夹上的放回去，它又是最靠前的，先把它再夹上)；这一格连着 raw_ct_skip_stops 次停下都不是它就记没夹到、等下一个。
        (10-21 实车"一会儿夹取一会儿不夹取")夹过的颜色又出现在原料盘上(刚才没夹上)：raw_near_regrab 次以内等它转到这一格再夹。
        每夹到一个，等下一个最多 raw_track_s 秒；连着 raw_ct_skip_stops 次停下这一格都没有要的颜色(原料盘上没有？)就不再等。"""
        cfg, v = self.cfg, self.vision
        near_k = str(D.get('near'))
        P, keys, reach = ctx['P'], ctx['keys'], ctx['reach']
        sp, match, T, margin, spd = ctx['sp'], ctx['match'], ctx['T'], ctx['margin'], ctx['spd']
        home, comp, watch, cyc, known = ctx['home'], ctx['comp'], ctx['watch'], ctx['cyc'], ctx['known']
        in_order = bool(ctx.get('in_order', False))
        Jc = self.store.get('RAW', 'ch') if self.store is not None else None
        Jc = None if Jc is None else np.asarray(Jc, float)

        def shift():
            c0 = getattr(self, '_raw_center_disp', None)
            if Jc is None or c0 is None or self.act is None:
                return np.zeros(2)
            return Jc @ (np.array([float(self.act.disp['S']), float(self.act.disp['F'])]) - np.asarray(c0, float))

        def t_go(k):
            a = reach[k]
            d = max(abs(a[0] - home[0]), abs(a[1] - home[1]))
            return 0.05 if d <= 0.5 else d / spd + 0.4         # (10-24)已经在抓取的角度上：不发 AP

        def occupancy(px, Pn):
            occ = {}
            for c_, p_ in sorted(px.items(), key=lambda kv: min(float(np.linalg.norm(kv[1] - Pn[k_])) for k_ in keys)):
                k = min(keys, key=lambda k_: float(np.linalg.norm(p_ - Pn[k_])))
                if float(np.linalg.norm(p_ - Pn[k])) <= match and k not in occ:
                    occ[k] = int(c_)
            return occ

        left = list(todo)                                      # 这一批还没夹的
        if near_k not in reach:
            self.log('    ★ 最近那一格没示教抓取的角度：这次不抓(重新 rteach RAW near 颜色号)')
            left = []
            for it_ in todo:
                self.log(f'    ★ {it_.color_name} 没夹到')
                self.stats.grab(False)
            self._show_stats()
        got_at = {}                                            # 夹过的颜色 -> 物料(又出现在原料盘上 = 刚才没夹上)
        regrab, wait_n = {}, {}                                # 再夹了几次；再夹的那个又等了几次停下还没转到这一格
        n_regrab = int(cfg.get('raw_near_regrab', 1))
        track = float(cfg.get('raw_track_s', 40.0))
        t_end = self.now() + track
        misses, recheck, after, last_stop = 0, False, True, None
        checked = not cfg.get('raw_near_check_last', False)
        reuse = None                                           # 刚下过爪(夹的颜色)：原料盘还停在这一次、这一格上还是它(没夹上)就马上再夹
        cont = None                                            # (10-26)亲眼看到的上一次停下、之后手臂和车都没动：接着等时量它停了多久、转了多久

        def missed(c_, where):
            """夹过的颜色 c_ 还在原料盘上(where)：刚才那次没夹上。还能再夹就放回 left(等它转到这一格)。"""
            nonlocal t_end
            it0 = got_at.pop(int(c_))
            self.stats.grab_ok = max(0, self.stats.grab_ok - 1)
            self.stats.grab_total = max(0, self.stats.grab_total - 1)   # (10-22)这一个重新算：再夹到/没夹到时再记一次(不重复记)
            if self.in_tray.get(it0.slot) is it0:
                self.in_tray.pop(it0.slot, None)               # 转盘这一格是空的(粗加工区不去取)
            self._show_stats()
            if regrab.get(int(c_), 0) < n_regrab:
                regrab[int(c_)] = regrab.get(int(c_), 0) + 1
                wait_n[int(c_)] = 0
                left.append(it0)
                t_end = max(t_end, self.now() + track)
                self.log(f'  [抓 {it0.color_name}] ★ 夹过的 {it0.color_name} 还在原料盘上({where})：刚才那次没夹上，'
                         '等它转到这一格再夹一次')
            else:
                self.log(f'  [抓 {it0.color_name}] ★ 夹过的 {it0.color_name} 还在原料盘上({where})：又没夹上，不再夹')

        while self.now() < t_end:
            if not left:
                if checked or not got_at:
                    break
                checked = True                                 # 都夹了：再看几秒原料盘，夹过的还在上面就是没夹上
                self.log('    这一批都夹过了：再看一眼原料盘(夹过的还在上面 = 刚才没夹上，再夹)')
                Pn_ = {k_: P[k_] + shift() for k_ in keys}
                cont = None
                c_ = self._raw_near_leftover(sorted(got_at), float(cfg.get('raw_near_check_s', 3.0)),
                                             at_slot=lambda p_: any(float(np.linalg.norm(p_ - Pn_[k_])) <= match for k_ in keys))
                if c_ is None:
                    self.log('    夹过的都不在原料盘上了')
                    break
                missed(c_, '再看的时候')
                continue
            self._check_abort()
            if self.disabled:
                break
            if not cfg.get('full_course', True) and self.time_left() < 0:
                self.log('    ★ 时间到，不再抓取')
                break
            self._ui('stage', f'RAW GRAB {len(todo) - len(left) + 1}/{len(todo)}')
            sh = shift()
            cen = [] if recheck else None
            ob = self._raw_ct_watch(watch, min(20.0, max(1.0, t_end - self.now())), after_move=after,
                                    focus=(P[near_k] + sh, 0.5 * sp), centers=cen, known_stop=known,
                                    known_why=None if reuse else '在对准的时候', known_color=reuse or None, prev_stop=cont,
                                    say_unknown=reuse is not None or last_stop is None)
            known, after = None, True
            same, reuse = reuse is not None, None
            cont = ob[1] if (ob is not None and getattr(self, '_ct_seen', False)) else None
            if recheck and cen:
                recheck = False
                r_ = self._raw_near_redo(D, comp, np.median(np.asarray(cen[-3:]), axis=0), shift=sh)
                if r_ is not None:                             # 车被带偏了、对回去了：停靠位跟着挪，重新等原料盘停
                    P = r_[1]
                    cont = None
                    continue
            if ob is None:
                continue
            _t, t_stop, px = ob
            same = same and t_stop is not None and last_stop is not None and abs(t_stop - last_stop) < 0.01
            if not same and t_stop is not None and last_stop is not None and abs(t_stop - last_stop) < 0.75 * cyc:
                continue                                       # 还是刚才那一次停下(看过了)：不重复算
            if t_stop is not None:
                last_stop = t_stop
            self._ct_used = t_stop
            Pn = {k_: P[k_] + sh for k_ in keys}
            occ = occupancy(px, Pn)
            # 夹过的颜色又出现在原料盘上：刚才那次没夹上(最近那一格上的要旁边没有别的物料才算，免得把爪子当成物料)
            for k_, c_ in sorted(occ.items()):
                if int(c_) not in got_at:
                    continue
                if k_ == near_k and any(int(c2) != int(c_) and float(np.linalg.norm(np.asarray(p2, float) - Pn[near_k])) <= match
                                        for c2, p2 in px.items()):
                    continue
                missed(c_, '这一格' if k_ == near_k else f'{k_} 号位')
            at = occ.get(near_k)                               # 最近那一格上停的是什么(别的格不管)
            for it_ in list(left):                             # 再夹的那个转一圈(3 次停下)都没转到这一格：刚才多半看错了
                c_ = int(it_.color)
                if c_ in wait_n and c_ != at:
                    wait_n[c_] += 1
                    if wait_n[c_] >= 3:
                        left.remove(it_)
                        del wait_n[c_]
                        self.in_tray[it_.slot] = it_                # 多半看错了：当它还在转盘里(和刚才夹的时候一样算)
                        self.stats.grab_ok += 1
                        self.stats.grab_total += 1
                        self._show_stats()
                        self.log(f'  [抓 {it_.color_name}] 等了 3 次停下都没转到这一格：刚才多半看错了，当它已经夹到了')
            if not left:
                break
            names = f'这一格上是 {COLOR_NAMES.get(int(at), at)}' if at is not None else '这一格上没认出物料'
            head = min(left, key=lambda it_: int(it_.index))   # 按任务码顺序下一个要夹的
            if in_order:
                item = head if (at is not None and int(head.color) == at) else None
            else:
                item = next((it_ for it_ in left if int(it_.color) == at), None) if at is not None else None
            if same and item is None:
                continue                                       # 刚下过爪的那一次停下：这一格上没有没夹上的，等下一次停
            if item is None:
                misses += 1
                want = head.color_name if in_order else '、'.join(it_.color_name for it_ in left)
                self.log(f'  [抓] 原料盘停了：{names}；' + (f'按任务码顺序下一个要 {want}' if in_order else f'还要 {want}') + '，等它转过来')
                if misses >= int(cfg.get('raw_ct_skip_stops', 4)):
                    if in_order and len(left) > 1:
                        left.remove(head)                      # 按顺序：这一个一直没转过来(原料盘上没有？)，记没夹到，接着等下一个
                        wait_n.pop(int(head.color), None)
                        self.log(f'    ★ 连着 {misses} 次停下这一格上都不是 {want}(原料盘上没有？)：{want} 没夹到，接着等下一个')
                        self.stats.grab(False)
                        self._show_stats()
                        misses = 0
                        continue
                    self.log(f'    ★ 连着 {misses} 次停下这一格上都没有要的颜色({want})(原料盘上没有？)：不再等')
                    break
                continue
            misses = 0
            tag = f'[抓 {item.color_name}]'
            if t_stop is None:
                self.log(f'  {tag} 原料盘停了：{names}，但不知道它停了多久：等下一次停')
                continue
            left_s = t_stop + self._raw_ct_stop_len() - self.now()
            clear_s = self._raw_clear_s()
            need = t_go(near_k) + T + margin + clear_s
            fix_s = float(cfg.get('raw_grab_fix_s', 1.5)) if cfg.get('raw_grab_fix', False) else 0.0
            if left_s < need:
                self.log(f'  {tag} 原料盘停了：{names}，这次停下只剩 {max(0.0, left_s):.1f} 秒(下爪、夹紧、抬离原料盘要 {need:.1f} 秒)：'
                         '等下一次停')
                continue
            self.log(f'  {tag} 原料盘停了：{names}；马上下爪，放 {item.slot} 号槽')
            q_seen = None                                      # (10-24)刚看着停下的那一帧：按它查物料在不在这一格，不再另拍一帧
            if px.get(int(item.color)) is not None and self.now() - float(_t) <= float(cfg.get('raw_seen_fresh_s', 0.5)):
                q_seen = np.asarray(px[int(item.color)], float) - np.asarray(v.claw('RAW'), float)
            cont = None                                        # 手臂要动了：下一次停了多久量不了
            t_g0 = self.now()
            try:
                got = self._raw_ct_grab(item, near_k, reach[near_k], tag, t_end=t_stop + self._raw_ct_stop_len() - T - margin - clear_s,
                                        near=left_s >= need + fix_s, watch_px=px, at=home, seen=q_seen)
            except (ArmError, VisionError) as ex:
                if isinstance(ex, ArmAbort):
                    raise
                got = self._grab_failed(item, ex) if isinstance(ex, ArmError) else False
                if not got and isinstance(ex, VisionError):
                    self._recover(f'抓 {item.color_name} 出错：{ex}')
            if got:
                self.stats.grab(True)
                self.in_tray[item.slot] = item
                self._show_stats()
                left.remove(item)
                wait_n.pop(int(item.color), None)
                got_at[int(item.color)] = item
                t_end = self.now() + track
            elif not self.disabled and regrab.get(int(item.color), 0) < n_regrab:
                # (10-23)这次没夹(下爪出错、串口偶尔一次没回话，或者下爪前一看不对)，物料还在原料盘上：给它再转一圈回来的时间。
                #   以前等的时间不重新算，要的那个转一圈回来之前就到时间了，这一批后面的全记"没夹到"
                regrab[int(item.color)] = regrab.get(int(item.color), 0) + 1
                t_end = max(t_end, self.now() + track)
            if got and not left and checked:
                # (10-26 用户"最后一个夹取完了把爪子缩回来就完事了啊还把爪子转过去干嘛呢")这一批夹完了、不再看原料盘：
                #   手臂不转回原料盘上方，从车上转盘这里直接收臂(_raw_ct_body 收)
                self.log(f'    ({tag} 是这一批最后一个：夹完直接收臂，不再转回原料盘上方，用了 {self.now() - t_g0:.1f} 秒)')
                break
            t_g1 = self.now()
            try:
                home, comp = self._raw_ct_pose(D)              # 回到看原料盘的姿态(同一边转过去)，顺便算手臂角度还差的一点让画面挪了多少
                self._ct_home = home
            except ArmAbort:
                raise
            except ArmError as ex:
                self._recover(f'回到看原料盘的姿态出错：{ex}')
                self.log('    ★ 回不到看原料盘的姿态：这个原料区不再接着夹(收着的手臂直接转过去会刮到电池盒)')
                break
            self._ct_t_arm = self.now()
            if left:                                           # (10-26)看时间花在哪：原料盘转一圈(停 + 转)多久，夹一个回来又要多久
                self.log(f'    ({tag} 下爪到放进转盘 {t_g1 - t_g0:.1f} 秒、回到看原料盘 {self.now() - t_g1:.1f} 秒；'
                         f'从它停下算 {self.now() - t_stop:.1f} 秒，原料盘转一圈约 {self._raw_ct_period():.1f} 秒)')
            known, reuse = t_stop, int(item.color)             # 原料盘还停在这一次：这一格上还是它(没夹上)就马上再夹
            if cfg.get('raw_near_recheck', True) and not self.disabled and left:
                recheck = True                                 # 车被带偏了就车轮慢慢对回去(还有要夹的才看；等原料盘时顺便看，不另外停)
        for it_ in left:
            self.log(f'    ★ {it_.color_name} 没夹到')
            self.stats.grab(False)
        if left:
            self._show_stats()

    def _raw_near_leftover(self, cols, max_s, at_slot=None):
        """(10-21)这一批都夹过了：看 max_s 秒原料盘(转着也行)，夹过的颜色还在上面(连着两帧看到)就返回那个颜色；没看到返回 None。
        at_slot(像素) = 这个位置在不在原料盘的停靠位上：只算停在停靠位上的(画面里别处的一块同色，比如蓝色爪子，不算)。"""
        n = {}
        claw = np.array(self.vision.claw('RAW'), float)
        t_lim = self.now() + float(max_s)
        while self.now() < t_lim:
            self._check_abort()
            errs = self.vision.material_errors(list(cols))
            for c_ in cols:
                q = errs.get(c_)
                hit = q is not None and (at_slot is None or at_slot(claw + np.asarray(q, float)))
                n[c_] = n.get(c_, 0) + 1 if hit else 0
                if n[c_] >= 2:
                    return int(c_)
        return None

    def _raw_ct_teach_cols(self):
        """示教时认原料盘中心看哪些颜色：配置里给了就用配置的；有任务码就用第一批的三个颜色；都没有就看红、黄、蓝、绿(RAW_COLORS，
        10-19 用户：省赛没有黑、浅蓝)。"""
        if self.cfg.get('raw_center_colors'):
            return None
        return sorted({int(it.color) for it in self.plan.items(1)}) if self.plan is not None else list(RAW_COLORS)

    def _raw_center_still(self, n=5, max_s=8.0):
        """(10-18)车停着：连着认 n 帧原料盘中心(和对准时同一个办法：外圈，认不出外圈才用物料)，取中值。返回 (圆心, 半径) 或 None。"""
        v = self.vision
        claw = np.array(v.claw('RAW'), float)
        colors = self._raw_center_colors()
        cs, rs, near = [], [], claw
        t_lim = self.now() + float(max_s)
        while len(cs) < int(n) and self.now() < t_lim:
            self._check_abort()
            c, r, _n = self._raw_center_frame(colors, claw, near)
            if c is not None:
                cs.append(np.asarray(c, float))
                rs.append(float(r))
                near = cs[-1]
        if len(cs) < max(2, int(n) // 2 + 1):
            return None
        return np.median(np.asarray(cs), axis=0), float(np.median(rs))

    def _raw_teach_seen_cols(self, cols):
        """示教时没有任务码、要认 4 种颜色(每帧慢)：先看几帧原料盘上有哪几种颜色，以后只认这几种。返回颜色列表。"""
        if not cols or len(cols) <= 3:
            return cols
        seen = set()
        for _ in range(3):
            self._check_abort()
            errs = self.vision.material_errors(cols)
            seen |= {int(k) for k, q in errs.items() if q is not None}
        if not seen:
            return cols
        self.log('  原料盘上认出的颜色：' + '、'.join(COLOR_NAMES.get(c, str(c)) for c in sorted(seen)) + '(之后只认这几种，每帧快一些)')
        return sorted(seen)

    def raw_ct_teach_center(self, path, move=True):
        """rteach RAW center：车停在原料区停车点(附近)，原料盘停着、上面放好物料。手臂摆到看原料盘的姿态(OBS RAW)；
        move=True：车轮把原料盘中心对到爪子点下面(和比赛时一样边走边看，只用 V 速度指令，车头不会转)，
        车轮和画面的对应关系每次都现测(测完检查合不合理)；没对准到 raw_ct_tol_mm 以内就不保存，说明原因。
        move=False(rteach RAW center here)：车不动，按现在的位置记。
        记下原料盘中心在画面里的位置(车停着认几帧取中值)和手臂姿态；以前示教的三个停靠位清掉(要重新示教)。返回 (圆心, 半径)。"""
        cfg, v = self.cfg, self.vision
        sc = float(v.scale('RAW'))
        tol = float(cfg.get('raw_ct_tol_mm', 3.0))
        old_cols = getattr(self, '_raw_center_cols', None)
        self._raw_center_cols = self._raw_ct_teach_cols() or old_cols
        m = None
        try:
            self.arm.obs('RAW', open_claw=True)
            P_ = self.arm.params() or {}
            if 'A1G' in P_ and 'A2E' in P_:
                # OBS 到位误差约 1°：用 AP(0.3°)、从同一边(齿轮间隙)精确转到 A1G、A2E，以后每次都这样转到这里
                self._raw_ct_approach(float(P_['A1G']), float(P_['A2E']))
            home = self.arm.read_angles()
            if home[0] is None or home[1] is None:
                raise ArmError('读不到手臂角度，没保存')
            if self._raw_center_cols:
                self._raw_center_cols = self._raw_teach_seen_cols(self._raw_center_cols)
            if move:
                if self.act is None:
                    raise ArmError('没有底盘(车轮)：车手动摆好以后用 rteach RAW center here')
                self.log('  车轮边走边看，把原料盘中心对到爪子点下面(手臂不动，只用 V 慢慢走，车头不转)。'
                         '另开终端 python3 vview.py 可以看实时画面……')
                m = self._raw_center_align(tol=tol, fresh=True)
                info = getattr(self, '_rc_last', None) or {}
                if m is None:
                    raise VisionError(f'没对准，没保存：{info.get("why") or "认不出原料盘中心"}。'
                                      '看实时画面(python3 vview.py)：黄圈要套住原料盘外圈')
                if not info.get('ok'):
                    raise VisionError(f'没对准，没保存：{info.get("why") or "没对到"}。'
                                      '车别动，再输入一次 rteach RAW center(会从现在的位置接着对)')
            m2 = self._raw_center_still(5)
            if m2 is None and not move:
                m2 = self._raw_plate_center(float(cfg.get('raw_center_wait_s', 12.0)))
            if m2 is not None:
                m = m2
        finally:
            self._raw_center_cols = old_cols
        if m is None:
            raise VisionError('认不出原料盘中心：原料盘上要有物料、外圈在画面里，没保存')
        c, rr = np.asarray(m[0], float), float(m[1])
        claw = np.array(v.claw('RAW'), float)
        dmm = float(np.linalg.norm(c - claw)) / sc
        if move and dmm > tol + 1.0:
            raise VisionError(f'停稳以后再看，原料盘中心离爪子点 {dmm:.1f}mm(要 {tol:g}mm 以内)，没保存：车别动，再输入一次 rteach RAW center')
        d = load_place_adj(path)
        d['RAW_CT'] = dict(home=[round(float(home[0]), 2), round(float(home[1]), 2)],
                           c=[round(float(c[0]), 1), round(float(c[1]), 1)], r=round(rr, 1), slots={})
        if getattr(self, '_rc_rim', None):
            d['RAW_CT']['rim'] = round(float(self._rc_rim), 1)          # (10-20)外圈半径(像素)
        save_json(path, d)
        if self.store is not None:
            self.store.save()                                  # 刚测的车轮和画面的对应关系存进 servo_cal.json：比赛时直接用
        self.log(f'  记下了：原料盘中心在画面 ({c[0]:.0f}, {c[1]:.0f})(离爪子点 {dmm:.1f}mm)，'
                 f'物料所在圆半径约 {rr / sc:.0f}mm。以前示教的停靠位清掉了。'
                 '车别动，原料盘停着，接着 rteach RAW 1 颜色号、rteach RAW 2 颜色号、rteach RAW 3 颜色号')
        return c, rr

    def _raw_near_material(self, color, n=3, max_s=8.0):
        """车、手臂停着：连着认 color 颜色的物料，有 n 帧位置差不多(原料盘停着)就返回这几帧的中值(物料 - 爪子点，像素)；认不出返回 None。
        (10-19 实车)用 plate_scan 认：这一帧连同认出的外圈、物料写给 vview.py，示教时能看到认的是哪个。"""
        v = self.vision
        still = float(self.cfg.get('raw_still_px', 6.0))
        qs = []
        t_lim = self.now() + float(max_s)
        while self.now() < t_lim:
            self._check_abort()
            errs = v.plate_scan([int(color)])[1] if hasattr(v, 'plate_scan') else v.material_errors([int(color)])
            q = errs.get(int(color))
            if q is None:
                continue
            q = np.asarray(q, float)[:2]
            qs = [x for x in qs if float(np.linalg.norm(x - q)) <= still] + [q]
            if len(qs) >= int(n):
                return np.median(np.asarray(qs), axis=0)
        return None

    def raw_ct_teach_near(self, color, path):
        """(10-18 用户)rteach RAW near 颜色号："最近那格"的做法。齿条不够长、手臂够不全三个停靠位：车就停在原料盘上离车最近的
        那一格前面，手臂只在看原料盘的姿态(OBS RAW；arm_calib 里"原料盘上方的姿态"那一步把爪子摆到这一格正上方)，
        原料盘转一次停一次，按任务码顺序，要的颜色转到这一格就直接下爪。
        示教：原料盘停着，这一格上是颜色 color 的物料。手臂摆到看的姿态不动，车轮慢慢挪(只用 V，车头不转)，按原料盘中心把这个物料
        对到爪子点正下方(每一轮看物料还差多少再对)；对准以后记下原料盘中心在画面里的位置(比赛时车轮把原料盘中心对到这里)、
        这一格和另外两格的画面位置、车轮从停车点挪了多少(比赛时车轮能挪的范围以那里为中心)。以前示教的停靠位清掉。返回 (圆心, 半径)。"""
        cfg, v, act = self.cfg, self.vision, self.act
        sc = float(v.scale('RAW'))
        tol = float(cfg.get('raw_near_tol_mm', 2.0))
        claw = np.array(v.claw('RAW'), float)
        color = int(color)
        name = COLOR_NAMES.get(color, str(color))
        if act is None:
            raise ArmError('没有底盘(车轮)：rteach RAW near 要用车轮对准')
        lim = (float(cfg.get('raw_near_teach_s_mm', 100.0)), float(cfg.get('raw_near_teach_f_mm', 100.0)))
        old_cols = getattr(self, '_raw_center_cols', None)
        cols = self._raw_ct_teach_cols() or old_cols
        self._raw_center_cols = None if cols is None else sorted({int(x) for x in cols} | {color})
        try:
            self.arm.obs('RAW', open_claw=True)
            P_ = self.arm.params() or {}
            if 'A1G' in P_ and 'A2E' in P_:
                self._raw_ct_approach(float(P_['A1G']), float(P_['A2E']))   # 和比赛时一样从同一边精确转到看的姿态(齿轮间隙)
            home = self.arm.read_angles()
            if home[0] is None or home[1] is None:
                raise ArmError('读不到手臂角度，没保存')
            if self._raw_center_cols:
                self._raw_center_cols = sorted(set(self._raw_teach_seen_cols(self._raw_center_cols)) | {color})
            q = self._raw_near_material(color)
            if q is None:
                look = sorted({int(x) for x in (self._raw_center_cols or RAW_COLORS)} | set(RAW_COLORS))
                errs = v.material_errors(look)
                got = [f'{COLOR_NAMES.get(int(k), k)}(离爪子点 {float(np.linalg.norm(np.asarray(e_, float))) / sc:.0f}mm)'
                       for k, e_ in sorted(errs.items()) if e_ is not None]
                seen_txt = ('画面里认出的是：' + '、'.join(got)) if got else '画面里红、黄、蓝、绿一个也没认出来'
                raise VisionError(f'画面里没看到 {name} 物料(颜色 {color})，{seen_txt}；没保存：原料盘要停着、{name} 物料放在离车最近的那一格，'
                                  '爪子要在它上方附近(arm_calib 里"原料盘上方的姿态"那一步摆)。看实时画面：python3 vview.py')
            m = self._raw_center_still(5)
            if m is None:
                raise VisionError('认不出原料盘中心，没保存：看实时画面(python3 vview.py)，黄圈要套住原料盘外圈')
            self.log(f'  {name} 物料离爪子点 {float(np.linalg.norm(q)) / sc:.1f}mm：车轮慢慢挪，把它对到爪子正下方'
                     '(手臂不动，只用 V，车头不转)。另开终端 python3 vview.py 可以看实时画面……')
            for k in range(max(1, int(cfg.get('raw_near_iters', 3)))):
                if k > 0 and float(np.linalg.norm(q)) / sc <= tol:
                    break
                goal = np.asarray(m[0], float) - q             # 原料盘中心要对到的地方：画面里整个往回挪 q，物料就到爪子点了
                r_ = self._raw_center_align(goal=goal, tol=tol, fresh=(k == 0), lim=lim)    # 第一轮现测车轮和画面的对应关系
                info = getattr(self, '_rc_last', None) or {}
                if r_ is None:
                    raise VisionError(f'没对准，没保存：{info.get("why") or "认不出原料盘中心"}。'
                                      '看实时画面(python3 vview.py)：黄圈要套住原料盘外圈')
                m = self._raw_center_still(5) or (np.asarray(r_[0], float), float(r_[1]))
                q = self._raw_near_material(color)
                if q is None:
                    raise VisionError(f'车轮挪过去以后看不到 {name} 物料了，没保存：原料盘别转、车别碰，再输入一次 rteach RAW near {color}')
                self.log(f'    第 {k + 1} 轮对完：{name} 物料离爪子点 {float(np.linalg.norm(q)) / sc:.1f}mm')
            dq = float(np.linalg.norm(q)) / sc
            if dq > tol + 1.0:
                raise VisionError(f'对了几轮，{name} 物料还离爪子点 {dq:.1f}mm(要 {tol:g}mm 以内)，没保存：'
                                  f'车别动，再输入一次 rteach RAW near {color}')
            c, rr = np.asarray(m[0], float), float(m[1])
            p = claw + q
            # 另外两格：这一格绕原料盘中心转 ±120°(画面是斜着看的，只是大概；看得到那两格上的物料就用看到的位置)
            u = p - c
            guess = []
            for ang in (2.0 * math.pi / 3.0, -2.0 * math.pi / 3.0):
                ca, sa = math.cos(ang), math.sin(ang)
                guess.append(c + np.array([ca * u[0] - sa * u[1], sa * u[0] + ca * u[1]]))
            other_cols = [int(x) for x in (self._raw_center_cols or self._raw_center_colors()) if int(x) != color]
            seen_px = [claw + np.asarray(e_, float)[:2] for e_ in (v.material_errors(other_cols).values() if other_cols else [])
                       if e_ is not None]
            gap = 0.5 * math.sqrt(3.0) * float(np.linalg.norm(u))
            others = []
            for g in guess:
                near_ = [x for x in seen_px if float(np.linalg.norm(x - g)) <= gap]
                best = min(near_, key=lambda x: float(np.linalg.norm(x - g))) if near_ else None
                others.append((g, True) if best is None else (best, False))
        finally:
            self._raw_center_cols = old_cols
        d = load_place_adj(path)
        slots = {'1': dict(px=[round(float(p[0]), 1), round(float(p[1]), 1)],
                           a=[round(float(home[0]), 2), round(float(home[1]), 2)], color=color)}
        for i, (g, gs) in enumerate(others, 2):
            slots[str(i)] = dict(px=[round(float(g[0]), 1), round(float(g[1]), 1)], a=None, guessed=bool(gs))
        d['RAW_CT'] = dict(home=[round(float(home[0]), 2), round(float(home[1]), 2)],
                           c=[round(float(c[0]), 1), round(float(c[1]), 1)], r=round(rr, 1), slots=slots, near='1',
                           shift_mm=[round(float(act.disp['S']), 1), round(float(act.disp['F']), 1)])
        if getattr(self, '_rc_rim', None):
            d['RAW_CT']['rim'] = round(float(self._rc_rim), 1)          # (10-20)外圈半径(像素)：比赛时车停偏了按它投票找圆心
        save_json(path, d)
        if self.store is not None:
            self.store.save()                                  # 刚测的车轮和画面的对应关系存进 servo_cal.json：比赛时直接用
        self.log(f'  记下了(最近那格的做法)：{name} 物料离爪子点 {dq:.1f}mm；原料盘中心在画面 ({c[0]:.0f}, {c[1]:.0f})，'
                 f'比赛时车轮把它对到这里，爪子就在这一格上方；车轮从停车点挪了 横移 {float(act.disp["S"]):+.0f}mm、'
                 f'前进 {float(act.disp["F"]):+.0f}mm(比赛时车轮以这里为中心挪)。手臂 ID1 {home[0]:.2f}°、ID2 {home[1]:.2f}°。'
                 '以前示教的停靠位清掉了。接着 mtest RAW 1 试一下整个原料区')
        return c, rr

    def _raw_near_recheck(self, D, comp, off, P):
        """(最近那格的做法)夹完一个回到看的姿态：认几帧原料盘中心(原料盘转着也认得出外圈)，离对准时的位置超过 raw_near_redo_mm
        (车被带偏了)就车轮再慢慢对回去。返回新的 (off, P)：对准以后还剩的偏差、跟着挪的停靠位画面位置。
        (10-20)比赛时不再用它(要停下来专门看几帧，夹完回来常常差一秒赶不上下一次停)：等原料盘时顺便看，_raw_near_redo。"""
        m = self._raw_center_still(3, max_s=4.0)
        if m is None:
            return off, P
        r = self._raw_near_redo(D, comp, m[0])
        return (off, P) if r is None else r

    def _raw_near_redo(self, D, comp, c_now, shift=None):
        """(最近那格)看到的原料盘中心 c_now 离对准时的位置超过 raw_near_redo_mm(车被带偏了)：车轮再慢慢对回去，
        返回新的 (off, P)(对准以后还剩的偏差、跟着挪的停靠位画面位置)；没偏、对不回去返回 None。
        shift = (10-21)下爪前补位故意挪过车(夹完不挪回去)，原料盘中心在画面里该跟着挪的像素：按"对准时的位置 + shift"算偏没偏。"""
        cfg, v = self.cfg, self.vision
        sc = float(v.scale('RAW'))
        goal = np.asarray(D['c'], float)[:2] + comp
        want = goal if shift is None else goal + np.asarray(shift, float)
        dc = float(np.linalg.norm(np.asarray(c_now, float) - want)) / sc
        if dc <= float(cfg.get('raw_near_redo_mm', 4.0)):
            return None
        self.log(f'    车被带偏了(原料盘中心离' + ('对准时的位置' if shift is None or float(np.linalg.norm(shift)) < 0.5 * sc
                                         else '补位挪过以后该在的位置') + f' {dc:.1f}mm)：车轮慢慢对回去')
        self._rc_hist = []
        try:
            r = self._raw_center_align(goal=goal, tol=float(cfg.get('raw_near_tol_mm', 2.0)), rng_c=D.get('shift'),
                                       search=self._raw_near_search(D))
        except (ArmError, VisionError) as ex:
            if isinstance(ex, ArmAbort):
                raise
            self.log(f'    ★ 对回去出错：{ex}')
            return None
        if r is None:
            return None
        if self.act is not None:
            self._raw_center_disp = (float(self.act.disp['S']), float(self.act.disp['F']))   # 补位从这里重新算
        off = np.asarray(r[0], float) - np.asarray(D['c'], float)
        return off, {str(k): np.asarray(s_['px'], float) + off for k, s_ in D['slots'].items()}

    def raw_ct_teach_slot(self, n, color, path):
        """rteach RAW n 颜色号：原料盘停着，颜色 color 的物料停在 n 号停靠位。
        ① 手臂摆到看原料盘的姿态、精确转到示教时的角度；车被碰过(原料盘中心离示教时的位置超过 raw_ct_tol_mm)就先让车轮对回去
           (每次都从头对，10-18 用户)；② 记下物料在画面里的位置；③ 只动手臂对准它，对上了记下角度(抓取动作)。
        手臂每个关节只在按画面估的角度附近转(一个关节到头了不拿另一个关节硬凑)。
        对不上(够不着/看不清)只记位置：用 arm AD 1 / arm AD 2 把爪子摆到物料正上方，再 rteach RAW n save。返回记没记下角度。"""
        cfg, v = self.cfg, self.vision
        sc = float(v.scale('RAW'))
        tol = float(cfg.get('raw_ct_tol_mm', 3.0))
        D = load_place_adj(path).get('RAW_CT')
        if not isinstance(D, dict) or not D.get('c'):
            raise ValueError('先做 rteach RAW center(车轮把原料盘中心对到爪子点下面)')
        if D.get('near'):
            raise ValueError('现在是"最近那格"的做法(rteach RAW near)：只在离车最近那一格夹，不用示教别的停靠位；'
                             '重新示教用 rteach RAW near 颜色号。要换回三个停靠位的做法：先 rteach RAW center')
        home, comp = self._raw_ct_pose(D)
        if math.hypot(home[0] - float(D['home'][0]), (home[1] - float(D['home'][1])) / 10.0) > 5.0:
            raise ArmError('看原料盘的姿态和 rteach RAW center 时不一样(改过 A1G/A2E？)：先重新 rteach RAW center')
        goal = np.asarray(D['c'], float)[:2] + comp
        old_cols = getattr(self, '_raw_center_cols', None)
        self._raw_center_cols = self._raw_ct_teach_cols() or old_cols
        try:
            m = self._raw_center_still(5)                      # 和 rteach RAW center 同一个办法认圆心(外圈)
            dc = None if m is None else float(np.linalg.norm(np.asarray(m[0], float) - goal)) / sc
            if dc is not None and dc > tol:
                if self.act is None:
                    raise ArmError(f'原料盘中心离示教时的位置 {dc:.0f}mm，又没有底盘：先重新 rteach RAW center')
                self.log(f'   原料盘中心离示教时的位置 {dc:.1f}mm(车被碰过？)：先让车轮对回去(只用 V，车头不转)')
                mm = self._raw_center_align(goal=goal, tol=tol)
                info = getattr(self, '_rc_last', None) or {}
                if mm is None or not info.get('ok'):
                    raise VisionError('车轮对不回示教时的位置，没示教：' + str(info.get('why') or '认不出原料盘中心'))
        finally:
            self._raw_center_cols = old_cols
        e = v.material_error(int(color), n=3)
        claw = np.array(v.claw('RAW'), float)
        guessed = False
        if e is None:
            g = self._raw_ct_guess_px(D, n)
            if g is None:
                raise VisionError('画面里没看到这个颜色的物料：原料盘停着、物料在画面里再试。'
                                  '物料被爪子挡住看不到的话：先示教另外两个位，这个位会按它们推出来')
            guessed = True
            g = np.asarray(g, float) + comp                    # 示教数据是按示教时的角度记的：换到现在的画面
            e = (float(g[0] - claw[0]), float(g[1] - claw[1]))
            self.log(f'   画面里看不到 {n} 号位的物料(可能被爪子挡住)：按圆心和另外两个位推出它在画面 ({g[0]:.0f}, {g[1]:.0f})，'
                     '手臂先按这个位置转过去，再看着对准')
        px = claw + np.asarray(e, float) - comp                # 存的画面位置按示教时看原料盘的角度算
        a = None
        J = self.store.get('RAW', 'arm') if self.store is not None else None
        if J is None:
            self.log('   还没有手臂和画面的对应关系(vcal RAW 颜色号 arm)：只记位置。用 arm AD 1 / arm AD 2 把爪子摆到物料正上方，'
                     f'再 rteach RAW {n} save')
        else:
            dd = -np.linalg.solve(np.asarray(J, float), np.asarray(e, float))       # [ID2, ID1] 度
            lim = dict(id1=60.0, id2=700.0)
            lim.update(cfg.get('raw_center_reach_deg') or {})
            a1r = self._raw_a1_range()
            a2lo, a2hi = self._a2_limits()
            a1, a2 = home[0] + float(dd[1]), home[1] + float(dd[0])
            # 10-18：离得远(八九十毫米)时按画面估的角度会差一二成：稍微超出范围也先在范围里试着对，看能对到多近
            over1 = max(0.0, abs(float(dd[1])) - float(lim['id1']), a1r[0] - a1, a1 - a1r[1])
            over2 = max(0.0, abs(float(dd[0])) - float(lim['id2']), a2lo - a2, a2 - a2hi)
            lim_txt = (f'ID1 {a1r[0]:.0f}~{a1r[1]:.0f}°(限位往里收 {float(cfg.get("raw_a1_margin_deg", 5.0)):g}°) 且离看的姿态 ±{float(lim["id1"]):.0f}°以内，'
                       f'ID2 {a2lo:.0f}~{a2hi:.0f}°(STM32 限位 S2MIN/S2MAX) 且离看的姿态 ±{float(lim["id2"]):.0f}°以内')
            if over1 > float(cfg.get('raw_teach_over_id1', 15.0)) or over2 > float(cfg.get('raw_teach_over_id2', 200.0)):
                self.log(f'   按画面算要转到 ID1 {a1:.1f}°、ID2 {a2:.1f}°(看的姿态 ID1 {home[0]:.1f}°、ID2 {home[1]:.1f}°)，'
                         f'超出范围太多(范围：{lim_txt})：手臂不动，只记位置。'
                         '三个位的位置都记下以后 rteach RAW shift 让车挪到够得着的地方')
            else:
                if over1 > 0.0 or over2 > 0.0:
                    self.log(f'   按画面算要转到 ID1 {a1:.1f}°、ID2 {a2:.1f}°(看的姿态 ID1 {home[0]:.1f}°、ID2 {home[1]:.1f}°)，'
                             f'稍微超出范围(范围：{lim_txt})：先在范围里对，看能对到多近')
                w1 = float(cfg.get('raw_teach_slack_id1', 8.0))   # 每个关节只在估的角度附近转：一个到头了不拿另一个硬凑(10-18 实车 ID2 被推到 -300)
                w2 = float(cfg.get('raw_teach_slack_id2', 150.0))
                d1, d2 = float(dd[1]), float(dd[0])
                rng = dict(id1=(max(a1r[0] - home[0], -float(lim['id1']), min(0.0, d1) - w1),
                                min(a1r[1] - home[0], float(lim['id1']), max(0.0, d1) + w1)),
                           id2=(max(a2lo - home[1], -float(lim['id2']), min(0.0, d2) - w2),
                                min(a2hi - home[1], float(lim['id2']), max(0.0, d2) + w2)))
                if guessed:                                    # 看不到：手臂先一下转到推出来的位置(物料就到爪子点附近、看得到了)，再从那里看着对
                    g1 = min(max(a1, home[0] + rng['id1'][0]), home[0] + rng['id1'][1])
                    g2 = min(max(a2, home[1] + rng['id2'][0]), home[1] + rng['id2'][1])
                    self.arm.ap(g1, g2)
                    m1, m2 = g1 - home[0], g2 - home[1]
                    rng = dict(id1=(rng['id1'][0] - m1, rng['id1'][1] - m1), id2=(rng['id2'][0] - m2, rng['id2'][1] - m2))
                res = self.servo.run('RAW', lambda: v.material_error(int(color), n=2), v.scale('RAW'),
                                     float(cfg.get('raw_tol_mm') or cfg['tol_mm']['RAW']), allow_chassis=False, wheels_only=False,
                                     label=f'示教停靠位{n}', bounds=v.bounds('RAW'), arm_range=rng, max_iter=12)
                self.log(f'   对准结果：{res}')
                if res.ok or res.err_mm <= float(cfg.get('raw_tol_mm', 3.0)) + 2.0:
                    b = self.arm.read_angles()
                    if b[0] is not None and b[1] is not None:
                        a = [float(b[0]), float(b[1])]
                        if cfg.get('raw_teach_verify', True):
                            a = self._raw_ct_verify(int(color), a, home)
                        a = [round(float(a[0]), 2), round(float(a[1]), 2)]
                else:
                    try:
                        b = self.arm.read_angles()
                    except ArmAbort:
                        raise
                    except ArmError:
                        b = (None, None)
                    self.log('   没对上(' + (f'最近还差 {res.err_mm:.1f}mm' if math.isfinite(res.err_mm) else '转过去以后还是看不到物料')
                             + (f'，这时 ID1 {b[0]:.1f}°、ID2 {b[1]:.1f}°' if b[0] is not None and b[1] is not None else '')
                             + ')：只记位置。三个位的位置都记下以后 rteach RAW shift 让车挪到够得着的地方')
        self._raw_ct_approach(home[0], home[1])                # 回到看的姿态(从同一边转过去)
        d = load_place_adj(path)
        r_ = d.setdefault('RAW_CT', D)
        r_.setdefault('slots', {})[str(int(n))] = dict(px=[round(float(px[0]), 1), round(float(px[1]), 1)], a=a, color=int(color),
                                                       **({'guessed': True} if guessed else {}))
        save_json(path, d)
        done = sorted(k for k, s in r_['slots'].items() if s.get('px'))
        self.log(f'   {n} 号停靠位：画面位置 ({px[0]:.0f}, {px[1]:.0f})' + (f'，抓取角度 ID1 {a[0]:.2f}°、ID2 {a[1]:.2f}°' if a else '，还没有抓取角度')
                 + f'。已示教 {"、".join(done)} 号位' + ('，三个都有了' if len(done) >= 3 else ''))
        return a is not None

    def _raw_ct_verify(self, color, a, home):
        """(10-18)示教对上以后，按比赛时的走法再试：回到看的姿态(从同一边转过去)，再一下转到记下的角度，看物料是不是在爪子点下面。
        一点一点对和一下转过去停的地方会不一样(齿轮间隙、舵机过冲)：差了就按差的修角度，最多试 raw_teach_verify_n 次。返回修过的角度。"""
        cfg, v = self.cfg, self.vision
        sc = float(v.scale('RAW'))
        tolmm = float(cfg.get('raw_tol_mm') or cfg['tol_mm']['RAW'])
        Ja = self.store.get('RAW', 'arm') if self.store is not None else None
        (lo1, hi1), (lo2, hi2) = self._raw_a1_range(), self._a2_limits()
        a = [float(a[0]), float(a[1])]
        for k in range(int(cfg.get('raw_teach_verify_n', 3))):
            self._raw_ct_approach(home[0], home[1])
            self.arm.ap(a[0], a[1])
            e = v.material_error(int(color), n=2)
            if e is None:
                self.log('   按比赛的走法(从看的姿态一下转过去)试了一次：看不到物料，按对准时的角度记')
                return a
            em = float(np.linalg.norm(e)) / sc
            if em <= tolmm:
                self.log(f'   按比赛的走法(从看的姿态一下转过去)试了一次：离爪子点 {em:.1f}mm，可以' + ('(角度修过)' if k else ''))
                return a
            if Ja is None:
                return a
            dd = -np.linalg.solve(np.asarray(Ja, float), np.asarray(e, float))       # [ID2, ID1] 度
            a = [min(max(a[0] + float(dd[1]), lo1), hi1), min(max(a[1] + float(dd[0]), lo2), hi2)]
            self.log(f'   按比赛的走法(从看的姿态一下转过去)试了一次：离爪子点 {em:.1f}mm(一点一点对和一下转过去停的地方不一样)，'
                     f'角度修成 ID1 {a[0]:.2f}°、ID2 {a[1]:.2f}°')
        self._raw_ct_approach(home[0], home[1])
        self.arm.ap(a[0], a[1])
        e = v.material_error(int(color), n=2)
        if e is not None:
            self.log(f'   修了 {int(cfg.get("raw_teach_verify_n", 3))} 次以后离爪子点 {float(np.linalg.norm(e)) / sc:.1f}mm')
        return a

    def _raw_ct_slot_px(self, D):
        """示教数据里三个停靠位的画面位置 {k: 像素}；有一个没示教(被爪子挡住)就按另外两个推出来。不够三个返回 None。"""
        sl = {str(k): np.asarray(s['px'], float)[:2] for k, s in (D.get('slots') or {}).items() if isinstance(s, dict) and s.get('px')}
        if len(sl) == 2:
            k = next(x for x in ('1', '2', '3') if x not in sl)
            g = self._raw_ct_guess_px(D, k)
            if g is not None:
                sl[k] = np.asarray(g, float)
        return sl if len(sl) >= 3 else None

    def _raw_ct_margins(self, P, home, Ja, shifts):
        """(10-18)车挪了 shifts(N×2 像素：画面里所有东西一起挪多少)以后，每个停靠位手臂要转到哪、离能转的范围边上还剩多少毫米
        (负的 = 够不着，差这么多)。按手臂和画面的对应关系 Ja 估。返回 {k: (ID1 N个, ID2 N个, 余量毫米 N个)}。"""
        cfg, v = self.cfg, self.vision
        sc = float(v.scale('RAW'))
        claw = np.array(v.claw('RAW'), float)
        lim = dict(id1=60.0, id2=700.0)
        lim.update(cfg.get('raw_center_reach_deg') or {})
        a1r = self._raw_a1_range()
        a2lo, a2hi = self._a2_limits()
        Ja = np.asarray(Ja, float)
        Ainv = np.linalg.inv(Ja)
        mm2 = float(np.linalg.norm(Ja[:, 0])) / sc                # ID2 每度多少毫米
        mm1 = float(np.linalg.norm(Ja[:, 1])) / sc                # ID1 每度多少毫米
        out = {}
        for k, px in P.items():
            e = (np.asarray(px, float) - claw)[:, None] + np.asarray(shifts, float).T      # 2×N
            d = -Ainv @ e                                          # [ID2, ID1] 度
            a1, a2 = float(home[0]) + d[1], float(home[1]) + d[0]
            m2 = np.minimum.reduce([a2 - a2lo, a2hi - a2, float(lim['id2']) - np.abs(d[0])]) * mm2
            m1 = np.minimum.reduce([a1 - float(a1r[0]), float(a1r[1]) - a1, float(lim['id1']) - np.abs(d[1])]) * mm1
            out[k] = (a1, a2, np.minimum(m1, m2))
        return out

    def raw_ct_shift(self, path):
        """rteach RAW shift(10-18)：三个停靠位的画面位置都示教过以后用(够不着的位 rteach RAW n 颜色号 也会记下位置)。
        手臂伸缩(ID2)的行程不够同时够着三个停靠位时：按手臂和画面、车轮和画面的对应关系，算车停在哪里够得着的停靠位最多、
        离能转的范围边上最远；车轮(只用 V，车头不转)挪过去，停稳了重新认圆心，记成以后比赛对准的位置；
        停靠位的画面位置跟着挪，抓取角度清掉(要重新示教够得着的那几个)。返回车挪了多少 (横移, 前进) 毫米。"""
        cfg, v, act = self.cfg, self.vision, self.act
        sc = float(v.scale('RAW'))
        d = load_place_adj(path)
        D = d.get('RAW_CT')
        if not isinstance(D, dict) or not D.get('c'):
            raise ValueError('先做 rteach RAW center')
        if D.get('near'):
            raise ValueError('现在是"最近那格"的做法(rteach RAW near)：车停在哪是示教时车轮对的，不用 shift')
        P = self._raw_ct_slot_px(D)
        if P is None:
            raise ValueError('三个停靠位的画面位置还没示教全：先 rteach RAW 1/2/3 颜色号(够不着也会记下位置；被爪子挡住的那个最后做)')
        Ja = self.store.get('RAW', 'arm') if self.store is not None else None
        Jc = self.store.get('RAW', 'ch') if self.store is not None else None
        if Ja is None:
            raise ValueError('还没有手臂和画面的对应关系：先 vcal RAW 颜色号 arm')
        if Jc is None or self._raw_J_bad(Jc, sc):
            raise ValueError('没有车轮和画面的对应关系(或者不合理)：先 rteach RAW center(会现测)')
        if act is None:
            raise ArmError('没有底盘(车轮)')
        Jc = np.asarray(Jc, float)
        home = [float(x) for x in D['home'][:2]]
        smax, fmax = float(cfg.get('raw_center_s_mm', 40.0)), float(cfg.get('raw_center_f_mm', 60.0))
        ok_mm = float(cfg.get('raw_shift_margin_mm', 3.0))
        _a, comp = self._raw_ct_pose(D)
        self.log('  先让车轮把原料盘中心对到示教时的位置(车可能被碰过)……')
        m = self._raw_center_align(goal=np.asarray(D['c'], float)[:2] + comp, tol=float(cfg.get('raw_ct_tol_mm', 3.0)))
        if m is None or not (getattr(self, '_rc_last', None) or {}).get('ok'):
            raise VisionError('没对准示教时的位置，车不挪：' + str((getattr(self, '_rc_last', None) or {}).get('why') or ''))
        # 10-18 实车：被爪子挡住、按另外两个位推出来的位置不准(摄像头斜着看，画面上面压扁了)：按它算会以为够得着、其实差几十毫米。
        # 看到过的位有两个以上就只按看到过的算
        sl_ = D.get('slots') or {}
        guessed = {k for k in P if not isinstance(sl_.get(k), dict) or not sl_[k].get('px') or sl_[k].get('guessed')}
        if len(P) - len(guessed) >= 2:
            P = {k: px for k, px in P.items() if k not in guessed}
            if guessed:
                self.log('  ' + '、'.join(f'{k}号位' for k in sorted(guessed)) + '的位置是按另外两个位推出来的(被爪子挡住看不到)，不准：只按看到过的位算')
        keys = sorted(P)
        now = self._raw_ct_margins(P, home, Ja, np.zeros((1, 2)))

        def say(mg, j):
            return '  '.join(f'{k}号位 ' + (f'够得着(离边上还有 {mg[k][2][j]:.0f}mm)' if mg[k][2][j] >= 0 else f'够不着(差 {-mg[k][2][j]:.0f}mm)')
                             for k in keys)
        self.log('  现在的位置：' + say(now, 0))
        old = np.asarray(D.get('shift_mm') or [0.0, 0.0], float)[:2]   # 以前 shift 已经挪过的(车轮能挪的范围按停车点算总共挪了多少)
        S, F = np.meshgrid(np.arange(-smax, smax + 0.5, 1.0) - old[0], np.arange(-fmax, fmax + 0.5, 1.0) - old[1])
        cand = np.c_[S.ravel(), F.ravel()]                       # 车(从现在的位置)再挪多少 (横移, 前进) 毫米
        mg = self._raw_ct_margins(P, home, Ja, cand @ Jc.T)
        M = np.stack([mg[k][2] for k in keys], axis=1)          # N×(几个位) 余量
        n_ok = (M >= ok_mm).sum(axis=1)
        Ms = -np.sort(-M, axis=1)                               # 每行从大到小
        worst = np.where(n_ok > 0, np.take_along_axis(Ms, np.maximum(n_ok - 1, 0)[:, None], axis=1)[:, 0], Ms[:, 0])
        good_mm = float(cfg.get('raw_shift_good_mm', 8.0))      # 余量到这么多就够了：再多也不值得多挪车
        score = n_ok * 1e6 + np.minimum(np.round(worst, 1), good_mm) * 1e3 - np.hypot(cand[:, 0], cand[:, 1])
        j = int(np.argmax(score))
        ds, df = float(cand[j, 0]), float(cand[j, 1])
        n_now = int((np.array([now[k][2][0] for k in keys]) >= ok_mm).sum())
        if int(n_ok[j]) <= n_now and math.hypot(ds, df) < 3.0:
            self.log('  现在的位置已经是最好的了，不用挪')
            return 0.0, 0.0
        self.log(f'  车停在横移 {ds:+.0f}mm、前进 {df:+.0f}mm 的地方最好：' + say(mg, j))
        if int(n_ok[j]) < len(keys):
            self.log('  (手臂够不着三个停靠位：比赛时要的颜色停在够不着的位上就等下一次停，'
                     '原料盘每次转 120°，下一次就转到够得着的位了)')
        d0 = (float(act.disp['S']), float(act.disp['F']))
        self.log(f'  车轮只用 V 慢慢挪过去(车头不转)：横移 {ds:+.0f}mm、前进 {df:+.0f}mm……')
        okv = self._vel_goto(d0[0] + ds, d0[1] + df, tol_mm=1.0)
        ms_, mf_ = float(act.disp['S']) - d0[0], float(act.disp['F']) - d0[1]
        left = math.hypot(ds - ms_, df - mf_)
        if not okv and left > 3.0:
            raise ArmError(f'车轮没挪到(只用 V 挪：{getattr(self, "_vel_why", "") or "不知道为什么"}；挪了横移 {ms_:+.0f}mm、前进 {mf_:+.0f}mm，'
                           f'还差 {left:.0f}mm)：没保存。车被挡住了吗？再输入一次 rteach RAW shift')
        ds, df = ms_, mf_                                      # 按真挪了多少记(挪完重新认圆心，差一两毫米没关系)
        ms = self._raw_center_still(5)
        if ms is None:
            raise VisionError('挪过去以后认不出原料盘中心：没保存')
        c_new = np.asarray(ms[0], float) - comp                # 按示教时看原料盘的角度记(还差的那一点角度让画面挪了 comp)
        dc = c_new - np.asarray(D['c'], float)[:2]
        D['c'] = [round(float(c_new[0]), 1), round(float(c_new[1]), 1)]
        D['shift_mm'] = [round(float(old[0]) + ds, 1), round(float(old[1]) + df, 1)]   # 从停车点算一共挪了多少
        for k, sv in (D.get('slots') or {}).items():
            if isinstance(sv, dict) and sv.get('px'):
                sv['px'] = [round(float(sv['px'][0] + dc[0]), 1), round(float(sv['px'][1] + dc[1]), 1)]
                sv['a'] = None
        save_json(path, d)
        good = [k for k in keys if mg[k][2][j] >= ok_mm]
        cols = {str(k): (sv or {}).get('color') for k, sv in (D.get('slots') or {}).items()}
        self.log(f'  记下了新的对准位置：原料盘中心在画面 ({c_new[0]:.0f}, {c_new[1]:.0f})(比赛时车轮对到这里)。抓取角度清掉了，'
                 '车别动、原料盘别转，接着重新示教够得着的位：'
                 + '、'.join(f'rteach RAW {k} ' + (str(cols.get(k)) if cols.get(k) else '颜色号') for k in good)
                 + '。以后别再做 rteach RAW center(会回到正中、清掉这些)')
        return ds, df

    def raw_ct_save_slot(self, n, path):
        """rteach RAW n save：手臂现在的角度记成 n 号停靠位的抓取角度(rteach RAW n 颜色号 没对上时，用 arm AD 手动摆到物料正上方以后用)。"""
        d = load_place_adj(path)
        D = d.get('RAW_CT')
        if not isinstance(D, dict) or not D.get('c'):
            raise ValueError('先做 rteach RAW center')
        s = (D.get('slots') or {}).get(str(int(n)))
        if not isinstance(s, dict) or not s.get('px'):
            g = self._raw_ct_guess_px(D, n)                    # 物料被爪子挡住看不到：按另外两个位推出画面位置
            if g is None:
                raise ValueError(f'先做 rteach RAW {n} 颜色号(记下这个停靠位在画面里的位置)；'
                                 '这个位的物料被爪子挡住看不到的话，先示教另外两个位')
            s = D.setdefault('slots', {})[str(int(n))] = dict(px=[round(float(g[0]), 1), round(float(g[1]), 1)], a=None,
                                                              color=None, guessed=True)
            self.log(f'   {n} 号停靠位的画面位置按另外两个位推出来：({g[0]:.0f}, {g[1]:.0f})')
        a = self.arm.read_angles()
        if a[0] is None or a[1] is None:
            raise ArmError('读不到手臂角度，没保存')
        s['a'] = [round(float(a[0]), 2), round(float(a[1]), 2)]
        save_json(path, d)
        self.log(f'   {n} 号停靠位的抓取角度记成现在的：ID1 {s["a"][0]:.2f}°、ID2 {s["a"][1]:.2f}°')

    def raw_ct_goto_slot(self, n, path):
        """rteach RAW n go：手臂(在看原料盘的高度)转到 n 号停靠位示教的角度，看爪子是不是在物料正上方。"""
        D = load_place_adj(path).get('RAW_CT')
        s = ((D or {}).get('slots') or {}).get(str(int(n))) if isinstance(D, dict) else None
        if not isinstance(s, dict) or not s.get('a'):
            raise ValueError(f'{n} 号停靠位还没有抓取角度(rteach RAW {n} 颜色号 或 rteach RAW {n} save)')
        self.arm.obs('RAW', open_claw=True)
        self.arm.ap(float(s['a'][0]), float(s['a'][1]))
        self.log(f'   手臂转到 {n} 号停靠位示教的角度：ID1 {s["a"][0]:.2f}°、ID2 {s["a"][1]:.2f}°(在看原料盘的高度)。'
                 f'爪子不在物料正上方就用 arm AD 1 / arm AD 2 调，再 rteach RAW {n} save')

    def _raw_wheels_on(self):
        """原料区这次只动车轮对准吗：align_by.RAW='wheels'，而且有车轮和画面的对应关系(vcal RAW 颜色号 不加 arm 测的)。
        只有手臂的(以前 vcal RAW 颜色号 arm 测的)、没有车轮的：这次还是按手臂对准(原料盘一停一转时现测车轮会测错)，提醒一次。"""
        if getattr(self, '_raw_follow_now', False):
            return False                                      # 跟随抓取：只动手臂
        if not self._by_wheels('RAW'):
            return False
        st = self.store
        if st is not None and st.get('RAW', 'ch') is None and st.get('RAW', 'arm') is not None:
            if not getattr(self, '_raw_wheels_fallback_said', False):
                self._raw_wheels_fallback_said = True
                self.log('    ★ 原料区要只动车轮对准(align_by.RAW=wheels)，但还没有车轮和画面的对应关系，只有手臂的：这次先按手臂对准。'
                         '原料盘停转、物料放在爪子下面，做一次 vcal RAW 颜色号(不加 arm)就能只动车轮了')
            return False
        return True

    def _raw_J(self):
        """原料区对准用的 J：只动车轮时是车轮的(RAW ch)，否则是手臂的(RAW arm)；没有返回 None。"""
        if self.store is None:
            return None
        return self.store.get('RAW', 'ch' if self._raw_wheels_on() else 'arm')

    def _raw_wheels_kw(self):
        """原料区只动车轮对准：横移累计在 raw_wheels_s_range、前后累计在 ±raw_fix_max_mm 以内(都相对原料区停车点)。"""
        cfg = self.cfg
        d = self.act.disp if self.act is not None else {'S': 0.0, 'F': 0.0}
        r = cfg.get('raw_wheels_s_range') or [-15.0, 15.0]
        fm = float(cfg.get('raw_fix_max_mm') or 0.0)
        ds, df = float(d['S']), float(d['F'])
        return dict(wheels_only=True, chassis_s_range=(float(r[0]) - ds, float(r[1]) - ds), chassis_f_range=(-fm - df, fm - df))

    def _raw_reachable(self, J, e):
        """按手臂和画面的对应关系 J 算：对准偏差 e(像素)要转多少度，超出手臂能转的范围就返回说明，够得着返回 ''。
        只动车轮时 J 是车轮的：按车轮横移/前后还能挪的范围算。"""
        if self._raw_wheels_on():
            return self._raw_reachable_wheels(J, e)
        try:
            d = -np.linalg.solve(np.asarray(J, float), np.asarray(e, float))       # [ID2, ID1] 度
        except np.linalg.LinAlgError:
            return ''
        lim = (self.servo.cfg.get('arm_limit_deg') or {}) if self.servo is not None else {}
        out = []
        dc = np.array(d, float)
        for i, key in enumerate(('id2', 'id1')):
            lm = 0.95 * float(lim.get(key, 250.0 if key == 'id2' else 12.0))
            if abs(d[i]) > lm:
                out.append(f'{"ID2" if i == 0 else "ID1"} 要转 {d[i]:+.0f}°，最多 ±{lm:.0f}°')
                dc[i] = math.copysign(lm, d[i])
        if out and self.vision is not None:
            # (10-11 用户：误差 30mm 以内都可以夹)手臂转到头以后剩下的偏差在 raw_grab_max_mm 以内：也算够得着，去夹
            try:
                rest = float(np.linalg.norm(np.asarray(e, float) + np.asarray(J, float) @ dc)) / float(self.vision.scale('RAW'))
                if rest <= float(self.cfg.get('raw_grab_max_mm', 5.0)):
                    return ''
            except Exception:
                pass
        return '，'.join(out)

    def _raw_reachable_wheels(self, J, e):
        """只动车轮：对准偏差 e(像素)要横移/前后挪多少，超出允许的范围就返回说明；挪到范围边上剩下的偏差不超过 raw_grab_max_mm 也算够得着。"""
        try:
            sf = -np.linalg.solve(np.asarray(J, float), np.asarray(e, float))       # [S, F] 毫米
        except np.linalg.LinAlgError:
            return ''
        kw = self._raw_wheels_kw()
        out = []
        sc = np.array(sf, float)
        for i, (name, r) in enumerate((('横移', kw['chassis_s_range']), ('前后', kw['chassis_f_range']))):
            if not r[0] - 0.5 <= sf[i] <= r[1] + 0.5:
                out.append(f'车轮{name}要挪 {sf[i]:+.0f}mm，只许 {r[0]:+.0f}~{r[1]:+.0f}mm')
                sc[i] = max(r[0], min(r[1], sf[i]))
        if out and self.vision is not None:
            try:
                rest = float(np.linalg.norm(np.asarray(e, float) + np.asarray(J, float) @ sc)) / float(self.vision.scale('RAW'))
                if rest <= float(self.cfg.get('raw_grab_max_mm', 5.0)):
                    return ''
            except Exception:
                pass
        return '，'.join(out)

    def _raw_wheels(self):
        """原料区这次对准先不先动车轮：配置(或 mtest RAW … wheels)要求，而且存了车轮和画面的对应关系(vcal RAW 颜色号)。"""
        want = bool(self.cfg.get('raw_wheels_first', True)) or bool(getattr(self, 'raw_wheels_once', False))
        if RAW_CAR_FIXED or not want:
            return False
        if self.store is None or self.store.get('RAW', 'ch') is None:
            if not getattr(self, '_raw_wheels_said', False):
                self._raw_wheels_said = True
                self.log('    (原料区先动车轮：还没有车轮和画面的对应关系，这次只动手臂。原料盘停转时做一次 vcal RAW 颜色号(不加 arm)就有了)')
            return False
        return True

    def _raw_align_go(self, color, slot, mode, e0, deadline, label, probe=False):
        """物料停着：只动手臂快速对准(用存好的 J，不探测、不改 J，一下修到位，最多 raw_max_iter 次)；
        到 deadline(这一次停下必须下爪的时刻)就不再修，按当时的偏差下爪——只要不超过 raw_grab_max_mm。
        probe=True：还没有 J、原料盘一直停着：按普通的闭环对准(先小幅动几下测 J，存下来)。"""
        cfg, v = self.cfg, self.vision
        n = int(cfg.get('raw_frames', 1))
        tol = float(cfg.get('raw_tol_mm') or cfg['tol_mm']['RAW'])
        grab_max = float(cfg.get('raw_grab_max_mm', 5.0))
        budget = deadline - self.now()
        kw = dict(fixed_j=True, max_iter=int(cfg.get('raw_max_iter', 3)), gain_arm=float(cfg.get('raw_gain', 1.0)), near_avg=0.0,
                  timeout_s=max(0.01, budget - 0.4))          # 修一下 + 再看一眼大约 0.4 秒：留出来，不超过必须下爪的时刻
        if probe:
            kw = dict(fixed_j=False, timeout_s=max(0.3, budget))
        allow = False                                        # (10-11 用户：写死)原料区车轮不动，只动手臂(配置里的 raw_chassis 不管)
        if self._raw_wheels_on():
            # (10-12 用户)只动车轮：手臂停在 OBS RAW 不动，车轮横移/前后(限范围)把物料对到爪子下面；修正比例 raw_gain
            kw.pop('gain_arm', None)
            kw.update(self._raw_wheels_kw())
            kw['gain_ch'] = float(cfg.get('raw_gain', 1.0))
        res = self.servo.run('RAW', _first(tuple(e0), lambda: v.material_error(color, n=n)), v.scale('RAW'), tol,
                             allow_chassis=allow, label=label, bounds=v.bounds('RAW'), confirm=False, **kw)
        self.log(f'    对准结果：{res}')
        err = float(res.err_mm)
        force = bool(cfg.get('raw_force_grab', True)) and math.isfinite(err) and err <= float(cfg.get('raw_force_max_mm', 60.0))
        if force and ('误差变大' in (res.reason or '') or err > grab_max or self.now() > deadline + 0.25):
            self.log(f'    ★ 偏差 {err:.1f}mm({res.reason or "到时间了"})：一定要夹，照样下爪')
        elif '误差变大' in (res.reason or ''):
            self._raw_bad_j = getattr(self, '_raw_bad_j', 0) + 1
            if self._raw_bad_j == 2:
                self.log('    ★ 连着两次越对越偏：存的原料区对应关系(J)可能不对(以前在转盘转的时候测的？)。'
                         '先让原料盘停转，物料放在爪子下面，重新做一次 vcal RAW 颜色号 arm')
            self.log('    ★ 越对越偏(转盘转起来了？)：不下爪，等下一次停')
            return False
        if not math.isfinite(err):
            self.log(f'    ★ 对准中途看不到物料了({res.reason})：不下爪，等下一次停')
            return False
        if err > grab_max and not force:
            self.log(f'    ★ 偏差 {err:.1f}mm，超过 {grab_max:g}mm，下爪会砸到物料：不下爪，等下一次停')
            return False
        if self.now() > deadline + 0.25 and not force:
            self.log('    ★ 对准超时太多，原料盘马上要转了：不下爪，等下一次停')
            return False
        if mode == 'nogo':
            self.log(f'    nogo：对准了(偏差 {err:.1f}mm)，不夹。看看爪子是不是在物料正上方')
            return True
        self.log(f'    下爪(偏差 {err:.1f}mm' + ('' if res.ok else '，停的时间到了，按现在的位置夹') + ')')
        if mode != 'nogo':
            self._raw_remember_pose()
        if mode == 'lift':
            P = self.arm.params() or {}
            self.arm.do(f'LIFT {float(P["ZGRAB"]):g}')
            self.arm.do('CLAW C')
            self.arm.do(f'LIFT {float(P["ZHI"]):g}')
            return True
        self.arm.grab_here(slot)
        return True

    # ------------------------------------------------------------------ ROUGH：放下，再取回
    def _rough(self, visit):
        if not self._can_work():
            return
        batch = 1 if visit <= 1 else 2
        items = self.plan.items(batch)
        placed = []
        self._zone_start('ROUGH', items)
        for item in self._code_order([(it, it.ring) for it in items]):
            item = item[0]
            self._check_abort()
            if self.disabled:                                     # 中途出了不能继续夹放的错(见 _put_back)
                break
            if not same_item(self.in_tray.get(item.slot), item):
                self.log(f'    转盘 {item.slot} 号槽里没有 {item.color_name}(没抓到)，不放')
                continue
            if self._no_time('place', '放置'):
                break
            t_it = self.now()
            ok = self._place_item(item, 'ROUGH', item.ring, stack=False, label=f'粗加工放{item.color_short}')
            self._note_item('place', t_it)
            if ok:
                placed.append(item)
        seq = self._pickback_sequence('ROUGH', placed)
        k = self._pickback_budget(len(seq))
        if k < len(seq):
            seq = seq[:k]                                        # 取回了也来不及去暂存区放的：留在粗加工区(放下时已经算分)
        for item in seq:
            self._check_abort()
            if self.disabled:                                     # 中途出了不能继续夹放的错(见 _put_back)
                break
            if self._no_time('pick', '取回(物料留在粗加工区，放在那儿也算分)'):
                break
            t_it = self.now()
            self._pickback_item(item, 'ROUGH')
            self._note_item('pick', t_it)
        self._stow_quiet()                                       # 先收臂(放完不缩回时爪子还伸在圆环上方)，再挪底盘
        self._zone_back('ROUGH')

    def _goto_recorded(self, zone, item):
        """取回时直接回到放下那一刻的底盘位置和手臂角度(物料就在那儿)。成功回到返回 True；没有记录或不允许返回 False。
        回去以后下一步的 servo.run 会先测一次：偏差在容差内就不动，直接夹。"""
        rec = self.pose_at.get((zone, item.slot))
        if not rec or not self.cfg.get('pickback_fast', True):
            return False
        d = self.act.disp
        dS, dF = int(round(rec['S'] - d['S'])), int(round(rec['F'] - d['F']))   # 放下时横移过(手臂够不着)的话也回到那里
        if abs(dS) < 3 and self._by_wheels(zone):
            dS = 0                                           # 只动车轮：差两三毫米的横移不单独走一条指令，对准时一起修
        if dS or dF:
            self.log(f'    底盘回到放下时的位置：前进 {dF:+d}mm、横移 {dS:+d}mm')
            self._zone_v_only = False
            self.act.chassis_move(dS, dF)
        self._return_to_pose(rec['a1'], rec['a2'], from_tray=not self._ring_ready)   # 刚把上一个夹回转盘：从转盘那边转回来
        zobr = float(self.arm.params().get('ZOBRNG', 0.0))
        if zobr > 0.05:
            self.arm.lift(zobr)                              # 摄像头标定比例时用的高度
        return True

    def _pickback_sequence(self, zone, placed):
        mode = self.cfg.get('pickback_order', 'code')
        if mode == 'reverse':
            return list(reversed(placed))
        if mode == 'near':
            def pos_of(it):                                      # 这个物料放下时底盘的前后位置
                rec = self.pose_at.get((zone, it.slot))
                if rec is not None:
                    return float(rec['F'])
                f = self.ring_f.get((zone, int(it.ring)))
                return f if f is not None else self._ring_nominal(zone, it.ring) + self.learn['F']
            left, out, pos = list(placed), [], self.act.disp['F']
            while left:
                nxt = min(left, key=lambda it: abs(pos_of(it) - pos))
                out.append(nxt)
                left.remove(nxt)
                pos = pos_of(nxt)
            return out
        return sorted(placed, key=lambda it: int(it.index))       # (10-22)按任务码顺序取回(以前跟着放的顺序，放是顺路 3→2→1 时取回也成了 3→2→1)

    def _pickback_item(self, item, zone, again=False):
        cfg = self.cfg
        self.log(f'  ▶ 从{zone}取回 {item.color_name}(环{item.ring}) -> 转盘 {item.slot} 号槽' + ('(再取一次)' if again else ''))
        self._ui('stage', f'{zone[:5]} PICK {item.index + 1}/3 R{item.ring}')
        wheels = self._by_wheels(zone)
        acc = self._wheels_accept('PICK') if wheels else cfg['accept_mm']['PICK']
        try:
            # (10-19)只有车没动、只转手臂放下的(_place_item_arm)，车还在那时的位置，才按记下的角度直接取；挪车放的照旧对准(车挪一次差几毫米)
            rec0, d0 = self.pose_at.get((zone, item.slot)), self.act.disp
            direct = bool(rec0 and rec0.get('arm') and self._p2.get(zone) is not None
                          and abs(float(rec0['S']) - float(d0['S'])) < 2.0 and abs(float(rec0['F']) - float(d0['F'])) < 2.0)
            recorded = self._goto_recorded(zone, item)
            if recorded and direct:
                self.log('    按放下时的角度直接取(车没动、只转手臂放的，不再对准)')
                if self.nogo:
                    return True
                self._ring_ready = self._at_obs = False
                self._apply_adj('RING')
                self.arm.pick_here(item.slot)
                self.in_tray[item.slot] = item
                stack = self.on_ring.get((zone, item.ring))
                if stack and item in stack:
                    stack.remove(item)
                return True
            self._zone_v_only = False                            # 下面照旧对准(可能发 S/F)
            if not recorded:
                self._goto_ring(zone, item.ring)
                if wheels:
                    self._obs_fixed()
                else:
                    self._obs_ring()
            res, how = self._align_covered(item.color, 'PICK', f'取回{item.color_short}', confirm=False, wheels=wheels)
            if res is None:
                if not recorded:
                    self._recover('物料和圆环都看不到，不取')
                    return False
                self.log('    物料和圆环都看不到：按放下时记下的位置直接取')
                self._goto_recorded(zone, item)                  # 按物料对准时可能已经动过手臂/底盘：先回到记下的位置
            else:
                self.log(f'    对准结果(按{how})：{res}')
                if not res.ok and not res.err_mm <= acc:
                    if recorded and not math.isfinite(res.err_mm):
                        self.log('    对准时目标丢了：回到放下时记下的位置直接取')
                        self._goto_recorded(zone, item)
                    else:
                        self._recover(f'没对准({res.reason})，不取，免得夹歪碰倒')
                        return False
                else:
                    self._learn_from(zone, item.ring)
            if self.nogo:
                self.log('    (nogo：到了取回的位置，不夹)')
                return True
            self._ring_ready = self._at_obs = False              # 夹起来放进转盘，手臂停在转盘上方
            self._apply_adj('RING')                              # 夹回时爪子也按第一层的补偿对正(和放下时一样的偏差)
            self.arm.pick_here(item.slot)
        except ArmAbort:
            raise
        except ArmError as ex:
            # (10-23)以前一律"收臂"：夹紧以后才出错时物料还在爪子里、STM32 也记着夹着，后面取回、放置全回 ERR HOLD，整轮不再夹放。
            #   和 GRAB 出错一样按夹爪最后一次动作看：夹紧了 -> 放进它的转盘槽(算取回了)；松进转盘了 -> 算取回了；
            #   还没夹 -> 物料还在环上，收臂以后再取一次
            claw = getattr(self.arm, 'claw_last', None)
            if self._grab_failed(item, ex, what='取回'):
                stack = self.on_ring.get((zone, item.ring))
                if stack and item in stack:
                    stack.remove(item)
                return True
            if (not again and not self.disabled and claw not in ('C', 'OT')
                    and not any(k in str(ex) for k in ('ERR HOLD', 'ERR LIFT', 'ERR NOZERO'))):
                return self._pickback_item(item, zone, again=True)
            return False
        except VisionError as ex:
            self._recover(f'取回出错：{ex}')
            return False
        self.in_tray[item.slot] = item
        stack = self.on_ring.get((zone, item.ring))
        if stack and item in stack:
            stack.remove(item)
        return True

    # ------------------------------------------------------------------ TEMP
    def _temp(self, visit):
        if not self._can_work():
            return
        batch = 1 if visit <= 1 else 2
        todo = []
        for item in self.plan.items(batch):
            if not same_item(self.in_tray.get(item.slot), item):
                self.log(f'    转盘 {item.slot} 号槽里没有 {item.color_name}(不在车上)，不放')
                continue
            ring, stack = item.ring, False
            if batch == 2:
                ring = self._stack_target(item)
                if ring is None:
                    # 规则：第二批在暂存区只能码垛在同色的第一批物料上；平放 0 分还白花时间(放置数也不算)
                    self.log(f'    {item.color_name} 没有同色的第一批物料可叠(第一批那个没放成功)：规则只许码垛，不放、不算放置，留在车上')
                    continue
                stack = True
                self.log(f'    {item.color_name} 码垛到环{ring}(下面是第一批的 {self.plan.lower_item(item).color_name})')
            todo.append((item, ring, stack))
        self._zone_start('TEMP', [it for it, _r, _s in todo])
        for item, ring, stack in self._code_order(todo):
            self._check_abort()
            if self.disabled:                                     # 中途出了不能继续夹放的错(见 _put_back)
                break
            kind = 'stack' if stack else 'place'
            if self._no_time(kind, '码垛' if stack else '放置'):
                break
            t_it = self.now()
            self._place_item(item, 'TEMP', ring, stack=stack, label=('码垛' if stack else '暂存放') + item.color_short)
            self._note_item(kind, t_it)
        self._stow_quiet()                                       # 先收臂(放完不缩回时爪子还伸在物料上方)，再挪底盘
        self._zone_back('TEMP')

    def _code_order(self, todo):
        """(10-22 用户"这个也要按顺序来")放的顺序：按任务码的顺序(规则要求)，不再顺路 3→2→1。todo 里每一项第 2 个是圆环号。"""
        todo = list(todo)
        if str(self.cfg.get('place_order', 'code')).lower() == 'sweep' and not getattr(self, '_sweep_said', False):
            self._sweep_said = True
            self.log('    (配置里的 place_order=sweep(顺路 3→2→1)不再用：规则要求按任务码的顺序放)')
        if len(todo) > 1:
            self.log('    按任务码顺序放：' + ' → '.join(f'环{t[1]}' for t in todo))
        return todo

    def _stack_target(self, item):
        """第二批的 item 在暂存区码垛到哪个环：第一批同色的那个物料放在暂存区的环(按记录确实放在那儿)；没有返回 None。"""
        low = self.plan.lower_item(item) if self.plan is not None else None
        if low is None:
            return None
        if any(same_item(x, low) for x in self.on_ring.get(('TEMP', low.ring)) or []):
            return low.ring
        return None

    # ------------------------------------------------------------------ 放到圆环(核心)
    def _place_take_first(self, item, zone, ring, label):
        """先去转盘取物料，再到圆环上方带着物料对准一次就放下(用户 10-10 选的做法，省掉"空爪先对准、再回去取、再回到原位")。
        返回 (这个物料处理完没有, 放成没有, 误差mm)。处理不了(高度不够、看不到圆环、对不准)：物料放回转盘槽，
        返回 (False, …)，调用的地方按以前的做法(空爪先对准)再放这一个。"""
        cfg = self.cfg
        P = self.arm.params() or {}
        if not all(k in P for k in ('ZOBRNG', 'ZPLC', 'ZHI', 'A1P', 'A2P')):
            return False, False, None
        up = 1.0 if float(P['ZHI']) >= float(P['ZPLC']) else -1.0     # 升降往上是加(现在的车)还是减(以前/模拟)
        clear = up * (float(P['ZOBRNG']) - float(P['ZPLC']))
        if clear < float(cfg.get('take_first_clear_mm', 15.0)):
            if not getattr(self, '_tf_said', False):
                self._tf_said = True
                self.log(f'    (先取物料再对准：观察高度 ZOBRNG 时物料底面离地只有 {clear:.0f}mm，太低会蹭到圆环/地面，按以前的做法空爪先对准)')
            return False, False, None
        self._goto_ring(zone, ring)
        self._ring_ready = self._at_obs = False
        try:
            self._take(item)                                     # 先去转盘取物料
        except ArmAbort:
            raise
        except ArmError as ex:                                   # (10-23)以前这里出错整个停车点都不做了
            if self.holding is item:
                self._put_back(item, f'先取物料出错：{ex}')
                self.holding = None
            else:
                self._recover(f'先取物料出错：{ex}')
            return False, False, None
        self.holding = item
        try:
            d2 = self._predict_d2() if cfg.get('ring_precorrect', True) else None
            d1 = self._predict_d1(d2) if d2 is not None else 0.0
            env = self._a2_env() or self._a2_limits()
            a2 = min(env[1], max(env[0], float(P['A2P']) + (d2 or 0.0)))
            a1 = float(P['A1P']) + (d1 or 0.0)
            self.log(f'    先取了物料，带着它到圆环上方对准(ID2 {a2 - float(P["A2P"]):+.0f}°)，对准一次就放下')
            self.arm.apz(a1, a2, float(P['ZOBRNG']))             # 转过去、降到看圆环的高度(新程序边转边降)
            self._a1_hint, self._a2_hint = a1, a2
            kw = self._zone_servo_kw()
            kw['max_iter'] = int(cfg.get('take_first_iter') or 4)
            has_j = self.store is not None and self.store.get('RING', 'arm') is not None
            res = self.servo.run('RING', self._ring_measure(), self.vision.scale('RING'), cfg['tol_mm']['RING'],
                                 allow_chassis=True, label=label, bounds=self.vision.bounds('RING'),
                                 confirm=bool(cfg.get('place_confirm', False)), fixed_j=has_j, **kw)
            self.log(f'    对准结果(手里带着物料)：{res}')
            # 物料已经在爪子里：尽量不放回车上(放回去这个环就是 0 分)。对得准就放；差一些就接着修(这次允许现场测 J)；
            # 偏差在 take_first_drop_mm 以内照样放(还在内几环，有分)。圆环一直看不到、或者差得太远(会砸到旁边的环/物料，
            # 碰到已经放好的物料本轮直接结束)才放回转盘，这个环再按空爪先对准的做法放
            drop_mm = float(cfg.get('take_first_drop_mm') or 12.0)
            if not self.ring_gate_px:                            # 没看清三个环(不知道旁边的环在哪)：偏差大时可能对的是旁边那个环，放宽不安全
                drop_mm = min(drop_mm, float(cfg['accept_mm']['RING']))

            def good(r):
                return r is not None and math.isfinite(r.err_mm) and (r.ok or r.err_mm <= drop_mm)

            far = res is None or not math.isfinite(res.err_mm) or res.err_mm > float(cfg.get('take_first_retry_max_mm') or 40.0)
            if not good(res) and not far and self.ring_gate_px:                        # 差得太远(可能对到旁边那个环)不追，免得放错环
                kw2 = self._zone_servo_kw()
                res2 = self.servo.run('RING', self._ring_measure(), self.vision.scale('RING'), cfg['tol_mm']['RING'],
                                      allow_chassis=True, label=label, bounds=self.vision.bounds('RING'),
                                      confirm=False, **kw2)
                self.log(f'    接着修(手里带着物料)：{res2}')
                if res2 is not None and math.isfinite(res2.err_mm):
                    res = res2
            err = res.err_mm if (res is not None and math.isfinite(res.err_mm)) else None
            if err is None or err > drop_mm:
                why = '看不到圆环' if err is None else f'还差 {err:.1f}mm'
                self.log(f'    ★ 带着物料{why}：放下去会砸到旁边，只好放回转盘，这个环再按空爪先对准的做法放')
                self._put_back(item, f'带着物料{why}')
                self.holding = None
                return False, False, None
            if not res.ok:
                self.log(f'    带着物料还差 {err:.1f}mm(在 {drop_mm:.0f}mm 以内)：照样放下')
            self._learn_from(zone, ring)
            if hasattr(self.vision, 'note_ring_size'):
                self.vision.note_ring_size(getattr(self.vision, 'last_ring_rmax', None))   # 圆环外圈大小：取回时白心被盖住也认得出
            a1n, a2n = self.arm.read_angles()
            d = self.act.disp
            if a1n is not None and a2n is not None:
                self.pose_at[(zone, item.slot)] = dict(S=d['S'], F=d['F'], a1=a1n, a2=a2n)
                self._zone_d2.append((float(d['F']), float(a2n) - float(P['A2P'])))
            self._apply_adj('RING')                              # pcal 1 量过：按第一层的补偿再转一点
            self._drop(False)                                    # 下降、松手、抬起
            self.holding = None
            self._ring_ready = True
            return True, True, err
        except ArmAbort:
            raise
        except DroppedError as ex:                               # (10-23)已经在环上松手了：算放了，只收臂
            self.holding = None
            self._recover(f'{item.color_name}已经放在环{ring}上，抬起来时出错：{ex}')
            return True, True, None
        except VisionError as ex:
            if self.holding is None:
                return False, False, None
            self.log(f'    ★ 带着物料对准时摄像头出错({ex})：按算出来的位置直接放下(不放回车上)')
            try:
                self._drop(False)
                self.holding = None
                self._ring_ready = True
                return True, True, None
            except ArmAbort:
                raise
            except DroppedError as ex2:
                self.holding = None
                self._recover(f'{item.color_name}已经放在环{ring}上，抬起来时出错：{ex2}')
                return True, True, None
            except ArmError as ex2:
                self._put_back(item, f'放下出错：{ex2}')
                self.holding = None
                return False, False, None
        except ArmError as ex:
            if self.holding is not None:
                self._put_back(item, f'先取物料再对准时机械臂出错：{ex}')
                self.holding = None
            return False, False, None

    def _place_item(self, item, zone, ring, stack, label):
        cfg = self.cfg
        key = 'STACK' if stack else 'RING'
        self.log(f'  ▶ {zone} 放 {item.color_name}(转盘{item.slot}号槽) 到环{ring}' + (' [码垛]' if stack else ''))
        self._ui('stage', f'{zone[:5]} PLACE {item.index + 1}/3 R{ring}' + (' S' if stack else ''))
        err_mm = None
        ok = False
        held = False                                             # 爪子里夹着从转盘取出来的物料
        dropped = None                                           # (10-23)已经松手、抬起来才出的错
        wheels = self._by_wheels(zone)                           # (10-12 用户)只动车轮对准，手臂停在观察姿态不动
        if not stack and cfg.get('check_ring_empty', True) and self.on_ring.get((zone, ring)):
            names = '、'.join(it.color_name for it in self.on_ring[(zone, ring)])
            self.log(f'    ★ 按记录环{ring} 上还放着 {names}(程序记着之前放在这里、没取回；不是摄像头看到的)：'
                     '不去这个环、不放，免得砸上去；物料留在车上')
            self.stats.place(False)
            self._show_stats()
            return False
        T = self._teach(zone)
        if T is not None:
            r = self._place_item_arm(item, zone, ring, stack, label, T)
            if r is not None:
                return r
        self._zone_v_only = False                                # 下面照旧挪车对准(可能发 S/F)
        if not stack and not self.nogo and cfg.get('take_first', True) and not getattr(self, '_tf_off', False) and not wheels:
            done, ok_tf, err_tf = self._place_take_first(item, zone, ring, label)
            if not done and self.disabled:
                self.stats.place(False)
                self._show_stats()
                return False
            if done:
                self.stats.place(ok_tf)
                if ok_tf:
                    self.in_tray.pop(item.slot, None)
                    self.on_ring.setdefault((zone, ring), []).append(item)
                    self.placements.append((zone, ring, False, err_tf))
                self._show_stats()
                return ok_tf
        try:
            self._goto_ring(zone, ring)
            if wheels:
                self._obs_fixed()                                # 空爪摆到看圆环的观察姿态，之后手臂不再动
            else:
                self._obs_ring()                                 # 空爪到圆环上方：圆环不会被挡住
            if stack:                                            # 下面那层物料盖住了白心：先认它的顶面对准
                res, how = self._align_covered(item.color, key, label, confirm=bool(cfg.get('place_confirm', False)), wheels=wheels)
            else:
                kw = self._wheels_kw() if wheels else self._zone_servo_kw()
                res, how = self.servo.run('RING', self._ring_measure(), self.vision.scale('RING'), cfg['tol_mm'][key],
                                          allow_chassis=True, label=label, bounds=self.vision.bounds('RING'),
                                          confirm=bool(cfg.get('place_confirm', False)), **kw), '圆环'
            if res is not None:
                self.log('    对准结果' + (f'(按{how})' if stack else '') + f'：{res}')
                err_mm = res.err_mm
            acc = self._wheels_accept(key) if wheels else cfg['accept_mm'][key]
            aligned = res is not None and (res.ok or res.err_mm <= acc)
            if aligned and wheels and not res.ok:
                self.log(f'    只动车轮还差 {res.err_mm:.1f}mm(在 {acc:g}mm 以内)：照样放，不把物料留在车上')
            taken = self._ring_taken_why(zone, ring) if (aligned and not stack and cfg.get('check_ring_empty', True)) else None
            if res is None:
                self._recover('下面那层物料和圆环都看不到，物料留在车上，不放')
            elif not aligned:
                if not math.isfinite(res.err_mm) and (zone, int(ring)) in self.ring_f:
                    self.log(f'    ★ 开到算出来的环{ring}位置，爪子附近没看到圆环：可能认错了是几号环(车要停在 2 号环正对爪子)，'
                             '或者这个环被挡住了/反光')
                self._recover(f'没对准({res.reason})，物料留在车上，不放')
            elif taken == 'record':
                names = '、'.join(it.color_name for it in self.on_ring.get((zone, ring)) or [])
                self._recover(f'按记录环{ring} 上还放着 {names}(程序记着之前放在这里、没取回；不是摄像头看到的)，不放，免得砸上去；物料留在车上')
            elif taken:
                self._recover(f'摄像头看到环{ring} 的白心里有东西(别的物料？)，不放，免得砸上去；物料留在车上')
            else:
                self._learn_from(zone, ring)                     # 底盘为了对准挪了多少，下一个圆环直接带上
                if wheels and not stack and res.ok:
                    self._zone_s_ref = (zone, int(ring), float(self.act.disp['S']))
                if not stack and hasattr(self.vision, 'note_ring_size'):
                    self.vision.note_ring_size(getattr(self.vision, 'last_ring_rmax', None))   # 空圆环的大小：取回时白心被盖住也认得出
                if not stack and hasattr(self.vision, 'note_claw_clear'):
                    self.vision.note_claw_clear()                # 爪子空着时的爪子区域：取回蓝色物料时用
                a1, a2 = self.arm.read_angles()                  # 记下对准时 ID1、ID2 的角度
                if a1 is None or a2 is None:
                    raise ArmError('读不到对准时的手臂角度(A? 没回复)，不去取物料')
                d = self.act.disp
                self.pose_at[(zone, item.slot)] = dict(S=d['S'], F=d['F'], a1=a1, a2=a2)
                a2p = (self.arm.params() or {}).get('A2P')
                if a2p is not None:
                    self._zone_d2.append((float(d['F']), float(a2) - float(a2p)))   # 车离圆环那一排的远近：下一个环直接伸到这么远
                if self.nogo:
                    self.log('    (nogo：对准好了，不取物料、不放)')
                    return True
                self._ring_ready = self._at_obs = False
                self._take(item)                                 # 去转盘取物料
                held = True
                self.holding = item
                adj = self.place_adj('STACK' if stack else 'RING')
                if adj is not None:                              # pcal 量过：落点按这一层的补偿挪
                    self._return_to_pose(a1 + adj[0], a2 + adj[1], from_tray=True)
                else:
                    self._return_to_pose(a1, a2, from_tray=True)     # 回到记下的角度
                self._drop(stack)                                # 下降、松手、抬起
                held = False
                self.holding = None
                ok = True
                self._ring_ready = True                          # 爪子张着停在这个环上方：去下一个环不用整套 OBS
        except ArmAbort:
            raise
        except DroppedError as ex:                               # (10-23)已经在环上松手了：算放了，只收臂
            held, ok, dropped = False, True, ex
            self.holding = None
        except (ArmError, VisionError) as ex:
            if held or self.holding is item:                     # (10-23)_take 夹紧了才出错的也算夹着
                self._put_back(item, f'放置出错：{ex}')
            else:
                self._recover(f'放置出错：{ex}')
        self.stats.place(ok)
        if ok:
            self.in_tray.pop(item.slot, None)
            self.on_ring.setdefault((zone, ring), []).append(item)
            self.placements.append((zone, ring, bool(stack), err_mm))
        self._show_stats()
        if ok and dropped is not None:
            self._recover(f'{item.color_name}已经放在环{ring}上，抬起来时出错：{dropped}')
            return ok
        if ok and not stack and self._want_pick(item):
            self._learn_pick(item, self.pose_at[(zone, item.slot)])
        return ok

    # ------------------------------------------------------------------ (10-14 用户)只在 2 号环前对准一次，1、3 号环按示教转手臂
    def _teach(self, zone):
        """zone_reach='arm'：{环号: (dID1, dID2)}(相对 2 号环对准时的角度)。1、3 号环调过(arm_calib 的 R1、R3，或者以前的 rteach)就是调的；
        'cal' = True：(10-20)是 arm_calib 调的(车对准 2 号环以后不动，1、3 号环直接按它转过去，不再用摄像头对、不挪车)。
        (10-19)没调过、zone_arm_geom 时 1、3 是 None(对准 2 号环以后按环距算，见 _zone_geom)。不能只动手臂返回 None(大声提醒去 arm_calib 调)。"""
        if not hasattr(self, '_teach_all'):
            self._teach_all = load_place_adj(self.cfg.get('ring_teach_file'))
        t = (self._teach_all or {}).get(zone) or {}
        warned = self.__dict__.setdefault('_teach_warned', set())
        if str(self.cfg.get('zone_reach') or '').lower() != 'arm':
            if not (str(t.get('src') or '') == 'arm_calib' and '1' in t and '3' in t):
                return None
            said = self.__dict__.setdefault('_reach_said', set())
            if zone not in said:                                 # (10-20)车上配置是老写法：arm_calib 调过就按新做法
                said.add(zone)
                self.log(f'    (配置里 zone_reach 不是 arm，但 {zone} 的 1、3 号环在 arm_calib 里调过：按新做法，只对准 2 号环)')
        try:
            out = {1: (float(t['1'][0]), float(t['1'][1])), 2: (0.0, 0.0), 3: (float(t['3'][0]), float(t['3'][1]))}
            out['e'] = {k: t.get('e' + str(k)) for k in (1, 3)}
            out['cal'] = str(t.get('src') or '') == 'arm_calib'
            mind = float(self.cfg.get('zone_cal_min_deg', 5.0))
            small = [k for k in (1, 3) if abs(out[k][0]) < mind]
            if small:                                            # (10-20)环距 150mm 要转二三十度：差不到 5° 多半是没调就回车了(存成 0)
                if zone not in warned:
                    warned.add(zone)
                    self.log(f'    ★★ {zone} 存的' + '、'.join(f' {k} 号环' for k in small) + ' 角度和 2 号环几乎一样(ID1 只差 ' +
                             '、'.join(f'{out[k][0]:+.1f}°' for k in small) + ')：多半是没调就回车了。这次当没调，照旧一个环一个环挪车对准；'
                             '在 arm_calib 里重调：python3 arm_calib.py --from A1P')
                return None
            if zone not in warned:
                warned.add(zone)
                if out['cal']:
                    self.log(f'    ({zone} 1、3 号环按 arm_calib 调的角度：车对准 2 号环以后不动，只转手臂过去放、取'
                             f'(比 2 号环多转 1 号 ID1 {out[1][0]:+.1f}°/ID2 {out[1][1]:+.0f}°，3 号 ID1 {out[3][0]:+.1f}°/ID2 {out[3][1]:+.0f}°)'
                             + (f'；转过去以后空爪先用摄像头只动手臂微调(最多 {float(self.cfg.get("zone_cal_fine_s", 2.0)):g} 秒)再放'
                                if self.cfg.get('zone_cal_fine', True) else '') + ')')
                else:
                    self.log(f'    ({zone} 1、3 号环按以前 rteach 示教的角度，转过去以后还用摄像头再对一次。'
                             '建议在 arm_calib 里重新调：python3 arm_calib.py --from A1P)')
            return out
        except (KeyError, TypeError, ValueError, IndexError):
            pass
        if zone in self._zone_geom_why:
            why = self._zone_geom_why[zone]
        else:
            why = self._zone_geom_missing() if self.cfg.get('zone_arm_geom', True) else 'zone_arm_geom 关着'
        if why is None:
            if zone not in warned:
                warned.add(zone)
                self.log(f'    ({zone} 没示教 1、3 号环：对准 2 号环以后按环距 {float(self.cfg.get("ring_spacing_mm") or 150.0):g}mm '
                         '算手臂转多少，只动手臂、车不动)')
            return {1: None, 2: (0.0, 0.0), 3: None, 'e': {}}
        if zone not in warned:
            warned.add(zone)
            self.log(f'    ★★ {zone} 还没在 arm_calib 里调 1、3 号环的手臂角度：这次照旧一个环一个环挪车对准(慢)。'
                     '调法：先在 run_live 里 q 退出，python3 arm_calib.py --from A1P(车停在 2 号环旁边，依次调 2 号环、1 号环、3 号环)'
                     + (f'；也没法按环距算({why})' if self.cfg.get('zone_arm_geom') else ''))
        return None

    def _zone_geom_missing(self):
        """按环距算 1、3 号环要用的对应关系有没有：有返回 None，没有返回原因。"""
        st = self.store
        if st is None or st.get('RING', 'arm') is None:
            return '没量过手臂和画面的对应关系：做一次 vcal RING arm'
        if st.get('RING', 'ch') is None:
            return '没量过车轮和画面的对应关系：做一次 vcal RING'
        return None

    def _zone_limits(self):
        """只动手臂转去 1、3 号环时 ID1、ID2 能到的范围(绝对角度)：限位(配置 a1_range 优先)往里收 zone_a1/a2_margin_deg。"""
        (l1, h1), (l2, h2) = self._arm_limits()
        a1r = self.cfg.get('a1_range')
        if a1r:
            l1, h1 = float(a1r[0]), float(a1r[1])
        m1, m2 = float(self.cfg.get('zone_a1_margin_deg', 5.0)), float(self.cfg.get('zone_a2_margin_deg', 15.0))
        return (float(l1) + m1, float(h1) - m1), (float(l2) + m2, float(h2) - m2)

    def _zone_offset(self, zone, ring):
        """ring 号环在 2 号环沿车头哪里(毫米，正 = 前方)：到工位时看清过三个环就按看到的，不然按环距(_ring_nominal)。"""
        f, f2 = self.ring_f.get((zone, int(ring))), self.ring_f.get((zone, 2))
        if f is not None and f2 is not None:
            return float(f) - float(f2)
        return float(self._ring_nominal(zone, ring))

    def _zone_geom(self, zone, ring, p2):
        """(10-19 用户：三个环的距离是知道的)没示教 1、3 号环：车对准 2 号环以后(手臂在 p2)，算只动手臂转到 ring 号环要转的 (dID1, dID2)。
        用 vcal RING 量的对应关系(都是"目标在画面里动多少像素")：手臂 J 两列 = ID2、ID1 每度，底盘 J 第二列 = 车往前 1mm。
          这个环在画面里离爪子点 = 车往前时画面动的方向(底盘J前进列) × 沿车头的距离 × 每毫米多少像素，反过来
          (车往前开这么多它才到爪子下面)；
          polar：手臂是绕转轴转(ID1) + 齿条伸缩(ID2)：爪子往外伸的方向 = ID2 那一列反过来，转轴在爪子往里 R 像素处
          (R = ID1 每度动的像素 × 180/π)；算出转到那个环要转多少度、伸缩多少。linear：按手臂 J 直接解(转得少时两种一样)。
        摄像头装在爪子上跟着手臂转，算的时候把对准 2 号环那一刻的画面当成平面图，不影响。算不出来返回 None。"""
        st = self.store
        Ja = st.get('RING', 'arm') if st is not None else None
        Jc = st.get('RING', 'ch') if st is not None else None
        if Ja is None or Jc is None:
            return None
        Ja, Jc = np.asarray(Ja, float), np.asarray(Jc, float)
        off = self._zone_offset(zone, ring)
        jf = Jc[:, 1]
        nf = float(np.linalg.norm(jf))
        if nf < 1e-6:
            return None
        # 底盘 J 只拿来定"车往前时画面往哪边动"：它的大小对准时会被一点点改(实测漂过 8%)，距离按画面比例(每毫米多少像素)算
        T = -jf / nf * float(self.vision.scale('RING')) * off   # 这个环现在在画面里离爪子点多少像素
        if str(self.cfg.get('zone_arm_model') or 'polar').lower() == 'linear':
            try:
                d = -np.linalg.solve(Ja, T)                  # [ID2, ID1]
            except np.linalg.LinAlgError:
                return None
            return float(d[1]), float(d[0])
        a, b = Ja[:, 0], Ja[:, 1]
        na, nb = float(np.linalg.norm(a)), float(np.linalg.norm(b))
        if na < 1e-6 or nb < 1e-6:
            return None
        out = 1.0 if float(self.cfg.get('zone_a2_out', 1) or 1) >= 0 else -1.0
        o = -a / na * out                                    # 爪子往外伸时在画面里往哪边走(目标往反方向动)
        t = -b                                               # ID1 正转时爪子在画面里往哪边走
        sgn = 1.0 if float(o[0] * t[1] - o[1] * t[0]) >= 0.0 else -1.0
        R = nb * 180.0 / math.pi                             # 转轴离爪子多少像素
        w = T + R * o                                        # 转轴 -> 这个环
        Rn = float(np.linalg.norm(w))
        phi = math.atan2(float(o[0] * w[1] - o[1] * w[0]), float(o @ w))
        return sgn * math.degrees(phi), (Rn - R) / na * out

    def _zone_target(self, zone, ring, p2, T):
        """只动手臂转去 ring 号环的 (ID1, ID2, 来源)：这一轮前面只动手臂对准过(第一批) > 示教 > 按环距算。超出手臂能到的范围、算不出来返回 None。"""
        learned = (self._zone_learn.get(zone) or {}).get(int(ring))
        if learned is not None:
            d1, d2, src, how = float(learned[0]), float(learned[1]), 'learn', '按第一批对准时量的角度'
        elif T.get(int(ring)) is not None:
            d1, d2 = T[int(ring)]
            f1, f2 = (self._teach_fix.get(zone) or {}).get(int(ring), (0.0, 0.0))
            d1, d2, src, how = d1 + f1, d2 + f2, 'teach', '按示教'
        else:
            g = self._zone_geom(zone, ring, p2)
            if g is None:
                self.log(f'    环{ring}：按环距算不出手臂要转多少({self._zone_geom_missing() or "对应关系不对"})：这个环照旧挪车对准')
                return None
            d1, d2 = g
            src, how = 'geom', f'按环距 {self._zone_offset(zone, ring):+.0f}mm 算'
        a1, a2 = p2[0] + d1, p2[1] + d2
        (l1, h1), (l2, h2) = self._zone_limits()
        if not (l1 <= a1 <= h1 and l2 <= a2 <= h2):
            self.log(f'    环{ring}：{how}要转到 ID1 {a1:.1f}°、ID2 {a2:.0f}°，超出手臂能到的范围(ID1 {l1:.0f}~{h1:.0f}°、'
                     f'ID2 {l2:.0f}~{h2:.0f}°)：这个环照旧挪车对准')
            return None
        self.log(f'    环{ring}：{how}只转手臂(ID1 {d1:+.1f}°、ID2 {d2:+.0f}°)，车不动')
        return a1, a2, src

    def _cal_target(self, ring, a1, a2):
        """(10-20)arm_calib 调的 ring 号环角度(加上 2 号环对准时手臂补的)在不在 ID1、ID2 的限位里(配置 a1_range 优先；调的时候手臂到过，
        不再往里收余量)。超出一点(zone_cal_over_deg 以内)：按限位放；超出更多：返回 None(这个环不放，车也不挪，物料留在车上)。"""
        (l1, h1), (l2, h2) = self._arm_limits()
        a1r = self.cfg.get('a1_range')
        if a1r:
            l1, h1 = float(a1r[0]), float(a1r[1])
        ov = self.cfg.get('zone_cal_over_deg') or {}
        o1, o2 = float(ov.get('id1', 2.0)), float(ov.get('id2', 40.0))
        b1, b2 = min(float(h1), max(float(l1), float(a1))), min(float(h2), max(float(l2), float(a2)))
        if abs(b1 - a1) > o1 or abs(b2 - a2) > o2:
            self.log(f'    ★ 环{ring}：按 arm_calib 调的角度要转到 ID1 {a1:.1f}°、ID2 {a2:.0f}°，超出限位(ID1 {l1:.0f}~{h1:.0f}°、'
                     f'ID2 {l2:.0f}~{h2:.0f}°)太多：这个环不放(车不挪，物料留在车上)。车停得太偏了？或者在 arm_calib 里重新调 1、3 号环')
            return None
        if abs(b1 - a1) > 1e-6 or abs(b2 - a2) > 1e-6:
            self.log(f'    环{ring}：要转到的角度超出限位一点(ID1 {a1 - b1:+.1f}°、ID2 {a2 - b2:+.0f}°)：按限位放')
        return b1, b2

    def _arm_only_kw(self, a1, a2):
        """(10-19)只动手臂对准(车不动)给 servo.run 的参数：手臂能偏的范围按限位(往里收余量)、最多偏 arm_limit_deg；
        只用存着的 J(按环距算还要用它)，不探测、不改。"""
        cfg = self.cfg
        (l1, h1), (l2, h2) = self._zone_limits()
        lim = (self.servo.cfg.get('arm_limit_deg') or {}) if self.servo is not None else {}
        c1, c2 = float(lim.get('id1', 12.0)), float(lim.get('id2', 250.0))
        kw = dict(wheels_only=False, filt=bool(cfg.get('zone_filter', True)), fixed_j=True,
                  arm_range={'id1': (max(l1 - a1, -c1), min(h1 - a1, c1)), 'id2': (max(l2 - a2, -c2), min(h2 - a2, c2))})
        if cfg.get('align_max_iter'):
            kw['max_iter'] = int(cfg['align_max_iter'])
        if cfg.get('zone_timeout_s'):
            kw['timeout_s'] = float(cfg['zone_timeout_s'])
        return kw

    def _cal_fine(self, zone, ring, a1, a2):
        """(10-26 用户"在 1、3 号环运动到指定位置之后再通过摄像头微调机械臂的位置，车的位置不动，对准之间必须控制在两秒左右")
        arm_calib 调过的 1、3 号环：空爪转到调的角度 (a1, a2) 上方(看圆环的高度 ZOBRNG)，摄像头只动手臂(车不动)把这个环的中心
        对到它自己的十字(claw_px.RING1 / RING3，没调过就是 2 号环的红十字)上，最多 zone_cal_fine_s 秒(不算转过去的时间)。
        返回对准以后的 (ID1, ID2)；到时间没对准但比调的角度准就返回现在的；看不到、一开始就差太多(认错环)、没变准返回 None(按调的角度放)。"""
        cfg = self.cfg
        kind = f'RING{int(ring)}'
        P = self.arm.params() or {}
        self._ring_ready = self._at_obs = False
        if P.get('ZOBRNG') is not None:
            self.arm.apz(a1, a2, float(P['ZOBRNG']))          # 边转边升降到看圆环的高度(空爪)
        else:
            self.arm.ap(a1, a2)
        self._ring_ready = True                               # 张着爪停在圆环上方的观察高度
        t0 = self.now()
        kw = self._arm_only_kw(a1, a2)
        kw['timeout_s'] = max(0.3, float(cfg.get('zone_cal_fine_s', 2.0)))
        kw['max_iter'] = int(cfg.get('zone_cal_fine_iter', 3))
        res = self.servo.run('RING', self._ring_measure(kind), self.vision.scale('RING'), cfg['tol_mm']['RING'],
                             allow_chassis=False, label=f'{zone}环{ring}微调', bounds=self.vision.bounds(kind), confirm=False, **kw)
        took = self.now() - t0
        if res is None:
            self.log(f'    ★ 环{ring} 微调：没结果({took:.1f} 秒)：按调的角度放')
            return None
        hist = list(getattr(res, 'history', None) or getattr(res, 'hist', None) or [])
        e0 = float(hist[0]) if hist else None
        cap = float(cfg.get('zone_cal_fine_max_mm', 30.0))
        if e0 is not None and e0 > cap:
            self.log(f'    ★ 环{ring} 微调：一开始就差 {e0:.1f}mm(超过 {cap:g}mm，多半认错了环)，用了 {took:.1f} 秒：不信它，按调的角度放')
            return None
        ok = bool(res.ok) or float(res.err_mm) <= float(cfg['accept_mm']['RING'])
        better = e0 is not None and math.isfinite(float(res.err_mm)) and float(res.err_mm) < e0 - 0.3
        if not (ok or better):
            self.log(f'    ★ 环{ring} 微调没对上({res})，用了 {took:.1f} 秒：按调的角度放')
            return None
        b1, b2 = self.arm.read_angles()
        if b1 is None or b2 is None:
            return None
        self.log(f'    环{ring} 微调' + ('好了' if ok else '到时间了(比调的角度准，按现在的放)') +
                 f'：还差 {float(res.err_mm):.1f}mm' + (f'(一开始差 {e0:.1f}mm)' if e0 is not None else '') +
                 f'，手臂比调的角度多转 ID1 {b1 - a1:+.2f}°、ID2 {b2 - a2:+.1f}°，用了 {took:.1f} 秒')
        return b1, b2

    def _zone_arm_fine(self, zone, ring, a1, a2, covered=None):
        """(10-19)空爪转到 ring 号环上方 (a1, a2)，只动手臂(车不动)用摄像头对准这个环；covered = 码垛：环上已经有这个颜色的物料，
        对准它(_align_covered)。返回对准以后的 (ID1, ID2)；看不到/对不准返回 None。"""
        cfg = self.cfg
        P = self.arm.params() or {}
        self._ring_ready = self._at_obs = False
        if P.get('ZOBRNG') is not None:
            self.arm.apz(a1, a2, float(P['ZOBRNG']))          # 边转边降到看圆环的高度(比放好的物料高，扫过去碰不到)
        else:
            self.arm.ap(a1, a2)
        self._ring_ready = True                               # 张着爪停在圆环上方的观察高度
        kw = self._arm_only_kw(a1, a2)
        if covered is not None:
            res, _how = self._align_covered(covered, 'STACK', f'{zone}环{ring}码垛只动手臂', confirm=False, arm_kw=kw)
            acc = cfg['accept_mm']['STACK']
        else:
            res = self.servo.run('RING', self._ring_measure(), self.vision.scale('RING'), cfg['tol_mm']['RING'], allow_chassis=False,
                                 label=f'{zone}环{ring}只动手臂', bounds=self.vision.bounds('RING'), confirm=False, **kw)
            acc = cfg['accept_mm']['RING']
        self.log(f'    环{ring} 只动手臂对准：{res}')
        if res is None or not (res.ok or res.err_mm <= acc):
            return None
        b1, b2 = self.arm.read_angles()
        if b1 is None or b2 is None:
            return None
        return b1, b2

    def _p2_align(self, zone):
        """车在 2 号环前：对准 2 号环一次，记下这时 ID1/ID2 的角度。对不准返回 None。
        (10-20 用户"工位就像原料区对圆盘那样只定位 2 号环这个点，车不要乱跑")arm_calib 调过 1、3 号环的区(cal)：只用车轮连续慢慢对准
        (_ring2_smooth，align_by 写的什么都这样)；对不上就是对不上(调用的地方这个区这次不放)，不再退回一步一步挪、更不一个环一个环挪车。
        只有连续对准做不了(STM32 不认 V、没做过 vcal RING)、或者暂存区第二批 2 号环被物料盖住认不出时，2 号环才一步一步对(也只对 2 号环)。
        没调过的区：align_by 是 wheels 时也先连续对准，不行再一步一步对准；还不行照旧一个环一个环对准。
        (10-19)对准以后车又挪过：回到 2 号环重新对准。"""
        p2 = self._p2.get(zone)
        if p2 is not None:
            at = self._p2_at.get(zone)
            d = self.act.disp
            if at is None or (abs(float(d['S']) - at[0]) < 1.0 and abs(float(d['F']) - at[1]) < 1.0):
                return p2
            self.log('    对准 2 号环以后车挪过了：回到 2 号环重新对准')
            self._p2.pop(zone, None)
        if zone in self._p2_failed:
            return None                                  # 这一趟已经没对准过：不再花时间试
        cfg = self.cfg
        T = self._teach(zone)
        cal = bool(T and T.get('cal'))
        wheels = self._by_wheels(zone)
        if cal and not wheels and zone not in self.__dict__.setdefault('_cal_wheels_said', set()):
            self._cal_wheels_said.add(zone)
            self.log(f'    (配置里 align_by.{zone} 是手臂对准，但 1、3 号环在 arm_calib 里调过：2 号环按车轮连续慢慢对准)')
        sm = None
        if (wheels or cal) and cfg.get('zone_smooth', True):
            self._obs_fixed()
            sm = self._ring2_smooth(zone)               # (10-20 用户)像原料区对圆盘一样车轮连续慢慢对准
            if sm == 'nofw':
                self.log('    (STM32 是旧程序，不认 V：2 号环一步一步挪着对准。刷新的 main.c / chassis.c 才能连续慢慢对)')
                sm = None
            elif sm == 'noj':
                sm = None
            elif sm == 'diverge':
                if cal:
                    self.log('    ★ 2 号环没对准：车轮走的方向和画面里圆环动的方向对不上(车轮和画面的对应关系不对)。这个区这次不放；'
                             '车停在工位停车点做一次 vcal RING 再试')
                    self._p2_failed.add(zone)
                    return None
                sm = None
        if isinstance(sm, dict) and not sm.get('ok') and sm.get('unseen') and self.on_ring.get((zone, 2)):
            self.log('    2 号环上放着第一批的物料(白心盖住了)，车轮连续对准时认不出圆环：改按物料顶面/圆环外圈一步一步对准')
            sm = None
        if sm is not None:
            if not sm.get('ok'):
                self.log(f'    ★ 2 号环没对准({sm.get("why") or "还差 %.1fmm" % (sm.get("err_mm") or 0.0)})：' +
                         ('这个区这次不放(调过 1、3 号环的区不再一个环一个环挪车对准)' if cal else '这个区照旧一个环一个环挪车对准'))
                self._p2_failed.add(zone)
                return None
            self.log(f'    2 号环对准结果：还差 {sm["err_mm"]:.1f}mm({sm.get("how") or "车轮"})')
            self._zone_v_only = not sm.get('sf')            # 停着算好再挪用的是 S/F：挪回停车点也用 S/F
        else:
            self._goto_ring(zone, 2)
            if wheels or cal:
                self._obs_fixed()
            else:
                self._obs_ring()
            kw = self._wheels_kw() if (wheels or cal) else self._zone_servo_kw()
            low = (self.on_ring.get((zone, 2)) or [None])[-1]
            if low is not None:                          # 暂存区第二批：2 号环上放着第一批的物料(白心被盖住)：认物料顶面/圆环外圈
                res, _how = self._align_covered(low.color, 'STACK', f'{zone}对准2号环', confirm=False, wheels=(wheels or cal))
            else:
                res = self.servo.run('RING', self._ring_measure(), self.vision.scale('RING'),
                                     cfg['tol_mm']['RING'], allow_chassis=True, label=f'{zone}对准2号环',
                                     bounds=self.vision.bounds('RING'), confirm=False, **kw)
            self.log(f'    2 号环对准结果：{res}')
            acc = self._wheels_accept('RING') if (wheels or cal) else cfg['accept_mm']['RING']
            if res is None or not (res.ok or res.err_mm <= acc):
                self.log('    ★ 2 号环对不准：' + ('这个区这次不放(调过 1、3 号环的区不再一个环一个环挪车对准)' if cal
                                                else '这个区照旧一个环一个环对准'))
                self._p2_failed.add(zone)
                return None
        a1, a2 = self.arm.read_angles()
        if a1 is None or a2 is None:
            return None
        self._p2[zone] = (a1, a2)
        self._p2_at[zone] = (float(self.act.disp['S']), float(self.act.disp['F']))
        self._anchor_mark(zone)
        self._learn_from(zone, 2)
        self.log(f'    2 号环对准了(ID1 {a1:.1f}°、ID2 {a2:.1f}°)：1、3 号环只转手臂过去(' +
                 ('按 arm_calib 调的角度' if cal else '示教的、或者按环距算的') + ')，车不再动')
        self._teach_fix[zone] = self._teach_photo(zone) if not cal else {1: (0.0, 0.0), 3: (0.0, 0.0)}
        return self._p2[zone]

    def _ring2_smooth(self, zone):
        """(10-20 用户"工位就像原料区对圆盘那样定位 2 号环这个点，车不要乱跑；找不到环不要后退，往前慢慢去找")只动车轮对准 2 号环：
        手臂停在看圆环的姿态(精确回到 A1P/A2P)不动，车轮连续慢慢走，只发 V 速度(不转车头)。
          ① 找：停着按看三个环时记下的位置找 2 号环；找不到不后退，沿车头往前慢慢挪着找(zone_seek_speed，最多 zone_seek_mm，按地图查过
             不出场地、不碰障碍物)，画面里出现圆环就停下重新看清三个环、认出哪个是 2 号；
          ② 认环：每帧按车走了多少推 2 号环在画面里哪儿，只认推算位置附近、大小和到工位时看到的三个环一样(±25%)的；车走了它在画面里
             却不动的东西(车上的：爪子、电池盒……)记下来，以后都不认；
          ③ 对：偏多少毫米就每秒走多少(越近越慢)，到容差停下，停稳拍 3 帧取平均确认，差一点再挪一点；边走边按实际走的修正车轮和画面的
             对应关系(存的那份是别的手臂角度量的，画面会整体转一点)；
          ④ 能挪的范围：横移挪到"对上 2 号环时车该在的远近"(就是 arm_calib 调 2 号环时车离圆环那一排的远近 = 比赛停车点)，
             最多比那里再靠近工位 ring_strafe_max_mm，离停车点最多 zone_s_max_mm，按地图查过不压工位；沿那一排 ±zone_f_mm(以看三个环时
             算出的 2 号环位置为中心)。车轮到头了/挪了 zone_stall_s 秒没进展：剩下的手臂补(只动手臂，车不动)。
          跟丢了(zone_lost_s 秒)：停下重新找(还找不到就往前慢慢找)；一共最多 zone_find_s 秒。
        返回 dict(ok=, err_mm=, why=, how=, unseen=)；STM32 不认 V 返回 'nofw'；没有 vcal RING 量的车轮和画面的对应关系(或者只测过前后)
        返回 'noj'；车轮走的方向和画面动的方向对不上(修不过来)返回 'diverge'。"""
        cfg, v, act, store = self.cfg, self.vision, self.act, self.store
        if act is None or not hasattr(act, 'vel') or getattr(act, '_no_vel', False):
            return 'nofw'
        self._ring_scale_seed()
        J = store.get('RING', 'ch') if store is not None else None
        sc = float(v.scale('RING'))
        if J is None or (hasattr(store, 'synth') and store.synth('RING')):
            self.log('    (还没有 vcal RING 量过的车轮和画面的对应关系(横移、前进两列)：2 号环一步一步挪着对准，'
                     '对准时会顺便测；做一次 vcal RING 以后就能连续慢慢对)')
            return 'noj'
        J0 = np.asarray(J, float)
        bad = self._raw_J_bad(J0, sc)
        vc_ = getattr(v, 'cfg', None)
        unsure = isinstance(vc_, dict) and vc_.get('_ring_scale_src') == 'J'   # 比例就是按它算的：大小没核对过，修的时候也放宽
        if bad:
            J1 = self._j_rescaled(J0, sc)
            if J1 is None:
                self.log(f'    (存的车轮和画面的对应关系不合理：{bad}：2 号环一步一步挪着对准)')
                return 'noj'
            # (10-26)只是大小不对(两列还是垂直的；多半是量的时候看圆环的高度不一样)：方向照用、大小按现在的比例，挪的时候按车实际走的再修。
            #   以前退回一步一步挪，横移只许 ring_strafe_max_mm(20mm)，车停偏了就对不上
            self.log(f'    (存的车轮和画面的对应关系大小不对：{bad}。方向照用、大小按每毫米 {sc:.2f} 像素算，挪的时候按车实际走的再修)')
            J0, unsure = J1, True
        P = self.arm.params() or {}
        if cfg.get('zone_obs_fine', True) and 'A1P' in P and 'A2P' in P:
            try:
                a1, a2 = self.arm.read_angles()
                if a1 is not None and a2 is not None and (abs(a1 - float(P['A1P'])) > 0.15 or abs(a2 - float(P['A2P'])) > 1.5):
                    self.arm.ap(float(P['A1P']), float(P['A2P']))     # 精确回到 A1P/A2P(arm_calib 调 1、3 号环时手臂就在这里)
            except ArmAbort:
                raise
            except ArmError:
                pass
        claw = np.array(v.claw('RING'), float)
        off = self._place_shift_px()
        goal0 = claw + (np.asarray(off, float) if off is not None else 0.0)
        goalp = goal0
        tol = float(cfg.get('zone_vel_tol_mm') or cfg['tol_mm']['RING'])
        acc = self._wheels_accept('RING')
        vmax = float(cfg.get('zone_vel_speed', 15.0))
        vmin = float(cfg.get('zone_vel_min_speed', 5.0))
        smax = float(cfg.get('ring_strafe_max_mm') or 0.0) if not cfg.get('chassis_strafe', False) else 1000.0
        sh_f, sh_l = getattr(self, '_cur_shift', (0.0, 0.0))
        scap = max(smax, float(cfg.get('zone_s_max_mm', 70.0))) + abs(float(sh_l))   # (10-20)规划时停车点横着挪过：多挪回来这么多
        fmax = float(cfg.get('zone_f_mm', 60.0))
        wonly = bool(cfg.get('zone_wheels_only', True))     # (10-21 用户)只用车轮修，手臂不补
        pose0 = self._stop_pose(zone)
        obs = self._map_obstacles()
        if wonly and pose0 is not None:                      # 能按地图查车身碰不碰工位/场地边：修正距离放开
            scap = max(scap, float(cfg.get('zone_wheels_s_mm', 150.0)) + abs(float(sh_l)))
            fmax = max(fmax, float(cfg.get('zone_wheels_f_mm', 150.0)))
        seek_mm = float(cfg.get('zone_seek_mm', 200.0)) + max(0.0, -float(sh_f))     # 停车点往后挪过：往前多找这么多
        seek_v = float(cfg.get('zone_seek_speed', 10.0))
        sp_px = float(cfg.get('ring_spacing_mm') or 150.0) * sc
        gate = max(40.0, 0.4 * sp_px)                         # 每帧只认推算位置这么近以内的(旁边的环隔一个间距)
        lost_s = float(cfg.get('zone_lost_s', 3.0))
        settle = float(cfg.get('zone_settle_s', 0.15) or 0.0)
        stall_s = float(cfg.get('zone_stall_s', 6.0) or 0.0)
        look_s = float(cfg.get('zone_look_s', 10.0))
        dl = [self.now() + float(cfg.get('zone_find_s', 40.0))]   # 找 + 对准的截止时刻(往前找时会延长)
        d0 = act.disp
        f2 = self.ring_f.get((zone, 2))
        fc = float(f2) if f2 is not None else float(d0['F'])
        lo, hi = np.array([-scap, fc - fmax]), np.array([scap, fc + fmax])   # 底盘位移(相对停车点)能到的范围；横移找到环以后再收
        drv = _VelDrive(self, lo, hi, float(cfg.get('raw_vel_keep_s', 0.15)), bool(cfg.get('raw_frame_thread', True)))
        if not drv.start():
            return 'nofw'
        Jc = [J0.copy()]                                      # 现在用的车轮和画面的对应关系(边走边修)
        static, seen, trk = [], [], []                        # 车上不动的假圆环；最近认到的 (像素, 拍时车走了多少)；认准的 2 号环
        note = dict(size=0, static=0, jfix_at=None, jlog=False, slip=False, jsave=unsure)
        k_lo, k_hi = (0.3, 3.0) if unsure else (0.6, 1.6)   # 按车实际走的修对应关系时，大小能改多少(大小没核对过/不对改过的放宽)
        out = dict(ok=False, err_mm=None, why='', how='车轮')
        try:
            v.live_goal = (float(goalp[0]), float(goalp[1]))
        except Exception:
            pass

        def r_ref():
            r = getattr(self, '_ring_rmed', None)            # 到工位时看到的三个环的大小(像素，最外圈半径)
            if not r and isinstance(getattr(v, 'cfg', None), dict):
                r = v.cfg.get('ring_rmax_cal')               # vclaw RING 量过的
            return float(r) if r else None

        def c0_from_survey():
            uv = (self.ring_uv or {}).get((zone, 2))
            if uv is None:
                return None
            return np.asarray(uv[:2], float) + Jc[0] @ (drv.pos() - np.asarray(uv[2:4], float))

        def look(ex, gmax):
            rp = v.ring_px(n=1, expect=(float(ex[0]), float(ex[1])), max_px=float(gmax))
            if rp is None:
                return None
            return np.asarray(rp, float), getattr(v, 'last_ring_rmax', None)

        def accept(res, pc):
            """认到的这个能不能当 2 号环：大小要和到工位时看到的三个环差不多；车走了它在画面里却不动的(车上的东西)不算。"""
            if res is None:
                return None
            rp, rr = res
            r0 = r_ref()
            if r0 and rr and not 0.75 <= float(rr) / r0 <= 1.33:
                note['size'] += 1
                return None
            if any(float(np.linalg.norm(rp - s)) <= 12.0 for s in static):
                note['static'] += 1
                return None
            pcv = np.asarray(pc, float)
            for rq, pq in seen[-30:]:
                if float(np.linalg.norm(Jc[0] @ (pcv - pq))) >= 25.0 and float(np.linalg.norm(rp - rq)) <= 8.0:
                    what = fixed_or_slip(rp)                  # 车走了它在画面里不动：车上的东西？还是车轮根本没走？
                    if what == 'fixed':
                        static.append(rp.copy())
                        note['static'] += 1
                        self.log(f'    ★ 画面 ({rp[0]:.0f},{rp[1]:.0f}) 有个像圆环的东西，车走了它在画面里一点不动(别的圆环都跟着车动了)：'
                                 '是车上的东西(爪子、电池盒……)，以后不认它')
                    elif what == 'slip':
                        note['slip'] = True
                    return None
            seen.append((rp.copy(), pcv.copy()))
            del seen[:-60]
            return rp

        def fixed_or_slip(rp):
            """认到的东西车走了它在画面里却不动：看一眼整幅画面，到工位时看到的那几个环跟着车动了 = 它是车上的东西('fixed')；
            那几个环也都没动 = 车轮根本没走起来('slip'，打滑)；看不出来返回 None(这一帧先不认它)。"""
            uv_all = [u for u in ((self.ring_uv or {}).get((zone, k)) for k in (1, 2, 3)) if u is not None]
            if not uv_all:
                return None
            try:
                rings, pc2 = drv.frame(lambda: v.ring_list(n=1))
            except VisionError:
                return None
            pts = [np.asarray(r[:2], float) for r in (rings or []) if float(np.linalg.norm(np.asarray(r[:2], float) - rp)) > 15.0]
            moved = unmoved = 0
            for u in uv_all:
                u0, pu = np.asarray(u[:2], float), np.asarray(u[2:4], float)
                shift = Jc[0] @ (drv.base + np.asarray(pc2, float) - pu)      # 从看三个环到现在，车走了这么多，环该在画面里挪这么多
                if float(np.linalg.norm(shift)) < 25.0:
                    continue
                if any(float(np.linalg.norm(q - (u0 + shift))) <= 15.0 for q in pts):
                    moved += 1
                elif any(float(np.linalg.norm(q - u0)) <= 15.0 for q in pts):
                    unmoved += 1
            if moved > 0 and moved >= unmoved:
                return 'fixed'
            if unmoved > 0:
                return 'slip'
            return None

        def jfix():
            """边走边修车轮和画面的对应关系：拿认准的 2 号环(拍时车走了多少 -> 画面里在哪)拟合一个转角 + 比例(摄像头装在手臂上，
            存的那份量的时候手臂角度/高度不一样，画面整体转一点、大小差一点)。车走了 15mm 以上才修；修不合理(转 35° 以上)不用。"""
            if len(trk) < 4:
                return False
            Pm = np.array([q[0] for q in trk], float)
            if float(np.ptp(Pm[:, 0]) + np.ptp(Pm[:, 1])) < 15.0:
                return False
            X = (J0 @ Pm.T).T
            Y = np.array([q[1] for q in trk], float)
            X, Y = X - X.mean(axis=0), Y - Y.mean(axis=0)
            sxx = float((X * X).sum())
            if sxx < 1e-6:
                return False
            a = float((X * Y).sum()) / sxx
            b = float((X[:, 0] * Y[:, 1] - X[:, 1] * Y[:, 0]).sum()) / sxx
            R = np.array([[a, -b], [b, a]])
            rms = float(np.sqrt(((Y - (R @ X.T).T) ** 2).sum(axis=1).mean()))
            k, ang = math.hypot(a, b), math.degrees(math.atan2(b, a))
            if not ((0.35 if unsure else 0.7) <= k <= (2.8 if unsure else 1.4) and abs(ang) <= 35.0) or rms > 8.0:
                return False
            Jc[0] = R @ J0
            note['jfix_at'] = (ang, k)
            if not note['jlog'] and (abs(ang) >= 3.0 or abs(k - 1.0) >= 0.05):
                note['jlog'] = True
                self.log(f'    (车轮和画面的对应关系按车实际走的修正：画面转了 {ang:+.0f}°、大小 ×{k:.2f}；存的那份可能是手臂角度不一样时量的)')
            return True

        def s_range(c_, p_):
            """横移能挪的范围：以"对上 2 号环时车该在的横移位置"(= arm_calib 调 2 号环时车离圆环那一排的远近)为中心 ±ring_strafe_max_mm，
            离停车点最多 ±zone_s_max_mm；按地图查过：车身离工位/场地边太近的那边少挪。"""
            e_ = c_ + Jc[0] @ (drv.est() - p_) - goalp
            need = drv.pos() - np.linalg.solve(Jc[0], e_)    # 对上 2 号环时车该在哪(底盘位移)
            s_t = float(need[0])
            sm_ = scap if (wonly and pose0 is not None) else smax   # (10-21)只用车轮：不再只许在"该在的远近"附近挪
            l_, h_ = max(-scap, s_t - sm_), min(scap, s_t + sm_)
            if pose0 is not None:
                s_ok = max(-scap, min(scap, s_t))
                for _k in range(60):
                    if self._body_clear(pose0, (s_ok, float(need[1])), obs)[0]:
                        break
                    s_ok = s_ok - math.copysign(min(5.0, abs(s_ok)), s_ok)
                    if abs(s_ok) < 1e-6:
                        break
                if abs(s_ok - s_t) > 0.5:                     # 横移到 s_t 车身会离工位/场地边太近：最多挪到 s_ok
                    if s_t < 0:
                        l_ = max(l_, s_ok)
                        h_ = max(h_, l_)
                    else:
                        h_ = min(h_, s_ok)
                        l_ = min(l_, h_)
            drv.lo[0], drv.hi[0] = l_, h_
            return s_t, l_, h_

        def find(guess, t_look):
            """停着找 2 号环(最多 t_look 秒)：guess = 推算的位置(没有就找离爪子点最近的那个)。连着两帧差不多才算。
            返回 (像素, 拍时车走了多少)；到时间返回 None。"""
            drv.send(0.0, 0.0)
            pc = drv.meas()
            hist, t0 = [], self.now()
            while self.now() < min(dl[0], t0 + t_look):
                self._check_abort()
                if guess is not None:
                    g = gate if self.now() - t0 < 1.5 else 0.45 * sp_px      # 推算的地方附近没有：放宽一点(还是不到半个间距)
                    rp = accept(look(guess, g), pc)
                else:
                    rp = accept(look(goalp, self.ring_gate_px or 0.6 * sp_px), pc)
                if rp is not None:
                    hist.append(rp)
                    if len(hist) >= 2 and float(np.linalg.norm(hist[-1] - hist[-2])) <= 6.0:
                        return (hist[-1] + hist[-2]) / 2.0, pc
                    guess = hist[-1]
            return None

        def creep():
            """(10-20 用户"找不到环不要后退，要往前慢慢去找")停着找不到 2 号环：沿车头往前慢慢挪(每秒 zone_seek_speed mm)，边挪边看画面里
            有没有圆环(大小和到工位时看到的一样、不是车上的东西)；看到了就停下，重新看清三个环、认出哪个是 2 号，再找它。
            往前最多 zone_seek_mm，按地图查过不出场地、不碰障碍物。找到返回 (像素, 拍时车走了多少)；没找到返回 None。"""
            if seek_mm <= 0 or seek_v <= 0:
                return None
            r0 = r_ref()

            def rings_now():
                rr_, pc_ = drv.frame(lambda: v.ring_list(n=1))
                return [r for r in (rr_ or [])
                        if (not r0 or 0.75 <= float(r[2]) / r0 <= 1.33)
                        and not any(float(np.linalg.norm(np.asarray(r[:2], float) - s)) <= 12.0 for s in static)]
            drv.send(0.0, 0.0)
            if len(rings_now()) >= 2:                         # 画面里就看得到几个环：不是没看到，是认不出 2 号环，往前挪也没用
                self.log('    画面里看得到圆环，但在该在的地方认不出 2 号环(被挡住/盖住了？)')
                return None
            f0 = float(drv.pos()[1])
            f_end = f0 + seek_mm
            drv.hi[1] = max(drv.hi[1], f_end)
            self.log(f'    ★ 停着看不到 2 号环：不后退，沿车头往前慢慢挪着找(每秒 {seek_v:g}mm，最多往前 {seek_mm:.0f}mm)')
            why = f'往前找了 {seek_mm:.0f}mm 也没看清圆环'
            t_end = self.now() + seek_mm / seek_v + 10.0      # 往前找单独算时间
            got = None
            while self.now() < t_end:
                self._check_abort()
                fpos = float(drv.pos()[1])
                if fpos >= f_end - 0.5:
                    break
                if pose0 is not None and not self._body_clear(pose0, (float(drv.pos()[0]), fpos + 20.0), obs)[0]:
                    why = f'往前 {fpos - f0:.0f}mm 再走车身会碰到东西(按地图)，没看清圆环'
                    break
                drv.send(0.0, seek_v, run_s=min(1.0, (f_end - fpos) / seek_v))
                ok_r = rings_now()
                if len(ok_r) >= 2:                            # 看到两个以上才停：从后面开过来先看到的是边上那个环，一个认不出是几号
                    got = ok_r
                    break
            if got is None:
                drv.send(0.0, 0.0)
                if not rings_now():                           # 走到头了一个环都没看到
                    self.log(f'    ★ {why}')
                    return None
            drv.finish()                                      # 停下，走过的记进 disp
            ok_r = got or rings_now()
            # 一开始一个环都看不到、往前挪着才看到：整排环都在前面，最先进画面的是这一排后面那头的环(1 号或 3 号)，
            # 从后往前第二个就是 2 号环(不按"爪子下面那个是 2 号"认：从后面开过来时爪子下面的是头上那个)
            jf = Jc[0][:, 1]                                  # 车往前 1mm，地上的东西在画面里动多少
            fwd = -jf / max(float(np.linalg.norm(jf)), 1e-9)  # 画面里"车前方"(前面的环在这个方向)
            pts = sorted((np.asarray(r[:2], float) for r in ok_r), key=lambda q: float((q - claw) @ fwd))
            pick = pts[1] if len(pts) >= 2 else pts[0]
            self.log(f'    往前挪了 {float(drv.pos()[1]) - f0:.0f}mm 看到 {len(pts)} 个圆环：从后往前第' +
                     ('二个' if len(pts) >= 2 else '一个(只看到一个，当它是 2 号)') + f'是 2 号环，画面 ({pick[0]:.0f},{pick[1]:.0f})')
            r_seen = [float(r[2]) for r in ok_r]
            if r_seen and not getattr(self, '_ring_rmed', None):
                self._ring_rmed = float(np.median(r_seen))
            a_now = drv.pos()
            d2 = -np.linalg.solve(Jc[0], pick - goalp)        # 对上它车还要走多少 [横移, 前进]
            self.ring_f[(zone, 2)] = float(a_now[1] + d2[1])
            for k_, q_ in ((2, pick),) + (((3 if (self._ring1_dir(zone) or 1.0) > 0 else 1, pts[0]),) if len(pts) >= 2 else ()):
                self.ring_uv[(zone, k_)] = (float(q_[0]), float(q_[1]), float(a_now[0]), float(a_now[1]))
            drv.lo[1] = min(float(a_now[1] + d2[1]) - fmax, float(a_now[1]))
            drv.hi[1] = max(float(a_now[1] + d2[1]) + fmax, float(a_now[1]))
            dl[0] = max(dl[0], self.now() + float(cfg.get('zone_find_s', 40.0)))   # 往前找花的时间不算进对准的时间
            return find(pick, look_s)

        def clampv(d, lim):
            n = float(np.linalg.norm(d))
            return d if n <= lim else d * (lim / n)

        calc_n = int(cfg.get('zone_wheels_moves', 8)) if wonly else int(cfg.get('zone_calc_moves', 4))
        calc_fr = int(cfg.get('zone_calc_frames', 3))
        calc_settle = float(cfg.get('zone_calc_settle_s', 0.3))
        calc_arm = float(cfg.get('zone_calc_arm_mm', 2.5))
        calc_v = float(cfg.get('zone_calc_speed', 30.0))
        calc_smooth = bool(cfg.get('zone_calc_smooth', True))

        def still_rings():
            """车停着拍几帧：画面里所有像圆环的(大小对、不是车上的东西)的中心 [(u, v)]。"""
            r0 = r_ref()
            rr_ = v.ring_list(n=calc_fr) or []
            return [np.asarray(r[:2], float) for r in rr_
                    if (not r0 or 0.75 <= float(r[2]) / r0 <= 1.33)
                    and not any(float(np.linalg.norm(np.asarray(r[:2], float) - q)) <= 12.0 for q in static)]

        def match_ring2(prev, c_prev, cur, exp):
            """车挪了一下：地上的圆环在画面里一起平移同一段。找让挪之前的圆环和现在的圆环对上最多的那个平移(对上一样多的，挑离按
            对应关系推算的位置近的)，2 号环 = 挪之前的 2 号环 + 这段平移。不靠对应关系准不准(对应关系不对时推算的位置会偏很远)。
            挪之前和现在都在画面同一个地方的(车上的东西)记下来以后不认。返回 (2 号环现在在哪, 对上了几个) 或 (None, 0)。"""
            if not cur:
                return None, 0
            best = None
            for q in cur:
                t = q - c_prev
                sc_ = sum(1 for p_ in prev if any(float(np.linalg.norm(q2 - (p_ + t))) <= 15.0 for q2 in cur))
                dev = float(np.linalg.norm(c_prev + t - exp))
                if dev > 0.9 * sp_px + float(np.linalg.norm(exp - c_prev)):
                    continue                                  # 差得比一个环距还多：不是它
                key = (-sc_, dev)
                if best is None or key < best[0]:
                    best = (key, q, sc_, t)
            if best is None:
                return None, 0
            _, q, n_, t = best
            if float(np.linalg.norm(t)) >= 25.0:              # 车真挪了：画面里没跟着动的是车上的东西
                for p_ in prev:
                    if any(float(np.linalg.norm(q2 - p_)) <= 6.0 for q2 in cur) and \
                            not any(float(np.linalg.norm(p_ - s_)) <= 12.0 for s_ in static):
                        static.append(p_.copy())
                        self.log(f'    ★ 画面 ({p_[0]:.0f},{p_[1]:.0f}) 有个像圆环的东西，车挪了它在画面里一点不动：是车上的东西，以后不认它')
            return q, n_

        def fit_j(moves):
            """按挪过的几下(车实际走了多少 -> 圆环在画面里实际动了多少)重新算车轮和画面的对应关系。
            挪过两个方向(横移、前进都有)：直接解出整个对应关系；只有一个方向：在存的那份上加一个转角 + 比例(也试横移反了/前进反了)，
            挑转得最少的。不合理(大小差太多)返回 None(照旧用原来的)。返回 (J, 说明)。"""
            Da = np.array([m[0] for m in moves], float)
            Di = np.array([m[1] for m in moves], float)
            sv0 = np.linalg.svd(J0, compute_uv=False)
            if len(moves) >= 2:
                sv = np.linalg.svd(Da, compute_uv=False)
                if sv[-1] >= 3.0 and sv[-1] >= 0.25 * sv[0]:
                    Jf = np.linalg.lstsq(Da, Di, rcond=None)[0].T
                    s1 = np.linalg.svd(Jf, compute_uv=False)
                    if k_lo * sv0[-1] <= s1[-1] and s1[0] <= k_hi * sv0[0]:
                        return Jf, '按挪过的两个方向重新算'
            best = None
            for tag, Jb in (('', J0), ('横移方向是反的', J0 @ np.diag([-1.0, 1.0])), ('前进方向是反的', J0 @ np.diag([1.0, -1.0]))):
                X = (Jb @ Da.T).T
                sxx = float((X * X).sum())
                if sxx < 1e-6:
                    continue
                a_ = float((X * Di).sum()) / sxx
                b_ = float((X[:, 0] * Di[:, 1] - X[:, 1] * Di[:, 0]).sum()) / sxx
                k_, ang = math.hypot(a_, b_), math.degrees(math.atan2(b_, a_))
                if not k_lo <= k_ <= k_hi:
                    continue
                R = np.array([[a_, -b_], [b_, a_]])
                rms = float(np.sqrt(((Di - (R @ X.T).T) ** 2).sum(axis=1).mean()))
                key = (rms > 6.0, abs(ang) + (15.0 if tag else 0.0))
                if best is None or key < best[0]:
                    best = (key, R @ Jb, (tag + '，' if tag else '') + f'画面转了 {ang:+.0f}°、大小 ×{k_:.2f}', tag, ang, k_)
            if best is None:
                return None, ''
            return best[1], best[2]

        def clamp_to(t, a):
            """要挪到的位置(底盘位移，相对停车点)限在能挪的范围里：横移 ±zone_s_max_mm，沿那一排按看三个环时算的 2 号环位置 ±zone_f_mm；
            按地图查过车身不碰工位/场地边/障碍物(碰就少挪)。返回 (位置, 被限了没有)。"""
            t2 = np.array([max(-scap, min(scap, float(t[0]))), max(float(drv.lo[1]), min(float(drv.hi[1]), float(t[1])))])
            cl = float(np.linalg.norm(t2 - t)) > 0.5
            free_f = float(cfg.get('zone_map_free_mm', 60.0))
            free_s = float(cfg.get('zone_map_free_s_mm', 10.0))

            def ok_(q):
                """离停车点这么近(沿那一排 free_f 以内、横着 free_s 以内)不按地图查；再远按地图查过车身不碰东西才行。
                横着往工位那边挪照样按地图查(压进工位/别的颜色区域这一轮就结束了)。"""
                if abs(float(q[0])) <= free_s and abs(float(q[1])) <= free_f:
                    return True
                return self._body_clear(pose0, (float(q[0]), float(q[1])), obs)[0]
            if pose0 is not None and not ok_(t2):
                for fr_ in np.linspace(0.9, 0.0, 10):
                    q = a + (t2 - a) * fr_
                    if ok_(q):
                        t2, cl = q, True
                        break
                else:
                    t2, cl = a.copy(), True
            return t2, cl

        def calc(c_f):
            """(10-21 用户"不要一开始就往前运动，要根据实际情况来计算位置然后修正")车停着看清 2 号环在哪，按车轮和画面的对应关系算出
            要挪多少，S/F 一次挪过去；停稳再看：车实际走了多少、2 号环在画面里实际动了多少，按它修正对应关系，还差就再算再挪
            (最多 zone_calc_moves 次)。挪完反而更偏(对应关系不对)也不放弃，按实际的改过来再挪。
            返回 (对准了, 到范围边上了, 跟丢了, 车轮没走起来, 还差多少 mm, 剩下太少车轮挪不准)。"""
            drv.finish()                                      # 不再用 V：V 走过的记进 act.disp
            a = np.array([float(act.disp['S']), float(act.disp['F'])])
            pts = still_rings()
            near = [q for q in pts if float(np.linalg.norm(q - c_f)) <= 15.0]
            c_ = min(near, key=lambda q: float(np.linalg.norm(q - c_f))) if near else np.asarray(c_f, float)
            if not any(float(np.linalg.norm(q - c_)) <= 1e-6 for q in pts):
                pts = pts + [c_]
            moves = []
            J = Jc[0]
            em_ = float(np.linalg.norm(c_ - goalp)) / sc
            for it in range(calc_n + 1):
                e_ = c_ - goalp
                em_ = float(np.linalg.norm(e_)) / sc
                if em_ <= tol:
                    return True, False, False, False, em_, False
                if it > 0 and em_ <= calc_arm and not wonly:
                    return False, False, False, False, em_, True      # 只差一点：车轮小挪动不准，剩下的手臂补
                if it >= calc_n:
                    break
                try:
                    d = -np.linalg.solve(J, e_)
                except np.linalg.LinAlgError:
                    out['why'] = '车轮和画面的对应关系算不出来'
                    break
                if it == 0 and abs(float(a[0] + d[0])) > smax + 5.0:
                    self.log(f'    ★ 车离圆环那一排的远近和调 2 号环的时候差了约 {abs(float(a[0] + d[0])):.0f}mm(车停偏了)：车轮横着挪过去'
                             + (f'(最多 {scap:.0f}mm)' if abs(float(a[0] + d[0])) > scap else ''))
                tgt, cl = clamp_to(a + d, a)
                da = np.round(tgt - a)
                if float(np.linalg.norm(da)) < 1.0:
                    if cl:
                        self.log(f'    2 号环还偏 {em_:.1f}mm：车轮到能挪的范围边上了')
                        return False, True, False, False, em_, False
                    return False, False, False, False, em_, not wonly  # 剩下不到 1mm 车轮挪不准：手臂补(只用车轮时就到这里)
                self.log(f'    {"" if it == 0 else "再"}算一次：2 号环偏 {em_:.1f}mm，车停着算好要 横移 {d[0]:+.0f}mm、前进 {d[1]:+.0f}mm'
                         + (f'(能挪的范围限成 横移 {da[0]:+.0f}、前进 {da[1]:+.0f})' if cl else '') +
                         ('，慢慢走过去' if calc_smooth and not note.get('no_v') else '，一次挪过去'))
                went = False
                if calc_smooth and not note.get('no_v'):
                    went = self._vel_goto(float(a[0] + da[0]), float(a[1] + da[1]), tol_mm=0.6, vmax=calc_v)
                    if not went:
                        note['no_v'] = True
                        self.log(f'    (V 慢慢走不了：{self._vel_why or "没走到"}：改用 S/F)')
                        a = np.array([float(act.disp['S']), float(act.disp['F'])])
                        da = np.round(tgt - a)
                if not went:
                    note['sf'] = True
                    act.chassis_move(float(da[0]), float(da[1]))
                self.sleep(calc_settle)
                a_new = np.array([float(act.disp['S']), float(act.disp['F'])])
                mv = a_new - a
                exp = c_ + J @ mv
                cur = still_rings()
                c_new, n_ok = match_ring2(pts, c_, cur, exp)
                if c_new is None:
                    res = look(exp, 0.45 * sp_px)
                    c_new = res[0] if res is not None else None
                if c_new is None:
                    out['why'] = f'挪了 横移 {mv[0]:+.0f}、前进 {mv[1]:+.0f}mm 以后画面里认不出 2 号环了'
                    self.log(f'    ★ {out["why"]}')
                    return False, False, True, False, em_, False
                dimg = c_new - c_
                ex_px = float(np.linalg.norm(J @ mv))
                if ex_px >= 6.0 and float(np.linalg.norm(dimg)) < max(2.0, 0.2 * ex_px):
                    try:                                       # (10-21)车其实没走(打滑)：记的位移按画面里实际动了多少改回来，
                        seen_ = np.linalg.solve(J, dimg)       #   不然做完"挪回停车点"会差出没走的那一段
                        act.disp['S'], act.disp['F'] = float(a[0] + seen_[0]), float(a[1] + seen_[1])
                        a_new = np.array([float(act.disp['S']), float(act.disp['F'])])
                    except np.linalg.LinAlgError:
                        pass
                    if wonly and went:                               # 只用车轮：V 慢慢走没走起来，改用 S/F(按位置走)再挪一次，不交给手臂
                        note['no_v'] = True
                        self.log('    ★ 车挪了，画面里的圆环一点没动：V 慢慢走车轮好像没走起来(打滑？)，改用 S/F 再挪一次')
                        a = a_new
                        continue
                    if wonly and note.get('slips', 0) < 1:
                        note['slips'] = note.get('slips', 0) + 1
                        self.log('    ★ 车挪了，画面里的圆环一点没动：车轮好像没走起来(打滑？)，按画面再算一次再挪')
                        a = a_new
                        continue
                    self.log('    ★ 车挪了，画面里的圆环一点没动：车轮好像没走起来(打滑？)')
                    return False, False, False, True, em_, False
                if float(np.linalg.norm(mv)) >= float(cfg.get('zone_fit_min_mm', 8.0)):
                    moves.append((mv.copy(), dimg.copy()))     # (10-21)只按挪得够多的修对应关系：挪一两毫米时认圆环差一两个像素，
                pred_err = float(np.linalg.norm(exp - c_new)) / sc   # 算出来的转角可以是任何数(以前算出过"画面转了 +98°"还存了下来)
                Jn, how_ = fit_j(moves) if moves else (None, '')
                if Jn is not None:
                    J = Jn
                    Jc[0] = J
                    if pred_err >= 3.0 or len(moves) >= 2:
                        note['jsave'] = True
                    if pred_err >= 3.0:
                        self.log(f'    这一下挪完 2 号环和按对应关系推算的差了 {pred_err:.0f}mm：按车实际走的修正对应关系({how_})')
                c_, a, pts = c_new, a_new, (cur if cur else [c_new])
            return False, False, False, False, em_, False

        def tail(done, edge, lost, stalled, em, d_, small=False, calc_mode=False):
            """车轮对完了：记下还差多少；对准了就把修正过的对应关系存下来；没对准(到边上/没进展/到时间/剩下太少)剩下的只动手臂补。"""
            out['err_mm'] = em
            rest_ = '手臂不补' if wonly else '剩下的手臂补'
            if stalled:
                why_ = (f'(★ 车挪了画面里的圆环没动：打滑/没走起来？{rest_})' if calc_mode
                        else f'(★ 车轮挪了 {stall_s:g} 秒偏差都没小下去：打滑/没走起来？{rest_})')
            elif small:
                why_ = '(只差一点，车轮小挪动不准：剩下的手臂补)'
            else:
                why_ = f'(横移/前后到头了，{rest_})' if edge else ('' if lost else
                       (f'(挪了 {calc_n} 次还差一点，{rest_})' if calc_mode else f'(到时间了，{rest_})'))
            self.log(f'    车轮停在 横移 {d_[0]:+.0f}mm、前进 {d_[1]:+.0f}mm(相对停车点)：2 号环还偏 {em:.1f}mm' + ('' if done else why_))
            ja = note['jfix_at']
            if store is not None and ((note.get('jsave') and em <= max(5.0, 3.0 * tol))
                                      or (done and ja is not None and (abs(ja[0]) >= 3.0 or abs(ja[1] - 1.0) >= 0.05))):
                store.put('RING', 'ch', Jc[0])                # 修正过的对应关系存下来(下次到工位直接用)
                if hasattr(store, 'set_synth'):
                    store.set_synth('RING', False)
                if hasattr(store, 'save'):
                    store.save()
            if done:
                out['ok'] = True
                return out
            if lost:
                return out
            if wonly:                                         # (10-21 用户)只用车轮：手臂一下都不动，还差不到 accept_mm 照样放
                out['ok'] = em <= acc
                if not out['ok']:
                    out['why'] = f'只用车轮对准还差 {em:.1f}mm(超过 {acc:g}mm，手臂不补)'
                return out
            # 车轮到头了/到时间了/没进展：只动手臂补剩下的(车不动)
            if store.get('RING', 'arm') is None:
                out['ok'] = em <= acc
                out['why'] = '' if out['ok'] else f'车轮对不上还差 {em:.1f}mm，又没量过手臂和画面的对应关系(vcal RING arm)补不了'
                return out
            a1, a2 = self.arm.read_angles()
            if a1 is None or a2 is None:
                out['why'] = '读不到手臂角度'
                return out
            res = self.servo.run('RING', self._ring_measure(), sc, cfg['tol_mm']['RING'], allow_chassis=False,
                                 label=f'{zone}2号环手臂补', bounds=v.bounds('RING'), confirm=False, **self._arm_only_kw(a1, a2))
            self.log(f'    手臂补(车不动)：{res}')
            self._at_obs = False
            out['how'] = '车轮 + 手臂补'
            if res is not None and math.isfinite(res.err_mm):
                out['err_mm'] = res.err_mm
            out['ok'] = res is not None and (res.ok or res.err_mm <= acc)
            if not out['ok']:
                out['why'] = f'车轮对不上，手臂补也没对上({res.reason if res is not None else "看不到圆环"})'
            else:
                try:
                    _b1, b2 = self.arm.read_angles()
                except ArmAbort:
                    raise
                except ArmError:
                    b2 = None
                if b2 is not None and abs(b2 - float(P.get('A2P', b2))) > 25.0:
                    self.log(f'    ★ 手臂伸缩补了 ID2 {b2 - float(P["A2P"]):+.0f}°(车离圆环那一排的远近和调 2 号环时不一样)：'
                             '1、3 号环可能偏几毫米；停车点离圆环那一排的远近要和 arm_calib 调的时候一样')
            return out
        try:
            g0 = c0_from_survey()
            f = find(g0, look_s if g0 is not None else min(look_s, 3.0))   # 看三个环时没看到 2 号环：不干等，早点往前找
            if f is None:
                f = creep()
            if f is None:
                drv.finish()
                out['why'], out['unseen'] = '停着找、往前慢慢找都没认出 2 号环', True
                return out
            c, p_c = f
            if cfg.get('zone_calc', True):
                # (10-21 用户"不要一开始就往前运动，要根据实际情况来计算位置然后修正")停着算好、一次挪过去、停稳再看、按实际修正
                dn, ed, lo_, st_, em, sm_ = calc(c)
                out['sf'] = bool(note.get('sf'))
                out['how'] = '车轮(停着算好再挪)'
                return tail(dn, ed, lo_, st_, em, np.array([float(act.disp['S']), float(act.disp['F'])]), small=sm_, calc_mode=True)
            trk.append((np.asarray(p_c, float).copy(), np.asarray(c, float).copy()))
            e0 = float(np.linalg.norm(c - goalp)) / sc
            s_t, l_s, h_s = s_range(c, p_c)
            self.log(f'    车轮边走边对准 2 号环(只发 V 速度，不转车头；最快 {vmax:g}mm/s，越近越慢)：现在偏 {e0:.1f}mm；'
                     f'横移能挪 {l_s:+.0f}~{h_s:+.0f}mm(对上时车该在 {s_t:+.0f}mm：和 arm_calib 调 2 号环时离圆环那一排一样远)、'
                     f'沿那一排 {drv.lo[1]:+.0f}~{drv.hi[1]:+.0f}mm(相对停车点)')
            if abs(s_t) > smax + 5.0:
                self.log(f'    ★ 车离圆环那一排的远近和调 2 号环的时候差了约 {abs(s_t):.0f}mm(车停偏了)：车轮横着挪过去'
                         + (f'(最多 {scap:.0f}mm)' if abs(s_t) > scap else ''))
            t_meas, rest, done, edge, lost, stalled = self.now(), [], False, False, False, False
            best, t_best = e0, self.now()                     # 偏差最小到过多少、什么时候(车轮没走起来就别一直磨)
            t_log, frames, seen_n = self.now(), 0, 0
            e_start, t_start, em = e0, self.now(), e0
            p_fix = np.asarray(p_c, float).copy()             # 上次修对应关系时车在哪
            while self.now() < dl[0]:
                self._check_abort()
                still = float(np.linalg.norm(drv.cmd)) < 1e-9 and self.now() >= drv.t_stop + settle   # 停稳了
                pred = c + Jc[0] @ (drv.est() - p_c)
                res, pc = drv.frame(lambda: look(pred, gate))
                rp = accept(res, pc)
                frames += 1
                if note['slip']:
                    self.log('    ★ 车轮好像没走起来(打滑？)：车按 V 走了，画面里的圆环一个都没动')
                    stalled = True
                    break
                still = still and float(np.linalg.norm(drv.cmd)) < 1e-9
                cp = c + Jc[0] @ (pc - p_c)                   # 拍这一帧时 2 号环按车走了多少应该在这里
                if rp is not None:
                    t_meas = self.now()
                    seen_n += 1
                    trk.append((np.asarray(pc, float).copy(), rp.copy()))
                    del trk[:-40]
                    if still:
                        rest.append(rp)                       # 停着拍的：几帧取平均(去掉一帧一帧的抖动)
                        c = np.mean(rest[-3:], axis=0)
                    else:
                        rest = []
                        c = 0.3 * cp + 0.7 * rp
                    if float(np.linalg.norm(np.asarray(pc, float) - p_fix)) >= 10.0 and jfix():
                        p_fix = np.asarray(pc, float).copy()
                        s_t, l_s, h_s = s_range(c, pc)
                else:
                    c = cp
                    if not still:
                        rest = []
                p_c = pc
                if self.now() - t_meas > lost_s:
                    self.log(f'    ★ 连着 {lost_s:g} 秒认不出 2 号环(跟丢了)：停下重新找')
                    f = find(c + Jc[0] @ (drv.est() - p_c), 2.0 * lost_s)
                    if f is None:
                        f = creep()
                    if f is None:
                        out['why'], lost = '跟丢了，重新找、往前慢慢找都没找到', True
                        break
                    c, p_c = f
                    t_meas, rest = self.now(), []
                    e_start, t_start = float(np.linalg.norm(c - goalp)) / sc, self.now()
                    best, t_best = e_start, self.now()
                    s_t, l_s, h_s = s_range(c, p_c)
                    continue
                e = c + Jc[0] @ (drv.est() - p_c) - goalp
                em = float(np.linalg.norm(e)) / sc
                if rp is not None and em < best - 0.5:
                    best, t_best = em, self.now()
                if stall_s > 0 and self.now() - t_best > stall_s:
                    stalled = True                            # 挪了这么久偏差都没小下去：剩下的手臂补
                    break
                d = -np.linalg.solve(Jc[0], e)                # 还要走多少 [横移, 前进] 毫米
                a_now = drv.pos()
                dr = np.minimum(np.maximum(a_now + d, drv.lo), drv.hi) - a_now   # 能走的那一部分(横移、沿那一排都限着)
                if still and len(rest) >= 3:
                    if em <= tol:
                        done = True                           # 停着 3 帧平均在容差内：对准了
                        break
                    if float(np.linalg.norm(dr)) < 0.6:
                        edge = True                           # 到能挪的范围边上了还差：剩下的手臂补
                        break
                if em <= tol or float(np.linalg.norm(dr)) < 0.6:
                    if float(np.linalg.norm(drv.cmd)) > 1e-9:
                        drv.send(0.0, 0.0)                    # 到了：停下，停着拍几帧确认
                        rest = []
                    elif not still:
                        rest = []
                elif still and len(rest) < 3 and em < max(3.0 * tol, 4.0):
                    pass                                      # 已经很近、停着：先拍够 3 帧取平均再决定挪不挪(不按一帧的抖动挪来挪去)
                else:
                    vv = clampv(dr * 1.0, vmax)               # 偏多少毫米就每秒走多少(越近越慢)
                    nv = float(np.linalg.norm(vv))
                    if 1e-6 < nv < vmin:
                        vv, nv = vv * (vmin / nv), vmin
                    drv.send(float(vv[0]), float(vv[1]), run_s=max(0.1, float(np.linalg.norm(dr)) / max(nv, 1e-6)))
                    if still:
                        rest = []
                if self.now() - t_start > 3.0 and em > e_start + 8.0 and not jfix():
                    self.log(f'    ★ 越走越偏({e_start:.0f}mm → {em:.0f}mm)，按车实际走的也修不过来：车轮和画面的对应关系不对')
                    drv.finish()
                    return 'diverge'
                if self.now() - t_log >= 1.0:
                    dt = self.now() - t_log
                    self.log(f'      {frames / dt:.1f} 帧/秒(认出 {seen_n}' +
                             (f'，大小不对 {note["size"]}' if note['size'] else '') +
                             (f'，车上的东西 {note["static"]}' if note['static'] else '') +
                             f')  2 号环还偏 {em:.1f}mm  车轮 横移 {drv.cmd[0]:+.0f}、前进 {drv.cmd[1]:+.0f}mm/s  {drv.timing()}')
                    t_log, frames, seen_n = self.now(), 0, 0
                    note['size'] = note['static'] = 0
            drv.finish()
            c = c + Jc[0] @ (drv.p - p_c)
            em = float(np.linalg.norm(c - goalp)) / sc
            return tail(done, edge, lost, stalled, em, drv.pos())
        except BaseException:
            try:
                drv.finish()
            except Exception:
                pass
            raise
        finally:
            try:
                v.live_goal = None
            except Exception:
                pass

    def _teach_photo(self, zone):
        """对准 2 号环以后拍一张：1、3 号环在画面里和示教时差多少像素，换成 ID1/ID2 要多转多少度(补车停偏、车身斜)。"""
        T = self._teach(zone)
        J = self.store.get('RING', 'arm') if self.store is not None else None
        fix = {1: (0.0, 0.0), 3: (0.0, 0.0)}
        if T is None or J is None or not hasattr(self.vision, 'ring_list') or not self.cfg.get('teach_photo_fix', True):
            return fix
        try:
            cu, cv_ = self.vision.claw('RING')
            seen = [(r[0] - cu, r[1] - cv_) for r in (self.vision.ring_list(n=self.cfg.get('survey_frames') or None) or [])]
        except Exception:
            return fix
        lim1 = float(self.cfg.get('teach_fix_max_deg', 3.0))
        lim2 = float(self.cfg.get('teach_fix_max_deg2', 60.0))
        Jm = np.asarray(J, float)
        for k in (1, 3):
            et = T['e'].get(k)
            if not et or not seen:
                continue
            ek = min(seen, key=lambda e: math.hypot(e[0] - et[0], e[1] - et[1]))
            if math.hypot(ek[0] - et[0], ek[1] - et[1]) > float(self.cfg.get('teach_fix_max_px', 80.0)):
                continue                                  # 差太多：多半认错了环，不补
            try:
                du = -np.linalg.solve(Jm, np.array([ek[0] - et[0], ek[1] - et[1]]))     # [ID2, ID1]
            except np.linalg.LinAlgError:
                continue
            d2 = max(-lim2, min(lim2, float(du[0])))
            d1 = max(-lim1, min(lim1, float(du[1])))
            fix[k] = (d1, d2)
            self.log(f'    环{k} 和示教时比偏了 {ek[0] - et[0]:+.0f},{ek[1] - et[1]:+.0f} 像素：多转 ID1 {d1:+.2f}°、ID2 {d2:+.1f}°')
        return fix

    def _stack_carry(self, zone, ring, p2, T, a1t, a2t):
        """(10-23 用户"先放好三环我再去放1环的时候容易把三环放好的东西给撞掉")码垛：夹着物料从车上转盘转去 ring 号环，路上要经过
        已经叠了两层的环：升降到最高，夹着的物料底也只比两层垛顶高一点，照旧按目标环的长度一边转一边扫过去会擦到。
        (10-24 用户"往里面缩去躲避然后到位置了再伸过去不要一会缩一会伸")stack_carry_mode='in'(默认)：绕过电池盒在 stack_carry_a1 停下、
        往里缩到 stack_carry_a2，缩着转到 ring 号环的角度，再只伸出去(见 stack_in_plan；两个值 arm_calib 的 STKA1/STKA2 调)，一路在搬运高度 ZHI。
        'plan' = 10-23 按三个环的几何算的走法(见 stack_carry_plan)。
        (10-25 用户"不管是先放哪个环……在经过环3之前就缩回到指定位置然后到指定的环的位置然后再伸长出来")'in' 默认第二层每一个都这样走，
        路上有没有两层的垛都一样(stack_carry_always=False 才回到以前：只有路上有两层的垛才躲)。
        走到了 (a1t, a2t) 返回 True；不用躲(always 关着、路上没有两层的环)、或者算不出各个环在哪，返回 False(调用的人照旧走)。"""
        cfg = self.cfg
        if not cfg.get('stack_carry', True):
            return False
        tall = [r for r in (1, 2, 3) if r != int(ring) and len(self.on_ring.get((zone, r)) or []) >= 2]
        always = str(cfg.get('stack_carry_mode') or 'in') != 'plan' and bool(cfg.get('stack_carry_always', True))
        if (not tall and not always) or not T:
            return False
        try:
            poses = {2: (float(p2[0]), float(p2[1]))}
            for r in (1, 3):
                poses[r] = (float(p2[0]) + float(T[r][0]), float(p2[1]) + float(T[r][1]))
        except (KeyError, TypeError, ValueError, IndexError):
            return False
        P = self.arm.params() or {}

        def num(v):
            try:
                v = float(v)
            except (TypeError, ValueError):
                return None
            return v if math.isfinite(v) else None
        start, a2_in = num(P.get('A1D')), num(cfg.get('stack_carry_a2'))
        if a2_in is None:
            a2_in = num(P.get('A2R'))
        if start is None or a2_in is None:
            return False
        lim_lo, lim_hi = num(P.get('S2MIN')), num(P.get('S2MAX'))
        a2_lim = (None if lim_lo is None else lim_lo + 20.0, None if lim_hi is None else lim_hi - 20.0)
        clear = float(cfg.get('stack_carry_clear_mm', 50.0))
        if str(cfg.get('stack_carry_mode') or 'in') != 'plan':
            return self._stack_in(ring, poses, tall, P, num, start, a2_in, a2_lim, clear, a1t, a2t, always=always)
        plan = stack_carry_plan(start, (a1t, a2t), poses, tall, float(cfg.get('ring_spacing_mm', 150.0)),
                                clear, a2_in, a2_lim=a2_lim,
                                pre_deg=float(cfg.get('stack_carry_pre_deg', 20.0)), outer=bool(cfg.get('stack_carry_outer', True)),
                                avoid_a1=num(P.get('R1T1')))
        if plan is None:
            return False
        how = '、'.join(f'环{r}({side}' + ('' if gap is None else f'，离它中心 {gap:.0f}mm') + ')' for r, side, gap in plan['dodge'])
        self.log(f'    码垛：去环{ring} 路上要经过已经叠了两层的 {how}：先缩着转，在两个环中间改伸缩长度，最后伸到环{ring} 再转过去')
        if plan['fit'] is None:
            self.log('    ★ 按三个环的角度算不出手臂的几何(或者对不上一条直线)：只缩着转，不知道够不够远')
        else:
            a, k, res = plan['fit']
            near = [f'环{r} {gap:.0f}mm' for r, _s, gap in plan['dodge'] if gap is not None and gap < clear - 0.5]
            if near:
                self.log(f'    ★ 离两层垛还是不够远({"、".join(near)}，要 {clear:.0f}mm)：伸缩到头了，可能还会擦到')
            self.log(f'      (按三个环的角度算：爪子离转轴 = {a:.0f} + {k:.4f}×ID2 毫米，对不上 {res:.1f}mm)')
        wps = plan['wps']
        z = None
        if cfg.get('return_lower', True):
            z = num(P.get('ZOBRNG'))                         # 和 _return_to_pose 一样：从车上转盘转出来时边转边到看圆环的高度
        if z is not None:
            self.arm.apz(wps[0][0], wps[0][1], z)            # 第一段从车上转盘出发：STM32 照旧按圆环那边的参数绕电池盒
        else:
            self.arm.ap(wps[0][0], wps[0][1])
        for w1, w2 in wps[1:]:
            self.arm.ap(w1, w2)
        self._return_to_pose(a1t, a2t, from_tray=False)
        return True

    def _stack_in(self, ring, poses, tall, P, num, start, a2_in, a2_lim, clear, a1t, a2t, always=False):
        """_stack_carry 的 'in' 走法(10-24)：绕过电池盒 -> 停下往里缩 -> 缩着转到 ring 号环的角度 -> 只伸出去。
        always：(10-25)路上没有两层的垛也这样走，在路上第一个环(一般是 3 号环)前面缩。"""
        cfg = self.cfg
        ring_set = float(P.get('AEXT', 2.0) or 2.0) == 2.0 and abs(num(P.get('R2T')) or 0.0) > 0.5
        after = num(P.get('R1T1')) if ring_set else None             # 圆环那组绕电池盒：转过 R1T1 就过了电池盒
        if after is not None and abs(after) <= 0.5:
            after = None
        want = num(cfg.get('stack_carry_a1'))
        hold = num(P.get('R2T')) if ring_set else num(P.get('A2R'))
        fit = fit_arm_polar(poses, float(cfg.get('ring_spacing_mm', 150.0)))
        if fit is not None and fit[2] > 6.0:
            fit = None                                          # 三个环的角度对不上一条直线：不算离垛多远
        plan = stack_in_plan(start, (a1t, a2t), poses, tall, a2_in, a1_in=want if want is not None else after, after=after,
                             hold=hold, fit=fit, a2_lim=a2_lim, a2_start=num(P.get('A2R')), always=always)
        if plan is None:
            return False
        a1, a2 = plan['a1_in'], plan['a2_in']
        how = f'绕过电池盒转到 ID1 {a1:.1f}° 停下往里缩到 ID2 {a2:.0f}°，缩着转到环{ring}，再伸出去'
        if plan['cross']:
            self.log(f'    码垛：去环{ring} 路上要经过已经叠了两层的' + '、'.join(f'环{r}' for r in plan['cross']) + '：' + how)
        else:
            self.log(f'    码垛(第二层一律这样走)：去环{ring}，' + how)
        if plan['moved']:
            self.log(f'    ★ 往里缩的地方 stack_carry_a1={want:.1f}°(arm_calib 的 STKA1)不在' + ('绕过电池盒(R1T1)' if after is not None else '车上转盘') +
                     f'和环{plan.get("first_ring") or ring} 中间：这次在 {a1:.1f}° 缩')
        near = [f'环{r} {g:.0f}mm' for r, g in plan['gaps'].items() if g < clear - 0.5]
        if near:
            zhi, zstk = num(P.get('ZHI')), num(P.get('ZSTK'))
            over = None if zhi is None or zstk is None else zhi - zstk - float(cfg.get('stack_material_mm', 60.0))
            up = '' if over is None else f'，按 ZHI-ZSTK-{float(cfg.get("stack_material_mm", 60.0)):g} 算夹着的物料底比两层垛顶高 {over:.0f}mm'
            self.log(('    ★' if over is not None and over < 3.0 else '     ') +
                     f' (按三个环的角度算，缩着转过去离{"、".join(near)}(中心到中心，要 {clear:.0f}mm)：物料要从垛顶上面过{up}。'
                     '擦到了就 arm_calib --from STKA2 再缩短一点，或者升降调高)')
        self._dodge = (a1, a2)                                  # 路上出错要放回转盘：先往里缩、缩着转回来(_to_tray_detour)
        wps = plan['wps']
        z = num(P.get('ZHI'))
        if abs(wps[0][0] - start) > 1.0 and z is not None:
            self.arm.apz(wps[0][0], wps[0][1], z)               # 从车上转盘出发：STM32 照旧按圆环那组绕电池盒，停在 a1_in(还伸在 R2T)
        else:
            self.arm.ap(wps[0][0], wps[0][1])
        for w1, w2 in wps[1:]:
            self.arm.ap(w1, w2)                                 # 只往里缩；缩着只转
        self._return_to_pose(a1t, a2t, from_tray=False)          # 到了目标环的角度：只伸出去(再回读、差得多再转一次)
        return True

    def _place_item_arm(self, item, zone, ring, stack, label, T):
        """zone_reach='arm'：车只在 2 号环前对准一次，1、3 号环按示教/环距算的角度只转手臂过去；(10-19)转过去以后空爪只动手臂
        用摄像头对准这个环再放(zone_arm_fine；码垛时下面的环被物料盖着，按第一批对准时量的角度放)。
        返回 True/False；不能只动手臂放(2 号环对不准、这个环算不出来/超出手臂能到的范围、按环距算的又看不到环)返回 None(调用的人照旧挪车对准)。"""
        key = 'STACK' if stack else 'RING'
        cal = bool(T.get('cal'))
        try:
            p2 = self._p2_align(zone)
        except ArmAbort:
            raise
        except (ArmError, VisionError) as ex:
            self.log(f'    ★ 对准 2 号环出错：{ex}' + ('' if cal else '：照旧一个环一个环对准'))
            self._p2_failed.add(zone)
            p2 = None
        if p2 is None:
            if not cal:
                return None
            # (10-20 用户"车不要乱跑")调过 1、3 号环的区：2 号环没对准就不放，不再一个环一个环挪车对准
            self.log(f'    ★ {zone} 2 号环没对准：{item.color_name}这次不放(留在车上)')
            self.stats.place(False)
            self._show_stats()
            return False
        ring = int(ring)
        if ring == 2:
            a1, a2 = p2
            self.log('    环2：就在对准的这个环，不再对准')
        elif T.get('cal'):
            # (10-20 用户)arm_calib 调过：直接按调的角度转过去(放置补偿默认不加，见 place_comp_on)，绝不挪车
            fine_on = bool(self.cfg.get('zone_cal_fine', True)) and not self.nogo
            learned = (self._zone_learn.get(zone) or {}).get(ring) if fine_on else None
            d1, d2 = learned if learned is not None else T[ring]
            tgt = self._cal_target(ring, p2[0] + d1, p2[1] + d2)
            if tgt is None:
                self.stats.place(False)
                self._show_stats()
                return False
            a1, a2 = tgt
            if learned is not None:
                self.log(f'    环{ring}：按这个区第一次用摄像头微调好的角度只转手臂(比 2 号环多转 ID1 {d1:+.1f}°、ID2 {d2:+.0f}°)，车不动、不再对')
            elif fine_on and not stack:
                # (10-26 用户)空爪先转过去，摄像头只动手臂对准这个环自己的十字(最多 zone_cal_fine_s 秒)，再去夹物料回来放
                self.log(f'    环{ring}：按 arm_calib 调的角度只转手臂(比 2 号环多转 ID1 {d1:+.1f}°、ID2 {d2:+.0f}°)，车不动；'
                         '空爪先用摄像头只动手臂微调')
                try:
                    fine = self._cal_fine(zone, ring, a1, a2)
                except ArmAbort:
                    raise
                except (ArmError, VisionError) as ex:
                    self.log(f'    ★ 环{ring} 微调出错：{ex}：按调的角度放')
                    fine = None
                if fine is not None:
                    a1, a2 = fine
                    self._zone_learn.setdefault(zone, {})[ring] = (a1 - p2[0], a2 - p2[1])   # 这个区第二轮(码垛也是)直接用
            else:
                self.log(f'    环{ring}：按 arm_calib 调的角度只转手臂(比 2 号环多转 ID1 {d1:+.1f}°、ID2 {d2:+.0f}°)，车不动、不再用摄像头对')
        else:
            try:
                tgt = self._zone_target(zone, ring, p2, T)
                if tgt is None:
                    return None
                a1, a2, src = tgt
                if self.cfg.get('zone_arm_fine', True) and not self.nogo:
                    fine = self._zone_arm_fine(zone, ring, a1, a2, covered=item.color if stack else None)
                    if fine is not None:
                        a1, a2 = fine
                        if not stack:                       # 码垛对准的是下面物料的顶面(和圆环不是同一个点)：不当成这个环的角度记
                            self._zone_learn.setdefault(zone, {})[ring] = (a1 - p2[0], a2 - p2[1])
                    elif src == 'geom':
                        self.log(f'    ★ 环{ring} 按环距算的位置上只动手臂没对准：这个环照旧挪车对准')
                        return None
                    else:
                        self.log(f'    ★ 环{ring} 只动手臂没对准：按{"示教" if src == "teach" else "第一批对准时"}的角度放')
            except ArmAbort:
                raise
            except (ArmError, VisionError) as ex:
                self.log(f'    ★ 只动手臂转到环{ring} 出错：{ex}：这个环照旧挪车对准')
                return None
        d = self.act.disp
        self.pose_at[(zone, item.slot)] = dict(S=d['S'], F=d['F'], a1=a1, a2=a2, arm=True)   # arm：车没动、只转手臂放的
        if self.nogo:
            self.arm.ap(a1, a2)
            self.log('    (nogo：转到了，不取物料、不放)')
            return True
        ok, held, dropped = False, False, None
        self._dodge = None
        try:
            self._ring_ready = self._at_obs = False
            self._take(item)
            held = True
            self.holding = item
            adj = self.place_adj(key)
            ta1, ta2 = a1 + (adj[0] if adj else 0.0), a2 + (adj[1] if adj else 0.0)
            if not (stack and self._stack_carry(zone, ring, p2, T, ta1, ta2)):
                self._return_to_pose(ta1, ta2, from_tray=True)
            self._drop(stack)
            held = False
            self.holding = None
            ok = True
            self._ring_ready = True
        except ArmAbort:
            raise
        except DroppedError as ex:                               # (10-23)已经在环上松手了：算放了，只收臂
            held, ok, dropped = False, True, ex
            self.holding = None
        except (ArmError, VisionError) as ex:
            if held or self.holding is item:                     # (10-23)_take 夹紧了才出错的也算夹着
                self._put_back(item, f'放置出错：{ex}')
            else:
                self._recover(f'放置出错：{ex}')
        self._dodge = None
        self.stats.place(ok)
        if ok:
            self.in_tray.pop(item.slot, None)
            self.on_ring.setdefault((zone, ring), []).append(item)
            self.placements.append((zone, ring, bool(stack), None))
        self._show_stats()
        if ok and dropped is not None:
            self._recover(f'{item.color_name}已经放在环{ring}上，抬起来时出错：{dropped}')
        return ok

    # ------------------------------------------------------------------ 粗加工区 / 暂存区：到工位先看清三个圆环
    def hold_heading(self):
        """车是手搬到这个工位的：把"现在的车头方向"记为 STM32 要保持的方向(HOME)。
        不然 STM32 还按开机时(或上一次转弯后)的方向保持，底盘一前后挪就把车头往那个方向转，车身就歪了。"""
        link = getattr(self.act, 'link', None) if self.act is not None else None
        if link is None:
            return False
        try:
            out = link.request('HOME', 3.0)
        except Exception as ex:
            self.log(f'    (记车头方向出错：{ex!r})')
            return False
        ok, reply = out[0], out[1]
        if 'ABORT' in (reply or ''):
            raise Abort(f'HOME -> {reply}')
        if ok:
            self.log('  车头方向：以现在的方向为准(底盘前后挪时保持这个方向)')
        else:
            self.log(f'  ★ 记车头方向失败({reply})：底盘挪动时可能会把车头转回开机时的方向')
        return bool(ok)

    def _arm_side(self, side):
        """(10-20)告诉 STM32 手臂接下来在哪边干活(SIDE RAW / SIDE RING)：去/离开车上转盘绕开电池盒用那一边的参数
        (圆环那边 R1T1/R2T/R1T2/R2SPD，没调过照旧原料区那组)。OBS、GRAB/PICK、PLACE 也会设，这里是保险。旧程序不认就算了。"""
        arm = self.arm
        if arm is None or getattr(arm, '_no_side', False):
            return
        try:
            ok, reply, _info = arm.request(f'SIDE {side}', 2.0)
        except ArmAbort as ex:
            raise Abort(str(ex))
        except Exception:
            return
        if not ok and 'CMD' in str(reply or ''):
            arm._no_side = True

    def _zone_start(self, zone, items=()):
        """到粗加工区/暂存区：刷新 STM32 参数(可能刚 set 过)，看清三个圆环(算出到每个环底盘要挪多少)。
        没有要放的物料、或者时间到了，就不看(省时间)。"""
        self.ring_f = {}
        self.ring_px = {}
        self.ring_uv = {}
        self._ring_rmed = None
        self._p2.pop(zone, None)                  # 车重新开到这个区：2 号环要重新对准
        self._p2_failed.discard(zone)
        self._zone_v_only = False                 # (10-20)这一趟车只用 V 动过(2 号环车轮连续对准，之后不动)：挪回停车点也只用 V
        self._zone_geom_why[zone] = self._zone_geom_missing() if self.cfg.get('zone_arm_geom', True) else 'zone_arm_geom 关着'
        self._zone_s_ref = None
        self.ring_gate_px = None
        self._tf_off = False
        self._ring_ready = self._at_obs = False
        self._zone_d2 = []
        self._tilt = None
        try:
            self.arm.params(refresh=True)
        except ArmAbort as ex:
            raise Abort(str(ex))
        except Exception:
            pass
        self._arm_side('RING')                    # (10-20)夹着物料去/离开车上转盘按圆环那组绕电池盒(R1T1~R2SPD)
        if not self.cfg.get('ring_survey', True) or not hasattr(self.vision, 'ring_list'):
            return
        if self._no_time('place', '放置', quiet=True) or (items and not any(same_item(self.in_tray.get(it.slot), it) for it in items)):
            return
        if not items:
            return
        try:
            self._survey(zone)
            if self.cfg.get('take_first', True) and hasattr(self.vision, 'note_claw_clear'):
                # 先取物料再对准时，对准时爪子里有物料：趁现在爪子空着、在观察姿态，记下干净的爪子区域(取回蓝色物料要用)
                try:
                    self.vision.ring_error(n=1)
                    self.vision.note_claw_clear()
                except (VisionError, TypeError):
                    pass
        except ArmAbort as ex:
            raise Abort(str(ex))
        except (ArmError, VisionError) as ex:
            self.ring_f = {}
            self.log(f'    ★ 看圆环出错：{ex}；按配置里的圆环间距走')

    def _jac_f(self, rings0, fresh=False):
        """底盘往前走 1mm，圆环在画面里移动多少像素(2 维向量)。返回 (jf, 这次是不是现测的)；测不出来 jf 是 None。
        以前测过(servo_cal.json)就直接用(fresh=True 不用)；没有就现测：往前挪 40mm 再看一眼、再退回来(只前后动，不横移)。
        rings0 = 挪之前看到的圆环(ring_list)。挪 40mm 画面只动几十像素，两个圆环之间隔着两百多像素：
        取挪动前后离得最近的一对，就是同一个环(不会认成旁边那个)。挪得多一点方向量得准一点(车身斜度要用)。"""
        import numpy as np
        J = self.store.get('RING', 'ch') if (self.store is not None and not fresh) else None
        if J is not None:
            jf = np.asarray(J, float)[:, 1]
            if float(np.hypot(*jf)) >= 0.2:
                return jf, False
        old = None                                               # (10-26)重新测之前存的(vcal RING 量的、横移那一列是实测的)
        if fresh and self.store is not None and not self.store.synth('RING'):
            old = self.store.get('RING', 'ch')
        if not rings0:
            return None, True
        step = 40
        self.log(f'    第一次在圆环这里：底盘前进 {step}mm 再退回来，看画面怎么动(以后不用再测)')
        self.act.chassis_move(0, step)
        try:
            self.sleep(0.3)
            rings1 = self.vision.ring_list(n=self.cfg.get('survey_frames') or None)
        finally:
            self.act.chassis_move(0, -step)
        if not rings1:
            return None, True
        a = np.array([[r[0], r[1]] for r in rings0], float)
        b = np.array([[r[0], r[1]] for r in rings1], float)
        pairs = [(float(np.hypot(*(qb - qa))), qb - qa) for qa in a for qb in b]
        _dmin, d0 = min(pairs, key=lambda t: t[0])
        same = [d for _dist, d in pairs if np.hypot(*(d - d0)) <= 8.0]   # 其他环挪得一样的也算上，取平均
        jf = np.mean(same, axis=0) / float(step)
        if float(np.hypot(*jf)) < 0.2:
            return None, True
        if self.store is not None:
            keep = None
            if old is not None:
                o = np.asarray(old, float)
                of, os_ = o[:, 1], o[:, 0]
                if float(of @ jf) >= 0.9 * float(np.hypot(*of)) * float(np.hypot(*jf)) and float(np.hypot(*os_)) > 1e-6:
                    # (10-26)前进的方向没变(摄像头没转)，只是大小不对(看圆环的高度变了)：横移那一列方向照用，大小跟着前进那一列改，
                    #   不当"补出来的"(以前当补的，2 号环就只能一步一步挪、横移只许 20mm)
                    keep = np.column_stack([os_ / float(np.hypot(*os_)) * float(np.hypot(*jf)), jf])
            if keep is not None:
                self.store.put('RING', 'ch', keep)
            else:
                self.store.put('RING', 'ch', [[-jf[1], jf[0]], [jf[0], jf[1]]])     # S 那一列用不到(不横移)，补一个和 F 垂直的
                self.store.set_synth('RING', True)                                # 记下"横移那一列是补的"：真要横移时重新测
            self.store.save()
        return jf, True

    def _survey(self, zone, obs=True):
        """手臂摆到圆环上方(空爪)，一次看清画面里的圆环，按排列认出 1、2、3 号，算出每个环正对爪子时底盘的前后位置(self.ring_f)。
        画面里看到三个：直接认；两个：哪一头能看到却没有环，那一头就是这一排的尽头(圆环上放着物料时不这样认，可能只是没认出来)；
        两个隔了两个间距：中间那个没认出来，补上；一个、或者认不准：当作离爪子近的是 2 号(车停在 2 号环前面)。
        看到的圆环间距和"底盘走的距离"对不上(认错了/漏了)：这次不按看到的走，按配置里的间距走。
        顺便用圆环间距算出每毫米多少像素(没做 vclaw RING 时用它)。"""
        import numpy as np
        v, cfg = self.vision, self.cfg
        self._ring_scale_seed()
        if obs:                                                  # (10-20)obs=False：手臂已经在看圆环的姿态(往前找环时停下重看)
            self.arm.obs('RING', open_claw=True)
        self._a2_hint = (self.arm.params() or {}).get('A2P')
        self._a1_hint = (self.arm.params() or {}).get('A1P')
        self._ring_ready = self._at_obs = True
        rings = v.ring_list(n=cfg.get('survey_frames') or None)
        if not rings:
            self._ring_odd_size()
            self.log('    ★ 画面里看不到圆环：按配置里的圆环间距走')
            return
        self._ring_rmed = float(np.median([r[2] for r in rings]))   # (10-20)圆环在画面里多大(最外圈半径)：对准 2 号环时只认这么大的
        jf, measured = self._jac_f(rings)
        if jf is None:
            self.log('    ★ 测不出底盘前后挪和画面的关系：按配置里的圆环间距走')
            return
        jf = self._jf_best(jf)
        cu, cv_ = v.claw('RING')
        claw = np.array([cu, cv_], float)
        pts = np.array([[r[0], r[1]] for r in rings], float)
        sp_mm = float(cfg.get('ring_spacing_mm') or 150.0)
        # 这一排圆环在画面里的方向：看到两个以上用它们自己连线的方向，否则用底盘前后的方向
        if len(pts) >= 2:
            axis = np.linalg.svd(pts - pts.mean(axis=0))[2][0]
        else:
            axis = jf / np.hypot(*jf)
        if abs(axis[0]) >= abs(axis[1]):
            axis = axis if axis[0] > 0 else -axis                # "从左到右"
        else:
            axis = axis if axis[1] > 0 else -axis                # 竖着排的："从上到下"
        pts = pts[np.argsort(pts @ axis)]
        if len(pts) > 3:                                         # 多看到了(别的工位的环？)：取爪子点附近连着的三个
            proj = pts @ axis
            mid = int(np.argmin(np.abs(proj - float(claw @ axis))))
            lo = min(max(mid - 1, 0), len(pts) - 3)
            pts = pts[lo:lo + 3]

        def jf_px():                                             # 按"底盘走 1mm 画面动多少"算，相邻两个环在画面里隔多少像素
            return abs(float(jf @ axis)) * sp_mm

        if len(pts) >= 2:
            gaps = np.diff(pts @ axis)
            sp_px = float(np.median(gaps))
            ratio = sp_px / max(jf_px(), 1e-9)
            if not measured and not (0.6 <= ratio <= 1.6 or (len(pts) == 2 and 1.7 <= ratio <= 2.3)):
                self.log(f'    存着的"底盘前后挪 1mm 画面动多少"({np.hypot(*jf):.2f} 像素)和圆环间距({sp_px / sp_mm:.2f} 像素/mm)对不上，重新测')
                jf2, _m = self._jac_f(rings, fresh=True)
                if jf2 is not None:
                    jf = jf2
                    ratio = sp_px / max(jf_px(), 1e-9)
            if len(pts) == 2 and 1.7 <= ratio <= 2.3:
                pts = np.array([pts[0], (pts[0] + pts[1]) / 2.0, pts[1]])     # 隔了两个间距：中间那个没认出来(比如被盖住)，补上
                sp_px /= 2.0
                ratio /= 2.0
                self.log('    中间那个圆环没认出来，按两边的位置补上')
            if len(pts) == 3 and max(gaps) > 1.3 * min(gaps):
                self.log(f'    ★ 看到的三个圆环间距不一样({gaps[0]:.0f}、{gaps[1]:.0f} 像素)，可能认错了：这次按配置里的间距走')
                return
            if not 0.7 <= ratio <= 1.4:
                self.log(f'    ★ 看到的圆环间距({sp_px:.0f} 像素)和按底盘走的距离算的({jf_px():.0f} 像素)对不上：'
                         '可能漏认/认错了圆环，或者底盘走的距离不准(FPPM)。这次按配置里的间距走')
                return
        else:
            sp_px = jf_px()
        h_img, w_img = self._frame_size()
        rmax = float(np.median([r[2] for r in rings]))
        vc = getattr(v, 'cfg', None)
        if isinstance(vc, dict):
            vc['ring_rmax_hint'] = rmax                          # 圆环在画面里多大：对准时只在爪子附近那一块画面里找要用

        def inside(q):
            return rmax <= q[0] <= w_img - rmax and rmax <= q[1] <= h_img - rmax

        step = axis * sp_px
        n = len(pts)
        covered = any(st for (z, _k), st in self.on_ring.items() if z == zone)   # 圆环上已经放着物料(白心被盖住，可能认不全)
        dist = np.abs((pts - claw) @ axis)                      # 沿这一排离爪子点多远(离圆环远近的差靠手臂伸缩补，不算在里面)
        guess = False
        if n == 3:
            first = 0                                            # pts[0] 是排在最前面(左/上)的那个
        elif n == 2:
            near = int(np.argmin(dist))
            before, after = inside(pts[0] - step), inside(pts[1] + step)
            end_first = 0 if (before and not after) else (1 if (after and not before) else None)   # "那一头看得到却没有环"推出来的
            if float(dist[near]) <= 0.35 * sp_px:
                # 爪子下面(附近)就有一个环：车按要求停在 2 号环前面，它就是 2 号。
                # 另一边那个环没认出来(被爪子挡住、反光……)时，"那一头没有环"会推错成停在 1 号/3 号，整排认错一位，所以不信它
                first = 1 if near == 0 else 0
                if end_first is not None and end_first != first:
                    self.log('    ★ 画面里' + ('左(上)' if near == 0 else '右(下)') + '边那个圆环没认出来(被爪子挡住了？反光？)：'
                             '按"车停在 2 号环"算，爪子下面这个是 2 号')
            elif end_first is not None and not covered:
                first = end_first                                # 车停在两个环中间：看哪一头是这一排的尽头
            else:
                first = 1 if near == 0 else 0                    # 认不准：当作离爪子点近的那个是中间(2 号)
                guess = True
        else:
            first = 1                                            # 只看到一个：当作中间那个(2 号)
            guess = True
        if guess and float(np.min(dist)) > 0.3 * sp_px:
            self.log('    ★ 只看到' + ('两个' if n == 2 else '一个') + '圆环、又离爪子挺远，认不准哪个是 2 号：按离爪子近的那个算。'
                     '车要停在 2 号环正对爪子的地方(画面里能同时看到三个环)')
        pos = [pts[0] + (k - first) * step for k in range(3)]    # 这一排三个位置(看不到的按间距推)
        lr = str((cfg.get('ring_order') or {}).get(zone, 'lr')).lower() != 'rl'
        if self.ring_rev:
            lr = not lr
        ids = (1, 2, 3) if lr else (3, 2, 1)
        n2 = float(jf @ jf)
        many = n >= 2
        nrm = np.array([-axis[1], axis[0]])                      # 垂直于这一排的方向(画面里)：离圆环远/近
        tilt, cos_t = None, 1.0
        if many:
            jd = jf if float(jf @ axis) > 0 else -jf
            t0 = math.degrees(math.atan2(float(axis[0] * jd[1] - axis[1] * jd[0]), float(axis @ jd)))
            tmax = float(cfg.get('survey_tilt_max_deg', 12.0))
            if abs(t0) > tmax:
                # 车不可能停斜这么多：是"底盘挪 40mm 看画面怎么动"测歪了(挪的时候认错了环、画面抖)。方向按圆环连线算，不按它
                self.log(f'    ★ 按底盘挪动测出来车身斜了 {t0:+.0f}°，不可信(车停得没这么斜)：方向按看到的圆环连线算')
                jf = (1.0 if float(jf @ axis) > 0 else -1.0) * axis * float(np.hypot(*jf))
                n2 = float(jf @ jf)
            # 看到两个以上：沿这一排每毫米多少像素按圆环间距算(比底盘挪 40mm 测出来的准)，底盘往哪边挪画面往哪边动看 jf
            m = (1.0 if float(jf @ axis) > 0 else -1.0) * sp_px / sp_mm
            # 车身和这一排的夹角 = 底盘前进时圆环在画面里移动的方向(jf) 和 这一排圆环连线的方向(axis) 差多少
            jd = jf if float(jf @ axis) > 0 else -jf
            tilt = math.degrees(math.atan2(float(axis[0] * jd[1] - axis[1] * jd[0]), float(axis @ jd)))
            if abs(tilt) < 15.0:
                cos_t = math.cos(math.radians(tilt))             # 车身斜了：前进 1mm 沿这一排只走 cos 那么多
        jn = float(jf @ nrm) if many else 0.0                    # 底盘每前进 1mm，圆环往"离圆环远/近"的方向动多少像素(车身斜了才不是 0)
        seen_pts = np.array([[r[0], r[1]] for r in rings], float)
        perp0 = float(np.mean((pts - claw) @ nrm))               # 现在(看圆环的位置)圆环那一排离爪子点多远(像素，垂直于这一排)
        parts = []
        for k, q in zip(ids, pos):
            off = np.asarray(q, float) - claw
            if many:
                f = -float(off @ axis) / (m * cos_t)             # 底盘前后挪多少，这个环到爪子点
                side = abs(float(off @ nrm) + jn * f)            # 挪到这个环以后垂直于这一排还差多少(像素)：靠手臂伸缩补
            else:
                f = -float(jf @ off) / n2
                side = float(np.hypot(*(off + jf * f)))
            parts.append((k, f, side))
        far = max(abs(f) for _k, f, _s in parts)
        if far > 2.6 * sp_mm:
            self.log(f'    ★ 算出来要挪 {far:.0f}mm 才到圆环，不对劲(认错了？)：这次按配置里的间距走')
            return
        d = self.act.disp
        for k, f, _side in parts:
            self.ring_f[(zone, k)] = d['F'] + f
        for k, q in zip(ids, pos):                               # (10-20)对准 2 号环时按这个位置找(车轮连续对准)
            self.ring_uv[(zone, k)] = (float(q[0]), float(q[1]), float(d['S']), float(d['F']))
        if many:
            for k, q in zip(ids, pos):
                off = np.asarray(q, float) - claw
                f = -float(off @ axis) / (m * cos_t)
                self.ring_px[(zone, k)] = (float(off @ nrm) + jn * f, nrm.copy(), float(d['S']))
        scale = sp_px / sp_mm
        vc = getattr(v, 'cfg', None)
        if isinstance(vc, dict) and not vc.get('ring_rmax_cal') and many:
            vc.setdefault('px_per_mm', {})['RING'] = scale       # 没做 vclaw RING：按圆环间距算的比例换算毫米
            vc['_ring_scale_src'] = 'survey'
            if self.store is not None and hasattr(self.store, 'set_extra'):
                self.store.set_extra('RING', 'px_mm_seen', round(float(scale), 3))   # (10-26)记下来：下次开机只看到一个环也按它算
                self.store.save()
        # 对准时只认这么近的圆环：沿这一排不到半个间距(旁边那个环至少隔一个间距)，再加上挪到每个环以后离这一排远近的偏差
        side_px = max(side for _k, _f, side in parts)
        self.ring_gate_px = math.hypot(0.45 * sp_px, side_px + 10.0)
        seen = '、'.join(str(ids[i]) for i in range(3) if any(np.hypot(*(pos[i] - p)) < 0.3 * sp_px for p in pts))
        self.log('    圆环在画面里：' + '  '.join(f'({r[0]:.0f},{r[1]:.0f}) 半径{r[2]:.0f}' for r in rings) +
                 f'；爪子点 ({cu:.0f},{cv_:.0f})')
        self.log(f'    看到 {len(rings)} 个圆环(认出 {seen} 号)，每毫米 {scale:.2f} 像素：' +
                 '  '.join(f'环{k} ' + ('不用挪' if abs(f) < 1 else ('前进' if f > 0 else '后退') + f' {abs(f):.0f}mm') +
                           f'(横向差约 {side / scale:.0f}mm)' for k, f, side in sorted(parts)))
        self._survey_tilt(zone, jf, axis, nrm, tilt, jn, perp0, seen_pts, sp_px, float(d['F']))
        worst = max(side for _k, _f, side in parts) / scale
        if worst > 40.0:
            smax = float(cfg.get('ring_strafe_max_mm') or 0.0)
            self.log(f'    ★ 车离圆环那一排的远近差了约 {worst:.0f}mm：手臂伸缩补不过来的部分' +
                     (f'车轮会横着挪(最多 {smax:.0f}mm)' if smax > 0 and not cfg.get('chassis_strafe') else '补不了') +
                     '；车停得再正一点更快')
        if n2 > 0 and many:
            ratio = (float(np.hypot(*jf)) / scale)
            if not 0.75 <= ratio <= 1.33:
                self.log(f'    ★ 按底盘走的距离算是每毫米 {np.hypot(*jf):.2f} 像素，按圆环间距 {sp_mm:g}mm 算是 {scale:.2f}：'
                         '底盘前后走的距离不准(FPPM)，或者圆环间距不是这么多(mission_cfg.ring_spacing_mm)')

    def _ring_scale_seed(self):
        """(10-26 实车：开机后第一次到暂存区只认出一个"圆"，没量出圆环这里每毫米多少像素，还是配置里默认的 2.96：vcal RING 量的
        车轮和画面的对应关系(横移 1mm 动 1.12 像素)被当成不合理，2 号环退回一步一步挪(横移只许 20mm)，三次都没对上；按 2.96 算圆环
        该有多大，半径 259 像素的大圆也被当成了圆环(10-23 看清三个环时量的是 1.27，圆环半径约 55 像素))。
        没做 vclaw RING(量圆环大小)、这次运行也还没看清过三个环时，按量过的来：以前看清三个环时按环距量的(servo_cal.json 的
        RING.px_mm_seen)，没有就按 vcal RING 量的车轮和画面的对应关系；都没有才用配置里的 px_per_mm.RING。一次运行只定一次。"""
        v, store = self.vision, self.store
        vc = getattr(v, 'cfg', None)
        if not isinstance(vc, dict) or vc.get('ring_rmax_cal') or vc.get('_ring_scale_src'):
            return
        val, src, key = None, '', 'cfg'
        try:
            seen = float(store.extra('RING', 'px_mm_seen') or 0.0) if (store is not None and hasattr(store, 'extra')) else 0.0
        except (TypeError, ValueError):
            seen = 0.0
        if 0.2 <= seen <= 20.0:
            val, src, key = seen, '按以前看清三个圆环时量的环距', 'seen'
        elif store is not None:
            J = store.get('RING', 'ch')
            if J is not None:
                n = np.linalg.norm(np.asarray(J, float), axis=0)
                m = float(n[1]) if (hasattr(store, 'synth') and store.synth('RING')) else float(np.mean(n))   # 只测过前进那一列
                if 0.2 <= m <= 20.0:
                    val, src, key = m, '按 vcal RING 量的车轮和画面的对应关系', 'J'   # 'J'：大小没有别的量法核对过
        vc['_ring_scale_src'] = key
        if val is None:
            return
        pm = vc.setdefault('px_per_mm', {})
        old = pm.get('RING')
        pm['RING'] = val
        if old is None or abs(float(old) - val) > 0.1 * val:
            self.log(f'    (圆环这里每毫米多少像素：没做 vclaw RING，{src}算 {val:.2f}' +
                     (f'，不用配置里的 {float(old):.2f}' if old is not None else '') + ')')

    def _ring_odd_size(self):
        """(10-26)按大小认不出圆环时：不限大小再看一帧，画面里有圆就说出它多大、圆环该多大(认成了别的圆？摄像头离地的高度变了？)。"""
        v = self.vision
        vc = getattr(v, 'cfg', None)
        if not hasattr(v, 'ring_px') or not isinstance(vc, dict):
            return
        try:
            p = v.ring_px(n=1, any_size=True)
        except (VisionError, TypeError, ValueError):
            return
        r = getattr(v, 'last_ring_rmax', None)
        if p is None or not r:
            return
        try:
            exp = (float(vc.get('ring_outer_diam_mm') or 95.0) + float(vc.get('ring_line_mm') or 0.0)) / 2.0 * float(v.scale('RING'))
        except (KeyError, TypeError, ValueError):
            return
        self.log(f'    (画面 ({p[0]:.0f},{p[1]:.0f}) 有个圆，最外圈半径 {float(r):.0f} 像素；圆环最外圈半径该约 {exp:.0f} 像素'
                 f'(每毫米 {float(v.scale("RING")):.2f} 像素)：大小对不上，不当圆环。车没停在圆环前面？要是它真是圆环，'
                 '就是摄像头离地的高度变了(看圆环的高度 ZOBRNG、升降高度 arm LIFT? 对不对)，重新 vclaw RING)')

    # ---- 车身和圆环那一排不平行(车停斜了)：量出来、提醒、提前补手臂伸缩
    def _jf_best(self, jf):
        """车身斜度要用的"底盘前进时画面怎么动"的方向：换环时量准过(RING.ch_f_ref)、而且和现在存的差不多(15° 以内，
        摄像头没动过)就用量准的那个方向(存的 J 可能被对准时 12mm 的小步探测换掉了，方向差一两度)，大小按现在的。"""
        import numpy as np
        ref = self.store.extra('RING', 'ch_f_ref') if (self.store is not None and hasattr(self.store, 'extra')) else None
        try:
            r = np.asarray(ref, float)
            jf = np.asarray(jf, float)
            nr, nj = float(np.hypot(*r)), float(np.hypot(*jf))
            if nr > 1e-6 and nj > 1e-6 and float(r @ jf) / (nr * nj) >= math.cos(math.radians(15.0)):
                return r / nr * nj
        except Exception:
            pass
        return jf

    def _jf_refined(self, jf):
        """jf(底盘前进时画面怎么动)是不是换环时用一两百毫米量准过的那个(存在 servo_cal.json 的 RING.ch_f_ref)。"""
        import numpy as np
        ref = self.store.extra('RING', 'ch_f_ref') if (self.store is not None and hasattr(self.store, 'extra')) else None
        try:
            r, j = np.asarray(ref, float), np.asarray(jf, float)
            return float(r @ j) / max(float(np.hypot(*r)) * float(np.hypot(*j)), 1e-12) >= math.cos(math.radians(0.5))
        except Exception:
            return False

    def _survey_tilt(self, zone, jf, axis, nrm, tilt, jn, perp0, seen_pts, sp_px, F0):
        """看三个环时：报车身和这一排的夹角(斜到 tilt_warn_deg 以上打 ★)，记下提前补手臂要用的东西(self._tilt)。
        手臂的 J(ID2/ID1 转 1° 画面动多少)知道时：垂直于这一排差 1 像素，ID2/ID1 要转多少度(g)。"""
        import numpy as np
        cfg = self.cfg
        warn = float(cfg.get('tilt_warn_deg', 3.0))
        refined = self._jf_refined(jf)
        Ja = self.store.get('RING', 'arm') if self.store is not None else None
        g = None
        if Ja is not None:
            try:
                g = -np.linalg.solve(np.asarray(Ja, float), nrm)
            except np.linalg.LinAlgError:
                g = None
        pre = bool(cfg.get('tilt_precorrect', True)) and bool(cfg.get('ring_precorrect', True)) and g is not None
        warned = False
        if tilt is not None:
            src = '' if refined else '(按底盘前后挪测的方向算，可能差 1° 左右；换环时再量准)'
            if abs(tilt) >= warn:
                warned = True
                self.log(f'    ★ 车身和圆环那一排斜了 {tilt:+.1f}°{src}：把车摆正(停车时车身要和圆环那一排平行)' +
                         ('；这次已按斜的角度提前补手臂伸缩' if pre else '；挪到别的环时离圆环会越来越远/近，靠手臂伸缩补'))
            else:
                self.log(f'    车身和圆环那一排的夹角 {tilt:+.1f}°{src}')
        self._tilt = dict(zone=zone, F0=float(F0), perp0=float(perp0), jn=float(jn), nrm=np.asarray(nrm, float),
                          axis=np.asarray(axis, float), jf=np.asarray(jf, float), angle=tilt, warned=warned,
                          refined=refined, g=g, rings=np.asarray(seen_pts, float), sp_px=float(sp_px), relooked=False)

    def _relook(self):
        """刚看完三个环、手臂没动过，底盘开到了第一个要放的环(离看圆环的地方 100mm 以上)：再看一眼画面。
        圆环在画面里整体移动的方向就是"底盘前进"在画面里的方向；挪了一两百毫米量的比第一次挪 40mm 测的准得多：
        用它把车身和圆环那一排的夹角量准(存进 servo_cal.json，以后到工位直接用)，这个环和后面的环按它提前补手臂伸缩。
        顺便返回这个环现在离爪子点多少像素(爪子附近的那个环；没看到返回 None)：手臂直接转过去，省掉对准的第一步。"""
        import numpy as np
        t = self._tilt
        if not t or t.get('relooked') or not self.cfg.get('tilt_precorrect', True):
            return None
        dF = float(self.act.disp['F']) - t['F0']
        if abs(dF) < 100.0:
            return None
        t['relooked'] = True
        try:
            rings = self.vision.ring_list(n=self.cfg.get('survey_frames') or None)
        except VisionError:
            return None
        if not rings:
            return None
        pts1 = np.array([[r[0], r[1]] for r in rings], float)
        claw = np.array(self.vision.claw('RING'), float)
        dc = np.hypot(pts1[:, 0] - claw[0], pts1[:, 1] - claw[1])
        gate = self.ring_gate_px or 0.45 * t['sp_px']
        p_now = (pts1[int(np.argmin(dc))] - claw) if float(np.min(dc)) <= gate else None
        if t.get('angle') is None:
            return p_now
        shifts = []
        for q in t['rings']:
            e = q + t['jf'] * dF                                 # 按原来的 jf 估计它现在在哪
            dist = np.hypot(pts1[:, 0] - e[0], pts1[:, 1] - e[1])
            j = int(np.argmin(dist))
            if float(dist[j]) < 0.3 * t['sp_px']:
                shifts.append(pts1[j] - q)
        if not shifts:
            return p_now
        jd = np.mean(shifts, axis=0) / dF
        if float(np.hypot(*jd)) < 0.2:
            return p_now
        jf = jd / float(np.hypot(*jd)) * float(np.hypot(*t['jf']))   # 只要方向(走的距离不一定准)，大小按原来的
        axis, nrm = t['axis'], t['nrm']
        jdd = jf if float(jf @ axis) > 0 else -jf
        ang = math.degrees(math.atan2(float(axis[0] * jdd[1] - axis[1] * jdd[0]), float(axis @ jdd)))
        old = t['angle']
        t.update(jf=jf, jn=float(jf @ nrm), angle=ang, refined=True)
        warn = float(self.cfg.get('tilt_warn_deg', 3.0))
        self.log(f'    换到这个环再看一眼(底盘挪了 {dF:+.0f}mm)：车身和圆环那一排的夹角量准了 {ang:+.1f}°(刚才按 {old:+.1f}° 算)')
        if abs(ang) >= warn and not t.get('warned'):
            t['warned'] = True
            self.log(f'    ★ 车身和圆环那一排斜了 {ang:+.1f}°：把车摆正(停车时车身要和圆环那一排平行)；这次已按斜的角度提前补手臂伸缩')
        elif abs(ang) < warn and t.get('warned'):
            self.log(f'    (没有刚才算的那么斜，{ang:+.1f}° 不用管)')
        st = self.store
        J = st.get('RING', 'ch') if st is not None else None
        if J is not None:                                        # 存下量准的方向：以后到工位直接用
            J = np.array(J, float)
            if st.synth('RING'):
                J[:, 0] = (-jf[1], jf[0])
            J[:, 1] = jf
            st.put('RING', 'ch', J)
            st.set_extra('RING', 'ch_f_ref', [round(float(jf[0]), 5), round(float(jf[1]), 5)])
            st.save()
        return p_now

    def _arm_to(self, p):
        """圆环在画面里离爪子点 p 像素(手臂在观察姿态)：手臂 ID2、ID1 各要转多少度才对上(用存着的手臂 J)。没有 J 返回 None。"""
        import numpy as np
        Ja = self.store.get('RING', 'arm') if self.store is not None else None
        if Ja is None:
            return None
        try:
            du = -np.linalg.solve(np.asarray(Ja, float), np.asarray(p, float))
        except np.linalg.LinAlgError:
            return None
        lim1 = 0.9 * float(((self.servo.cfg if self.servo is not None else {}).get('arm_limit_deg') or {}).get('id1', 12.0))
        return float(du[0]), max(-lim1, min(lim1, float(du[1])))

    def _survey_line(self):
        """看三个环时量的(能提前补手臂伸缩才有)：self._tilt；不能补返回 None。"""
        import numpy as np
        t = self._tilt
        if not t or not self.cfg.get('tilt_precorrect', True):
            return None
        if t.get('g') is None and self.store is not None:
            Ja = self.store.get('RING', 'arm')               # 看圆环时还没有手臂的 J(第一次在这里)，对准过一次就有了
            if Ja is not None:
                try:
                    t['g'] = -np.linalg.solve(np.asarray(Ja, float), t['nrm'])
                except np.linalg.LinAlgError:
                    t['g'] = None
        return t if t.get('g') is not None else None

    def _predict_d1(self, d2):
        """ID2 提前伸缩 d2 度时，ID1 跟着补多少度：车身斜了，伸缩的方向不正好垂直于圆环那一排，ID1 补上沿这一排的那一点。
        太小(舵机动不了)就不补。"""
        t = self._survey_line()
        if t is None or d2 is None or abs(float(t['g'][0])) < 1e-6:
            return 0.0
        d1 = float(d2) * float(t['g'][1]) / float(t['g'][0])
        d1 = max(-3.0, min(3.0, d1))
        return d1 if abs(d1) >= 0.35 else 0.0

    def _frame_size(self):
        cam = (getattr(self.vision, 'cfg', None) or {}).get('camera') or {}
        return float(cam.get('height', 480)), float(cam.get('width', 640))

    def _ring_measure(self, kind='RING'):
        """对准圆环用的测量：只认离爪子点 ring_gate_px 以内的圆环(看清过三个环以后才有)，不去追旁边那个。
        place_shift_mm：放下的位置整体挪一点(爪子点没量准时的实测修正)——把"要对到的点"在画面里挪过去。
        kind = 'RING1' / 'RING3'：(10-26)按 1、3 号环自己的十字算。"""
        gate = self.ring_gate_px
        n = self.cfg.get('zone_frames') or None
        kw = {} if kind == 'RING' else {'kind': kind}
        if gate:
            base = lambda: self.vision.ring_error(n=n, max_px=gate, **kw)
        else:
            base = lambda: self.vision.ring_error(n=n, **kw)
        off = self._place_shift_px()
        if off is None:
            return base

        def shifted():
            e = base()
            return None if e is None else (float(e[0]) - off[0], float(e[1]) - off[1])
        return shifted

    def _place_shift_px(self):
        """place_shift_mm 换成画面里的像素偏移(对准时要让"圆环 - 爪子点"等于它)。用底盘 J：车往前走 1mm 地上的东西在画面里动 J[:,1]；
        要放的点往前挪 d(车往前走 d 才对准)，对准时"圆环 - 爪子点"就该是 J[:,1]·d。没有底盘 J 或者挪 0 返回 None。"""
        if not self.cfg.get('place_comp_on', False):
            return None                                          # (10-20 用户)放置补偿都不用
        if getattr(self, '_pcal_raw', False) or self.place_adj('RING') is not None:   # pcal 量的时候要按摄像头原样对准；量过就不用这个估计值
            return None
        sh = self.cfg.get('place_shift_mm') or {}
        try:
            f, sl = float(sh.get('F', 0.0) or 0.0), float(sh.get('S', 0.0) or 0.0)
        except (TypeError, ValueError, AttributeError):
            return None
        if abs(f) < 0.5 and abs(sl) < 0.5:
            return None
        J = self.store.get('RING', 'ch') if self.store is not None else None
        if J is None:
            if not getattr(self, '_shift_said', False):
                self._shift_said = True
                self.log('    (place_shift_mm：还没有底盘和画面的对应关系，这次不挪)')
            return None
        import numpy as np
        J = np.asarray(J, float)
        off = J[:, 1] * f + J[:, 0] * sl
        if not getattr(self, '_shift_said', False):
            self._shift_said = True
            self.log(f'    放下位置按 place_shift_mm 挪：前后 {f:+.0f}mm、左右 {sl:+.0f}mm(画面里 {off[0]:+.1f},{off[1]:+.1f} 像素)')
        return (float(off[0]), float(off[1]))

    def _zone_servo_kw(self):
        """工位里对准的底盘限制：前后挪最多 ring_fix_max_mm；横移只在手臂伸缩够不着时用，离停车点累计不超过 ring_strafe_max_mm。"""
        cfg = self.cfg
        kw = {}
        if not cfg.get('chassis_strafe', False):
            smax = float(cfg.get('ring_strafe_max_mm') or 0.0)
            if smax > 0:
                ds = float(self.act.disp['S']) if self.act is not None else 0.0
                kw['chassis_axes'] = 'Fs'
                kw['chassis_s_range'] = (-smax - ds, smax - ds)
            else:
                kw['chassis_axes'] = 'F'
        if cfg.get('ring_fix_max_mm'):
            kw['chassis_fix_max_mm'] = float(cfg['ring_fix_max_mm'])
        if cfg.get('align_max_iter'):
            kw['max_iter'] = int(cfg['align_max_iter'])
        if cfg.get('zone_timeout_s'):
            kw['timeout_s'] = float(cfg['zone_timeout_s'])
        if cfg.get('zone_gain'):
            kw['gain_arm'] = float(cfg['zone_gain'])
        kw['filt'] = bool(cfg.get('zone_filter', True))
        # 手臂可能已经提前转过(ring_precorrect/tilt_precorrect)：按机构的行程(观察姿态 ± arm_limit_deg)算这次还能往哪边转多少，
        # 伸缩够不着的才让车轮横移
        ang = (getattr(self.arm, 'angle', None) or {}) if self.arm is not None else {}
        rng = {}
        for key, env, cur in (('id2', self._a2_env(), ang.get(2) if ang.get(2) is not None else self._a2_hint),
                              ('id1', self._a1_env(), ang.get(1) if ang.get(1) is not None else self._a1_hint)):
            if env is not None and cur is not None:
                rng[key] = (env[0] - float(cur), env[1] - float(cur))
        if rng:
            kw['arm_range'] = rng
        kw['wheels_first'] = bool(cfg.get('wheels_first', False))
        if self.servo is not None and kw['wheels_first']:
            for k in ('wheels_step_mm', 'wheels_min_mm', 'wheels_rpm'):
                if cfg.get(k) is not None:
                    self.servo.cfg[k] = cfg[k]
        return kw

    # ------------------------------------------------------------------ 只动车轮对准(10-12 用户：每个任务点都微调车轮，不调机械臂)
    def _by_wheels(self, role):
        """这个任务点(RAW / ROUGH / TEMP)按 align_by 是不是只动车轮对准(手臂停在观察姿态不动)。"""
        ab = self.cfg.get('align_by')
        v = ab.get(role, 'arm') if isinstance(ab, dict) else ab
        return str(v or 'arm').strip().lower() == 'wheels'

    def _wheels_accept(self, key):
        """只动车轮对准完、没到容差时照样放/取的偏差上限：accept_mm 和 place_anyway_mm 取大的。"""
        return max(float(self.cfg['accept_mm'][key]), float(self.cfg.get('place_anyway_mm') or 0.0))

    def _wheels_kw(self):
        """工位里只动车轮对准：横移离停车点累计不超过 ±ring_strafe_max_mm(不压进工位)，这次前后最多再走 ring_fix_max_mm。"""
        cfg = self.cfg
        kw = dict(wheels_only=True)
        if not cfg.get('chassis_strafe', False):
            smax = float(cfg.get('ring_strafe_max_mm') or 0.0)
            ds = float(self.act.disp['S']) if self.act is not None else 0.0
            kw['chassis_s_range'] = (-smax - ds, smax - ds)
        if cfg.get('ring_fix_max_mm'):
            kw['chassis_fix_max_mm'] = float(cfg['ring_fix_max_mm'])
        if cfg.get('zone_timeout_s'):
            kw['timeout_s'] = float(cfg['zone_timeout_s'])
        return kw

    def _obs_fixed(self):
        """只动车轮：空爪摆到看圆环的观察姿态(A1P/A2P、ZOBRNG)，对准时手臂就停在这里不动。刚看完三个环、手臂没动过就不用再摆。"""
        if self._at_obs:
            return
        self.arm.obs('RING', open_claw=True)
        P = self.arm.params() or {}
        self._a2_hint, self._a1_hint = P.get('A2P'), P.get('A1P')
        self._ring_ready = self._at_obs = True

    def _a2_env(self):
        """ID2(伸缩)在工位里能到的范围(绝对角度)：观察姿态 A2P ± 视觉闭环的 arm_limit_deg.id2(约 ±44mm)，
        再和 STM32 的 A2 参数范围取交集。不知道 A2P 返回 None。"""
        P = (self.arm.params() or {}) if self.arm is not None else {}
        if 'A2P' not in P:
            return None
        lim = float(((self.servo.cfg if self.servo is not None else {}).get('arm_limit_deg') or {}).get('id2', 250.0))
        a2p = float(P['A2P'])
        lo, hi = self._a2_limits()                             # (10-18)STM32 的限位 S2MIN/S2MAX(齿条重装过，不再写死 -1210 ~ -300)
        return max(lo, a2p - lim), min(hi, a2p + lim)

    def _a1_env(self):
        """ID1 在工位里能到的范围(绝对角度)：观察角度 A1P ± arm_limit_deg.id1。不知道 A1P 返回 None。"""
        P = (self.arm.params() or {}) if self.arm is not None else {}
        if 'A1P' not in P:
            return None
        lim = float(((self.servo.cfg if self.servo is not None else {}).get('arm_limit_deg') or {}).get('id1', 12.0))
        return float(P['A1P']) - lim, float(P['A1P']) + lim

    def place_adj(self, key):
        """pcal 实测的放置补偿：摄像头对准以后，ID1、ID2 再多转多少度物料才正好落在圆环(RING=第一层)/下面那个物料(STACK=第二层)正中。
        没量过返回 None。(10-20 用户"28、29 步的补偿去掉")place_comp_on 关着(默认)：一律不用，返回 None。"""
        if not self.cfg.get('place_comp_on', False):
            if not getattr(self, '_adj_off_said', False):
                self._adj_off_said = True
                old = load_place_adj(self.cfg.get('place_adj_file'))
                if old:
                    self.log('    (放置补偿不用了：' + '  '.join(f'{"第一层" if k == "RING" else "第二层"} ID1 {v[0]:+.2f}°、ID2 {v[1]:+.2f}°'
                                                     for k, v in old.items() if isinstance(v, (list, tuple)) and len(v) >= 2) +
                             ' 存着也不加，按对准的位置直接放)')
            return None
        if not hasattr(self, '_adj'):
            self._adj = load_place_adj(self.cfg.get('place_adj_file'))
        v = self._adj.get(key)
        if not v:
            return None
        try:
            return float(v[0]), float(v[1])
        except (TypeError, ValueError, IndexError):
            return None

    def _apply_adj(self, key):
        """手臂在摄像头对准的姿态上：按 pcal 的补偿再转一点(第一层/第二层各自的)。返回补偿了没有。"""
        adj = self.place_adj(key)
        if adj is None or (abs(adj[0]) < 0.05 and abs(adj[1]) < 0.05):
            return False
        a1, a2 = self.arm.read_angles()
        if a1 is None or a2 is None:
            return False
        self._return_to_pose(a1 + adj[0], a2 + adj[1])
        return True

    def _take(self, item):
        """(10-23)从车上转盘取物料(TAKE)。偶尔一次出错时看这条指令里夹爪最后一次动作(CLAWSET)：
        - 还没夹紧：物料还在转盘槽里 -> 收臂、再取一次(以前直接不放了，少一个环的分)；
        - 夹紧了(C)：物料在爪子里(抬起来时出的错) -> 记下夹着它，先升到最高(别在转盘里就转)，接着放。
        升降位置不知道、打滑、爪子里本来就有东西(ERR NOZERO / LIFT / HOLD …)照旧抛出去。"""
        try:
            self.arm.take(item.slot)
            return
        except ArmAbort:
            raise
        except ArmError as ex:
            if any(k in str(ex) for k in ('ERR HOLD', 'ERR LIFT', 'ERR NOZERO', 'ERR NOCAL', 'ERR CMD', 'ERR RANGE')):
                raise
            if str(getattr(self.arm, 'claw_last', None) or '') == 'C':
                self.log(f'    取{item.color_name}时出错({ex})，夹爪已经夹紧了：先升到最高，接着放')
                self.holding = item
                zhi = (self.arm.params() or {}).get('ZHI')
                if zhi is not None:
                    self.arm.lift(float(zhi))
                return
            self.log(f'    取{item.color_name}时出错({ex})，夹爪还没夹：物料还在转盘里，收臂再取一次')
        self._stow_retry()
        self.arm.take(item.slot)

    def _drop(self, stack):
        """手臂已经对准：下降、松手、抬起。drop_retract=False 时不缩回伸缩舵机(接着去下一个环，工位做完再收臂)。
        (10-23)松手以后才出错(抬起来时串口没回话之类)：再抬一次；还不行抛 DroppedError(物料已经放在环上了)。"""
        P = self.arm.params() or {}
        try:
            z = float(P['ZSTK' if stack else 'ZPLC'])
            zhi = float(P['ZHI'])
        except (KeyError, TypeError, ValueError):
            z = zhi = None
        def opened():                                            # 这条指令里 STM32 张开过夹爪(CLAWSET O…)：物料已经放下
            return str(getattr(self.arm, 'claw_last', None) or '').startswith('O')

        if self.cfg.get('drop_retract', False) or z is None:
            try:
                self.arm.drop(stack)
            except ArmAbort:
                raise
            except ArmError as ex:
                if opened():
                    raise DroppedError(f'{ex}(已经松手)') from ex
                raise
            return
        self.arm.lift(z)                                         # 还夹着：出错就抛(LIFT 偶尔没回话 ArmLink 已经重发过一次)
        try:
            self.arm.claw(True)
        except ArmAbort:
            raise
        except ArmError:
            if not opened():
                raise
        try:
            self.arm.lift(zhi)
        except ArmAbort:
            raise
        except ArmError as ex:
            raise DroppedError(f'{ex}(已经松手)') from ex

    # ------------------------------------------------------------------ 白心被物料盖住时对准(取回、码垛)
    def _align_covered(self, color, key, label, confirm, wheels=False, arm_kw=None):
        """对准一个白心被物料盖住的圆环(取回：上面就是要夹的物料；码垛：上面是下面那层)：
        1 claw_px.PICK 量过、认得到这个颜色的物料：把物料顶面对到 PICK 点(最准，不用认圆环)
        2 否则认圆环(vclaw RING 量过圆环大小时，白心被盖住也能认最外圈)
        wheels=True：只动车轮(手臂不动)。arm_kw：(10-19)只动手臂、车不动(这些是给 servo.run 的参数，见 _arm_only_kw)。
        返回 (Result, '物料'/'圆环')；物料和圆环都看不到返回 (None, None)。"""
        v, cfg = self.vision, self.cfg
        tol, acc = cfg['tol_mm'][key], cfg['accept_mm'][key]
        if wheels:
            acc = self._wheels_accept(key)
        n = cfg.get('zone_frames') or None
        kwf = self._wheels_kw if wheels else self._zone_servo_kw
        allow = True
        if arm_kw is not None:
            kwf, allow = (lambda: dict(arm_kw)), False
        if int(color) in PICK_COLORS and hasattr(v, 'has_pick') and v.has_pick():
            e = v.pick_error(color, n=n)
            if e is not None:
                self._seed_pick_jac(wheels=wheels)
                res = self.servo.run('PICK', _first(e, lambda: v.pick_error(color, n=n)), v.scale('PICK'), tol, allow_chassis=allow,
                                     label=label, bounds=v.bounds('PICK'), confirm=confirm, **kwf())
                if res.ok or res.err_mm <= acc:
                    return res, '物料'
                self.log(f'    按物料没对准({res.reason})，改认圆环')
            else:
                self.log('    认不到圆环上的物料，改认圆环')
        measure = self._ring_measure()
        e = measure()
        if e is None:
            return None, None
        return self.servo.run('RING', _first(e, measure), v.scale('RING'), tol, allow_chassis=allow, label=label,
                              bounds=v.bounds('RING'), confirm=confirm, **kwf()), '圆环'

    def _seed_pick_jac(self, wheels=False):
        """PICK(物料顶面)还没有自己的 J：用圆环的 J 按两个高度的像素/毫米之比换算一份，省得再小幅动几下探测。
        wheels=True(只动车轮)：PICK 的车轮 J 横移那一列是补的、圆环的已经实测过了，也重新换算(省得再横移探测一次)。"""
        st = self.store
        if st is None:
            return
        try:
            k = float(self.vision.scale('PICK')) / float(self.vision.scale('RING'))
        except Exception:
            return
        if not (0.3 < k < 4.0):
            return
        for g in ('arm', 'ch'):
            J = st.get('RING', g)
            if wheels and g == 'ch' and J is not None and st.get('PICK', g) is not None and st.synth('PICK') and not st.synth('RING'):
                st.put('PICK', g, J * k)
                st.set_synth('PICK', False)
                continue
            if st.get('PICK', g) is None and J is not None:
                st.put('PICK', g, J * k)
                if g == 'ch':
                    st.set_synth('PICK', st.synth('RING'))

    def _want_pick(self, item):
        v = self.vision
        if self.nogo or not self.cfg.get('learn_pick', True) or int(item.color) not in PICK_COLORS:
            return False
        if not hasattr(v, 'pick_px') or not hasattr(v, 'has_pick') or v.has_pick():
            return False
        return self._pick_tries < 3

    def _learn_pick(self, item, rec):
        """刚放下的物料就在爪子正下方：回到对准时的姿态、降到观察高度，量它顶面圆心在画面里的位置 = claw_px.PICK。
        以后取回、码垛时白心被物料盖住，就把物料顶面对到这个点。量完收臂(爪子伸在刚放的物料上方，不能这样挪底盘)。"""
        from vision import save_vision_cal
        self._pick_tries += 1
        v = self.vision
        self.log(f'    顺便量一次：{item.color_name}物料在爪子正下方时，它的顶面在画面里的位置(claw_px.PICK)。'
                 '以后取回、码垛直接认物料对准；只量这一次')
        try:
            self._return_to_pose(rec['a1'], rec['a2'])
            P = self.arm.params() or {}
            if 'ZOBRNG' in P:
                self.arm.lift(float(P['ZOBRNG']))
            self.sleep(0.3)
            p = v.pick_px(item.color, n=7)
            r = getattr(v, 'last_pick_r', None)
            why = self._pick_bad(p, r)
            if why:
                self.log(f'    ★ 没量成：{why}；下一个物料放下以后再量')
            else:
                path = self.cfg.get('vision_cal_file')
                extra = {'pick_r_px': round(float(r), 2)}
                if 'ZOBRNG' in P:
                    extra['pick_ZOBRNG'] = float(P['ZOBRNG'])
                if path:
                    save_vision_cal('PICK', p, path, extra)
                v.set_pick(p, r)
                rc = v.claw('RING')
                self.log(f'    claw_px.PICK = ({p[0]:.1f}, {p[1]:.1f})(圆环的爪子点 ({rc[0]:.1f}, {rc[1]:.1f}))，'
                         f'物料顶面半径 {r:.0f} 像素' + (f'，已存进 {path}' if path else ''))
        except ArmAbort:
            raise
        except (ArmError, VisionError) as ex:
            self.log(f'    ★ 量物料顶面位置出错：{ex}')
        self._stow_quiet()

    def _pick_bad(self, p, r):
        """刚量的 PICK 点哪里不对(返回原因)，没问题返回 None。"""
        if p is None or not r:
            return '认不到刚放下的物料(看看 vview 画面：物料是不是被爪子挡住太多、蓝色物料是不是和蓝色爪子连成了一块)'
        rc = self.vision.claw('RING')
        d = math.hypot(p[0] - rc[0], p[1] - rc[1])
        if d > 200.0:
            return f'认到的物料离圆环的爪子点 {d:.0f} 像素，太远，可能认错了'
        vc = getattr(self.vision, 'cfg', {}) or {}
        if vc.get('ring_rmax_cal') and vc.get('ring_outer_diam_mm') and vc.get('material_diam_mm'):
            r_ground = float(vc['material_diam_mm']) / 2.0 * float(self.vision.scale('RING'))
            if r < 0.85 * r_ground:
                return f'认到的物料顶面半径 {r:.0f} 像素，比放在地上应有的 {r_ground:.0f} 像素还小，可能认错了'
        return None

    def _ring_taken(self, zone=None, ring=None):
        """对准的那个圆环白心里明显放着东西(返回 True)；空的或看不清都返回 False(看不清就照常放)。"""
        return self._ring_taken_why(zone, ring) is not None

    def _ring_taken_why(self, zone=None, ring=None):
        """圆环占着的话按什么判断的：'record' = 记录里这个环上还留着我们放的物料(比如取回失败；不用看，一定是占着的)；
        'vision' = 摄像头看到白心里明显放着东西；空的或看不清返回 None(看不清就照常放)。"""
        if zone is not None and self.on_ring.get((zone, ring)):
            return 'record'
        f = getattr(self.vision, 'ring_centre_free', None)
        if f is None:
            return None
        try:
            return 'vision' if f() is False else None
        except Exception:
            return None

    def _return_to_pose(self, a1, a2, from_tray=False):
        """取完物料回到对准时记下的 ID1、ID2 角度。回读一下，差得多(> return_tol_deg)就再转一次，最多再转 2 次。
        from_tray=True：从车上转盘那边大幅转回来(每次走法一样，多转/少转的度数也差不多)：按前几次学到的提前补上，补完多半就不用再转一次。"""
        if a1 is None or a2 is None:
            raise ArmError('读不到对准时的手臂角度(A? 没回复)，不知道回哪里')
        pre = self.cfg.get('return_preload_deg') or [0.0, 0.0]
        learn = bool(self.cfg.get('return_bias', True)) and from_tray
        bias = self._ret_bias if learn else [0.0, 0.0]
        c1, c2 = a1 - bias[0], a2 - bias[1]                      # 前几次转回来总是多转/少转多少，这次提前补上
        zr = None
        if from_tray and self.cfg.get('return_lower', True):
            try:
                zr = float((self.arm.params() or {}).get('ZOBRNG'))       # (10-14)从转盘转回来时边转边降到看圆环的高度，再放
            except (TypeError, ValueError):
                zr = None
            except ArmAbort:
                raise
            except Exception:
                zr = None
        if abs(pre[0]) > 1e-6 or abs(pre[1]) > 1e-6:
            if zr is not None:
                self.arm.apz(c1 + pre[0], c2 + pre[1], zr)
                zr = None
            else:
                self.arm.ap(c1 + pre[0], c2 + pre[1])            # 先从反方向靠近，再回来，消除齿轮间隙
        if zr is not None:
            self.arm.apz(c1, c2, zr)
        else:
            self.arm.ap(c1, c2)
        tol = float(self.cfg.get('return_tol_deg') or 0.4)
        try:
            tol = max(tol, float((self.arm.params() or {}).get('ATOL', 0.0)) + 0.05)   # 比 ATOL 还小的差舵机不会再动
        except ArmAbort:
            raise
        except Exception:
            pass
        for k in range(3):
            self.sleep(0.2)
            try:
                b1, b2 = self.arm.read_angles()
            except ArmAbort:
                raise
            except ArmError as ex:                               # 物料已经在爪子里：读不到角度也照常放，不能带着物料收臂
                self.log(f'    回读角度失败({ex})，按已经回到位继续')
                return
            if k == 0 and learn and b1 is not None and b2 is not None:
                # 只学第一下(从转盘那边大幅转回来)：后面"再转一次"是小幅修正，走法不一样
                self._ret_bias = [max(-2.5, min(2.5, self._ret_bias[0] + 0.6 * (b1 - a1))),
                                  max(-2.5, min(2.5, self._ret_bias[1] + 0.6 * (b2 - a2)))]
            if b1 is None or b2 is None or (abs(b1 - a1) <= tol and abs(b2 - a2) <= tol):
                return
            if k == 2:
                self.log(f'    ★ 回不到对准姿态：还差 ID1 {b1 - a1:+.2f}°、ID2 {b2 - a2:+.2f}°，照常放')
                return
            self.log(f'    回到对准姿态差了 ID1 {b1 - a1:+.2f}°、ID2 {b2 - a2:+.2f}°，再转一次')
            self.arm.ap(a1, a2)

    # ------------------------------------------------------------------ 底盘沿车头方向在圆环间挪动
    def _ring1_dir(self, zone):
        """1 号环在 2 号环的哪一边(沿车头)：+1 = 前方，-1 = 后方；不知道返回 None。
        按存着的"底盘前进时画面往哪边动"(servo_cal.json 里 RING 的底盘 J)和 ring_order(画面里 1 号在左/上还是右/下)推，
        和 _survey 认环的办法一致(没看清圆环、按名义位置走时也不会前后弄反)。"""
        import numpy as np
        J = self.store.get('RING', 'ch') if self.store is not None else None
        if J is None:
            return None
        jf = np.asarray(J, float)[:, 1]
        s = float(jf[0] if abs(jf[0]) >= abs(jf[1]) else jf[1])
        if abs(s) < 0.1:
            return None
        lr = str((self.cfg.get('ring_order') or {}).get(zone, 'lr')).lower() != 'rl'
        if self.ring_rev:
            lr = not lr
        d = 1.0 if s > 0 else -1.0                               # 前进时圆环往右(下)移：左(上)边的环要往前开才到爪子下面
        return d if lr else -d

    def _ring_nominal(self, zone, ring):
        """圆环相对停车点(2 号环)沿车头的名义位置(毫米，正 = 前方)。方向能推出来就按 ring_order(_ring1_dir)，不然按 ring_offset_mm。"""
        d = self._ring1_dir(zone)
        if d is not None:
            return (2 - int(ring)) * float(self.cfg.get('ring_spacing_mm') or 150.0) * d
        offs = self.cfg['ring_offset_mm'].get(zone) or {}
        v = float(offs.get(str(ring), 0.0))
        return -v if self.ring_rev else v

    def _goto_ring(self, zone, ring):
        """底盘沿圆环那一排前后挪到这个圆环正对爪子的位置。
        到工位时摄像头看清过三个环(_survey)：按看到的位置直接开过去；没看清：圆环的名义位置 + 前面圆环对准时学到的停车误差。
        工位里不横移(chassis_strafe=False)：离圆环远一点近一点由手臂伸缩补。"""
        d = self.act.disp
        f = self.ring_f.get((zone, int(ring)))
        if f is not None:
            dF = int(round(f - d['F']))
            dS = self._ring_dS(zone, ring) if self._by_wheels(zone) else 0
            if abs(dF) < float(self.cfg.get('ring_move_min_mm') or 0.0):
                if not dS:
                    return                                       # 很近：手臂(只动车轮时：对准时的车轮)够得着，底盘不动
                dF = 0
            self.log(f'    底盘挪到环{ring}：' + (('前进' if dF > 0 else '后退') + f' {abs(dF)}mm' if dF else '') +
                     (('、' if dF else '') + f'横移 {dS:+d}mm(按看圆环时算的远近)' if dS else ''))
            self._zone_v_only = False
            self.act.chassis_move(dS, dF)
            return
        dF = int(round(self._ring_nominal(zone, ring) + self.learn['F'] - d['F']))
        dS = int(round(self.learn['S'] - d['S'])) if self.cfg.get('chassis_strafe', False) else 0
        if dF == 0 and dS == 0:
            return
        self.log(f'    底盘挪到环{ring}：前进 {dF:+d}mm' + (f'、横移 {dS:+d}mm' if dS else ''))
        self._zone_v_only = False
        self.act.chassis_move(dS, dF)

    def _ring_dS(self, zone, ring):
        """只动车轮：开到这个环时顺便横移多少毫米(让它离圆环那一排的远近也差不多对上，对准时少挪一次)。
        按看圆环时这个环在画面里的位置和车轮的 J(横移那一列要实测过)算；这个工位里已经对准过一个环就按那个环的横移位置加上两个环的差
        (只差车身斜的那一点)。横移累计不超过 ±ring_strafe_max_mm；算不了或不到 3mm 返回 0(对准时一起修，不单独走一条指令)。"""
        rec = self.ring_px.get((zone, int(ring)))
        st = self.store
        J = st.get('RING', 'ch') if st is not None else None
        if rec is None or J is None or st.synth('RING') or self.cfg.get('chassis_strafe', False):
            return 0
        js = np.asarray(J, float)[:, 0]                          # 横移 1mm 画面动多少(只用它垂直于圆环那一排的那一段)

        def s_of(k):
            r = self.ring_px.get((zone, int(k)))
            if r is None:
                return None
            jn = float(js @ r[1])
            if abs(jn) < 0.5 * float(np.hypot(*js)):
                return None                                      # 横移在画面里几乎是沿着圆环那一排动的：算不准
            return r[2] - r[0] / jn
        tgt = s_of(ring)
        ref = self._zone_s_ref
        if tgt is not None and ref is not None and ref[0] == zone and s_of(ref[1]) is not None:
            tgt = ref[2] + (tgt - s_of(ref[1]))
        if tgt is None:
            return 0
        smax = float(self.cfg.get('ring_strafe_max_mm') or 0.0)
        tgt = max(-smax, min(smax, tgt))
        dS = int(round(tgt - float(self.act.disp['S'])))
        return dS if abs(dS) >= 3 else 0

    def _obs_ring(self):
        """空爪摆到圆环上方的观察姿态。这个工位里已经对准过圆环、手臂还张着爪停在圆环上方(刚放完/刚看完三个环)：
        ID1 回到观察角度 A1P，ID2(伸缩)直接到前面对准时的远近，不先缩回 A2P 再伸出来——省一次大的伸缩修正。
        其他情况(刚从转盘回来、收过臂…)照常用 STM32 的 OBS。
        刚看完三个环、开到了第一个要放的环：按看圆环时量的远近和车身斜度，第一次测量之前手臂伸缩(必要时 ID1)就先补上(tilt_precorrect)。"""
        P = self.arm.params() or {}
        pre = self.cfg.get('ring_precorrect', True)
        at_obs, self._at_obs = self._at_obs, False
        p_now = None
        if at_obs and pre and self._ring_ready:
            p_now = self._relook()                               # 底盘挪了 100mm 以上：再看一眼，把车身斜度量准(顺便看到这个环在哪)
        d2 = self._predict_d2() if pre else None
        d1 = None
        full = self._arm_to(p_now) if (p_now is not None and 'A1P' in P and 'A2P' in P) else None
        if full is not None:
            d2, d1 = full                                        # 刚看到这个环在哪：ID2、ID1 一起转过去(相当于对准的第一步)
        if d2 is None and self._ring_ready and at_obs:
            return                                               # 刚看完三个环：手臂还在观察姿态(底盘挪过不影响)，不用再 OBS 一遍
        if d2 is None or not self._ring_ready or not all(k in P for k in ('A1P', 'A2P', 'ZHI', 'ZOBRNG')):
            self.arm.obs('RING', open_claw=True)
            self._a2_hint = P.get('A2P')
            self._a1_hint = P.get('A1P')
            self._ring_ready = True
            return
        if d1 is None:
            d1 = self._predict_d1(d2)
        env = self._a2_env() or self._a2_limits()
        a2 = min(env[1], max(env[0], float(P['A2P']) + d2))    # 伸缩的行程(A2P ± arm_limit_deg.id2，STM32 里 A2 的范围)
        d2 = a2 - float(P['A2P'])
        a1 = float(P['A1P']) + d1
        if at_obs:
            # 手臂还在观察姿态和高度(刚看完三个环)：直接转过去，不用升降
            if abs(d2) < 2.0 and abs(d1) < 0.35:
                return
            t = self._tilt or {}
            why = '按刚看到的位置' if full is not None else '按看圆环时量的远近'
            self.log(f'    {why}' + (f'(车身斜 {t["angle"]:+.1f}°)' if t.get('angle') is not None else '') +
                     f'，手臂先转过去：ID2 {d2:+.0f}°' + (f'、ID1 {d1:+.1f}°' if abs(d1) >= 0.05 else '') + '，再测')
            self.arm.ap(a1, a2)
            self._a1_hint, self._a2_hint = a1, a2                # AP 没回报角度时，对准的行程按转到的这个角度算(不是观察角度)
            return
        if abs(d2) >= 3.0:
            self.log(f'    手臂伸缩直接到前面对准时的远近(ID2 {d2:+.0f}°)' + (f'，ID1 {d1:+.1f}°' if d1 else ''))
        self.arm.lift(float(P['ZHI']))                       # 和 OBS 一样：先在搬运高度转过去(已经在这个高度就不动)
        self.arm.ap(a1, a2)
        self._a1_hint, self._a2_hint = a1, a2
        self.arm.lift(float(P['ZOBRNG']))

    def _predict_d2(self):
        """到现在这个底盘前后位置，对准圆环时 ID2 要比 A2P 多转多少度(= 车离圆环那一排远了/近了多少)。推不出来返回 None。
          对准过两个隔得够远的环：按这两个(以上)的直线推(车身和那一排不平行时远近随位置变；不往外推太远)；
          对准过一个：那一次的 + 看圆环时量的车身斜度 × 挪了多少(tilt_precorrect；量不到斜度就照那一次的)；
          还没对准过：按看圆环时量的(圆环那一排离爪子多远 + 斜度 × 挪了多少)换成 ID2 度数(要有手臂的 J)。"""
        rec = self._zone_d2
        t = self._survey_line()
        F = float(self.act.disp['F']) if self.act is not None else None
        b = None                                                 # 底盘每前进 1mm，ID2 要多转多少度(看圆环时量的斜度算的)
        if t is not None:
            b = max(-0.6, min(0.6, float(t['g'][0]) * float(t['jn'])))
        if len(rec) >= 2:
            fs = [float(r[0]) for r in rec]
            ds = [float(r[1]) for r in rec]
            if max(fs) - min(fs) >= 60.0:
                fb, db = sum(fs) / len(fs), sum(ds) / len(ds)
                var = sum((f - fb) ** 2 for f in fs)
                bf = sum((f - fb) * (d - db) for f, d in zip(fs, ds)) / var
                bf = max(-0.6, min(0.6, bf))                     # 每毫米最多 0.6°
                Fq = F if F is not None else fb
                pad = 0.6 * abs(bf) * 150.0 + 10.0
                return float(max(min(ds) - pad, min(max(ds) + pad, db + bf * (Fq - fb))))
        if rec:
            F1, d1 = float(rec[-1][0]), float(rec[-1][1])
            if b is not None and F is not None:
                return d1 + b * (F - F1)
            return d1
        if t is not None and F is not None:
            perp = float(t['perp0']) + float(t['jn']) * (F - float(t['F0']))
            lim = 0.8 * float(((self.servo.cfg if self.servo is not None else {}).get('arm_limit_deg') or {}).get('id2', 250.0))
            return max(-lim, min(lim, float(t['g'][0]) * perp))   # 只按手臂的 J 推的：不一下伸到头
        return None

    def _learn_from(self, zone, ring):
        d = self.act.disp
        self.learn = {'S': d['S'], 'F': d['F'] - self._ring_nominal(zone, ring)}

    def _zone_back(self, zone):
        """(10-20)工位做完挪回停车点：这一趟车只用 V 动过(2 号环车轮连续对准，之后车不动)、离停车点不远(raw_vel_back_mm 以内)就也只用 V
        挪回去(不转车头：S/F 走完会把车头转回 STM32 记着的方向)；照旧挪车对准过(发过 S/F)、远了、STM32 不认 V 才用 S/F。"""
        d = self.act.disp if self.act is not None else None
        if d is None:
            return
        ts, tf = self._zone_back_target(zone)
        tried = False
        if (abs(d['S'] - ts) >= 1 or abs(d['F'] - tf) >= 1) and getattr(self, '_zone_v_only', False) \
                and math.hypot(float(d['S']) - ts, float(d['F']) - tf) <= float(self.cfg.get('raw_vel_back_mm', 160.0)):
            tried = True
            try:
                if self._vel_goto(ts, tf, tol_mm=0.6):
                    self.log('    车轮挪回停车点(只用 V，不转车头)')
            except ArmAbort as ex:
                raise Abort(str(ex))
            except ArmError:
                pass
        d = self.act.disp
        if not tried or abs(d['S'] - ts) >= 1 or abs(d['F'] - tf) >= 1:
            self._return_to_stop(zone, (ts, tf))             # 远了 / V 没挪到：S/F

    def _anchor_mark(self, zone):
        """(10-26 用户"这个车在这四个停靠点停稳了但是后面第二轮运行夹取第二轮物料的时候还能在同样的地方吗")
        这一趟对准了(2 号环/原料盘中心)：记下车这时相对规定的停车点在哪(车轮挪的 + 规划时停车点挪过的)。对准以后车和工位/原料盘的
        相对位置每次都一样，所以第一次记的当基准：以后再来，和第一次差多少，这一次停的位置就和第一次差多少。"""
        if self.act is None:
            return
        d = self.act.disp
        fwd, left = getattr(self, '_cur_shift', None) or (0.0, 0.0)
        a = (float(d['S']) + float(left), float(d['F']) + float(fwd))
        self._anchor_now = (zone, a)
        self._anchor0.setdefault(zone, a)

    def _zone_back_target(self, zone):
        """做完挪回停车点挪到哪(底盘位移 S、F)。(0, 0) = 这一次停的地方。这一趟对准过、以前也对准过：挪到第一次停的地方
        (这一次比第一次偏了多少就多挪多少)，路线下一段照样按停车点算，车轮打滑的误差不再一圈圈累计。
        差得不到 zone_anchor_min_mm 不多挪；超过 zone_anchor_max_mm 多半认错了，不信，照旧挪回这一次停的地方。"""
        cfg = self.cfg
        now, a0 = self._anchor_now, self._anchor0.get(zone)
        if not cfg.get('zone_anchor', True) or now is None or now[0] != zone or a0 is None or now[1] is a0:
            return 0.0, 0.0
        es, ef = a0[0] - now[1][0], a0[1] - now[1][1]          # 这一次停的位置比第一次 往左 es、往前 ef
        e = math.hypot(es, ef)
        if e < float(cfg.get('zone_anchor_min_mm', 8.0)):
            return 0.0, 0.0
        name = {'RAW': '原料区', 'ROUGH': '粗加工区', 'TEMP': '暂存区'}.get(zone, zone)
        off = '、'.join(t for t in ((f'往{"前" if ef > 0 else "后"} {abs(ef):.0f}mm' if abs(ef) >= 0.5 else ''),
                                    (f'往{"左" if es > 0 else "右"} {abs(es):.0f}mm' if abs(es) >= 0.5 else '')) if t)
        if e > float(cfg.get('zone_anchor_max_mm', 60.0)):
            self.log(f'    ★ 这次停的位置比第一次来{name}时{off}，差 {e:.0f}mm：差太多，多半对准时认错了，不按它挪，照旧挪回这一次停的地方')
            return 0.0, 0.0
        self.log(f'    这次停的位置比第一次来{name}时{off}(一路车轮打滑累计的)：做完挪回第一次停的地方，'
                 '下一段路线照样按停车点算(误差不再一轮轮累计)')
        return -es, -ef

    def _return_to_stop(self, zone, target=(0.0, 0.0)):
        d = self.act.disp
        s, f = int(round(float(target[0]) - d['S'])), int(round(float(target[1]) - d['F']))
        if s or f:
            self.log(f'    底盘挪回停车点：前进 {f:+d}mm、横移 {s:+d}mm')
            try:
                self.act.chassis_move(s, f)
            except ArmAbort:
                raise
            except ArmError as ex:
                raise Abort(f'底盘挪回停车点失败：{ex}')

    def _recenter(self, min_mm):
        """底盘离停车点超过 min_mm 就挪回去。返回是否挪了。"""
        d = self.act.disp
        if (d['S'] ** 2 + d['F'] ** 2) ** 0.5 < min_mm:
            return False
        s, f = -int(round(d['S'])), -int(round(d['F']))
        self.log(f'    底盘挪回停车点：前进 {f:+d}mm、横移 {s:+d}mm')
        self.act.chassis_move(s, f)
        return True

    def _stow_quiet(self):
        """每个工位做完收臂(开车前手臂要收好)。收不好就停下(带着伸出的手臂开车不安全)。"""
        self._ring_ready = self._at_obs = False
        self._pre_pending = None
        try:
            self._stow_retry()
        except ArmAbort:
            raise
        except ArmError as ex:
            raise Abort(f'收臂失败：{ex}')

    # ------------------------------------------------------------------ START：回到启停区
    def _start(self, visit):
        if self.arm is not None:
            self.park()                                          # 收臂、升降停到开机高度 60(下次上电编码器能认出来)；夹着东西时不做
        s = self.stats
        self._ui('stage', 'DONE')
        self._ui('grab', s.final_grab_text())                    # 回家以后只显示做成的个数(赛规要"正确的数量")
        self._ui('place', s.final_place_text())
        self._ui('msg', f'T={self.elapsed():.0f}s')
        self.log(f'  ■ 回到启停区。用时 {self.elapsed():.0f}s；屏上显示 {s.final_grab_text()}、{s.final_place_text()}'
                 f'(抓了 {s.grab_total} 次、放了 {s.place_total} 次)')
        if self.stop_times:
            self.log('      各停车点用时：' + '  '.join(f'{st} {dt:.0f}s' for st, _r, _v, dt in self.stop_times))
        for zone, ring, stack, err in self.placements:
            self.log(f'      {zone} 环{ring}{" 码垛" if stack else ""}：对准误差 {err:.2f}mm' if err is not None else f'      {zone} 环{ring}')
