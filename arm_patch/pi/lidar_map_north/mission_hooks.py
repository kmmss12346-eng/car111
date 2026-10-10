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
import math
import time

import numpy as np

from arm_link import ArmLink, ArmActuators, ArmError, ArmAbort
from task_plan import parse_code, TaskError, role_of, Stats
from visual_servo import VisualServo, JacStore
from vision import Vision, VisionError, apply_vision_cal, load_vision_cal

try:
    from auto_run import Abort
except Exception:                       # 单独测试时没有 auto_run
    class Abort(Exception):
        pass


DEFAULTS = dict(
    enabled=True,
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
    raw_fix_max_mm=40.0,                # raw_chassis='F' 时一次对准车轮最多前后挪多少毫米
    raw_wheels_first=False,             # (10-10 用户：原料区只动爪子)True = 原料区对准先动车轮(沿车头方向前后小步慢慢挪，每次最多 wheels_step_mm、wheels_rpm 转/分)，
                                        #   差不到 wheels_min_mm 再动手臂；车轮不横移(不会压进原料区)。要先做过 vcal RAW 颜色号(不加 arm)，
                                        #   存了车轮和画面的对应关系才有用，没有就只动手臂。False = 只动手臂(以前的做法)
    raw_track_s=30.0,                   # 等要抓的物料转过来、停在爪子下面最多等多久(秒)。原料盘转 3 秒停 6 秒、三个物料轮流停过来，一圈约 27 秒
    raw_stop_s=4.0,                     # 原料盘每次停多久(秒)，没看到它完整停过一次之前先按这个算(保守一点)；看到以后按实测的
    raw_still_px=6.0,                   # 这一帧物料位置和前两帧比变了不到这么多像素 = 没在动(转盘转的时候一帧要走二三十像素)
    raw_motion_frac=0.004,              # 前后两帧画面(爪子以外)变了的地方超过这个比例 = 原料盘在转
    raw_static_s=9.0,                   # 还没做 vcal RAW(没有手臂和画面的对应关系)时：看到原料盘连续这么多秒都不转(关了)，才现场小幅动几下测
    raw_keepalive_s=12.0,               # 等物料转过来时手臂这么多秒没动过，就轻轻摆一下 ID1(规则：机器人停止运行 15 秒、等转盘时 23 秒本轮结束)；0 = 不摆
    raw_keepalive_deg=1.0,              # 摆多少度(ID1，摆过去马上回来)
    raw_frames=1,                       # 原料区对准时每次测量用几帧(一帧最快，停下来的时间有限)
    raw_max_iter=3,                     # 原料区对准最多修正几次(果断：差不多就下爪)
    raw_gain=1.0,                       # 原料区对准每次修正偏差的多少(用 vcal RAW 测好的 J，一下修到位，不留余量)
    raw_grab_max_mm=5.0,                # 原料盘停的时间到了就按当时的偏差下爪，只要不超过这个(爪子每边约 5mm 余量，再大会砸到物料)
    raw_tol_mm=3.0,                     # 原料区对准到多小就马上下爪(夹爪合上时会把物料夹正，不用像放圆环那么准)
    raw_any_order=True,                 # True(10-10 用户要的：爪子下面停的是哪个就夹哪个) = 不按任务码顺序：哪个颜色停在爪子附近就先夹哪个(放进它自己的槽)。
                                        #   规则按任务码顺序算"正确抓取"的 2 分，不按顺序可能拿不到这 2 分(放置分不受影响)
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
    zone_frames=2,                      # 粗加工区/暂存区对准时每次测量拍几帧(2 帧快；偏差刚好超出一点点时会自动再测一次取平均)
    zone_gain=1.0,                      # 粗加工区/暂存区对准时手臂每次修正掉偏差的多少(J 是 vcal 测好、每次对准都在修正的，一下修到位)
    zone_filter=True,                   # 粗加工区/暂存区对准时把几次测量合起来用(按动作推算 + 这次测的)：不按一次测量的噪声来回微调
    wheels_first=True,                  # (10-10 用户要的：尽量先慢慢动车)True = 粗加工区/暂存区对准时沿圆环那一排先让车轮前后小步慢慢挪(每次最多 wheels_step_mm)，
                                        #   差不到 wheels_min_mm 再动手臂；离圆环的远近还是手臂伸缩(车轮不横着往圆环那边挪，不压线)。
                                        #   模拟里比默认慢、失败多(车轮只能走整毫米，小步不准)，所以默认关；上车对比：mtest TEMP 1 force wheels
    wheels_step_mm=15.0,                # wheels_first：车轮每次最多挪多少毫米
    wheels_min_mm=4.0,                  # wheels_first：沿那一排的偏差比这个小就交给手臂(1.5 时模拟里失败多，4 好一些)
    wheels_rpm=60,                      # wheels_first：车轮小步挪的速度(转/分，慢一点准一点)
    ring_precorrect=True,               # 同一个工位里对准过一个环以后，去下一个环时手臂伸缩直接伸到同样的远近(不先缩回观察姿态再伸出来，省一次大的修正)
    tilt_precorrect=True,               # 车身和圆环那一排不平行(车停斜了)：看三个环时算出斜了几度，挪到每个环以后离圆环远/近多少，
                                        #   第一次测量之前手臂伸缩(必要时 ID1)就先补上
    tilt_warn_deg=3.0,                  # 斜到这么多度就在终端打 ★ 提醒"把车摆正"
    survey_frames=3,                    # 到工位看三个圆环时拍几帧
    servo=dict(),                                                # 覆盖 visual_servo.DEFAULTS
    servo_cal_file='servo_cal.json',
    vision_cal_file='vision_cal.json',  # vclaw 实测的爪子像素(claw_px)存在这里，覆盖上面的 claw_px
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
    #   TEMP  车头朝南，板转了 90°           -> 1 号在后方 -150(如果实际相反，把正负号对调)
    ring_offset_mm=dict(ROUGH={'1': 150.0, '2': 0.0, '3': -150.0}, TEMP={'1': -150.0, '2': 0.0, '3': 150.0}),
    pickback_fast=True,                 # 粗加工区取回时直接回到放下时记下的底盘位置和手臂角度，只测一次确认，容差内就不重新对准(省 4~5 秒/个)
    pickback_order='code',              # 粗加工区取回的顺序：'code'=按任务码顺序(最符合规则)；'reverse'=倒序；'near'=就近(底盘走得最少，省时间)
    learn_pick=False,                   # True = 还没量过 claw_px.PICK 时，放下第一个物料后回去再拍一张量一次(要多花几秒)；默认不量，取回按圆环对准
    stow_at_start=True,                 # go 开始时先把手臂收到待机姿态(STOW)
    return_tol_deg=0.4,                 # 回到记下的姿态后回读，差得比这个多就再转一次
    return_bias=True,                   # 回到记下的姿态时，按前几次"转回来总是多转/少转几度"提前补上(省掉"再转一次")
    return_preload_deg=[0.0, 0.0],      # 回到记下的姿态前，先从反方向多转这些度(ID1, ID2)再回来，消除齿轮间隙；0=不用
    stop_aliases=None,                  # 停车点名字不叫 QR/RAW/ROUGH/TEMP/START 时，例如 {'ROUGH': ['PROC']}
    screen=dict(code='t0', code2='t7', stage='t1', grab='t2', place='t3', msg='t4', b1='t5', b2='t6'),   # 任务码分两行(字高 ≥12mm 一行放不下)：t0=前两组，t7=后两组
)


A2_MIN, A2_MAX = -1220.0, -503.5       # STM32 里 A2(伸缩)参数的范围

BOOT_TEXT = {'ENC': '按编码器认的高度', 'NOCAL': '没标定，当成 60mm', 'NOENC': '读不到编码器，当成 60mm', 'POS': '按上次停的位置认的高度'}

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


def same_item(a, b):
    """两个 Item 是不是同一个物料(重新 mcode / 扫码以后 Item 对象换了新的，按批次、序号、颜色、圆环比)。"""
    if a is None or b is None:
        return False
    return a is b or (a.batch, a.index, a.color, a.ring) == (b.batch, b.index, b.color, b.ring)


def _first(p, measure):
    """第一次返回已经测好的 p(刚测过、还没动，不用再拍一遍)，以后每次调 measure()。"""
    box = [p]

    def m():
        if box[0] is not None:
            q, box[0] = box[0], None
            return q
        return measure()
    return m


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
        self._prep = {}                     # prepare() 做成了哪几步：'arm' / 'qr' / 'cam'
        self._t_alive = -1e9                # 上一次 keepalive 摆手臂的时刻
        self.holding = None                 # 爪子里现在夹着的物料(从转盘取出来、还没放下)
        self.maybe_holding = None           # 不为 None = 爪子里可能还夹着东西(原因)：不降升降、不张爪子
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
        self.ring_gate_px = None            # 对准时只认离爪子点这么多像素以内的圆环(不去追旁边那个)
        self._ring_ready = False            # 手臂现在就在这个工位的圆环上方、爪子张开(刚放完/刚看完圆环)：去下一个环不用整套 OBS
        self._at_obs = False                # 手臂就在观察姿态(刚 OBS 过、还没动过)：去第一个环不用再 OBS 一遍
        self._zone_d2 = []                  # 这个工位里每次对准好时 (底盘前后位移, ID2 比 A2P 多转的度数)：车离圆环那一排的远近
        self._ret_bias = [0.0, 0.0]         # 从转盘回到记下的姿态时 ID1、ID2 总是多转了多少度(学出来的，下次提前补上)

    # ------------------------------------------------------------------ 给 auto_run 的接口
    def adjust(self, stop, link, log):
        """到停车点后调姿态：目前什么也不做(车位已经由雷达定位好；精确对准在 task 里用摄像头做)。"""
        return None

    def task(self, stop, link, log):
        self.log = log or self.log
        role = role_of(stop, self.cfg.get('stop_aliases'))
        if role is None:
            self.log(f'    (停车点 {stop} 没有对应任务，跳过)')
            return
        self._stop_name = stop
        self._ensure(link)
        self.act.reset_disp()                       # 路线把车开到了新的停车点，位移从 0 重新算
        self.learn = {'S': 0.0, 'F': 0.0}
        self.visits[role] = self.visits.get(role, 0) + 1
        t_start = self.now()
        self._items_t = 0.0
        try:
            if (role in ('RAW', 'ROUGH', 'TEMP') and self.plan is None and not self.disabled
                    and self.cfg.get('qr_retry_at_stops', True)):
                self._qr_again()                    # QR 点没读到码：STM32 可能在路上读到了，再问一次
            self.run_role(role, self.visits[role])
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

    # ------------------------------------------------------------------ 给导航的接口：准备、计时、时间判断、保活、收臂停 60
    def prepare(self, link, log=None):
        """一键出发以后马上调(第二站动作之前)：把开跑前能做的先做掉，到了 QR 点就不用再等。
          机械臂初始化 + 收臂(STOW)、同步参数(和第一次 task 一样)；
          QR CLR：清掉 STM32 里存的旧任务码(上一轮/调试时扫过的)，屏上任务码写 ---(STM32 的 QR CLR 不清屏)；
          打开摄像头。
        调多次没关系(做成过的不再做)；出错只记日志、不抛异常(导航照常走；到 QR 点 task 会再初始化)。返回机械臂是否就绪。
        要在 ZONE LOCK(屏回到比赛画面、清屏)之后调：这里写的 --- 才不会被清掉。"""
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
                ok = self.arm.qr_clear()
                if ok is False:
                    self.sleep(0.1)
                    ok = self.arm.qr_clear()        # 串口偶尔一次没回话：再清一次
                if ok is False:
                    raise ArmError('QR CLR 没有回 DONE')
                self.arm.screen_reset()             # 屏可能刚清过(ZONE LOCK)：每个控件都重新写
                self._ui('code', '---')
                self._ui('code2', '---')
                self._ui('b1', 'B1 ---')
                self._ui('b2', 'B2 ---')
                self._ui('stage', 'RUN')
                self._show_stats()
                self._prep['qr'] = True
                self.log('  已清掉 STM32 里存的旧任务码(QR CLR)，屏上任务码先显示 ---')
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
        if self.home_eta_s is None or n <= 0:
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
                self.disabled = 'STM32 里没有机械臂参数(GET 里找不到 ARMOK)：STM32 程序是不是新版(arm.c)'
            elif params['ARMOK'] < 0.5:
                self.disabled = '机械臂参数还没标定(ARMOK=0)：标定好各姿态后 set ARMOK 1'
            else:
                known, mm = self.arm.lift_state()
                boot = getattr(self.arm, 'lift_boot', None)
                if known:
                    self.log(f'  升降已回零，现在 {mm:.1f}mm' + (f'(开机时{BOOT_TEXT.get(boot, boot)})' if boot else ''))
                    if boot in ('NOCAL', 'NOENC'):
                        self.log('  ★ 开机时没用编码器认高度，当成了 60mm：升降一定要停在 60mm 再关电(跑完/退出时会自动停 60；没停就输入 park)')
                elif self.cfg['lift_init'] == 'home':
                    self.arm.do('LIFT HOME')
                    self.log('  升降：驱动器回零完成')
                else:
                    self.disabled = '升降位置不知道了(急停打断过升降？)：把升降放到最低点，输入 arm LIFT ZERO'
                if not self.disabled and self.cfg.get('stow_at_start', True):
                    self._stow_retry()              # 开机时两个舵机是松的：出发前先收到待机姿态(升到 ZHI，ID1=A1H、ID2=A2R)
                    self.log('  手臂已收到待机姿态')
        except ArmAbort as ex:
            raise Abort(str(ex))
        except (ArmError, Exception) as ex:
            self.disabled = f'初始化机械臂出错：{ex}'
        if self.disabled:
            self.log(f'★ 本轮不做夹取/放置：{self.disabled}')
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
        obj = self.cfg['screen'].get(key)
        if obj:
            self.arm.screen(obj, text)

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
            return False
        if self.plan is None:
            self.log('    没有任务码，不能做夹取/放置')
            return False
        return True

    def _show_stats(self):
        self._ui('grab', self.stats.grab_text())
        self._ui('place', self.stats.place_text())

    def _recover(self, why):
        """单个物料出错后：尽量把手臂收好。收不好就停下整个路线(带着伸出的手臂乱走不安全)。"""
        self.log(f'    ★ {why}；收臂')
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

    def _put_back(self, item, why):
        """从转盘取出物料以后出错(回不到对准姿态、DROP 失败…)：把物料放回它的转盘槽，再收臂。
        不能夹着物料直接收臂：后面任何一步一张开爪子(OBS … O、TAKE、GRAB)，物料就掉在半路上。
        DROP 中途出错时爪子里可能已经空了，照样走一遍也没坏处(空爪在转盘上方张开)。
        放回也失败：爪子保持夹紧收臂，本轮不再做夹放(不再张开爪子)。"""
        self.log(f'    ★ {why}；爪子里夹着{item.color_name}，先放回转盘 {item.slot} 号槽')
        try:
            P = self.arm.params() or {}
            self.arm.lift(float(P['ZHI']))
            self.arm.ap(float(P['A1D']), float(P['A2R']))
            self.arm.tt(item.slot)
            self.arm.lift(float(P['ZDROP']))
            self.arm.claw(True)
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
        self._ui('stage', 'QR SCAN')
        search = bool(cfg.get('qr_search', True))
        code = self._qr_poll(float(cfg.get('qr_first_wait_s', 0.6)) if search else float(cfg['qr_timeout_s']))
        if not code and search:
            code = self._qr_search()
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
        """QR 点没读到码：到后面的停车点再问一次(STM32 在路上读到的码也会存着)。"""
        try:
            code = self.arm.qr()
        except ArmAbort:
            raise
        except ArmError:
            return
        if code:
            self.log(f'  ★ 到 {self._stop_name} 时 STM32 里有任务码了(路上读到的)：{code}')
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
        batch = 1 if visit <= 1 else 2
        try:
            self.arm.params(refresh=True)                         # 升降速度可能刚 set 过：算下爪提前量要用最新的
        except ArmAbort:
            raise
        except ArmError:
            pass
        if (self.cfg.get('raw_any_order') or getattr(self, 'raw_any_once', False)) and hasattr(self.vision, 'material_stream'):
            self._raw_any(self.plan.items(batch))
            return
        for item in self.plan.items(batch):
            self._check_abort()
            if self.disabled:                                     # 中途出了不能继续夹放的错(见 _put_back)
                break
            if self._no_time('grab', '抓取'):
                break
            left = self.in_tray.get(item.slot)
            if left is not None and not same_item(left, item):    # 前面没放出去的物料还在这个槽里：再放进去会砸在它上面
                self.log(f'    ★ 转盘 {item.slot} 号槽里还有没放出去的 {left.color_name}，{item.color_name} 不夹')
                self.stats.grab(False)
                self._show_stats()
                continue
            ok = False
            t_it = self.now()
            try:
                ok = self._grab_item(item)
            except (ArmError, VisionError) as ex:
                if isinstance(ex, ArmAbort):
                    raise
                self._recover(f'抓 {item.color_name} 出错：{ex}')
            self._note_item('grab', t_it)
            self.stats.grab(ok)
            if ok:
                self.in_tray[item.slot] = item
            self._show_stats()
        self._stow_quiet()

    def _raw_any(self, items):
        """不按任务码顺序抓：每次原料盘停下，要抓的几个颜色里哪个停在爪子附近、够得着，就先夹哪个，放进它自己的槽。"""
        cfg = self.cfg
        todo = []
        for item in items:
            left = self.in_tray.get(item.slot)
            if left is not None and not same_item(left, item):
                self.log(f'    ★ 转盘 {item.slot} 号槽里还有没放出去的 {left.color_name}，{item.color_name} 不夹')
                self.stats.grab(False)
                continue
            todo.append(item)
        self.log('  原料区：不按任务码顺序，哪个颜色停在爪子附近就先夹哪个(放进它自己的槽)：'
                 + '  '.join(f'{it.color_name}->{it.slot}号槽' for it in todo))
        md = self.vision.material_detector_obj() if hasattr(self.vision, 'material_detector_obj') else None
        if md is not None and hasattr(md, 'need_vmask') and any(md.need_vmask(it.color) for it in todo) and not self._warned_vmask:
            self._warned_vmask = True
            self.log('    ★ 没标定爪子区域(claw_mask.png)：蓝色/浅蓝物料挨着蓝色爪子时可能认错。赛前在 map_merge_live 里做一次 vmask')
        while todo:
            self._check_abort()
            if self.disabled:
                break
            if self._no_time('grab', '抓取'):
                break
            self._ui('stage', f'RAW GRAB {len(items) - len(todo) + 1}/3 ANY')
            self._recenter(min_mm=20.0)
            got = None
            t_it = self.now()
            try:
                self.arm.obs('RAW', open_claw=True)
                t_end = min(self.now() + float(cfg.get('raw_track_s', 30.0)), self._work_deadline())
                got = self._raw_stop_pick([(it.color, it.slot, it) for it in todo], 'grab', t_end, '抓')
            except (ArmError, VisionError) as ex:
                if isinstance(ex, ArmAbort):
                    raise
                self._recover(f'抓取出错：{ex}')
            self._note_item('grab', t_it)
            if got is None:
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

    def _grab_item(self, item):
        cfg = self.cfg
        self.log(f'  ▶ 抓第{item.batch}批 #{item.index + 1}：{item.color_name} -> 转盘 {item.slot} 号槽')
        self._ui('stage', f'RAW GRAB {item.index + 1}/3 {item.color_short}')
        self._recenter(min_mm=20.0)                 # 上一个物料对准时底盘挪过的话先挪回停车点，免得这个物料出了视野
        self.arm.obs('RAW', open_claw=True)
        md = self.vision.material_detector_obj() if hasattr(self.vision, 'material_detector_obj') else None
        if md is not None and hasattr(md, 'need_vmask') and md.need_vmask(item.color) and not self._warned_vmask:
            self._warned_vmask = True
            self.log(f'    ★ 没标定爪子区域(claw_mask.png)：{item.color_name}物料挨着蓝色爪子时可能认错。赛前在 map_merge_live 里做一次 vmask')
        if hasattr(self.vision, 'material_stream'):
            t_end = min(self.now() + max(float(cfg['raw_wait_s']), float(cfg.get('raw_track_s', 30.0))),
                        self._work_deadline())
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
        mode = str(cfg.get('raw_chassis') or 'off').upper()
        allow = still and mode in ('F', 'SF')
        kw = dict(chassis_axes='F', chassis_fix_max_mm=float(cfg.get('raw_fix_max_mm') or 40.0)) if allow and mode == 'F' else {}
        res = self.servo.run('RAW', lambda: self.vision.material_error(item.color), self.vision.scale('RAW'),
                             cfg['tol_mm']['RAW'], allow_chassis=allow, label=f'抓{item.color_short}', bounds=self.vision.bounds('RAW'),
                             confirm=False, **kw)
        self.log(f'    对准结果：{res}')
        if not res.ok and res.err_mm > cfg['accept_mm']['RAW']:
            if not allow:
                self.log('    (原料区对准只动手臂、车轮不动：物料停在手臂够不着的地方就跳过。车要停得让物料停下时在爪子附近)')
            self._recover(f'没对准({res.reason})，不夹，免得夹偏')
            return False
        self.arm.grab_here(item.slot)
        return True

    # ------------------------------------------------------------------ 原料盘转一会儿、停一会儿：物料一停下就对准、下爪
    def _raw_stream(self, color):
        """一帧一帧认物料的数据流(Vision.material_stream)；没有这个功能(旧的视觉模块)返回 None。"""
        f = getattr(self.vision, 'material_stream', None)
        return f(None if color is None else int(color)) if f is not None else None

    def _raw_close_delay(self, mode):
        """从发出下爪指令到夹爪合上要多久(秒)：升降从 ZOBRAW 降到 ZGRAB(和 STM32 算等待时间的办法一样，含 LFMRG) + 夹爪合上。"""
        P = self.arm.params() or {}
        try:
            sps = float(P['LFRPM']) / 60.0 * float(P['LFSPR'])
            lift = abs(float(P['ZOBRAW']) - float(P['ZGRAB'])) * float(P['LFPPM']) / sps + float(P.get('LFMRG', 0.0)) / 1000.0
        except (KeyError, TypeError, ValueError, ZeroDivisionError):
            lift = 0.45
        return lift + float(self.cfg.get('raw_claw_close_s', 0.15)) + (0.04 if mode == 'lift' else 0.02)

    def _raw_stop_len(self):
        """原料盘每次停多久：看到过完整的停(从停下到开始转)就按实测的中值打九折，没看到过按配置的 raw_stop_s。"""
        seen = sorted(self.raw_stops[-5:])
        if seen:
            return 0.9 * seen[len(seen) // 2]
        return float(self.cfg.get('raw_stop_s', 5.0))

    def _raw_predict_stop_start(self, t):
        """一开始就看到原料盘停着(没看到它停下的那一刻)：按前面量到的周期(停多久 + 转多久)推算这一次是什么时候停下的。
        推算不了(还没量到周期、太久以前量的、推算出来这会儿应该在转)返回 None。"""
        c = self.raw_cycle
        if not c.get('t_stop') or not self.raw_stops or not self.raw_moves:
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
        if single and cands[0][2] is not None and label == '抓':
            label = f'抓{cands[0][2].color_short}'           # 不按顺序抓、只剩最后一个：日志里也写上颜色
        tag = f'[{label}]'
        J = self.store.get('RAW', 'arm') if self.store is not None else None
        static_s = float(cfg.get('raw_static_s', 9.0))
        if J is None:
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
                            far_c = self._raw_reachable(J, np.array(q, float)) if (J is not None and str(cfg.get('raw_chassis') or 'off').upper() == 'OFF' and not self._raw_wheels()) else ''
                            seen.append((d, far_c, c, np.array(q, float)))
                    near = [x for x in seen if not x[1]]
                    if not near:
                        if ('none', stop_id) not in said:
                            said.add(('none', stop_id))
                            what = '、'.join(f'{x[2][2].color_name if x[2][2] is not None else x[2][0]} {x[0]:.0f}mm' for x in seen) or '一个也没看到'
                            self.log(f'  {tag} 这次停下，爪子附近没有要夹的颜色({what})：等下一次停')
                        tries = 2
                        continue
                    d, _f, cand, e = min(near, key=lambda x: x[0])
                    if cand[2] is not None:
                        lab = f'抓{cand[2].color_short}'
                        self.log(f'  {tag} 这次停在爪子附近的是{cand[2].color_name}：先夹它(放 {cand[1]} 号槽)')
                far = self._raw_reachable(J, e) if (J is not None and str(cfg.get('raw_chassis') or 'off').upper() == 'OFF' and not self._raw_wheels()) else ''
                if far:
                    if ('far', stop_id) not in said:
                        said.add(('far', stop_id))
                        self.log(f'  {tag} 这次物料停在别的位置(离爪子点约 {np.linalg.norm(e) / v.scale("RAW"):.0f}mm，手臂够不着)：等它转到爪子下面')
                    tries = 2
                    continue
                probe = False
                if J is None:
                    if not static:
                        continue                              # 没有对应关系：等确定转盘是一直停着的再说
                    probe = True
                    self.log(f'  {tag} 原料盘 {now - t_still:.0f} 秒都没转，当它是停着的：先小幅动几下测出手臂和画面的对应关系(只测这一次)')
                tries += 1
                if static:
                    deadline = now + 20.0
                elif left is not None:
                    deadline = now + left - T - 0.15                # 到这个时刻必须下爪(下爪到合上要 T 秒，再留一点余量)
                else:                                         # 一开始就停着，不知道已经停了多久：按刚看到它时才停下算(果断先试)
                    deadline = now + max(0.8, (t_still if t_still is not None else now) + stop_len - now - T - 0.15)
                self.log(f'  {tag} 停着，离爪子点 {np.linalg.norm(e) / v.scale("RAW"):.1f}mm：马上对准'
                         + ('' if left is None else f'(这次停下大约还剩 {left:.1f} 秒)'))
                if self._raw_align_go(cand[0], cand[1], mode, e, deadline, lab, probe=probe):
                    return cand
                J = self.store.get('RAW', 'arm') if self.store is not None else None
                if home[0] is not None and home[1] is not None:
                    self.arm.ap(home[0], home[1])             # 回到 OBS RAW 姿态等
                t_arm = self.now()
                stream.close()
                stream = self._raw_stream(color)              # 手臂动过：重新比画面(不然会把手臂动当成转盘在转)；好几个颜色时 color=None 只看画面
                hist, cnt, fresh = [], 0, 0
        finally:
            stream.close()

    def _raw_reachable(self, J, e):
        """按手臂和画面的对应关系 J 算：对准偏差 e(像素)要转多少度，超出手臂能转的范围就返回说明，够得着返回 ''。"""
        try:
            d = -np.linalg.solve(np.asarray(J, float), np.asarray(e, float))       # [ID2, ID1] 度
        except np.linalg.LinAlgError:
            return ''
        lim = (self.servo.cfg.get('arm_limit_deg') or {}) if self.servo is not None else {}
        out = []
        for i, key in enumerate(('id2', 'id1')):
            lm = 0.95 * float(lim.get(key, 250.0 if key == 'id2' else 12.0))
            if abs(d[i]) > lm:
                out.append(f'{"ID2" if i == 0 else "ID1"} 要转 {d[i]:+.0f}°，最多 ±{lm:.0f}°')
        return '，'.join(out)

    def _raw_wheels(self):
        """原料区这次对准先不先动车轮：配置(或 mtest RAW … wheels)要求，而且存了车轮和画面的对应关系(vcal RAW 颜色号)。"""
        want = bool(self.cfg.get('raw_wheels_first', True)) or bool(getattr(self, 'raw_wheels_once', False))
        if not want:
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
        cmode = str(cfg.get('raw_chassis') or 'off').upper()
        allow = cmode in ('F', 'SF')                         # 'off'：只动手臂，车轮不动
        if cmode == 'F':
            kw.update(chassis_axes='F', chassis_fix_max_mm=float(cfg.get('raw_fix_max_mm') or 40.0))
        if not probe and self._raw_wheels():
            # 先动车轮：沿车头方向前后小步慢慢挪找物料(不横移，不会压进原料区)，差不多了再用手臂补
            allow = True
            kw.update(chassis_axes='F', chassis_fix_max_mm=float(cfg.get('raw_fix_max_mm') or 40.0), wheels_first=True)
        res = self.servo.run('RAW', _first(tuple(e0), lambda: v.material_error(color, n=n)), v.scale('RAW'), tol,
                             allow_chassis=allow, label=label, bounds=v.bounds('RAW'), confirm=False, **kw)
        self.log(f'    对准结果：{res}')
        err = float(res.err_mm)
        if '误差变大' in (res.reason or ''):
            self._raw_bad_j = getattr(self, '_raw_bad_j', 0) + 1
            if self._raw_bad_j == 2:
                self.log('    ★ 连着两次越对越偏：存的原料区对应关系(J)可能不对(以前在转盘转的时候测的？)。'
                         '先让原料盘停转，物料放在爪子下面，重新做一次 vcal RAW 颜色号 arm')
            self.log('    ★ 越对越偏(转盘转起来了？)：不下爪，等下一次停')
            return False
        if not math.isfinite(err):
            self.log(f'    ★ 对准中途看不到物料了({res.reason})：不下爪，等下一次停')
            return False
        if err > grab_max:
            self.log(f'    ★ 偏差 {err:.1f}mm，超过 {grab_max:g}mm，下爪会砸到物料：不下爪，等下一次停')
            return False
        if self.now() > deadline + 0.25:
            self.log('    ★ 对准超时太多，原料盘马上要转了：不下爪，等下一次停')
            return False
        if mode == 'nogo':
            self.log(f'    nogo：对准了(偏差 {err:.1f}mm)，不夹。看看爪子是不是在物料正上方')
            return True
        self.log(f'    下爪(偏差 {err:.1f}mm' + ('' if res.ok else '，停的时间到了，按现在的位置夹') + ')')
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
        for item in items:
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
        self._return_to_stop('ROUGH')

    def _goto_recorded(self, zone, item):
        """取回时直接回到放下那一刻的底盘位置和手臂角度(物料就在那儿)。成功回到返回 True；没有记录或不允许返回 False。
        回去以后下一步的 servo.run 会先测一次：偏差在容差内就不动，直接夹。"""
        rec = self.pose_at.get((zone, item.slot))
        if not rec or not self.cfg.get('pickback_fast', True):
            return False
        d = self.act.disp
        dS, dF = int(round(rec['S'] - d['S'])), int(round(rec['F'] - d['F']))   # 放下时横移过(手臂够不着)的话也回到那里
        if dS or dF:
            self.log(f'    底盘回到放下时的位置：前进 {dF:+d}mm、横移 {dS:+d}mm')
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
        return list(placed)

    def _pickback_item(self, item, zone):
        cfg = self.cfg
        self.log(f'  ▶ 从{zone}取回 {item.color_name}(环{item.ring}) -> 转盘 {item.slot} 号槽')
        self._ui('stage', f'{zone[:5]} PICK {item.index + 1}/3 R{item.ring}')
        try:
            recorded = self._goto_recorded(zone, item)
            if not recorded:
                self._goto_ring(zone, item.ring)
                self._obs_ring()
            res, how = self._align_covered(item.color, 'PICK', f'取回{item.color_short}', confirm=False)
            if res is None:
                if not recorded:
                    self._recover('物料和圆环都看不到，不取')
                    return False
                self.log('    物料和圆环都看不到：按放下时记下的位置直接取')
                self._goto_recorded(zone, item)                  # 按物料对准时可能已经动过手臂/底盘：先回到记下的位置
            else:
                self.log(f'    对准结果(按{how})：{res}')
                if not res.ok and res.err_mm > cfg['accept_mm']['PICK']:
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
            self.arm.pick_here(item.slot)
        except ArmAbort:
            raise
        except (ArmError, VisionError) as ex:
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
        for item, ring, stack in todo:
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
        self._return_to_stop('TEMP')

    def _stack_target(self, item):
        """第二批的 item 在暂存区码垛到哪个环：第一批同色的那个物料放在暂存区的环(按记录确实放在那儿)；没有返回 None。"""
        low = self.plan.lower_item(item) if self.plan is not None else None
        if low is None:
            return None
        if any(same_item(x, low) for x in self.on_ring.get(('TEMP', low.ring)) or []):
            return low.ring
        return None

    # ------------------------------------------------------------------ 放到圆环(核心)
    def _place_item(self, item, zone, ring, stack, label):
        cfg = self.cfg
        key = 'STACK' if stack else 'RING'
        self.log(f'  ▶ {zone} 放 {item.color_name}(转盘{item.slot}号槽) 到环{ring}' + (' [码垛]' if stack else ''))
        self._ui('stage', f'{zone[:5]} PLACE {item.index + 1}/3 R{ring}' + (' S' if stack else ''))
        err_mm = None
        ok = False
        held = False                                             # 爪子里夹着从转盘取出来的物料
        if not stack and cfg.get('check_ring_empty', True) and self.on_ring.get((zone, ring)):
            names = '、'.join(it.color_name for it in self.on_ring[(zone, ring)])
            self.log(f'    ★ 按记录环{ring} 上还放着 {names}(程序记着之前放在这里、没取回；不是摄像头看到的)：'
                     '不去这个环、不放，免得砸上去；物料留在车上')
            self.stats.place(False)
            self._show_stats()
            return False
        try:
            self._goto_ring(zone, ring)
            self._obs_ring()                                     # 空爪到圆环上方：圆环不会被挡住
            if stack:                                            # 下面那层物料盖住了白心：先认它的顶面对准
                res, how = self._align_covered(item.color, key, label, confirm=bool(cfg.get('place_confirm', False)))
            else:
                res, how = self.servo.run('RING', self._ring_measure(), self.vision.scale('RING'), cfg['tol_mm'][key],
                                          allow_chassis=True, label=label, bounds=self.vision.bounds('RING'),
                                          confirm=bool(cfg.get('place_confirm', False)), **self._zone_servo_kw()), '圆环'
            if res is not None:
                self.log('    对准结果' + (f'(按{how})' if stack else '') + f'：{res}')
                err_mm = res.err_mm
            aligned = res is not None and (res.ok or res.err_mm <= cfg['accept_mm'][key])
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
                self.arm.take(item.slot)                         # 去转盘取物料
                held = True
                self.holding = item
                self._return_to_pose(a1, a2, from_tray=True)     # 回到记下的角度
                self._drop(stack)                                # 下降、松手、抬起
                held = False
                self.holding = None
                ok = True
                self._ring_ready = True                          # 爪子张着停在这个环上方：去下一个环不用整套 OBS
        except ArmAbort:
            raise
        except (ArmError, VisionError) as ex:
            if held:
                self._put_back(item, f'放置出错：{ex}')
            else:
                self._recover(f'放置出错：{ex}')
        self.stats.place(ok)
        if ok:
            self.in_tray.pop(item.slot, None)
            self.on_ring.setdefault((zone, ring), []).append(item)
            self.placements.append((zone, ring, bool(stack), err_mm))
        self._show_stats()
        if ok and not stack and self._want_pick(item):
            self._learn_pick(item, self.pose_at[(zone, item.slot)])
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

    def _zone_start(self, zone, items=()):
        """到粗加工区/暂存区：刷新 STM32 参数(可能刚 set 过)，看清三个圆环(算出到每个环底盘要挪多少)。
        没有要放的物料、或者时间到了，就不看(省时间)。"""
        self.ring_f = {}
        self.ring_gate_px = None
        self._ring_ready = self._at_obs = False
        self._zone_d2 = []
        self._tilt = None
        try:
            self.arm.params(refresh=True)
        except ArmAbort as ex:
            raise Abort(str(ex))
        except Exception:
            pass
        if not self.cfg.get('ring_survey', True) or not hasattr(self.vision, 'ring_list'):
            return
        if self._no_time('place', '放置', quiet=True) or (items and not any(same_item(self.in_tray.get(it.slot), it) for it in items)):
            return
        if not items:
            return
        try:
            self._survey(zone)
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
            self.store.put('RING', 'ch', [[-jf[1], jf[0]], [jf[0], jf[1]]])     # S 那一列用不到(不横移)，补一个和 F 垂直的
            self.store.set_synth('RING', True)                                # 记下"横移那一列是补的"：真要横移时重新测
            self.store.save()
        return jf, True

    def _survey(self, zone):
        """手臂摆到圆环上方(空爪)，一次看清画面里的圆环，按排列认出 1、2、3 号，算出每个环正对爪子时底盘的前后位置(self.ring_f)。
        画面里看到三个：直接认；两个：哪一头能看到却没有环，那一头就是这一排的尽头(圆环上放着物料时不这样认，可能只是没认出来)；
        两个隔了两个间距：中间那个没认出来，补上；一个、或者认不准：当作离爪子近的是 2 号(车停在 2 号环前面)。
        看到的圆环间距和"底盘走的距离"对不上(认错了/漏了)：这次不按看到的走，按配置里的间距走。
        顺便用圆环间距算出每毫米多少像素(没做 vclaw RING 时用它)。"""
        import numpy as np
        v, cfg = self.vision, self.cfg
        self.arm.obs('RING', open_claw=True)
        self._a2_hint = (self.arm.params() or {}).get('A2P')
        self._a1_hint = (self.arm.params() or {}).get('A1P')
        self._ring_ready = self._at_obs = True
        rings = v.ring_list(n=cfg.get('survey_frames') or None)
        if not rings:
            self.log('    ★ 画面里看不到圆环：按配置里的圆环间距走')
            return
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
        dist = np.hypot(pts[:, 0] - cu, pts[:, 1] - cv_)
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
        scale = sp_px / sp_mm
        vc = getattr(v, 'cfg', None)
        if isinstance(vc, dict) and not vc.get('ring_rmax_cal') and many:
            vc.setdefault('px_per_mm', {})['RING'] = scale       # 没做 vclaw RING：按圆环间距算的比例换算毫米
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

    def _ring_measure(self):
        """对准圆环用的测量：只认离爪子点 ring_gate_px 以内的圆环(看清过三个环以后才有)，不去追旁边那个。"""
        gate = self.ring_gate_px
        n = self.cfg.get('zone_frames') or None
        if gate:
            return lambda: self.vision.ring_error(n=n, max_px=gate)
        return lambda: self.vision.ring_error(n=n)

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

    def _a2_env(self):
        """ID2(伸缩)在工位里能到的范围(绝对角度)：观察姿态 A2P ± 视觉闭环的 arm_limit_deg.id2(约 ±44mm)，
        再和 STM32 的 A2 参数范围取交集。不知道 A2P 返回 None。"""
        P = (self.arm.params() or {}) if self.arm is not None else {}
        if 'A2P' not in P:
            return None
        lim = float(((self.servo.cfg if self.servo is not None else {}).get('arm_limit_deg') or {}).get('id2', 250.0))
        a2p = float(P['A2P'])
        return max(A2_MIN, a2p - lim), min(A2_MAX, a2p + lim)

    def _a1_env(self):
        """ID1 在工位里能到的范围(绝对角度)：观察角度 A1P ± arm_limit_deg.id1。不知道 A1P 返回 None。"""
        P = (self.arm.params() or {}) if self.arm is not None else {}
        if 'A1P' not in P:
            return None
        lim = float(((self.servo.cfg if self.servo is not None else {}).get('arm_limit_deg') or {}).get('id1', 12.0))
        return float(P['A1P']) - lim, float(P['A1P']) + lim

    def _drop(self, stack):
        """手臂已经对准：下降、松手、抬起。drop_retract=False 时不缩回伸缩舵机(接着去下一个环，工位做完再收臂)。"""
        if self.cfg.get('drop_retract', False):
            self.arm.drop(stack)
            return
        P = self.arm.params() or {}
        try:
            z = float(P['ZSTK' if stack else 'ZPLC'])
            zhi = float(P['ZHI'])
        except (KeyError, TypeError, ValueError):
            self.arm.drop(stack)
            return
        self.arm.lift(z)
        self.arm.claw(True)
        self.arm.lift(zhi)

    # ------------------------------------------------------------------ 白心被物料盖住时对准(取回、码垛)
    def _align_covered(self, color, key, label, confirm):
        """对准一个白心被物料盖住的圆环(取回：上面就是要夹的物料；码垛：上面是下面那层)：
        1 claw_px.PICK 量过、认得到这个颜色的物料：把物料顶面对到 PICK 点(最准，不用认圆环)
        2 否则认圆环(vclaw RING 量过圆环大小时，白心被盖住也能认最外圈)
        返回 (Result, '物料'/'圆环')；物料和圆环都看不到返回 (None, None)。"""
        v, cfg = self.vision, self.cfg
        tol, acc = cfg['tol_mm'][key], cfg['accept_mm'][key]
        n = cfg.get('zone_frames') or None
        if int(color) in PICK_COLORS and hasattr(v, 'has_pick') and v.has_pick():
            e = v.pick_error(color, n=n)
            if e is not None:
                self._seed_pick_jac()
                res = self.servo.run('PICK', _first(e, lambda: v.pick_error(color, n=n)), v.scale('PICK'), tol, allow_chassis=True,
                                     label=label, bounds=v.bounds('PICK'), confirm=confirm, **self._zone_servo_kw())
                if res.ok or res.err_mm <= acc:
                    return res, '物料'
                self.log(f'    按物料没对准({res.reason})，改认圆环')
            else:
                self.log('    认不到圆环上的物料，改认圆环')
        measure = self._ring_measure()
        e = measure()
        if e is None:
            return None, None
        return self.servo.run('RING', _first(e, measure), v.scale('RING'), tol, allow_chassis=True, label=label,
                              bounds=v.bounds('RING'), confirm=confirm, **self._zone_servo_kw()), '圆环'

    def _seed_pick_jac(self):
        """PICK(物料顶面)还没有自己的 J：用圆环的 J 按两个高度的像素/毫米之比换算一份，省得再小幅动几下探测。"""
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
        if abs(pre[0]) > 1e-6 or abs(pre[1]) > 1e-6:
            self.arm.ap(c1 + pre[0], c2 + pre[1])                # 先从反方向靠近，再回来，消除齿轮间隙
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
            if abs(dF) < float(self.cfg.get('ring_move_min_mm') or 0.0):
                return                                           # 很近：手臂够得着，底盘不动
            self.log(f'    底盘挪到环{ring}：' + ('前进' if dF > 0 else '后退') + f' {abs(dF)}mm')
            self.act.chassis_move(0, dF)
            return
        dF = int(round(self._ring_nominal(zone, ring) + self.learn['F'] - d['F']))
        dS = int(round(self.learn['S'] - d['S'])) if self.cfg.get('chassis_strafe', False) else 0
        if dF == 0 and dS == 0:
            return
        self.log(f'    底盘挪到环{ring}：前进 {dF:+d}mm' + (f'、横移 {dS:+d}mm' if dS else ''))
        self.act.chassis_move(dS, dF)

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
        env = self._a2_env() or (A2_MIN, A2_MAX)
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

    def _return_to_stop(self, zone):
        d = self.act.disp
        s, f = -int(round(d['S'])), -int(round(d['F']))
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
