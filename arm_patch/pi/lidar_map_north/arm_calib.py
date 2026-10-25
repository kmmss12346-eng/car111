"""机械臂一次性标定向导：夹爪(PA2)、转盘(PB3)、ID1/ID2 两个舵机的各个姿态、升降的各个高度，一次调完。

在树莓派终端运行(先在 map_merge_live 里输入 q 退出，它占着串口)：

    cd ~/lidar_map_north
    python3 arm_calib.py

程序一步一步带着调。每一步先把机械臂摆到这一步的起始状态(要转舵机之前会先把升降抬到 ZHI，转的时候不会刮到东西)，
然后你输入下面的东西一点点调，满意了直接回车，就保存并进入下一步：

    +5  -5        这一步要调的东西加/减(舵机 = 度，升降 = 毫米，夹爪/转盘 = 微秒)
    350           直接到 350(不带符号的数 = 直接转到这个角度)
    2 =-862       ID2 的角度本来就是负数：前面加 = 才是直接转到 -862 度。
                  (10-16 起)带 + 或 - 的数一律是加减：2 -400 = ID2 再转 -400 度，不再当成"转到 -400 度"
    1 +5   2 -3   调 ID1 / ID2 舵机(摆姿态的那几步要同时调两个舵机)
    z -10         调升降(任何一步都能用)
    o   c         夹爪张开 / 夹紧
    u             两个舵机松开，用手直接摆；摆好以后回车 = 读出角度并保存
    r             重新读一下两个舵机现在的角度
    t             (车上转盘去圆环那几步 R1T2/R2T/R1T1/R2SPD)按 STM32 里已经保存的值真走一遍：去车上转盘 -> 去 2 号环 -> 回车上转盘
                  (码垛躲避那两步 STKA2/STKA1)按现在调的值走一遍码垛的路：去车上转盘 -> 绕电池盒到 STKA1 -> 往里缩到 STKA2
                  -> 缩着转到最远的环 -> 伸出去 -> 原路缩回来 -> 回车上转盘
    take          (放地上 ZPLC、码垛 ZSTK 那两步)和比赛时一样从车上转盘 1 号格夹一个物料(格子里先放好)，回到 2 号环上方：
                  物料在爪子里的位置和比赛时一模一样，量出来的高度才准(用手塞进爪子夹的偏高，量出的 ZPLC 偏低，比赛时就往地上顶)
    v             打开/关掉摄像头画面(vlive.py 的窗口：看物料、圆环认得怎么样)。原料盘上方姿态、ZGRAB、ZOBRAW、
                  圆环上方姿态、ZPLC、ZOBRNG 这几步会自动打开(要在树莓派桌面的终端里运行向导)
    g -16 -18     (看原料盘的那几步：A1G、ZGRAB、CLWOR、ZOBRAW)原料画面里的绿十字往左 16、往上 18 像素(往右、往下为正)，
                  存进 vision_cal.json，画面里马上跟着动。画面上 off 后面那两个数 = 物料离绿十字差多少：
                  先把爪子调到正好能夹住物料，再照抄这两个数，绿十字就到物料中心上了。只输入 g = 看绿十字现在在哪
                  (10-24 看圆环的那几步：A1P、R1、R3、ZPLC、ZOBRNG)挪的是圆环画面里的红十字(粗加工区、暂存区的爪子点)，
                  和原料的绿十字分开存、互不影响。爪子在 2 号环正上方时照抄画面上 off 后面那两个数，红十字就到 2 号环中心上了
    回车(或 ok)   保存当前值，下一步
    s             跳过这一步(保持原来的值)
    b             回上一步
    q             退出(已经保存的都保留；这一步调过还没按回车的不保存，会先提醒一次)
    h             再看一遍这个说明

保存 = 发给 STM32(SET) + 写进配置文件的 stm32_params(备份原文件为 .json.bak)。
以后每次启动 map_merge_live 都会自动把这些值发给 STM32，不用重新烧录。
(码垛躲避的 STKA1/STKA2 是树莓派自己用的：存进配置文件的 mission_cfg(stack_carry_a1/stack_carry_a2)，不发给 STM32。)

    python3 arm_calib.py --from ZGRAB    从某一步开始(只重调其中几项时用)
    python3 arm_calib.py --from A1P      只重调圆环那几步(2 号环、1 号环、3 号环上方的姿态……)
    python3 arm_calib.py --from STKA2    只重调码垛躲避(往里缩到多短、在哪里往里缩)
    python3 arm_calib.py --config xxx.json --port /dev/serial0

需要 STM32 里是带 CLWO/TT1/AEXT 参数的新程序(2026-10-08 以后的 arm.c)。
"""
import argparse
import json
import os
import re
import subprocess
import sys
import time
import unicodedata
from pathlib import Path

try:
    from arm_link import lift_slip_text
except ImportError:                      # 车上的 arm_link.py 还是旧的
    def lift_slip_text(line):
        return None

ROOT = Path(__file__).resolve().parent

# 夹爪、转盘脉宽的限位(和 main.c 里 CLAW_MIN_US/CLAW_MAX_US、TURNTABLE_MIN_US/TURNTABLE_MAX_US 一样)
CLAW_RANGE = (500, 2910)
TT_RANGE = (500, 2608)
MATERIAL_MM = 60.0            # 物料高度：码垛高度 ZSTK 默认 = ZPLC + 这个

TIMEOUT = dict(LIFT=25.0, AP=15.0, AF=10.0, CLAW=5.0, TT=5.0, U=3.0, SET=3.0, GET=5.0, PING=2.0)
TIMEOUT['A?'] = 3.0
TIMEOUT['LIFT?'] = 3.0
TIMEOUT.update(TRAY=40.0, APZ=40.0, SIDE=3.0)     # (10-21)试走车上转盘 <-> 圆环那条路

HELP = __doc__.split('然后你输入下面的东西', 1)[1].split('保存 = ', 1)[0]


def save_json(path, d, indent=2):
    """(10-23)一键启动就是整车电闸，标定完可能马上就关电：先写临时文件、fsync，再改名替换(不会只写了一半)。"""
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


class CalError(Exception):
    pass


class Quit(Exception):
    pass


# ---------------------------------------------------------------- 每一步
# kind：choice 选一个数 / claw 夹爪脉宽 / speed 夹爪速度 / tt 转盘脉宽 / lift 升降高度 / pose 两个舵机 / servo 一个舵机
# setup：这一步开始前把机械臂摆成什么样(见 Wizard._setup)
STEPS = [
    dict(key='AEXT', kind='choice', title='哪个舵机管前后伸缩', setup='high',
         text='输入 1 = ID1 管伸缩(ID2 管旋转)；输入 2 = ID2 管伸缩(ID1 管旋转)。\n'
              '不确定就先输入 w1 或 w2：那个舵机会来回动 5 度，看它是伸缩还是旋转。'),
    dict(key='CLWO', kind='claw', title='夹爪张开(默认)', setup='high',
         text='调到爪子张开得足够大、能从上面套住物料，又不会碰到旁边的东西。\n'
              '(10-21)比赛时张开只分两种：原料盘上方下爪前用 CLWOR，其他所有地方(车上转盘、圆环上放下……)都用车上转盘上方的 CLWOT。\n'
              '这一步只是默认值：后面 CLWOT、CLWOR 两步跳过(存 0)时用它。'),
    dict(key='CLWC', kind='claw', title='夹爪夹紧', setup='high',
         text='拿一个物料放在两个爪子中间，调到刚好夹稳(用手拽不下来)。\n'
              '不要比需要的更紧：夹得太紧，夹爪舵机会一直使劲顶着，很快发烫。'),
    dict(key='CLSPD', kind='speed', title='夹爪夹紧(夹取)的速度 CLSPD', setup='high',
         text='夹爪合上时转得多快(脉宽每秒变多少微秒)：越小越慢，0 = 一下子合上。\n'
              '直接输入数字(例如 800)，或者 +100 / -100，夹爪会张开再合上一次给你看。原来的做法是 0(一下子夹紧)。'),
    dict(key='CLSPO', kind='speed', title='夹爪张开(放下)的速度 CLSPO', setup='high',
         text='夹爪张开时转得多快：越小越慢，0 = 一下子张开。\n'
              '直接输入数字(例如 800)，或者 +100 / -100，夹爪会合上再张开一次给你看。'),
    dict(key='ZHI', kind='lift', title='搬运高度 ZHI', setup='zhi',
         text='手臂转动、搬运时用的高度：在这个高度，手臂怎么转、伸都碰不到车上的东西。一般就用最高点。'),
    dict(key=('A1D', 'A2R'), kind='pose', title='转盘上方的姿态', setup='turntable',
         text='把爪子移到转盘 1 号位的正上方(爪子中心对准放物料的格子)。\n'
              '用 1 +5 / 2 -5 调，或者输入 u 松开舵机用手摆，摆好回车。'),
    dict(key='TT1', kind='tt', slot=1, title='转盘 1 号位', setup='turntable',
         text='微调转盘，让 1 号格子的中心正好在爪子下面。觉得高处看不准，可以先 z -40 降低一点再看。'),
    dict(key='TT2', kind='tt', slot=2, title='转盘 2 号位', setup='turntable',
         text='微调转盘，让 2 号格子的中心正好在爪子下面。'),
    dict(key='TT3', kind='tt', slot=3, title='转盘 3 号位', setup='turntable',
         text='微调转盘，让 3 号格子的中心正好在爪子下面。'),
    dict(key='ZDROP', kind='lift', title='往转盘里放 / 从转盘里取的高度 ZDROP', setup='turntable_down',
         text='先输入 c 让爪子夹住一个物料，再用 -10 / -5 / -1 慢慢往下降，\n'
              '降到物料底部离转盘格子大约 2~5mm(松开就落进去)。从转盘里取物料时也用这个高度，爪子要能夹住格子里的物料。'),
    # (10-20 用户)夹爪张开分三个角度：车上转盘上方 CLWOT、原料盘上方 CLWOR、其他地方 CLWO。CLWOT/CLWOR 存 0 = 和 CLWO 一样
    dict(key='CLWOT', kind='claw', title='夹爪在车上转盘上方张开 CLWOT(除了原料盘上方，其他所有地方张开都用它)', setup='tray_open_down',
         needs_fw=True,
         text='手臂在转盘 1 号格上方、降到了 ZDROP。调爪子张开的大小：松开时物料能落进格子，张着下去也能套住格子里的物料，\n'
              '又不会碰到格子边。用 +50 / -50 调；c = 夹住看看，o = 回到正在调的张开。满意了回车。不用单独调(和 CLWO 一样)就输入 s。'),
    dict(key=('A1G', 'A2E'), kind='pose', title='原料盘上方的姿态', setup='raw',
         text='车按比赛时的位置停在原料盘前面，原料盘停着。把爪子移到原料盘上离车最近那一格(那个物料)的正上方：\n'
              '比赛时车就在这一格等，要的颜色转过来就夹(10-18 用户"最近那格"的做法)。用 2 +50 / 2 -50 调伸缩，1 +1 / 1 -1 调左右。\n'
              '之后在 map_merge_live 里 rteach RAW near 颜色号，车轮会按摄像头再细对；齿条伸缩到头了、差一两厘米也没关系，车轮会补。\n'
              '(10-23)画面里的绿十字不在爪子真正夹的地方：先输入 c 看爪子能不能正好夹住，夹得住以后输入 g 加画面上 off 后面那两个数，\n'
              '绿十字就挪到物料中心上了(rteach RAW near 就是把物料对到绿十字下面，所以要先把绿十字调准)。'),
    # (10-19 用户：电池盒挡着)夹起以后伸缩不动转到 A1T1 -> 伸到 A2T -> 伸着转到 A1T2 -> 缩回 A2R -> 转到车上转盘；伸缩都在两个圆盘外面
    dict(key='A1T1', kind='servo', sid=1, title='绕开电池盒①：开始伸长的角度 A1T1', setup='raw', needs_fw=True,
         text='夹起物料以后，伸缩先不动，手臂往车上转盘那边转，转到这个角度开始伸长(要已经离开原料盘那一块)。\n'
              '用 1 +10 / 1 -10 转到这里，回车保存。不需要绕开电池盒就输入 s 跳过后面几步(以后要关掉：set A2T 0)。'),
    dict(key='A2T', kind='servo', sid=2, title='绕开电池盒②：伸多长 A2T', setup='detour1', needs_fw=True,
         text='手臂在开始伸长的角度：用 2 +50 / 2 -50 把爪子伸到转过去碰不到电池盒的长度，回车保存。'),
    dict(key='A1T2', kind='servo', sid=1, title='绕开电池盒③：开始缩回的角度 A1T2', setup='detour2', needs_fw=True,
         text='伸着往车上转盘那边转：用 1 +10 / 1 -10 转过电池盒，转到开始缩回的地方(还没到车上转盘正上方)，回车保存。'),
    dict(key='A2SPD', kind='speed', title='绕开电池盒④：伸长、缩回的速度 A2SPD(度/秒)', setup='detour3', needs_fw=True, back_path=True,
         text='直接输入数字(例如 400)，或者 +100 / -100：伸缩会按这个速度缩回再伸出给你看。越大越快，0 = 和 ASPD 一样。\n'
              '满意了回车保存，手臂按原路(伸着转回开始伸长的角度再缩回)回到原料盘上方。'),
    dict(key='ZGRAB', kind='lift', title='在原料盘上夹物料的高度 ZGRAB', setup='raw_down',
         text='爪子张开，用 -10 / -5 / -1 慢慢往下降，降到张开的爪子正好套住物料、夹下去能夹稳的位置。\n'
              '可以输入 c 试夹一下，再 o 张开。爪子不能碰到盘子。'),
    # (10-23 用户"原料区域抓取物料之后再新加上一个往回缩的动作，因为我怕这个爪子挡住其他物料了")
    dict(key='A2GB', kind='servo', sid=2, title='原料区夹起以后先往回缩 A2GB', setup='raw_gb', needs_fw=True,
         text='比赛时在原料盘上夹紧以后：先直着抬一小段离开原料盘 → 一边升到最高一边把伸缩缩回到这里(ID1 不转) → 再转去车上转盘。\n'
              '手臂现在在原料盘上方夹物料的姿态、升到了最高(可以输入 c 夹一个物料看)。往回缩输入负数(这辆车 ID2 越小越短)，\n'
              '例如 -50；缩到爪子和物料都离开原料盘、转过去不会碰到别的物料，回车保存。不要这一下就输入 s 跳过(以后要关掉：set A2GB 0)。'),
    dict(key='CLWOR', kind='claw', title='夹爪在原料盘上方张开 CLWOR(夹原料之前)', setup='raw_open_down', needs_fw=True,
         text='手臂在原料盘上方、降到了 ZGRAB。调爪子张开的大小：要能套住物料，又不会碰到旁边的物料和盘子。\n'
              '用 +50 / -50 调；c = 夹住看看，o = 回到正在调的张开。满意了回车。不用单独调(和 CLWO 一样)就输入 s。'),
    dict(key='ZOBRAW', kind='lift', title='摄像头看原料盘的高度 ZOBRAW', setup='raw_obs', optional=True,
         text='摄像头对准原料盘时手臂停的高度，要能从画面里看到整个物料。\n'
              '已经用 vcal 标定过摄像头的话不要改它，输入 s 跳过(改了要重新 vcal)。'),
    dict(key=('A1P', 'A2P'), kind='pose', title='地上圆环上方的姿态(2 号环)', setup='ring',
         text='车按比赛时的位置停在 2 号环旁边(车身离工位色块的远近和比赛时停车点一样)，把爪子移到 2 号环中心的正上方。\n'
              '比赛时车轮先把 2 号环对到这里；接下来两步调 1、3 号环，这期间车别动。\n'
              '(10-24)画面里的红十字 = 粗加工区、暂存区这边的爪子点：比赛时车轮把 2 号环中心(黄十字)对到红十字上。\n'
              '爪子摆到 2 号环正上方以后，画面上 off 后面那两个数就是 2 号环离红十字多少：输入 g 加这两个数，红十字就挪到 2 号环中心上\n'
              '(只改圆环这边的，原料的绿十字不动)；先 z -40 降低看准爪子在不在正上方，再升回来、在原来的高度挪红十字。'),
    # (10-20 用户)1、3 号环：车不动，只转手臂过去。存的是比 2 号环(A1P/A2P)多转多少，粗加工区、暂存区共用(两个区圆环一样摆)
    dict(key='R1', kind='ringpose', ring=1, title='1 号环上方的姿态(比 2 号环多转多少)', setup='ring',
         text='车别动(上一步爪子在 2 号环正上方)。把爪子移到 1 号环中心的正上方：1 +5 / 1 -5 转、2 +50 / 2 -50 伸缩，\n'
              '或者 u 松开舵机用手摆，摆好回车。存的是比 2 号环多转了多少度，粗加工区、暂存区都用它。\n'
              '比赛时车只对准 2 号环一次，1、3 号环按这里的角度直接转过去放、取，车不动。高处看不准可以先 z -40 降低一点再看。\n'
              '以前存过的话手臂先转到存过的位置，对就直接回车。要重调：python3 arm_calib.py --from A1P，先看爪子正对 2 号环再做这两步。\n'
              '(10-24)红十字挪准了(上一步)的话，也可以在原来的高度看：把 1 号环中心(黄十字)对到红十字上。'),
    dict(key='R3', kind='ringpose', ring=3, title='3 号环上方的姿态(比 2 号环多转多少)', setup='ring',
         text='车别动。手臂转到了 2 号环上方(以前存过 3 号环就是存过的位置)：把爪子移到 3 号环中心的正上方(和上一步一样调)，摆好回车。\n'
              '(10-24)红十字挪准了的话，也可以在原来的高度看：把 3 号环中心(黄十字)对到红十字上。'),
    # (10-20 用户：夹着物料去圆环怕刮到电池盒)车上转盘 <-> 地上圆环这条路单独一组(和原料区 A1T1~A2SPD 一样的意思)。
    # (10-21 用户"从车上转盘到暂存区先转一个角度再伸长再转一个角度再缩回，让我自由调"；"ID1 连续转动，不要伸缩一下就停一下")按从车上转盘出去的顺序调；
    #   比赛时 ID1 一直转不停：转过 R1T2 开始伸到 R2T，转过 R1T1 开始缩到圆环那边要的长度；回车上转盘反过来(R2STG=1 才分段停下来走)。
    #   任何一步输入 t 按保存的值真走一遍
    dict(key='R1T2', kind='servo', sid=1, title='车上转盘去圆环①：先只转到的角度 R1T2', setup='rtray0', needs_fw=True,
         text='手臂在车上转盘上方(最高处)。先 o 张开、把一个物料塞进爪子，c 夹住(夹着物料调)。\n'
              '用 1 +10 / 1 -10 往圆环那边转(伸缩不动)，转到再转物料就要碰到电池盒的地方，回车保存。\n'
              '比赛时 ID1 一直转不停：转过这里开始伸到 R2T，转过 R1T1 开始缩到圆环那边的长度(回车上转盘反过来走)。\n'
              '这几步任何时候输入 t：按 STM32 里已经保存的值真走一遍(去车上转盘 -> 去 2 号环上方 -> 回车上转盘)，和比赛时一样。\n'
              '不用绕(直接转过去碰不到电池盒)：输入 s 跳过这四步(以后要关掉：set R2T 0)。'),
    dict(key='R2T', kind='servo', sid=2, title='车上转盘去圆环②：伸到多长 R2T', setup='rtray1', needs_fw=True,
         text='手臂停在刚才的角度(不转)：用 2 +50 / 2 -50 把爪子(连同物料)伸到转过去碰不到电池盒的长度，回车保存。'),
    dict(key='R1T1', kind='servo', sid=1, title='车上转盘去圆环③：伸着再转到的角度 R1T1', setup='rtray2', needs_fw=True,
         text='伸着往圆环那边转(伸缩不动)：用 1 +10 / 1 -10 转过电池盒，转到缩回(缩到 2 号环那边的长度)也碰不到电池盒的地方，回车保存。'),
    dict(key='R2SPD', kind='speed', title='车上转盘去圆环④：伸长、缩回的速度 R2SPD(度/秒)', setup='rtray3', needs_fw=True,
         back_path='ring',
         text='直接输入数字(例如 400)，或者 +100 / -100：伸缩会按这个速度缩到 2 号环的长度再伸回来给你看。越大越快，\n'
              '0 = 和 A2SPD 一样(A2SPD 也是 0 就按伸缩舵机的速度 EXTSPD)。满意了回车保存，手臂缩到 2 号环的长度、转到 2 号环上方。\n'
              '物料还夹着，下一步用得上。输入 t 按保存的值真走一遍。'),
    # (10-25 用户"降到最低设置 z=5 留出这么多冗余，还是会往下面压触发过载保护")以前这一步让人用手把物料塞进爪子夹住再量：
    #   比赛时物料是从车上转盘格子里按 ZDROP 夹出来的，在爪子里的位置不一样(ZDROP 那一步让物料底离格子 2~5mm，
    #   从格子里夹的就比那时往下多出这么多；手塞得深还要多)，量出来的 ZPLC 偏低几毫米，比赛时每次都往地上顶。改成 take 和比赛时一样夹
    dict(key='ZPLC', kind='lift', title='往地上放物料的高度 ZPLC', setup='ring_down',
         text='物料要和比赛时一样从车上转盘里夹：爪子里有物料先拿出来(o 张开)，在车上转盘 1 号格放一个物料，输入 take：\n'
              '手臂按比赛时的路去车上转盘、降到 ZDROP 夹起来、回到 2 号环上方(物料在爪子里的位置和比赛时一模一样)。\n'
              '再用 -10 / -5 / -1 慢慢往下降，降到物料底部刚好碰到地面(圆环里)，回车。\n'
              '(用手塞进爪子夹的不准：量出来偏低几毫米，比赛时就会往地上顶、触发升降的堵转保护)'),
    dict(key='ZSTK', kind='lift', title='码垛高度 ZSTK', setup='ring_down',
         text='在已经放好的物料上面再放一个时的高度，一般 = ZPLC + 物料高度(60mm)。\n'
              '可以在圆环里放一个物料，车上转盘 1 号格再放一个，输入 take 和比赛时一样夹出来，往下降到刚好叠上去。'),
    # (10-24 用户"第二层码放躲避的时候……往里面缩去躲避然后到位置了再伸过去不要一会缩一会伸")码垛时夹着物料去圆环，路上要经过
    #   已经叠了两层的环：绕过电池盒在 STKA1 停下、往里缩到 STKA2，缩着转到要放的环，再只伸出去。路上没有两层的垛照旧走。
    #   这两个值树莓派自己用：存进配置的 mission_cfg(store 是名字)，不发给 STM32
    dict(key='STKA2', kind='servo', sid=2, store='stack_carry_a2', title='码垛躲避①：往里缩到多短 STKA2(ID2)', setup='stk_a2',
         text='码垛(暂存区第二层)每一个都这样走(10-25：不管先放哪个环)：绕过电池盒停下(在 3 号环前面) -> 往里缩到这里 ->\n'
              '缩着转到要放的环 -> 再只伸出去(只缩一次、伸一次)。手臂已经按这条路走到了 2 号环上方(离转轴最近、最难躲的环)，\n'
              '缩在现在存的长度。爪子里没有物料就先 o 张开、塞一个物料、c 夹住；在 2 号环上叠两个物料看：用 2 -50 往里缩\n'
              '(这辆车 ID2 越小越短)，缩到夹着的物料不会碰到垛(或者已经缩到头)。也可以 1 +10 / 1 -10 转到 3 号、1 号环上方看看，\n'
              '回车只存伸缩(ID2)。下面"按三个环的角度算"是爪子中心离各个环中心多远：不到 50mm 的那个，物料要从垛顶上面过(升降够高才碰不到)。\n'
              '输入 t：按现在调的长度把码垛的路真走一遍(不用先保存)。不用躲：s 跳过(没调过 = 缩到车上转盘上方的长度 A2R)。'),
    dict(key='STKA1', kind='servo', sid=1, store='stack_carry_a1', title='码垛躲避②：在哪里往里缩 STKA1(ID1)', setup='stk_a1',
         text='手臂夹着物料从车上转盘出来、绕过电池盒，停在往里缩的地方(没调过就是绕电池盒的最后一个角度 R1T1)，已经缩到上一步的长度。\n'
              '看缩着的物料碰不碰电池盒：碰的话往圆环那边多转一点(1 +10 还是 1 -10 看下面那行)，要在第一个环前面；回车只存这里的 ID1。\n'
              '比 R1T1 还靠车上转盘的地方不行(那一段还在绕电池盒)：要早点缩，先把"车上转盘去圆环③ R1T1"调早。\n'
              '输入 t：按现在调的角度把码垛的路真走一遍(不用先保存)。'),
    dict(key='ZOBRNG', kind='lift', title='摄像头看圆环的高度 ZOBRNG', setup='ring_obs', optional=True,
         text='摄像头对准地上圆环时手臂停的高度。已经用 vcal 标定过的话输入 s 跳过。'),
    dict(key='A1H', kind='servo', sid=1, title='收起待命的姿态(ID1)', setup='stow',
         text='车跑起来时手臂待命的位置：爪子收在车身范围内，不会碰到任何东西。\n'
              '这一步只调 ID1(ID2 用转盘上方那一步的角度)。'),
]
STEP_KEYS = [s['key'] if isinstance(s['key'], str) else s['key'][0] for s in STEPS]
STORE_KEYS = {s['key']: s['store'] for s in STEPS if s.get('store')}   # (10-24)树莓派自己用的值 -> 配置 mission_cfg 里的名字

# 这几步自动打开摄像头画面(vlive.py)：raw = 看物料，ring = 看圆环
CAM_STEPS = {'A1G': 'raw', 'ZGRAB': 'raw', 'A2GB': 'raw', 'CLWOR': 'raw', 'ZOBRAW': 'raw', 'A1P': 'ring', 'R1': 'ring', 'R3': 'ring', 'ZPLC': 'ring', 'ZOBRNG': 'ring'}
REMOVED_STEPS = ('PL1', 'PL2')          # (10-20 用户"把 28、29 步的补偿去掉")以前的第一层/第二层放置补偿，比赛时也不再用
OPEN_KEYS = ('CLWO', 'CLWOR', 'CLWOT')  # 夹爪张开的三个角度(10-20)：其他地方 / 原料盘上方 / 车上转盘上方
RING_TEACH_FILE = 'ring_teach.json'     # 和 mission_cfg.ring_teach_file 一样：R1/R3 两步把 1、3 号环比 2 号环多转多少存在这里
VIEW_LOG = '/tmp/vlive_wizard.log'


def has_display():
    return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))


def _num(text):
    try:
        return float(text)
    except ValueError:
        return None


def config_from_run_live(folder=ROOT):
    """run_live.sh 里写了 --config 的话，用它(和 map_merge_live 用同一个配置文件)。"""
    sh = Path(folder) / 'run_live.sh'
    try:
        m = re.search(r'--config[ =]+["\']?([^\s"\']+)', sh.read_text(encoding='utf-8', errors='ignore'))
    except OSError:
        return None
    if not m:
        return None
    p = Path(m.group(1)).expanduser()
    return p if p.is_absolute() else Path(folder) / p


class SerialLink:
    """一行一条指令，等 DONE / ERR / PONG，中间的信息行收集起来(YAW100 丢掉)。"""

    def __init__(self, port):
        import serial
        self.err = serial.SerialException
        self.s = serial.Serial(port, 115200, timeout=0.1)
        time.sleep(0.2)
        self.s.reset_input_buffer()

    def request(self, text, timeout=5.0):
        self.s.reset_input_buffer()
        self.s.write((text + '\n').encode('ascii'))
        end = time.time() + timeout
        buf = b''
        info = []
        while time.time() < end:
            try:
                buf += self.s.read(256)
            except self.err:
                time.sleep(0.05)
            while b'\n' in buf:
                line, buf = buf.split(b'\n', 1)
                t = line.decode('ascii', 'ignore').strip()
                if not t or t.startswith('YAW100'):
                    continue
                if t.startswith(('DONE', 'ERR', 'PONG')):
                    return t, info
                info.append(t)
        return None, info

    def abort(self):
        try:
            self.s.write(b'!')
        except Exception:
            pass


class Wizard:
    def __init__(self, link, cfg_path, inp=input, out=print, view=False, popen=None, sleep=time.sleep):
        self.link = link
        self.sleep = sleep
        self.view = view                 # 摄像头那几步自动打开 vlive.py 的窗口
        self._popen = popen or subprocess.Popen
        self._viewer = None              # vlive.py 进程
        self._view_mode = None
        self._view_told = False
        self.cfg_path = Path(cfg_path) if cfg_path else None
        self.inp = inp
        self.out = out
        self.P = {}                      # STM32 里现在的参数
        self.spd = 0.0                   # 夹爪速度那两步正在试的值
        self.saved = {}                  # 这次保存过的
        self.claw = None                 # 夹爪现在的脉宽(只知道发过去的值)
        self.tune = None                 # 调夹爪那几步正在调的脉宽(中途 c 夹住看看、o 回来，不会丢)
        self.tt = None                   # 转盘现在的脉宽
        self.released = False            # 舵机松开了(用手摆)
        self.z_cmd = None                # 最后一次让升降去的高度(一样就不重复发)
        self.store_unset = set()         # (10-24)配置 mission_cfg 里还没调过的码垛躲避值(STKA1/STKA2)：self.P 里放的是比赛时的默认
        self.spacing = 150.0             # 相邻圆环中心距(mission_cfg.ring_spacing_mm)：算离各个环多远用

    # ------------------------------------------------------------ 和 STM32 说话
    def do(self, cmd, quiet=False):
        head = cmd.split(' ', 1)[0]
        rep, info = self.link.request(cmd, TIMEOUT.get(head, 10.0))
        for line in info:
            if line.startswith(('SERVO', 'LIFTBOOT', 'CLAWKEEP')) or 'STUCK' in line:
                self.out('  ' + line)          # 舵机卡住/没到位、夹着物料没让张开这类提示要让人看到
            slip = lift_slip_text(line)
            if slip:
                self.out('  ' + slip)          # (10-21)升降顶住没到要去的高度
        if rep is None:
            raise CalError(f'{cmd}：STM32 没回复(串口线？STM32 在跑吗？)')
        if not rep.startswith(('DONE', 'PONG')):
            raise CalError(f'{cmd}：STM32 回 {rep}')
        if not quiet:
            self.out(f'  > {cmd}')
        return info

    def angle(self, sid, tries=3):
        """读舵机角度。总线偶尔一次没回话(ERR READ)：歇一下再读，最多 tries 次。"""
        last = None
        for k in range(tries):
            try:
                for line in self.do(f'A? {sid}', quiet=True):
                    m = re.match(r'ANG (\d) (-?\d+(?:\.\d+)?)', line)
                    if m and int(m.group(1)) == sid:
                        return float(m.group(2))
                last = CalError(f'读不到 ID{sid} 的角度')
            except CalError as ex:
                if 'READ' not in str(ex):
                    raise
                last = ex
            self.sleep(0.05 + 0.05 * k)
        raise last

    def lift_now(self):
        for line in self.do('LIFT?', quiet=True):
            m = re.match(r'LIFT (\d) (-?\d+(?:\.\d+)?)', line)
            if m:
                return float(m.group(2)) if m.group(1) == '1' else None
        return None

    def lift_to(self, mm):
        mm = max(0.0, min(self.P.get('LFMAX', 100.0), mm))
        if self.z_cmd is None or abs(self.z_cmd - mm) > 0.05:
            self.z_cmd = None                  # 发的过程中出错(急停)就不知道在哪了
            self.do(f'LIFT {mm:g}')
            self.z_cmd = mm
        return mm

    def servo_to(self, sid, deg):
        self.released = False
        self.do(f'AF {sid} {deg:g}')
        try:
            return self.angle(sid)
        except CalError:
            self.out(f'  (转完读不到 ID{sid} 的角度，先当它到了 {deg:g}°)')
            return float(deg)

    def pose(self, a1, a2):
        self.released = False
        self.do(f'AP {a1:g} {a2:g}')
        self._last_pose = (float(a1), float(a2))

    def claw_to(self, us):
        us = int(round(max(CLAW_RANGE[0], min(CLAW_RANGE[1], us))))
        if us != self.claw:
            self.do(f'CLAW {us}')
            self.claw = us
        return us

    def claw_val(self, key):
        """参数 key 的夹爪脉宽；CLWOT 存的 0(没单独调)= CLWO；CLWOR 存的 0 = 和其他地方一样(CLWOT，它也是 0 才是 CLWO)，
        和固件的 Claw_OAt(RAW) 一样(10-21：以前 CLWOR=0 时这里用 CLWO，比赛时却是 CLWOT)。"""
        v = float(self.P.get(key, 0.0) or 0.0)
        if key not in ('CLWOR', 'CLWOT') or v >= CLAW_RANGE[0]:
            return v
        return self.claw_val('CLWOT') if key == 'CLWOR' else float(self.P['CLWO'])

    def open_us(self, st=None):
        """(10-20)这一步张开爪子用哪个角度：原料盘那几步 CLWOR、车上转盘那几步 CLWOT、其他 CLWO。"""
        kind = (st or {}).get('setup', '')
        if kind.startswith('raw'):
            return self.claw_val('CLWOR')
        return self.claw_val('CLWOT')                         # (10-21)其他地方都和车上转盘上方一样

    def tt_to(self, us):
        us = int(round(max(TT_RANGE[0], min(TT_RANGE[1], us))))
        if us != self.tt:
            self.do(f'TT {us}')
            self.tt = us
        return us

    def release(self):
        self.do('U 1')
        self.do('U 2')
        self.released = True

    # ------------------------------------------------------------ 保存
    def save(self, values):
        stm = {k: v for k, v in values.items() if k not in STORE_KEYS}
        mine = {k: v for k, v in values.items() if k in STORE_KEYS}     # (10-24)码垛躲避：树莓派自己用，不发给 STM32
        for name, v in stm.items():
            self.do(f'SET {name} {v:g}', quiet=True)
            self.P[name] = v
            self.saved[name] = v
        for name, v in mine.items():
            self.P[name] = v
            self.saved[name] = v
            self.store_unset.discard(name)
        if self.cfg_path is not None:
            cfg = json.loads(self.cfg_path.read_text(encoding='utf-8'))
            bak = self.cfg_path.with_suffix('.json.bak')
            if not bak.exists():
                bak.write_text(self.cfg_path.read_text(encoding='utf-8'), encoding='utf-8')
            if stm:
                sp = dict(cfg.get('stm32_params') or {})
                sp.update(stm)
                cfg['stm32_params'] = sp
            if mine:
                mc = dict(cfg.get('mission_cfg') or {})
                for name, v in mine.items():
                    mc[STORE_KEYS[name]] = round(float(v), 2)
                cfg['mission_cfg'] = mc
            save_json(self.cfg_path, cfg)
        elif mine:
            self.out('  ★ 没有配置文件：' + '、'.join(mine) + ' 存不下来(比赛时用不上)。用 --config 指定 map_merge_live 用的配置文件')
        self.out('  已保存：' + '  '.join(f'{k}={v:g}' for k, v in values.items()))

    # ------------------------------------------------------------ 开始前检查
    def check(self):
        self.do('PING', quiet=True)
        P = {}
        for line in self.do('GET', quiet=True):
            m = re.match(r'P (\w+)=(-?\d+(?:\.\d+)?)', line)
            if m:
                P[m.group(1)] = float(m.group(2))
        need = ['CLWO', 'CLWC', 'TT1', 'TT2', 'TT3', 'AEXT', 'A1G', 'A2E', 'ZHI', 'ZGRAB']
        miss = [k for k in need if k not in P]
        if miss:
            raise CalError('STM32 里还是旧程序(没有 ' + ' '.join(miss) + ' 参数)：先把新的 arm.c/main.c 编译烧录进去。')
        self.P = P
        if self.cfg_path is not None:
            sp = json.loads(self.cfg_path.read_text(encoding='utf-8')).get('stm32_params') or {}
            for k, v in sp.items():                # 配置里存的值 STM32 刚上电时还没收到：以配置为准
                if k in P and abs(P[k] - float(v)) > 1e-6:
                    try:
                        self.do(f'SET {k} {float(v):g}', quiet=True)
                        P[k] = float(v)
                    except CalError:
                        pass
        self._load_store()
        z = self.lift_now()
        if z is None:
            raise CalError('升降现在位置不明(LIFT? 回 0)：先在 map_merge_live 里看 arm LIFT ENC?，或者重新上电让它自动回到 60mm。')
        self.out(f'STM32 连接正常。升降现在 {z:g}mm，最高 {P.get("LFMAX", 100):g}mm。')
        self.out('夹爪 CLWO={CLWO:g}/CLWC={CLWC:g}  转盘 TT1={TT1:g} TT2={TT2:g} TT3={TT3:g}  伸缩舵机 AEXT={AEXT:g}'.format(**P))
        if 'CLWOR' in P and 'CLWOT' in P:
            self.out('夹爪张开：原料盘上方 CLWOR={:g}  车上转盘上方 CLWOT={:g}(0 = 和 CLWO 一样)'.format(P['CLWOR'], P['CLWOT']))

    # ------------------------------------------------------------ 每一步开始前摆好
    def _setup(self, st):
        P = self.P
        kind = st['setup']
        if kind in ('high', 'zhi'):
            self.lift_to(P['ZHI'])
            return
        self.lift_to(P['ZHI'])                     # 转舵机之前先抬高
        if kind in ('turntable', 'turntable_down'):
            self.pose(P['A1D'], P['A2R'])
            slot = st.get('slot', 1)
            self.tt_to(P[f'TT{slot}'])
        elif kind in ('raw', 'raw_down', 'raw_obs'):
            self.claw_to(self.claw_val('CLWOR'))
            self.pose(P['A1G'], P['A2E'])
        elif kind == 'raw_gb':                                # (10-23)A2GB：原料盘上方夹物料的姿态、升到最高(夹爪不动：夹着物料调)
            self.pose(P['A1G'], P['A2E'])
        elif kind == 'raw_open_down':                         # (10-20)CLWOR：在原料盘上方降到 ZGRAB 调张开
            self.pose(P['A1G'], P['A2E'])
            self.claw_to(self.claw_val(st['key']))
            self.lift_to(P['ZGRAB'])
        elif kind == 'tray_open_down':                        # (10-20)CLWOT：在车上转盘 1 号格上方降到 ZDROP 调张开
            self.pose(P['A1D'], P['A2R'])
            self.tt_to(P['TT1'])
            self.claw_to(self.claw_val(st['key']))
            self.lift_to(P['ZDROP'])
        elif kind in ('ring', 'ring_down', 'ring_obs'):
            o = self._ring_teach_now(st['ring']) if st['kind'] == 'ringpose' else None
            if o:                                          # 1、3 号环存过：从存过的位置开始(对就直接回车)
                self.pose(P['A1P'] + o[0], P['A2P'] + o[1])
            else:
                self.pose(P['A1P'], P['A2P'])
        elif kind == 'stow':
            self.pose(P['A1H'], P['A2R'])
        elif kind in ('detour1', 'detour2', 'detour3'):
            a1 = P.get('A1T2' if kind == 'detour3' else 'A1T1', 0.0)
            a2 = P.get('A2E') if kind == 'detour1' else P.get('A2T', 0.0)
            if abs(a1) > 0.5 and abs(a2) > 0.5:               # 前面一步跳过了(还是 0)就不动
                self.pose(a1, a2)
        elif kind == 'rtray0':                                 # (10-21)圆环那组：从车上转盘上方开始(夹爪不动：夹着物料调)
            self._to_tray()
        elif kind in ('rtray1', 'rtray2', 'rtray3'):
            self._tray_stage(kind)
        elif kind in ('stk_a1', 'stk_a2'):                     # (10-24)码垛躲避：按码垛的路走到这一步要看的地方(夹爪不动：夹着物料调)
            for key in self.store_unset:                       # 没调过的跟着这次刚调的 R1T1、A2R、1/3 号环走
                try:
                    P[key] = self._store_default(key)
                except (KeyError, TypeError, ValueError):
                    pass
            self._stack_path(kind)
        if kind in ('raw_obs', 'ring_obs'):
            self.lift_to(P['ZOBRAW' if kind == 'raw_obs' else 'ZOBRNG'])

    # ------------------------------------------------------------ 一步
    def _show(self, st):
        P = self.P
        k = st['kind']
        if k == 'pose':
            n1, n2 = st['key']
            return f'ID1={self.cur[1]:g}°  ID2={self.cur[2]:g}°   (原来 {n1}={P[n1]:g} {n2}={P[n2]:g})'
        if k == 'ringpose':
            d1, d2 = self.cur[1] - P['A1P'], self.cur[2] - P['A2P']
            o = self._ring_teach_now(st['ring'])
            return (f'ID1={self.cur[1]:g}°  ID2={self.cur[2]:g}°   比 2 号环(A1P/A2P)多转 ID1 {d1:+.2f}° ID2 {d2:+.2f}°' +
                    (f'   (原来存的 {o[0]:+.2f} {o[1]:+.2f})' if o else '   (还没存过)'))
        if k == 'servo':
            key = st['key']
            was = (f'原来没调：比赛时用 {self._store_default_text(key)}' if key in self.store_unset else f'原来 {key}={P[key]:g}')
            line = f'ID{st["sid"]}={self.cur[st["sid"]]:g}°   ({was})'
            if key == 'STKA2' and self.cur[2] is not None:
                line += self._stack_gap_text(self.cur[2])
            return line
        if k == 'claw':
            was = (f'原来 {st["key"]}=0，和 CLWO={P["CLWO"]:g} 一样' if st['key'] in ('CLWOR', 'CLWOT') and P[st['key']] < CLAW_RANGE[0]
                   else f'原来 {st["key"]}={P[st["key"]]:g}')
            now = '' if self.claw == self.tune else f'，夹爪现在在 {self.claw}us(' + ('o' if st['key'] in OPEN_KEYS else 'c') + ' 回到正在调的)'
            return f'{st["key"]} {self.tune:g}us{now}   ({was})'
        if k == 'speed':
            return f'{st["key"]}={self.spd:g} 微秒/秒(0 = 一下子到)   (原来 {P.get(st["key"], 0):g})'
        if k == 'tt':
            return f'转盘 {self.tt}us   (原来 {st["key"]}={P[st["key"]]:g})'
        if k == 'lift':
            extra = ''
            if st['key'] == 'ZSTK':
                extra = f'，建议 ZPLC+{MATERIAL_MM:g}={P["ZPLC"] + MATERIAL_MM:g}'
            return f'升降 {self.z:g}mm   (原来 {st["key"]}={P[st["key"]]:g}{extra})'
        return f'AEXT={P["AEXT"]:g}(2 = ID2 管伸缩，1 = ID1 管伸缩)'

    def _pending(self, st):
        """这一步调过、但还没按回车保存的值(文字)；没改过返回 None。"""
        P, k = self.P, st['kind']
        try:
            if k == 'lift' and self.z is not None and abs(self.z - P[st['key']]) > 0.05:
                return f'{st["key"]}={self.z:g}'
            if k == 'claw' and self.tune is not None and abs(self.tune - self.claw_val(st['key'])) > 0.5:
                return f'{st["key"]}={self.tune:g}'
            if k == 'speed' and abs(self.spd - P.get(st['key'], 0.0)) > 0.5:
                return f'{st["key"]}={self.spd:g}'
            if k == 'tt' and self.tt is not None and abs(self.tt - P[st['key']]) > 0.5:
                return f'{st["key"]}={self.tt:g}'
            if k == 'pose':
                n1, n2 = st['key']
                if any(self.cur[i] is not None and abs(self.cur[i] - P[n]) > 0.2 for i, n in ((1, n1), (2, n2))):
                    return f'{n1}={self.cur[1]:g} {n2}={self.cur[2]:g}'
            if k == 'servo':
                a = self.cur[st['sid']]
                if a is not None and abs(a - P[st['key']]) > 0.2:
                    return f'{st["key"]}={a:g}'
            if k == 'ringpose':
                o = self._ring_teach_now(st['ring'])
                d1, d2 = self.cur[1] - P['A1P'], self.cur[2] - P['A2P']
                if o is None or abs(d1 - o[0]) > 0.2 or abs(d2 - o[1]) > 0.2:
                    return f'{st["key"]}(ID1 {d1:+.2f}° ID2 {d2:+.2f}°)'
        except (KeyError, TypeError):
            return None
        return None

    # ------------------------------------------------------------ 摄像头画面(另开一个 vlive.py 进程，向导只用串口，不冲突)
    def _view_open(self, mode):
        if self._viewer is not None and self._viewer.poll() is None and self._view_mode == mode:
            return
        self._view_close()
        args = [sys.executable, str(ROOT / 'vlive.py')] + (['--ring'] if mode == 'ring' else ['1'])
        if self.cfg_path is not None:
            args += ['--config', str(self.cfg_path)]           # (10-23)和向导用同一份配置(g 挪的十字存在它说的 vision_cal.json)
        try:
            with open(VIEW_LOG, 'w') as logf:              # 子进程自己拿着这个文件，这边可以关
                self._viewer = self._popen(args, stdout=logf, stderr=subprocess.STDOUT, cwd=str(ROOT))
        except Exception as e:
            self.out(f'  (打不开摄像头画面：{e})')
            self._viewer = None
            return
        self._view_mode, self._view_told = mode, False
        self.out('  摄像头画面已打开(另一个窗口' + ('，看圆环' if mode == 'ring' else '，看物料，窗口里按 1~6 换颜色') +
                 ')；输入 v 关掉/再打开')
        if mode == 'ring':
            self.out('  画面里的红十字 = 粗加工区、暂存区这边的爪子点(比赛时 2 号环中心对到它)：输入 g 左右 上下 挪它，原料的绿十字不动')

    def _claw_shift(self, line):
        """(10-23 用户"把这个绿色的十字搞成可移动的…可以根据我的输入的数字改变这个绿色十字的位置")g 左右 上下：
        看原料盘的那几步挪原料画面里的绿十字(爪子点 claw_px.RAW)；(10-24 用户"粗加工区域和暂存区域的十字中心做成可调节的……
        两个互不干扰的独立的十字")看圆环的那几步挪圆环画面里的红十字(claw_px.RING)。往右、往下为正(像素)，只改这一个，
        存进 vision_cal.json，开着的 vlive 马上跟着动。"""
        mode = getattr(self, '_step_mode', None)
        if mode not in ('raw', 'ring'):
            raise CalError('g 只在看摄像头画面的那几步用：看原料盘的(A1G、ZGRAB、CLWOR、ZOBRAW)挪原料的绿十字，'
                           '看圆环的(A1P、R1、R3、ZPLC、ZOBRNG)挪圆环的红十字')
        kind, name = ('RAW', '原料的绿十字') if mode == 'raw' else ('RING', '圆环的红十字(粗加工区、暂存区)')
        parts = line.split()
        if len(parts) == 1:
            try:
                from vision import Vision
                from vlive import load_vision_cfg
                u, v = Vision(load_vision_cfg(self.cfg_path)).claw(kind)
                self.out(f'  {name}现在在 ({u:.0f}, {v:.0f}) 像素。')
            except Exception as e:
                self.out(f'  (读不出{name}的位置：{e})')
            self.out('  输入 g 左右 上下 挪它(像素，往右、往下为正)，例如 g -16 -18 = 往左 16、往上 18。\n' +
                     ('  画面上 off 后面那两个数 = 物料离绿十字差多少：先把爪子调到正好能夹住物料，再照抄这两个数，绿十字就到物料中心上了。'
                      if kind == 'RAW' else
                      '  画面上 off 后面那两个数 = 圆环离红十字差多少：爪子摆到 2 号环正上方以后照抄这两个数，红十字就到 2 号环中心上了。'))
            return None
        du, dv = (_num(parts[1]), _num(parts[2])) if len(parts) == 3 else (None, None)
        if du is None or dv is None:
            raise CalError('格式：g 左右 上下(像素，往右、往下为正)，例如 g -16 -18')
        if abs(du) > 200 or abs(dv) > 200:
            raise CalError('一次最多挪 200 像素')
        try:
            from vlive import claw_shift
            old, new, path = claw_shift(self.cfg_path, du, dv, kind=kind)
        except Exception as e:
            raise CalError(f'{name}没挪：{e}')
        self.out(f'  {name}：({old[0]:.0f}, {old[1]:.0f}) → ({new[0]:.0f}, {new[1]:.0f}) 像素，已存进 {Path(path).name}'
                 '(画面里马上跟着动；想挪回去就输入相反的数)。' +
                 ('绿十字改了，原料区要重新 rteach RAW near' if kind == 'RAW' else
                  '比赛时车轮把 2 号环中心对到这里(1、3 号环按调的角度跟着转过去，不用重调)'))
        if self._viewer is None or self._viewer.poll() is not None:
            self.out('  (摄像头画面没开着：输入 v 打开看看)')
        return None

    def _view_close(self):
        v = self._viewer
        self._viewer, self._view_mode = None, None
        if v is not None and v.poll() is None:
            try:
                v.terminate()
                v.wait(timeout=3)
            except Exception:
                try:
                    v.kill()
                except Exception:
                    pass

    def _view_check(self):
        """vlive 窗口自己退出了(摄像头被占、不在桌面终端里…)：说一次原因。"""
        v = self._viewer
        if v is None or self._view_told or v.poll() is None:
            return
        self._view_told = True
        why = ''
        try:
            lines = [l.strip() for l in open(VIEW_LOG, encoding='utf-8', errors='ignore') if l.strip()]
            why = lines[-1] if lines else ''
        except Exception:
            pass
        self.out(f'  (摄像头画面关了{("：" + why) if why else ""}。输入 v 再打开)')

    def _ring_teach_path(self):
        name = RING_TEACH_FILE
        try:
            mc = json.loads(self.cfg_path.read_text(encoding='utf-8')).get('mission_cfg') or {}
            name = mc.get('ring_teach_file') or name
        except Exception:
            pass
        p = Path(name).expanduser()
        return p if p.is_absolute() else (self.cfg_path.parent if self.cfg_path is not None else ROOT) / p

    def _ring_teach_now(self, ring):
        """ring_teach.json 里存的 ring 号环比 2 号环多转多少 (dID1, dID2)(粗加工区的，两个区存的一样)；没存过返回 None。"""
        try:
            v = json.loads(self._ring_teach_path().read_text(encoding='utf-8'))['ROUGH'][str(ring)]
            return float(v[0]), float(v[1])
        except Exception:
            return None

    def _to_tray(self):
        """(10-21)转到车上转盘上方(最高处)：STM32 按比赛时去车上转盘的路走(TRAY，绕电池盒)；旧程序/ARMOK=0 走不了就直接摆过去。"""
        P = self.P
        try:
            self.do('SIDE RING', quiet=True)
        except CalError:
            pass
        try:
            self.do('TRAY')
            self.z_cmd = None
            return True
        except CalError as e:
            self.out(f'  (STM32 走不了去车上转盘的路：{e}。直接摆到车上转盘上方)')
            self.lift_to(P['ZHI'])
            self.pose(P['A1D'], P['A2R'])
            return False

    def _tray_stage(self, kind):
        """(10-21)从车上转盘上方按分段的顺序走到这一步开始的地方：rtray1 = 只转到 R1T2；rtray2 = 再只伸到 R2T；rtray3 = 再只转到 R1T1。
        前面哪一步没调(还是 0)就停在那里。"""
        P = self.P
        seq = [(1, 'R1T2'), (2, 'R2T'), (1, 'R1T1')][:{'rtray1': 1, 'rtray2': 2, 'rtray3': 3}[kind]]
        for sid, key in seq:
            v = float(P.get(key, 0.0) or 0.0)
            if abs(v) <= 0.5:
                return
            try:
                if abs(self.angle(sid) - v) > 0.3:
                    self.servo_to(sid, v)
            except CalError:
                self.servo_to(sid, v)

    def _try_ring_path(self, st):
        """(10-21 用户"让我来自由调试")t：按 STM32 里已经保存的值真走一遍，和比赛时一模一样：去车上转盘上方(TRAY) -> 从车上转盘去 2 号环上方
        (APZ) -> 再回车上转盘上方；走完回到这一步开始的地方。这一步调了还没回车的值不算(先回车保存再 t)。"""
        P = self.P
        self.out('  试走(按已经保存的 R1T2/R2T/R1T1/R2SPD，ID1 一直转、路上伸缩；R2STG=1 才分段)：去车上转盘上方 -> 去 2 号环上方 -> 回车上转盘上方')
        if not self._to_tray():
            self.out('  ★ 试走要新的 STM32 程序(TRAY 指令)，并且 ARMOK=1(最后一步打开，或者 run_live 里 set ARMOK 1)')
            return
        self.do(f'APZ {P["A1P"]:g} {P["A2P"]:g} {P["ZHI"]:g}')
        self.do('TRAY')
        self.z_cmd = None
        if st.get('setup') in ('rtray1', 'rtray2', 'rtray3'):
            self._tray_stage(st['setup'])
        self._read_servos()
        self.out('  试走完了，回到这一步开始的地方')

    def _ring_detour_back(self):
        """(10-21)调完圆环那组绕电池盒(手臂在 R1T1、伸在 R2T)：只缩到 2 号环的长度，再转到 2 号环上方 A1P/A2P(和比赛时最后两段一样)。"""
        P = self.P
        try:
            self.lift_to(P['ZHI'])
            if abs(P.get('R2T', 0.0)) > 0.5 and abs(P.get('R1T1', 0.0)) > 0.5:
                self.servo_to(2, P['A2P'])
            self.pose(P['A1P'], P['A2P'])
        except CalError as e:
            self.out(f'  !! 回 2 号环上方失败：{e}')

    def _take_like_race(self):
        """(10-25)take：量 ZPLC/ZSTK 用的物料和比赛时一样从车上转盘 1 号格夹出来(比赛时 TAKE 就是这样夹的)：
        去车上转盘上方(TRAY，绕电池盒) -> 转盘转到 1 号格、张开(CLWOT) -> 降到 ZDROP -> 夹紧(CLWC) -> 升到 ZHI -> 回 2 号环上方(APZ)。
        TRAY/APZ 是 STM32 按比赛时的路走，要 ARMOK=1；没打开就不动，说怎么办。"""
        P = self.P
        if float(P.get('ARMOK', 0.0) or 0.0) < 0.5:
            self.out('  ★ take 要按比赛时的路去车上转盘、再回圆环(STM32 的 TRAY/APZ)，要先打开 ARMOK：q 退出，'
                     'run_live 里 set ARMOK 1，再 python3 arm_calib.py --from ZPLC')
            return
        self.out('  和比赛时一样从车上转盘 1 号格夹物料(格子里要先放好一个)：去车上转盘 -> 降到 ZDROP 夹紧 -> 升起 -> 回 2 号环上方')
        self.lift_to(P['ZHI'])
        if not self._to_tray():
            self.out('  ★ STM32 走不了去车上转盘的路(要新的 arm.c、ARMOK=1)：没夹')
            return
        self.tt_to(P['TT1'])
        self.claw_to(self.claw_val('CLWOT'))
        self.sleep(max(0.3, float(P.get('TTWAIT', 800.0) or 0.0) / 1000.0))   # 转盘转到位
        self.lift_to(P['ZDROP'])
        self.claw_to(P['CLWC'])
        self.sleep(max(0.2, float(P.get('CLWAIT', 400.0) or 0.0) / 1000.0))
        self.lift_to(P['ZHI'])
        self.do(f'APZ {P["A1P"]:g} {P["A2P"]:g} {P["ZHI"]:g}')
        self.z_cmd = None
        self.z = self.lift_now()
        if self.z is None:
            self.z = float(P['ZHI'])
        self._read_servos()
        self.out('  夹好了(物料在爪子里的位置和比赛时一样)，手臂在 2 号环上方。用 -10 / -5 / -1 往下降，降到物料底部刚好碰到，回车')

    # ------------------------------------------------------------ (10-24)码垛躲避 STKA1/STKA2(树莓派用，存在 mission_cfg)
    def _load_store(self):
        """从配置的 mission_cfg 读码垛躲避那两个值放进 self.P；没调过放比赛时的默认(和 mission_hooks 一样)，记在 store_unset。"""
        mc = {}
        if self.cfg_path is not None:
            try:
                mc = json.loads(self.cfg_path.read_text(encoding='utf-8')).get('mission_cfg') or {}
            except Exception:
                mc = {}
        try:
            self.spacing = float(mc.get('ring_spacing_mm') or 150.0)
        except (TypeError, ValueError):
            self.spacing = 150.0
        self.store_unset = set()
        for key, name in STORE_KEYS.items():
            try:
                v = float(mc.get(name))
            except (TypeError, ValueError):
                v = None
            if v is None:
                self.store_unset.add(key)
                try:
                    v = self._store_default(key)
                except (KeyError, TypeError, ValueError):
                    v = 0.0
            self.P[key] = v

    def _ring_set(self):
        """圆环那组绕电池盒调过(R2T 不是 0，ID2 管伸缩)：从车上转盘去圆环要先伸到 R2T 转过电池盒。"""
        P = self.P
        return float(P.get('AEXT', 2.0) or 2.0) == 2.0 and abs(float(P.get('R2T', 0.0) or 0.0)) > 0.5

    def _ring_poses(self):
        """{环号: (ID1, ID2)}：爪子在 2 号环(A1P/A2P)和存过的 1、3 号环正上方时的角度。"""
        P = self.P
        out = {2: (float(P['A1P']), float(P['A2P']))}
        for r in (1, 3):
            o = self._ring_teach_now(r)
            if o:
                out[r] = (out[2][0] + o[0], out[2][1] + o[1])
        return out

    def _far_ring(self):
        """离车上转盘最远的环(码垛去它要经过另外两个)：(环号, ID1, ID2)，和它路上要经过的环(先经过的在前)。"""
        poses, a1d = self._ring_poses(), float(self.P['A1D'])
        far = max(poses, key=lambda r: abs(poses[r][0] - a1d))
        lo, hi = sorted((a1d, poses[far][0]))
        passed = sorted((r for r in poses if r != far and lo + 1.0 < poses[r][0] < hi - 1.0), key=lambda r: abs(poses[r][0] - a1d))
        return (far, poses[far][0], poses[far][1]), passed

    def _a1_base(self):
        """往里缩最早能在哪(ID1)：绕电池盒调过就是 R1T1(那一段还在绕电池盒)，不然是车上转盘上方 A1D。"""
        P = self.P
        t1 = float(P.get('R1T1', 0.0) or 0.0)
        return t1 if self._ring_set() and abs(t1) > 0.5 else float(P['A1D'])

    def _store_default(self, key):
        P = self.P
        if key == 'STKA2':
            return float(P['A2R'])
        base = self._a1_base()
        if base != float(P['A1D']):
            return base
        (_r, f1, _f2), passed = self._far_ring()
        first = self._ring_poses()[passed[0]][0] if passed else f1
        return (float(P['A1D']) + first) / 2.0                   # 和 mission_hooks 一样：车上转盘和第一个环中间

    def _store_default_text(self, key):
        v = self._store_default(key)
        if key == 'STKA2':
            return f'车上转盘上方的长度 A2R={v:g}'
        return (f'绕电池盒的最后一个角度 R1T1={v:g}' if self._a1_base() != float(self.P['A1D'])
                else f'车上转盘和第一个环中间 {v:.1f}')

    def _stka1_bad(self, v):
        """STKA1 调在比赛时用不上的地方：说为什么；能用返回 None。"""
        P = self.P
        (far, f1, _f2), passed = self._far_ring()
        base, a1d = self._a1_base(), float(P['A1D'])
        sgn = 1.0 if f1 > a1d else -1.0
        if (v - base) * sgn < 0.0:
            return ('比绕电池盒的最后一个角度 R1T1 还靠车上转盘(那一段还在绕电池盒)' if base != a1d else '在车上转盘后面') + \
                   f'，比赛时会改在 {self._store_default("STKA1"):.1f}° 缩'
        if passed:
            r = passed[0]
            if (self._ring_poses()[r][0] - v) * sgn <= 2.0:
                return f'已经到了(或者过了) {r} 号环，比赛时会改在 {self._store_default("STKA1"):.1f}° 缩'
        return None

    def _go(self, sid, deg):
        """只转一个舵机到 deg(已经在那儿就不发)。"""
        try:
            if abs(self.angle(sid) - deg) <= 0.3:
                return
        except CalError:
            pass
        self.servo_to(sid, deg)

    def _stack_path(self, kind, vals=None, hint=True):
        """和比赛码垛时一样从车上转盘出发(夹爪不动：夹着物料调)：去车上转盘上方 -> 绕电池盒(只转到 R1T2 -> 只伸到 R2T)只转到往里缩的地方
        STKA1 -> 只往里缩到 STKA2；stk_a2 再缩着只转到 2 号环上方。一段一段走(每段只动一个舵机，ARMOK=0 也能走)。
        vals = {'STKA1': , 'STKA2': }：按这两个值走(t 试走完回到正在调的值)；不给按 self.P。"""
        P = self.P
        v = dict(STKA1=float(P['STKA1']), STKA2=float(P['STKA2']))
        v.update(vals or {})
        self._to_tray()
        if self._ring_set():
            t2 = float(P.get('R1T2', 0.0) or 0.0)
            if abs(t2) > 0.5:
                self._go(1, t2)
            self._go(2, float(P['R2T']))
        self._go(1, v['STKA1'])
        self._go(2, v['STKA2'])
        if kind == 'stk_a2':
            self._go(1, float(P['A1P']))
        if hint:
            sign = '+' if float(P['A1P']) > float(P['A1D']) else '-'
            self.out(f'  (往圆环那边转：1 {sign}10；往车上转盘那边：1 {"-" if sign == "+" else "+"}10)')

    def _try_stack_path(self, st):
        """t：按 STKA1/STKA2 真走一遍码垛的路，和比赛时一样：去车上转盘上方(TRAY) -> STM32 绕电池盒转到 STKA1(APZ) -> 只往里缩
        -> 缩着只转到最远的环 -> 只伸出去；再原路回来(夹着物料：只缩 -> 缩着转回 STKA1 -> TRAY)，回到这一步开始的地方。
        这一步正在调的值(还没回车)按现在舵机的角度算：调一下、t 试一下，满意了再回车。"""
        P = self.P
        (far, f1, f2), passed = self._far_ring()
        vals = dict(STKA1=float(P['STKA1']), STKA2=float(P['STKA2']))
        now = self.cur.get(st['sid']) if isinstance(getattr(self, 'cur', None), dict) else None
        tuning = now is not None and abs(now - vals[st['key']]) > 0.2
        if tuning:
            vals[st['key']] = float(now)
        a1, a2 = vals['STKA1'], vals['STKA2']
        via = '、'.join(f'{r} 号' for r in passed) or '(没有)'
        self.out(f'  试走(STKA1={a1:g} STKA2={a2:g}' + (f'，{st["key"]} 是正在调的、还没保存' if tuning else '') +
                 f'，和比赛码垛时一样)：去车上转盘上方 -> 绕电池盒转到 STKA1 -> 往里缩 -> '
                 f'缩着转到 {far} 号环(路上经过 {via}环) -> 伸出去 -> 原路缩回来 -> 回车上转盘上方')
        if not self._to_tray():
            self.out('  ★ 试走要新的 STM32 程序(TRAY 指令)，并且 ARMOK=1(最后一步打开，或者 run_live 里 set ARMOK 1)')
            self._stack_path(st['setup'], vals, hint=False)
            self._read_servos()
            return
        hold = float(P['R2T']) if self._ring_set() else float(P['A2R'])
        if abs(a1 - float(P['A1D'])) > 1.0:
            self.do(f'APZ {a1:g} {hold:g} {P["ZHI"]:g}')          # STM32 照比赛时绕电池盒
        self._go(2, a2)
        self._go(1, f1)
        self._go(2, f2)
        self.out(f'  到了 {far} 号环上方(比赛时在这里降到 ZSTK 码上去)。原路回去：')
        self._go(2, a2)
        self._go(1, a1)
        self.do('TRAY')
        self.z_cmd = None
        self._stack_path(st['setup'], vals, hint=False)
        self._read_servos()
        self.out('  试走完了，回到这一步开始的地方' + ('(还是正在调的值，满意了回车保存)' if tuning else ''))

    def _stack_gap_text(self, a2):
        """缩到 ID2 = a2 转过去时，爪子中心离路上那几个环的中心多远(按三个环的角度算手臂几何，见 mission_hooks.fit_arm_polar)。"""
        poses = self._ring_poses()
        if len(poses) < 3:
            return '\n  (1、3 号环还没调(R1/R3)：算不出离各个环多远)'
        key = tuple(sorted(poses.items())) + (self.spacing,)
        if getattr(self, '_fit_key', None) != key:
            try:
                from mission_hooks import fit_arm_polar
            except Exception:                              # 少了 numpy 之类：不显示
                return ''
            self._fit = fit_arm_polar(poses, self.spacing)
            self._fit_key = key
        fit = self._fit
        if fit is None or fit[2] > 6.0:
            return '\n  (三个环的角度对不上一条直线：算不出离各个环多远)'
        a, k, _res = fit
        _far, passed = self._far_ring()
        if not passed:
            return ''
        r_in = a + k * float(a2)
        parts = [f'{r} 号环 {abs(a + k * poses[r][1] - r_in):.0f}mm' for r in passed]
        out = '\n  按三个环的角度算，缩着转过去爪子中心离：' + '、'.join(parts) + '(不到 50mm 的，物料要从垛顶上面过)'
        try:
            zhi, zstk = float(self.P['ZHI']), float(self.P['ZSTK'])
            out += (f'\n  从垛顶上面过：ZHI {zhi:g} - ZSTK {zstk:g} - 物料高 {MATERIAL_MM:g} = {zhi - zstk - MATERIAL_MM:.0f}mm'
                    '(夹着的物料底比两层垛顶高这么多；ZSTK 时物料底正好在第一层顶上)')
        except (KeyError, TypeError, ValueError):
            pass
        return out

    def _read_servos(self):
        """读两个舵机现在的角度。读不到(总线接触不好)：按刚才发的姿态算，不让整个向导停下。"""
        last = getattr(self, '_last_pose', None)
        cur = {}
        for sid in (1, 2):
            try:
                cur[sid] = self.angle(sid)
            except CalError:
                if last is None:
                    raise
                cur[sid] = last[sid - 1]
                self.out(f'  (读不到 ID{sid} 的角度，按刚才转到的 {cur[sid]:g}° 算。老是读不到要查舵机的线)')
        self.cur = cur

    def step(self, i):
        """返回 +1 下一步，-1 上一步。"""
        st = STEPS[i]
        P = self.P
        k = st['kind']
        self.out('')
        self.out(f'==== 第 {i + 1}/{len(STEPS)} 步：{st["title"]} ====')
        self.out(st['text'])
        self._setup(st)
        mode = CAM_STEPS.get(st['key'] if isinstance(st['key'], str) else st['key'][0])
        if self.view and mode:
            self._view_open(mode)
        elif self._viewer is not None:
            self._view_close()
        self._step_mode = mode
        self.z = self.lift_now()
        if self.z is None:
            self.z = P['ZHI']
        self.cur = {1: None, 2: None}
        self._q_warned = False
        if k in ('pose', 'servo', 'ringpose'):
            self._read_servos()
        if k == 'claw':
            self.tune = self.claw_to(self.claw_val(st['key']))
        if k == 'speed':
            if st['key'] not in P:
                self.out(f'  STM32 里还没有 {st["key"]} 这个参数(是旧程序)：先烧录新的 arm.c，这一步跳过。')
                return +1
            self.spd = float(P[st['key']])
        if k == 'tt':
            self.tt_to(P[st['key']])
        if k == 'lift' and st['key'] in ('ZHI', 'ZOBRAW', 'ZOBRNG'):
            self.z = self.lift_to(P[st['key']])
        while True:
            self._view_check()
            self.out('  现在：' + self._show(st))
            try:
                raw = self.inp('  调整(回车=保存，h=说明)> ')
            except EOFError:
                raise Quit()
            line = unicodedata.normalize('NFKC', raw).replace('　', ' ').strip().lower()
            try:
                r = self._handle(st, line)
            except CalError as e:
                self.out(f'  !! {e}')
                continue
            if r is not None:
                return r

    def _handle(self, st, line):
        P = self.P
        k = st['kind']
        if line in ('', 'ok', 'y'):
            lim = {'ZPLC': 40.0, 'ZSTK': 100.0}.get(st['key'])
            if k == 'lift' and lim is not None and self.z > lim and getattr(self, '_hi_warned', None) != (st['key'], self.z):
                self._hi_warned = (st['key'], self.z)     # 升降还在高处就按了回车(10-10 实车把 ZPLC、ZSTK 都存成了 100)：先提醒
                self.out(f'  !! 现在升降在 {self.z:g}mm，{st["key"]} 存这么高，放物料时会从高处掉下去。'
                         '先输入 c 夹住物料，用 -10 / -5 / -1 降到刚好碰到' + ('地面' if st['key'] == 'ZPLC' else '下面那个物料') +
                         '再回车；确实要存这个值就再按一次回车')
                return None
            return self._accept(st)
        if line == 'q':
            pend = self._pending(st)
            if pend and not self._q_warned:
                self._q_warned = True
                self.out(f'  ★ 这一步改成了 {pend}，还没保存：按回车 = 保存(再输入 q 退出)；再输入 q = 不保存直接退出')
                return None
            raise Quit()
        if line == 's':
            self.out(f'  跳过，保持原来的值。')
            return +1
        if line == 'b':
            return -1
        if line in ('h', '?', 'help'):
            self.out(HELP)
            return None
        if line == 'o':                                   # 调张开那几步：回到正在调的；其他步：按这一步的地方张开(原料盘/转盘/其他)
            self.claw_to(self.tune if (k == 'claw' and st['key'] in OPEN_KEYS) else self.open_us(st))
            return None
        if line == 'c':
            self.claw_to(self.tune if (k == 'claw' and st['key'] == 'CLWC') else P['CLWC'])
            return None
        if line == 'u':
            self.release()
            self.out('  两个舵机松开了，用手摆好以后直接回车(会读出角度)。想先看看读数就输入 r。')
            return None
        if line == 't' and st['key'] in ('R1T2', 'R2T', 'R1T1', 'R2SPD'):
            self._try_ring_path(st)
            return None
        if line == 't' and st['key'] in ('STKA1', 'STKA2'):
            self._try_stack_path(st)
            return None
        if line == 'take' and st['key'] in ('ZPLC', 'ZSTK'):
            self._take_like_race()
            return None
        if line == 'v':
            if self._viewer is not None and self._viewer.poll() is None:
                self._view_close()
                self.out('  摄像头画面已关')
            else:
                self._view_open(getattr(self, '_step_mode', None) or 'raw')
            return None
        if line == 'r':
            self._read_servos()
            self.out(f'  ID1={self.cur[1]:g}°  ID2={self.cur[2]:g}°')
            return None
        if line == 'g' or line.startswith('g '):
            return self._claw_shift(line)
        if k == 'choice':
            if line in ('w1', 'w2'):
                sid = int(line[1])
                a = self.angle(sid)
                self.servo_to(sid, a + 5)
                self.servo_to(sid, a)
                return None
            if line in ('1', '2'):
                self.save({'AEXT': float(line)})
                return +1
            raise CalError('这一步输入 1 或 2(哪个舵机管伸缩)，或者 w1 / w2 让它动一下看看')
        if k == 'speed':
            v = _num(line)
            if v is None:
                raise CalError('这一步输入速度数字(例如 800)，或者 +100 / -100')
            want = self.spd + v if line[:1] in '+-' else v
            want = max(0.0, min(1500.0 if st['key'] in ('A2SPD', 'R2SPD') else 20000.0, want))
            self.do(f'SET {st["key"]} {want:g}', quiet=True)         # 先试，回车才存进配置
            self.spd = want
            if st['key'] in ('A2SPD', 'R2SPD'):
                ext = 'A2T' if st['key'] == 'A2SPD' else 'R2T'
                sp = want if want > 0.5 else (float(P.get('A2SPD', 0.0)) if st['key'] == 'R2SPD' else 0.0)
                if sp <= 0.5:
                    sp = float(P.get('EXTSPD', 0.0) or 0.0) or float(P.get('ASPD', 90.0))   # (10-21)和比赛时一样：都是 0 按伸缩舵机速度
                if abs(P.get(ext, 0.0)) > 0.5:
                    back = P['A2R'] if st['key'] == 'A2SPD' else P['A2P']        # 圆环那组：在 R1T1(过了电池盒)缩到 2 号环的长度
                    self.do(f'AFS 2 {back:g} {sp:g}')                    # 按新速度缩回再伸出
                    self.do(f'AFS 2 {P[ext]:g} {sp:g}')
                return None
            a, b = (P['CLWO'], P['CLWC']) if st['key'] == 'CLSPD' else (P['CLWC'], P['CLWO'])
            self.claw_to(a)
            self.claw_to(b)                                           # 这一下就是按新速度转的
            return None
        parts = line.split()
        axis, val = (parts[0], parts[1]) if len(parts) == 2 else (None, parts[0] if parts else '')
        if axis is None:
            axis = {'claw': 'c', 'tt': 't', 'lift': 'z', 'servo': str(st.get('sid', 1))}.get(k)
            if axis is None:
                raise CalError('这一步要说清楚调哪个：1 +5 (ID1) 或 2 -5 (ID2)，升降用 z -10')
        absolute = val[:1] == '='
        v = _num(val[1:] if absolute else val)
        if v is None or axis not in ('1', '2', 'z', 'c', 't'):
            raise CalError(f'看不懂 "{line}"。输入 h 看说明')
        # (10-16 用户：输入 2 -400 一会往前一会往后)带 + / - 的一律是加减；直接转到某个角度要写 =，例如 2 =-862
        rel = (not absolute) and val[:1] in '+-'
        if axis in ('1', '2'):
            sid = int(axis)
            base = None
            if rel:
                try:
                    base = self.angle(sid)
                except CalError:
                    base = self.cur.get(sid)              # 总线偶尔读不到：按上一次转到的角度算
                    if base is None:
                        raise
                    self.out(f'  (读不到 ID{sid} 的角度，按上一次的 {base:g}° 算；老是读不到要查 ID{sid} 的线)')
            want = base + v if rel else v
            got = self.servo_to(sid, want)
            self.cur[sid] = got
            if abs(got - want) > 1.0:
                self.out(f'  注意：ID{sid} 停在 {got:g}°，没到 {want:g}°(到限位了，或者被挡住了)')
        elif axis == 'z':
            self.z = self.lift_to(self.z + v if rel else v)
        elif axis == 'c':
            base = self.tune if (k == 'claw' and self.tune is not None) else (self.claw or P['CLWO'])
            want = base + v if rel else v
            got = self.claw_to(want)
            if k == 'claw':
                self.tune = got
            if abs(got - want) > 0.5:
                self.out(f'  夹爪到限位了({CLAW_RANGE[0]}~{CLAW_RANGE[1]}us，main.c 里 CLAW_MIN_US / CLAW_MAX_US)')
        elif axis == 't':
            want = (self.tt or P['TT1']) + v if rel else v
            got = self.tt_to(want)
            if abs(got - want) > 0.5:
                self.out(f'  转盘到限位了({TT_RANGE[0]}~{TT_RANGE[1]}us，main.c 里 TURNTABLE_MIN_US / TURNTABLE_MAX_US)')
        return None

    def _accept(self, st):
        k = st['kind']
        if k == 'choice':
            self.save({'AEXT': self.P['AEXT']})
            return +1
        if k == 'claw':
            v = float(self.tune)
            if st['key'] in ('CLWOR', 'CLWOT') and float(self.P.get(st['key'], 0.0)) < CLAW_RANGE[0] and abs(v - self.P['CLWO']) <= 0.5:
                self.out(f'  没改：{st["key"]} 还是 0(和 CLWO 一样)')
            else:
                self.save({st['key']: v})
            self.claw_to(self.open_us(st))         # 张开，免得一直夹着
        elif k == 'speed':
            self.save({st['key']: float(self.spd)})
            if st['key'] in ('CLSPD', 'CLSPO'):
                self.claw_to(self.P['CLWO'])
        elif k == 'tt':
            self.save({st['key']: float(self.tt)})
        elif k == 'lift':
            self.save({st['key']: round(self.z, 1)})
        elif k == 'pose':
            self._read_servos()
            n1, n2 = st['key']
            self.save({n1: self.cur[1], n2: self.cur[2]})
        elif k == 'servo':
            self._read_servos()
            key, v = st['key'], self.cur[st['sid']]
            if key in self.store_unset and abs(v - self.P[key]) <= 0.3:
                self.out(f'  没改：{key} 还是没调(比赛时用 {self._store_default_text(key)})')
            else:
                why = self._stka1_bad(v) if key == 'STKA1' else None
                if why:                                    # 比赛时用不上(会改在别处缩)：不存
                    self.out(f'  !! ID1 {v:g}° {why}：不存。往圆环那边转一点再回车')
                    return None
                self.save({key: v})
        elif k == 'ringpose':
            self._read_servos()
            d1, d2 = self.cur[1] - self.P['A1P'], self.cur[2] - self.P['A2P']
            path = self._ring_teach_path()
            try:
                d = json.loads(path.read_text(encoding='utf-8')) if path.exists() else {}
            except Exception:
                d = {}
            for zone in ('ROUGH', 'TEMP'):                 # 两个区圆环一样摆：共用
                z = d.setdefault(zone, {})
                z[str(st['ring'])] = [round(d1, 2), round(d2, 2)]
                z.pop('e' + str(st['ring']), None)         # 以前 rteach(摄像头)记的"画面里在哪"对不上了
                z['src'] = 'arm_calib'
                z['p2'] = [round(self.P['A1P'], 2), round(self.P['A2P'], 2)]
            save_json(path, d, indent=1)
            self.saved[st['key']] = (round(d1, 2), round(d2, 2))
            self.out(f'  已保存：{st["ring"]} 号环比 2 号环多转 ID1 {d1:+.2f}°、ID2 {d2:+.2f}°(存在 {path.name}，粗加工区、暂存区共用)')
        if self.released:
            self._read_servos()
            self.pose(self.cur[1], self.cur[2])     # 用手摆的：让舵机重新出力，停在摆好的位置
        if st['setup'].endswith('_down'):
            self.claw_to(self.open_us(st))         # 往下降的那几步：松开(物料留在转盘/盘子/地上)，再抬起来
            self.lift_to(self.P['ZHI'])
        return +1

    # ------------------------------------------------------------ 整个流程
    def _detour_back(self):
        """(10-19)调完绕开电池盒那几步：伸着转回开始伸长的角度 A1T1，再缩回 A2E、回到原料盘上方(不扫到电池盒)。"""
        P = self.P
        try:
            if abs(P.get('A2T', 0.0)) > 0.5 and abs(P.get('A1T1', 0.0)) > 0.5:
                self.lift_to(P['ZHI'])
                self.servo_to(2, P['A2T'])
                self.servo_to(1, P['A1T1'])
                self.servo_to(2, P['A2E'])
                self.pose(P['A1G'], P['A2E'])
        except CalError as e:
            self.out(f'  !! 回原料盘上方失败：{e}')

    def run(self, start=0):
        self.check()
        self.out(HELP)
        i = start
        try:
            while 0 <= i < len(STEPS):
                if STEPS[i].get('needs_fw') and STEPS[i]['key'] not in self.P:
                    self.out(f"(STM32 里没有 {STEPS[i]['key']} 参数，是旧固件：跳过这一步)")
                    i += 1
                    continue
                r = self.step(i)
                if STEPS[i].get('back_path') == 'ring':
                    self._ring_detour_back()
                elif STEPS[i].get('back_path'):
                    self._detour_back()
                i += r
                if i < 0:
                    i = 0
        except Quit:
            self.out('退出。')
        finally:
            self._view_close()
        try:
            self.lift_to(self.P['ZHI'])
            self.pose(self.P['A1H'], self.P['A2R'])
        except CalError as e:
            self.out(f'  !! 收臂失败：{e}')
        if self.saved:
            self.out('')
            self.out('这次保存的参数：' + '  '.join(f'{k}=' + ('/'.join(f'{x:g}' for x in v) if isinstance(v, tuple) else f'{v:g}') for k, v in self.saved.items()))
            if self.cfg_path is not None and any(not isinstance(v, tuple) and k not in STORE_KEYS for k, v in self.saved.items()):
                self.out(f'已写进 {self.cfg_path}(原文件备份为 .json.bak)，以后 map_merge_live 启动会自动发给 STM32。')
            mine = [k for k in STORE_KEYS if k in self.saved]
            if self.cfg_path is not None and mine:
                self.out('码垛躲避(' + '、'.join(mine) + f')存在 {self.cfg_path.name} 的 mission_cfg 里(树莓派比赛时用，不发给 STM32)，重开 run_live 生效。')
            if any(k in self.saved for k in ('R1', 'R3')):
                self.out(f'1、3 号环的角度存在 {self._ring_teach_path()}：比赛时车对准 2 号环以后不动，手臂直接按它转过去放、取'
                         '(粗加工区、暂存区共用)。重开 run_live 生效。')
            self.out('回到正常使用：bash run_live.sh')
        if i >= len(STEPS) and self.P.get('ARMOK', 0) < 0.5:
            try:
                ans = self.inp('全部标定完了。打开 ARMOK(允许自动夹放流程 GRAB/TAKE/PLACE 等)？输入 y 打开，其他键不打开> ')
            except EOFError:
                ans = ''
            if ans.strip().lower() == 'y':
                self.save({'ARMOK': 1.0})
        return self.saved


def main(argv=None):
    ap = argparse.ArgumentParser(description='机械臂一次性标定向导(先退出 map_merge_live)')
    ap.add_argument('--port', default='/dev/serial0')
    ap.add_argument('--config', default=None, help='默认：run_live.sh 里的 --config，没有就用 map_config_start2_roi.json')
    ap.add_argument('--from', dest='start', default=None, help='从哪一步开始，例如 --from ZGRAB')
    ap.add_argument('--no-view', action='store_true', help='摄像头那几步不自动打开画面')
    args = ap.parse_args(argv)

    cfg = Path(args.config) if args.config else (config_from_run_live() or ROOT / 'map_config_start2_roi.json')
    if not cfg.exists():
        print(f'找不到配置文件 {cfg}：参数只发给 STM32，不存(重新上电会丢)。可以用 --config 指定 map_merge_live 用的配置文件。')
        cfg = None
    else:
        print(f'参数会存进：{cfg}')
    start = 0
    if args.start:
        key = args.start.upper()
        if key in REMOVED_STEPS:
            print(f'{key}(放置补偿)这一步已经去掉了：比赛时按摄像头对准的位置(1、3 号环按 R1/R3 调的角度)直接放，不再加补偿。'
                  '要重调圆环那几步：python3 arm_calib.py --from A1P')
            return 1
        if key not in STEP_KEYS:
            print('--from 可以用：' + ' '.join(STEP_KEYS))
            return 1
        start = STEP_KEYS.index(key)
    try:
        link = SerialLink(args.port)
    except Exception as e:
        print(f'打不开串口 {args.port}：{e}\n是不是 map_merge_live 还开着？先在它里面输入 q 退出再运行。')
        return 1
    view = not args.no_view and has_display() and (ROOT / 'vlive.py').exists()
    if not args.no_view and not has_display():
        print('(不是在树莓派桌面的终端里运行：摄像头那几步不会自动打开画面)')
    wiz = Wizard(link, cfg, view=view)
    try:
        wiz.run(start)
    except CalError as e:
        print(f'停止：{e}')
        return 1
    except KeyboardInterrupt:
        link.abort()
        print('\n按了 Ctrl+C：已给 STM32 发急停。已经保存的参数都保留。')
        return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
