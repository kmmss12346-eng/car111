/* arm.c  机械臂总控 v3
 *
 * 硬件对应：
 *   升降      Emm 步进电机 ID5(和轮子 1~4 号同一路 UART4，皮带升降)
 *   前后伸缩  飞特总线舵机 ID2      (main.c 里的 Servo_Move / Servo_Start，限位在那里)
 *   整臂旋转  飞特总线舵机 ID1      (同上)。哪个舵机管伸缩由参数 AEXT 决定(默认 2 = ID2 伸缩)
 *   夹爪      PA2  TIM2 通道 3      (main.c 里的 Claw_Set；张开/夹紧的脉宽是参数 CLWO / CLWC；
 *                                   原料盘上方、车上转盘上方张开另有 CLWOR / CLWOT，0 = 和 CLWO 一样)
 *   转盘      PB3  TIM2 通道 2      (main.c 里的 Turntable_Set；三个位置的脉宽是参数 TT1 / TT2 / TT3)
 *   扫码模块  UART5 (PC12/PD2)   GM65，出厂 9600 8N1
 *   串口屏    USART2(PD5/PD6)    淘晶驰 TJC4832T135，出厂 9600
 *
 * 高度约定：升降"最低点 = 0 mm"，往上为正数(数字越大越高)。所有姿态都是参数，用 SET 在线改，不用重新烧录。
 *
 * 设计思路(为了放得准)：
 *   - 树莓派用摄像头做闭环对准：车到工位后，先用 OBS 把手臂摆到"观察姿态"，
 *     然后反复 AD(ID1/ID2 小角度微调)、测偏差、再调，直到对准；
 *     对准以后才叫 GRAB n H / PICK n H(夹起放进转盘)或 AP + DROP(放下)。
 *   - 不带 H 的 GRAB / PICK / PLACE 是"不用摄像头、全按固定姿态"的完整流程，调试和应急用。
 *
 * 安全措施：
 *   - 没回零(LIFT ZERO / LIFT HOME)之前，不允许任何升降动作和夹放流程
 *   - 参数 ARMOK=0(默认)时不允许夹放流程：姿态都标定好后 SET ARMOK 1
 *   - 升降目标会被限制在 0 ~ LFMAX 毫米；舵机角度由 main.c 里的限位保护
 *   - 任何动作进行中收到 '!' 都会马上停升降、结束流程
 */
#include "arm.h"
#include "Emm_V5.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef  htim2;
extern volatile uint8_t   car_abort;
extern int Car_Motor_Query(uint8_t addr, uint8_t func, uint8_t *out, uint8_t len);   /* chassis.c：读驱动器参数 */

/* main.c 里已有的函数 */
extern void Claw_Set(uint32_t pulse_us);
extern void Turntable_Set(uint32_t pulse_us);
extern uint32_t Servo_Start(uint8_t id, float *angle_deg, float speed_dps);
extern void Servo_Move(uint8_t id, float angle_deg, float speed_dps, float tol_deg, uint32_t extra_ms);
extern int  Servo_ReadAngle(uint8_t id, float *angle);
extern void Servo_StopIfStuck(uint8_t id, float target);
extern void Servo_Report(uint8_t id);
extern void Servo_SetPower(uint16_t mw);
extern void Servo_SetComp(uint8_t id, float deg);
extern void Servo_SetLimits(uint8_t id, float lo, float hi);
extern void Servo_Hold(uint8_t id, float angle_deg);
extern void Servo_Track(uint8_t id, float angle_deg, float speed_dps);
extern int  Servo_Arrived(uint8_t id, float a, float tol_deg);
extern void Servo_Params(uint8_t id);
extern int  Servo_WriteParam(uint8_t id, const char *name, long value);
extern void Delay_Report(uint32_t ms);

#define LIFT_ADDR   5          /* 升降步进电机的 Emm 地址 */

/* 两个模块出厂都是 9600。工程里 USART2/UART5 初始化成了 115200，Arm_Init 里会改成下面的值。
 * 如果您把屏或扫码模块改过波特率，只改这两个数 */
/* 开机后升降要停的高度(离最低点往上多少 mm)。开机前把升降大概放在这个高度附近(±20mm)，
 * 上电后程序读电机编码器算出准确高度，自动走到正好这个高度(要先 LIFT CAL 标定一次)。改了这里要重新编译烧录 */
#define LIFT_BOOT_MM   60.0f
#define LIFT_TOP_MM    130.0f     /* LFMAX 能设的最大值(机构最多到这么高) */

#define SCREEN_BAUD    9600u
#define QR_BAUD        9600u

/* 开机时夹爪、转盘先到这个位置(要和 main.c 里 CLAW_OPEN_US / TURNTABLE_SLOT1_US 一致) */
#define CLAW_BOOT_US   2090u
#define TT_BOOT_US     2608u

#define SERVO_EXTRA_MS 800     /* ID1/ID2 一起转时，按速度算好的时间之外最多再等多久 */

/* ================= 可调参数(SET 名字 数值) ================= */
static float g_lppm   = 80.0f;     /* LFPPM  升降：1 毫米要多少脉冲。= 每圈脉冲数(16 细分是 3200) / 皮带每圈走的毫米数 */
static float g_lrpm   = 60.0f;     /* LFRPM  升降速度(转/分)。实测 60 稳 */
static float g_lacc   = 0.0f;      /* LFACC  升降加速度档位 0~255(0=不加速直接到速度) */
static float g_lmax   = 115.0f;    /* LFMAX  最高点：离零点(最低点)往上 115mm(10-16 用户：机构到 115 就顶住了，从 118 改成 115)。10-19 起 SET 最多 130 */
static float g_ldir   = 0.0f;      /* LFDIR  升降方向：0 或 1。LIFT 数字变大时走反了就改这个 */
static float g_lspr   = 3200.0f;   /* LFSPR  电机每圈脉冲数，只用来算要等多久 */
static float g_lmrg   = 200.0f;    /* LFMRG  升降走完后多等多少毫秒(加减速余量) */
static float g_lhmd   = 2.0f;      /* LFHMD  LIFT HOME 用的 Emm 回零模式(0~3，见张大头手册) */
static float g_lhtm   = 8000.0f;   /* LFHTM  LIFT HOME 最多等多少毫秒 */
static float g_lchk   = 1.5f;      /* LFCHK  (10-21 用户：下到最低了升降还往下动)每次走完用编码器核对实际高度，差这么多毫米以上(丢步：要去的高度
                                    *        机构到不了、顶到东西)就按编码器改过来并报 LIFTSLIP；0 = 不核对(以前的做法) */

static float g_zhi    = 100.0f;    /* ZHI    搬运途中的高度(抬高，越大越高) */
static float g_zgrab  = 20.0f;     /* ZGRAB  在原料盘上夹物料时的高度 */
static float g_zdrop  = 60.0f;     /* ZDROP  放进车上转盘 / 从转盘取物料时的高度 */
static float g_zplc   = 0.0f;      /* ZPLC   放到地上圆环时的高度 */
static float g_zstk   = 60.0f;     /* ZSTK   码垛时放下的高度(= ZPLC + 一个物料的高度，物料高 60mm) */
static float g_zobraw = 100.0f;    /* ZOBRAW 观察原料盘时的高度(摄像头标定比例时用的高度) */
static float g_zobrng = 100.0f;    /* ZOBRNG 观察地上圆环时的高度 */

static float g_a1g    = 323.0f;    /* A1G    ID1 角度：对准原料盘 */
static float g_a1d    = 323.0f;    /* A1D    ID1 角度：对准车上转盘 */
static float g_a1h    = 323.0f;    /* A1H    ID1 角度：收起/待命 */
static float g_a1p    = 323.0f;    /* A1P    ID1 角度：对准地上圆环 */
static float g_a2e    = -862.0f;   /* A2E    ID2 角度：伸出夹原料盘上的物料 */
static float g_a2r    = -862.0f;   /* A2R    ID2 角度：转盘上方(收起时 ID2 也用这个角度) */
static float g_a2p    = -862.0f;   /* A2P    ID2 角度：伸出到地上圆环 */
/* (10-19 用户：电池盒挡着)转去车上转盘的路：夹起以后 ID1 一直转到转盘上方 A1D，转过 A1T1 时伸到 A2T、转过 A1T2 时缩回 A2R
 * (ID1 不停)。伸缩都在两个圆盘外面，不挡物料。离开转盘反过来走。A2T = 0 不用(照旧)；A1T1/A1T2 = 0 当作不转这一段 */
static float g_a1t1   = 0.0f;      /* A1T1   ID1 角度：转到这里开始伸长(已经离开原料盘/圆环那一块) */
static float g_a2t    = 0.0f;      /* A2T    ID2 角度：伸到这么长转，碰不到电池盒 */
static float g_a1t2   = 0.0f;      /* A1T2   ID1 角度：转过电池盒，到这里开始缩回(还没到车上转盘正上方) */
static float g_a2spd  = 0.0f;      /* A2SPD  这几段伸长、缩回的速度(度/秒)；0 = 用 ASPD */
/* (10-23 用户"原料区域抓取物料之后再新加上一个往回缩的动作，因为我怕这个爪子挡住其他物料")在原料盘上夹紧以后：
 * 先直着抬 OVCLR 离开原料盘，再一边升到 ZHI 一边把伸缩缩回到 A2GB(ID1 不转)，然后照旧转去车上转盘。0 = 不缩(照旧) */
static float g_a2gb   = 0.0f;      /* A2GB   ID2 角度：在原料盘夹起以后先缩回到这里 */
/* (10-20 用户：夹着物料去地上圆环也怕刮到电池盒)车上转盘 <-> 地上圆环这条路单独一组，意思和上面 A1T1~A2SPD 一样：
 * 从圆环往车上转盘转，转过 R1T1 伸到 R2T、转过 R1T2 缩回 A2R；离开车上转盘反过来走(转过 R1T2 伸到 R2T，
 * 转过 R1T1 再伸缩到圆环那边要的长度)。R2T = 0 = 这一组没调，照旧走原料区那组 */
static float g_r1t1   = 0.0f;      /* R1T1   ID1 角度：从圆环往车上转盘转，到这里开始伸长 */
static float g_r2t    = 0.0f;      /* R2T    ID2 角度：伸到这么长转，碰不到电池盒 */
static float g_r1t2   = 0.0f;      /* R1T2   ID1 角度：转过电池盒，到这里开始缩回(还没到车上转盘正上方) */
static float g_r2spd  = 0.0f;      /* R2SPD  这一组伸长、缩回的速度(度/秒)；0 = 和 A2SPD 一样 */
static uint8_t g_side = 0;         /* 手臂在哪边干活：0 = 原料盘，1 = 地上圆环(OBS RAW/RING、GRAB、PICK、PLACE、SIDE 指令设) */
static float g_aspd   = 90.0f;     /* ASPD   ID1/ID2 大动作转速(度/秒) */
static float g_aspdf  = 40.0f;     /* ASPDF  ID1/ID2 微调转速(度/秒) */
static float g_lvdeg  = 0.0f;      /* LVDEG  夹着物料离开车上转盘(没有绕电池盒、或者一边转一边伸缩时)：先只转 ID1 这么多度(伸缩不动)再走；0 = 不先转 */
static float g_r2stg  = 0.0f;      /* R2STG  车上转盘 <-> 地上圆环这条路：0 = (10-21 用户"ID1 整个转动过程要连续")ID1 一直转不停，转过 R1T2 开始伸到 R2T、
                                    *        转过 R1T1 开始伸缩到要的长度(去车上转盘反过来，和原料区一样)；
                                    *        1 = 分段：只转到 R1T2 -> 只伸缩到 R2T -> 只转到 R1T1 -> 只伸缩到要的长度 -> 再转过去，每段停下再走下一段 */
static float g_a2stg  = 0.0f;      /* A2STG  原料区那组(A1T1~A2SPD)也分段走：1 = 分段；0 = 一边转一边伸缩(10-19 用户要 ID1 不停) */
static float g_rotspd = 0.0f;      /* ROTSPD (10-21 用户)旋转舵机大动作转速(度/秒)；0 = 用 ASPD */
static float g_extspd = 0.0f;      /* EXTSPD (10-21 用户)伸缩舵机大动作转速(度/秒，舵机最快多少就按多少转)；0 = 用 ASPD。
                                    *        哪个是伸缩舵机按 AEXT。绕电池盒那几段伸缩还是按 A2SPD / R2SPD(它们是 0 时用这个) */
static float g_atolc  = 1.0f;      /* ATOLC  大动作离目标多少度以内算到位 */
static float g_atol   = 0.3f;      /* ATOL   微调离目标多少度以内算到位 */
static float g_amaxd  = 60.0f;     /* AMAXD  AD 指令一次最多转多少度(防止发错数) */
static float g_para   = 1.0f;      /* PARA   1 = ID1、ID2 一起转(快)；0 = 一个一个转 */

static float g_clwait = 400.0f;    /* CLWAIT 夹爪动作后等多久(毫秒) */
static float g_clspd  = 0.0f;      /* CLSPD  夹爪合上(夹取)的速度(微秒/秒，脉宽每秒变多少)；0 = 一下子转过去(以前的做法)。张开到夹紧约差 500 微秒：1000 = 约 0.5 秒 */
static float g_clspo  = 800.0f;    /* CLSPO  夹爪张开(放下)的速度(微秒/秒)；0 = 一下子张开。10-10 用户定：张开 800，合上照旧一下子(CLSPD 0) */
static float g_ovlap  = 1.0f;      /* OVLAP  1 = 升降和转动同时做(10-14 用户：夹起后边升边转、到转盘上方前边降边转)；0 = 以前的一步一步做 */
static float g_ovclr  = 20.0f;     /* OVCLR  边升边转之前先直着升多少毫米(离开物料/转盘槽)，放下前最后直着降多少毫米 */
static float g_ttwait = 800.0f;    /* TTWAIT 转盘转到新位置要等多久(毫秒) */
static float g_armok  = 0.0f;      /* ARMOK  1 = 姿态都标定好了，允许夹放流程 */
static float g_scrmode = 1.0f;     /* SCRMODE 串口屏：1 = 程序自己画字(屏工程里只要两个字库，不用放控件)；0 = 写控件 t0.txt=… */
static float g_clwo   = 2090.0f;   /* CLWO   夹爪张开的脉宽(微秒) */
static float g_clwc   = 2910.0f;   /* CLWC   夹爪夹紧的脉宽(微秒)。太紧了舵机一直使劲会发烫：调到刚好夹稳 */
static float g_clwor  = 0.0f;      /* CLWOR  (10-20 用户)在原料盘上方张开的脉宽(夹原料之前张开)；0 = 和 CLWO 一样 */
static float g_clwot  = 0.0f;      /* CLWOT  (10-20 用户)在车上转盘上方张开的脉宽(松开放进转盘、从转盘里取之前张开)；0 = 和 CLWO 一样。
                                    *        CLWO 是其他地方(地上圆环放下、取回之前、OBS RING O 等)张开的脉宽 */
static float g_tt1    = 2608.0f;   /* TT1    转盘 1 号位的脉宽(微秒) */
static float g_tt2    = 1708.0f;   /* TT2    转盘 2 号位 */
static float g_tt3    = 808.0f;    /* TT3    转盘 3 号位 */
static float g_aext   = 2.0f;      /* AEXT   哪个舵机管前后伸缩：2 = ID2 伸缩、ID1 旋转；1 = ID1 伸缩、ID2 旋转 */
static float g_spow   = 30000.0f;  /* SPOW   ID1/ID2 转动时允许的最大功率(mW)，30000 = 当时 set SPOW 30000 的值。发烫就减小 */
static float g_s1comp = 5.0f;      /* S1COMP ID1 补偿(度)：手臂有摩擦，舵机总停在离目标差这么多的地方，就往前多给这么多，到了目标马上停 */
static float g_s2comp = 0.0f;      /* S2COMP ID2 补偿(度)，0 = 不补 */
static float g_s1min  = 44.0f;     /* S1MIN  ID1 限位(度，多圈角度)：10-16 用户松开 ID1 用手转到两头量的 42.4~244.2，各留一点余量 */
static float g_s1max  = 244.0f;    /* S1MAX */
static float g_s2min  = -1210.0f;  /* S2MIN  ID2 限位(伸缩齿条，度)：10-16 用户 -1210 ~ -300；10-18 用户：齿条够长，可以调大 */
static float g_s2max  = -300.0f;   /* S2MAX */

typedef struct
{
    const char *name;
    float      *p;
    float       lo;
    float       hi;
} ArmTun;

static const ArmTun tun[] =
{
    { "LFPPM",  &g_lppm,    1.0f,    500.0f },
    { "LFRPM",  &g_lrpm,    10.0f,   600.0f },
    { "LFACC",  &g_lacc,    0.0f,    255.0f },
    { "LFMAX",  &g_lmax,    10.0f,   LIFT_TOP_MM },   /* (10-19 用户：机构改过还能往上)最高点最多 130mm，实际用 set LFMAX 设成量到的值 */
    { "LFDIR",  &g_ldir,    0.0f,    1.0f },
    { "LFSPR",  &g_lspr,    200.0f,  51200.0f },
    { "LFMRG",  &g_lmrg,    0.0f,    2000.0f },
    { "LFHMD",  &g_lhmd,    0.0f,    3.0f },
    { "LFHTM",  &g_lhtm,    500.0f,  30000.0f },
    { "LFCHK",  &g_lchk,    0.0f,    20.0f },
    { "ZHI",    &g_zhi,     0.0f,    400.0f },
    { "ZGRAB",  &g_zgrab,   0.0f,    400.0f },
    { "ZDROP",  &g_zdrop,   0.0f,    400.0f },
    { "ZPLC",   &g_zplc,    0.0f,    400.0f },
    { "ZSTK",   &g_zstk,    0.0f,    400.0f },
    { "ZOBRAW", &g_zobraw,  0.0f,    400.0f },
    { "ZOBRNG", &g_zobrng,  0.0f,    400.0f },
    /* (10-18 用户：齿条重装、ID1 放开以后姿态角度会超出原来的 44~244 / -1210~-300)姿态角度不单独限范围：
     * 真转的时候 main.c 按限位 S1MIN/S1MAX、S2MIN/S2MAX 截住(转不到限位外面) */
    { "A1G",    &g_a1g,    -720.0f,  720.0f },
    { "A1D",    &g_a1d,    -720.0f,  720.0f },
    { "A1H",    &g_a1h,    -720.0f,  720.0f },
    { "A1P",    &g_a1p,    -720.0f,  720.0f },
    { "A2E",    &g_a2e,    -5000.0f, 5000.0f },
    { "A2R",    &g_a2r,    -5000.0f, 5000.0f },
    { "A2P",    &g_a2p,    -5000.0f, 5000.0f },
    { "A1T1",   &g_a1t1,   -720.0f,  720.0f },
    { "A2T",    &g_a2t,    -5000.0f, 5000.0f },
    { "A1T2",   &g_a1t2,   -720.0f,  720.0f },
    { "A2SPD",  &g_a2spd,   0.0f,    1500.0f },
    { "A2GB",   &g_a2gb,   -5000.0f, 5000.0f },
    { "R1T1",   &g_r1t1,   -720.0f,  720.0f },
    { "R2T",    &g_r2t,    -5000.0f, 5000.0f },
    { "R1T2",   &g_r1t2,   -720.0f,  720.0f },
    { "R2SPD",  &g_r2spd,   0.0f,    1500.0f },
    { "ASPD",   &g_aspd,    10.0f,   300.0f },
    { "ASPDF",  &g_aspdf,   5.0f,    150.0f },
    { "LVDEG",  &g_lvdeg,   0.0f,    120.0f },
    { "R2STG",  &g_r2stg,   0.0f,    1.0f },
    { "A2STG",  &g_a2stg,   0.0f,    1.0f },
    { "ROTSPD", &g_rotspd,  0.0f,    300.0f },   /* 0 = 用 ASPD；不是 0 要 >= 10(下面查) */
    { "EXTSPD", &g_extspd,  0.0f,    1500.0f },
    { "ATOLC",  &g_atolc,   0.2f,    5.0f },
    { "ATOL",   &g_atol,    0.05f,   2.0f },
    { "AMAXD",  &g_amaxd,   1.0f,    300.0f },
    { "PARA",   &g_para,    0.0f,    1.0f },
    { "CLWAIT", &g_clwait,  0.0f,    3000.0f },
    { "CLSPD",  &g_clspd,   0.0f,    20000.0f },
    { "CLSPO",  &g_clspo,   0.0f,    20000.0f },
    { "TTWAIT", &g_ttwait,  0.0f,    5000.0f },
    { "OVLAP",  &g_ovlap,   0.0f,    1.0f },
    { "OVCLR",  &g_ovclr,   0.0f,    100.0f },
    { "ARMOK",  &g_armok,   0.0f,    1.0f },
    { "SCRMODE", &g_scrmode, 0.0f,   1.0f },
    { "CLWO",   &g_clwo,    500.0f,  2910.0f },  /* 范围 = main.c 里夹爪的限位 CLAW_MIN_US ~ CLAW_MAX_US */
    { "CLWC",   &g_clwc,    500.0f,  2910.0f },
    { "CLWOR",  &g_clwor,   0.0f,    2910.0f },  /* 0 = 用 CLWO；不是 0 就要在夹爪限位 500 ~ 2910 里面 */
    { "CLWOT",  &g_clwot,   0.0f,    2910.0f },
    { "TT1",    &g_tt1,     500.0f,  2608.0f },  /* 范围 = main.c 里转盘的限位 TURNTABLE_MIN_US ~ TURNTABLE_MAX_US */
    { "TT2",    &g_tt2,     500.0f,  2608.0f },
    { "TT3",    &g_tt3,     500.0f,  2608.0f },
    { "AEXT",   &g_aext,    1.0f,    2.0f },
    { "SPOW",   &g_spow,    1000.0f, 30000.0f },
    { "S1COMP", &g_s1comp,  0.0f,    15.0f },
    { "S2COMP", &g_s2comp,  0.0f,    15.0f },
    { "S1MIN",  &g_s1min,  -720.0f,  720.0f },
    { "S1MAX",  &g_s1max,  -720.0f,  720.0f },
    { "S2MIN",  &g_s2min,  -5000.0f, 5000.0f },
    { "S2MAX",  &g_s2max,  -5000.0f, 5000.0f },
};
#define TUN_N  ((int)(sizeof(tun) / sizeof(tun[0])))

int Arm_Param_Set(const char *name, float v)
{
    int i;

    for (i = 0; i < TUN_N; i++)
    {
        if (strcmp(tun[i].name, name) == 0)
        {
            if (!(v >= tun[i].lo && v <= tun[i].hi))  /* (10-21)这样写 nan 也挡得住 */
            {
                return 2;
            }
            if ((tun[i].p == &g_rotspd || tun[i].p == &g_extspd) && v > 0.5f && v < 10.0f)
            {
                return 2;                          /* 0 = 用 ASPD；太慢的不要 */
            }
            if ((tun[i].p == &g_clwor || tun[i].p == &g_clwot) && v > 0.5f && v < 500.0f)
            {
                return 2;                          /* 0 = 和 CLWO 一样；其他要在夹爪限位里面 */
            }
            if ((tun[i].p == &g_s1min && v >= g_s1max) || (tun[i].p == &g_s1max && v <= g_s1min) ||
                (tun[i].p == &g_s2min && v >= g_s2max) || (tun[i].p == &g_s2max && v <= g_s2min))
            {
                return 2;                          /* 下限要比上限小 */
            }
            *tun[i].p = v;
            if (tun[i].p == &g_s1min || tun[i].p == &g_s1max)
            {
                Servo_SetLimits(1, g_s1min, g_s1max);   /* main.c 里的限位马上换 */
            }
            if (tun[i].p == &g_s2min || tun[i].p == &g_s2max)
            {
                Servo_SetLimits(2, g_s2min, g_s2max);
            }
            if (tun[i].p == &g_spow)
            {
                Servo_SetPower((uint16_t)v);       /* main.c 里的舵机功率马上换 */
            }
            if (tun[i].p == &g_s1comp)
            {
                Servo_SetComp(1, v);               /* 补偿角度马上换 */
            }
            if (tun[i].p == &g_s2comp)
            {
                Servo_SetComp(2, v);
            }
            return 1;
        }
    }
    return 0;
}

static float Absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

/* 把 v 写成 "-12.3456"，不用 %f(有的编译设置不支持浮点 printf) */
static void FmtF(char *b, int n, float v)
{
    long iv, fr;
    const char *sg = "";

    if (v < 0.0f)
    {
        sg = "-";
        v = -v;
    }
    iv = (long)v;
    fr = (long)((v - (float)iv) * 10000.0f + 0.5f);
    if (fr >= 10000)
    {
        iv++;
        fr -= 10000;
    }
    snprintf(b, (size_t)n, "%s%ld.%04ld", sg, iv, fr);
}

static void Say(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

void Arm_Param_Dump(void)
{
    char m[48];
    char b[20];
    int i;

    for (i = 0; i < TUN_N; i++)
    {
        FmtF(b, (int)sizeof(b), *tun[i].p);
        snprintf(m, sizeof(m), "P %s=%s\r\n", tun[i].name, b);
        Say(m);
    }
}

/* ================= 等待(可被 '!' 打断) ================= */
/* 返回 1 = 等满了，0 = 被急停打断。每 100 毫秒顺便发一次车头角度，树莓派那边的角度显示不会断 */
static int Wait_Ms(uint32_t ms)
{
    uint32_t t0 = HAL_GetTick();
    uint32_t el;

    while ((el = HAL_GetTick() - t0) < ms)
    {
        uint32_t left = ms - el;
        if (car_abort)
        {
            return 0;
        }
        Delay_Report(left < 100u ? left : 100u);
    }
    return car_abort ? 0 : 1;
}

/* ================= 升降 (Emm ID5) ================= */
static float   lift_mm    = 0.0f;
static uint8_t lift_known = 0;
/* 开机时升降高度是怎么认的(LIFT? 回复里的 BOOT=)：
 *   ENC   编码器找准的(Flash 里没有上次的位置记录，在 60mm 上下找)
 *   POS   编码器找准的(在 Flash 里记的上次停下的位置上下找)
 *   NOCAL 没标定，直接当作在 60mm(高度可能不准)
 *   NOENC 读不到编码器，直接当作在 60mm(高度可能不准) */
static const char *lift_boot_how = "NOCAL";
static uint8_t     lift_guess    = 0;         /* (10-21)开机没读到编码器，高度是猜的：之后一读到编码器就按它认准(±20mm 以内)，回 LIFTSYNC */

/* 下面"编码器"那一节里的函数，Lift_Goto 要先用到 */
static void Pos_Save(float z);
static int  Lift_EncPos(float near_mm, float *z, uint16_t *raw);
static int Lift_Check(float mm);

static uint32_t Lift_Time_Ms(uint32_t pulses)
{
    float sps = g_lrpm / 60.0f * g_lspr;          /* 每秒多少脉冲 */
    return (uint32_t)((float)pulses / sps * 1000.0f) + (uint32_t)g_lmrg;
}

/* 发出去还没等完的升降(Lift_Start 发的)：目标、出发时的高度、预计到的时刻 */
static float    lift_pend = -1.0f;
static float    lift_from = 0.0f;
static uint32_t lift_end  = 0;
/* (10-21)上次用编码器核对过以后升降走过的高度范围：编码器一圈 40mm，按它认的高度只有落在这个范围里才信(差出去就是认错了圈) */
static float    lift_span_lo = 0.0f;
static float    lift_span_hi = 0.0f;
static uint8_t  lift_chk_due = 0;     /* Lift_Done_Now 到点了还没核对(手臂正在转，不能停下来等编码器)：Lift_Finish 里补 */

static void Lift_Span(float mm)
{
    if (mm < lift_span_lo)  lift_span_lo = mm;
    if (mm > lift_span_hi)  lift_span_hi = mm;
}

static void Lift_Span_Reset(void)
{
    lift_span_lo = lift_span_hi = lift_mm;
}

/* 只发升降指令不等(和舵机同时动用)。之后一定要调 Lift_Finish。成功返回 NULL */
static const char *Lift_Start(float mm)
{
    float delta;
    uint32_t pulses;
    uint8_t dir_down = (g_ldir > 0.5f) ? 1 : 0;
    uint8_t dir;

    lift_pend = -1.0f;
    if (!lift_known)
    {
        return "ERR NOZERO";
    }
    if (mm < 0.0f)     mm = 0.0f;
    if (mm > g_lmax)   mm = g_lmax;
    delta = mm - lift_mm;
    if (delta > -0.05f && delta < 0.05f)
    {
        return NULL;
    }
    dir = (delta > 0.0f) ? dir_down : (uint8_t)(1 - dir_down);
    pulses = (uint32_t)((delta > 0.0f ? delta : -delta) * g_lppm + 0.5f);
    if (pulses == 0)
    {
        return NULL;
    }
    Emm_V5_En_Control(LIFT_ADDR, true, false);
    HAL_Delay(5);
    Emm_V5_Pos_Control(LIFT_ADDR, dir, (uint16_t)g_lrpm, (uint8_t)g_lacc, pulses, 0, false);
    Lift_Span(lift_mm);
    Lift_Span(mm);
    lift_pend = mm;
    lift_from = lift_mm;
    lift_end  = HAL_GetTick() + Lift_Time_Ms(pulses);
    return NULL;
}

/* 升降到了没有(不等)：到了就记下高度、返回 1 */
static int Lift_Done_Now(void)
{
    if (lift_pend < 0.0f)
    {
        return 1;
    }
    if ((int32_t)(HAL_GetTick() - lift_end) >= 0)
    {
        lift_mm = lift_pend;
        lift_pend = -1.0f;
        lift_chk_due = 1;                          /* (10-21)用编码器核对等手臂转完再做(Lift_Finish)：这里在 15ms 一次的舵机循环里，
                                                    * 停下来等编码器最多要 1 秒，ID1 会转过头(往前多给的 S1COMP 5°) */
        Pos_Save(lift_mm);
        return 1;
    }
    return 0;
}

/* 等 Lift_Start 发的升降走完。急停打断：停下，用编码器把高度找回来(和 Lift_Goto 一样) */
static const char *Lift_Finish(void)
{
    int32_t left;
    float mm = lift_pend;

    if (mm < 0.0f)
    {
        if (lift_chk_due)                          /* (10-21)Lift_Done_Now 到点时手臂在转、没核对：现在补上 */
        {
            lift_chk_due = 0;
            if (Lift_Check(lift_mm))  return "ERR LIFT";
            Pos_Save(lift_mm);
        }
        return NULL;
    }
    left = (int32_t)(lift_end - HAL_GetTick());
    if (left > 0 && !Wait_Ms((uint32_t)left))
    {
        float z;
        uint16_t e;

        lift_pend = -1.0f;
        Emm_V5_Stop_Now(LIFT_ADDR, false);
        lift_known = 0;
        if (Absf(mm - lift_from) <= 36.0f)
        {
            HAL_Delay(50);
            if (Lift_EncPos((mm + lift_from) / 2.0f, &z, &e))
            {
                lift_mm = z;
                lift_known = 1;
                Pos_Save(z);
            }
        }
        return NULL;
    }
    lift_pend = -1.0f;
    lift_mm = mm;
    lift_chk_due = 0;
    if (Lift_Check(mm))  return "ERR LIFT";        /* (10-21)用编码器核对到了没有；认的高度不可信就不往下做(免得在半空夹/松爪子) */
    Pos_Save(lift_mm);
    return NULL;
}

/* 升降到离零点 mm 毫米处。成功返回 NULL；出错返回错误文字。被 '!' 打断时也返回 NULL(调用者看 car_abort) */
static const char *Lift_Goto(float mm)
{
    float delta;
    uint32_t pulses;
    uint8_t dir_down = (g_ldir > 0.5f) ? 1 : 0;
    uint8_t dir;

    if (!lift_known)
    {
        return "ERR NOZERO";
    }
    if (mm < 0.0f)     mm = 0.0f;
    if (mm > g_lmax)   mm = g_lmax;

    delta = mm - lift_mm;
    if (delta > -0.05f && delta < 0.05f)
    {
        return NULL;
    }
    dir = (delta > 0.0f) ? dir_down : (uint8_t)(1 - dir_down);
    pulses = (uint32_t)((delta > 0.0f ? delta : -delta) * g_lppm + 0.5f);
    if (pulses == 0)
    {
        return NULL;
    }

    Emm_V5_En_Control(LIFT_ADDR, true, false);
    HAL_Delay(5);                                  /* 两条 Emm 指令之间要留空隙，连着发驱动器会当成一帧坏数据丢掉 */
    Emm_V5_Pos_Control(LIFT_ADDR, dir, (uint16_t)g_lrpm, (uint8_t)g_lacc, pulses, 0, false);
    Lift_Span(lift_mm);
    Lift_Span(mm);

    if (!Wait_Ms(Lift_Time_Ms(pulses)))
    {
        float z;
        uint16_t e;

        Emm_V5_Stop_Now(LIFT_ADDR, false);
        lift_known = 0;                           /* 停在半路，不知道在哪了 */
        /* 标定过编码器：停在起点和终点之间，用编码器把准确高度找回来(这一段不超过 36mm 才找得准) */
        if (Absf(mm - lift_mm) <= 36.0f)
        {
            HAL_Delay(50);
            if (Lift_EncPos((mm + lift_mm) / 2.0f, &z, &e))
            {
                lift_mm = z;
                lift_known = 1;
                Pos_Save(z);
            }
        }
        return NULL;
    }
    lift_mm = mm;
    if (Lift_Check(mm))  return "ERR LIFT";        /* (10-21)用编码器核对到了没有(丢步就改过来)；认的高度不可信：不往下做 */
    Pos_Save(lift_mm);                             /* 记进 Flash：下次上电从这里找 */
    return NULL;
}

/* ================= 升降编码器：开机自动找到准确高度 =================
 * 升降电机转一圈 = LFSPR/LFPPM 毫米(3200/80 = 40mm)。驱动器能读出电机"一圈里转到哪个角度"，
 * 这个角度断电也不会丢(磁编码器)，只是分不出是第几圈。所以：
 *   开机前把升降大概放在 LIFT_BOOT_MM(60mm) 附近，上下差不超过 ±20mm(半圈)，也就是 40~80mm 之间；
 *   开机时读这个角度，就能算出准确高度，再自动走到正好 60mm。整个过程不碰任何东西。
 * 要先标定一次：LIFT CAL <毫米>(升降现在离最低点多少毫米，用尺子量)。程序上下动 10mm，
 * 测出"高度 0 对应的角度"和"每毫米角度变多少"，存进 STM32 内部 Flash 最后一个扇区，
 * 断电、重新烧录程序都不会丢(除非烧录时选了"整片擦除")。 */
#ifndef ARM_CAL_ADDR
#define ARM_CAL_ADDR   0x08060000u        /* Flash 扇区 7(128KB)。程序只占前面几十 KB，不冲突 */
#endif
#define CAL_MAGIC      0x4C465432u        /* "LFT2"(第 2 版：多存了"读哪个编码器值") */
#ifndef ARM_CAL_SIZE
#define ARM_CAL_SIZE   0x20000u           /* 扇区 7 一共 128KB */
#endif
/* 扇区前 256 字节放标定，后面是"升降停下的位置"记录：每停一次追加一条(8 字节)，不用擦除；
 * 一共能记一万六千多条，用掉四分之三后，开机时擦一次扇区重新开始(擦除要 1~2 秒，只在开机做) */
#define POS_BASE       (ARM_CAL_ADDR + 0x100u)
#define POS_N          ((uint32_t)((ARM_CAL_SIZE - 0x100u) / 8u))

static float   lift_enc0   = 0.0f;        /* 高度 0mm 时的编码器读数 */
static float   lift_cpm    = 0.0f;        /* 每升高 1mm 编码器读数变多少(带正负) */
static float   lift_cpr    = 65536.0f;    /* 编码器一圈的读数(标定时自动判断 65536 / 16384 / 4096) */
static uint8_t lift_cal_ok = 0;
static uint8_t lift_src    = 0x31;        /* 读哪个值：0x31 编码器值；有的驱动器 0x31 总回 0，就用 0x36 实时位置(取一圈里的部分) */
static uint8_t lift_nochk  = 0;           /* (10-21)LIFT CAL 上下走那两步不核对：旧的标定可能不准，按它改高度会把这次标定带歪 */

static float   pos_last    = 0.0f;        /* 最后一次记进 Flash 的高度(一样就不重复写) */
static uint8_t pos_last_ok = 0;

/* 把编码器读数差折到 ±半圈 */
static float Enc_Wrap(float d, float cpr)
{
    return d - cpr * floorf((d + cpr / 2.0f) / cpr);
}

/* 读升降电机一圈里的角度(0~65535)。src = 0x31：回 [05 31 高 低 6B]；
 * src = 0x36：回 [05 36 符号 4字节位置 6B]，位置一圈 65536，取低 16 位就是一圈里的角度。读到返回 1 */
static int Lift_ReadSrc(uint8_t src, uint16_t *v)
{
    uint8_t r[8];
    uint32_t p;
    int k;

    for (k = 0; k < 3; k++)
    {
        if (src == 0x36)
        {
            if (Car_Motor_Query(LIFT_ADDR, 0x36, r, 8))
            {
                p = ((uint32_t)r[3] << 24) | ((uint32_t)r[4] << 16) | ((uint32_t)r[5] << 8) | (uint32_t)r[6];
                if (r[2] != 0)
                {
                    p = 0u - p;                    /* 负数：取补码，低 16 位仍是一圈里的角度 */
                }
                *v = (uint16_t)(p & 0xFFFFu);
                return 1;
            }
        }
        else if (Car_Motor_Query(LIFT_ADDR, 0x31, r, 5))
        {
            *v = (uint16_t)(((uint16_t)r[2] << 8) | r[3]);
            return 1;
        }
        HAL_Delay(10);
    }
    return 0;
}

static int Lift_ReadEnc(uint16_t *v)
{
    return Lift_ReadSrc(lift_src, v);
}

/* 用编码器算现在的高度：在 near_mm 上下半圈(±20mm)以内找。没标定或读不到返回 0 */
static int Lift_EncPos(float near_mm, float *z, uint16_t *raw)
{
    uint16_t e;
    float rev, top;

    if (!lift_cal_ok || !Lift_ReadEnc(&e))
    {
        return 0;
    }
    *raw = e;
    *z = near_mm + Enc_Wrap((float)e - (lift_enc0 + lift_cpm * near_mm), lift_cpr) / lift_cpm;
    /* 算出来的在行程外面(比最低点还低、比最高点还高)，那一定差了整圈：挪回行程里面。
     * (10-21)最高点按 LFMAX 能设的最大值 130 算：开机时配置里的 LFMAX 还没发过来，按程序里写的 115 算会把停在 118 以上的认错一整圈 */
    rev = lift_cpr / Absf(lift_cpm);
    top = (g_lmax > LIFT_TOP_MM) ? g_lmax : LIFT_TOP_MM;
    while (*z < -3.0f && *z + rev <= top + 3.0f)
    {
        *z += rev;
    }
    while (*z > top + 3.0f && *z - rev >= -3.0f)
    {
        *z -= rev;
    }
    return 1;
}

/* (10-21 用户：下到最低了升降还往下动)升降走完用编码器核对实际到了没有。以前只按发了多少脉冲、等了多久算高度：要去的高度机构到不了
 * (ZHI/STBY 比升降能到的最高还高，或者比最低点还低)、顶到东西丢了步，也照样当作到了，记的高度和实际越差越多，后面下到低处就一直往下顶。
 * 还在走(等的时间估短了)：等它停稳(最多约 1 秒)；差 0.5mm~LFCHK：顶到东西了，叫它别再顶，报 LIFTPUSH；差 LFCHK 毫米以上：按编码器改过来，
 * 报 LIFTSLIP。(10-21)编码器一圈 40mm，差 20mm 以上会认错圈：差 15mm 以上、或者认出的高度不在上次核对以后走过的范围里，不改、返回 1
 * (调用的地方报 ERR LIFT，不夹/不松爪子)。没标定编码器 / LFCHK=0 不核对 */
static int Lift_Check(float mm)
{
    float z, z2, d;
    uint16_t e;
    int k;
    int bad;
    char m[80];
    char s1[20];
    char s2[20];

    if (g_lchk <= 0.0f || !lift_cal_ok || lift_nochk)  { Lift_Span_Reset(); return 0; }
    if (!Lift_EncPos(mm, &z, &e))  { Lift_Span_Reset(); return 0; }
    for (k = 0; k < 12 && Absf(z - mm) >= 0.3f; k++)
    {
        Delay_Report(80);                          /* 还在走就等它停下(和 Wait_Ms 一样照常回报车头角度) */
        if (car_abort || !Lift_EncPos(mm, &z2, &e))  { Lift_Span_Reset(); return 0; }
        if (Absf(z2 - z) < 0.15f)
        {
            z = z2;
            break;
        }
        z = z2;
    }
    if (lift_guess && Absf(z - mm) <= 20.0f)
    {
        /* (10-21)开机没读到编码器、高度是猜的：第一次读到就按编码器认准 */
        FmtF(s1, (int)sizeof(s1), mm);
        FmtF(s2, (int)sizeof(s2), z);
        snprintf(m, sizeof(m), "LIFTSYNC %s (guessed %s)\r\n", s2, s1);
        Say(m);
        if (Absf(z - mm) >= g_lchk)
        {
            Emm_V5_Stop_Now(LIFT_ADDR, false);
            HAL_Delay(5);
        }
        lift_mm = z;
        lift_guess = 0;
        lift_boot_how = "SYNC";
        Lift_Span_Reset();
        return 0;
    }
    d = Absf(z - mm);
    if (d < 0.5f)  { Lift_Span_Reset(); return 0; }
    FmtF(s1, (int)sizeof(s1), mm);
    FmtF(s2, (int)sizeof(s2), z);
    if (d < g_lchk)
    {
        /* (10-21 实车"下降的时候不夹取，抬起来了再夹取")差一点没到(爪子顶到原料盘/地面/转盘槽底了)：闭环驱动器会一直使劲往下顶，
         * 爪子被压住合不上。叫它停住(要去的位置就成了现在的位置，不再顶)，报 LIFTPUSH 让树莓派提醒把那个高度调高一点 */
        Emm_V5_Stop_Now(LIFT_ADDR, false);
        HAL_Delay(5);
        lift_mm = z;
        snprintf(m, sizeof(m), "LIFTPUSH want=%s got=%s\r\n", s1, s2);
        Say(m);
        Lift_Span_Reset();
        return 0;
    }
    /* 编码器一圈 40mm：实际差 20mm 以上会认成差另一圈的某个数。只有落在这次走过的高度范围里(多 2mm)、差不到 15mm 才信 */
    bad = (d > 15.0f || z < lift_span_lo - 2.0f || z > lift_span_hi + 2.0f);
    snprintf(m, sizeof(m), "LIFTSLIP want=%s got=%s%s\r\n", s1, s2, bad ? " (too far, kept)" : "");
    Say(m);
    if (bad)
    {
        return 1;                                  /* 不可信：调用的地方报 ERR LIFT，不夹/不松爪子 */
    }
    Emm_V5_Stop_Now(LIFT_ADDR, false);             /* 闭环驱动器顶着到不了的位置会一直使劲：叫它停住，它要去的位置也就成了现在的位置 */
    HAL_Delay(5);
    lift_mm = z;
    Lift_Span_Reset();
    return 0;
}

/* 待机姿态(10-15 用户：上电、PARK 都回到自己定的高度/手臂角度/夹爪)。STBY SAVE 把现在的样子记下来，
 * 和升降标定放在同一个扇区(标定后面 0x40 字节处，6 个字)，Cal_Save 擦扇区时一起写回去 */
#define SB_ADDR        (ARM_CAL_ADDR + 0x40u)
#define SB_MAGIC       0x53544259u        /* "STBY" */
static uint8_t  sb_ok   = 0;
static float    sb_z    = 0.0f;
static float    sb_a1   = 0.0f;
static float    sb_a2   = 0.0f;
static uint32_t sb_claw = 0;

static void Sb_Load(void)
{
    const volatile uint32_t *f = (const volatile uint32_t *)SB_ADDR;
    uint32_t w[6];
    float z, a1, a2;
    int i;

    for (i = 0; i < 6; i++)
    {
        w[i] = f[i];
    }
    sb_ok = 0;
    if (w[0] != SB_MAGIC || w[5] != (w[0] ^ w[1] ^ w[2] ^ w[3] ^ w[4] ^ 0x5A5A5A5Au))
    {
        return;
    }
    memcpy(&z, &w[1], 4);
    memcpy(&a1, &w[2], 4);
    memcpy(&a2, &w[3], 4);
    if (!(z >= 0.0f && z <= 500.0f && w[4] >= 500u && w[4] <= 3000u))
    {
        return;
    }
    sb_z = z;
    sb_a1 = a1;
    sb_a2 = a2;
    sb_claw = w[4];
    sb_ok = 1;
}

static void Cal_Load(void)
{
    const volatile uint32_t *f = (const volatile uint32_t *)ARM_CAL_ADDR;
    uint32_t w[6];
    float e0, c, cpr;
    int i;

    Sb_Load();
    for (i = 0; i < 6; i++)
    {
        w[i] = f[i];
    }
    lift_cal_ok = 0;
    if (w[0] != CAL_MAGIC || w[5] != (w[0] ^ w[1] ^ w[2] ^ w[3] ^ w[4] ^ 0x5A5A5A5Au) || (w[4] != 0x31u && w[4] != 0x36u))
    {
        return;
    }
    memcpy(&e0, &w[1], 4);
    memcpy(&c, &w[2], 4);
    memcpy(&cpr, &w[3], 4);
    if (!(Absf(c) > 10.0f && Absf(c) < 50000.0f && cpr > 1000.0f && cpr < 70000.0f))
    {
        return;
    }
    lift_enc0 = e0;
    lift_cpm = c;
    lift_cpr = cpr;
    lift_src = (uint8_t)w[4];
    lift_cal_ok = 1;
}

static int Cal_Save(void)
{
    FLASH_EraseInitTypeDef er;
    uint32_t serr = 0;
    uint32_t w[6];
    int i;
    int ok = 1;

    w[0] = CAL_MAGIC;
    memcpy(&w[1], &lift_enc0, 4);
    memcpy(&w[2], &lift_cpm, 4);
    memcpy(&w[3], &lift_cpr, 4);
    w[4] = lift_src;
    w[5] = w[0] ^ w[1] ^ w[2] ^ w[3] ^ w[4] ^ 0x5A5A5A5Au;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    er.TypeErase = FLASH_TYPEERASE_SECTORS;
    er.Banks = FLASH_BANK_1;
    er.Sector = FLASH_SECTOR_7;
    er.NbSectors = 1;
    er.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    if (HAL_FLASHEx_Erase(&er, &serr) != HAL_OK)       /* 擦除 128KB 扇区约 1~2 秒 */
    {
        ok = 0;
    }
    for (i = 0; ok && i < 6; i++)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, ARM_CAL_ADDR + 4u * (uint32_t)i, w[i]) != HAL_OK)
        {
            ok = 0;
        }
    }
    if (ok && sb_ok)                               /* 待机姿态也在这个扇区：一起写回去 */
    {
        uint32_t s[6];
        s[0] = SB_MAGIC;
        memcpy(&s[1], &sb_z, 4);
        memcpy(&s[2], &sb_a1, 4);
        memcpy(&s[3], &sb_a2, 4);
        s[4] = sb_claw;
        s[5] = s[0] ^ s[1] ^ s[2] ^ s[3] ^ s[4] ^ 0x5A5A5A5Au;
        for (i = 0; ok && i < 6; i++)
        {
            if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, SB_ADDR + 4u * (uint32_t)i, s[i]) != HAL_OK)
            {
                ok = 0;
            }
        }
    }
    HAL_FLASH_Lock();
    pos_last_ok = 0;                               /* 扇区擦掉了，位置记录也没了 */
    if (ok)
    {
        Cal_Load();                                /* 读回来校验 */
        ok = lift_cal_ok;
    }
    return ok;
}

/* ---- 升降位置记录 ---- */
static int Pos_Empty(uint32_t i)
{
    const volatile uint32_t *f = (const volatile uint32_t *)(POS_BASE + 8u * i);
    return f[0] == 0xFFFFFFFFu && f[1] == 0xFFFFFFFFu;
}

/* 已经记了多少条(记录是从头往后连续写的，二分查找第一条空的) */
static uint32_t Pos_Count(void)
{
    uint32_t lo = 0;
    uint32_t hi = POS_N;

    while (lo < hi)
    {
        uint32_t mid = (lo + hi) / 2u;
        if (Pos_Empty(mid))
        {
            hi = mid;
        }
        else
        {
            lo = mid + 1u;
        }
    }
    return lo;
}

/* 读最后一次记下的高度。没有记录或记录坏了返回 0 */
static int Pos_Last(float *z)
{
    uint32_t n = Pos_Count();
    const volatile uint32_t *f;
    uint32_t w0;

    if (n == 0u)
    {
        return 0;
    }
    f = (const volatile uint32_t *)(POS_BASE + 8u * (n - 1u));
    w0 = f[0];
    if (f[1] != ~w0)
    {
        return 0;
    }
    memcpy(z, &w0, 4);
    return (*z > -10.0f && *z < 500.0f);
}

/* 记下升降现在的高度(标定过编码器才记)。每次 8 字节，写一次几十微秒 */
static void Pos_Save(float z)
{
    uint32_t n;
    uint32_t w0;
    uintptr_t a;

    if (!lift_cal_ok || (pos_last_ok && Absf(z - pos_last) < 0.01f))
    {
        return;
    }
    n = Pos_Count();
    if (n >= POS_N)
    {
        return;                                    /* 记满了：下次开机擦掉重来 */
    }
    memcpy(&w0, &z, 4);
    a = POS_BASE + 8u * n;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, a, w0) == HAL_OK &&
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, a + 4u, ~w0) == HAL_OK)
    {
        pos_last = z;
        pos_last_ok = 1;
    }
    HAL_FLASH_Lock();
}

/* LIFT CAL <mm>：升降现在离最低点 z0 毫米(尺子量的)。上下动 10mm 测编码器，存进 Flash */
static const char *Lift_Cal_Do(float z0)
{
    static const float cands[3] = { 65536.0f, 16384.0f, 4096.0f };
    static const uint8_t srcs[2] = { 0x31, 0x36 };
    uint16_t a[2];
    uint16_t b[2];
    uint8_t ga[2];
    uint8_t gb[2];
    float step, mm_rev, d = 0.0f, cpr = 0.0f;
    const char *r;
    char m[96];
    char s1[20];
    char s2[20];
    int i, j;

    if (z0 < 0.0f || z0 > g_lmax)
    {
        return "ERR RANGE";
    }
    for (j = 0; j < 2; j++)                        /* 两种值都读：走 10mm 后哪个变化对得上就用哪个 */
    {
        ga[j] = (uint8_t)Lift_ReadSrc(srcs[j], &a[j]);
    }
    if (!ga[0] && !ga[1])
    {
        return "ERR NOENC";                        /* 读不到：驱动器 TX 没接到 PC11，或 5 号驱动器没回复。(10-21)这时不改程序记的高度 */
    }
    lift_known = 1;
    lift_mm = z0;
    Lift_Span_Reset();
    lift_guess = 0;
    step = (z0 + 15.0f <= g_lmax) ? 10.0f : -10.0f;   /* 在最低点附近就往上走，离最高点近才往下走，不会撞 */
    r = Lift_Goto(z0 + step);
    if (r != NULL || car_abort)
    {
        return (r != NULL) ? r : "ERR ABORT";
    }
    HAL_Delay(300);
    for (j = 0; j < 2; j++)
    {
        gb[j] = (uint8_t)Lift_ReadSrc(srcs[j], &b[j]);
    }
    /* 判断编码器一圈是多少：10mm 应该是 10/40 圈 */
    mm_rev = g_lspr / g_lppm;
    for (j = 0; j < 2 && cpr == 0.0f; j++)
    {
        if (!ga[j] || !gb[j])
        {
            continue;
        }
        for (i = 0; i < 3; i++)
        {
            float di = Enc_Wrap((float)b[j] - (float)a[j], cands[i]);
            float ratio = Absf(di) / cands[i] * mm_rev / Absf(step);
            if (ratio > 0.8f && ratio < 1.25f)
            {
                d = di;
                cpr = cands[i];
                lift_src = srcs[j];
                break;
            }
        }
    }
    if (cpr == 0.0f)
    {
        /* 把读数打出来，方便查：走之前 -> 走之后(NO = 没回复) */
        snprintf(m, sizeof(m), "LIFTCAL RAW 31:%ld->%ld 36:%ld->%ld\r\n",
                 ga[0] ? (long)a[0] : -1L, gb[0] ? (long)b[0] : -1L, ga[1] ? (long)a[1] : -1L, gb[1] ? (long)b[1] : -1L);
        Say(m);
        Lift_Goto(z0);                             /* 回到原来的位置 */
        return "ERR ENCMOVE";                      /* 编码器变化和走的距离对不上：电机没转、或读的不是升降电机 */
    }
    r = Lift_Goto(z0);                             /* 回到原来的位置 */
    if (r != NULL || car_abort)
    {
        return (r != NULL) ? r : "ERR ABORT";
    }
    HAL_Delay(300);
    if (!Lift_ReadEnc(&a[0]))
    {
        return "ERR NOENC";
    }
    lift_cpr = cpr;
    lift_cpm = d / step;
    lift_enc0 = (float)a[0] - lift_cpm * z0;
    lift_enc0 -= cpr * floorf(lift_enc0 / cpr);   /* 折到 0~一圈 */
    if (!Cal_Save())
    {
        return "ERR FLASH";
    }
    FmtF(s1, (int)sizeof(s1), lift_enc0);
    FmtF(s2, (int)sizeof(s2), lift_cpm);
    snprintf(m, sizeof(m), "LIFTCAL OK ENC0=%s CPM=%s CPR=%ld SRC=%02X\r\n", s1, s2, (long)cpr, (unsigned)lift_src);
    Say(m);
    Pos_Save(z0);
    return NULL;
}

static const char *Lift_Cal(float z0)
{
    const char *r;

    lift_nochk = 1;                                /* 上下走那两步不核对 */
    r = Lift_Cal_Do(z0);
    lift_nochk = 0;
    return r;
}

/* (10-21)LIFT ZERO：标定过编码器就把编码器的"高度 0"也改到这里，存进 Flash(和在这里 LIFT CAL 0 一样，只是不再上下走一遍)。
 * 不改的话，每次走完核对、下次开机还按原来的 0 算，又把高度改回去了。本来就对得上(差不到 0.3mm)不用改 */
static void Lift_Rebase0(void)
{
    float z;
    uint16_t e;
    char m[48];
    char s1[20];

    if (!lift_cal_ok || !Lift_EncPos(0.0f, &z, &e) || Absf(z) < 0.3f)
    {
        return;
    }
    lift_enc0 = (float)e;
    lift_enc0 -= lift_cpr * floorf(lift_enc0 / lift_cpr);   /* 折到 0~一圈 */
    if (Cal_Save())
    {
        Pos_Save(0.0f);
    }
    FmtF(s1, (int)sizeof(s1), z);
    snprintf(m, sizeof(m), "LIFTZERO ENC %s -> 0\r\n", s1);   /* 编码器原来算的是多少 */
    Say(m);
}

/* 开机：用编码器找到准确高度，再走到 LIFT_BOOT_MM。
 * 以"上次停下的高度"(Flash 里记的)为中心找，所以断电时停在哪都行，只要断电后别用手把它挪开 2cm 以上。
 * 没标定/读不到编码器时，当作正好在 LIFT_BOOT_MM，不动 */
static void Lift_Boot(void)
{
    uint16_t e = 0;
    float z = LIFT_BOOT_MM;
    float near_mm = LIFT_BOOT_MM;
    char m[80];
    char s1[20];
    char s2[20];
    int k;
    int got = 0;

    lift_known = 1;
    lift_mm = LIFT_BOOT_MM;
    Lift_Span_Reset();
    lift_boot_how = "NOCAL";
    Cal_Load();
    if (!lift_cal_ok)
    {
        Say("LIFTBOOT NOCAL (assume 60)\r\n");
        return;
    }
    pos_last_ok = 0;
    if (Pos_Last(&near_mm))
    {
        pos_last = near_mm;
        pos_last_ok = 1;
    }
    else
    {
        near_mm = LIFT_BOOT_MM;                    /* 没有记录：当作在 60 附近 */
    }
    lift_boot_how = "NOENC";
    lift_guess = 1;
    for (k = 0; k < 40 && !got; k++)               /* 驱动器上电要一会儿才能回复，(10-21)最多等约 6 秒 */
    {
        got = Lift_EncPos(near_mm, &z, &e);
        if (!got)
        {
            HAL_Delay(100);
        }
    }
    if (!got)
    {
        if (pos_last_ok)
        {
            /* (10-16)读不到编码器：当作停在 Flash 里记的上次停下的高度(比一律当 60 准：停在待机高度关电的也不会认错) */
            lift_mm = pos_last;
            Lift_Span_Reset();
            FmtF(s2, (int)sizeof(s2), pos_last);
            snprintf(m, sizeof(m), "LIFTBOOT NOENC (assume last %s)\r\n", s2);
            Say(m);
            return;
        }
        Say("LIFTBOOT NOENC (assume 60)\r\n");
        return;
    }
    lift_mm = z;
    Lift_Span_Reset();
    lift_guess = 0;
    lift_boot_how = pos_last_ok ? "POS" : "ENC";
    FmtF(s1, (int)sizeof(s1), z);
    FmtF(s2, (int)sizeof(s2), near_mm);
    snprintf(m, sizeof(m), "LIFTBOOT %s (last %s) -> %ld\r\n", s1, s2, (long)LIFT_BOOT_MM);
    Say(m);
    if (Pos_Count() > POS_N / 4u * 3u)             /* 位置记录快满了：擦掉扇区，标定重新写回去 */
    {
        Cal_Save();
        Pos_Save(lift_mm);                         /* (10-21)擦掉以后马上记一条：这次开机升降一下都没动就关电，下次开机也知道停在哪 */
    }
    if (!sb_ok)                                    /* 存了待机姿态：Arm_Init 里整个回到待机姿态，这里先不动 */
    {
        Lift_Goto(LIFT_BOOT_MM);
    }
}

/* ================= ID1 / ID2 ================= */
/* 管前后伸缩的是哪个舵机(参数 AEXT)，另一个管旋转 */
static uint8_t Ext_Id(void)
{
    return (g_aext < 1.5f) ? 1 : 2;
}

/* (10-21 用户：伸缩的要最快、旋转的要慢一点)大动作时舵机 id 的转速：伸缩舵机 EXTSPD、旋转舵机 ROTSPD，是 0 就用 ASPD */
static float Big_Spd(uint8_t id)
{
    float v = (id == Ext_Id()) ? g_extspd : g_rotspd;

    return (v > 0.5f) ? v : g_aspd;
}

/* ID1、ID2 转到 a1、a2(度)。PARA=1 时两个一起转(省时间)；否则按 order 先后：
 *   order=1：先动伸缩舵机再转(缩回时先缩回再转，免得伸着转扫到东西)
 *   order=2：先转再动伸缩舵机(伸出时先转好再伸)
 * tol：到位误差(度)，speed：转速(度/秒)，0 = 大动作(各按 Big_Spd) */
static void Servos_To(float a1, float a2, uint8_t order, float tol, float speed)
{
    float s1 = (speed > 0.5f) ? speed : Big_Spd(1);
    float s2 = (speed > 0.5f) ? speed : Big_Spd(2);

    if (g_para > 0.5f)
    {
        uint32_t t1 = 0;
        uint32_t t2 = 0;
        uint32_t tmax;
        uint32_t t0;
        uint8_t ok1 = 0;
        uint8_t ok2 = 0;
        float a;
        float rem1 = 1e9f;
        float rem2 = 1e9f;

        /* 已经在目标上的那个不发转动指令(发了会往前多给补偿角度，白白动一下)，只让它在原地保持(松开过的舵机也重新上力) */
        if (Servo_ReadAngle(1, &a) && Absf(a - a1) <= tol)  { ok1 = 1; Servo_Hold(1, a); }  else  t1 = Servo_Start(1, &a1, s1);
        if (Servo_ReadAngle(2, &a) && Absf(a - a2) <= tol)  { ok2 = 1; Servo_Hold(2, a); }  else  t2 = Servo_Start(2, &a2, s2);
        tmax = ((t1 > t2) ? t1 : t2) + SERVO_EXTRA_MS;
        t0 = HAL_GetTick();
        while ((HAL_GetTick() - t0) < tmax && !(ok1 && ok2) && !car_abort)
        {
            /* 快到了查勤一点(8ms)，到了马上停住：发给舵机的目标往前多给了补偿角度，停晚了会转过头 */
            HAL_Delay((rem1 < SERVO_FINE_DEG || rem2 < SERVO_FINE_DEG) ? 8u : 30u);
            if (!ok1 && Servo_ReadAngle(1, &a))
            {
                if (Servo_Arrived(1, a, tol))  { ok1 = 1; Servo_Hold(1, a); }
                else  rem1 = Absf(a - a1);
            }
            if (!ok2 && Servo_ReadAngle(2, &a))
            {
                if (Servo_Arrived(2, a, tol))  { ok2 = 1; Servo_Hold(2, a); }
                else  rem2 = Absf(a - a2);
            }
            if (ok1)  rem1 = 1e9f;
            if (ok2)  rem2 = 1e9f;
        }
        if (car_abort)
        {                                          /* 急停：没到的也停在现在的位置 */
            if (!ok1 && Servo_ReadAngle(1, &a))  Servo_Hold(1, a);
            if (!ok2 && Servo_ReadAngle(2, &a))  Servo_Hold(2, a);
            return;
        }
        /* 到时间还没到位：接着等它；停住了(被挡住)就让它停在原地，别一直顶着发热 */
        if (!ok1)  Servo_StopIfStuck(1, a1);
        if (!ok2)  Servo_StopIfStuck(2, a2);
    }
    else
    {
        uint8_t first = (order == 1) ? Ext_Id() : (uint8_t)(3 - Ext_Id());
        uint8_t second = (uint8_t)(3 - first);

        Servo_Move(first, (first == 1) ? a1 : a2, (first == 1) ? s1 : s2, tol, SERVO_EXTRA_MS);
        Servo_Move(second, (second == 1) ? a1 : a2, (second == 1) ? s1 : s2, tol, SERVO_EXTRA_MS);
    }
}

/* ================= 夹爪 / 转盘 ================= */
static uint8_t  tt_slot     = 1;
static uint32_t tt_ready_at = 0;

/* 夹爪慢慢转到 target 微秒：往张开的方向用 CLSPO，往合上的方向用 CLSPD；速度 > 0 时每 10 毫秒挪一点
 * (用户嫌夹爪一下子张开/合上太猛)；速度 = 0 一下子到。
 * (10-21 用户"放 1、2 号环的时候爪子莫名其妙松开")夹爪每动一次都回一行 CLAWSET <微秒> <哪一步>(O 张开 / OR 原料盘上方张开 /
 * OT 车上转盘上方张开 / C 夹紧 / STBY 待机姿态 / US 直接给的脉宽)，树莓派日志里看得到是哪一步让它动的；
 * 爪子松了日志里却没有这一行 = 不是程序让它张开的(查夹爪舵机的线、插头、电源) */
static uint32_t claw_now  = CLAW_BOOT_US;
static uint8_t  claw_hold = 0;            /* (10-21 用户"运输过程中爪子一定要按第三步 CLWC 夹紧，不能松")GRAB/PICK/TAKE 夹紧了物料 = 1；
                                           * 放下(DROP、松进车上转盘、CLAW O/OR/OT/<微秒>)= 0。是 1 时别的流程都不张开爪子：
                                           * TAKE/GRAB/PICK 回 ERR HOLD，OBS … O、待机姿态只保持夹紧(回 CLAWKEEP) */
static void Claw_Move(uint32_t target, const char *why)
{
    uint8_t opening = ((g_clwo >= g_clwc) ? ((float)target > (float)claw_now) : ((float)target < (float)claw_now));
    float spd = opening ? g_clspo : g_clspd;

    if (target != claw_now)
    {
        char m[40];

        snprintf(m, sizeof(m), "CLAWSET %lu %s\r\n", (unsigned long)target, why);
        Say(m);
    }

    if (spd > 0.5f)
    {
        float cur  = (float)claw_now;
        float step = spd * 0.01f;
        float tgt  = (float)target;
        uint16_t guard = 0;

        if (step < 1.0f) step = 1.0f;
        while ((cur < tgt - 0.5f || cur > tgt + 0.5f) && guard < 2000)
        {
            if (cur < tgt) { cur += step; if (cur > tgt) cur = tgt; }
            else           { cur -= step; if (cur < tgt) cur = tgt; }
            Claw_Set((uint32_t)(cur + 0.5f));
            HAL_Delay(10);
            guard++;
        }
    }
    Claw_Set(target);
    claw_now = target;
}

/* 夹爪张开 / 夹紧(脉宽是参数 CLWO / CLWC) */
/* (10-21 用户)夹爪张开只分两种：原料区原料盘上方下爪前 CLWOR；其他所有地方(车上转盘、圆环上放下、观察姿态…)都和车上转盘上方一样用 CLWOT。
 * CLWOT 是 0(没调)就用 CLWO；CLWOR 是 0 就和其他地方一样 */
static uint32_t Open_Us(void)
{
    return (uint32_t)((g_clwot >= 500.0f) ? g_clwot : g_clwo);
}

static void Claw_O(void)
{
    Claw_Move(Open_Us(), "O");
}

#define CLAW_AT_RAW   1
#define CLAW_AT_TRAY  2
static void Claw_OAt(uint8_t where)
{
    if (where == CLAW_AT_RAW && g_clwor >= 500.0f)
    {
        Claw_Move((uint32_t)g_clwor, "OR");
        return;
    }
    Claw_Move(Open_Us(), (where == CLAW_AT_RAW) ? "OR" : "OT");
}

static void Claw_C(void)
{
    Claw_Move((uint32_t)g_clwc, "C");
}

static void TT_Go(uint8_t slot)
{
    float us = (slot == 1) ? g_tt1 : ((slot == 2) ? g_tt2 : g_tt3);

    if (slot != tt_slot)
    {
        tt_slot = slot;
        tt_ready_at = HAL_GetTick() + (uint32_t)g_ttwait;
    }
    Turntable_Set((uint32_t)us);
}

static int TT_WaitReady(void)
{
    int32_t left = (int32_t)(tt_ready_at - HAL_GetTick());
    if (left > 0)
    {
        return Wait_Ms((uint32_t)left);
    }
    return car_abort ? 0 : 1;
}

/* (10-14 用户)升降和转动一起做：先直着升 OVCLR(离开物料/转盘槽)，再一边升到 ZHI 一边转到 (a1, a2)；
 * 转过一半、而且已经升到 ZHI 以后开始往 z_end + OVCLR 降(一边降一边转)，转到了再直着降到 z_end。
 * OVLAP=0 或者 PARA=0(两个舵机一个一个转)：按以前的做法(升到 ZHI -> 转 -> 降)。 */
static const char *Move_Over(float a1, float a2, uint8_t order, float z_end)
{
    float clr = g_ovclr;
    float z_mid = z_end + clr;
    float zc;
    float a;
    uint32_t t1 = 0, t2 = 0, tmax, t0;
    uint8_t ok1 = 0, ok2 = 0, down = 0;
    const char *e;

    if (g_ovlap < 0.5f || g_para < 0.5f)
    {
        e = Lift_Goto(g_zhi);
        if (e) return e;
        if (car_abort) return NULL;
        Servos_To(a1, a2, order, g_atolc, 0.0f);
        if (car_abort) return NULL;
        return Lift_Goto(z_end);
    }
    if (z_mid > g_zhi)  z_mid = g_zhi;
    zc = lift_mm + clr;                            /* 先直着升一小段 */
    if (zc > g_zhi)  zc = g_zhi;
    if (zc > lift_mm + 0.05f)
    {
        e = Lift_Goto(zc);
        if (e) return e;
        if (car_abort) return NULL;
    }
    e = Lift_Start(g_zhi);                         /* 一边升一边转 */
    if (e) return e;
    if (Servo_ReadAngle(1, &a) && Absf(a - a1) <= g_atolc)  { ok1 = 1; Servo_Hold(1, a); }  else  t1 = Servo_Start(1, &a1, Big_Spd(1));
    if (Servo_ReadAngle(2, &a) && Absf(a - a2) <= g_atolc)  { ok2 = 1; Servo_Hold(2, a); }  else  t2 = Servo_Start(2, &a2, Big_Spd(2));
    tmax = ((t1 > t2) ? t1 : t2) + SERVO_EXTRA_MS;
    t0 = HAL_GetTick();
    while (!(ok1 && ok2) && (HAL_GetTick() - t0) < tmax && !car_abort)
    {
        HAL_Delay(15);
        if (!ok1 && Servo_ReadAngle(1, &a) && Servo_Arrived(1, a, g_atolc))  { ok1 = 1; Servo_Hold(1, a); }
        if (!ok2 && Servo_ReadAngle(2, &a) && Servo_Arrived(2, a, g_atolc))  { ok2 = 1; Servo_Hold(2, a); }
        /* 升到顶了、转过一半了：开始往目标上方降(一边降一边转) */
        if (!down && Lift_Done_Now() && (HAL_GetTick() - t0) * 2u >= tmax - SERVO_EXTRA_MS && z_mid < lift_mm - 0.05f)
        {
            down = 1;
            e = Lift_Start(z_mid);
            if (e) return e;
        }
    }
    if (car_abort)
    {
        if (!ok1 && Servo_ReadAngle(1, &a))  Servo_Hold(1, a);
        if (!ok2 && Servo_ReadAngle(2, &a))  Servo_Hold(2, a);
        return Lift_Finish();                      /* 停下升降、找回高度 */
    }
    if (!ok1)  Servo_StopIfStuck(1, a1);
    if (!ok2)  Servo_StopIfStuck(2, a2);
    e = Lift_Finish();
    if (e) return e;
    if (car_abort) return NULL;
    return Lift_Goto(z_end);                       /* 最后一段直着降(或升) */
}

/* ================= 夹取 / 放置流程 ================= */
#define GO(expr)        do { const char *e_ = (expr); if (e_) return e_; if (car_abort) return NULL; } while (0)
#define WAIT(ms)        do { if (!Wait_Ms((uint32_t)(ms))) return NULL; } while (0)
#define SERVOS(a1, a2, order, tol, spd)   do { Servos_To((a1), (a2), (order), (tol), (spd)); if (car_abort) return NULL; } while (0)

static const char *Seq_Ready(void)
{
    if (g_armok < 0.5f)        return "ERR NOCAL";     /* 姿态参数还没标定 */
    if (!lift_known)           return "ERR NOZERO";
    return NULL;
}

static int Slot_Ok(long slot)
{
    return slot >= 1 && slot <= 3;
}

static float A2_Spd(void)
{
    return (g_a2spd > 0.5f) ? g_a2spd : Big_Spd(2);
}

/* 去/离开车上转盘绕开电池盒用哪一组：t1 = 靠原料盘/圆环那头开始伸长的 ID1 角度，ext = 伸多长，t2 = 靠车上转盘那头开始缩回的角度，
 * spd = 伸缩速度。(10-20)手臂在圆环那边(g_side)、圆环那组调过(R2T 不是 0)用圆环那组，不然用原料区那组(A2T 不是 0)。
 * 返回 0 = 不绕(照旧直接转)，1 = 原料区那组，2 = 圆环那组。只在 ID2 管伸缩(AEXT=2)时绕 */
typedef struct { float t1, ext, t2, spd; uint8_t stg; } Detour;

static int Detour_Get(Detour *d)
{
    if (Ext_Id() != 2)  return 0;
    if (g_side && Absf(g_r2t) > 0.5f)
    {
        d->t1 = g_r1t1;  d->ext = g_r2t;  d->t2 = g_r1t2;  d->spd = (g_r2spd > 0.5f) ? g_r2spd : A2_Spd();
        d->stg = (g_r2stg > 0.5f);
        return 2;
    }
    if (Absf(g_a2t) > 0.5f)
    {
        d->t1 = g_a1t1;  d->ext = g_a2t;  d->t2 = g_a1t2;  d->spd = A2_Spd();
        d->stg = (g_a2stg > 0.5f);
        return 1;
    }
    return 0;
}

/* (10-23)PRE(提前摆臂)：1 = Sweep / Stages 里要看着角度发的那几条指令都发出去了就返回，不等舵机转完(车同时开过去) */
static uint8_t pre_nowait = 0;

/* (10-23)PRE 用：只发转动指令不等。已经在目标上(ATOLC 以内)就只在原地保持(和 Servos_To 一样，不白白往前多给补偿) */
static void Servo_Go(uint8_t id, float target, float spd)
{
    float a;

    if (Servo_ReadAngle(id, &a) && Absf(a - target) <= g_atolc)  Servo_Hold(id, a);
    else                                                          (void)Servo_Start(id, &target, spd);
}

/* (10-19 用户：伸缩的时候 ID1 不要停，一直转)ID1 一口气转到 a1_end；转过 at1 时 ID2 开始走到 b1(伸长)，
 * 转过 at2 时 ID2 开始走到 b2(缩回)。ID1 先到了还没转过的那一段就接着做。升降不动(调用前升到最高)。spd2 = ID2 的速度 */
static const char *Sweep(float a1_end, float at1, float b1, float at2, float b2, float spd2)
{
    float a = 0.0f, c, dir, tgt = a1_end, b;
    uint32_t t0, tmax;
    uint8_t ok1 = 0, ok2 = 1, ph = 0;

    if (!Servo_ReadAngle(1, &a))  a = at1;
    dir = (a1_end >= a) ? 1.0f : -1.0f;
    tmax = Servo_Start(1, &tgt, Big_Spd(1)) + SERVO_EXTRA_MS;
    t0 = HAL_GetTick();
    while (!car_abort)
    {
        HAL_Delay(15);
        if (!ok1 && Servo_ReadAngle(1, &a) && Servo_Arrived(1, a, g_atolc))  { ok1 = 1; Servo_Hold(1, a); }
        if (ph == 0 && (ok1 || (a - at1) * dir >= 0.0f))
        {
            b = b1;  tmax += Servo_Start(2, &b, spd2);  ph = 1;  ok2 = 0;          /* 转过 A1T1：开始伸长 */
        }
        if (ph == 1 && (ok1 ? ok2 : (a - at2) * dir >= 0.0f))
        {
            b = b2;  tmax += Servo_Start(2, &b, spd2);  ph = 2;  ok2 = 0;          /* 转过 A1T2：开始缩回 */
        }
        if (pre_nowait && ph == 2)  return NULL;   /* (10-23)PRE：最后一条伸缩指令也发了，剩下的让舵机自己转完 */
        if (!ok2 && Servo_ReadAngle(2, &c) && Servo_Arrived(2, c, g_atolc))  { ok2 = 1; Servo_Hold(2, c); }
        if (ok1 && ok2 && ph == 2)  break;
        if ((HAL_GetTick() - t0) > tmax + 3000u)  break;
    }
    if (car_abort)
    {
        if (Servo_ReadAngle(1, &a))  Servo_Hold(1, a);
        if (Servo_ReadAngle(2, &c))  Servo_Hold(2, c);
        return NULL;
    }
    if (!ok1)  Servo_StopIfStuck(1, a1_end);
    if (!ok2)  Servo_StopIfStuck(2, b2);
    return NULL;
}

/* (10-21 用户"先转动一个角度再伸长再转动一个角度再缩回，让我自由调")分段走：ID1 只转到 p1 -> ID2 只走到 l1 -> ID1 只转到 p2
 * -> ID2 只走到 l2 -> ID1 只转到 a1_end。每段走完才走下一段；p1、p2 是 0(没调)的那段不转。转按 ROTSPD/ASPD，伸缩按 spd2。升降不动 */
static const char *Stages(float p1, float l1, float p2, float l2, float a1_end, float spd2)
{
    if (Absf(p1) > 0.5f)
    {
        Servo_Move(1, p1, Big_Spd(1), g_atolc, SERVO_EXTRA_MS);
        if (car_abort)  return NULL;
    }
    Servo_Move(2, l1, spd2, g_atolc, SERVO_EXTRA_MS);
    if (car_abort)  return NULL;
    if (Absf(p2) > 0.5f)
    {
        Servo_Move(1, p2, Big_Spd(1), g_atolc, SERVO_EXTRA_MS);
        if (car_abort)  return NULL;
    }
    Servo_Move(2, l2, spd2, g_atolc, SERVO_EXTRA_MS);
    if (car_abort)  return NULL;
    if (pre_nowait)
    {
        Servo_Go(1, a1_end, Big_Spd(1));           /* (10-23)PRE：最后这一转不等(车同时开过去) */
        return NULL;
    }
    Servo_Move(1, a1_end, Big_Spd(1), g_atolc, SERVO_EXTRA_MS);
    return NULL;
}

/* 转到车上转盘上方，停在高度 z_end：升到最高 -> ID1 一直转到 A1D，路上转过 A1T1(R1T1) 伸到 A2T(R2T)、转过 A1T2(R1T2) 缩回 A2R -> 下降
 * (10-21)分段走(R2STG / A2STG = 1)：只转到 T1 -> 只伸到 T -> 只转到 T2 -> 只缩回 A2R -> 只转到 A1D -> 下降 */
static const char *Go_Tray(float z_end)
{
    float a;
    Detour d;

    if (!Detour_Get(&d))  return Move_Over(g_a1d, g_a2r, 1, z_end);
    GO(Lift_Goto(g_zhi));
    if (d.stg)
    {
        GO(Stages(d.t1, d.ext, d.t2, g_a2r, g_a1d, d.spd));
        return Lift_Goto(z_end);
    }
    if (!Servo_ReadAngle(1, &a))  a = g_a1d;
    GO(Sweep(g_a1d, (Absf(d.t1) > 0.5f) ? d.t1 : a, d.ext, (Absf(d.t2) > 0.5f) ? d.t2 : g_a1d, g_a2r, d.spd));
    return Lift_Goto(z_end);
}

/* 手臂在车上转盘上方(ID1 离 A1D 不到 15°)要转到 (a1_to, a2_to)：ID1 一直转。
 * 原料区那组：转过 A1T2(离开车上转盘)伸缩直接走到 a2_to(10-19 用户：回原料区不用先伸长再缩回，转到原料盘上面时伸缩就是最近那格的长度)；
 * (10-20)圆环那组：转过 R1T2 先伸到 R2T(要去的长度比 R2T 还长就直接伸到它)，转过 R1T1(过了电池盒)再伸缩到 a2_to */
static const char *Leave_Tray(float a1_to, float a2_to)
{
    float a, at, at1, b1;
    Detour d;
    int k = Detour_Get(&d);

    if (!Servo_ReadAngle(1, &a) || Absf(a - g_a1d) > 15.0f)  return NULL;
    if ((!k || !d.stg) && Ext_Id() == 2 && g_lvdeg > 0.5f && Absf((float)claw_now - g_clwc) < 30.0f && Absf(a1_to - a) > 1.0f)
    {
        /* (10-21 用户)夹着物料离开车上转盘：先只转 ID1 一段(伸缩不动)，离开电池盒那一块再伸缩 */
        float to = (Absf(a1_to - a) <= g_lvdeg) ? a1_to : ((a1_to > a) ? a + g_lvdeg : a - g_lvdeg);

        GO(Lift_Goto(g_zhi));
        Servo_Move(1, to, Big_Spd(1), g_atolc, SERVO_EXTRA_MS);
        if (car_abort)  return NULL;
        if (!Servo_ReadAngle(1, &a))  a = to;
    }
    if (!k)  return NULL;
    GO(Lift_Goto(g_zhi));
    at = (Absf(d.t2) > 0.5f) ? d.t2 : a;
    if (k == 1)  return d.stg ? Stages(d.t2, a2_to, 0.0f, a2_to, a1_to, d.spd) : Sweep(a1_to, at, a2_to, at, a2_to, d.spd);
    at1 = (Absf(d.t1) > 0.5f) ? d.t1 : a1_to;
    b1 = ((a2_to - g_a2r) * (d.ext - g_a2r) > 0.0f && Absf(a2_to - g_a2r) >= Absf(d.ext - g_a2r)) ? a2_to : d.ext;
    if (d.stg)  return Stages(d.t2, b1, d.t1, a2_to, a1_to, d.spd);   /* (10-21)只转到 R1T2 -> 只伸到 R2T -> 只转到 R1T1 -> 只缩到要的长度 -> 转过去 */
    return Sweep(a1_to, at, b1, at1, a2_to, d.spd);
}

/* 观察姿态：升到最高 -> ID1、ID2 转到工位上方 -> 升降到观察高度。ring=0 原料盘，ring=1 地上圆环。
 * open_claw=1 同时把夹爪张开(夹爪动的时候手臂也在动，不多花时间)。open_claw=0 时夹爪状态保持不变 */
static const char *Seq_Obs(uint8_t ring, uint8_t open_claw)
{
    const char *e = Seq_Ready();
    if (e) return e;

    g_side = ring;                                 /* (10-20)之后去/离开车上转盘按这一边的绕电池盒参数走 */
    if (open_claw)
    {
        if (claw_hold)  Say("CLAWKEEP OBS\r\n");   /* (10-21)夹着物料：不张开 */
        else if (ring)  Claw_O();
        else            Claw_OAt(CLAW_AT_RAW);    /* (10-20)原料盘上方张开用 CLWOR */
    }
    GO(Leave_Tray(ring ? g_a1p : g_a1g, ring ? g_a2p : g_a2e));
    GO(Move_Over(ring ? g_a1p : g_a1g, ring ? g_a2p : g_a2e, 2, ring ? g_zobrng : g_zobraw));
    return NULL;
}

/* (10-23 用户"到原料区、粗加工区、暂存区之前让机械臂提前转到需要的姿态，节省时间")PRE RAW / PRE RING：
 * 树莓派在车开往这个工位的最后一段直线(后面不再转弯)之前发。手臂照 OBS 的路线(先升到 ZHI；在车上转盘上方就照样绕电池盒)
 * 转向观察姿态，但不等转完：要看着 ID1 角度发的那几条(绕电池盒的伸缩)发完就回 DONE，剩下的舵机自己转完，车同时开过去。
 * 不降到观察高度：开车时爪子留在最高处 ZHI(OBS 也是在 ZHI 转过去的，比原料盘上的物料、障碍物都高)。
 * 到了树莓派照常发 OBS：已经在姿态上就只核对一下、补最后一点，再降到观察高度。爪子里夹着物料不做(ERR HOLD)。
 * 车走的时候来了急停：已经发出去的转动不会停下(最多再转一秒左右，转到观察姿态) */
static const char *Seq_Pre(uint8_t ring)
{
    float a1 = ring ? g_a1p : g_a1g;
    float a2 = ring ? g_a2p : g_a2e;
    float a;
    Detour d;
    const char *e = Seq_Ready();

    if (e) return e;
    if (claw_hold)  return "ERR HOLD";
    g_side = ring;                                 /* 之后去/离开车上转盘按这一边的绕电池盒参数走(和 OBS 一样) */
    if (ring)  Claw_O();
    else       Claw_OAt(CLAW_AT_RAW);
    GO(Lift_Goto(g_zhi));                          /* 先升到最高再转；收臂停在最高处时不动 */
    if (Servo_ReadAngle(1, &a) && Absf(a - g_a1d) <= 15.0f && Detour_Get(&d))
    {
        pre_nowait = 1;                            /* 在车上转盘上方：和 OBS 一样绕电池盒，最后一条指令发了就返回 */
        e = Leave_Tray(a1, a2);
        pre_nowait = 0;
        return e;
    }
    if (g_para > 0.5f)
    {
        Servo_Go(1, a1, Big_Spd(1));               /* 两个一起转，都不等 */
        Servo_Go(2, a2, Big_Spd(2));
        return NULL;
    }
    {
        uint8_t ext = Ext_Id();
        uint8_t rot = (uint8_t)(3 - ext);

        /* PARA=0：和 OBS 一样先转好再伸缩，伸缩这一步不等 */
        Servo_Move(rot, (rot == 1) ? a1 : a2, Big_Spd(rot), g_atolc, SERVO_EXTRA_MS);
        if (car_abort)  return NULL;
        Servo_Go(ext, (ext == 1) ? a1 : a2, Big_Spd(ext));
    }
    return NULL;
}

/* 手臂已经(用摄像头)对准了物料：下降夹紧 -> 抬起 -> 缩回并转到转盘上方 -> 下降 -> 转盘到位后松开 -> 抬起。
 * z = 夹取高度，ring = 1 是地上圆环(取回)、0 是原料盘。结束时手臂停在转盘上方的最高处 */
static const char *Seq_PickHere(uint8_t slot, float z, uint8_t ring)
{
    const char *e = Seq_Ready();
    if (e) return e;
    if (!Slot_Ok(slot))  return "ERR ARG";
    if (claw_hold)  return "ERR HOLD";             /* (10-21)爪子里还夹着物料：先张开就掉在半路 */

    TT_Go(slot);                                   /* 转盘先转，转的同时手臂也在动，省时间 */
    if (ring)  Claw_O();                           /* 确保下降时爪子是张开的(下降要一会儿，爪子这时候正好张开) */
    else       Claw_OAt(CLAW_AT_RAW);              /* (10-20)原料盘上方张开用 CLWOR */
    GO(Lift_Goto(z));                              /* 下降 */
    if (!TT_WaitReady()) return NULL;              /* (10-21 实车"下降的时候不夹取，抬起来了再夹取")车上转盘还在转就先等它停：
                                                    * 下降只要半秒，转盘要 TTWAIT 才转完，两个舵机一起使劲时夹爪会慢半拍、合不上 */
    Claw_C();      claw_hold = 1;  WAIT(g_clwait); /* 夹紧 */
    if (!ring && Absf(g_a2gb) > 0.5f && Ext_Id() == 2)
    {
        /* (10-23)原料盘上夹起以后先往回缩(A2GB)：直着抬 OVCLR、一边升到 ZHI 一边缩回，ID1 不转，爪子不从别的物料上面扫过去 */
        float c1 = 0.0f;
        if (Servo_ReadAngle(1, &c1))
        {
            GO(Move_Over(c1, g_a2gb, 1, g_zhi));
        }
    }
    GO(Go_Tray(g_zdrop));                          /* 抬起、缩回转到转盘上方、下降(OVLAP=1：边升边转、边转边降；A2T：伸着转过去) */
    if (!TT_WaitReady()) return NULL;              /* 转盘到位了才松手 */
    Claw_OAt(CLAW_AT_TRAY);  claw_hold = 0;  WAIT(g_clwait);   /* 松开：物料落进转盘((10-20)车上转盘上方张开用 CLWOT) */
    GO(Lift_Goto(g_zhi));
    return NULL;
}

/* 从车上转盘 slot 号位取出物料：手臂转到转盘上方 -> 下降夹紧 -> 抬起。结束时夹着物料停在转盘上方的最高处 */
static const char *Seq_Take(uint8_t slot)
{
    const char *e = Seq_Ready();
    if (e) return e;
    if (!Slot_Ok(slot))  return "ERR ARG";
    if (claw_hold)  return "ERR HOLD";             /* (10-21)爪子里还夹着物料(上一个没放下)：先张开就掉在半路 */

    TT_Go(slot);
    Claw_O();
    GO(Go_Tray((g_ovlap > 0.5f) ? g_zdrop + g_ovclr : g_zhi));   /* 转到转盘上方(OVLAP=1：停在槽口上面一点) */
    Claw_OAt(CLAW_AT_TRAY);                        /* (10-20)到了转盘上方：张开换成 CLWOT(没调就还是 CLWO，不动) */
    if (!TT_WaitReady()) return NULL;
    GO(Lift_Goto(g_zdrop));
    Claw_C();      claw_hold = 1;  WAIT(g_clwait); /* 夹起：之后到放下之前爪子一直夹紧 */
    GO(Lift_Goto(g_zhi));
    return NULL;
}

/* 手臂夹着物料，已经(用摄像头)对准了目标：下降 -> 松开 -> 抬起 -> ID2 缩回。
 * stack=1 是码垛(放在已有物料上，下降高度用 ZSTK)。
 * 最后只把伸缩舵机缩回(不转)：之后底盘要沿着圆环板挪动，爪子不能还伸在已经放好的物料上方 */
static const char *Seq_Drop(uint8_t stack)
{
    const char *e = Seq_Ready();
    if (e) return e;

    GO(Lift_Goto(stack ? g_zstk : g_zplc));
    Claw_O();      claw_hold = 0;  WAIT(g_clwait);
    if (g_ovlap > 0.5f && g_para > 0.5f)
    {
        /* (10-14 用户)放下以后先直着升 OVCLR，再一边升到 ZHI 一边缩回伸缩舵机(另一个保持现在的角度，不转) */
        float c1 = 0.0f, c2 = 0.0f;
        if (Servo_ReadAngle(1, &c1) && Servo_ReadAngle(2, &c2))
        {
            GO(Move_Over(Ext_Id() == 2 ? c1 : g_a1h, Ext_Id() == 2 ? g_a2r : c2, 1, g_zhi));
            return NULL;
        }
    }
    GO(Lift_Goto(g_zhi));
    if (Ext_Id() == 2)                             /* 只把伸缩舵机缩回，不转 */
    {
        Servo_Move(2, g_a2r, Big_Spd(2), g_atolc, SERVO_EXTRA_MS);
    }
    else
    {
        Servo_Move(1, g_a1h, Big_Spd(1), g_atolc, SERVO_EXTRA_MS);   /* ID1 管伸缩：缩到收起时的位置 */
    }
    if (car_abort) return NULL;
    return NULL;
}

/* 收臂待命：升到最高 -> 缩回、ID1 转到待命角度 */
static const char *Seq_Stow(void)
{
    const char *e = Seq_Ready();
    if (e) return e;

    GO(Lift_Goto(g_zhi));
    if (sb_ok)
    {
        /* (10-16 用户)只要一个默认位置：存了待机姿态(STBY SAVE)，收臂也回到它的手臂角度和高度。
         * 夹爪不动(收臂时可能夹着物料，张开会掉) */
        SERVOS(sb_a1, sb_a2, 1, g_atolc, 0.0f);
        GO(Lift_Goto(sb_z));
        return NULL;
    }
    SERVOS(g_a1h, g_a2r, 1, g_atolc, 0.0f);
    return NULL;
}

/* 停车待机：先像 STOW 那样收臂(升到最高 -> 缩回、ID1 转到待命角度)，再把升降降到 LIFT_BOOT_MM(60mm)。
 * 停在 60，下次开机不管有没有标定、读不读得到编码器，认出来的高度都对(见 Lift_Boot)。
 * ARMOK=0(姿态还没标定)时不动舵机，只降升降。升降位置不知道(急停打断过)就不动，回 ERR NOZERO */
/* 回到待机姿态(STBY SAVE 存的)：升到最高 -> 手臂转到待机角度 -> 夹爪 -> 升降到待机高度 */
static const char *Seq_Standby(void)
{
    if (!lift_known)           return "ERR NOZERO";
    if (!sb_ok)                return "ERR NOSTBY";

    GO(Lift_Goto(g_zhi));
    SERVOS(sb_a1, sb_a2, 1, g_atolc, 0.0f);
    if (claw_hold)  Say("CLAWKEEP STBY\r\n");     /* (10-21)夹着物料：夹爪保持夹紧，不换成待机姿态的 */
    else            Claw_Move(sb_claw, "STBY");
    GO(Lift_Goto(sb_z));
    return NULL;
}

static const char *Seq_Park(void)
{
    if (!lift_known)           return "ERR NOZERO";
    if (sb_ok)                 return Seq_Standby();   /* 存了待机姿态就回到它 */

    if (g_armok > 0.5f)
    {
        GO(Lift_Goto(g_zhi));
        SERVOS(g_a1h, g_a2r, 1, g_atolc, 0.0f);
    }
    GO(Lift_Goto(LIFT_BOOT_MM));
    return NULL;
}

/* ================= 串口屏 ================= */
/* 淘晶驰(TJC)串口屏，指令后面跟三个 0xFF。两种用法(参数 SCRMODE)：
 *  1(默认) 程序自己画字：用 xstr 指令把字画在固定位置。(10-23 用户"字太小、屏原来自带的东西容易点到、抓取放置要中文、排版要整齐")
 *          屏的工程(USART HMI 做)：横屏 480x320、page0 黑底、不放任何控件，只要两个字库：
 *            字库 0 = 大字，字高 120(任务码只用 0~9 和 +；数字、+ 一个 60 像素宽)，赛规任务码字高 >= 12mm
 *            字库 1 = 中字，字高 72，GB2312，要有"抓取放置"四个汉字和 0~9(汉字一个 72 宽、数字 36 宽)
 *  0       写控件：屏的工程里要放好名叫 t0、t7、t2、t3 的文本控件，发 t0.txt="文字"
 * 画模式下每个名字画在屏上哪里(480x320 横屏，字都在框里居中)：
 *   ┌────────────────────────────┐
 *   │          156+123+          │  t0 任务码前两组(带后面的 +)：8 个大字 = 480 宽     y=0~119
 *   ├────────────────────────────┤
 *   │          516+231           │  t7 任务码后两组：7 个大字 = 420 宽                 y=120~239
 *   ├──────────────┬─────────────┤
 *   │    抓取6     │   放置12    │  t2 / t3：树莓派回到启停区以后写，程序在数字前面    y=240~319
 *   └──────────────┴─────────────┘     加上"抓取"/"放置"(树莓派发 GRAB 6 / 6 都行，只取数字)
 * 开机、开跑前屏上什么都没有(黑底)；读到任务码画 t0、t7。以前的 t1/t4/t5/t6 小字不再画(表里没有的名字画模式下不发) */
#define SCR_FONT_BIG   0
#define SCR_FONT_MID   1
#define SCR_FONT_SMALL SCR_FONT_MID        /* 以前的选区页(ZONE ASK，旧流程)用的小字：现在只有中字 */
typedef struct { const char *obj; int16_t x, y, w, h; uint8_t font; uint16_t color; const char *label; } ScrBox;
static const ScrBox scr_box[] =
{
    { "t0",   0,   0, 480, 120, SCR_FONT_BIG, 65535u, 0 },                     /* 白 */
    { "t7",   0, 120, 480, 120, SCR_FONT_BIG, 65535u, 0 },
    { "t2",   0, 240, 240,  80, SCR_FONT_MID, 65535u, "\xD7\xA5\xC8\xA1" },   /* GB2312 "抓取" */
    { "t3", 240, 240, 240,  80, SCR_FONT_MID, 65535u, "\xB7\xC5\xD6\xC3" },   /* GB2312 "放置" */
};
#define SCR_BOX_N  ((int)(sizeof(scr_box) / sizeof(scr_box[0])))

static void Screen_Send(const char *buf, int n)
{
    static const uint8_t endb[3] = { 0xFF, 0xFF, 0xFF };

    if (n < 0)
    {
        return;
    }
    if (n >= 96)
    {
        n = 95;
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *)endb, 3, 100);
}

void Screen_Text(const char *obj, const char *text)
{
    char buf[96];
    int i;

    if (g_scrmode > 0.5f)
    {
        for (i = 0; i < SCR_BOX_N; i++)
        {
            if (strcmp(scr_box[i].obj, obj) == 0)
            {
                const ScrBox *b = &scr_box[i];
                char t[24];

                if (b->label)
                {
                    /* (10-23 用户"抓取放置要中文")"GRAB 3" / "3" 都只取最后那个数，前面加上中文：抓取3 */
                    const char *e = text + strlen(text);
                    const char *d;

                    while (e > text && (e[-1] < '0' || e[-1] > '9'))  e--;
                    d = e;
                    while (d > text && d[-1] >= '0' && d[-1] <= '9')  d--;
                    snprintf(t, sizeof(t), "%s%.*s", b->label, (int)(e - d), d);
                    text = t;
                }
                /* xstr x,y,w,h,字库,字色,背景色,水平居中(1中),垂直居中(1中),背景方式(1=纯色，顺便擦掉旧字),"文字" */
                Screen_Send(buf, snprintf(buf, sizeof(buf), "xstr %d,%d,%d,%d,%d,%u,0,1,1,1,\"%s\"",
                                          b->x, b->y, b->w, b->h, b->font, b->color, text));
                return;
            }
        }
        return;                                    /* 表里没有的名字：画模式下忽略 */
    }
    Screen_Send(buf, snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", obj, text));
}

/* 直接发一条原始的淘晶驰指令，例如 "page 1"、"t0.pco=63488" */
static void Screen_Raw(const char *cmd)
{
    static const uint8_t endb[3] = { 0xFF, 0xFF, 0xFF };

    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, (uint16_t)strlen(cmd), 100);
    HAL_UART_Transmit(&huart2, (uint8_t *)endb, 3, 100);
}

/* 回到比赛布局：画模式清屏(黑底)；写控件模式重新载入 page 0(控件都回来) */
static void Screen_Clear(void)
{
    char buf[16];

    if (g_scrmode > 0.5f)
    {
        Screen_Send(buf, snprintf(buf, sizeof(buf), "cls 0"));
    }
    else
    {
        Screen_Send(buf, snprintf(buf, sizeof(buf), "page 0"));
    }
}

/* ================= 选启停区(串口屏触摸) =================
 * (10-23 用户"不需要显示启停区1还是2")开机只清屏；树莓派 race 1|2 发 ZONE 1|2 进准备页：只有下面一个大按钮 START
 * (不写 "ZONE n"、没有状态行、没有 BACK)。ZONE ASK(旧流程)还会画选区页：左半边大按钮 "1"、右半边大按钮 "2"。
 * 按 START：s=1，状态显示 GO，以后不再收触摸(只启动一次)。
 * 树莓派用 ZONE? 查选了几区、按没按 START；开跑时发 ZONE LOCK：关掉触摸、清屏回到比赛布局。
 * 按下和松开都在同一个按钮里才算(按下去滑到别处再松开不算)。
 * 屏的工程不用改：发 sendxy=1 以后，屏被按下/松开时发回 67 XH XL YH YL 事件 FF FF FF(事件 01 按下、00 松开；
 * 屏睡眠时首字节是 68)。坐标高字节最大 01，数据里不会有连续三个 FF，所以按 FF FF FF 分帧。
 * sendxy 屏断电就忘了：开机、ZONE ASK / ZONE n、屏重新上电(发来 88 FF FF FF)时都再发一次。
 * 中断里只认按钮(几次比较)；画页面要阻塞两三百毫秒(9600 波特率)，都在主循环 Arm_Poll / 指令里做 */
#define ZUI_OFF     0          /* 比赛布局，不收触摸 */
#define ZUI_PICK    1          /* 选区页 */
#define ZUI_READY   2          /* 准备页 */
#define ZUI_GO      3          /* 准备页，已经按了 START(不再收触摸) */

#define ZBTN_1      1          /* 按钮号 1、2 正好是区号 */
#define ZBTN_2      2
#define ZBTN_START  3
#define ZBTN_BACK   4

/* 按钮位置(480x320 横屏)。画按钮和认触摸用同一组数 */
#define ZP_BTN_Y    52         /* 选区页：两个大按钮从 y=52 到底 */
#define ZP_BTN_W    236        /* 按钮 1：x 0~235；按钮 2：x 244~479(中间 8 像素的缝按了不算) */
#define ZR_START_Y  128        /* 准备页：START 从 y=128 到底，整个宽度 */
#define ZR_BACK_X   336        /* 准备页：BACK 画在右上角 x 336~471、y 8~71(认触摸时右上角 x≥330、y<80 都算) */
#define ZONE_TAP_GUARD_MS 500u /* 换页以后这么久以内按下的不算：手指抖一下、连按两下，不会选完区顺手就按到 START */

#define SCR_FB      12
static uint8_t          scr_rx;
static uint8_t          scr_fb[SCR_FB];    /* 正在收的一帧 */
static uint8_t          scr_n = 0;         /* 这一帧已经收了几个字节(超过 SCR_FB 的只计数不存) */
static uint8_t          scr_ff = 0;        /* 末尾连着几个 0xFF */
static uint8_t          scr_ok = 0;        /* 1 = USART2 接收开起来了(收得到触摸) */
static volatile uint8_t scr_boot = 0;      /* 屏刚上电(收到 88 FF FF FF)：主循环里重画选区页 */
static volatile uint8_t zone_ui = ZUI_OFF; /* 现在显示哪一页(中断里按它认按钮) */
static volatile uint8_t scr_down = 0;      /* 按下时落在哪个按钮上(0 = 没落在按钮上) */
static volatile uint8_t scr_tap = 0;       /* 按下、松开都在同一个按钮里：按钮号，主循环取走后清 0 */
static volatile uint32_t zone_page_at = 0; /* 最近一次换页/画完页面的时刻(毫秒) */
static uint8_t          zone_sel = 0;      /* 选了几区：0 = 还没选 */
static uint8_t          zone_go = 0;       /* 1 = 上次 ZONE ASK / ZONE n / 选区以后按过 START */
static char             zone_msg[21];      /* 状态行(ZONE MSG 写的，最多 20 个字) */

/* 触摸点落在这一页的哪个按钮上(在中断里调用，只比较几次) */
static uint8_t Zone_Hit(uint8_t ui, uint16_t x, uint16_t y)
{
    if (x >= 480 || y >= 320)
    {
        return 0;
    }
    if (ui == ZUI_PICK && y >= ZP_BTN_Y)
    {
        if (x < ZP_BTN_W)          return ZBTN_1;
        if (x >= 480 - ZP_BTN_W)   return ZBTN_2;
    }
    if (ui == ZUI_READY && y >= ZR_START_Y)
    {
        return ZBTN_START;                         /* (10-23)准备页只有 START，没有 BACK(右上角按了也不算) */
    }
    return 0;
}

/* 在中断里：一次按下(ev=01)或松开(ev=00)。松开时还在按下的那个按钮里，才记成一次触摸 */
static void Zone_Touch(uint16_t x, uint16_t y, uint8_t ev)
{
    uint8_t b = Zone_Hit(zone_ui, x, y);

    if (ev == 0x01)
    {
        if ((uint32_t)(HAL_GetTick() - zone_page_at) < ZONE_TAP_GUARD_MS)
        {
            b = 0;                                 /* 页面刚换(或还在画)：这一下不算 */
        }
        scr_down = b;
    }
    else if (ev == 0x00)
    {
        if (b != 0 && b == scr_down)
        {
            scr_tap = b;
        }
        scr_down = 0;
    }
}

/* 在串口中断里：收屏发回来的一个字节。收满一帧(末尾三个 FF)就认：触摸帧换成按钮，开机帧记下来，
 * 别的(比如指令出错时屏回的错误码)不管 */
void Arm_Scr_RxCplt(void)
{
    uint8_t c = scr_rx;

    if (scr_n == 0 && c == 0xFF)
    {
        /* 帧不会以 FF 开头(上一帧多出来的 FF)：丢掉 */
    }
    else
    {
        if (scr_n < SCR_FB)
        {
            scr_fb[scr_n] = c;
        }
        if (scr_n < 255)
        {
            scr_n++;
        }
        scr_ff = (c == 0xFF) ? (uint8_t)(scr_ff + 1) : 0;
        if (scr_ff >= 3)
        {
            if (scr_n == 9 && (scr_fb[0] == 0x67 || scr_fb[0] == 0x68))
            {
                Zone_Touch((uint16_t)(((uint16_t)scr_fb[1] << 8) | scr_fb[2]),
                           (uint16_t)(((uint16_t)scr_fb[3] << 8) | scr_fb[4]), scr_fb[5]);
            }
            else if (scr_n == 4 && scr_fb[0] == 0x88)
            {
                scr_boot = 1;
            }
            scr_n = 0;
            scr_ff = 0;
        }
    }
    HAL_UART_Receive_IT(&huart2, &scr_rx, 1);
}

/* USART2 出错(溢出、噪声)后重新开始接收：不重新开，以后就再也收不到触摸了 */
void Arm_Scr_RxRestart(void)
{
    scr_n = 0;
    scr_ff = 0;
    scr_down = 0;                                  /* 按下的那一帧可能丢了，这次按的不算 */
    HAL_UART_Receive_IT(&huart2, &scr_rx, 1);
}

/* 画一块：底色 bg 铺满 (x,y,w,h)，文字居中(字库 font、字色 fg) */
static void Zone_Box(int x, int y, int w, int h, int font, unsigned fg, unsigned bg, const char *txt)
{
    char buf[96];

    Screen_Send(buf, snprintf(buf, sizeof(buf), "xstr %d,%d,%d,%d,%d,%u,%u,1,1,1,\"%s\"", x, y, w, h, font, fg, bg, txt));
}

/* 状态行：只在选区页(ZONE ASK，旧流程)右上角画。(10-23 用户"按要求显示必要的字")准备页、比赛布局都不画，ZONE MSG 只记下来 */
static void Zone_ShowMsg(void)
{
    if (zone_ui == ZUI_PICK)
    {
        Zone_Box(240, 0, 240, 48, SCR_FONT_SMALL, 65504u, 0u, zone_msg);          /* 黄字 */
    }
}

/* 准备页下半部分：没按 START 时是绿色 START；按了以后换成黄色 GO。(10-23)不再有 BACK */
static void Zone_ShowButtons(void)
{
    if (zone_ui == ZUI_READY)
    {
        Zone_Box(0, ZR_START_Y, 480, 320 - ZR_START_Y, SCR_FONT_BIG, 0u, 2016u, "START");      /* 绿底黑字 */
    }
    else
    {
        Zone_Box(0, ZR_START_Y, 480, 320 - ZR_START_Y, SCR_FONT_BIG, 0u, 65504u, "GO");        /* 黄底黑字 */
    }
}

/* 把现在这一页整页画出来(约 250 字节，9600 波特率要 0.25 秒)。xy=1 先发 sendxy=1(打开触摸坐标上报) */
static void Zone_Show(uint8_t xy)
{
    char buf[16];

    if (zone_ui == ZUI_OFF)
    {
        return;
    }
    if (xy)
    {
        if (!scr_ok)
        {
            /* 开机时 USART2 接收没开起来：再试一次 */
            scr_n = 0;
            scr_ff = 0;
            scr_ok = (HAL_UART_Receive_IT(&huart2, &scr_rx, 1) == HAL_OK) ? 1 : 0;
        }
        Screen_Send(buf, snprintf(buf, sizeof(buf), "sendxy=1"));
    }
    Screen_Send(buf, snprintf(buf, sizeof(buf), "cls 0"));
    if (zone_ui == ZUI_PICK)
    {
        Zone_Box(0, 0, 240, 48, SCR_FONT_SMALL, 65535u, 0u, "START ZONE 1 OR 2");
        Zone_Box(0, ZP_BTN_Y, ZP_BTN_W, 320 - ZP_BTN_Y, SCR_FONT_BIG, 65535u, 31u, "1");                  /* 蓝底 */
        Zone_Box(480 - ZP_BTN_W, ZP_BTN_Y, ZP_BTN_W, 320 - ZP_BTN_Y, SCR_FONT_BIG, 65535u, 63488u, "2");  /* 红底 */
    }
    else
    {
        Zone_ShowButtons();                        /* (10-23)准备页只有 START：不写 "ZONE n"(启停区在终端 race 1|2 选) */
    }
    Zone_ShowMsg();
    zone_page_at = HAL_GetTick();                  /* 画完才开始算保护时间(画的时候按的也不算) */
}

/* 换页：先换状态再画。关中断把还没处理的按下/触摸清掉，免得上一页的按钮算到这一页上。
 * 选区页、准备页的状态行清空；GO 页状态行是 "GO" */
static void Zone_Page(uint8_t ui, uint8_t sel, uint8_t xy)
{
    __disable_irq();
    zone_ui = ui;
    scr_down = 0;
    scr_tap = 0;
    zone_page_at = HAL_GetTick();
    __enable_irq();
    zone_sel = sel;
    zone_go = (ui == ZUI_GO) ? 1 : 0;
    snprintf(zone_msg, sizeof(zone_msg), "%s", (ui == ZUI_GO) ? "GO" : "");
    Zone_Show(xy);
}

/* 主循环里处理一次触摸(中断里已经认好了按钮) */
static void Zone_Tap(uint8_t b)
{
    if (zone_ui == ZUI_PICK && (b == ZBTN_1 || b == ZBTN_2))
    {
        Zone_Page(ZUI_READY, b, 0);                /* 选好区：进准备页 */
    }
    else if (zone_ui == ZUI_READY && b == ZBTN_START)
    {
        /* 按了 START：只改下半部分和状态行(比整页重画快一半，树莓派早一点查到 s=1) */
        __disable_irq();
        zone_ui = ZUI_GO;
        scr_down = 0;
        scr_tap = 0;
        __enable_irq();
        zone_go = 1;
        snprintf(zone_msg, sizeof(zone_msg), "GO");
        Zone_ShowButtons();
        Zone_ShowMsg();
    }
    else if (zone_ui == ZUI_READY && b == ZBTN_BACK)
    {
        Zone_Page(ZUI_PICK, 0, 0);                 /* 回去重新选 */
    }
}

/* 选区结束：关掉触摸，清屏回到比赛布局(和开机以前一样，只是不写 READY)。选的区、按没按 START 还记着，ZONE? 照样能查 */
static void Zone_Lock(void)
{
    __disable_irq();
    zone_ui = ZUI_OFF;
    scr_down = 0;
    scr_tap = 0;
    __enable_irq();
    Screen_Clear();
}

/* 开机：(10-23 用户"不需要显示启停区1还是2，按要求显示必要的字")不再显示选区页，只清屏(黑底，什么字都没有)。
 * 树莓派 race 1|2 发 ZONE 1|2 才出 START 页；开跑后屏上只有树莓派写的任务码、抓取数、放置数 */
static void Screen_Boot(void)
{
    zone_sel = 0;
    zone_go = 0;
    zone_msg[0] = 0;
    __disable_irq();
    zone_ui = ZUI_OFF;
    scr_down = 0;
    scr_tap = 0;
    __enable_irq();
    Screen_Clear();
}

/* ================= 二维码 (UART5) ================= */
/* 扫码模块(连续扫描模式)每读到一次码就发一行(码 + 回车/换行)。
 * 中断里收到回车/换行就马上在这一行里找任务码，找到就锁存起来：F/S/R、LIFT、STOW 这些指令要阻塞好几秒，
 * 这期间主循环不跑，以前字节只能堆在缓冲里、堆满了清掉，车边走边扫到的码会丢。
 * 主循环(Arm_Poll)只负责写屏和告诉树莓派。不发换行的模块：60 毫秒没有新字节也算收完一条(Arm_Poll 里)；
 * 连着发、缓冲快满时，清空前在中断里也找一遍 */
#define QR_BUF   64
static uint8_t           qr_rx;
static char              qr_buf[QR_BUF];
static volatile uint8_t  qr_len = 0;
static volatile uint32_t qr_tick = 0;
static char              qr_latch[16];     /* 中断里锁存的任务码 */
static volatile uint8_t  qr_new = 0;       /* 1 = qr_latch 里有主循环还没取走的码 */
static char              qr_code[16];
static uint8_t           qr_valid = 0;
static uint8_t           qr_show = 0;      /* 1 = 新码还没写屏、还没告诉树莓派 */
static uint8_t           qr_said = 0;      /* 1 = 新码已经在 QR? 的回复里告诉树莓派了：Arm_Poll 只写屏，不再主动发 */
/* 诊断(QR RAW)：扫码器到底发来了什么。收到的字节数、串口出错次数、最后一条(收到换行或缓冲满时存下) */
static volatile uint32_t qr_nbytes = 0;
static volatile uint16_t qr_nerr = 0;
static char              qr_last[QR_BUF];
static volatile uint8_t  qr_last_len = 0;
/* 扫到码以后关掉扫码模块(不再一直扫、一直响)，开跑(QR CLR)时再打开。
 * 用 GM65 的串口指令：先读 0x0000 区(工作模式在最低两位：10 = 连续扫，01 = 等指令才扫)，只改这两位再写回去；
 * 只写模块的内存不存 flash：模块断电重启就回到原来的连续扫。全程不阻塞主循环(Arm_Poll 里一步步走) */
#define QRC_IDLE   0
#define QRC_READ   1
#define QRC_WRITE  2
#define QRC_ON     2u                      /* 连续扫 */
#define QRC_OFF    1u                      /* 等指令才扫(相当于关掉：不扫、不响、不亮) */
static uint8_t           qrc_state = QRC_IDLE;
static uint8_t           qrc_want = QRC_ON;
static uint8_t           qrc_tries = 0;
static uint32_t          qrc_t = 0;
static volatile uint8_t  qrc_cap = 0;      /* 1 = 中断里把收到的字节也存进 qrc_rsp(等模块的回复) */
static volatile uint8_t  qrc_n = 0;
static uint8_t           qrc_rsp[32];
static uint8_t           qr_auto_off = 1;  /* 1 = 扫到任务码就自动关(QR AUTO 0 / 1 改) */
static int8_t            qr_scan = -1;     /* 扫码模块现在：1 开着 0 关了 -1 不知道(还没设成功过) */
static uint8_t           qrc_fail = 0;     /* 上一次开/关模块没收到回复 */

/* 在 s 里找 "ddd+ddd+ddd+ddd" 这种 15 个字符的格式，找到就复制到 out。前后有多余字符也没关系 */
static int QR_Match(const char *s, int len, char *out)
{
    int i, k;

    for (i = 0; i + 15 <= len; i++)
    {
        int ok = 1;
        for (k = 0; k < 15 && ok; k++)
        {
            char ch = s[i + k];
            if (k == 3 || k == 7 || k == 11)
            {
                ok = (ch == '+');
            }
            else
            {
                ok = (ch >= '0' && ch <= '9');
            }
        }
        if (ok)
        {
            memcpy(out, s + i, 15);
            out[15] = 0;
            return 1;
        }
    }
    return 0;
}

/* 在中断里：缓冲里这一条有任务码就锁存(最多比较几百次，几微秒) */
static void QR_Latch(void)
{
    char c[16];

    if (qr_len >= 15 && QR_Match(qr_buf, qr_len, c))
    {
        memcpy(qr_latch, c, sizeof(qr_latch));
        qr_new = 1;
    }
}

/* 在串口中断里：存一个字节；收到回车/换行(一条收完)就找任务码 */
void Arm_QR_RxCplt(void)
{
    char ch = (char)qr_rx;

    qr_nbytes++;
    if (qrc_cap && qrc_n < sizeof(qrc_rsp))
    {
        qrc_rsp[qrc_n++] = (uint8_t)ch;           /* 等模块回复时也存一份(回复是二进制，下面照常当文字处理也没关系) */
    }
    if (ch == '\r' || ch == '\n')
    {
        if (qr_len > 0)
        {
            memcpy(qr_last, qr_buf, qr_len);       /* 诊断用：存下这一条 */
            qr_last_len = qr_len;
        }
        QR_Latch();
        qr_len = 0;
    }
    else
    {
        if (qr_len >= QR_BUF - 1)
        {
            memcpy(qr_last, qr_buf, qr_len);
            qr_last_len = qr_len;
            QR_Latch();                            /* 太长(没有换行、连着发)：清空前先找一遍 */
            qr_len = 0;
        }
        qr_buf[qr_len++] = ch;
    }
    qr_tick = HAL_GetTick();
    HAL_UART_Receive_IT(&huart5, &qr_rx, 1);
}

void Arm_QR_RxRestart(void)
{
    qr_nerr++;
    qr_len = 0;
    HAL_UART_Receive_IT(&huart5, &qr_rx, 1);
}

/* ---- 开/关扫码模块(GM65 串口指令) ---- */
static void QRC_Send(uint8_t type, uint8_t data)
{
    uint8_t f[9];

    f[0] = 0x7E; f[1] = 0x00; f[2] = type; f[3] = 0x01;   /* 7E 00 类型 长度1 */
    f[4] = 0x00; f[5] = 0x00;                              /* 地址 0x0000(工作模式等) */
    f[6] = data;                                           /* 读：读几个字节；写：写的值 */
    f[7] = 0xAB; f[8] = 0xCD;                              /* AB CD = 不校验 */
    __disable_irq();
    qrc_n = 0;
    qrc_cap = 1;
    __enable_irq();
    HAL_UART_Transmit(&huart5, f, 9, 30);
    qrc_t = HAL_GetTick();
}

/* 模块的回复 02 00 00 01 数据 校验2：找到就取出数据字节 */
static int QRC_Reply(uint8_t *d)
{
    int i, n = qrc_n;

    for (i = 0; i + 5 <= n; i++)
    {
        if (qrc_rsp[i] == 0x02 && qrc_rsp[i + 1] == 0x00 && qrc_rsp[i + 2] == 0x00 && qrc_rsp[i + 3] == 0x01)
        {
            *d = qrc_rsp[i + 4];
            return 1;
        }
    }
    return 0;
}

static void QRC_Start(uint8_t want, uint8_t tries)
{
    qrc_want = want;
    qrc_tries = tries;
    qrc_state = QRC_READ;
    QRC_Send(0x07, 0x01);                                  /* 读 0x0000 区 1 个字节 */
}

static void QRC_Done(int ok)
{
    qrc_cap = 0;
    qrc_state = QRC_IDLE;
    qrc_fail = ok ? 0 : 1;
    if (ok)
    {
        qr_scan = (qrc_want == QRC_ON) ? 1 : 0;
    }
}

/* 主循环里一步步走：读 -> 只改最低两位 -> 写；300ms 没回复就重试 */
static void QRC_Poll(void)
{
    uint8_t d, nb;

    if (qrc_state == QRC_IDLE)
    {
        return;
    }
    if (QRC_Reply(&d))
    {
        if (qrc_state == QRC_READ)
        {
            nb = (uint8_t)((d & 0xFCu) | qrc_want);
            if (nb == d)
            {
                QRC_Done(1);                               /* 本来就是要的模式 */
                return;
            }
            qrc_state = QRC_WRITE;
            QRC_Send(0x08, nb);
            return;
        }
        QRC_Done(1);                                       /* 写好了(回 02 00 00 01 00 33 31) */
        return;
    }
    if (HAL_GetTick() - qrc_t > 300u)
    {
        if (qrc_tries > 1)
        {
            qrc_tries--;
            qrc_state = QRC_READ;
            QRC_Send(0x07, 0x01);
        }
        else
        {
            QRC_Done(0);
        }
    }
}

/* 主循环里：收到一个合格的任务码。扫码模块在连续模式下会反复发同一个码：
 * 同一个码只处理一次，免得一直重写串口屏、占用主循环 */
static void QR_Accept(const char *c)
{
    if (!qr_valid || strcmp(c, qr_code) != 0)
    {
        memcpy(qr_code, c, sizeof(qr_code));
        qr_valid = 1;
        qr_show = 1;
        qr_said = 0;
        if (qr_auto_off)
        {
            QRC_Start(QRC_OFF, 3);                 /* 扫到了：关掉扫码模块，不再一直扫、一直响 */
        }
    }
}

/* 主循环里：取走中断里锁存的码 */
static void QR_Fetch(void)
{
    char c[16];

    if (!qr_new)
    {
        return;
    }
    __disable_irq();
    memcpy(c, qr_latch, sizeof(c));
    qr_new = 0;
    __enable_irq();
    QR_Accept(c);
}

/* 屏上显示任务码。赛规要求字高不小于 12mm，3.5 寸屏一行放不下 15 个字符，所以分两行，组之间的 + 都留着：
 * t0 = 前两组带后面的 +(如 452+321+)，t7 = 后两组(如 254+312)。控件名不一样就改这里 */
static void QR_Draw(void)
{
    char h1[12];
    char h2[8];

    memcpy(h1, qr_code, 8);      h1[8] = 0;
    memcpy(h2, qr_code + 8, 7);  h2[7] = 0;
    Screen_Text("t0", h1);
    Screen_Text("t7", h2);
}

/* (10-23 用户"一键启动是一个物理的上电开关、整车的电闸")上电就开跑：树莓派开机、程序启动要几十秒，这段时间车一动不动会按
 * "停止运行 15 秒本轮结束"判掉。上电以后、树莓派发来第一条机械臂指令(之后树莓派自己保活)之前，每 BOOT_KA_MS 升降往上 BOOT_KA_MM
 * 再回来(手臂不左右动)，最多 BOOT_KA_MAX_MS(调试时开着电不开程序，过一会儿就不动了)。升降高度不知道(lift_known=0)不动。
 * (10-23 晚 命题解析："机器人 15 秒没有移动，本轮比赛结束")开机这段只有升降在动，4mm 裁判不一定看得出来：改成往上 15mm、每 5 秒一次。
 * 上面不够 15mm 就少走一点；不够 BOOT_KA_MIN_MM 才往下 BOOT_KA_MIN_MM(不会比原来低很多，碰不到车上的东西) */
#define BOOT_KA_MS      5000u
#define BOOT_KA_MAX_MS  90000u
#define BOOT_KA_MM      15.0f
#define BOOT_KA_MIN_MM  4.0f
static uint8_t  boot_ka    = 0;           /* 1 = 上电以后还没收到过机械臂指令(Arm_Init 设、Arm_Command 清) */
static uint32_t boot_ka_t0 = 0;           /* 上电(Arm_Init 做完)的时刻 */
static uint32_t boot_ka_t  = 0;           /* 上次摆的时刻 */

static void Boot_Keepalive(void)
{
    uint32_t t = HAL_GetTick();
    float z = lift_mm;
    float up = g_lmax - 0.5f - z;

    if (!boot_ka)
    {
        return;
    }
    if (t - boot_ka_t0 > BOOT_KA_MAX_MS)
    {
        boot_ka = 0;
        return;
    }
    if (t - boot_ka_t < BOOT_KA_MS || !lift_known || lift_pend >= 0.0f)
    {
        return;
    }
    boot_ka_t = t;
    if (up > BOOT_KA_MM)  up = BOOT_KA_MM;
    if (Lift_Goto((up >= BOOT_KA_MIN_MM) ? z + up : z - BOOT_KA_MIN_MM) == NULL && !car_abort)
    {
        (void)Lift_Goto(z);
    }
    boot_ka_t = HAL_GetTick();
}

void Arm_Poll(void)
{
    char tmp[QR_BUF];
    char newc[16];
    int n;
    uint8_t b;

    /* 屏刚重新上电(忘了 sendxy、画面也没了)：选区页还在用就整页重画；
     * 比赛布局时把任务码重新写上(赛规要一直显示；树莓派写过的同样内容不会再发) */
    if (scr_boot)
    {
        scr_boot = 0;
        if (zone_ui != ZUI_OFF)
        {
            Zone_Show(1);
        }
        else if (qr_valid)
        {
            QR_Draw();
        }
    }
    /* 触摸：中断里已经认好了按钮 */
    if (scr_tap)
    {
        __disable_irq();
        b = scr_tap;
        scr_tap = 0;
        __enable_irq();
        Zone_Tap(b);
    }

    /* 二维码：收到回车/换行的，中断里已经认好、锁存了 */
    QR_Fetch();
    QRC_Poll();
    /* 不发换行的模块：60 毫秒没有新字节，就把收到的拿去解析 */
    if (qr_len != 0 && (HAL_GetTick() - qr_tick) > 60u)
    {
        __disable_irq();
        n = qr_len;
        memcpy(tmp, qr_buf, (size_t)n);
        memcpy(qr_last, qr_buf, (size_t)n);
        qr_last_len = (uint8_t)n;
        qr_len = 0;
        __enable_irq();
        if (QR_Match(tmp, n, newc))
        {
            QR_Accept(newc);
        }
    }
    if (qr_show)
    {
        char m[32];

        qr_show = 0;
        /* 选区页还在显示时不写屏(会盖掉按钮)，码照样记着、照样告诉树莓派 */
        if (zone_ui == ZUI_OFF)
        {
            QR_Draw();
        }
        if (!qr_said)
        {
            snprintf(m, sizeof(m), "QR %s\r\n", qr_code);
            Say(m);
        }
        qr_said = 0;
    }
    Boot_Keepalive();                              /* (10-23)上电就开跑：树莓派还没接手之前隔几秒动一下升降 */
}

/* ================= 初始化 ================= */
void Arm_Init(void)
{
    /* 工程里原来没有启动 TIM2 的 PWM，夹爪和转盘一直没有输出。先设好初始脉宽再启动，避免开机时舵机猛地跳一下。
     * (10-21)存了待机姿态就直接用它的夹爪脉宽(不先张开到 2090 再合上：STM32 万一中途重启，夹着的物料不会先掉) */
    Sb_Load();
    claw_now = (sb_ok && sb_claw >= 500u && sb_claw <= 3000u) ? sb_claw : CLAW_BOOT_US;
    claw_hold = 0;
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, claw_now);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, TT_BOOT_US);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);

    tt_slot = 1;
    tt_ready_at = HAL_GetTick();

    huart2.Init.BaudRate = SCREEN_BAUD;            /* 屏 */
    HAL_UART_Init(&huart2);
    huart5.Init.BaudRate = QR_BAUD;                /* 扫码模块 */
    HAL_UART_Init(&huart5);

    qr_len = 0;
    qr_new = 0;
    HAL_UART_Receive_IT(&huart5, &qr_rx, 1);       /* UART5 的中断在 hal_msp.c 里已经打开 */
    QRC_Start(QRC_ON, 10);                         /* 单片机复位、扫码模块没断电时它可能还关着：打开(模块刚上电没回话就每 0.3 秒再试，最多 3 秒) */
    /* 收屏发回来的触摸坐标(选启停区)。USART2 的中断在 hal_msp.c 里也已经打开(main.c 里把优先级降到 3) */
    scr_n = 0;
    scr_ff = 0;
    scr_ok = (HAL_UART_Receive_IT(&huart2, &scr_rx, 1) == HAL_OK) ? 1 : 0;

    Servo_SetPower((uint16_t)g_spow);
    Servo_SetComp(1, g_s1comp);
    Servo_SetComp(2, g_s2comp);
    Servo_SetLimits(1, g_s1min, g_s1max);
    Servo_SetLimits(2, g_s2min, g_s2max);

    HAL_Delay(300);                                /* 等屏上电启动 */
    scr_boot = 0;                                  /* 这之前屏发来的开机帧不用管：下面马上画；这之后再来说明屏又重启了，Arm_Poll 重画 */
    Screen_Boot();                                 /* (10-23)只清屏，不再显示选区页 */

    /* 升降：0 = 最低点，往上为正，最高 100mm(LFMAX)。开机读编码器找到准确高度，自动走到 60mm(见 Lift_Boot) */
    Lift_Boot();
    if (sb_ok && (strcmp(lift_boot_how, "ENC") == 0 || strcmp(lift_boot_how, "POS") == 0))
    {
        Seq_Standby();                             /* 编码器找准了高度才自动动；没标定/读不到编码器就不动 */
    }
    boot_ka = 1;                                   /* (10-23)上电保活：树莓派发来第一条机械臂指令之前隔几秒动一下升降 */
    boot_ka_t0 = boot_ka_t = HAL_GetTick();
}

/* ================= 树莓派指令 ================= */
#define FAIL(s)   do { snprintf(err, (size_t)errlen, "%s", (s)); return -1; } while (0)
#define MAXTOK    4

/* 把一行指令按空格拆成最多 MAXTOK 个词，拆在 buf 里 */
static int Tokenize(const char *cmd, char *buf, int buflen, char *tok[MAXTOK])
{
    int n = 0;
    char *p;

    strncpy(buf, cmd, (size_t)buflen - 1);
    buf[buflen - 1] = 0;
    p = buf;
    while (*p != 0 && n < MAXTOK)
    {
        while (*p == ' ')
        {
            *p++ = 0;
        }
        if (*p == 0)
        {
            break;
        }
        tok[n++] = p;
        while (*p != 0 && *p != ' ')
        {
            p++;
        }
    }
    return n;
}

static int ParseL(const char *s, long *v)
{
    char *e;

    *v = strtol(s, &e, 10);
    return (e != s && *e == 0);
}

static int ParseF(const char *s, float *v)
{
    char *e;

    *v = (float)strtod(s, &e);
    if (!(*v - *v == 0.0f))  return 0;             /* (10-21)nan / inf：后面的范围检查都挡不住，不认 */
    return (e != s && *e == 0);
}

static void SayAngle(uint8_t id)
{
    float a;

    if (Servo_ReadAngle(id, &a))
    {
        char m[40];
        char b[20];

        FmtF(b, (int)sizeof(b), a);
        snprintf(m, sizeof(m), "ANG %d %s\r\n", (int)id, b);
        Say(m);
    }
}

int Arm_Command(const char *cmd, char *err, int errlen)
{
    char buf[64];
    char *t[MAXTOK];
    int n = Tokenize(cmd, buf, (int)sizeof(buf), t);
    const char *v;
    long id, slot;
    float f, g;
    const char *r;

    if (n == 0)
    {
        return 0;
    }
    v = t[0];
    boot_ka = 0;                                   /* (10-23)树莓派接手了：上电保活不再动 */

    /* ---- 夹爪：CLAW O / CLAW OR(原料盘上方张开) / CLAW OT(车上转盘上方张开) / CLAW C / CLAW <微秒> ---- */
    if (strcmp(v, "CLAW") == 0 && n == 2)
    {
        if (strcmp(t[1], "O") == 0)       Claw_O();
        else if (strcmp(t[1], "OR") == 0) Claw_OAt(CLAW_AT_RAW);
        else if (strcmp(t[1], "OT") == 0) Claw_OAt(CLAW_AT_TRAY);
        else if (strcmp(t[1], "C") == 0)  Claw_C();
        else
        {
            long us;
            if (!ParseL(t[1], &us) || us < 500 || us > 3000)  FAIL("ERR ARG");   /* 夹爪合上是 2910 */
            Claw_Move((uint32_t)us, "US");         /* main.c 里会再按夹爪限位夹住 */
        }
        if (strcmp(t[1], "C") != 0)
        {
            claw_hold = 0;                         /* (10-21)树莓派明说张开(放下、放回转盘、人手拿出物料)：不再当夹着物料 */
        }
        Wait_Ms((uint32_t)g_clwait);
        return 1;
    }

    /* ---- 转盘：TT <1~3> / TT <微秒> ---- */
    if (strcmp(v, "TT") == 0 && n == 2)
    {
        if (!ParseL(t[1], &slot))  FAIL("ERR ARG");
        if (slot >= 1 && slot <= 3)
        {
            TT_Go((uint8_t)slot);
        }
        else if (slot >= 500 && slot <= 2700)
        {
            Turntable_Set((uint32_t)slot);
            tt_slot = 0;                           /* (10-21)不在哪个号位了：之后 TT_Go 到哪个号位都要等它转到 */
            tt_ready_at = HAL_GetTick() + (uint32_t)g_ttwait;
        }
        else
        {
            FAIL("ERR ARG");
        }
        TT_WaitReady();
        return 1;
    }

    /* ---- 升降 ---- */
    if (strcmp(v, "LIFT?") == 0)
    {
        char m[48];
        char b[20];

        FmtF(b, (int)sizeof(b), lift_mm);
        snprintf(m, sizeof(m), "LIFT %d %s BOOT=%s\r\n", (int)lift_known, b, lift_boot_how);
        Say(m);
        return 1;
    }
    if (strcmp(v, "LIFT") == 0 && n == 2)
    {
        if (strcmp(t[1], "ZERO") == 0)
        {
            Emm_V5_En_Control(LIFT_ADDR, true, false);
            HAL_Delay(5);                                  /* 两条 Emm 指令之间要留空隙，连着发驱动器会当成一帧坏数据丢掉 */
            Emm_V5_Reset_CurPos_To_Zero(LIFT_ADDR);
            lift_mm = 0.0f;
            lift_known = 1;
            Lift_Rebase0();                        /* (10-21)编码器的 0 也改到这里 */
            Lift_Span_Reset();
            return 1;
        }
        if (strcmp(t[1], "HOME") == 0)
        {
            Emm_V5_En_Control(LIFT_ADDR, true, false);
            HAL_Delay(5);                                  /* 两条 Emm 指令之间要留空隙，连着发驱动器会当成一帧坏数据丢掉 */
            Emm_V5_Origin_Trigger_Return(LIFT_ADDR, (uint8_t)g_lhmd, false);
            if (!Wait_Ms((uint32_t)g_lhtm))
            {
                Emm_V5_Origin_Interrupt(LIFT_ADDR);
                return 1;                          /* 被急停，位置不明，不标记已回零 */
            }
            lift_mm = 0.0f;
            lift_known = 1;
            Lift_Span_Reset();
            return 1;
        }
        if (strcmp(t[1], "SYNC") == 0)             /* (10-21)按编码器把高度认准(在现在记的高度上下 20mm 里找)，不动升降 */
        {
            char m[64];
            char s1[20];
            char s2[20];
            uint16_t e;
            float z;

            if (!lift_cal_ok)  FAIL("ERR NOCAL");
            if (!lift_known)  FAIL("ERR NOZERO");      /* (10-21)急停停在半路(走了 36mm 以上)：记的高度是出发的地方，在它附近找会认错圈 */
            if (!Lift_EncPos(lift_mm, &z, &e))  FAIL("ERR NOENC");
            FmtF(s1, (int)sizeof(s1), z);
            FmtF(s2, (int)sizeof(s2), lift_mm);
            snprintf(m, sizeof(m), "LIFTSYNC %s (was %s)\r\n", s1, s2);
            Say(m);
            lift_mm = z;
            lift_known = 1;
            Lift_Span_Reset();
            if (lift_guess || strcmp(lift_boot_how, "NOENC") == 0)  lift_boot_how = "SYNC";
            lift_guess = 0;
            Pos_Save(z);
            return 1;
        }
        if (strcmp(t[1], "ENC?") == 0)             /* 读编码器，看程序算出来的高度对不对 */
        {
            char m[80];
            char s1[20];
            char s2[20];
            uint16_t e;
            float z;

            if (lift_cal_ok && Lift_EncPos(lift_mm, &z, &e))
            {
                FmtF(s1, (int)sizeof(s1), z);
                FmtF(s2, (int)sizeof(s2), lift_mm);
                snprintf(m, sizeof(m), "LIFTENC %u Z=%s NOW=%s SRC=%02X\r\n", (unsigned)e, s1, s2, (unsigned)lift_src);
            }
            else
            {
                uint16_t e2 = 0;
                int g1 = Lift_ReadSrc(0x31, &e);          /* 没标定(或读不到)：两种值都打出来，-1 = 没回复 */
                int g2 = Lift_ReadSrc(0x36, &e2);

                if (!g1 && !g2)  FAIL("ERR NOENC");
                snprintf(m, sizeof(m), "LIFTENC 31=%ld 36=%ld %s\r\n", g1 ? (long)e : -1L, g2 ? (long)e2 : -1L,
                         lift_cal_ok ? "READFAIL" : "NOCAL");
            }
            Say(m);
            return 1;
        }
        if (!ParseF(t[1], &f))  FAIL("ERR ARG");
        r = Lift_Goto(f);
        if (r)  FAIL(r);
        return 1;
    }
    if (strcmp(v, "LIFT") == 0 && n == 3 && strcmp(t[1], "CAL") == 0)
    {
        if (!ParseF(t[2], &f))  FAIL("ERR ARG");
        r = Lift_Cal(f);
        if (r)  FAIL(r);
        return 1;
    }

    /* ---- 舵机内部设置：SVP? <id> 读；SVW <id> <名字> <数值> 改(名字见 SVP? 的输出) ---- */
    if (strcmp(v, "SVP?") == 0 && n == 2)
    {
        if (!ParseL(t[1], &id) || (id != 1 && id != 2))  FAIL("ERR ARG");
        Servo_Params((uint8_t)id);
        return 1;
    }
    if (strcmp(v, "SVW") == 0 && n == 4)
    {
        long val;
        int w;

        if (!ParseL(t[1], &id) || (id != 1 && id != 2) || !ParseL(t[3], &val))  FAIL("ERR ARG");
        w = Servo_WriteParam((uint8_t)id, t[2], val);
        if (w == 0)  FAIL("ERR NAME");
        if (w == 2)  FAIL("ERR RANGE");
        if (w < 0)   FAIL("ERR WRITE");
        Servo_Params((uint8_t)id);                 /* 改完读回来给人看 */
        return 1;
    }

    /* ---- 舵机状态：SV? <id>(电压/电流/功率/温度/堵转标志，查发热、没劲用) ---- */
    if (strcmp(v, "SV?") == 0 && n == 2)
    {
        if (!ParseL(t[1], &id) || (id != 1 && id != 2))  FAIL("ERR ARG");
        Servo_Report((uint8_t)id);
        return 1;
    }

    /* ---- ID1 / ID2 精确控制：A? <id> / AF <id> <度> / AD <id> <增量度> / AP <ID1度> <ID2度> ---- */
    if (strcmp(v, "A?") == 0 && n == 2)
    {
        float a;

        if (!ParseL(t[1], &id) || (id != 1 && id != 2))  FAIL("ERR ARG");
        if (!Servo_ReadAngle((uint8_t)id, &a))  FAIL("ERR READ");
        SayAngle((uint8_t)id);
        return 1;
    }
    if (strcmp(v, "AF") == 0 && n == 3)
    {
        if (!ParseL(t[1], &id) || (id != 1 && id != 2))  FAIL("ERR ARG");
        if (!ParseF(t[2], &f))  FAIL("ERR ARG");
        {
            float a0;                              /* (10-21 用户：arm_calib 里伸缩的速度要和比赛时一样)大动作按 ROTSPD/EXTSPD(ASPD)，小调整照旧慢 */
            float spd = (Servo_ReadAngle((uint8_t)id, &a0) && Absf(f - a0) > 8.0f) ? Big_Spd((uint8_t)id) : g_aspdf;
            Servo_Move((uint8_t)id, f, spd, g_atol, 400);
        }
        SayAngle((uint8_t)id);
        return 1;
    }
    if (strcmp(v, "AFS") == 0 && n == 4)          /* AFS <id> <度> <度/秒>：按这个速度转到(调伸缩速度时演示) */
    {
        if (!ParseL(t[1], &id) || (id != 1 && id != 2))  FAIL("ERR ARG");
        if (!ParseF(t[2], &f) || !ParseF(t[3], &g) || g < 1.0f || g > 1500.0f)  FAIL("ERR ARG");
        Servo_Move((uint8_t)id, f, g, g_atol, 400);
        SayAngle((uint8_t)id);
        return 1;
    }
    if (strcmp(v, "AD") == 0 && n == 3)
    {
        float a0;

        if (!ParseL(t[1], &id) || (id != 1 && id != 2))  FAIL("ERR ARG");
        if (!ParseF(t[2], &f))  FAIL("ERR ARG");
        if (Absf(f) > g_amaxd)  FAIL("ERR RANGE");
        if (!Servo_ReadAngle((uint8_t)id, &a0))  FAIL("ERR READ");
        Servo_Move((uint8_t)id, a0 + f, (Absf(f) > 8.0f) ? Big_Spd((uint8_t)id) : g_aspdf, g_atol, 400);
        SayAngle((uint8_t)id);
        return 1;
    }
    if (strcmp(v, "AP") == 0 && n == 3)
    {
        float c1, c2;
        float spd = g_aspdf * 2.0f;                /* 小调整用微调速度的 2 倍 */

        if (!ParseF(t[1], &f) || !ParseF(t[2], &g))  FAIL("ERR ARG");
        if (Servo_ReadAngle(1, &c1) && Servo_ReadAngle(2, &c2) && (Absf(f - c1) > 10.0f || Absf(g - c2) > 10.0f))
        {
            if (g_aspd > spd)  spd = 0.0f;         /* 角度变化大(比如从转盘回到记下的姿态)：用大动作速度(各按 Big_Spd)，省时间 */
        }
        Servos_To(f, g, 2, g_atol, spd);
        SayAngle(1);
        SayAngle(2);
        return 1;
    }

    /* ---- APN a1 a2 [ms]：(10-16 用户)跟随原料用。先回报现在的两个角度，再让 ID1、ID2 往 (a1, a2) 转，
     *      不等转完马上回 DONE(树莓派每帧发一次新目标，手臂一直平滑地跟着动，不再"转一下停一下")。
     *      ms = 大约多少毫秒转到(按现在离目标多远算两个舵机的转速，限在 1~900 度/秒)；不给就用 ASPD ---- */
    if (strcmp(v, "APN") == 0 && (n == 3 || n == 4))
    {
        float ms = 0.0f;
        float tg[3];
        uint8_t id;

        if (!ParseF(t[1], &f) || !ParseF(t[2], &g))  FAIL("ERR ARG");
        if (n == 4 && (!ParseF(t[3], &ms) || ms < 1.0f))  FAIL("ERR ARG");
        tg[1] = f;
        tg[2] = g;
        for (id = 1; id <= 2; id++)
        {
            float a;
            float spd = Big_Spd(id);
            char m[40];
            char bb[20];

            if (Servo_ReadAngle(id, &a))
            {
                FmtF(bb, (int)sizeof(bb), a);
                snprintf(m, sizeof(m), "ANG %d %s\r\n", (int)id, bb);
                Say(m);
                if (ms > 0.0f)
                {
                    spd = Absf(tg[id] - a) * 1000.0f / ms;
                    if (spd < 1.0f)    spd = 1.0f;
                    if (spd > 900.0f)  spd = 900.0f;
                }
            }
            Servo_Track(id, tg[id], spd);
        }
        return 1;
    }

    /* ---- APZ a1 a2 z：转到 (a1, a2) 并停在高度 z；OVLAP=1 时边升边转、边转边降(10-14) ---- */
    if (strcmp(v, "APZ") == 0 && n == 4)
    {
        float z;

        if (!ParseF(t[1], &f) || !ParseF(t[2], &g) || !ParseF(t[3], &z))  FAIL("ERR ARG");
        r = Seq_Ready();
        if (r)  FAIL(r);
        r = Leave_Tray(f, g);                      /* 从车上转盘那边转走：ID1 一直转，路上按 A1T2、A1T1 伸缩绕开电池盒 */
        if (r)  FAIL(r);
        r = Move_Over(f, g, 2, z);
        if (r)  FAIL(r);
        SayAngle(1);
        SayAngle(2);
        return 1;
    }

    /* ---- 观察姿态：OBS RAW|RING [O]  (O = 同时张开夹爪) ---- */
    if (strcmp(v, "OBS") == 0 && (n == 2 || n == 3))
    {
        uint8_t ring;
        uint8_t open_claw = 0;

        if (strcmp(t[1], "RAW") == 0)        ring = 0;
        else if (strcmp(t[1], "RING") == 0)  ring = 1;
        else                                 FAIL("ERR ARG");
        if (n == 3)
        {
            if (strcmp(t[2], "O") != 0)  FAIL("ERR ARG");
            open_claw = 1;
        }
        r = Seq_Obs(ring, open_claw);
        if (r)  FAIL(r);
        return 1;
    }

    /* ---- (10-23)提前摆臂：PRE RAW|RING。车开往工位的最后一段直线之前发：手臂转向观察姿态，不等转完就回 DONE(见 Seq_Pre) ---- */
    if (strcmp(v, "PRE") == 0 && n == 2)
    {
        uint8_t ring;

        if (strcmp(t[1], "RAW") == 0)        ring = 0;
        else if (strcmp(t[1], "RING") == 0)  ring = 1;
        else                                 FAIL("ERR ARG");
        r = Seq_Pre(ring);
        if (r)  FAIL(r);
        return 1;
    }

    /* ---- 夹进转盘：GRAB n [H] (原料盘) / PICK n [H] (地上圆环)。带 H = 手臂已经对准，直接夹 ---- */
    if ((strcmp(v, "GRAB") == 0 || strcmp(v, "PICK") == 0) && (n == 2 || n == 3))
    {
        uint8_t ring = (v[1] == 'I') ? 1 : 0;
        uint8_t here = 0;

        if (!ParseL(t[1], &slot) || !Slot_Ok(slot))  FAIL("ERR ARG");
        if (n == 3)
        {
            if (strcmp(t[2], "H") != 0)  FAIL("ERR ARG");
            here = 1;
        }
        g_side = ring;                             /* (10-20)夹起来转去车上转盘：按这一边的绕电池盒参数走 */
        if (!here)
        {
            r = Seq_Obs(ring, 1);
            if (r)  FAIL(r);
            if (car_abort)  return 1;
        }
        r = Seq_PickHere((uint8_t)slot, ring ? g_zplc : g_zgrab, ring);
        if (r)  FAIL(r);
        return 1;
    }

    /* ---- (10-20)SIDE RAW|RING：手臂接下来在哪边干活(去/离开车上转盘按这一边的绕电池盒参数走)。OBS/GRAB/PICK/PLACE 也会设 ---- */
    if (strcmp(v, "SIDE") == 0 && n == 2)
    {
        if (strcmp(t[1], "RAW") == 0)        g_side = 0;
        else if (strcmp(t[1], "RING") == 0)  g_side = 1;
        else                                 FAIL("ERR ARG");
        return 1;
    }

    /* ---- 从转盘取出：TAKE n ---- */
    if (strcmp(v, "TAKE") == 0 && n == 2)
    {
        if (!ParseL(t[1], &slot) || !Slot_Ok(slot))  FAIL("ERR ARG");
        r = Seq_Take((uint8_t)slot);
        if (r)  FAIL(r);
        return 1;
    }

    /* ---- 放到地上：DROP [S]  (S = 码垛) ---- */
    if (strcmp(v, "DROP") == 0 && (n == 1 || n == 2))
    {
        uint8_t stack = 0;

        if (n == 2)
        {
            if (strcmp(t[1], "S") != 0)  FAIL("ERR ARG");
            stack = 1;
        }
        r = Seq_Drop(stack);
        if (r)  FAIL(r);
        return 1;
    }

    /* ---- 不用摄像头的完整放置：PLACE n [S] = TAKE n + OBS RING + DROP [S] ---- */
    if (strcmp(v, "PLACE") == 0 && (n == 2 || n == 3))
    {
        uint8_t stack = 0;

        if (!ParseL(t[1], &slot) || !Slot_Ok(slot))  FAIL("ERR ARG");
        if (n == 3)
        {
            if (strcmp(t[2], "S") != 0)  FAIL("ERR ARG");
            stack = 1;
        }
        g_side = 1;                                /* (10-20)在圆环那边干活 */
        r = Seq_Take((uint8_t)slot);
        if (r)  FAIL(r);
        if (car_abort)  return 1;
        r = Seq_Obs(1, 0);
        if (r)  FAIL(r);
        if (car_abort)  return 1;
        r = Seq_Drop(stack);
        if (r)  FAIL(r);
        return 1;
    }

    /* ---- (10-21)TRAY：转到车上转盘上方(最高处，不下降)，按去车上转盘的路走(绕电池盒：分段 / 一边转一边伸缩)。调绕电池盒那几个参数时试走用 ---- */
    if (strcmp(v, "TRAY") == 0 && n == 1)
    {
        r = Seq_Ready();
        if (r)  FAIL(r);
        r = Go_Tray(g_zhi);
        if (r)  FAIL(r);
        SayAngle(1);
        SayAngle(2);
        return 1;
    }

    /* ---- 收臂待命 ---- */
    if (strcmp(v, "STOW") == 0 && n == 1)
    {
        r = Seq_Stow();
        if (r)  FAIL(r);
        return 1;
    }

    /* ---- 停车待机：存了待机姿态(STBY SAVE)就回到它；没存：收臂 + 升降停到 60mm(ARMOK=0 时只降升降) ---- */
    if (strcmp(v, "PARK") == 0 && n == 1)
    {
        r = Seq_Park();
        if (r)  FAIL(r);
        return 1;
    }

    /* ---- 待机姿态：STBY = 回到待机姿态；STBY SAVE = 把现在的高度/手臂角度/夹爪记成待机姿态(存 Flash，上电自动回到这里)；
     *      STBY? = 看存的是什么；STBY CLR = 不用了(开机照旧停 60mm、PARK 照旧) ---- */
    if (strcmp(v, "STBY") == 0 && n == 1)
    {
        r = Seq_Standby();
        if (r)  FAIL(r);
        return 1;
    }
    if (strcmp(v, "STBY?") == 0 && n == 1)
    {
        char m[96];
        char b1[20];
        char b2[20];
        char b3[20];

        if (!sb_ok)
        {
            Say("STBY NONE\r\n");
            return 1;
        }
        FmtF(b1, (int)sizeof(b1), sb_z);
        FmtF(b2, (int)sizeof(b2), sb_a1);
        FmtF(b3, (int)sizeof(b3), sb_a2);
        snprintf(m, sizeof(m), "STBY Z=%s A1=%s A2=%s CLAW=%lu\r\n", b1, b2, b3, (unsigned long)sb_claw);
        Say(m);
        return 1;
    }
    if (strcmp(v, "STBY") == 0 && n == 2 && strcmp(t[1], "SAVE") == 0)
    {
        float a1, a2;

        if (!lift_known)                              FAIL("ERR NOZERO");
        if (!lift_cal_ok)                             FAIL("ERR NOCAL");      /* 升降没标定：开机认不准高度，存了也不敢动 */
        {
            int k, g1 = 0, g2 = 0;
            for (k = 0; k < 3 && !(g1 && g2); k++)    /* 总线偶尔读不到：各多试两次 */
            {
                if (!g1)  g1 = Servo_ReadAngle(1, &a1);
                if (!g2)  g2 = Servo_ReadAngle(2, &a2);
            }
            if (!g1)  FAIL("ERR READ1");
            if (!g2)  FAIL("ERR READ2");
        }
        sb_z = lift_mm;
        sb_a1 = a1;
        sb_a2 = a2;
        sb_claw = claw_now;
        sb_ok = 1;
        if (!Cal_Save())                              FAIL("ERR FLASH");
        Pos_Save(lift_mm);                         /* 擦扇区把位置记录也擦了：马上记一条，下次开机从这里找 */
        if (!sb_ok)                                   FAIL("ERR FLASH");
        return Arm_Command("STBY?", err, errlen);
    }
    if (strcmp(v, "STBY") == 0 && n == 2 && strcmp(t[1], "CLR") == 0)
    {
        sb_ok = 0;
        if (lift_cal_ok && !Cal_Save())               FAIL("ERR FLASH");
        if (lift_cal_ok)  Pos_Save(lift_mm);
        return 1;
    }

    /* ---- 二维码 ---- */
    if (strcmp(v, "QR?") == 0 && n == 1)
    {
        char m[32];

        QR_Fetch();                                /* 中断里刚锁存、主循环还没取走的码也算 */
        if (qr_show)
        {
            qr_said = 1;                           /* 这里已经回了：Arm_Poll 写屏时不再主动发一遍 */
        }
        if (qr_valid)  snprintf(m, sizeof(m), "QR %s\r\n", qr_code);
        else           snprintf(m, sizeof(m), "QR NONE\r\n");
        Say(m);
        return 1;
    }
    if (strcmp(v, "QR") == 0 && n == 2 && strcmp(t[1], "RAW") == 0)
    {
        /* 诊断：QRRAW n=收到的字节数 err=串口出错次数 last=最后一条(看不见的字符写成 <十六进制>) */
        char m[120];
        char l[QR_BUF];
        int k, ln, pos;

        __disable_irq();
        ln = qr_last_len;
        memcpy(l, qr_last, (size_t)ln);
        __enable_irq();
        pos = snprintf(m, sizeof(m), "QRRAW n=%lu err=%u last=", (unsigned long)qr_nbytes, (unsigned)qr_nerr);
        for (k = 0; k < ln && pos < (int)sizeof(m) - 8; k++)
        {
            unsigned char ch = (unsigned char)l[k];
            if (ch >= 0x20 && ch < 0x7F) m[pos++] = (char)ch;
            else                          pos += snprintf(m + pos, sizeof(m) - (size_t)pos, "<%02X>", ch);
        }
        m[pos++] = '\r'; m[pos++] = '\n'; m[pos] = 0;
        Say(m);
        return 1;
    }
    if (strcmp(v, "QR") == 0 && n == 2 && strcmp(t[1], "CLR") == 0)
    {
        __disable_irq();
        qr_new = 0;                                /* 中断里锁存了还没取走的、收了一半的，一起清掉 */
        qr_len = 0;
        __enable_irq();
        qr_valid = 0;
        qr_code[0] = 0;
        qr_show = 0;
        QRC_Start(QRC_ON, 3);                      /* 开跑前清码：扫码模块重新打开(上一轮扫到以后关掉了) */
        return 1;
    }
    /* QR ON / QR OFF：手动开/关扫码模块；QR AUTO 0|1：扫到码以后要不要自动关；QR SCAN?：现在开着还是关了 */
    if (strcmp(v, "QR") == 0 && n == 2 && (strcmp(t[1], "ON") == 0 || strcmp(t[1], "OFF") == 0))
    {
        QRC_Start(strcmp(t[1], "ON") == 0 ? QRC_ON : QRC_OFF, 3);
        return 1;
    }
    if (strcmp(v, "QR") == 0 && n == 3 && strcmp(t[1], "AUTO") == 0 && (strcmp(t[2], "0") == 0 || strcmp(t[2], "1") == 0))
    {
        qr_auto_off = (uint8_t)(t[2][0] - '0');
        return 1;
    }
    if (strcmp(v, "QR") == 0 && n == 2 && strcmp(t[1], "SCAN?") == 0)
    {
        char m[48];

        snprintf(m, sizeof(m), "QRSCAN %s auto=%u%s%s\r\n",
                 qr_scan == 1 ? "ON" : (qr_scan == 0 ? "OFF" : "UNKNOWN"), (unsigned)qr_auto_off,
                 qrc_state != QRC_IDLE ? " busy" : "", qrc_fail ? " noreply" : "");
        Say(m);
        return 1;
    }

    /* ---- 选启停区(屏上触摸)：ZONE? / ZONE ASK / ZONE 1|2 / ZONE MSG <文字> / ZONE LOCK ---- */
    if (strcmp(v, "ZONE?") == 0 && n == 1)
    {
        char m[24];

        snprintf(m, sizeof(m), "ZONE %d %d\r\n", (int)zone_sel, (int)zone_go);   /* 选了几区(0=还没选)、按没按 START */
        Say(m);
        return 1;
    }
    if (strcmp(v, "ZONE") == 0)
    {
        if (n >= 2 && strcmp(t[1], "MSG") == 0)
        {
            const char *p = strstr(cmd, "MSG") + 3;
            int k = 0;

            while (*p == ' ')
            {
                p++;
            }
            while (*p != 0 && k < (int)sizeof(zone_msg) - 1)
            {
                if (*p >= 0x20 && *p <= 0x7E && *p != '"')   /* 字库只有 ASCII；双引号会弄坏屏的指令，去掉 */
                {
                    zone_msg[k++] = *p;
                }
                p++;
            }
            zone_msg[k] = 0;
            Zone_ShowMsg();
            return 1;
        }
        if (n != 2)                                FAIL("ERR ARG");
        if (strcmp(t[1], "ASK") == 0)              Zone_Page(ZUI_PICK, 0, 1);
        else if (strcmp(t[1], "1") == 0)           Zone_Page(ZUI_READY, 1, 1);
        else if (strcmp(t[1], "2") == 0)           Zone_Page(ZUI_READY, 2, 1);
        else if (strcmp(t[1], "LOCK") == 0)        Zone_Lock();
        else                                       FAIL("ERR ARG");
        return 1;
    }

    /* ---- 串口屏原始指令：SCMD <淘晶驰指令> ---- */
    if (strcmp(v, "SCMD") == 0 && strlen(cmd) > 5)
    {
        Screen_Raw(cmd + 5);
        return 1;
    }

    /* ---- 串口屏：SCR <控件名> <文字> ---- */
    if (strcmp(v, "SCR") == 0 && strlen(cmd) > 4)
    {
        char obj[12];
        char txt[40];
        const char *p = cmd + 4;
        int k = 0;

        while (*p != 0 && *p != ' ' && k < (int)sizeof(obj) - 1)
        {
            obj[k++] = *p++;
        }
        obj[k] = 0;
        if (k == 0 || *p != ' ')  FAIL("ERR ARG");
        p++;
        k = 0;
        while (*p != 0 && k < (int)sizeof(txt) - 1)
        {
            if (*p != '"')                         /* 双引号会弄坏屏的指令，直接去掉 */
            {
                txt[k++] = *p;
            }
            p++;
        }
        txt[k] = 0;
        if (zone_ui != ZUI_OFF)
        {
            Zone_Lock();                           /* 选区页还在显示(没发 ZONE LOCK 的旧树莓派程序)：先回到比赛布局再写 */
        }
        Screen_Text(obj, txt);
        return 1;
    }

    return 0;
}
