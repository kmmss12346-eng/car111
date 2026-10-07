/* arm.c  机械臂总控 v3
 *
 * 硬件对应：
 *   升降      Emm 步进电机 ID5(和轮子 1~4 号同一路 UART4，皮带升降)
 *   前后伸缩  飞特总线舵机 ID2      (main.c 里的 Servo_Move / Servo_Start，限位在那里)
 *   整臂旋转  飞特总线舵机 ID1      (同上)
 *   夹爪      PA2  TIM2 通道 3      (main.c 里的 Claw_Open / Claw_Close)
 *   转盘      PB3  TIM2 通道 2      (main.c 里的 Turntable_GoTo)
 *   扫码模块  UART5 (PC12/PD2)   GM65，出厂 9600 8N1
 *   串口屏    USART2(PD5/PD6)    淘晶驰 TJC4832T135，出厂 9600
 *
 * 高度约定：升降"最高点 = 0 mm"，往下为正数。所有姿态都是参数，用 SET 在线改，不用重新烧录。
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

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart5;
extern TIM_HandleTypeDef  htim2;
extern volatile uint8_t   car_abort;

/* main.c 里已有的函数 */
extern void Claw_Set(uint32_t pulse_us);
extern void Claw_Open(void);
extern void Claw_Close(void);
extern void Turntable_Set(uint32_t pulse_us);
extern void Turntable_GoTo(uint8_t slot);
extern uint32_t Servo_Start(uint8_t id, float *angle_deg, float speed_dps);
extern void Servo_Move(uint8_t id, float angle_deg, float speed_dps, float tol_deg, uint32_t extra_ms);
extern int  Servo_ReadAngle(uint8_t id, float *angle);
extern void Delay_Report(uint32_t ms);

#define LIFT_ADDR   5          /* 升降步进电机的 Emm 地址 */

/* 两个模块出厂都是 9600。工程里 USART2/UART5 初始化成了 115200，Arm_Init 里会改成下面的值。
 * 如果您把屏或扫码模块改过波特率，只改这两个数 */
#define SCREEN_BAUD    9600u
#define QR_BAUD        9600u

/* 开机时夹爪、转盘先到这个位置(要和 main.c 里 CLAW_OPEN_US / TURNTABLE_SLOT1_US 一致) */
#define CLAW_BOOT_US   2090u
#define TT_BOOT_US     2608u

#define SERVO_EXTRA_MS 800     /* ID1/ID2 一起转时，按速度算好的时间之外最多再等多久 */

/* ================= 可调参数(SET 名字 数值) ================= */
static float g_lppm   = 80.0f;     /* LFPPM  升降：1 毫米要多少脉冲。= 每圈脉冲数(16 细分是 3200) / 皮带每圈走的毫米数 */
static float g_lrpm   = 150.0f;    /* LFRPM  升降速度(转/分) */
static float g_lacc   = 200.0f;    /* LFACC  升降加速度档位 0~255(0=不加速直接到速度) */
static float g_lmax   = 150.0f;    /* LFMAX  升降最大行程(毫米)，从零点往下算 */
static float g_ldir   = 0.0f;      /* LFDIR  "往下"对应的 Emm 方向：0 或 1。反了就改这个 */
static float g_lspr   = 3200.0f;   /* LFSPR  电机每圈脉冲数，只用来算要等多久 */
static float g_lmrg   = 200.0f;    /* LFMRG  升降走完后多等多少毫秒(加减速余量) */
static float g_lhmd   = 2.0f;      /* LFHMD  LIFT HOME 用的 Emm 回零模式(0~3，见张大头手册) */
static float g_lhtm   = 8000.0f;   /* LFHTM  LIFT HOME 最多等多少毫秒 */

static float g_zhi    = 0.0f;      /* ZHI    搬运途中的高度(最高) */
static float g_zgrab  = 100.0f;    /* ZGRAB  在原料盘上夹物料时的高度 */
static float g_zdrop  = 60.0f;     /* ZDROP  放进车上转盘 / 从转盘取物料时的高度 */
static float g_zplc   = 100.0f;    /* ZPLC   放到地上圆环时的高度 */
static float g_zstk   = 40.0f;     /* ZSTK   码垛时放下的高度(比 ZPLC 抬高一个物料的高度，物料高 60mm) */
static float g_zobraw = 0.0f;      /* ZOBRAW 观察原料盘时的高度(摄像头标定比例时用的高度) */
static float g_zobrng = 0.0f;      /* ZOBRNG 观察地上圆环时的高度 */

static float g_a1g    = 495.0f;    /* A1G    ID1 角度：对准原料盘 */
static float g_a1d    = 495.0f;    /* A1D    ID1 角度：对准车上转盘 */
static float g_a1h    = 495.0f;    /* A1H    ID1 角度：收起/待命 */
static float g_a1p    = 495.0f;    /* A1P    ID1 角度：对准地上圆环 */
static float g_a2e    = -862.0f;   /* A2E    ID2 角度：伸出夹原料盘上的物料 */
static float g_a2r    = -862.0f;   /* A2R    ID2 角度：缩回(转盘上方) */
static float g_a2p    = -862.0f;   /* A2P    ID2 角度：伸出到地上圆环 */
static float g_aspd   = 90.0f;     /* ASPD   ID1/ID2 大动作转速(度/秒) */
static float g_aspdf  = 40.0f;     /* ASPDF  ID1/ID2 微调转速(度/秒) */
static float g_atolc  = 1.0f;      /* ATOLC  大动作离目标多少度以内算到位 */
static float g_atol   = 0.3f;      /* ATOL   微调离目标多少度以内算到位 */
static float g_amaxd  = 60.0f;     /* AMAXD  AD 指令一次最多转多少度(防止发错数) */
static float g_para   = 1.0f;      /* PARA   1 = ID1、ID2 一起转(快)；0 = 一个一个转 */

static float g_clwait = 400.0f;    /* CLWAIT 夹爪动作后等多久(毫秒) */
static float g_ttwait = 800.0f;    /* TTWAIT 转盘转到新位置要等多久(毫秒) */
static float g_armok  = 0.0f;      /* ARMOK  1 = 姿态都标定好了，允许夹放流程 */

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
    { "LFMAX",  &g_lmax,    10.0f,   400.0f },
    { "LFDIR",  &g_ldir,    0.0f,    1.0f },
    { "LFSPR",  &g_lspr,    200.0f,  51200.0f },
    { "LFMRG",  &g_lmrg,    0.0f,    2000.0f },
    { "LFHMD",  &g_lhmd,    0.0f,    3.0f },
    { "LFHTM",  &g_lhtm,    500.0f,  30000.0f },
    { "ZHI",    &g_zhi,     0.0f,    400.0f },
    { "ZGRAB",  &g_zgrab,   0.0f,    400.0f },
    { "ZDROP",  &g_zdrop,   0.0f,    400.0f },
    { "ZPLC",   &g_zplc,    0.0f,    400.0f },
    { "ZSTK",   &g_zstk,    0.0f,    400.0f },
    { "ZOBRAW", &g_zobraw,  0.0f,    400.0f },
    { "ZOBRNG", &g_zobrng,  0.0f,    400.0f },
    { "A1G",    &g_a1g,     390.0f,  600.0f },
    { "A1D",    &g_a1d,     390.0f,  600.0f },
    { "A1H",    &g_a1h,     390.0f,  600.0f },
    { "A1P",    &g_a1p,     390.0f,  600.0f },
    { "A2E",    &g_a2e,    -1220.0f, -503.5f },
    { "A2R",    &g_a2r,    -1220.0f, -503.5f },
    { "A2P",    &g_a2p,    -1220.0f, -503.5f },
    { "ASPD",   &g_aspd,    10.0f,   300.0f },
    { "ASPDF",  &g_aspdf,   5.0f,    150.0f },
    { "ATOLC",  &g_atolc,   0.2f,    5.0f },
    { "ATOL",   &g_atol,    0.05f,   2.0f },
    { "AMAXD",  &g_amaxd,   1.0f,    300.0f },
    { "PARA",   &g_para,    0.0f,    1.0f },
    { "CLWAIT", &g_clwait,  0.0f,    3000.0f },
    { "TTWAIT", &g_ttwait,  0.0f,    5000.0f },
    { "ARMOK",  &g_armok,   0.0f,    1.0f },
};
#define TUN_N  ((int)(sizeof(tun) / sizeof(tun[0])))

int Arm_Param_Set(const char *name, float v)
{
    int i;

    for (i = 0; i < TUN_N; i++)
    {
        if (strcmp(tun[i].name, name) == 0)
        {
            if (v < tun[i].lo || v > tun[i].hi)
            {
                return 2;
            }
            *tun[i].p = v;
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

static uint32_t Lift_Time_Ms(uint32_t pulses)
{
    float sps = g_lrpm / 60.0f * g_lspr;          /* 每秒多少脉冲 */
    return (uint32_t)((float)pulses / sps * 1000.0f) + (uint32_t)g_lmrg;
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
    Emm_V5_Pos_Control(LIFT_ADDR, dir, (uint16_t)g_lrpm, (uint8_t)g_lacc, pulses, 0, false);

    if (!Wait_Ms(Lift_Time_Ms(pulses)))
    {
        Emm_V5_Stop_Now(LIFT_ADDR, false);
        lift_known = 0;                           /* 停在半路，不知道在哪了，要重新回零 */
        return NULL;
    }
    lift_mm = mm;
    return NULL;
}

/* ================= ID1 / ID2 ================= */
/* ID1、ID2 转到 a1、a2(度)。PARA=1 时两个一起转(省时间)；否则按 order 先后：
 *   order=1：先动 ID2 再动 ID1(缩回时先缩回再转，免得伸着转扫到东西)
 *   order=2：先动 ID1 再动 ID2(伸出时先转好再伸)
 * tol：到位误差(度)，speed：转速(度/秒) */
static void Servos_To(float a1, float a2, uint8_t order, float tol, float speed)
{
    if (g_para > 0.5f)
    {
        uint32_t t1 = Servo_Start(1, &a1, speed);
        uint32_t t2 = Servo_Start(2, &a2, speed);
        uint32_t tmax = ((t1 > t2) ? t1 : t2) + SERVO_EXTRA_MS;
        uint32_t t0 = HAL_GetTick();
        uint8_t ok1 = 0;
        uint8_t ok2 = 0;

        while ((HAL_GetTick() - t0) < tmax && !(ok1 && ok2) && !car_abort)
        {
            float a;

            HAL_Delay(30);
            if (!ok1 && Servo_ReadAngle(1, &a) && Absf(a - a1) <= tol)  ok1 = 1;
            if (!ok2 && Servo_ReadAngle(2, &a) && Absf(a - a2) <= tol)  ok2 = 1;
        }
    }
    else if (order == 1)
    {
        Servo_Move(2, a2, speed, tol, SERVO_EXTRA_MS);
        Servo_Move(1, a1, speed, tol, SERVO_EXTRA_MS);
    }
    else
    {
        Servo_Move(1, a1, speed, tol, SERVO_EXTRA_MS);
        Servo_Move(2, a2, speed, tol, SERVO_EXTRA_MS);
    }
}

/* ================= 夹爪 / 转盘 ================= */
static uint8_t  tt_slot     = 1;
static uint32_t tt_ready_at = 0;

static void TT_Go(uint8_t slot)
{
    if (slot != tt_slot)
    {
        tt_slot = slot;
        tt_ready_at = HAL_GetTick() + (uint32_t)g_ttwait;
    }
    Turntable_GoTo(slot);
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

/* 观察姿态：升到最高 -> ID1、ID2 转到工位上方 -> 升降到观察高度。ring=0 原料盘，ring=1 地上圆环。
 * open_claw=1 同时把夹爪张开(夹爪动的时候手臂也在动，不多花时间)。open_claw=0 时夹爪状态保持不变 */
static const char *Seq_Obs(uint8_t ring, uint8_t open_claw)
{
    const char *e = Seq_Ready();
    if (e) return e;

    if (open_claw)  Claw_Open();
    GO(Lift_Goto(g_zhi));
    SERVOS(ring ? g_a1p : g_a1g, ring ? g_a2p : g_a2e, 2, g_atolc, g_aspd);
    GO(Lift_Goto(ring ? g_zobrng : g_zobraw));
    return NULL;
}

/* 手臂已经(用摄像头)对准了物料：下降夹紧 -> 抬起 -> 缩回并转到转盘上方 -> 下降 -> 转盘到位后松开 -> 抬起。
 * z = 夹取高度。结束时手臂停在转盘上方的最高处 */
static const char *Seq_PickHere(uint8_t slot, float z)
{
    const char *e = Seq_Ready();
    if (e) return e;
    if (!Slot_Ok(slot))  return "ERR ARG";

    TT_Go(slot);                                   /* 转盘先转，转的同时手臂也在动，省时间 */
    Claw_Open();                                   /* 确保下降时爪子是张开的(下降要一会儿，爪子这时候正好张开) */
    GO(Lift_Goto(z));                              /* 下降 */
    Claw_Close();  WAIT(g_clwait);                 /* 夹紧 */
    GO(Lift_Goto(g_zhi));                          /* 抬起 */
    SERVOS(g_a1d, g_a2r, 1, g_atolc, g_aspd);      /* 缩回，转到转盘上方 */
    GO(Lift_Goto(g_zdrop));                        /* 下降 */
    if (!TT_WaitReady()) return NULL;              /* 转盘到位了才松手 */
    Claw_Open();   WAIT(g_clwait);                 /* 松开：物料落进转盘 */
    GO(Lift_Goto(g_zhi));
    return NULL;
}

/* 从车上转盘 slot 号位取出物料：手臂转到转盘上方 -> 下降夹紧 -> 抬起。结束时夹着物料停在转盘上方的最高处 */
static const char *Seq_Take(uint8_t slot)
{
    const char *e = Seq_Ready();
    if (e) return e;
    if (!Slot_Ok(slot))  return "ERR ARG";

    TT_Go(slot);
    Claw_Open();
    GO(Lift_Goto(g_zhi));
    SERVOS(g_a1d, g_a2r, 1, g_atolc, g_aspd);
    if (!TT_WaitReady()) return NULL;
    GO(Lift_Goto(g_zdrop));
    Claw_Close();  WAIT(g_clwait);                 /* 夹起 */
    GO(Lift_Goto(g_zhi));
    return NULL;
}

/* 手臂夹着物料，已经(用摄像头)对准了目标：下降 -> 松开 -> 抬起 -> ID2 缩回。
 * stack=1 是码垛(放在已有物料上，下降高度用 ZSTK)。
 * 最后缩回 ID2(不转 ID1)：之后底盘要沿着圆环板挪动，爪子不能还伸在已经放好的物料上方 */
static const char *Seq_Drop(uint8_t stack)
{
    const char *e = Seq_Ready();
    if (e) return e;

    GO(Lift_Goto(stack ? g_zstk : g_zplc));
    Claw_Open();   WAIT(g_clwait);
    GO(Lift_Goto(g_zhi));
    Servo_Move(2, g_a2r, g_aspd, g_atolc, SERVO_EXTRA_MS);
    if (car_abort) return NULL;
    return NULL;
}

/* 收臂待命：升到最高 -> 缩回、ID1 转到待命角度 */
static const char *Seq_Stow(void)
{
    const char *e = Seq_Ready();
    if (e) return e;

    GO(Lift_Goto(g_zhi));
    SERVOS(g_a1h, g_a2r, 1, g_atolc, g_aspd);
    return NULL;
}

/* ================= 串口屏 ================= */
/* 默认按淘晶驰(TJC)串口屏的格式发：控件名.txt="文字" 加三个 0xFF。
 * 如果屏的品牌不同，只需要改这个函数。 */
void Screen_Text(const char *obj, const char *text)
{
    static const uint8_t endb[3] = { 0xFF, 0xFF, 0xFF };
    char buf[64];
    int n = snprintf(buf, sizeof(buf), "%s.txt=\"%s\"", obj, text);

    if (n < 0)
    {
        return;
    }
    if (n >= (int)sizeof(buf))
    {
        n = (int)sizeof(buf) - 1;
    }
    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 100);
    HAL_UART_Transmit(&huart2, (uint8_t *)endb, 3, 100);
}

/* 直接发一条原始的淘晶驰指令，例如 "page 1"、"t0.pco=63488" */
static void Screen_Raw(const char *cmd)
{
    static const uint8_t endb[3] = { 0xFF, 0xFF, 0xFF };

    HAL_UART_Transmit(&huart2, (uint8_t *)cmd, (uint16_t)strlen(cmd), 100);
    HAL_UART_Transmit(&huart2, (uint8_t *)endb, 3, 100);
}

/* ================= 二维码 (UART5) ================= */
#define QR_BUF   64
static uint8_t           qr_rx;
static char              qr_buf[QR_BUF];
static volatile uint8_t  qr_len = 0;
static volatile uint8_t  qr_end = 0;
static volatile uint32_t qr_tick = 0;
static char              qr_code[16];
static uint8_t           qr_valid = 0;

/* 在串口中断里：把收到的字节存起来。是否收完一条，交给主循环判断 */
void Arm_QR_RxCplt(void)
{
    if (qr_len < QR_BUF - 1)
    {
        qr_buf[qr_len++] = (char)qr_rx;
    }
    else
    {
        qr_len = 0;                                /* 太长，丢掉重来 */
    }
    qr_tick = HAL_GetTick();
    if (qr_rx == '\r' || qr_rx == '\n')
    {
        qr_end = 1;
    }
    HAL_UART_Receive_IT(&huart5, &qr_rx, 1);
}

void Arm_QR_RxRestart(void)
{
    qr_len = 0;
    qr_end = 0;
    HAL_UART_Receive_IT(&huart5, &qr_rx, 1);
}

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

void Arm_Poll(void)
{
    char tmp[QR_BUF];
    char newc[16];
    int n;

    /* 收完一条(收到换行)，或者 60 毫秒没有新字节(有的模块不发换行)，就拿去解析 */
    if (qr_len == 0)
    {
        return;
    }
    if (!qr_end && (HAL_GetTick() - qr_tick) <= 60u)
    {
        return;
    }

    __disable_irq();
    n = qr_len;
    memcpy(tmp, qr_buf, (size_t)n);
    qr_len = 0;
    qr_end = 0;
    __enable_irq();

    if (QR_Match(tmp, n, newc))
    {
        /* 扫码模块在连续模式下会反复发同一个码：同一个码只处理一次，免得一直重写串口屏、占用主循环 */
        if (!qr_valid || strcmp(newc, qr_code) != 0)
        {
            char m[32];
            char h1[8];
            char h2[8];

            memcpy(qr_code, newc, sizeof(qr_code));
            qr_valid = 1;
            /* 屏上显示任务码。赛规要求字高不小于 12mm，3.5 寸屏一行放不下 15 个字符，所以分两行：
             * t0 = 前两组(如 452+321)，t7 = 后两组(如 254+312)。控件名不一样就改这里 */
            memcpy(h1, qr_code, 7);      h1[7] = 0;
            memcpy(h2, qr_code + 8, 7);  h2[7] = 0;
            Screen_Text("t0", h1);
            Screen_Text("t7", h2);
            snprintf(m, sizeof(m), "QR %s\r\n", qr_code);
            Say(m);
        }
    }
}

/* ================= 初始化 ================= */
void Arm_Init(void)
{
    /* 工程里原来没有启动 TIM2 的 PWM，夹爪和转盘一直没有输出。先设好初始脉宽再启动，避免开机时舵机猛地跳一下 */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, CLAW_BOOT_US);
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
    qr_end = 0;
    HAL_UART_Receive_IT(&huart5, &qr_rx, 1);       /* UART5 的中断在 hal_msp.c 里已经打开 */
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

    /* ---- 夹爪：CLAW O / CLAW C / CLAW <微秒> ---- */
    if (strcmp(v, "CLAW") == 0 && n == 2)
    {
        if (strcmp(t[1], "O") == 0)       Claw_Open();
        else if (strcmp(t[1], "C") == 0)  Claw_Close();
        else
        {
            long us;
            if (!ParseL(t[1], &us) || us < 500 || us > 3000)  FAIL("ERR ARG");   /* 夹爪合上是 2910 */
            Claw_Set((uint32_t)us);                /* main.c 里会再按夹爪限位夹住 */
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
        char m[40];
        char b[20];

        FmtF(b, (int)sizeof(b), lift_mm);
        snprintf(m, sizeof(m), "LIFT %d %s\r\n", (int)lift_known, b);
        Say(m);
        return 1;
    }
    if (strcmp(v, "LIFT") == 0 && n == 2)
    {
        if (strcmp(t[1], "ZERO") == 0)
        {
            Emm_V5_En_Control(LIFT_ADDR, true, false);
            Emm_V5_Reset_CurPos_To_Zero(LIFT_ADDR);
            lift_mm = 0.0f;
            lift_known = 1;
            return 1;
        }
        if (strcmp(t[1], "HOME") == 0)
        {
            Emm_V5_En_Control(LIFT_ADDR, true, false);
            Emm_V5_Origin_Trigger_Return(LIFT_ADDR, (uint8_t)g_lhmd, false);
            if (!Wait_Ms((uint32_t)g_lhtm))
            {
                Emm_V5_Origin_Interrupt(LIFT_ADDR);
                return 1;                          /* 被急停，位置不明，不标记已回零 */
            }
            lift_mm = 0.0f;
            lift_known = 1;
            return 1;
        }
        if (!ParseF(t[1], &f))  FAIL("ERR ARG");
        r = Lift_Goto(f);
        if (r)  FAIL(r);
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
        Servo_Move((uint8_t)id, f, g_aspdf, g_atol, 400);
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
        Servo_Move((uint8_t)id, a0 + f, (Absf(f) > 8.0f) ? g_aspd : g_aspdf, g_atol, 400);
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
            if (g_aspd > spd)  spd = g_aspd;       /* 角度变化大(比如从转盘回到记下的姿态)：用大动作速度，省时间 */
        }
        Servos_To(f, g, 2, g_atol, spd);
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
        if (!here)
        {
            r = Seq_Obs(ring, 1);
            if (r)  FAIL(r);
            if (car_abort)  return 1;
        }
        r = Seq_PickHere((uint8_t)slot, ring ? g_zplc : g_zgrab);
        if (r)  FAIL(r);
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

    /* ---- 收臂待命 ---- */
    if (strcmp(v, "STOW") == 0 && n == 1)
    {
        r = Seq_Stow();
        if (r)  FAIL(r);
        return 1;
    }

    /* ---- 二维码 ---- */
    if (strcmp(v, "QR?") == 0 && n == 1)
    {
        char m[32];

        if (qr_valid)  snprintf(m, sizeof(m), "QR %s\r\n", qr_code);
        else           snprintf(m, sizeof(m), "QR NONE\r\n");
        Say(m);
        return 1;
    }
    if (strcmp(v, "QR") == 0 && n == 2 && strcmp(t[1], "CLR") == 0)
    {
        qr_valid = 0;
        qr_code[0] = 0;
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
        Screen_Text(obj, txt);
        return 1;
    }

    return 0;
}
