/* arm.c  机械臂总控 v1
 *
 * 硬件对应：
 *   升降      Emm 步进电机 ID5(和轮子 1~4 号同一路 UART4，皮带升降)
 *   前后伸缩  飞特总线舵机 ID2      (main.c 里的 Servo_SetAngle，限位已在那里)
 *   整臂旋转  飞特总线舵机 ID1      (同上)
 *   夹爪      PA2  TIM2 通道 3      (main.c 里的 Claw_Open / Claw_Close)
 *   转盘      PB3  TIM2 通道 2      (main.c 里的 Turntable_GoTo)
 *   扫码模块  UART5 (PC12/PD2)
 *   串口屏    USART2(PD5/PD6)
 *
 * 高度约定：升降"最高点 = 0 mm"，往下为正数。所有姿态都是参数，用 SET 在线改，不用重新烧录。
 *
 * 安全措施：
 *   - 没回零(LIFT ZERO / LIFT HOME)之前，不允许任何升降动作和 GRAB / PLACE
 *   - 参数 ARMOK=0(默认)时不允许 GRAB / PLACE：姿态都标定好后 SET ARMOK 1
 *   - 升降目标会被限制在 0 ~ LFMAX 毫米
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
extern void Servo_SetAngle(uint8_t id, float angle_deg, float speed_dps);
extern void Delay_Report(uint32_t ms);

#define LIFT_ADDR   5          /* 升降步进电机的 Emm 地址 */

/* 开机时夹爪、转盘先到这个位置(要和 main.c 里 CLAW_OPEN_US / TURNTABLE_SLOT1_US 一致) */
#define CLAW_BOOT_US   2090u
#define TT_BOOT_US     2608u

/* ================= 可调参数(SET 名字 数值) ================= */
static float g_lppm   = 80.0f;     /* LFPPM  升降：1 毫米要多少脉冲。= 每圈脉冲数(16 细分是 3200) / 皮带每圈走的毫米数 */
static float g_lrpm   = 150.0f;    /* LFRPM  升降速度(转/分) */
static float g_lacc   = 200.0f;    /* LFACC  升降加速度档位 0~255(0=不加速直接到速度) */
static float g_lmax   = 150.0f;    /* LFMAX  升降最大行程(毫米)，从零点往下算 */
static float g_ldir   = 0.0f;      /* LFDIR  "往下"对应的 Emm 方向：0 或 1。反了就改这个 */
static float g_lspr   = 3200.0f;   /* LFSPR  电机每圈脉冲数，只用来算要等多久 */
static float g_lmrg   = 300.0f;    /* LFMRG  升降走完后多等多少毫秒(加减速余量) */
static float g_lhmd   = 2.0f;      /* LFHMD  LIFT HOME 用的 Emm 回零模式(0~3，见张大头手册) */
static float g_lhtm   = 8000.0f;   /* LFHTM  LIFT HOME 最多等多少毫秒 */

static float g_zhi    = 0.0f;      /* ZHI    搬运途中的高度(最高) */
static float g_zgrab  = 100.0f;    /* ZGRAB  在原料盘上夹物料时的高度 */
static float g_zdrop  = 60.0f;     /* ZDROP  放进车上转盘 / 从转盘取物料时的高度 */
static float g_zplc   = 100.0f;    /* ZPLC   放到地上圆环时的高度 */

static float g_a1g    = 495.0f;    /* A1G    ID1 角度：对准原料盘 */
static float g_a1d    = 495.0f;    /* A1D    ID1 角度：对准车上转盘 */
static float g_a1h    = 495.0f;    /* A1H    ID1 角度：收起/待命 */
static float g_a1p    = 495.0f;    /* A1P    ID1 角度：对准地上圆环 */
static float g_a2e    = -862.0f;   /* A2E    ID2 角度：伸出夹物料 */
static float g_a2r    = -862.0f;   /* A2R    ID2 角度：缩回(转盘上方) */
static float g_a2p    = -862.0f;   /* A2P    ID2 角度：伸出放到圆环 */
static float g_aspd   = 90.0f;     /* ASPD   ID1/ID2 转速(度/秒) */

static float g_clwait = 400.0f;    /* CLWAIT 夹爪动作后等多久(毫秒) */
static float g_ttwait = 800.0f;    /* TTWAIT 转盘转到新位置要等多久(毫秒) */
static float g_armok  = 0.0f;      /* ARMOK  1 = 姿态都标定好了，允许 GRAB / PLACE */

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
    { "A1G",    &g_a1g,     390.0f,  600.0f },
    { "A1D",    &g_a1d,     390.0f,  600.0f },
    { "A1H",    &g_a1h,     390.0f,  600.0f },
    { "A1P",    &g_a1p,     390.0f,  600.0f },
    { "A2E",    &g_a2e,    -1220.0f, -503.5f },
    { "A2R",    &g_a2r,    -1220.0f, -503.5f },
    { "A2P",    &g_a2p,    -1220.0f, -503.5f },
    { "ASPD",   &g_aspd,    10.0f,   300.0f },
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
#define SERVO(id, a)    do { Servo_SetAngle((id), (a), g_aspd); if (car_abort) return NULL; } while (0)
#define WAIT(ms)        do { if (!Wait_Ms((uint32_t)(ms))) return NULL; } while (0)

static const char *Seq_Check(uint8_t slot)
{
    if (slot < 1 || slot > 3)  return "ERR ARG";
    if (g_armok < 0.5f)        return "ERR NOCAL";     /* 姿态参数还没标定 */
    if (!lift_known)           return "ERR NOZERO";
    return NULL;
}

/* 从原料盘夹一个，放进车上转盘 slot 号位 */
static const char *Seq_Grab(uint8_t slot)
{
    const char *e = Seq_Check(slot);
    if (e) return e;

    TT_Go(slot);                                   /* 转盘先转，转的同时手臂也在动，省时间 */
    Claw_Open();
    GO(Lift_Goto(g_zhi));
    SERVO(1, g_a1g);                               /* 转向原料盘 */
    SERVO(2, g_a2e);                               /* 伸出 */
    GO(Lift_Goto(g_zgrab));                        /* 下降 */
    Claw_Close();  WAIT(g_clwait);                 /* 夹紧 */
    GO(Lift_Goto(g_zhi));                          /* 抬起 */
    SERVO(2, g_a2r);                               /* 缩回 */
    SERVO(1, g_a1d);                               /* 转到车上转盘上方 */
    GO(Lift_Goto(g_zdrop));                        /* 下降到转盘上方 */
    if (!TT_WaitReady()) return NULL;              /* 转盘到位了才松手 */
    Claw_Open();   WAIT(g_clwait);                 /* 松开：物料落进转盘 */
    GO(Lift_Goto(g_zhi));
    SERVO(1, g_a1h);                               /* 收起待命 */
    return NULL;
}

/* 从车上转盘 slot 号位取出物料，放到地上圆环 */
static const char *Seq_Place(uint8_t slot)
{
    const char *e = Seq_Check(slot);
    if (e) return e;

    TT_Go(slot);
    Claw_Open();
    GO(Lift_Goto(g_zhi));
    SERVO(2, g_a2r);
    SERVO(1, g_a1d);                               /* 转到车上转盘上方 */
    if (!TT_WaitReady()) return NULL;
    GO(Lift_Goto(g_zdrop));
    Claw_Close();  WAIT(g_clwait);                 /* 夹起 */
    GO(Lift_Goto(g_zhi));
    SERVO(1, g_a1p);                               /* 转向地上圆环 */
    SERVO(2, g_a2p);                               /* 伸出 */
    GO(Lift_Goto(g_zplc));                         /* 下降到物料底面快贴地 */
    Claw_Open();   WAIT(g_clwait);                 /* 松开：物料立在圆环上 */
    GO(Lift_Goto(g_zhi));
    SERVO(2, g_a2r);                               /* 缩回，不要带倒物料 */
    SERVO(1, g_a1h);
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

    if (QR_Match(tmp, n, qr_code))
    {
        char m[32];

        qr_valid = 1;
        Screen_Text("t0", qr_code);                /* 屏上显示任务码(控件名 t0，不一样就改这里) */
        snprintf(m, sizeof(m), "QR %s\r\n", qr_code);
        Say(m);
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

    qr_len = 0;
    qr_end = 0;
    HAL_UART_Receive_IT(&huart5, &qr_rx, 1);       /* UART5 的中断在 hal_msp.c 里已经打开 */
}

/* ================= 树莓派指令 ================= */
#define FAIL(s)   do { snprintf(err, (size_t)errlen, "%s", (s)); return -1; } while (0)

int Arm_Command(const char *cmd, char *err, int errlen)
{
    char *e;

    /* ---- 夹爪：CLAW O / CLAW C / CLAW <微秒> ---- */
    if (strncmp(cmd, "CLAW ", 5) == 0)
    {
        const char *a = cmd + 5;

        if (strcmp(a, "O") == 0)       Claw_Open();
        else if (strcmp(a, "C") == 0)  Claw_Close();
        else
        {
            long us = strtol(a, &e, 10);
            if (e == a || *e != 0 || us < 500 || us > 2500)  FAIL("ERR ARG");
            Claw_Set((uint32_t)us);                /* main.c 里会再按夹爪限位夹住 */
        }
        Wait_Ms((uint32_t)g_clwait);
        return 1;
    }

    /* ---- 转盘：TT <1~3> / TT <微秒> ---- */
    if (strncmp(cmd, "TT ", 3) == 0)
    {
        const char *a = cmd + 3;
        long v = strtol(a, &e, 10);

        if (e == a || *e != 0)  FAIL("ERR ARG");
        if (v >= 1 && v <= 3)
        {
            TT_Go((uint8_t)v);
        }
        else if (v >= 500 && v <= 2700)
        {
            Turntable_Set((uint32_t)v);
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
    if (strcmp(cmd, "LIFT?") == 0)
    {
        char m[40];
        char b[20];

        FmtF(b, (int)sizeof(b), lift_mm);
        snprintf(m, sizeof(m), "LIFT %d %s\r\n", (int)lift_known, b);
        Say(m);
        return 1;
    }
    if (strncmp(cmd, "LIFT ", 5) == 0)
    {
        const char *a = cmd + 5;

        if (strcmp(a, "ZERO") == 0)
        {
            Emm_V5_En_Control(LIFT_ADDR, true, false);
            Emm_V5_Reset_CurPos_To_Zero(LIFT_ADDR);
            lift_mm = 0.0f;
            lift_known = 1;
            return 1;
        }
        if (strcmp(a, "HOME") == 0)
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
        {
            double mm = strtod(a, &e);
            const char *r;

            if (e == a || *e != 0)  FAIL("ERR ARG");
            r = Lift_Goto((float)mm);
            if (r)  FAIL(r);
            return 1;
        }
    }

    /* ---- 夹取 / 放置 ---- */
    if (strncmp(cmd, "GRAB ", 5) == 0 || strncmp(cmd, "PLACE ", 6) == 0)
    {
        int is_grab = (cmd[0] == 'G');
        const char *a = cmd + (is_grab ? 5 : 6);
        long slot = strtol(a, &e, 10);
        const char *r;

        if (e == a || *e != 0)  FAIL("ERR ARG");
        r = is_grab ? Seq_Grab((uint8_t)slot) : Seq_Place((uint8_t)slot);
        if (r)  FAIL(r);
        return 1;
    }

    /* ---- 二维码 ---- */
    if (strcmp(cmd, "QR?") == 0)
    {
        char m[32];

        if (qr_valid)  snprintf(m, sizeof(m), "QR %s\r\n", qr_code);
        else           snprintf(m, sizeof(m), "QR NONE\r\n");
        Say(m);
        return 1;
    }
    if (strcmp(cmd, "QR CLR") == 0)
    {
        qr_valid = 0;
        qr_code[0] = 0;
        return 1;
    }

    /* ---- 串口屏：SCR <控件名> <文字> ---- */
    if (strncmp(cmd, "SCR ", 4) == 0)
    {
        char obj[12];
        char txt[40];
        const char *p = cmd + 4;
        int n = 0;

        while (*p != 0 && *p != ' ' && n < (int)sizeof(obj) - 1)
        {
            obj[n++] = *p++;
        }
        obj[n] = 0;
        if (n == 0 || *p != ' ')  FAIL("ERR ARG");
        p++;
        n = 0;
        while (*p != 0 && n < (int)sizeof(txt) - 1)
        {
            if (*p != '"')                         /* 双引号会弄坏屏的指令，直接去掉 */
            {
                txt[n++] = *p;
            }
            p++;
        }
        txt[n] = 0;
        Screen_Text(obj, txt);
        return 1;
    }

    return 0;
}
