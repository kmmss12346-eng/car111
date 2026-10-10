/* chassis.c  v13 (2026-10-10)
 *  - v13 让底盘更平稳、最后停得更准(树莓派那边 F/S/R 的格式和 DONE/ERR 回复都没变)：
 *    1. 转弯(包括 F/S 走完后自动补的小转弯)：快到目标时按"还差多少度"连续地减速，不再在 0 和 ±4 转/分之间来回切；
 *       刹车不超过驱动器自己的加减速斜坡；"还没转出来的角度"按驱动器斜坡估计；要反向时先停稳一下再往回。
 *       新参数 TCSTOP / TCTOL / TCMIN(见 tun 表)。
 *    2. 横移：前馈从"加速/匀速"切到"减速"时不再一下跳几转/分(前馈限制变化速度，参数 SSFFR)；
 *       航向环照直行 v12a 的做法改：低通滤波、连续死区、纠偏量限制变化速度、轮速没变不重发(参数 SSLPF / SSRL)。
 *    3. 精确模式：F/S 的速度 ≤ PSPD(默认 60 转/分，树莓派对准/回家的小步修正就用 60)时，加减速更柔(PACC)、
 *       最后低速爬到位(PMIN)、停稳多等一会(PSET)、车头偏差超过 PALTOL(0.5°)就转回去(平时是 ALTOL 1.5°)。PSPD=0 关掉。
 *    4. R 支持 1 位小数(Car_Parse_Move，main.c 里用)。
 *  - v12a 只改了闭环直行的纠偏：车头误差先低通滤波、死区改成连续的、纠偏量限制变化速度、轮速没变就不重复发指令。
 *    目的是让匀速直行时轮速不再跟着陀螺仪噪声一直小幅跳动，减轻车身抖动。
 *    参数 SCLPF(滤波时间，秒，0=不滤波) 默认 0.08，SCRL(纠偏量每秒最多变多少 转/分，0=不限制) 默认 30，都是开着的；
 *    两个都 SET 成 0，纠偏算法就和 v12 一样。
 *  - 直行、横移、转弯用同一套"加速→匀速→按剩余距离减速 + 陀螺仪闭环"的方法，路线里的 F / S / R 都走这里
 *  - 转弯绕车中心(转的同时叠加平移)，并且按"累计的目标航向"转，不会越转越歪
 *  - 横移的车头补偿(前馈)带自适应：加速度/速度改了之后，几次横移内自己把补偿量调准
 *  - 直行带积分和"自动学习的偏向补偿"：车天生往一边偏，走几次就自己补上
 *  - 所有重要参数都能用串口命令 SET 名字 数值 在线修改(GET 打印全部)，不用重新烧录
 *  - 收到串口字符 '!' 立刻停车(car_abort)
 *  - 2026-10-07：转弯时按发出去的转速车早该转了 90°、陀螺仪却几乎没变(轮子没转起来)，马上停车并回 ERR STALL，
 *    不再空转 5 秒再试 6 次；加 Car_Motor_Report(MOT?) / Car_Motor_Enable(MOT EN) 读驱动器状态、解除堵转保护
 */
#include "chassis.h"
#include "Emm_V5.h"
#include "hwt101.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
extern UART_HandleTypeDef huart3;
extern float yaw_target;

volatile uint8_t car_abort = 0;     /* 串口收到 '!' 时置 1：正在跑的闭环动作马上停车 */
volatile float   car_last_err = 0;  /* 上一个闭环动作结束时的车头误差(度) */
volatile uint8_t car_stalled = 0;   /* 转弯时轮子没转起来(陀螺仪几乎不变)置 1：这条指令回 ERR STALL */

/* 把角度差折回 -180 ~ 180 度，不管陀螺仪返回的是 -180~180 还是 0~360 都能算对 */
static float Wrap180(float d)
{
    while (d > 180.0f)
    {
        d -= 360.0f;
    }
    while (d < -180.0f)
    {
        d += 360.0f;
    }
    return d;
}
static float Absf(float x)
{
    return (x < 0.0f) ? -x : x;
}
/* 本文件里所有 HWT101_AngleDiff 都改用上面的安全版本 */
#define HWT101_AngleDiff(a, b)   Wrap180((float)(a) - (float)(b))

/* 读 5 次陀螺仪(每次隔 11 毫秒，保证读到不同的帧)，取中间值。
 * 偶尔有一帧角度被干扰读错，单次读数会让车白转好几度甚至十几度；取中间值就不怕单帧出错。 */
static float Yaw_Median5(void)
{
    float a0 = HWT101_GetYaw();
    float d[5];
    uint8_t i, j;

    d[0] = 0.0f;
    for (i = 1; i < 5; i++)
    {
        HAL_Delay(11);
        d[i] = Wrap180(HWT101_GetYaw() - a0);          /* 都换算成相对第一个读数的差，避免 ±180 附近绕圈 */
    }
    for (i = 0; i < 4; i++)                             /* 从小到大排序，取中间那个 */
    {
        for (j = 0; j < 4 - i; j++)
        {
            if (d[j] > d[j + 1])
            {
                float t = d[j];
                d[j] = d[j + 1];
                d[j + 1] = t;
            }
        }
    }
    return Wrap180(a0 + d[2]);
}

/* 防干扰：横移/直行时，车头一个控制周期(约 33 毫秒)转不到 1°。
 * 如果这次读数比上一次的"好读数"突然差了 3° 以上，先当作这一帧读错，沿用上一次的值；
 * 连续 3 次都这样才认(说明车真的转了)。 */
static float Yaw_Guard(float *last, uint8_t *bad)
{
    float y = HWT101_GetYaw();
    float d = Wrap180(y - *last);

    if ((d > 3.0f || d < -3.0f) && *bad < 2)
    {
        (*bad)++;
        return *last;
    }
    *bad = 0;
    *last = y;
    return y;
}


/* ================= 可以在线修改的参数 =================
 * 串口命令：SET 名字 数值 / GET。名字(大写)和含义见下面的 tun 表。
 * 数值改完立刻生效，到下次断电前有效；要长期保存就写到树莓派配置里的 stm32_params(树莓派每次启动都会发给 STM32)。 */
static float g_fppm   = 12.84f;     /* 直行前进每 mm 脉冲数 */
static float g_bppm   = 12.78f;     /* 后退 */
static float g_lppm   = 13.593f;    /* 闭环左移 */
static float g_rppm   = 13.741f;    /* 闭环右移 */
static float g_ppd    = 25.4f;      /* 原地转 1° 要的脉冲数 */

static float g_sc_acc  = 300.0f;    /* 直行加减速(转/分/秒) */
static float g_sc_kp   = 2.0f;      /* 直行：车头每偏 1°，左右轮差多少 转/分 */
static float g_sc_ki   = 1.0f;      /* 直行积分 */
static float g_sc_lpf  = 0.08f;     /* v12a：直行时车头误差的低通滤波时间常数(秒)，0=不滤波 */
static float g_sc_rate = 30.0f;     /* v12a：直行纠偏量每秒最多变化多少 转/分，0=不限制 */
static float g_sc_bias[2] = { 0.0f, 0.0f };   /* 直行/后退 天生的偏向补偿(自动学习) */

static float g_ss_acc  = 200.0f;    /* 横移加减速(转/分/秒) */
static float g_ss_kp   = 2.0f;      /* 横移：车头每偏 1°，左右轮差多少 转/分(再大，车身 0.12 秒的反应延迟会让它来回晃) */
static float g_ss_ki   = 1.0f;      /* 横移积分 */
static float g_ss_lead = 0.14f;     /* 横移前馈提前量(秒)：≈ 车身对轮速指令的延迟 */
static float g_ss_ffg[2] = { 1.0f, 1.0f };    /* 横移前馈增益[左,右]，自动学习 */
static float g_ss_learn = 0.0f;     /* 1=横移结束后再按整次的数据批量修正一次前馈增益(一般只用下面的实时修正就够了) */
static float g_ss_rt    = 1.0f;     /* 横移中实时修正前馈增益的速度(每秒)，0=不实时修正 */
/* 横移时车自己转的模型：偏转速度(度/秒) = C × 速度^1.5(速度单位 转/分)；加速和匀速阶段用 CA，减速阶段用 CD。
 * 左移车头顺时针转(负数)，右移逆时针转(正数)。用 Car_Strafe_Calib(CAL 命令)自动测，不用手填。 */
static float g_ss_ca[2] = { -0.0134f, 0.0128f };      /* [左,右] */
static float g_ss_cd[2] = { -0.0060f, 0.0041f };
static float g_drift[2] = { -0.037f, -0.033f };       /* 横移时往前(+)/后(-)漂：每横移 1mm 漂多少 mm [左,右] */
/* v13：横移航向环照直行 v12a 的做法。SSLPF 取 0.04 秒(直行的一半：横移时车自己转得多，滤波太慢车头会多偏，
 * 模拟里 0.08 秒停下时车头多偏约 0.3°；0.04 秒多偏约 0.1°，陀螺仪噪声引起的轮速来回变化少 15% 左右)；
 * SSRL 取 50 转/分/秒(每个控制周期最多变约 1.7 转/分，前馈没标好时 0.2 秒内也能加到 10 转/分的纠偏)。
 * 两个都 SET 成 0，就和 v12a 的横移一样(硬死区、不滤波、不限速)。 */
static float g_ss_lpf  = 0.04f;     /* 横移时车头误差的低通滤波时间常数(秒)，0=不滤波 */
static float g_ss_rate = 50.0f;     /* 横移纠偏量(含前馈)每秒最多变化多少 转/分，0=不限制 */
/* v13：前馈从 CA 切到 CD(开始减速)时，CA 比 CD 大 2~3 倍，前馈会一下少 4~6 转/分(一个周期左右轮差跳 4~6 转/分)，车头被拽一下。
 * 现在前馈每秒最多变 SSFFR 转/分(40：一个周期最多变约 1.3 转/分，4~6 转/分的落差 0.1~0.15 秒变完)。
 * 加速时前馈涨得慢(约 15 转/分/秒)，不受影响。代价：减速开始那 0.1 秒前馈慢一点，模拟里车头最多多偏不到 0.1°。0=不限制。 */
static float g_ss_ffr  = 40.0f;

static float g_tc_vmax = 80.0f;     /* 转弯最高速(转/分) */
static float g_tc_acc  = 400.0f;    /* 转弯起步加速 */
static float g_tc_dec  = 250.0f;    /* 转弯刹车减速 */
static float g_tc_kp   = 3.0f;
static float g_tc_lag  = 0.16f;     /* 车身对轮速指令的延迟(秒) */
/* v13：转弯末段。预测误差(把已经发出去还没转出来的算进去)≤ TCSTOP 就给 0 转速停着等；停着以后误差 ≤ TCTOL 连续 3 次算到位，
 * 超过 TCTOL 才再动(中间留一段，不会停了又动、动了又停)。离目标近时转速 = TCKP×还差的度数，越近越慢，最低 TCMIN。
 * 以前是 0.6° / 0.8° / 4 转/分 固定的：最后半秒在 0 和 ±8 转/分(后轮)之间来回切好几次。 */
static float g_tc_stop = 0.15f;     /* 预测误差小于它就不再给转速(度) */
static float g_tc_tol  = 0.5f;      /* 停下以后误差在这以内算到位(度) */
static float g_tc_min  = 1.0f;      /* 还没到 TCSTOP 时最低给多少转速(转/分，车身) */
static float g_pv_back = 103.0f;    /* 转轴在车中心后多少 mm */
static float g_pv_left = 0.0f;      /* 转轴在车中心左边多少 mm */
static float g_settle  = 60.0f;     /* 闭环动作走完后等车停稳(毫秒) */
static float g_altol   = 1.5f;      /* 闭环动作走完后车头偏差超过这么多度，就绕车中心转回去(转一次约 1.2 秒；小于它的偏差留给下一条指令的纠偏) */
static float g_dbg     = 0.0f;      /* 1=横移时每 0.1 秒发一行 SC 数据 */
/* v13：精确模式。F/S 带的速度 ≤ PSPD 时用(树莓派对准停车点、回家的小步修正都发 60 转/分)：
 * 加减速 PACC(比平时柔)、最后低速 PMIN 爬到位、走完多等 PSET 毫秒再量车头、车头偏差超过 PALTOL 就转回去。PSPD=0 关掉。
 * 树莓派看 GET 里有没有 PSPD，就知道 STM32 是不是这一版。 */
static float g_p_spd   = 60.0f;     /* 速度 ≤ 它(转/分)的 F/S 用精确模式，0=不用 */
static float g_p_acc   = 120.0f;    /* 精确模式的加减速(转/分/秒)；比平时的 SCACC/SSACC 大时按平时的 */
static float g_p_min   = 3.0f;      /* 精确模式最后爬行的速度(转/分)，平时是 6 */
static float g_p_set   = 120.0f;    /* 精确模式走完后等车停稳(毫秒)，平时是 SETTLE */
static float g_p_altol = 0.5f;      /* 精确模式走完后车头偏差超过这么多度就转回去，平时是 ALTOL */

typedef struct
{
    const char *name;
    float      *p;
    float       lo;
    float       hi;
} Tun;

static const Tun tun[] =
{
    { "FPPM",   &g_fppm,        8.0f,   20.0f },
    { "BPPM",   &g_bppm,        8.0f,   20.0f },
    { "LPPM",   &g_lppm,        8.0f,   20.0f },
    { "RPPM",   &g_rppm,        8.0f,   20.0f },
    { "PPD",    &g_ppd,         10.0f,  60.0f },
    { "SCACC",  &g_sc_acc,      30.0f,  600.0f },
    { "SCKP",   &g_sc_kp,       0.0f,   6.0f },
    { "SCKI",   &g_sc_ki,       0.0f,   3.0f },
    { "SCLPF",  &g_sc_lpf,      0.0f,   0.5f },
    { "SCRL",   &g_sc_rate,     0.0f,   500.0f },
    { "SCBF",   &g_sc_bias[0], -0.06f,  0.06f },
    { "SCBB",   &g_sc_bias[1], -0.06f,  0.06f },
    { "SSACC",  &g_ss_acc,      30.0f,  600.0f },
    { "SSKP",   &g_ss_kp,       0.0f,   6.0f },
    { "SSKI",   &g_ss_ki,       0.0f,   3.0f },
    { "SSLEAD", &g_ss_lead,     0.0f,   0.5f },
    { "SSGL",   &g_ss_ffg[0],   0.0f,   2.5f },
    { "SSGR",   &g_ss_ffg[1],   0.0f,   2.5f },
    { "SSLRN",  &g_ss_learn,    0.0f,   1.0f },
    { "SSRT",   &g_ss_rt,       0.0f,   10.0f },
    { "SSCAL",  &g_ss_ca[0],   -0.2f,   0.2f },
    { "SSCDL",  &g_ss_cd[0],   -0.2f,   0.2f },
    { "SSCAR",  &g_ss_ca[1],   -0.2f,   0.2f },
    { "SSCDR",  &g_ss_cd[1],   -0.2f,   0.2f },
    { "DRL",    &g_drift[0],   -0.2f,   0.2f },
    { "DRR",    &g_drift[1],   -0.2f,   0.2f },
    { "SSLPF",  &g_ss_lpf,      0.0f,   0.5f },
    { "SSRL",   &g_ss_rate,     0.0f,   500.0f },
    { "SSFFR",  &g_ss_ffr,      0.0f,   1000.0f },
    { "TCV",    &g_tc_vmax,     10.0f,  200.0f },
    { "TCA",    &g_tc_acc,      50.0f,  1500.0f },
    { "TCD",    &g_tc_dec,      50.0f,  1500.0f },
    { "TCKP",   &g_tc_kp,       0.5f,   10.0f },
    { "TCLAG",  &g_tc_lag,      0.0f,   0.5f },
    { "TCSTOP", &g_tc_stop,     0.05f,  3.0f },
    { "TCTOL",  &g_tc_tol,      0.1f,   5.0f },
    { "TCMIN",  &g_tc_min,      0.0f,   20.0f },
    { "PVB",    &g_pv_back,    -300.0f, 300.0f },
    { "PVL",    &g_pv_left,    -300.0f, 300.0f },
    { "SETTLE", &g_settle,      0.0f,   1000.0f },
    { "ALTOL",  &g_altol,       0.3f,   10.0f },
    { "DBG",    &g_dbg,         0.0f,   1.0f },
    { "PSPD",   &g_p_spd,       0.0f,   300.0f },
    { "PACC",   &g_p_acc,       30.0f,  600.0f },
    { "PMIN",   &g_p_min,       1.0f,   20.0f },
    { "PSET",   &g_p_set,       0.0f,   1000.0f },
    { "PALTOL", &g_p_altol,     0.2f,   10.0f },
};
#define TUN_N  ((int)(sizeof(tun) / sizeof(tun[0])))

/* 把 v 写成 "-12.3456"(4 位小数)，不用 %f(有的编译设置不支持浮点 printf) */
static void Fmt4(char *b, int n, float v)
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

static void Link_Say(const char *s)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)s, (uint16_t)strlen(s), 100);
}

/* 返回 0=没有这个名字，1=成功，2=数值超出范围 */
int Car_Param_Set(const char *name, float v)
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

/* 通过 USART3 打印全部参数："P 名字=数值" 一行一个 */
void Car_Param_Dump(void)
{
    char m[48];
    char b[20];
    int i;

    for (i = 0; i < TUN_N; i++)
    {
        Fmt4(b, (int)sizeof(b), *tun[i].p);
        snprintf(m, sizeof(m), "P %s=%s\r\n", tun[i].name, b);
        Link_Say(m);
    }
}

/* 旧代码里的名字，指向可调参数 */
#define PULSE_PER_MM_FORWARD   g_fppm
#define PULSE_PER_MM_BACKWARD  g_bppm
#define PULSE_PER_DEG          g_ppd


/* ================= 固定参数 ================= */
#define PULSE_PER_MM_LEFT      13.23f   /* 开环左移(旧函数 Car_Left_mm 用) */
#define PULSE_PER_MM_RIGHT     13.10f

uint8_t move_acc = 200;           /* 旧的位置模式动作、转弯后备方法用的加速度档位 */
#define MOVE_ACC          move_acc

#define STRAFE_ACC       30       /* Car_Left_mm / Car_Right_mm(旧函数)的加速度档位 */

#define PULSES_PER_REV    3200.0f /* 电机转一圈的脉冲数 */
#define TURN_SPEED        200u    /* 后备转向速度 */
#define TURN_ACC          30      /* 后备转向加速度 */
#define TURN_TOL_DEG      1.0f    /* 转向允许误差 */
#define TURN_MAX_ITER     6       /* 转向最多修正几次 */

#define ALIGN_TOL_DEG     1.0f    /* 走完后偏差超过这个角度才转回去 */
#define ALIGN_EXTRA_MS    1000    /* Car_Forward_Align 预计时间之外多等多久 */
#define DEBUG_ALIGN       0

/* 旧函数 Car_Left_mm / Car_Right_mm 的补偿 */
#define STRAFE_LEFT_ROT_DEG_PER_MM   0.0250f
#define STRAFE_RIGHT_ROT_DEG_PER_MM  0.0380f
#define STRAFE_LEFT_FWD_DRIFT_PER_MM 0.100f

#define HOLD_STEP_PULSE   400
#define HOLD_KP           0.12f
#define HOLD_SIGN         1

/* ===== 闭环直行 / 横移 ===== */
#define SC_PERIOD_MS     20       /* 控制周期(毫秒) */
#define SC_MIN_RPM       6.0f     /* 快到终点时的最低爬行速度 */
#define P_CREEP_MM       1.0f     /* v13 精确模式：最后这么多 mm 用 PMIN 慢慢爬(刹车曲线提前这么多降到 PMIN) */
#define SC_DEADBAND      0.2f     /* 偏差小于这么多度就不纠(陀螺仪噪声约 0.35°) */
#define SC_DEADBAND_F    0.05f    /* v12a：滤波以后的死区(滤波后噪声只剩约 0.05°) */
#define SC_CORR_MAX      0.25f    /* 直行纠偏量最多占当前速度的比例 */
#define SC_I_MAX         6.0f     /* 直行积分最多贡献多少 转/分 */
#define SC_BIAS_MAX      0.05f
#define SC_SIGN          1
#define SC_CMD_GAP_MS    2        /* 给 4 个电机发指令之间的间隔 */
#define SC_EMM_ACC       200      /* 电机自己在两次速度指令之间平滑过渡 */

#define SCS_I_MAX        8.0f
#define SCS_CORR_MAX     0.50f
#define SCS_FF_MAX_RPM   30.0f    /* 前馈最多推多少 转/分 */

/* ===== 闭环转弯 ===== */
#define TURN_CLOSED      1
#define TC_TIMEOUT_MS    5000
/* v13：驱动器自己的加减速斜坡：Emm 加速度档位 acc 时，每变 1 转/分要 (256-acc)×50 微秒，acc=200 时约 357 转/分/秒 */
#define TC_DRV_RAMP      (1000000.0f / ((256.0f - (float)SC_EMM_ACC) * 50.0f))
#define TC_BRAKE_USE     0.85f    /* 刹车最多用驱动器斜坡的这么多(留一点余量，模型不准也停得住) */
#define TC_REV_HOLD_MS   100u     /* 要反向时先给 0 转速停这么久，再往回转 */
#define TC_COMP_ON       1        /* 1=Car_TurnBy_Center 叠加平移让车中心不动 */
#define TC_PPM_LAT       13.67f
#define TURN_REF_TOL_DEG 6.0f     /* 车头和记录的目标航向差超过这个角度，就按现在实际的车头算 */
#define TC_STALL_CMD_DEG  90.0f   /* 卡住检测：发出去的转速累计够车转这么多度…… */
#define TC_STALL_MOVE_DEG 3.0f    /* ……陀螺仪却没转出这么多度，就判定轮子没转起来(没电/驱动器保护/卡住)，停车回 ERR STALL */
#define TC_STALL_PRED_DEG 45.0f   /* v13：按模型车身早该转了这么多度…… */
#define TC_STALL_RATIO    0.2f    /* ……实际连它的这么多都没转到，也算轮子没转起来(只转一点：打滑/没劲) */

static void Wheels_Vel(const float s[4]);


/* ================= 底层：4 个电机同时按指定方向走 pulse 脉冲 =================
 * dirs[0..3] 分别是 ID1..ID4 的方向。
 * 注意：ID1、ID3 在车的物理右侧，ID2、ID4 在物理左侧。 */
static void Car_Drive(const uint8_t dirs[4], uint16_t speed, uint32_t pulse)
{
    unsigned int i;

    for (i = 0; i < 4; i++)
    {
        Emm_V5_Pos_Control(i + 1, dirs[i], speed, MOVE_ACC, pulse, 0, true);
        HAL_Delay(10);
    }

    Emm_V5_Synchronous_motion(0x00);
}


/* 带符号脉冲：s>0 该轮前进，s<0 该轮后退；各轮速度按脉冲数成比例，保证同时到位 */
static void Car_Drive_Signed(const int32_t s[4], uint16_t speed)
{
    static const uint8_t fwd[4] = {0, 1, 1, 0};     /* 和 Car_Forward 的方向表一致 */
    int32_t smax = 0;
    unsigned int i;

    for (i = 0; i < 4; i++)
    {
        int32_t a = (s[i] < 0) ? -s[i] : s[i];
        if (a > smax) smax = a;
    }
    if (smax == 0)
    {
        return;
    }

    for (i = 0; i < 4; i++)
    {
        int32_t a = (s[i] < 0) ? -s[i] : s[i];
        uint8_t dir = (s[i] >= 0) ? fwd[i] : (uint8_t)(1 - fwd[i]);
        uint16_t v = (uint16_t)((float)speed * (float)a / (float)smax);

        if (a == 0)
        {
            continue;
        }
        if (v < 1)
        {
            v = 1;
        }

        {
            /* 慢的轮子加速度相应调小，让 4 个轮子同时加速完 */
            uint8_t acc_i = MOVE_ACC;
            if (MOVE_ACC > 0 && v < speed)
            {
                int32_t a2 = 256 - (int32_t)((256 - MOVE_ACC) * ((float)speed / (float)v) + 0.5f);
                if (a2 < 1) a2 = 1;
                acc_i = (uint8_t)a2;
            }
            Emm_V5_Pos_Control(i + 1, dir, v, acc_i, (uint32_t)a, 0, true);
        }
        HAL_Delay(10);
    }

    Emm_V5_Synchronous_motion(0x00);
}


/***********************
 * 停止
 ***********************/
void Car_Stop(void)
{
    Emm_V5_Stop_Now(1, true);
    HAL_Delay(10);

    Emm_V5_Stop_Now(2, true);
    HAL_Delay(10);

    Emm_V5_Stop_Now(3, true);
    HAL_Delay(10);

    Emm_V5_Stop_Now(4, true);
    HAL_Delay(10);

    Emm_V5_Synchronous_motion(0x00);
}

/* 闭环动作用的快速停车：4 个电机的停车指令隔 2 毫秒发，最后同步一起停 */
static void Stop_Quick(void)
{
    uint8_t i;

    for (i = 1; i <= 4; i++)
    {
        Emm_V5_Stop_Now(i, true);
        HAL_Delay(SC_CMD_GAP_MS);
    }
    Emm_V5_Synchronous_motion(0x00);
}


/***********************
 * 按脉冲运动
 ***********************/
void Car_Forward(uint16_t speed, uint32_t pulse)
{
    static const uint8_t dirs[4] = {0, 1, 1, 0};
    Car_Drive(dirs, speed, pulse);
}

void Car_Backward(uint16_t speed, uint32_t pulse)
{
    static const uint8_t dirs[4] = {1, 0, 0, 1};
    Car_Drive(dirs, speed, pulse);
}

void Car_Left(uint16_t speed, uint32_t pulse)
{
    static const uint8_t dirs[4] = {0, 0, 0, 0};
    Car_Drive(dirs, speed, pulse);
}

void Car_Right(uint16_t speed, uint32_t pulse)
{
    static const uint8_t dirs[4] = {1, 1, 1, 1};
    Car_Drive(dirs, speed, pulse);
}

/* 原地右转(顺时针，角度变小) */
void Car_RotateRight(uint16_t speed, uint32_t pulse)
{
    static const uint8_t dirs[4] = {1, 1, 0, 0};
    Car_Drive(dirs, speed, pulse);
}

/* 原地左转(逆时针，角度变大) */
void Car_RotateLeft(uint16_t speed, uint32_t pulse)
{
    static const uint8_t dirs[4] = {0, 0, 1, 1};
    Car_Drive(dirs, speed, pulse);
}


/***********************
 * 按毫米运动(开环，旧动作用)
 ***********************/
void Car_Forward_mm(uint16_t speed, float distance_mm)
{
    Car_Forward(speed, (uint32_t)(distance_mm * PULSE_PER_MM_FORWARD));
}

void Car_Backward_mm(uint16_t speed, float distance_mm)
{
    Car_Backward(speed, (uint32_t)(distance_mm * PULSE_PER_MM_BACKWARD));
}

/* 左平移(开环，旧动作) */
void Car_Left_mm(uint16_t speed, float distance_mm)
{
    int32_t P = (int32_t)(distance_mm * PULSE_PER_MM_LEFT);
    int32_t C = (int32_t)(distance_mm * STRAFE_LEFT_ROT_DEG_PER_MM * PULSE_PER_DEG);
    int32_t s[4];
    uint8_t old_acc = move_acc;

    s[0] =  P + C;      /* ID1 */
    s[1] = -P - C;      /* ID2 */
    s[2] = -P + C;      /* ID3 */
    s[3] =  P - C;      /* ID4 */
    {
        /* 4个轮子同时叠加一点后退，抵消往前漂 */
        int32_t F = (int32_t)(distance_mm * STRAFE_LEFT_FWD_DRIFT_PER_MM * PULSE_PER_MM_BACKWARD);
        s[0] -= F;
        s[1] -= F;
        s[2] -= F;
        s[3] -= F;
    }

    move_acc = STRAFE_ACC;
    Car_Drive_Signed(s, speed);
    move_acc = old_acc;
}

/* 右平移(开环，旧动作) */
void Car_Right_mm(uint16_t speed, float distance_mm)
{
    int32_t P = (int32_t)(distance_mm * PULSE_PER_MM_RIGHT);
    int32_t C = (int32_t)(distance_mm * STRAFE_RIGHT_ROT_DEG_PER_MM * PULSE_PER_DEG);
    int32_t s[4];
    uint8_t old_acc = move_acc;

    s[0] = -P - C;      /* ID1 */
    s[1] =  P + C;      /* ID2 */
    s[2] =  P - C;      /* ID3 */
    s[3] = -P + C;      /* ID4 */

    move_acc = STRAFE_ACC;
    Car_Drive_Signed(s, speed);
    move_acc = old_acc;
}


/* ================= 按陀螺仪角度转向 ================= */

/* 等转完：先等预计时间，再等角度稳定(后备方法用) */
static void Wait_Turn_Done(uint32_t pulse, uint16_t speed)
{
    uint32_t est_ms = (uint32_t)((float)pulse * 60000.0f / ((float)speed * PULSES_PER_REV));
    uint32_t t0 = HAL_GetTick();
    float last = HWT101_GetYaw();
    uint8_t stable = 0;

    HAL_Delay(est_ms);

    while ((HAL_GetTick() - t0) < (est_ms + 3000))
    {
        float now = HWT101_GetYaw();
        float d = HWT101_AngleDiff(now, last);

        if (d < 0) d = -d;
        stable = (d < 0.15f) ? (uint8_t)(stable + 1) : 0;
        last = now;

        if (stable >= 6)
        {
            break;
        }
        HAL_Delay(20);
    }
}


/* ===== 原地转弯时让"车中心"不动 =====
 * 车原地转时不是绕车中心转，而是绕"车中心后面约 100mm"的一个点转，所以每转 90° 车中心会被带偏。
 * 办法：转的同时叠加一个平移，把车中心被带偏的那一份抵消掉(车身转速 ω，转轴在中心后 d_b、偏左 d_l 时，
 * 要叠加的平移速度 = ω×(-d_l 向前, -d_b 向左))。
 * 转完车中心还偏多少，用下面的公式调(Car_TurnBy_Center(90)，量车中心位移 Δx=往车头为正，Δy=往左为正)：
 *   新的 PVB = 旧的 + (Δy - Δx)/2，新的 PVL = 旧的 + (Δx + Δy)/2 */
static uint8_t turn_comp = 0;        /* 1=这次转弯要叠加平移 */

#define TC_LAG_S        g_tc_lag

/* v13 转弯：
 *  - 远处：按刹车曲线 sqrt(2×刹车减速度×还差的角度) 减速；刹车减速度 = min(TCD, 驱动器斜坡×0.85)。
 *    驱动器斜坡按最快的那个轮子算(叠加平移时后轮约是车身转速的 2 倍，所以车身转速每秒最多变约 180 转/分)，
 *    以前 TCKP 那一项要的减速度到 500~1000 转/分/秒，驱动器跟不上，实际转得比预测的多，冲过头再往回。
 *  - 近处：转速 = TCKP×还差的度数，连续地越来越慢；预测误差 ≤ TCSTOP 才给 0。
 *  - "已经转出去、陀螺仪还没看到的角度"：轮子实际转速按驱动器斜坡估计(加减速时和指令差很多)，
 *    车身转速按一阶延迟(时间常数 TCLAG)跟着轮子，还没转出来的 = TCLAG×车身转速。
 *    以前按"纯延迟 TCLAG、用发出去的指令"算：猛刹车时车身比轮子慢得多，算少了，冲过头再往回。
 *  - 预测冲过头了：先给 0，停够 TC_REV_HOLD_MS 再往回，不在两个方向之间来回抽。 */
static void Car_Turn_Closed(float target)
{
    float v = 0.0f;                                     /* 现在给的车身转速大小(轮子 转/分) */
    float s[4];
    uint32_t last = HAL_GetTick();
    uint32_t t0 = last;
    uint32_t t_move = last;                             /* 最后一次给非 0 转速的时间(反向前要停够一会) */
    uint8_t inside = 0;
    uint8_t parked = 0;                                 /* 1 = 预测已经到位，给 0 转速停着等车身转完 */
    int last_dir = 0;                                   /* 上一次往哪边转：1 逆时针，-1 顺时针 */
    float k = PULSES_PER_REV / 60.0f / PULSE_PER_DEG;   /* 轮子 1 转/分 ≈ 车身转多少 度/秒 */
    float yaw_ref = HWT101_GetYaw();                    /* 卡住检测：上一次"车头真的转了"时的读数 */
    float cmd_deg = 0.0f;                               /* 卡住检测：从那以后发出去的转速累计应该转多少度 */
    float pred_deg = 0.0f;                              /* 卡住检测：从那以后按模型车身应该转了多少度 */
    float w_last = 0.0f;                                /* 上个周期发出去的车身转速(轮子 转/分) */
    float w_mot = 0.0f;                                 /* 按驱动器斜坡估计的轮子实际转速(车身转速，转/分，带符号) */
    float w_body = 0.0f;                                /* 按一阶延迟估计的车身实际转速(转/分，带符号) */
    float fac = 1.0f;                                   /* 最快的那个轮子是车身转速的几倍 */
    float ramp, acc, dec, tol;

    if (turn_comp)
    {
        float cu = k * 0.0174533f * g_pv_back * TC_PPM_LAT * 60.0f / PULSES_PER_REV;
        float cf = k * 0.0174533f * g_pv_left * PULSE_PER_MM_FORWARD * 60.0f / PULSES_PER_REV;

        fac = 1.0f + Absf(cu) + Absf(cf);
    }
    ramp = TC_DRV_RAMP / fac;                           /* 车身转速每秒最多能变多少(转/分) */
    acc  = g_tc_acc;                                    /* 起步照旧(驱动器自己按斜坡跟上，上面的模型也按斜坡算) */
    dec  = (g_tc_dec < TC_BRAKE_USE * ramp) ? g_tc_dec : TC_BRAKE_USE * ramp;
    tol  = (g_tc_tol > g_tc_stop) ? g_tc_tol : g_tc_stop;

    while (1)
    {
        uint32_t now;
        float dt, err, errp, errs, a, want, w, yaw_now;
        int dir;

        HAL_Delay(SC_PERIOD_MS);
        now  = HAL_GetTick();
        dt   = (float)(now - last) / 1000.0f;
        last = now;

        if (car_abort)
        {
            break;
        }

        {
            /* 刚过去这一段：轮子按驱动器斜坡从 w_mot 往上次的指令 w_last 变；车身转速按一阶延迟跟着轮子的平均转速 */
            float wm0 = w_mot;
            float st = ramp * dt;

            if (w_last > w_mot + st)       w_mot += st;
            else if (w_last < w_mot - st)  w_mot -= st;
            else                           w_mot = w_last;
            w_body += (0.5f * (wm0 + w_mot) - w_body) * ((TC_LAG_S > 0.001f) ? (1.0f - expf(-dt / TC_LAG_S)) : 1.0f);
        }

        yaw_now = HWT101_GetYaw();
        cmd_deg  += Absf(w_last) * k * dt;
        pred_deg += Absf(w_body) * k * dt;
        {
            float moved = Absf(HWT101_AngleDiff(yaw_now, yaw_ref));

            if (moved >= TC_STALL_MOVE_DEG && moved >= TC_STALL_RATIO * pred_deg)
            {
                yaw_ref  = yaw_now;                     /* 转起来了：重新开始算 */
                cmd_deg  = 0.0f;
                pred_deg = 0.0f;
            }
            else if ((cmd_deg >= TC_STALL_CMD_DEG && moved < TC_STALL_MOVE_DEG) ||
                     (pred_deg >= TC_STALL_PRED_DEG && moved < TC_STALL_RATIO * pred_deg))
            {
                car_stalled = 1;                        /* 轮子没转起来(或者只转一点)：别再空转了 */
                break;
            }
        }

        err  = HWT101_AngleDiff(target, yaw_now);
        if (last_dir != 0 && err * (float)last_dir < -90.0f)
        {
            err += (float)last_dir * 360.0f;           /* 转 180° 时目标在 ±180 附近，读数一抖就变成"往回转 360°"：按原来的方向算 */
        }
        errp = err - TC_LAG_S * w_body * k;            /* 把已经转出去还没看到的算进去 */
        errs = errp - w_mot * Absf(w_mot) / (2.0f * ramp) * k;   /* 现在给 0 转速：轮子按斜坡停下来以后，最后会停在哪 */
        a    = (errs < 0.0f) ? -errs : errs;
        dir  = (errs > 0.0f) ? 1 : -1;

        if (parked)
        {
            if (a > tol)
            {
                parked = 0;                             /* 停下以后差得还多：再动 */
            }
        }
        else if (a <= g_tc_stop)
        {
            parked = 1;
        }

        if (parked)
        {
            v = 0.0f;                                   /* 预测已经到位：不再给转速，等车身转完 */
            if (err <= tol && err >= -tol)
            {
                inside++;
                if (inside >= 3)
                {
                    break;                              /* 真的到位了，并且连续几次都稳 */
                }
            }
            else
            {
                inside = 0;
            }
        }
        else
        {
            uint8_t hold = 0;

            inside = 0;
            if (last_dir != 0 && dir != last_dir)
            {
                v = 0.0f;                               /* 预测冲过头了：先停住…… */
                if ((now - t_move) < TC_REV_HOLD_MS)
                {
                    hold = 1;                           /* ……停够一会再往回 */
                }
            }
            if (!hold)
            {
                /* 刹车曲线：按 dec 刹，刚好停在目标。新的转速要到下一个周期才起作用，所以先减掉这一个周期还要转的 */
                float ac = Absf(errp) - k * v * dt;
                float vb;

                if (ac < 0.0f) ac = 0.0f;
                vb = sqrtf(2.0f * dec * ac / k);

                want = vb;
                if (want > g_tc_kp * ac) want = g_tc_kp * ac;  /* 最后几度：差多少度转多快，越近越慢 */
                if (want > g_tc_vmax)    want = g_tc_vmax;
                if (want < g_tc_min)     want = g_tc_min;
                if (v < want)
                {
                    v += acc * dt;
                    if (v > want) v = want;
                }
                else
                {
                    /* 减速不超过 dec(驱动器跟得上)；已经超出刹车曲线了才用满驱动器斜坡 */
                    v -= ((v > vb) ? ramp : dec) * dt;
                    if (v < want) v = want;
                }
                last_dir = dir;
            }
        }
        if ((now - t0) > TC_TIMEOUT_MS)
        {
            break;
        }

        w = (dir > 0) ? v : -v;
        if (v == 0.0f)
        {
            w = 0.0f;
        }
        else
        {
            t_move = now;
        }
        w_last = w;
        {
            float u = 0.0f;                                   /* 要叠加的横移(向左为正)，转/分 */
            float f = 0.0f;                                   /* 要叠加的前进，转/分 */

            if (turn_comp)
            {
                float om = w * k * 0.0174533f;                /* 车身转速，弧度/秒，逆时针为正 */

                u = -om * g_pv_back * TC_PPM_LAT * 60.0f / PULSES_PER_REV;
                f = -om * g_pv_left * PULSE_PER_MM_FORWARD * 60.0f / PULSES_PER_REV;
            }
            s[0] =  w + u + f;    /* ID1 右前 */
            s[1] = -w - u + f;    /* ID2 左前 */
            s[2] =  w - u + f;    /* ID3 右后 */
            s[3] = -w + u + f;    /* ID4 左后 */
        }
        Wheels_Vel(s);
    }

    s[0] = s[1] = s[2] = s[3] = 0.0f;
    Wheels_Vel(s);
    Stop_Quick();
    HAL_Delay(80);
}

/* 转到绝对的陀螺仪角度 target(-180~180)，转完把 yaw_target 设成它 */
static void Turn_To(float target)
{
    uint8_t i;

#if TURN_CLOSED
    Car_Turn_Closed(target);          /* 先闭环转；转到位了，下面的旧方法检查一下就直接跳出 */
#endif

    for (i = 0; i < TURN_MAX_ITER && !car_abort && !car_stalled; i++)   /* 轮子转不起来时旧方法也没用，不再试 */
    {
        float err = HWT101_AngleDiff(target, HWT101_GetYaw());
        float a = (err < 0) ? -err : err;
        uint32_t pulse;

        if (a <= TURN_TOL_DEG)
        {
            break;
        }

        pulse = (uint32_t)(a * PULSE_PER_DEG * ((i == 0) ? 1.0f : 0.8f));
        if (pulse < 10)
        {
            pulse = 10;
        }

        {
            uint8_t old_acc = move_acc;
            move_acc = TURN_ACC;                     /* 转弯时用小加速度 */
            if (err > 0)
            {
                Car_RotateLeft(TURN_SPEED, pulse);   /* 要让角度变大 */
            }
            else
            {
                Car_RotateRight(TURN_SPEED, pulse);  /* 要让角度变小 */
            }
            move_acc = old_acc;
        }
        Wait_Turn_Done(pulse, TURN_SPEED);
    }

    Stop_Quick();
    car_last_err = HWT101_AngleDiff(target, HWT101_GetYaw());
    if (!car_abort && !car_stalled)
    {
        yaw_target = target;   /* 转完后，以新方向作为要保持的车头方向(被急停打断、轮子没转起来就不改) */
    }
}

/* 把"现在的车头方向"记为要保持的方向(用 5 次读数的中间值，不怕单帧读错) */
void Car_Home(void)
{
    if (HWT101_IsFresh(500))
    {
        yaw_target = Yaw_Median5();
    }
}

/* 相对现在的车头转 delta_deg 度(不叠加平移，绕车自己的转轴)。正数=逆时针，负数=顺时针。|delta| 不超过 180 */
void Car_TurnBy(float delta_deg)
{
    float target;

    if (!HWT101_IsFresh(500))
    {
        return;                                 /* 陀螺仪没数据，不盲转 */
    }

    target = Wrap180(Yaw_Median5() + delta_deg);
    Turn_To(target);
}

/* 路线里的 R 指令：绕车中心转(转的同时叠加平移)，并且按"累计的目标航向"转：
 * 目标 = 记录的目标航向 + delta，不是"现在量到的 + delta"，这样转很多次也不会越转越歪。 */
void Car_TurnBy_Center(float delta_deg)
{
    float meas, target;

    if (!HWT101_IsFresh(500))
    {
        return;
    }
    meas = Yaw_Median5();
    if (Absf(Wrap180(meas - yaw_target)) <= TURN_REF_TOL_DEG)
    {
        target = Wrap180(yaw_target + delta_deg);
    }
    else
    {
        target = Wrap180(meas + delta_deg);           /* 差太多(撞过/被拿起来过)：按现在的车头算 */
    }
#if TC_COMP_ON
    turn_comp = 1;
#endif
    Turn_To(target);
    turn_comp = 0;
}


/* ================= 直行(开环 + 走完对齐，旧动作用) ================= */

void Car_Forward_Align(uint16_t speed, float distance_mm)
{
    uint32_t pulses = (uint32_t)(distance_mm * PULSE_PER_MM_FORWARD);
    uint32_t wait_ms = (uint32_t)((float)pulses * 60000.0f / ((float)speed * PULSES_PER_REV)) + ALIGN_EXTRA_MS;
    float err;

    Car_Forward(speed, pulses);
    HAL_Delay(wait_ms);

    err = HWT101_AngleDiff(yaw_target, HWT101_GetYaw());

#if DEBUG_ALIGN
    {
        char m[48];
        int dn = snprintf(m, sizeof(m), "ALIGN err=%ld\r\n", (long)(err * 100.0f));
        HAL_UART_Transmit(&huart3, (uint8_t *)m, dn, 100);
    }
#endif

    if (err > ALIGN_TOL_DEG || err < -ALIGN_TOL_DEG)
    {
        Car_TurnBy(err);
    }
}

/* 旧名字，保留兼容 */
void Car_Forward_Yaw_mm(uint16_t speed, float distance_mm)
{
    Car_Forward_Align(speed, distance_mm);
}


/* 边走边纠偏的直行(旧的实验版本，保留不用) */
void Car_Forward_Hold(uint16_t speed, float distance_mm)
{
    static const uint8_t dirs[4] = {0, 1, 1, 0};
    int32_t total = (int32_t)(distance_mm * PULSE_PER_MM_FORWARD);

    if (!HWT101_IsFresh(500))
    {
        return;
    }

    while (total > 0)
    {
        int32_t step = (total > HOLD_STEP_PULSE) ? HOLD_STEP_PULSE : total;
        float err = HWT101_AngleDiff(yaw_target, HWT101_GetYaw());
        float corr = HOLD_SIGN * HOLD_KP * PULSE_PER_DEG * err;
        float lim = 0.4f * (float)step;
        int32_t pr, pl, pmax;
        uint8_t i;

        if (corr > lim)  corr = lim;
        if (corr < -lim) corr = -lim;

        pr = step + (int32_t)corr;     /* 物理右侧：ID1、ID3 */
        pl = step - (int32_t)corr;     /* 物理左侧：ID2、ID4 */

        for (i = 0; i < 4; i++)
        {
            uint32_t p = (uint32_t)((i == 0 || i == 2) ? pr : pl);

            Emm_V5_Pos_Control(i + 1, dirs[i], speed, MOVE_ACC, p, 0, true);
            HAL_Delay(10);
        }
        Emm_V5_Synchronous_motion(0x00);

        pmax = (pr > pl) ? pr : pl;
        HAL_Delay((uint32_t)((float)pmax * 60000.0f / ((float)speed * PULSES_PER_REV)) + 15);

        total -= step;
    }

    Car_Stop();
}


/* ================= 闭环：速度模式 + 陀螺仪边走边纠偏 ================= */

/* 4 个轮子按带符号速度转：s>0 该轮往前，s<0 往后，单位 转/分
 * ID1、ID3 在车的右侧，ID2、ID4 在左侧 */
static void Wheels_Vel(const float s[4])
{
    static const uint8_t fwd[4] = {0, 1, 1, 0};   /* 和 Car_Forward 的方向表一致 */
    uint8_t i;

    for (i = 0; i < 4; i++)
    {
        float a = (s[i] < 0.0f) ? -s[i] : s[i];
        uint8_t dir = (s[i] >= 0.0f) ? fwd[i] : (uint8_t)(1 - fwd[i]);

        Emm_V5_Vel_Control(i + 1, dir, (uint16_t)(a + 0.5f), SC_EMM_ACC, true);
        HAL_Delay(SC_CMD_GAP_MS);
    }
    Emm_V5_Synchronous_motion(0x00);
}

/* 走完后检查车头：偏了超过 tol 度(平时 ALTOL，精确模式 PALTOL)就绕车中心转回 yaw_target */
static void Heading_Fix(float yaw_meas, float tol)
{
    float err = HWT101_AngleDiff(yaw_target, yaw_meas);

    car_last_err = err;
    if (!car_abort && (err > tol || err < -tol))
    {
#if TC_COMP_ON
        turn_comp = 1;
#endif
        Turn_To(yaw_target);
        turn_comp = 0;
    }
}


/* v13：这条 F/S 用不用精确模式(速度 ≤ PSPD，PSPD=0 不用) */
static uint8_t Prec_On(uint16_t speed)
{
    return (g_p_spd > 0.5f && (float)speed <= g_p_spd + 0.001f) ? 1 : 0;
}

/* 闭环直行 distance_mm：正数前进，负数后退 */
void Car_Straight_Closed(uint16_t speed, float distance_mm)
{
    float sgn    = (distance_mm >= 0.0f) ? 1.0f : -1.0f;
    uint8_t di   = (distance_mm >= 0.0f) ? 0 : 1;
    float ppm    = (distance_mm >= 0.0f) ? PULSE_PER_MM_FORWARD : PULSE_PER_MM_BACKWARD;
    float target = ((distance_mm >= 0.0f) ? distance_mm : -distance_mm) * ppm;   /* 要走的脉冲数 */
    uint8_t prec = Prec_On(speed);                                    /* v13：精确模式 */
    float acc    = (prec && g_p_acc < g_sc_acc) ? g_p_acc : g_sc_acc;
    float vmin   = prec ? g_p_min : SC_MIN_RPM;
    float creep  = prec ? P_CREEP_MM * ppm : 0.0f;                    /* 最后慢慢爬的脉冲数 */
    float a_pps2 = acc * PULSES_PER_REV / 60.0f;
    float vmax   = (float)speed;
    float done   = 0.0f;
    float v      = 0.0f;
    float integ  = 0.0f;
    double bsum  = 0.0, bw = 0.0;                 /* 学习"天生偏向"：巡航阶段 PI 输出 / 速度 的加权平均 */
    uint16_t bn  = 0;
    float s[4];
    float err = 0.0f;
    float err_f;                                  /* v12a：滤波后的车头误差 */
    float corr_prev = 0.0f;                       /* v12a：上一次的纠偏量(限制变化速度用) */
    int16_t sent[4] = { 32767, 32767, 32767, 32767 };   /* v12a：上一次发给 4 个电机的速度(取整后) */
    uint8_t keep = 0;                             /* v12a：连续几次没重发了(最多隔 0.2 秒补发一次，防止某次指令丢了) */
    float yg_last;
    uint8_t yg_bad = 0;
    uint32_t last;

    if (!HWT101_IsFresh(500) || target < 1.0f || car_abort)
    {
        return;                                   /* 陀螺仪没数据，不盲走 */
    }

    last = HAL_GetTick();
    yg_last = Yaw_Median5();
    err_f = HWT101_AngleDiff(yaw_target, yg_last);

    while (1)
    {
        uint32_t now;
        float dt, rem, vbrake, pi_out, corr, lim, yaw_now;

        HAL_Delay(SC_PERIOD_MS);
        now  = HAL_GetTick();
        dt   = (float)(now - last) / 1000.0f;
        last = now;

        if (car_abort)
        {
            break;
        }

        done += v * PULSES_PER_REV / 60.0f * dt;
        rem   = target - done;
        if (rem <= 0.0f)
        {
            break;
        }

        /* 速度规划：加速 -> 匀速 -> 按剩余距离减速(精确模式最后 creep 脉冲用最低速度爬) */
        v += acc * dt;
        if (v > vmax)
        {
            v = vmax;
        }
        vbrake = sqrtf(2.0f * a_pps2 * ((rem > creep) ? (rem - creep) : 0.0f)) * 60.0f / PULSES_PER_REV;
        if (v > vbrake)
        {
            v = vbrake;
        }
        if (v < vmin)
        {
            v = vmin;
        }

        /* 方向纠偏：err>0 要往逆时针转 -> 右侧轮子快、左侧轮子慢 */
        yaw_now = Yaw_Guard(&yg_last, &yg_bad);
        err = HWT101_AngleDiff(yaw_target, yaw_now);
        if (g_sc_lpf > 0.001f)
        {
            /* v12a：一阶低通滤波，去掉陀螺仪 0.3° 左右的来回跳动 */
            float ep;
            err_f += (err - err_f) * dt / (g_sc_lpf + dt);
            /* 比例项：滤波后噪声小了，死区只留 SC_DEADBAND_F，而且是连续的(超出的部分才纠，不会突然跳一下) */
            ep = err_f;
            if (ep > SC_DEADBAND_F)       ep -= SC_DEADBAND_F;
            else if (ep < -SC_DEADBAND_F) ep += SC_DEADBAND_F;
            else                          ep = 0.0f;
            /* 积分项用滤波后的误差、不留死区，长时间的小偏差也会慢慢纠掉 */
            integ += g_sc_ki * err_f * dt;
            err = ep;
        }
        else
        {
            if (err > -SC_DEADBAND && err < SC_DEADBAND)
            {
                err = 0.0f;                       /* SCLPF=0：和 v12 一样 */
            }
            integ += g_sc_ki * err * dt;
        }
        lim = SC_CORR_MAX * v;
        if (integ >  SC_I_MAX) integ =  SC_I_MAX;
        if (integ < -SC_I_MAX) integ = -SC_I_MAX;
        pi_out = SC_SIGN * (g_sc_kp * err + integ);
        corr = pi_out + g_sc_bias[di] * v;
        if (g_sc_rate > 0.001f)
        {
            /* v12a：纠偏量每 20 毫秒最多变 SCRL*dt 转/分，轮速变化平滑 */
            float dmax = g_sc_rate * dt;
            if (corr > corr_prev + dmax) corr = corr_prev + dmax;
            if (corr < corr_prev - dmax) corr = corr_prev - dmax;
        }
        if (corr > lim)  corr = lim;
        if (corr < -lim) corr = -lim;
        corr_prev = corr;

        if (v >= 0.6f * vmax && rem > 0.25f * target)   /* 巡航中(还没开始减速)：记下 PI 输出，用来学习偏向 */
        {
            bsum += (double)pi_out * v;
            bw   += (double)v * v;
            bn++;
        }

        s[0] = sgn * v + corr;    /* ID1 右前 */
        s[2] = sgn * v + corr;    /* ID3 右后 */
        s[1] = sgn * v - corr;    /* ID2 左前 */
        s[3] = sgn * v - corr;    /* ID4 左后 */
        {
            /* v12a：4 个轮子取整后的速度都没变，就不重复发指令(电机只认整数 转/分)；但最多隔 0.2 秒补发一次 */
            int16_t q[4];
            uint8_t i, same = 1;
            for (i = 0; i < 4; i++)
            {
                q[i] = (int16_t)((s[i] >= 0.0f) ? (s[i] + 0.5f) : (s[i] - 0.5f));
                if (q[i] != sent[i]) same = 0;
            }
            if (!same || ++keep >= 10)
            {
                Wheels_Vel(s);
                for (i = 0; i < 4; i++) sent[i] = q[i];
                keep = 0;
            }
        }
    }

    s[0] = s[1] = s[2] = s[3] = 0.0f;
    Wheels_Vel(s);
    Stop_Quick();
    HAL_Delay((uint32_t)(prec ? g_p_set : g_settle));

    {
        float ym = Yaw_Median5();

        /* 学习偏向：巡航够久、结束时车头基本正，才更新(撞到东西/打滑的那次不学) */
        if (!car_abort && bn >= 12 && bw > 1.0 && Absf(Wrap180(yaw_target - ym)) < 2.0f)
        {
            float b = g_sc_bias[di] + 0.6f * (float)(bsum / bw);

            if (b >  SC_BIAS_MAX) b =  SC_BIAS_MAX;
            if (b < -SC_BIAS_MAX) b = -SC_BIAS_MAX;
            g_sc_bias[di] = b;
        }
        Heading_Fix(ym, prec ? g_p_altol : g_altol);
    }
}


/* ===== 闭环横移 ===== */
#define SCS_PERIOD_DBG_MS  100

/* 车自己转的模型里的"速度^1.5" */
static float Vpow(float v)
{
    return v * sqrtf(v);
}

/* 往前看 lead 秒：按和正式运行一样的速度规则(加减速 acc、最低速度 vmin)，预测那时的速度和加速度 */
static void Strafe_Lookahead(float v, float rem, float vmax, float lead, float acc, float vmin, float *vl, float *al)
{
    float h = 0.025f;
    float a_pps2 = acc * PULSES_PER_REV / 60.0f;
    float vv = v;
    float vlast = v;
    float rr = rem;
    int n = (int)(lead / h + 0.5f);
    int i;

    for (i = 0; i < n; i++)
    {
        float vb;

        vlast = vv;
        vv += acc * h;
        if (vv > vmax)
        {
            vv = vmax;
        }
        vb = sqrtf(2.0f * a_pps2 * ((rr > 1.0f) ? rr : 1.0f)) * 60.0f / PULSES_PER_REV;
        if (vv > vb)
        {
            vv = vb;
        }
        if (vv < vmin)
        {
            vv = vmin;
        }
        rr -= vv * PULSES_PER_REV / 60.0f * h;
    }
    *vl = vv;
    *al = (n > 0) ? (vv - vlast) / h : 0.0f;
}

/* calib=0：正常闭环横移；calib=1：不纠偏，边走边记录车头怎么转，最后算出 CA、CD */
static void Strafe_Run(uint16_t speed, float distance_mm, uint8_t calib)
{
    float sl     = (distance_mm >= 0.0f) ? 1.0f : -1.0f;              /* 左移 +1，右移 -1 */
    uint8_t di   = (distance_mm >= 0.0f) ? 0 : 1;
    float ppm    = (distance_mm >= 0.0f) ? g_lppm : g_rppm;
    float drift  = g_drift[di];
    float target = ((distance_mm >= 0.0f) ? distance_mm : -distance_mm) * ppm;
    uint8_t prec = (!calib && Prec_On(speed)) ? 1 : 0;              /* v13：精确模式(校准时不用) */
    float acc    = (prec && g_p_acc < g_ss_acc) ? g_p_acc : g_ss_acc;
    float vmin   = prec ? g_p_min : SC_MIN_RPM;
    float creep  = prec ? P_CREEP_MM * ppm : 0.0f;                    /* 最后慢慢爬的脉冲数 */
    float a_pps2 = acc * PULSES_PER_REV / 60.0f;
    float k_turn = PULSES_PER_REV / 60.0f / PULSE_PER_DEG;            /* 左右轮差 1 转/分 → 车身转多少 度/秒 */
    float vmax   = (float)speed;
    float done   = 0.0f;
    float v      = 0.0f;
    float v_prev = 0.0f;
    float integ  = 0.0f;
    float Za = 0.0f, Zd = 0.0f;                                       /* 速度^1.5 对时间的积分：加速阶段、减速阶段 */
    float yaw0;
    double Saa = 0, Sad = 0, Sdd = 0, Say = 0, Sdy = 0;
    double lnum = 0.0, lden = 0.0;                                    /* 学习前馈增益：PI 输出和前馈的相关 */
    uint16_t cnt = 0;
    float s[4];
    float err = 0.0f;
    float err_f;                                                      /* v13：滤波后的车头误差 */
    float corr_prev = 0.0f;                                           /* v13：上一次的纠偏量(限制变化速度用) */
    float ffu_prev = 0.0f;                                            /* v13：上一次的前馈(限制变化速度用) */
    int16_t sent[4] = { 32767, 32767, 32767, 32767 };                 /* v13：上一次发给 4 个电机的速度(取整后) */
    uint8_t keep = 0;                                                 /* v13：连续几次没重发了 */
    float yg_last;
    uint8_t yg_bad = 0;
    uint32_t last;
    uint32_t dbg_t = 0;

    if (!HWT101_IsFresh(500) || target < 1.0f || car_abort)
    {
        return;
    }

    last = HAL_GetTick();
    yaw0 = Yaw_Median5();
    yg_last = yaw0;
    err_f = HWT101_AngleDiff(yaw_target, yaw0);

    while (1)
    {
        uint32_t now;
        float dt, rem, vbrake, corr, lim, fc, vacc, yaw_now;
        float ffu = 0.0f;                                             /* 前馈(不带增益)，转/分 */
        float pi_out = 0.0f;

        HAL_Delay(SC_PERIOD_MS);
        now  = HAL_GetTick();
        dt   = (float)(now - last) / 1000.0f;
        last = now;

        if (car_abort)
        {
            break;
        }

        done += v * PULSES_PER_REV / 60.0f * dt;
        rem   = target - done;
        if (rem <= 0.0f)
        {
            break;
        }

        v += acc * dt;
        if (v > vmax)
        {
            v = vmax;
        }
        vbrake = sqrtf(2.0f * a_pps2 * ((rem > creep) ? (rem - creep) : 0.0f)) * 60.0f / PULSES_PER_REV;
        if (v > vbrake)
        {
            v = vbrake;
        }
        if (v < vmin)
        {
            v = vmin;
        }
        vacc   = (dt > 0.001f) ? (v - v_prev) / dt : 0.0f;            /* 速度曲线的加速度(转/分/秒)，减速时是负数 */
        v_prev = v;

        yaw_now = Yaw_Guard(&yg_last, &yg_bad);
        err = HWT101_AngleDiff(yaw_target, yaw_now);

        if (calib)
        {
            double y = (double)HWT101_AngleDiff(yaw_now, yaw0);   /* 车头已经转了多少度 */

            if (vacc >= 0.0f)
            {
                Za += Vpow(v) * dt;
            }
            else
            {
                Zd += Vpow(v) * dt;
            }
            Saa += (double)Za * Za;  Sad += (double)Za * Zd;  Sdd += (double)Zd * Zd;
            Say += (double)Za * y;   Sdy += (double)Zd * y;
            cnt++;
            corr = 0.0f;                                              /* 校准时完全不纠偏 */
        }
        else
        {
            float e2 = err;

            if (g_ss_lpf > 0.001f)
            {
                /* v13：和直行 v12a 一样：一阶低通滤波，死区连续，积分用滤波后的误差、不留死区 */
                err_f += (err - err_f) * dt / (g_ss_lpf + dt);
                e2 = err_f;
                if (e2 > SC_DEADBAND_F)       e2 -= SC_DEADBAND_F;
                else if (e2 < -SC_DEADBAND_F) e2 += SC_DEADBAND_F;
                else                          e2 = 0.0f;
                integ += g_ss_ki * err_f * dt;
            }
            else
            {
                if (e2 > -SC_DEADBAND && e2 < SC_DEADBAND)
                {
                    e2 = 0.0f;                                        /* SSLPF=0：和 v12a 一样 */
                }
                integ += g_ss_ki * e2 * dt;
            }
            if (integ >  SCS_I_MAX) integ =  SCS_I_MAX;
            if (integ < -SCS_I_MAX) integ = -SCS_I_MAX;

            pi_out = SC_SIGN * (g_ss_kp * e2 + integ);
            corr = pi_out;
            {
                float vl, al, ca;

                Strafe_Lookahead(v, rem, vmax, g_ss_lead, acc, vmin, &vl, &al);
                ca = (al >= 0.0f) ? g_ss_ca[di] : g_ss_cd[di];
                ffu = -ca * Vpow(vl) / k_turn;                         /* 提前顶住车自己要转的那一份 */
                if (ffu >  SCS_FF_MAX_RPM) ffu =  SCS_FF_MAX_RPM;
                if (ffu < -SCS_FF_MAX_RPM) ffu = -SCS_FF_MAX_RPM;
                if (g_ss_ffr > 0.001f)
                {
                    /* v13：CA 切到 CD 时前馈不再一下跳几转/分，每秒最多变 SSFFR */
                    float dmax = g_ss_ffr * dt;
                    if (ffu > ffu_prev + dmax) ffu = ffu_prev + dmax;
                    if (ffu < ffu_prev - dmax) ffu = ffu_prev - dmax;
                }
                ffu_prev = ffu;
                corr += g_ss_ffg[di] * ffu;
            }
            lnum += (double)pi_out * ffu * dt;
            lden += (double)ffu * ffu * dt;
            if (g_ss_rt > 0.0f)
            {
                /* 实时修正前馈增益(归一化 LMS)：PI 输出和前馈同方向说明前馈不够，反方向说明过头 */
                float g2 = g_ss_ffg[di] + g_ss_rt * dt * pi_out * ffu / (ffu * ffu + 20.0f);

                if (g2 < 0.3f) g2 = 0.3f;
                if (g2 > 2.0f) g2 = 2.0f;
                g_ss_ffg[di] = g2;
            }
        }
        if (!calib && g_ss_rate > 0.001f)
        {
            /* v13：纠偏量(PI + 前馈)每秒最多变 SSRL 转/分 */
            float dmax = g_ss_rate * dt;
            if (corr > corr_prev + dmax) corr = corr_prev + dmax;
            if (corr < corr_prev - dmax) corr = corr_prev - dmax;
        }
        lim = SCS_CORR_MAX * v;
        if (corr > lim)  corr = lim;
        if (corr < -lim) corr = -lim;
        corr_prev = corr;

        /* 抵消横移时顺带往前/往后漂 */
        fc = -drift * v * (PULSE_PER_MM_FORWARD / ppm);

        /* 左移时 ID1、ID4 往前，ID2、ID3 往后(和 Car_Left_mm 一致) */
        s[0] =  sl * v + corr + fc;   /* ID1 右前 */
        s[1] = -sl * v - corr + fc;   /* ID2 左前 */
        s[2] = -sl * v + corr + fc;   /* ID3 右后 */
        s[3] =  sl * v - corr + fc;   /* ID4 左后 */
        {
            /* v13：和直行一样，4 个轮子取整后的速度都没变就不重复发指令，但最多隔 10 个周期补发一次 */
            int16_t q[4];
            uint8_t i, same = 1;
            for (i = 0; i < 4; i++)
            {
                q[i] = (int16_t)((s[i] >= 0.0f) ? (s[i] + 0.5f) : (s[i] - 0.5f));
                if (q[i] != sent[i]) same = 0;
            }
            if (!same || ++keep >= 10)
            {
                Wheels_Vel(s);
                for (i = 0; i < 4; i++) sent[i] = q[i];
                keep = 0;
            }
        }

        if (g_dbg > 0.5f && (now - dbg_t) >= SCS_PERIOD_DBG_MS)
        {
            char m[64];
            int n = snprintf(m, sizeof(m), "SC v=%d err=%d corr=%d mm=%d\r\n",
                             (int)v, (int)(err * 100.0f), (int)corr, (int)(done / ppm));
            dbg_t = now;
            Link_Say(m);
            (void)n;
        }
    }

    s[0] = s[1] = s[2] = s[3] = 0.0f;
    Wheels_Vel(s);
    Stop_Quick();
    HAL_Delay((uint32_t)(prec ? g_p_set : g_settle));

    if (calib)
    {
        double det = Saa * Sdd - Sad * Sad;
        char m[96];

        if (cnt > 20 && det > 1e-9)
        {
            float ca = (float)((Say * Sdd - Sdy * Sad) / det);
            float cd = (float)((Sdy * Saa - Say * Sad) / det);

            g_ss_ca[di] = ca;                                         /* 立刻生效 */
            g_ss_cd[di] = cd;
            g_ss_ffg[di] = 1.0f;
            snprintf(m, sizeof(m), "CAL %s CA=%ld CD=%ld (x1e-7) n=%d\r\n",
                     di ? "RIGHT" : "LEFT", (long)(ca * 1.0e7f), (long)(cd * 1.0e7f), (int)cnt);
        }
        else
        {
            snprintf(m, sizeof(m), "CAL FAIL n=%d\r\n", (int)cnt);
        }
        Link_Say(m);
    }
    else if (g_ss_learn > 0.5f && !car_abort && lden > 25.0)
    {
        /* 自适应前馈增益：PI 输出和前馈同方向 → 前馈不够，加大；反方向 → 前馈过头，减小 */
        float g = g_ss_ffg[di] + 0.5f * (float)(lnum / lden);

        if (g < 0.3f) g = 0.3f;
        if (g > 2.0f) g = 2.0f;
        g_ss_ffg[di] = g;
    }

    {
        float ym = Yaw_Median5();

        if (g_dbg > 0.5f)
        {
            char m[64];
            int n = snprintf(m, sizeof(m), "SC END err=%d g=%d\r\n",
                             (int)(HWT101_AngleDiff(yaw_target, ym) * 100.0f), (int)(g_ss_ffg[di] * 1000.0f));
            (void)n;
            Link_Say(m);
        }
        if (!calib)
        {
            Heading_Fix(ym, prec ? g_p_altol : g_altol);
        }
    }
}

/* 闭环横移 distance_mm：正数左移，负数右移 */
void Car_Strafe_Closed(uint16_t speed, float distance_mm)
{
    Strafe_Run(speed, distance_mm, 0);
}

/* 校准：先不纠偏地左移 distance_mm，再右移 distance_mm(车回到起点)，
 * 自动测出左、右两个方向车自己要转的模型(CA、CD)，结果从 USART3 打印："CAL LEFT …" 和 "CAL RIGHT …"，并且立刻生效。
 * 要用和路线一样的速度和距离(距离够长，才有匀速阶段)。车的左边和周围要留够空地。 */
void Car_Strafe_Calib(uint16_t speed, float distance_mm)
{
    float d = (distance_mm >= 0.0f) ? distance_mm : -distance_mm;

    Strafe_Run(speed, d, 1);            /* 左移 */
    HAL_Delay(1500);
    Strafe_Run(speed, -d, 1);           /* 右移，回到起点 */
    /* 校准是不纠偏的，车头已经转歪了：转回 yaw_target */
    {
        float ym = Yaw_Median5();

        Heading_Fix(ym, g_altol);
    }
}


/* ================= 路线用：树莓派发来的 F/B/L/R 都走这里 =================
 * kind: 'F' 前进  'B' 后退  'L' 左移  'R' 右移，全部是闭环 */
void Car_Move_Align(char kind, uint16_t speed, float distance_mm)
{
    if (kind == 'F')
    {
        Car_Straight_Closed(speed, distance_mm);
    }
    else if (kind == 'B')
    {
        Car_Straight_Closed(speed, -distance_mm);
    }
    else if (kind == 'L')
    {
        Car_Strafe_Closed(speed, distance_mm);
    }
    else if (kind == 'R')
    {
        Car_Strafe_Closed(speed, -distance_mm);
    }
}


/* ================= 解析树莓派的 F / S / R 指令(main.c 的 Link_Poll 用) =================
 * "F <mm> [速度]"、"S <mm> [速度]"：mm 和速度都是整数(和以前一样)，*val = mm
 * "R <度> [速度]"：v13 起角度可以带小数，按 0.1° 四舍五入，*val = 角度×10(整数，例如 "R -1.4" → -14，"R 0.04" → 0 = HOME)；
 *                  整数写法和以前完全一样。速度照样检查格式和范围，但转弯不用它。
 * 返回 1 = 格式对(*kind、*val、*sp 填好，没带速度 *sp=0)；0 = 不是 F/S/R 指令；-1 = 格式不对(回 ERR ARG)。
 * 数值范围(±2500mm、±180°)不在这里查，main.c 先查陀螺仪再查范围，顺序和以前一样。 */
static int Parse_Long(const char *p, const char **end, long *out)
{
    long v = 0;
    int neg = 0;
    const char *q = p;

    while (*q == ' ' || *q == '\t')                      /* 和 strtol 一样：前面的空格跳过 */
    {
        q++;
    }
    if (*q == '+' || *q == '-')
    {
        neg = (*q == '-');
        q++;
    }
    if (*q < '0' || *q > '9')
    {
        return 0;
    }
    while (*q >= '0' && *q <= '9')
    {
        if (v < 100000000L)                               /* 太大的数不会溢出，照样回 ERR RANGE */
        {
            v = v * 10 + (*q - '0');
        }
        q++;
    }
    *out = neg ? -v : v;
    *end = q;
    return 1;
}

/* 带 1 位小数的角度：返回角度×10(第 2 位小数四舍五入)，格式不对返回 0 */
static int Parse_Tenths(const char *p, const char **end, long *out)
{
    long v = 0;
    int neg = 0;
    int digits = 0;
    const char *q = p;

    while (*q == ' ' || *q == '\t')
    {
        q++;
    }
    if (*q == '+' || *q == '-')
    {
        neg = (*q == '-');
        q++;
    }
    while (*q >= '0' && *q <= '9')
    {
        if (v < 10000000L)            /* 乘 10 以后也不会溢出 */
        {
            v = v * 10 + (*q - '0');
        }
        q++;
        digits++;
    }
    v *= 10;
    if (*q == '.')
    {
        q++;
        if (*q >= '0' && *q <= '9')
        {
            v += (*q - '0');                              /* 第 1 位小数 */
            q++;
            digits++;
            if (*q >= '5' && *q <= '9')
            {
                v++;                                      /* 第 2 位小数 ≥5：进一 */
            }
            while (*q >= '0' && *q <= '9')
            {
                q++;
            }
        }
    }
    if (digits == 0)
    {
        return 0;
    }
    *out = neg ? -v : v;
    *end = q;
    return 1;
}

int Car_Parse_Move(const char *cmd, char *kind, long *val, long *sp)
{
    const char *end;
    const char *e2;
    char c = cmd[0];
    int ok;

    if (!((c == 'F' || c == 'S' || c == 'R') && cmd[1] == ' '))
    {
        return 0;
    }
    *kind = c;
    *sp = 0;
    if (c == 'R')
    {
        ok = Parse_Tenths(&cmd[2], &end, val);
    }
    else
    {
        ok = Parse_Long(&cmd[2], &end, val);
    }
    if (!ok)
    {
        return -1;
    }
    if (*end == ' ')
    {
        if (!Parse_Long(end + 1, &e2, sp) || *e2 != 0 || *sp < 5 || *sp > 300)
        {
            return -1;
        }
    }
    else if (*end != 0)
    {
        return -1;
    }
    return 1;
}


/* ================= 电机驱动器诊断：MOT? / MOT EN =================
 * 轮子 1~4 号、升降 5 号都挂在 UART4 上。STM32 平时只发不收；这里发"读参数"指令，再从 UART4 读驱动器的回复：
 *   总线电压 0x24：回 地址 24 电压高 电压低 6B(单位 mV)
 *   状态标志 0x3A：回 地址 3A 标志 6B；bit0 使能、bit1 到位、bit2 堵转、bit3 堵转保护(触发后电机松轴，不再响应运动指令)
 * 收不到回复：驱动器没电、地址不对，或者驱动器的 TX 没接到 STM32 的 RX(PC11)(以前的程序不读回复，这根线可能本来就没接)。 */
#define MOT_REPLY_MS   20

static uint8_t mot_raw[24];          /* 最近一次收到的原始字节(对不上格式时打印出来) */
static uint8_t mot_raw_n = 0;

static uint8_t Mot_RxByte(uint8_t *b)
{
    if (huart4.Instance->SR & USART_SR_RXNE)          /* 溢出(ORE)时 RXNE 也是 1：读 DR 一起清掉 */
    {
        *b = (uint8_t)(huart4.Instance->DR & 0xFFu);
        return 1;
    }
    return 0;
}

/* 丢掉 UART4 里没读走的旧回复(平时每条运动指令驱动器都会回一帧，STM32 不读，所以里面总有旧字节) */
static void Mot_Flush(void)
{
    uint8_t b;
    uint8_t n = 0;

    while (n < 64 && Mot_RxByte(&b))
    {
        n++;
    }
}

/* 发 [addr func 6B]，等回复 [addr func 数据… 6B](共 len 字节)。收到返回 1，整帧放进 out */
static int Mot_Query(uint8_t addr, uint8_t func, uint8_t *out, uint8_t len)
{
    uint8_t tx[3];
    uint8_t b;
    uint8_t i;
    uint32_t t0;

    Mot_Flush();
    tx[0] = addr;
    tx[1] = func;
    tx[2] = 0x6B;
    UART4_SendArray(tx, 3);

    mot_raw_n = 0;
    t0 = HAL_GetTick();
    while ((HAL_GetTick() - t0) < MOT_REPLY_MS)
    {
        if (!Mot_RxByte(&b))
        {
            continue;
        }
        if (mot_raw_n < sizeof(mot_raw))
        {
            mot_raw[mot_raw_n++] = b;
        }
        /* 在收到的字节里找完整的一帧(前面可能混着自己发的回声或别的字节) */
        for (i = 0; i + len <= mot_raw_n; i++)
        {
            if (mot_raw[i] == addr && mot_raw[i + 1] == func && mot_raw[i + len - 1] == 0x6B)
            {
                memcpy(out, &mot_raw[i], len);
                return 1;
            }
        }
    }
    return 0;
}

/* 给别的文件用(arm.c 读升降电机的编码器)：发 [addr func 6B]，收 len 字节的回复。收到返回 1 */
int Car_Motor_Query(uint8_t addr, uint8_t func, uint8_t *out, uint8_t len)
{
    HAL_Delay(2);
    return Mot_Query(addr, func, out, len);
}

/* MOT?：每个驱动器打印一行，例如
 *   MOT 1 V=12.31 EN=1 ARR=1 STALL=0 PROT=0
 *   MOT 2 NOREPLY
 *   MOT 3 NOREPLY RX=01 00 EE 6B        (收到了字节但格式对不上，原样打印) */
void Car_Motor_Report(void)
{
    uint8_t a;
    uint8_t r[8];
    char m[72];
    char vb[24];
    int okv, okf, n, j;
    long mv = 0;

    HAL_Delay(5);                                      /* 等上一条指令的回复收完 */
    for (a = 1; a <= 5; a++)
    {
        okv = Mot_Query(a, 0x24, r, 5);
        if (okv)
        {
            mv = (long)(((uint16_t)r[2] << 8) | r[3]);
        }
        HAL_Delay(2);
        okf = Mot_Query(a, 0x3A, r, 4);

        if (!okv && !okf)
        {
            n = snprintf(m, sizeof(m), "MOT %d NOREPLY", (int)a);
            if (mot_raw_n > 0)
            {
                n += snprintf(m + n, sizeof(m) - (size_t)n, " RX=");
                for (j = 0; j < (int)mot_raw_n && n < (int)sizeof(m) - 6; j++)
                {
                    n += snprintf(m + n, sizeof(m) - (size_t)n, "%02X ", mot_raw[j]);
                }
            }
            snprintf(m + n, sizeof(m) - (size_t)n, "\r\n");
        }
        else
        {
            if (okv)
            {
                snprintf(vb, sizeof(vb), "%ld.%02ld", mv / 1000, (mv % 1000) / 10);
            }
            else
            {
                snprintf(vb, sizeof(vb), "?");
            }
            if (okf)
            {
                snprintf(m, sizeof(m), "MOT %d V=%s EN=%d ARR=%d STALL=%d PROT=%d\r\n", (int)a, vb,
                         (r[2] & 0x01) ? 1 : 0, (r[2] & 0x02) ? 1 : 0, (r[2] & 0x04) ? 1 : 0, (r[2] & 0x08) ? 1 : 0);
            }
            else
            {
                snprintf(m, sizeof(m), "MOT %d V=%s FLAG=?\r\n", (int)a, vb);
            }
        }
        Link_Say(m);
        HAL_Delay(2);
    }
}

/* MOT EN：1~5 号驱动器解除堵转保护、使能。
 * 驱动器触发堵转保护后会一直松轴不动；驱动器是电池单独供电的话，只重启/重新烧录 STM32 清不掉，
 * 所以开机和每次 HOME(一键流程开始时)都自动做一次。没触发保护时发了也没影响。 */
void Car_Motor_Enable(void)
{
    uint8_t a;

    for (a = 1; a <= 5; a++)
    {
        Emm_V5_Reset_Clog_Pro(a);
        HAL_Delay(3);
        Emm_V5_En_Control(a, true, false);
        HAL_Delay(3);
    }
}
