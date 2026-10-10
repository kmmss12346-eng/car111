/* arm.c  机械臂总控 v3
 *
 * 硬件对应：
 *   升降      Emm 步进电机 ID5(和轮子 1~4 号同一路 UART4，皮带升降)
 *   前后伸缩  飞特总线舵机 ID2      (main.c 里的 Servo_Move / Servo_Start，限位在那里)
 *   整臂旋转  飞特总线舵机 ID1      (同上)。哪个舵机管伸缩由参数 AEXT 决定(默认 2 = ID2 伸缩)
 *   夹爪      PA2  TIM2 通道 3      (main.c 里的 Claw_Set；张开/夹紧的脉宽是参数 CLWO / CLWC)
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
extern void Servo_Hold(uint8_t id, float angle_deg);
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
static float g_lmax   = 105.0f;    /* LFMAX  最高点：离零点(最低点)往上 105mm。写死：SET 也不能超过 105 */
static float g_ldir   = 0.0f;      /* LFDIR  升降方向：0 或 1。LIFT 数字变大时走反了就改这个 */
static float g_lspr   = 3200.0f;   /* LFSPR  电机每圈脉冲数，只用来算要等多久 */
static float g_lmrg   = 200.0f;    /* LFMRG  升降走完后多等多少毫秒(加减速余量) */
static float g_lhmd   = 2.0f;      /* LFHMD  LIFT HOME 用的 Emm 回零模式(0~3，见张大头手册) */
static float g_lhtm   = 8000.0f;   /* LFHTM  LIFT HOME 最多等多少毫秒 */

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
static float g_aspd   = 90.0f;     /* ASPD   ID1/ID2 大动作转速(度/秒) */
static float g_aspdf  = 40.0f;     /* ASPDF  ID1/ID2 微调转速(度/秒) */
static float g_atolc  = 1.0f;      /* ATOLC  大动作离目标多少度以内算到位 */
static float g_atol   = 0.3f;      /* ATOL   微调离目标多少度以内算到位 */
static float g_amaxd  = 60.0f;     /* AMAXD  AD 指令一次最多转多少度(防止发错数) */
static float g_para   = 1.0f;      /* PARA   1 = ID1、ID2 一起转(快)；0 = 一个一个转 */

static float g_clwait = 400.0f;    /* CLWAIT 夹爪动作后等多久(毫秒) */
static float g_ttwait = 800.0f;    /* TTWAIT 转盘转到新位置要等多久(毫秒) */
static float g_armok  = 0.0f;      /* ARMOK  1 = 姿态都标定好了，允许夹放流程 */
static float g_scrmode = 1.0f;     /* SCRMODE 串口屏：1 = 程序自己画字(屏工程里只要两个字库，不用放控件)；0 = 写控件 t0.txt=… */
static float g_clwo   = 2090.0f;   /* CLWO   夹爪张开的脉宽(微秒) */
static float g_clwc   = 2910.0f;   /* CLWC   夹爪夹紧的脉宽(微秒)。太紧了舵机一直使劲会发烫：调到刚好夹稳 */
static float g_tt1    = 2608.0f;   /* TT1    转盘 1 号位的脉宽(微秒) */
static float g_tt2    = 1708.0f;   /* TT2    转盘 2 号位 */
static float g_tt3    = 808.0f;    /* TT3    转盘 3 号位 */
static float g_aext   = 2.0f;      /* AEXT   哪个舵机管前后伸缩：2 = ID2 伸缩、ID1 旋转；1 = ID1 伸缩、ID2 旋转 */
static float g_spow   = 30000.0f;  /* SPOW   ID1/ID2 转动时允许的最大功率(mW)，30000 = 当时 set SPOW 30000 的值。发烫就减小 */
static float g_s1comp = 5.0f;      /* S1COMP ID1 补偿(度)：手臂有摩擦，舵机总停在离目标差这么多的地方，就往前多给这么多，到了目标马上停 */
static float g_s2comp = 0.0f;      /* S2COMP ID2 补偿(度)，0 = 不补 */

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
    { "LFMAX",  &g_lmax,    10.0f,   105.0f },   /* 最高点写死 105mm，配置里存的更大的值会被拒绝 */
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
    { "A1G",    &g_a1g,     232.0f,  440.0f },
    { "A1D",    &g_a1d,     232.0f,  440.0f },
    { "A1H",    &g_a1h,     232.0f,  440.0f },
    { "A1P",    &g_a1p,     232.0f,  440.0f },
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
    { "SCRMODE", &g_scrmode, 0.0f,   1.0f },
    { "CLWO",   &g_clwo,    1700.0f, 2910.0f },  /* 范围 = main.c 里夹爪的限位 CLAW_MIN_US ~ CLAW_MAX_US */
    { "CLWC",   &g_clwc,    1700.0f, 2910.0f },
    { "TT1",    &g_tt1,     500.0f,  2608.0f },  /* 范围 = main.c 里转盘的限位 TURNTABLE_MIN_US ~ TURNTABLE_MAX_US */
    { "TT2",    &g_tt2,     500.0f,  2608.0f },
    { "TT3",    &g_tt3,     500.0f,  2608.0f },
    { "AEXT",   &g_aext,    1.0f,    2.0f },
    { "SPOW",   &g_spow,    1000.0f, 30000.0f },
    { "S1COMP", &g_s1comp,  0.0f,    15.0f },
    { "S2COMP", &g_s2comp,  0.0f,    15.0f },
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

/* 下面"编码器"那一节里的函数，Lift_Goto 要先用到 */
static void Pos_Save(float z);
static int  Lift_EncPos(float near_mm, float *z, uint16_t *raw);

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
    HAL_Delay(5);                                  /* 两条 Emm 指令之间要留空隙，连着发驱动器会当成一帧坏数据丢掉 */
    Emm_V5_Pos_Control(LIFT_ADDR, dir, (uint16_t)g_lrpm, (uint8_t)g_lacc, pulses, 0, false);

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
    Pos_Save(mm);                                  /* 记进 Flash：下次上电从这里找 */
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
    float rev;

    if (!lift_cal_ok || !Lift_ReadEnc(&e))
    {
        return 0;
    }
    *raw = e;
    *z = near_mm + Enc_Wrap((float)e - (lift_enc0 + lift_cpm * near_mm), lift_cpr) / lift_cpm;
    /* 算出来的在行程外面(比最低点还低、比最高点还高)，那一定差了整圈：挪回 0~LFMAX 里面 */
    rev = lift_cpr / Absf(lift_cpm);
    while (*z < -3.0f && *z + rev <= g_lmax + 3.0f)
    {
        *z += rev;
    }
    while (*z > g_lmax + 3.0f && *z - rev >= -3.0f)
    {
        *z -= rev;
    }
    return 1;
}

static void Cal_Load(void)
{
    const volatile uint32_t *f = (const volatile uint32_t *)ARM_CAL_ADDR;
    uint32_t w[6];
    float e0, c, cpr;
    int i;

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
static const char *Lift_Cal(float z0)
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
    lift_known = 1;
    lift_mm = z0;
    for (j = 0; j < 2; j++)                        /* 两种值都读：走 10mm 后哪个变化对得上就用哪个 */
    {
        ga[j] = (uint8_t)Lift_ReadSrc(srcs[j], &a[j]);
    }
    if (!ga[0] && !ga[1])
    {
        return "ERR NOENC";                        /* 读不到：驱动器 TX 没接到 PC11，或 5 号驱动器没回复 */
    }
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
    for (k = 0; k < 20 && !got; k++)               /* 驱动器上电要一会儿才能回复，最多等约 3 秒 */
    {
        got = Lift_EncPos(near_mm, &z, &e);
        if (!got)
        {
            HAL_Delay(100);
        }
    }
    if (!got)
    {
        Say("LIFTBOOT NOENC (assume 60)\r\n");
        return;
    }
    lift_mm = z;
    lift_boot_how = pos_last_ok ? "POS" : "ENC";
    FmtF(s1, (int)sizeof(s1), z);
    FmtF(s2, (int)sizeof(s2), near_mm);
    snprintf(m, sizeof(m), "LIFTBOOT %s (last %s) -> %ld\r\n", s1, s2, (long)LIFT_BOOT_MM);
    Say(m);
    if (Pos_Count() > POS_N / 4u * 3u)             /* 位置记录快满了：擦掉扇区，标定重新写回去 */
    {
        Cal_Save();
    }
    Lift_Goto(LIFT_BOOT_MM);
}

/* ================= ID1 / ID2 ================= */
/* 管前后伸缩的是哪个舵机(参数 AEXT)，另一个管旋转 */
static uint8_t Ext_Id(void)
{
    return (g_aext < 1.5f) ? 1 : 2;
}

/* ID1、ID2 转到 a1、a2(度)。PARA=1 时两个一起转(省时间)；否则按 order 先后：
 *   order=1：先动伸缩舵机再转(缩回时先缩回再转，免得伸着转扫到东西)
 *   order=2：先转再动伸缩舵机(伸出时先转好再伸)
 * tol：到位误差(度)，speed：转速(度/秒) */
static void Servos_To(float a1, float a2, uint8_t order, float tol, float speed)
{
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
        if (Servo_ReadAngle(1, &a) && Absf(a - a1) <= tol)  { ok1 = 1; Servo_Hold(1, a); }  else  t1 = Servo_Start(1, &a1, speed);
        if (Servo_ReadAngle(2, &a) && Absf(a - a2) <= tol)  { ok2 = 1; Servo_Hold(2, a); }  else  t2 = Servo_Start(2, &a2, speed);
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

        Servo_Move(first, (first == 1) ? a1 : a2, speed, tol, SERVO_EXTRA_MS);
        Servo_Move(second, (second == 1) ? a1 : a2, speed, tol, SERVO_EXTRA_MS);
    }
}

/* ================= 夹爪 / 转盘 ================= */
static uint8_t  tt_slot     = 1;
static uint32_t tt_ready_at = 0;

/* 夹爪张开 / 夹紧(脉宽是参数 CLWO / CLWC) */
static void Claw_O(void)
{
    Claw_Set((uint32_t)g_clwo);
}

static void Claw_C(void)
{
    Claw_Set((uint32_t)g_clwc);
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

    if (open_claw)  Claw_O();
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
    Claw_O();                                      /* 确保下降时爪子是张开的(下降要一会儿，爪子这时候正好张开) */
    GO(Lift_Goto(z));                              /* 下降 */
    Claw_C();      WAIT(g_clwait);                 /* 夹紧 */
    GO(Lift_Goto(g_zhi));                          /* 抬起 */
    SERVOS(g_a1d, g_a2r, 1, g_atolc, g_aspd);      /* 缩回，转到转盘上方 */
    GO(Lift_Goto(g_zdrop));                        /* 下降 */
    if (!TT_WaitReady()) return NULL;              /* 转盘到位了才松手 */
    Claw_O();      WAIT(g_clwait);                 /* 松开：物料落进转盘 */
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
    Claw_O();
    GO(Lift_Goto(g_zhi));
    SERVOS(g_a1d, g_a2r, 1, g_atolc, g_aspd);
    if (!TT_WaitReady()) return NULL;
    GO(Lift_Goto(g_zdrop));
    Claw_C();      WAIT(g_clwait);                 /* 夹起 */
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
    Claw_O();      WAIT(g_clwait);
    GO(Lift_Goto(g_zhi));
    if (Ext_Id() == 2)                             /* 只把伸缩舵机缩回，不转 */
    {
        Servo_Move(2, g_a2r, g_aspd, g_atolc, SERVO_EXTRA_MS);
    }
    else
    {
        Servo_Move(1, g_a1h, g_aspd, g_atolc, SERVO_EXTRA_MS);   /* ID1 管伸缩：缩到收起时的位置 */
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
    SERVOS(g_a1h, g_a2r, 1, g_atolc, g_aspd);
    return NULL;
}

/* 停车待机：先像 STOW 那样收臂(升到最高 -> 缩回、ID1 转到待命角度)，再把升降降到 LIFT_BOOT_MM(60mm)。
 * 停在 60，下次开机不管有没有标定、读不读得到编码器，认出来的高度都对(见 Lift_Boot)。
 * ARMOK=0(姿态还没标定)时不动舵机，只降升降。升降位置不知道(急停打断过)就不动，回 ERR NOZERO */
static const char *Seq_Park(void)
{
    if (!lift_known)           return "ERR NOZERO";

    if (g_armok > 0.5f)
    {
        GO(Lift_Goto(g_zhi));
        SERVOS(g_a1h, g_a2r, 1, g_atolc, g_aspd);
    }
    GO(Lift_Goto(LIFT_BOOT_MM));
    return NULL;
}

/* ================= 串口屏 ================= */
/* 淘晶驰(TJC)串口屏，指令后面跟三个 0xFF。两种用法(参数 SCRMODE)：
 *  1(默认) 程序自己画字：用 xstr 指令把字画在固定位置。屏的工程里不用放任何控件，只要：
 *          横屏 480x320、字库 0 = 小字(20~24 像素高)、字库 1 = 大字(80 像素高，赛规字高 ≥12mm)，都只要 ASCII
 *          (ASCII 字宽是字高的一半：大字一个 40 像素宽，小字一个 10~12 像素宽)
 *  0       写控件：屏的工程里要放好名叫 t0~t7 的文本控件，发 t0.txt="文字"
 * 两种用法程序里都还是用 t0~t7 这几个名字，下面这张表就是"每个名字画在屏上哪里"(480x320 横屏)：
 *   ┌────────────────┬──────┐
 *   │ t0 任务码前半   │t5 一批│  大字行 0  y=0    t0 带后面的 +：156+123+ 共 8 个大字 = 320 像素宽
 *   │ 156+123+       │t6 二批│
 *   ├──────────────┬─┴──────┤
 *   │ t7 任务码后半 │ t1 阶段 │  大字行 1  y=80   t7：516+231 共 7 个大字 = 280 像素宽
 *   │ 516+231      │ t4 提示 │
 *   ├──────────────┴────────┤
 *   │ t2 抓取数              │  大字行 2  y=160
 *   ├───────────────────────┤
 *   │ t3 放置数              │  大字行 3  y=240
 *   └───────────────────────┘
 * (以前 t0 只有 288 宽、放不下 8 个大字，右上角是 t1/t4。t1 阶段、t4 提示的字比较长(最多 18 个小字)，
 *  所以和字短的 t5/t6(B1 R1 G2 B3，11 个小字)换了位置) */
#define SCR_FONT_SMALL 0
#define SCR_FONT_BIG   1
typedef struct { const char *obj; int16_t x, y, w, h; uint8_t font; uint16_t color; } ScrBox;
static const ScrBox scr_box[] =
{
    { "t0",   0,   0, 322, 80, SCR_FONT_BIG,   65535u },   /* 白 */
    { "t7",   0,  80, 288, 80, SCR_FONT_BIG,   65535u },
    { "t2",   0, 160, 480, 80, SCR_FONT_BIG,   2016u  },   /* 绿 */
    { "t3",   0, 240, 480, 80, SCR_FONT_BIG,   65504u },   /* 黄 */
    { "t5", 326,   2, 154, 38, SCR_FONT_SMALL, 65535u },
    { "t6", 326,  42, 154, 38, SCR_FONT_SMALL, 65535u },
    { "t1", 292,  82, 188, 38, SCR_FONT_SMALL, 65535u },
    { "t4", 292, 122, 188, 38, SCR_FONT_SMALL, 63488u },   /* 红 */
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
                /* xstr x,y,w,h,字库,字色,背景色,水平居中(0左 1中),垂直居中,背景方式(1=纯色，顺便擦掉旧字),"文字" */
                Screen_Send(buf, snprintf(buf, sizeof(buf), "xstr %d,%d,%d,%d,%d,%u,0,%d,1,1,\"%s\"",
                                          b->x, b->y, b->w, b->h, b->font, b->color,
                                          (b->font == SCR_FONT_BIG) ? 1 : 0, text));
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
 * 开机屏上先显示选区页：左半边大按钮 "1"、右半边大按钮 "2"(启停区 1 / 2)。
 * 选好进准备页：上面 "ZONE n"，中间一行状态(树莓派用 ZONE MSG 写，例如 SCAN、PLAN OK)，
 * 下面大按钮 START，右上角小按钮 BACK(回选区页重新选)。
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
    if (ui == ZUI_READY)
    {
        if (y >= ZR_START_Y)                   return ZBTN_START;
        if (x >= ZR_BACK_X - 6 && y < 80)      return ZBTN_BACK;
    }
    return 0;
}

/* 在中断里：一次按下(ev=01)或松开(ev=00)。松开时还在按下的那个按钮里，才记成一次触摸 */
static void Zone_Touch(uint16_t x, uint16_t y, uint8_t ev)
{
    uint8_t b = Zone_Hit(zone_ui, x, y);

    if (ev == 0x01)
    {
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

/* 状态行：选区页在右上角，准备页在 "ZONE n" 下面。比赛布局时只记下来，不画 */
static void Zone_ShowMsg(void)
{
    if (zone_ui == ZUI_PICK)
    {
        Zone_Box(240, 0, 240, 48, SCR_FONT_SMALL, 65504u, 0u, zone_msg);          /* 黄字 */
    }
    else if (zone_ui != ZUI_OFF)
    {
        Zone_Box(0, 84, 480, 40, SCR_FONT_SMALL, 65504u, 0u, zone_msg);
    }
}

/* 准备页下半部分：没按 START 时是绿色 START 和右上角 BACK；按了以后 BACK 擦掉、START 换成黄色 GO */
static void Zone_ShowButtons(void)
{
    if (zone_ui == ZUI_READY)
    {
        Zone_Box(ZR_BACK_X, 8, 136, 64, SCR_FONT_SMALL, 65535u, 33808u, "BACK");                 /* 灰底白字 */
        Zone_Box(0, ZR_START_Y, 480, 320 - ZR_START_Y, SCR_FONT_BIG, 0u, 2016u, "START");      /* 绿底黑字 */
    }
    else
    {
        Zone_Box(ZR_BACK_X, 8, 136, 64, SCR_FONT_SMALL, 0u, 0u, "");
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
        snprintf(buf, sizeof(buf), "ZONE %d", (int)zone_sel);
        Zone_Box(0, 0, 320, 80, SCR_FONT_BIG, 65535u, 0u, buf);
        Zone_ShowButtons();
    }
    Zone_ShowMsg();
}

/* 换页：先换状态再画。关中断把还没处理的按下/触摸清掉，免得上一页的按钮算到这一页上。
 * 选区页、准备页的状态行清空；GO 页状态行是 "GO" */
static void Zone_Page(uint8_t ui, uint8_t sel, uint8_t xy)
{
    __disable_irq();
    zone_ui = ui;
    scr_down = 0;
    scr_tap = 0;
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

/* 开机：收得到触摸就显示选区页(选启停区 1 还是 2)；
 * USART2 接收没开起来(收不到触摸)就和以前一样：画模式清屏、t1 显示 READY(终端里还能用 ZONE 1 / ZONE 2 选) */
static void Screen_Boot(void)
{
    zone_sel = 0;
    zone_go = 0;
    zone_msg[0] = 0;
    if (scr_ok)
    {
        Zone_Page(ZUI_PICK, 0, 1);
        return;
    }
    __disable_irq();
    zone_ui = ZUI_OFF;
    scr_down = 0;
    scr_tap = 0;
    __enable_irq();
    if (g_scrmode > 0.5f)
    {
        Screen_Clear();
        Screen_Text("t1", "READY");
    }
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

    if (ch == '\r' || ch == '\n')
    {
        QR_Latch();
        qr_len = 0;
    }
    else
    {
        if (qr_len >= QR_BUF - 1)
        {
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
    qr_len = 0;
    HAL_UART_Receive_IT(&huart5, &qr_rx, 1);
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

void Arm_Poll(void)
{
    char tmp[QR_BUF];
    char newc[16];
    int n;
    uint8_t b;

    /* 屏刚重新上电(忘了 sendxy、画面也没了)：选区页还在用就整页重画 */
    if (scr_boot)
    {
        scr_boot = 0;
        Zone_Show(1);
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
    /* 不发换行的模块：60 毫秒没有新字节，就把收到的拿去解析 */
    if (qr_len != 0 && (HAL_GetTick() - qr_tick) > 60u)
    {
        __disable_irq();
        n = qr_len;
        memcpy(tmp, qr_buf, (size_t)n);
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
        char h1[12];
        char h2[8];

        qr_show = 0;
        /* 屏上显示任务码。赛规要求字高不小于 12mm，3.5 寸屏一行放不下 15 个字符，所以分两行，组之间的 + 都留着：
         * t0 = 前两组带后面的 +(如 452+321+)，t7 = 后两组(如 254+312)。控件名不一样就改这里。
         * 选区页还在显示时不写(会盖掉按钮)，码照样记着、照样告诉树莓派 */
        if (zone_ui == ZUI_OFF)
        {
            memcpy(h1, qr_code, 8);      h1[8] = 0;
            memcpy(h2, qr_code + 8, 7);  h2[7] = 0;
            Screen_Text("t0", h1);
            Screen_Text("t7", h2);
        }
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

    huart2.Init.BaudRate = SCREEN_BAUD;            /* 屏 */
    HAL_UART_Init(&huart2);
    huart5.Init.BaudRate = QR_BAUD;                /* 扫码模块 */
    HAL_UART_Init(&huart5);

    qr_len = 0;
    qr_new = 0;
    HAL_UART_Receive_IT(&huart5, &qr_rx, 1);       /* UART5 的中断在 hal_msp.c 里已经打开 */
    /* 收屏发回来的触摸坐标(选启停区)。USART2 的中断在 hal_msp.c 里也已经打开(main.c 里把优先级降到 3) */
    scr_n = 0;
    scr_ff = 0;
    scr_ok = (HAL_UART_Receive_IT(&huart2, &scr_rx, 1) == HAL_OK) ? 1 : 0;

    Servo_SetPower((uint16_t)g_spow);
    Servo_SetComp(1, g_s1comp);
    Servo_SetComp(2, g_s2comp);

    HAL_Delay(300);                                /* 等屏上电启动 */
    scr_boot = 0;                                  /* 这之前屏发来的开机帧不用管：下面马上画；这之后再来说明屏又重启了，Arm_Poll 重画 */
    Screen_Boot();                                 /* 选区页(选启停区 1 / 2) */

    /* 升降：0 = 最低点，往上为正，最高 100mm(LFMAX)。开机读编码器找到准确高度，自动走到 60mm(见 Lift_Boot) */
    Lift_Boot();
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
        if (strcmp(t[1], "O") == 0)       Claw_O();
        else if (strcmp(t[1], "C") == 0)  Claw_C();
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

    /* ---- 停车待机：收臂 + 升降停到 60mm(ARMOK=0 时只降升降) ---- */
    if (strcmp(v, "PARK") == 0 && n == 1)
    {
        r = Seq_Park();
        if (r)  FAIL(r);
        return 1;
    }

    /* ---- 二维码 ---- */
    if (strcmp(v, "QR?") == 0 && n == 1)
    {
        char m[32];

        QR_Fetch();                                /* 中断里刚锁存、主循环还没取走的码也算 */
        if (qr_valid)  snprintf(m, sizeof(m), "QR %s\r\n", qr_code);
        else           snprintf(m, sizeof(m), "QR NONE\r\n");
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
