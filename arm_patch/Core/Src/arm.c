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
static float g_lmax   = 100.0f;    /* LFMAX  最高点：离零点(最低点)往上 100mm。写死：SET 也不能超过 100 */
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
static float g_scrmode = 1.0f;     /* SCRMODE 串口屏：1 = 程序自己画字(屏工程里只要两个字库，不用放控件)；0 = 写控件 t0.txt=… */

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
    { "LFMAX",  &g_lmax,    10.0f,   100.0f },   /* 最高点写死 100mm，配置里存的更大的值会被拒绝 */
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
    { "A1G",    &g_a1g,     232.0f,  417.6f },
    { "A1D",    &g_a1d,     232.0f,  417.6f },
    { "A1H",    &g_a1h,     232.0f,  417.6f },
    { "A1P",    &g_a1p,     232.0f,  417.6f },
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
/* 淘晶驰(TJC)串口屏，指令后面跟三个 0xFF。两种用法(参数 SCRMODE)：
 *  1(默认) 程序自己画字：用 xstr 指令把字画在固定位置。屏的工程里不用放任何控件，只要：
 *          横屏 480x320、字库 0 = 小字(约 24 像素高)、字库 1 = 大字(80 像素高，赛规字高 ≥12mm)，都只要 ASCII
 *  0       写控件：屏的工程里要放好名叫 t0~t7 的文本控件，发 t0.txt="文字"
 * 两种用法程序里都还是用 t0~t7 这几个名字，下面这张表就是"每个名字画在屏上哪里"(480x320 横屏)：
 *   ┌──────────────┬────────┐
 *   │ t0 任务码前半 │ t1 阶段 │  大字行 0  y=0
 *   │              │ t4 提示 │
 *   ├──────────────┤ t5 一批 │
 *   │ t7 任务码后半 │ t6 二批 │  大字行 1  y=80
 *   ├──────────────┴────────┤
 *   │ t2 抓取数              │  大字行 2  y=160
 *   ├───────────────────────┤
 *   │ t3 放置数              │  大字行 3  y=240
 *   └───────────────────────┘ */
#define SCR_FONT_SMALL 0
#define SCR_FONT_BIG   1
typedef struct { const char *obj; int16_t x, y, w, h; uint8_t font; uint16_t color; } ScrBox;
static const ScrBox scr_box[] =
{
    { "t0",   0,   0, 288, 80, SCR_FONT_BIG,   65535u },   /* 白 */
    { "t7",   0,  80, 288, 80, SCR_FONT_BIG,   65535u },
    { "t2",   0, 160, 480, 80, SCR_FONT_BIG,   2016u  },   /* 绿 */
    { "t3",   0, 240, 480, 80, SCR_FONT_BIG,   65504u },   /* 黄 */
    { "t1", 292,   2, 188, 38, SCR_FONT_SMALL, 65535u },
    { "t4", 292,  42, 188, 38, SCR_FONT_SMALL, 63488u },   /* 红 */
    { "t5", 292,  82, 188, 38, SCR_FONT_SMALL, 65535u },
    { "t6", 292, 122, 188, 38, SCR_FONT_SMALL, 65535u },
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

/* 画模式开机先清屏(黑底)，显示 READY */
static void Screen_Boot(void)
{
    char buf[16];

    if (g_scrmode > 0.5f)
    {
        Screen_Send(buf, snprintf(buf, sizeof(buf), "cls 0"));
        Screen_Text("t1", "READY");
    }
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

    HAL_Delay(300);                                /* 等屏上电启动 */
    Screen_Boot();

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
