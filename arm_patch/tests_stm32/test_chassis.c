/* chassis.c 主机测试(2026-10-07)：转弯卡住检测(ERR STALL) + 驱动器诊断(MOT? / MOT EN)。
 * 用一辆"假车"代替真车：轮速指令 -> 电机按加速度跟上 -> 车身转动有延迟 -> 陀螺仪读数。
 * 可以让轮子完全转不动(stall)、只转一点(weak)、反应很慢(slow)，看 chassis.c 的反应对不对。
 * 运行：bash run.sh */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "chassis.h"
#include "Emm_V5.h"
#include "hwt101.h"

/* ---------------- 假硬件 ---------------- */
static USART_TypeDef uart4_regs = { 0xFFFFFFFFu, 0 };
static USART_TypeDef uart3_regs = { 0xFFFFFFFFu, 0 };
UART_HandleTypeDef huart4 = { &uart4_regs };
UART_HandleTypeDef huart3 = { &uart3_regs };
float yaw_target = 0.0f;

static uint32_t now_ms = 0;
static uint32_t tick_calls = 0;

/* 假车 */
static double yaw = 0.0;          /* 度，连续 */
static double rate = 0.0;         /* 车身转速 度/秒 */
static double w_cmd[4];           /* 已生效的轮速指令(带符号 转/分，>0 该轮往前) */
static double w_pend[4];          /* 等同步指令的轮速 */
static int    pend_set[4];
static double w_act[4];           /* 电机实际转速 */
static double pos_left[4];        /* 位置模式剩余脉冲 */
static double pos_vel[4];
static double eff = 1.0;          /* 轮子转动有多少变成车身转动：1 正常，0 完全转不动 */
static double acc_rpm_s = 357.0;  /* 电机加速度(转/分/秒)：Emm acc=200 时约 357 */
static double tau_body = 0.08;    /* 车身反应延迟(秒) */
static double glitch_at = -1.0;   /* 某个时刻陀螺仪读错一帧 */
static long   block_at = -1;      /* 从这个时刻(毫秒)起轮子转不动(转到一半被挡住) */
static int    fwd_tab[4] = { 0, 1, 1, 0 };
static int    pos_cmds = 0;       /* 发了多少条位置模式指令(旧的后备转向方法) */

/* 假驱动器回复 */
static int    drv_reply[6] = { 0, 1, 1, 1, 1, 1 };
static int    drv_mv[6]    = { 0, 12310, 12290, 12305, 12300, 12280 };
static int    drv_flag[6]  = { 0, 0x03, 0x03, 0x03, 0x03, 0x03 };
static int    echo = 0;           /* 1 = RX 能收到自己发出去的字节(有的接法会这样) */
static uint8_t rxq[256];
static int    rx_head = 0, rx_tail = 0, rx_presented = 0;
static int    clog_cnt[6], en_cnt[6];

static char   out3[4096];         /* USART3 打印出来的东西 */
static int    out3_n = 0;

static void rx_push(uint8_t b) { rxq[rx_tail++ & 255] = b; }

uint32_t fake_uart4_poll(void)
{
    if (rx_presented)                 /* 上次放进 DR 的字节已经被读走 */
    {
        rx_head++;
        rx_presented = 0;
    }
    if (rx_head != rx_tail)
    {
        uart4_regs.DR = rxq[rx_head & 255];
        rx_presented = 1;
        return 0x20u;
    }
    return 0u;
}

static void step_1ms(void)
{
    double dt = 0.001, target_rate;
    int i;

    for (i = 0; i < 4; i++)
    {
        double want = w_cmd[i];
        if (pos_left[i] > 0.0)
        {
            want = pos_vel[i];
            pos_left[i] -= fabs(pos_vel[i]) * 3200.0 / 60.0 * dt;
            if (pos_left[i] <= 0.0) { pos_left[i] = 0.0; w_cmd[i] = 0.0; }
        }
        if (w_act[i] < want) { w_act[i] += acc_rpm_s * dt; if (w_act[i] > want) w_act[i] = want; }
        else                 { w_act[i] -= acc_rpm_s * dt; if (w_act[i] < want) w_act[i] = want; }
    }
    /* 和 chassis.c 一样：s0 - s1 + s2 - s3 的平均 = 车身转速对应的轮速；1 转/分 ≈ 3200/60/25.4 度/秒 */
    target_rate = eff * (w_act[0] - w_act[1] + w_act[2] - w_act[3]) / 4.0 * (3200.0 / 60.0 / 25.4);
    if (block_at >= 0 && (long)now_ms >= block_at)
    {
        eff = 0.0;
    }
    rate += (target_rate - rate) * dt / (tau_body + dt);
    yaw += rate * dt;
    now_ms++;
}

uint32_t HAL_GetTick(void)
{
    if (++tick_calls % 64u == 0u)     /* 忙等的循环里只调 HAL_GetTick：让时间也能走 */
    {
        step_1ms();
    }
    return now_ms;
}

void HAL_Delay(uint32_t ms)
{
    uint32_t i;
    for (i = 0; i < ms + 1u; i++)     /* HAL_Delay 实际会多等 1 毫秒 */
    {
        step_1ms();
    }
}

int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *p, uint16_t n, uint32_t t)
{
    (void)t;
    if (h == &huart3 && out3_n + n < (int)sizeof(out3) - 1)
    {
        memcpy(out3 + out3_n, p, n);
        out3_n += n;
        out3[out3_n] = 0;
    }
    return 0;
}

static double wrap180(double d)
{
    while (d > 180.0) d -= 360.0;
    while (d < -180.0) d += 360.0;
    return d;
}

float HWT101_GetYaw(void)
{
    double y = yaw;
    if (glitch_at >= 0.0 && fabs(now_ms / 1000.0 - glitch_at) < 0.006)
    {
        y += 25.0;                    /* 一帧读错 */
    }
    return (float)wrap180(y);
}
float HWT101_GetYawContinuous(void) { return (float)yaw; }
uint8_t HWT101_IsFresh(uint32_t timeout_ms) { (void)timeout_ms; return 1; }
uint8_t HWT101_IsReady(void) { return 1; }

static double signed_rpm(int i, uint8_t dir, uint16_t vel)
{
    return (dir == fwd_tab[i]) ? (double)vel : -(double)vel;
}

void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
    (void)acc;
    if (addr < 1 || addr > 4) return;
    if (snF) { w_pend[addr - 1] = signed_rpm(addr - 1, dir, vel); pend_set[addr - 1] = 1; }
    else     { w_cmd[addr - 1]  = signed_rpm(addr - 1, dir, vel); pos_left[addr - 1] = 0.0; }
}

void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
    (void)acc; (void)raF; (void)snF;
    if (addr < 1 || addr > 4) return;
    pos_cmds++;
    pos_vel[addr - 1] = signed_rpm(addr - 1, dir, vel);
    pos_left[addr - 1] = (double)clk;
}

void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
    if (addr < 1 || addr > 4) return;
    if (snF) { w_pend[addr - 1] = 0.0; pend_set[addr - 1] = 1; }
    else     { w_cmd[addr - 1] = 0.0; }
    pos_left[addr - 1] = 0.0;
}

void Emm_V5_Synchronous_motion(uint8_t addr)
{
    int i;
    (void)addr;
    for (i = 0; i < 4; i++)
    {
        if (pend_set[i]) { w_cmd[i] = w_pend[i]; pend_set[i] = 0; pos_left[i] = 0.0; }
    }
}

void Emm_V5_Reset_Clog_Pro(uint8_t addr) { if (addr <= 5) clog_cnt[addr]++; }
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF) { (void)snF; if (addr <= 5 && state) en_cnt[addr]++; }
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr) { (void)addr; }

/* 读参数指令 [addr func 6B]：按假驱动器的设定回复 */
void UART4_SendArray(uint8_t *a, uint16_t n)
{
    uint8_t addr, func;
    int i;

    if (echo)
    {
        for (i = 0; i < n; i++) rx_push(a[i]);
    }
    if (n != 3 || a[2] != 0x6B) return;
    addr = a[0];
    func = a[1];
    if (addr < 1 || addr > 5 || !drv_reply[addr]) return;
    if (func == 0x24)
    {
        rx_push(addr); rx_push(0x24);
        rx_push((uint8_t)(drv_mv[addr] >> 8)); rx_push((uint8_t)drv_mv[addr]); rx_push(0x6B);
    }
    else if (func == 0x3A)
    {
        rx_push(addr); rx_push(0x3A); rx_push((uint8_t)drv_flag[addr]); rx_push(0x6B);
    }
}
void UART4_SendByte(uint8_t b) { UART4_SendArray(&b, 1); }

/* ---------------- 测试 ---------------- */
static int fails = 0, checks = 0;
#define CHECK(c, ...) do { checks++; if (!(c)) { fails++; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

static void reset_car(void)
{
    int i;
    yaw = 0.0; rate = 0.0;
    for (i = 0; i < 4; i++) { w_cmd[i] = w_pend[i] = w_act[i] = pos_left[i] = pos_vel[i] = 0.0; pend_set[i] = 0; }
    eff = 1.0; acc_rpm_s = 357.0; tau_body = 0.08; glitch_at = -1.0; block_at = -1;
    yaw_target = 0.0f;
    car_stalled = 0; car_abort = 0; car_last_err = 0.0f;
    pos_cmds = 0;
}

static uint32_t turn(float deg)
{
    uint32_t t0 = now_ms;
    car_stalled = 0;
    Car_TurnBy_Center(deg);
    return now_ms - t0;
}

int main(void)
{
    uint32_t t;
    int i;

    printf("== 正常车：R -60 / R 90 / R 180 应该都转到，不能误判卡住\n");
    reset_car();
    t = turn(-60.0f);
    CHECK(!car_stalled, "R -60 误判卡住");
    CHECK(fabs(wrap180(yaw + 60.0)) < 1.5, "R -60 没转到：yaw=%.2f", yaw);
    CHECK(t < 2500, "R -60 用时太长 %u ms", (unsigned)t);
    CHECK(fabs(yaw_target + 60.0f) < 0.01f, "R -60 后 yaw_target 应该是 -60，实际 %.2f", yaw_target);
    printf("   R -60：%u ms，车头 %.2f°\n", (unsigned)t, yaw);
    t = turn(90.0f);
    CHECK(!car_stalled && fabs(wrap180(yaw - 30.0)) < 1.5, "R 90 没转到/误判：yaw=%.2f stalled=%d", yaw, car_stalled);
    printf("   R 90：%u ms，车头 %.2f°\n", (unsigned)t, yaw);
    t = turn(180.0f);
    CHECK(!car_stalled && fabs(wrap180(yaw - 210.0)) < 1.5, "R 180 没转到/误判：yaw=%.2f stalled=%d", yaw, car_stalled);
    printf("   R 180：%u ms，车头 %.2f°\n", (unsigned)t, wrap180(yaw));

    printf("== 反应很慢的车(电机加速度只有 1/3、车身延迟 0.3 秒)：也不能误判\n");
    reset_car();
    acc_rpm_s = 120.0; tau_body = 0.30;
    t = turn(-60.0f);
    CHECK(!car_stalled, "慢车 R -60 误判卡住");
    CHECK(fabs(wrap180(yaw + 60.0)) < 1.5, "慢车 R -60 没转到：yaw=%.2f", yaw);
    printf("   R -60：%u ms，车头 %.2f°\n", (unsigned)t, yaw);

    printf("== 陀螺仪偶尔读错一帧：不能误判，照样转到\n");
    reset_car();
    glitch_at = (now_ms + 150) / 1000.0;
    t = turn(-60.0f);
    CHECK(!car_stalled && fabs(wrap180(yaw + 60.0)) < 1.5, "读错一帧后出问题：yaw=%.2f stalled=%d", yaw, car_stalled);

    printf("== 轮子完全转不动(这次日志的情况)：R -60 要很快停下并标记卡住\n");
    reset_car();
    eff = 0.0;
    t = turn(-60.0f);
    CHECK(car_stalled, "轮子不动却没标记卡住");
    CHECK(t < 1500, "卡住后用了 %u ms 才停(原来要 6.8 秒)", (unsigned)t);
    CHECK(pos_cmds == 0, "卡住后不该再用旧方法试(发了 %d 条位置指令)", pos_cmds);
    CHECK(fabs(yaw_target) < 0.01f, "卡住时 yaw_target 不该改成目标(现在 %.2f)", yaw_target);
    CHECK(fabs(car_last_err + 60.0f) < 3.0f, "car_last_err 应该约 -60，实际 %.2f", car_last_err);
    for (i = 0; i < 4; i++) CHECK(fabs(w_cmd[i]) < 0.5, "卡住后 %d 号轮还有速度指令 %.1f", i + 1, w_cmd[i]);
    printf("   %u ms 后停下，车头误差 %.1f°\n", (unsigned)t, car_last_err);

    printf("== 轮子只能转一点(打滑/没劲，1/20)：也要标记卡住\n");
    reset_car();
    eff = 0.05;
    t = turn(-60.0f);
    CHECK(car_stalled, "只转一点却没标记卡住(yaw=%.2f)", yaw);
    printf("   %u ms 后停下，车头转了 %.1f°\n", (unsigned)t, yaw);

    printf("== 转到一半被挡住：转起来以后再卡住也要发现\n");
    reset_car();
    block_at = (long)now_ms + 300;   /* 转 0.3 秒以后轮子转不动 */
    t = turn(-90.0f);
    CHECK(car_stalled, "转到一半卡住没发现(yaw=%.2f)", yaw);
    CHECK(t < 2500, "转到一半卡住后用了 %u ms 才停", (unsigned)t);
    printf("   转了 %.1f° 后卡住，%u ms 后停下\n", yaw, (unsigned)t);

    printf("== MOT?：读到 4 个轮子 + 升降的电压和状态\n");
    reset_car();
    out3_n = 0; out3[0] = 0;
    drv_flag[3] = 0x09;              /* 3 号：使能 + 堵转保护 */
    drv_reply[5] = 0;                /* 5 号：没回复 */
    Car_Motor_Report();
    printf("%s", out3);
    CHECK(strstr(out3, "MOT 1 V=12.31 EN=1 ARR=1 STALL=0 PROT=0") != NULL, "1 号格式不对");
    CHECK(strstr(out3, "MOT 3 V=12.30 EN=1 ARR=0 STALL=0 PROT=1") != NULL, "3 号堵转保护没读出来");
    CHECK(strstr(out3, "MOT 5 NOREPLY\r\n") != NULL, "5 号应该是 NOREPLY");

    printf("== MOT?：RX 收得到自己发的字节(回声)也能读对\n");
    out3_n = 0; out3[0] = 0;
    echo = 1;
    drv_reply[5] = 1;
    Car_Motor_Report();
    CHECK(strstr(out3, "MOT 2 V=12.29 EN=1 ARR=1 STALL=0 PROT=0") != NULL, "有回声时 2 号读错：%s", out3);
    CHECK(strstr(out3, "MOT 5 V=12.28") != NULL, "有回声时 5 号读错");
    echo = 0;

    printf("== MOT?：驱动器回的东西格式对不上时，把原始字节打印出来\n");
    out3_n = 0; out3[0] = 0;
    for (i = 1; i <= 5; i++) drv_reply[i] = 0;
    rx_push(0x55); rx_push(0xAA);
    Car_Motor_Report();
    CHECK(strstr(out3, "MOT 1 NOREPLY") != NULL, "全部没回复时格式不对");
    for (i = 1; i <= 5; i++) drv_reply[i] = 1;

    printf("== MOT EN：1~5 号都解除堵转保护并使能\n");
    memset(clog_cnt, 0, sizeof(clog_cnt));
    memset(en_cnt, 0, sizeof(en_cnt));
    Car_Motor_Enable();
    for (i = 1; i <= 5; i++) CHECK(clog_cnt[i] == 1 && en_cnt[i] == 1, "%d 号没清保护/使能", i);

    printf("\n%d 项检查，%d 项失败\n", checks, fails);
    return fails ? 1 : 0;
}
