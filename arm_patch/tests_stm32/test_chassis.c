/* chassis.c 主机测试(2026-10-07)：转弯卡住检测(ERR STALL) + 驱动器诊断(MOT? / MOT EN)。
 * 用一辆"假车"代替真车：轮速指令 -> 电机按加速度跟上 -> 车身转动有延迟 -> 陀螺仪读数。
 * 可以让轮子完全转不动(stall)、只转一点(weak)、反应很慢(slow)，看 chassis.c 的反应对不对。
 * 2026-10-10(v13)：假车加了 x/y 里程(车中心，转轴在车中心后 103mm)、陀螺仪噪声(100Hz 一帧，帧内不变)、
 * 横移时车自己转(C×速度^1.5，加速/匀速用 CA、减速用 CD，和 chassis.c 的模型一样)、横移前后漂、直行天生偏转，
 * 并且记下每次同步发出去的 4 个轮速，用来查"末段来回切换""每周期差速跳变"。
 * 新测试：F 1000 / S ±300 的平稳和精度、R 90 / R -2 末段、精确模式开/关、急停、R 带小数的解析、参数表。
 * 运行：bash run.sh
 * 想看改之前的横移跳变：gcc ... -DCHASSIS_V12A 和旧的 chassis.c 一起编译(只跑不依赖新接口的那几项)。 */
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

/* v13：假车的位置、陀螺仪噪声、横移自转 */
#define K_ROT   (3200.0 / 60.0 / 25.4)    /* 轮子 1 转/分(转弯分量) → 车身 度/秒 */
#define K_FWD   (3200.0 / 60.0 / 12.84)   /* 1 转/分(前进分量) → mm/秒(和 chassis.c 的 FPPM 默认值一样) */
#define K_LAT   (3200.0 / 60.0 / 13.67)   /* 1 转/分(横移分量) → mm/秒 */
static double px = 0.0, py = 0.0;          /* 车中心位置(mm)，开始时车头朝 +x */
static double vf_b = 0.0, vl_b = 0.0, vl_w = 0.0;   /* 车身实际的前进/左移速度(mm/秒，有延迟) */
static double pivot_back = 103.0;          /* 车原地转时绕车中心后面多少 mm 的点转 */
static double ss_c[2][2];                  /* 横移自转 [左,右][0=加速/匀速 CA, 1=减速 CD]，度/秒 = C×速度^1.5 */
static double ss_drift[2];                 /* 横移时往前漂(每横移 1mm 往前多少 mm) */
static double fwd_yaw = 0.0;               /* 直行时车头天生往一边转：度/秒 每 转/分 */
static double lat_prev = 0.0;
static double noise_sd = 0.0;              /* 陀螺仪噪声(度，标准差)，每 10ms 一帧，帧内不变 */
static double gyro_frame = 0.0;
static uint32_t gyro_frame_t = 0xFFFFFFFFu;
static uint32_t rng = 12345u;
static long   abort_at = -1;               /* 到这个时刻(毫秒)就像收到 '!' 一样急停 */
static double kick_deg = 0.0;              /* 下一次停车时车头被碰歪这么多度(测走完后的车头修正) */

/* 每次同步(Synchronous_motion)后 4 个轮子的指令 */
#define LOGN 4000
static int    log_on = 0, log_n = 0;
static uint32_t log_t[LOGN];
static double log_w[LOGN][4];
static double log_y[LOGN];                 /* 那时真实的车头 */

static double urand(void)
{
    rng = rng * 1103515245u + 12345u;
    return (double)((rng >> 8) & 0xFFFFFFu) / 16777216.0;
}
static double grand(void)
{
    double a = 0.0;
    int i;
    for (i = 0; i < 12; i++) a += urand();
    return a - 6.0;
}

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
    double dt = 0.001, target_rate, rot, lat, fwd, al, c, vf_t, vl_t, yr;
    int i, di;

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
    rot = (w_act[0] - w_act[1] + w_act[2] - w_act[3]) / 4.0;
    lat = (w_act[0] - w_act[1] - w_act[2] + w_act[3]) / 4.0;     /* 左移为正 */
    fwd = (w_act[0] + w_act[1] + w_act[2] + w_act[3]) / 4.0;
    al  = fabs(lat);
    di  = (lat >= 0.0) ? 0 : 1;
    c   = (al < fabs(lat_prev) - 1e-9) ? ss_c[di][1] : ss_c[di][0];   /* 横移在减速：CD，否则 CA */
    lat_prev = lat;
    target_rate = eff * rot * K_ROT + c * al * sqrt(al) + fwd_yaw * fwd;
    if (block_at >= 0 && (long)now_ms >= block_at)
    {
        eff = 0.0;
    }
    rate += (target_rate - rate) * dt / (tau_body + dt);
    yaw += rate * dt;

    /* 车中心的速度：轮子的前进/横移分量 + 绕车中心后面的点转带出来的横移 + 横移时往前漂 */
    vf_t = eff * fwd * K_FWD + ss_drift[di] * al * K_LAT;
    vl_t = eff * lat * K_LAT;
    vf_b += (vf_t - vf_b) * dt / (tau_body + dt);           /* 车身跟上轮子和转动一样有延迟 */
    vl_w += (vl_t - vl_w) * dt / (tau_body + dt);
    vl_b = vl_w + rate * 0.01745329 * pivot_back;           /* 绕后面的点转，车中心往左移 ω×距离 */
    yr = yaw * 0.01745329;
    px += (vf_b * cos(yr) - vl_b * sin(yr)) * dt;
    py += (vf_b * sin(yr) + vl_b * cos(yr)) * dt;
    now_ms++;
    if (abort_at >= 0 && (long)now_ms >= abort_at)
    {
        car_abort = 1;
        abort_at = -1;
    }
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
    double y;
    if (gyro_frame_t == 0xFFFFFFFFu || now_ms - gyro_frame_t >= 10u)    /* 陀螺仪 100Hz 一帧 */
    {
        gyro_frame_t = now_ms;
        gyro_frame = yaw + ((noise_sd > 0.0) ? noise_sd * grand() : 0.0);
        gyro_frame = floor(gyro_frame / 0.0055 + 0.5) * 0.0055;
    }
    y = gyro_frame;
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
    if (kick_deg != 0.0)
    {
        yaw += kick_deg;                  /* 停车时车头被碰歪(只碰一次) */
        kick_deg = 0.0;
    }
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
    if (log_on && log_n < LOGN)
    {
        log_t[log_n] = now_ms;
        log_y[log_n] = yaw;
        for (i = 0; i < 4; i++) log_w[log_n][i] = w_cmd[i];
        log_n++;
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
    px = py = vf_b = vl_b = vl_w = 0.0; lat_prev = 0.0;
    memset(ss_c, 0, sizeof(ss_c)); memset(ss_drift, 0, sizeof(ss_drift));
    fwd_yaw = 0.0; noise_sd = 0.0; gyro_frame_t = 0xFFFFFFFFu; rng = 12345u;
    abort_at = -1; kick_deg = 0.0;
    log_on = 0; log_n = 0;
}

/* 停稳以后的真实车头(度) */
static double settle_yaw(void)
{
    HAL_Delay(400);
    return yaw;
}

/* 记录里第 k 条的车身转速分量(右侧 - 左侧，转/分) */
static double log_rot(int k)
{
    return (log_w[k][0] - log_w[k][1] + log_w[k][2] - log_w[k][3]) / 4.0;
}

/* 记录里相邻两次指令差速(转弯分量)最大跳了多少 转/分 */
static double max_rot_jump(void)
{
    double m = 0.0;
    int k;
    for (k = 1; k < log_n; k++)
    {
        double d = fabs(log_rot(k) - log_rot(k - 1));
        if (d > m) m = d;
    }
    return m;
}

/* 转弯分量在 0 / 正 / 负 之间切换了几次(从 0 开始算：干净的一次转弯是 0→转→0，2 次) */
static int rot_switches(void)
{
    int k, n = 0, st = 0;
    for (k = 0; k < log_n; k++)
    {
        double r = log_rot(k);
        int s2 = (r > 0.25) ? 1 : ((r < -0.25) ? -1 : 0);
        if (s2 != st) n++;
        st = s2;
    }
    return n;
}

/* 4 个轮子现在的指令都是 0 */
static int wheels_zero(void)
{
    int i;
    for (i = 0; i < 4; i++) if (fabs(w_cmd[i]) > 0.01 || fabs(w_pend[i]) > 0.01) return 0;
    return 1;
}

/* 用户车上的横移参数(map_config 的 stm32_params) */
static void user_params(void)
{
    Car_Param_Set("SCACC", 200.0f);
    Car_Param_Set("SSACC", 140.0f);
    Car_Param_Set("SSKP", 1.5f);
    Car_Param_Set("SSKI", 0.3f);
    Car_Param_Set("SSLEAD", 0.1f);
    Car_Param_Set("SSRT", 0.0f);
    Car_Param_Set("SSCAL", -0.0153f);
    Car_Param_Set("SSCDL", -0.00719f);
    Car_Param_Set("SSCAR", 0.01626f);
    Car_Param_Set("SSCDR", 0.00552f);
    Car_Param_Set("DRL", -0.00367f);
    Car_Param_Set("DRR", -0.045f);
    ss_c[0][0] = -0.0153;  ss_c[0][1] = -0.00719;   /* 假车横移自转和标定出来的一样 */
    ss_c[1][0] = 0.01626;  ss_c[1][1] = 0.00552;
    ss_drift[0] = -0.00367; ss_drift[1] = -0.045;
}

static uint32_t turn(float deg)
{
    uint32_t t0 = now_ms;
    car_stalled = 0;
    Car_TurnBy_Center(deg);
    return now_ms - t0;
}

/* ---------------- v13 新测试 ---------------- */

/* R deg：末段不来回切、停得准、车中心基本不动 */
static void test_turn_end(float deg, double noise, double tau)
{
    uint32_t t;
    double ye, err, dc;
    int sw;

    reset_car();
    noise_sd = noise; tau_body = tau;
    log_on = 1;
    t = turn(deg);
    log_on = 0;
    ye = settle_yaw();
    err = wrap180(ye - deg);
    sw = rot_switches();
    dc = sqrt(px * px + py * py);
    printf("   R %.1f (噪声 %.2f°，车身延迟 %.2f 秒)：%u ms，误差 %.2f°，0/转 切换 %d 次，指令 %d 条，车中心移了 %.1f mm\n",
           deg, noise, tau, (unsigned)t, err, sw, log_n, dc);
    CHECK(!car_stalled, "R %.1f 误判卡住", deg);
    CHECK(fabs(err) <= 0.5, "R %.1f 停下误差 %.2f° > 0.5°", deg, err);
    CHECK(sw <= 2, "R %.1f 末段在 0 和转之间切了 %d 次(应该 ≤2：开始转一次、停一次)", deg, sw);
    CHECK(fabs(wrap180(yaw_target - deg)) < 0.01, "R %.1f 后 yaw_target 应该是 %.1f，实际 %.2f", deg, deg, yaw_target);
}

static void test_straight(void)
{
    uint32_t t0, t;
    double ye, jump;

    printf("== F 1000(150 转/分，陀螺仪有噪声、车天生往一边偏)：差速每周期跳变 ≤2 转/分，停下车头误差 <0.3°\n");
    reset_car();
    user_params();
    noise_sd = 0.05;
    fwd_yaw = 0.004;
    log_on = 1;
    t0 = now_ms;
    Car_Straight_Closed(150, 1000.0f);
    t = now_ms - t0;
    log_on = 0;
    jump = max_rot_jump();
    ye = settle_yaw();
    printf("   %u ms，走了 %.1f mm，横偏 %.1f mm，车头 %.2f°(e=%.2f)，差速最大跳 %.1f 转/分，指令 %d 条\n",
           (unsigned)t, px, py, ye, car_last_err, jump, log_n);
    CHECK(fabs(ye) < 0.3, "F 1000 停下车头误差 %.2f° ≥ 0.3°", ye);
    CHECK(jump <= 2.0, "F 1000 差速一个周期跳了 %.1f 转/分(应 ≤2)", jump);
    CHECK(fabs(px - 1000.0) < 3.0, "F 1000 走了 %.1f mm", px);
    CHECK(fabs(py) < 10.0, "F 1000 横偏 %.1f mm", py);
}

static void test_strafe(float mm)
{
    uint32_t t0, t;
    double ye, jump;

    reset_car();
    user_params();
    noise_sd = 0.05;
    log_on = 1;
    t0 = now_ms;
    Car_Strafe_Closed(140, mm);
    t = now_ms - t0;
    log_on = 0;
    jump = max_rot_jump();
    ye = settle_yaw();
    printf("   S %.0f：%u ms，横移 %.1f mm，前后 %.1f mm，车头 %.2f°(e=%.2f)，差速最大跳 %.1f 转/分，指令 %d 条\n",
           mm, (unsigned)t, py, px, ye, car_last_err, jump, log_n);
    CHECK(jump <= 2.0, "S %.0f 差速一个周期跳了 %.1f 转/分(应 ≤2)", mm, jump);
    CHECK(fabs(ye) < 1.0, "S %.0f 停下车头误差 %.2f°", mm, ye);
    CHECK(fabs(py - mm) < 0.03 * fabs(mm) + 3.0, "S %.0f 实际横移 %.1f mm", mm, py);
    CHECK(fabs(px) < 10.0, "S %.0f 前后漂了 %.1f mm", mm, px);
}

/* 记录里第 k 条的走的速度(前进分量和横移分量里大的那个，转/分；差速不算) */
static double log_move(int k)
{
    double f = fabs(log_w[k][0] + log_w[k][1] + log_w[k][2] + log_w[k][3]) / 4.0;
    double l = fabs(log_w[k][0] - log_w[k][1] - log_w[k][2] + log_w[k][3]) / 4.0;
    return (f > l) ? f : l;
}

/* 一次 F/S 里：相邻两次指令速度最多加了多少(转/分)，最后一次非 0 指令的速度 */
static void speed_profile(double *max_up, double *last_nz)
{
    int k;
    *max_up = 0.0;
    *last_nz = 0.0;
    for (k = 0; k < log_n; k++)
    {
        double m = log_move(k);
        if (m > 0.0) *last_nz = m;
        if (k > 0 && m - log_move(k - 1) > *max_up) *max_up = m - log_move(k - 1);
    }
}

/* 精确模式：F 40 用 60 转/分；停车时车头被碰歪 kick 度 */
static uint32_t prec_run(float pspd, double kick, double *up, double *last, double *ye)
{
    uint32_t t0, t;

    reset_car();
    user_params();
    Car_Param_Set("PSPD", pspd);
    log_on = 1;
    kick_deg = kick;
    t0 = now_ms;
    Car_Straight_Closed(60, 40.0f);
    t = now_ms - t0;
    log_on = 0;
    speed_profile(up, last);
    *ye = settle_yaw();
    return t;
}

static void test_precision(void)
{
    uint32_t t_on, t_off;
    double up_on, up_off, last_on, last_off, ye_on, ye_off, x_on, x_off;

    printf("== 精确模式(速度 ≤ PSPD=60)：加速更柔、最后慢慢爬、多等一会、车头偏 0.5° 以上就转回去\n");
    t_on = prec_run(60.0f, 0.0, &up_on, &last_on, &ye_on);
    x_on = px;
    t_off = prec_run(0.0f, 0.0, &up_off, &last_off, &ye_off);
    x_off = px;
    printf("   F 40 / 60 转/分：精确模式 %u ms(每次最多加 %.0f 转/分，最后 %.0f 转/分，走了 %.1f mm)；关掉 %u ms(%.0f，%.0f，%.1f mm)\n",
           (unsigned)t_on, up_on, last_on, x_on, (unsigned)t_off, up_off, last_off, x_off);
    CHECK(up_on <= 120.0 * 0.034 + 1.0, "精确模式每个周期加速 %.0f 转/分，应按 PACC=120 转/分/秒", up_on);
    CHECK(up_off > up_on, "关掉精确模式加速应该更快(%.0f vs %.0f)", up_off, up_on);
    CHECK(last_on <= 3.0 + 0.5 && last_off >= 5.5, "最后爬行速度：精确模式 %.1f(应 ≤3)，平时 %.1f(应 6)", last_on, last_off);
    CHECK(t_on > t_off + 80, "精确模式应该多等(%u vs %u ms)", (unsigned)t_on, (unsigned)t_off);
    CHECK(fabs(x_on - 40.0) < 1.0 && fabs(x_off - 40.0) < 1.0, "F 40 走了 %.1f / %.1f mm", x_on, x_off);

    t_on = prec_run(60.0f, -1.0, &up_on, &last_on, &ye_on);
    t_off = prec_run(0.0f, -1.0, &up_off, &last_off, &ye_off);
    printf("   停车时车头被碰歪 1°：精确模式最后 %.2f°(%u ms)；关掉 %.2f°(%u ms)\n", ye_on, (unsigned)t_on, ye_off, (unsigned)t_off);
    CHECK(fabs(ye_on) <= 0.5, "精确模式车头偏 1° 应该转回 0.5° 以内，实际 %.2f°", ye_on);
    CHECK(fabs(ye_off + 1.0) < 0.2, "平时(ALTOL 1.5°)偏 1° 不转，实际 %.2f°", ye_off);

    reset_car();
    user_params();
    Car_Param_Set("PSPD", 60.0f);
    log_on = 1;
    Car_Straight_Closed(61, 40.0f);
    log_on = 0;
    speed_profile(&up_on, &last_on);
    CHECK(last_on >= 5.5, "61 转/分不该用精确模式(最后 %.0f 转/分)", last_on);

    reset_car();
    user_params();
    Car_Param_Set("PSPD", 60.0f);
    log_on = 1;
    kick_deg = 1.0;
    Car_Strafe_Closed(60, -30.0f);
    log_on = 0;
    speed_profile(&up_on, &last_on);
    ye_on = settle_yaw();
    printf("   S -30 / 60 转/分(停车时车头被碰歪 1°)：每次最多加 %.0f 转/分，最后 %.0f 转/分，横移 %.1f mm，车头 %.2f°\n", up_on, last_on, py, ye_on);
    CHECK(last_on <= 3.0 + 0.5, "精确模式横移最后爬行 %.1f 转/分", last_on);
    CHECK(fabs(ye_on) <= 0.5, "精确模式横移后车头 %.2f°", ye_on);
    CHECK(fabs(py + 30.0) < 2.0, "S -30 横移了 %.1f mm", py);
    Car_Param_Set("PSPD", 60.0f);
}

/* 运动中途急停('!')：所有轮子的指令都要归零，不再动 */
static void test_abort(void)
{
    double y0;

    printf("== 急停：F / S / R 中途收到 '!'，4 个轮子马上停\n");
    reset_car();
    abort_at = (long)now_ms + 500;
    Car_Straight_Closed(150, 1000.0f);
    CHECK(car_abort == 1 && wheels_zero(), "F 急停后轮子还有指令");
    CHECK(px < 300.0, "F 急停后还走了 %.0f mm", px);
    HAL_Delay(300);
    CHECK(wheels_zero(), "F 急停后又动了");

    reset_car();
    abort_at = (long)now_ms + 500;
    Car_Strafe_Closed(140, -600.0f);
    CHECK(car_abort == 1 && wheels_zero(), "S 急停后轮子还有指令");
    CHECK(py > -250.0, "S 急停后还走了 %.0f mm", py);

    reset_car();
    abort_at = (long)now_ms + 300;
    Car_TurnBy_Center(90.0f);
    CHECK(car_abort == 1 && wheels_zero(), "R 急停后轮子还有指令");
    CHECK(fabs(yaw_target) < 0.01, "R 急停后 yaw_target 不该改(%.2f)", yaw_target);
    CHECK(pos_cmds == 0, "R 急停后不该再用旧方法转(%d 条位置指令)", pos_cmds);
    HAL_Delay(600);                       /* 假车的电机按斜坡停(真的 Stop_Now 是马上停)：等它停稳 */
    y0 = yaw;
    HAL_Delay(300);
    CHECK(fabs(yaw - y0) < 0.05 && wheels_zero(), "R 急停后又转了 %.2f°", yaw - y0);
    printf("   F/S/R 急停后都停了(R 停在 %.1f°)\n", yaw);
}

#ifndef CHASSIS_V12A
/* R 带小数：解析和以前的整数写法一样，R 0 / R 0.04 = HOME */
static void test_parse(void)
{
    char k;
    long v, sp;

    printf("== 解析 F / S / R：R 支持 1 位小数，整数写法和以前一样\n");
    CHECK(Car_Parse_Move("R 90", &k, &v, &sp) == 1 && k == 'R' && v == 900 && sp == 0, "R 90 -> %ld", v);
    CHECK(Car_Parse_Move("R -1.4", &k, &v, &sp) == 1 && v == -14, "R -1.4 -> %ld", v);
    CHECK(Car_Parse_Move("R 12.5", &k, &v, &sp) == 1 && v == 125, "R 12.5 -> %ld", v);
    CHECK(Car_Parse_Move("R 12.25", &k, &v, &sp) == 1 && v == 123, "R 12.25 -> %ld(第 2 位四舍五入)", v);
    CHECK(Car_Parse_Move("R -0.04", &k, &v, &sp) == 1 && v == 0, "R -0.04 应该是 0(HOME)，实际 %ld", v);
    CHECK(Car_Parse_Move("R 0", &k, &v, &sp) == 1 && v == 0, "R 0 -> %ld", v);
    CHECK(Car_Parse_Move("R .5", &k, &v, &sp) == 1 && v == 5, "R .5 -> %ld", v);
    CHECK(Car_Parse_Move("R +7", &k, &v, &sp) == 1 && v == 70, "R +7 -> %ld", v);
    CHECK(Car_Parse_Move("R 180.04", &k, &v, &sp) == 1 && v == 1800, "R 180.04 -> %ld(取整后 180.0，不算超范围)", v);
    CHECK(Car_Parse_Move("R 180.05", &k, &v, &sp) == 1 && v == 1801, "R 180.05 -> %ld(超范围，main.c 回 ERR RANGE)", v);
    CHECK(Car_Parse_Move("R 99999999999", &k, &v, &sp) == 1 && v > 1800, "很大的数要回 ERR RANGE(%ld)", v);
    CHECK(Car_Parse_Move("R 90 100", &k, &v, &sp) == 1 && v == 900 && sp == 100, "R 90 100 -> %ld %ld", v, sp);
    CHECK(Car_Parse_Move("R -2.0 60", &k, &v, &sp) == 1 && v == -20 && sp == 60, "R -2.0 60");
    CHECK(Car_Parse_Move("R", &k, &v, &sp) == 0, "R 没有空格不是 R 指令");
    CHECK(Car_Parse_Move("R ", &k, &v, &sp) == -1, "R 后面没有数要 ERR ARG");
    CHECK(Car_Parse_Move("R .", &k, &v, &sp) == -1, "R . 要 ERR ARG");
    CHECK(Car_Parse_Move("R -", &k, &v, &sp) == -1, "R - 要 ERR ARG");
    CHECK(Car_Parse_Move("R 1e3", &k, &v, &sp) == -1, "R 1e3 要 ERR ARG");
    CHECK(Car_Parse_Move("R 1.5x", &k, &v, &sp) == -1, "R 1.5x 要 ERR ARG");
    CHECK(Car_Parse_Move("R 90 4", &k, &v, &sp) == -1, "R 90 4(速度 <5) 要 ERR ARG");
    CHECK(Car_Parse_Move("F 300", &k, &v, &sp) == 1 && k == 'F' && v == 300 && sp == 0, "F 300");
    CHECK(Car_Parse_Move("F -300 120", &k, &v, &sp) == 1 && v == -300 && sp == 120, "F -300 120");
    CHECK(Car_Parse_Move("S 25 60", &k, &v, &sp) == 1 && k == 'S' && v == 25 && sp == 60, "S 25 60");
    CHECK(Car_Parse_Move("F 10.5", &k, &v, &sp) == -1, "F 只认整数(和以前一样)");
    CHECK(Car_Parse_Move("F 300 301", &k, &v, &sp) == -1, "F 速度 >300 要 ERR ARG");
    CHECK(Car_Parse_Move("F 300 ", &k, &v, &sp) == -1, "F 300 后面多一个空格要 ERR ARG(和以前一样)");
    CHECK(Car_Parse_Move("F 300 6x", &k, &v, &sp) == -1, "F 300 6x 要 ERR ARG");
    CHECK(Car_Parse_Move("SET SSKP 1", &k, &v, &sp) == 0, "SET 不是 S 指令");
    CHECK(Car_Parse_Move("PING", &k, &v, &sp) == 0, "PING 不是 F/S/R");
}

/* 新参数在 GET 里能看到(树莓派靠 PSPD 认出这一版)、SET 有范围 */
static void test_params(void)
{
    printf("== 参数表：新参数 SET/GET\n");
    out3_n = 0; out3[0] = 0;
    Car_Param_Dump();
    CHECK(strstr(out3, "P PSPD=60.0000") != NULL, "GET 里没有 PSPD=60");
    CHECK(strstr(out3, "P PALTOL=0.5000") != NULL, "GET 里没有 PALTOL=0.5");
    CHECK(strstr(out3, "P TCSTOP=") && strstr(out3, "P TCTOL=") && strstr(out3, "P TCMIN="), "GET 里没有 TCSTOP/TCTOL/TCMIN");
    CHECK(strstr(out3, "P SSLPF=") && strstr(out3, "P SSRL=") && strstr(out3, "P SSFFR="), "GET 里没有 SSLPF/SSRL/SSFFR");
    CHECK(strstr(out3, "P PACC=") && strstr(out3, "P PMIN=") && strstr(out3, "P PSET="), "GET 里没有 PACC/PMIN/PSET");
    CHECK(Car_Param_Set("PSPD", 0.0f) == 1 && Car_Param_Set("PSPD", 301.0f) == 2, "PSPD 范围不对");
    CHECK(Car_Param_Set("TCSTOP", 0.01f) == 2 && Car_Param_Set("TCTOL", 0.05f) == 2, "TCSTOP/TCTOL 下限不对");
    CHECK(Car_Param_Set("NOSUCH", 1.0f) == 0, "没有的名字要回 0");
    Car_Param_Set("PSPD", 60.0f);
}
#endif

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

    printf("== R 90 / R -2 / R 180 / R -60 / R -1.4：末段连续减速，不在 0 和转之间来回切，停下误差 ≤0.5°\n");
    test_turn_end(90.0f, 0.0, 0.08);
    test_turn_end(-2.0f, 0.0, 0.08);
    test_turn_end(180.0f, 0.0, 0.08);
    test_turn_end(-60.0f, 0.0, 0.08);
    test_turn_end(90.0f, 0.05, 0.08);
    test_turn_end(-2.0f, 0.05, 0.08);
    test_turn_end(180.0f, 0.05, 0.08);
    test_turn_end(-1.4f, 0.05, 0.08);
    test_turn_end(90.0f, 0.05, 0.16);         /* 车身延迟和 TCLAG(0.16) 一样的车 */
    test_turn_end(-2.0f, 0.05, 0.16);
    test_turn_end(-60.0f, 0.05, 0.16);
    test_turn_end(0.6f, 0.05, 0.16);

    test_straight();

    printf("== S ±300(用户的横移参数，140 转/分)：CA→CD 时前馈不跳，差速每周期 ≤2 转/分(改之前 4~6)\n");
    test_strafe(300.0f);
    test_strafe(-300.0f);

    test_abort();
#ifndef CHASSIS_V12A
    test_precision();
    test_parse();
    test_params();
#endif

    printf("\n%d 项检查，%d 项失败\n", checks, fails);
    return fails ? 1 : 0;
}
