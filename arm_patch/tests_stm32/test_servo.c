/* main.c 里舵机那一段(Servo_Move / Servo_StopIfStuck / Servo_Report)在电脑上测：假舵机按限定速度转，
 * 可以起步慢(lag)、中途停顿要重发才接着转(pause)、被挡住(block，挡住时电流大)。
 * run.sh 会把 main.c 里 "ID 1、ID 2 串口舵机" 那一节摘出来放进 servo_section.c 再编译 */
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
typedef struct { int dummy; } Usart_DataTypeDef;
typedef struct { int dummy; } UART_HandleTypeDef;
typedef uint8_t FSUS_STATUS;
#define FSUS_STATUS_SUCCESS 0
#define FSUS_STATUS_FAIL 1
#define FSUS_PACK_RESPONSE_MAX_SIZE 50
#define FSUS_PARAM_VOLTAGE 1
#define FSUS_PARAM_CURRENT 2
#define FSUS_PARAM_POWER 3
#define FSUS_PARAM_TEMPRATURE 4
#define FSUS_PARAM_SERVO_STATUS 5
#define FSUS_PARAM_RESPONSE_SWITCH 33
#define FSUS_PARAM_STALL_PROTECT 37
#define FSUS_PARAM_STALL_POWER_LIMIT 38
#define FSUS_PARAM_OVER_VOLT_LOW 39
#define FSUS_PARAM_OVER_VOLT_HIGH 40
#define FSUS_PARAM_OVER_TEMPERATURE 41
#define FSUS_PARAM_OVER_POWER 42
#define FSUS_PARAM_OVER_CURRENT 43
#define FSUS_PARAM_ACCEL_SWITCH 44
#define FSUS_PARAM_POWER_ON_LOCK_SWITCH 46
#define FSUS_PARAM_ANGLE_LIMIT_SWITCH 48
#define FSUS_PARAM_SOFT_START_SWITCH 49
#define FSUS_PARAM_SOFT_START_TIME 50
#define FSUS_PARAM_ANGLE_LIMIT_HIGH 51
#define FSUS_PARAM_ANGLE_LIMIT_LOW 52
static long uparam[64];                              /* 假舵机里存的用户数据 */
static int write_fail = 0;
static uint16_t last_power = 0;
Usart_DataTypeDef usart2; UART_HandleTypeDef huart3;
static uint32_t now = 0;
static char out[8192];
static double pos[3] = {0, 300, -862}, tgt[3] = {0, 300, -862}, vmax[3] = {0, 1000, 1000}, vcmd[3] = {0, 60, 60};
static double block_hi = 1e9, block_lo = -1e9;
static uint32_t cmd_at[3], lag_ms = 0;               /* 舵机收到指令后过 lag_ms 才开始动 */
static double pause_at = 1e9;                        /* ID1 转到这里就停下，要重发指令才接着转 */
static int paused = 0, pause_every = 0;              /* pause_every=1：重发以后也不动 */
static int read_fail = 0, nset = 0, stall_flag = 0;
static int blocked(void) { return (pos[1] >= block_hi && tgt[1] > block_hi) || (pos[1] <= block_lo && tgt[1] < block_lo); }
static void step(uint32_t ms) {
    int id; for (id = 1; id <= 2; id++) {
        double v = vcmd[id] < vmax[id] ? vcmd[id] : vmax[id], d = tgt[id] - pos[id], s = v * ms / 1000.0, before = pos[id];
        if (now < cmd_at[id] + lag_ms) continue;
        if (id == 1 && paused) continue;
        if (fabs(d) <= s) pos[id] = tgt[id]; else pos[id] += (d > 0 ? s : -s);
        if (id == 1 && pos[1] > block_hi) pos[1] = block_hi;
        if (id == 1 && pos[1] < block_lo) pos[1] = block_lo;
        if (id == 1 && ((before < pause_at && pos[1] >= pause_at) || (before > pause_at && pos[1] <= pause_at))) { pos[1] = pause_at; paused = 1; pause_at = 1e9; }
    }
}
uint32_t HAL_GetTick(void) { return now; }
void HAL_Delay(uint32_t ms) { uint32_t i; for (i = 0; i < ms; i++) { now++; step(1); } }
int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *b, uint16_t n, uint32_t t) { (void)h; (void)t; strncat(out, (char *)b, n); return 0; }
FSUS_STATUS FSUS_QueryServoAngleMTurn(Usart_DataTypeDef *u, uint8_t id, float *a) { (void)u; HAL_Delay(3); if (read_fail) return FSUS_STATUS_FAIL; *a = (float)(floor(pos[id] * 10 + 0.5) / 10); return FSUS_STATUS_SUCCESS; }
FSUS_STATUS FSUS_SetServoAngleMTurnByVelocity(Usart_DataTypeDef *u, uint8_t id, float a, float v, uint16_t ta, uint16_t td, uint16_t p, uint8_t w) {
    (void)u; (void)ta; (void)td; (void)w; tgt[id] = a; vcmd[id] = v; nset++; cmd_at[id] = now; last_power = p;
    if (id == 1 && !pause_every) paused = 0;          /* 重发指令：停顿的舵机接着转(pause_every=1 的舵机还是不动) */
    return FSUS_STATUS_SUCCESS; }
FSUS_STATUS FSUS_ReadData(Usart_DataTypeDef *u, uint8_t id, uint8_t addr, uint8_t *val, uint8_t *sz) {
    uint16_t v = 0; (void)u; HAL_Delay(3);
    switch (addr) {
    case 1: v = 7412; break;
    case 2: v = (id == 1 && blocked()) ? 900 : (fabs(tgt[id] - pos[id]) > 0.05 && !paused ? 150 : 14); break;
    case 3: v = 6300; break;
    case 4: v = 1500; break;
    case 5: val[0] = (uint8_t)((fabs(tgt[id] - pos[id]) > 0.05 ? 1 : 0) | (stall_flag && blocked() ? 4 : 0)); *sz = 1; return 0;
    default:
        if (addr == 33 || addr == 37 || addr == 44 || addr == 46 || addr == 48 || addr == 49) { val[0] = (uint8_t)uparam[addr]; *sz = 1; return 0; }
        v = (uint16_t)(uparam[addr] & 0xFFFF); break;
    }
    val[0] = (uint8_t)v; val[1] = (uint8_t)(v >> 8); *sz = 2; return 0; }
FSUS_STATUS FSUS_WriteData(Usart_DataTypeDef *u, uint8_t id, uint8_t addr, uint8_t *val, uint8_t size) {
    (void)u; (void)id; HAL_Delay(3); if (write_fail) return FSUS_STATUS_FAIL;
    uparam[addr] = size == 1 ? val[0] : (long)(int16_t)(val[0] | (val[1] << 8));
    if (size == 2 && addr != 51 && addr != 52) uparam[addr] = (long)(uint16_t)(val[0] | (val[1] << 8));
    return FSUS_STATUS_SUCCESS; }
#include "servo_section.c"
static int fails = 0, checks = 0;
#define CHECK(c, m) do { checks++; if (!(c)) { fails++; printf("  FAIL: %s\n    out=[%s] pos1=%.1f tgt1=%.1f now=%u\n", m, out, pos[1], tgt[1], now); } else printf("  ok: %s\n", m); } while (0)
static void reset1(double p) { HAL_Delay(20000); out[0] = 0; pos[1] = tgt[1] = p; vmax[1] = 1000; block_hi = 1e9; block_lo = -1e9; lag_ms = 0; pause_at = 1e9; paused = 0; pause_every = 0; nset = 0; stall_flag = 0; read_fail = 0; }
int main(void) {
    uint32_t t0;
    /* 1. 正常转：到位就返回，不打印 */
    reset1(300); t0 = now;
    Servo_Move(1, 350, 40, 0.3f, 400);
    CHECK(fabs(pos[1] - 350) < 0.31 && out[0] == 0 && now - t0 < 1700, "正常转 50°：到位返回，不打印");
    /* 2. 被挡住(电流大)：停在原地，打印 STUCK 和电流 */
    reset1(300); block_hi = 367.6; t0 = now;
    Servo_Move(1, 417, 40, 0.3f, 400);
    CHECK(strstr(out, "SERVO1 STUCK, hold here at=3676 target=4170") && strstr(out, "I=900mA") && fabs(tgt[1] - 367.6) < 0.05, "挡在 367.6、电流大：STUCK，目标改成 367.6(不再顶)");
    CHECK(now - t0 < 3500, "被挡住后不白等到超时");
    /* 3. 舵机报堵转(电流不大)也算被挡住 */
    reset1(300); block_hi = 340; stall_flag = 1;
    Servo_Move(1, 400, 40, 0.3f, 400);
    CHECK(strstr(out, "STUCK") && fabs(tgt[1] - 340) < 0.05, "舵机报堵转：停在原地");
    /* 4. 起步慢 0.7 秒(现场 ID1 就像这样)：不当成卡住，等它转到 */
    reset1(413.3); lag_ms = 700; nset = 0;
    Servo_Move(1, 250, 40, 0.3f, 400);
    CHECK(fabs(pos[1] - 250) < 0.31 && !strstr(out, "STUCK") && !strstr(out, "PAUSED") && nset == 1, "起步慢 0.7s：照样转到 250，不打断");
    /* 5. 中途停顿、电流小：重发一次目标，接着转到 */
    reset1(400); pause_at = 337.3;
    Servo_Move(1, 300, 40, 0.3f, 400);
    CHECK(strstr(out, "SERVO1 PAUSED, resend at=3373 target=3000") && strstr(out, "I=14mA") && fabs(pos[1] - 300) < 0.5 && nset == 2, "中途停在 337.3、没使劲：重发一次，转到 300");
    /* 6. 重发以后还是停：不再等，也不拦它(目标还是 300)，打印 NOT ARRIVED */
    reset1(400); pause_at = 337.3; pause_every = 1;
    Servo_Move(1, 300, 40, 0.3f, 400);
    CHECK(strstr(out, "PAUSED") && strstr(out, "NOT ARRIVED") && !strstr(out, "STUCK") && fabs(tgt[1] - 300) < 0.01 && nset == 2, "重发后还停：不改目标，打印 NOT ARRIVED");
    pause_every = 0; pause_at = 1e9; paused = 0;
    /* 7. 舵机比设定的慢(实际 20°/s，设定 40°/s)：多等，不当成卡住 */
    reset1(300); vmax[1] = 20;
    Servo_Move(1, 360, 40, 0.3f, 400);
    CHECK(fabs(pos[1] - 360) < 0.31 && !strstr(out, "STUCK") && !strstr(out, "PAUSED"), "慢舵机 60°：多等一会儿到位");
    /* 8. 慢到超过多等的时间：不拦(不改目标)，大误差打印 NOT ARRIVED */
    reset1(240); vmax[1] = 15; t0 = now;
    Servo_Move(1, 410, 40, 2.0f, 400);
    CHECK(!strstr(out, "STUCK") && strstr(out, "NOT ARRIVED") && nset == 1 && fabs(tgt[1] - 410) < 0.01, "太慢超时：不改目标，让它接着转");
    CHECK(now - t0 <= 9300, "一次最多等 9 秒左右(树莓派 AF 等 10 秒)");
    /* 9. 目标就在附近、死区里不动：很快返回，不当成卡住 */
    reset1(330); block_hi = 330.6; t0 = now;
    Servo_Move(1, 331.0, 40, 0.3f, 400);
    CHECK(!strstr(out, "STUCK") && nset == 1 && now - t0 < 1000, "只差 0.4°：很快返回，不改目标");
    /* 10. 读不到角度：不改目标 */
    reset1(300); read_fail = 1;
    Servo_Move(1, 320, 40, 2.0f, 400);
    CHECK(!strstr(out, "STUCK") && nset == 1, "读不到角度：不改目标");
    /* 11. 往负方向被挡住 */
    reset1(300); block_lo = 260;
    Servo_Move(1, 232, 40, 0.3f, 400);
    CHECK(strstr(out, "SERVO1 STUCK, hold here at=2600 target=2320"), "往小的方向被挡在 260");
    /* 12. 一起转用的 Servo_StopIfStuck */
    reset1(340); tgt[1] = 400; block_hi = 340;
    Servo_StopIfStuck(1, 400);
    CHECK(strstr(out, "SERVO1 STUCK, hold here at=3400") && fabs(tgt[1] - 340) < 0.05, "StopIfStuck：被挡住 -> 停在原地");
    reset1(300); tgt[1] = 400; vmax[1] = 40;
    Servo_StopIfStuck(1, 400);
    CHECK(!strstr(out, "STUCK") && nset == 0 && fabs(pos[1] - 400) < 1.01, "StopIfStuck：还在动 -> 等它转到，不改目标");
    reset1(330); tgt[1] = 400; paused = 1;
    Servo_StopIfStuck(1, 400);
    CHECK(strstr(out, "PAUSED") && fabs(pos[1] - 400) < 1.01, "StopIfStuck：停顿了 -> 重发，转到");
    /* 13. SV? 状态 */
    reset1(367.6);
    Servo_Report(1);
    CHECK(strstr(out, "SV 1 ANG=367.6 V=7412mV I=14mA P=6300mW T=") && strstr(out, "ST=0x00"), "SV? 打印电压电流功率温度和状态");
    reset1(300); pos[2] = tgt[2] = -862.3; out[0] = 0;
    Servo_Report(2);
    CHECK(strstr(out, "SV 2 ANG=-862.3"), "负角度显示");
    /* 14. 舵机功率(SPOW) */
    reset1(300); Servo_SetPower(12000);
    Servo_Move(1, 320, 40, 0.3f, 400);
    CHECK(last_power == 12000, "SET SPOW 以后转动指令用新的功率");
    Servo_SetPower(8000);
    /* 15. 舵机内部设置 SVP? / SVW */
    uparam[33] = 0; uparam[37] = 0; uparam[38] = 4000; uparam[42] = 8000; uparam[43] = 1500; uparam[51] = -900; uparam[52] = 1800;
    out[0] = 0; Servo_Params(1);
    CHECK(strstr(out, "SVP 1 RESP=0 STALLM=0 STALLP=4000") && strstr(out, "PMAX=8000 IMAX=1500") && strstr(out, "AHIGH=-900 ALOW=1800"), "SVP? 打印舵机内部设置");
    CHECK(Servo_WriteParam(1, "PMAX", 15000) == 1 && uparam[42] == 15000, "SVW 改功率上限");
    CHECK(Servo_WriteParam(1, "ALOW", -1200) == 1 && uparam[52] == -1200, "SVW 有符号的数");
    CHECK(Servo_WriteParam(1, "NOPE", 1) == 0 && Servo_WriteParam(1, "STALLM", 300) == 2 && Servo_WriteParam(1, "RESP", 1) == 2, "SVW 名字/范围检查(RESP 只能 0)");
    write_fail = 1;
    CHECK(Servo_WriteParam(1, "IMAX", 2000) == -1, "SVW 写失败返回 -1");
    write_fail = 0;
    printf("%s: %d 项检查，%d 项失败\n", fails ? "失败" : "通过", checks, fails);
    return fails ? 1 : 0;
}
