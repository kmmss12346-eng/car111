/* main.c 里舵机那一段(Servo_Move / Servo_StopIfStuck / Servo_Report)在电脑上测：假舵机按限定速度转、可以被挡住。
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
Usart_DataTypeDef usart2; UART_HandleTypeDef huart3;
static uint32_t now = 0;
static char out[4096];
static double pos[3] = {0, 300, -862}, tgt[3] = {0, 300, -862}, vmax[3] = {0, 1000, 1000}, vcmd[3] = {0, 60, 60};
static double block_hi = 1e9, block_lo = -1e9;
static int read_fail = 0, nset = 0;
static void step(uint32_t ms) {
    int id; for (id = 1; id <= 2; id++) {
        double v = vcmd[id] < vmax[id] ? vcmd[id] : vmax[id], d = tgt[id] - pos[id], s = v * ms / 1000.0;
        if (fabs(d) <= s) pos[id] = tgt[id]; else pos[id] += (d > 0 ? s : -s);
        if (id == 1 && pos[1] > block_hi) pos[1] = block_hi;
        if (id == 1 && pos[1] < block_lo) pos[1] = block_lo;
    }
}
uint32_t HAL_GetTick(void) { return now; }
void HAL_Delay(uint32_t ms) { uint32_t i; for (i = 0; i < ms; i++) { now++; step(1); } }
int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *b, uint16_t n, uint32_t t) { (void)h; (void)t; strncat(out, (char *)b, n); return 0; }
FSUS_STATUS FSUS_QueryServoAngleMTurn(Usart_DataTypeDef *u, uint8_t id, float *a) { (void)u; now += 3; step(3); if (read_fail) return FSUS_STATUS_FAIL; *a = (float)(floor(pos[id] * 10 + 0.5) / 10); return FSUS_STATUS_SUCCESS; }
FSUS_STATUS FSUS_SetServoAngleMTurnByVelocity(Usart_DataTypeDef *u, uint8_t id, float a, float v, uint16_t ta, uint16_t td, uint16_t p, uint8_t w) {
    (void)u; (void)ta; (void)td; (void)p; (void)w; tgt[id] = a; vcmd[id] = v; nset++; return FSUS_STATUS_SUCCESS; }
FSUS_STATUS FSUS_ReadData(Usart_DataTypeDef *u, uint8_t id, uint8_t addr, uint8_t *val, uint8_t *sz) {
    (void)u; (void)id; uint16_t v = 0;
    switch (addr) { case 1: v = 7412; break; case 2: v = 850; break; case 3: v = 6300; break; case 4: v = 1500; break; case 5: val[0] = 0x05; *sz = 1; return 0; }
    val[0] = (uint8_t)v; val[1] = (uint8_t)(v >> 8); *sz = 2; return 0; }
#include "servo_section.c"
static int fails = 0, checks = 0;
#define CHECK(c, m) do { checks++; if (!(c)) { fails++; printf("  FAIL: %s\n    out=[%s] pos1=%.1f tgt1=%.1f now=%u\n", m, out, pos[1], tgt[1], now); } else printf("  ok: %s\n", m); } while (0)
int main(void) {
    uint32_t t0;
    /* 1. 正常转：到位就返回，不打印 */
    out[0] = 0; pos[1] = tgt[1] = 300; vmax[1] = 1000; t0 = now;
    Servo_Move(1, 350, 40, 0.3f, 400);
    CHECK(fabs(pos[1] - 350) < 0.31 && out[0] == 0 && now - t0 < 1700, "正常转 50°：到位返回，不打印");
    /* 2. 被挡住：很快发现，停在原地，打印 STUCK */
    out[0] = 0; block_hi = 367.6; t0 = now;
    Servo_Move(1, 417, 40, 0.3f, 400);
    CHECK(strstr(out, "SERVO1 STUCK at=3676 target=4170") && fabs(tgt[1] - 367.6) < 0.05, "挡在 367.6：打印 STUCK，目标改成 367.6(不再顶)");
    CHECK(now - t0 < 2200, "卡住后不白等到超时");
    block_hi = 1e9;
    /* 3. 舵机比设定的慢(实际 20°/s，设定 40°/s)：多等，不当成卡住 */
    out[0] = 0; pos[1] = tgt[1] = 300; vmax[1] = 20; t0 = now;
    Servo_Move(1, 360, 40, 0.3f, 400);
    CHECK(fabs(pos[1] - 360) < 0.31 && !strstr(out, "STUCK"), "慢舵机 60°：多等一会儿到位，不当成卡住");
    /* 4. 慢到超过多等的时间：不拦(不改目标)，大误差打印 NOT ARRIVED */
    out[0] = 0; pos[1] = tgt[1] = 240; vmax[1] = 15; nset = 0;
    Servo_Move(1, 410, 40, 2.0f, 400);
    CHECK(!strstr(out, "STUCK") && strstr(out, "NOT ARRIVED") && nset == 1 && fabs(tgt[1] - 410) < 0.01, "太慢超时：不改目标，让它接着转");
    vmax[1] = 1000; HAL_Delay(20000);
    /* 5. 目标就在附近、死区里不动：不当成卡住 */
    out[0] = 0; pos[1] = 330.0; tgt[1] = 330.0; block_hi = 330.6; nset = 0;
    Servo_Move(1, 331.0, 40, 0.3f, 400);
    CHECK(!strstr(out, "STUCK") && nset == 1, "只差 0.4°(挡住也不算)：不改目标");
    block_hi = 1e9;
    /* 6. 读不到角度：不改目标 */
    out[0] = 0; read_fail = 1; nset = 0;
    Servo_Move(1, 300, 40, 2.0f, 400);
    CHECK(!strstr(out, "STUCK") && nset == 1, "读不到角度：不改目标");
    read_fail = 0;
    /* 7. 一起转用的 Servo_StopIfStuck：停住了才拦，还在动不拦 */
    out[0] = 0; pos[1] = 340; tgt[1] = 400; block_hi = 340; nset = 0;
    Servo_StopIfStuck(1, 400);
    CHECK(strstr(out, "SERVO1 STUCK at=3400") && fabs(tgt[1] - 340) < 0.05, "StopIfStuck：停住了 -> 停在原地");
    block_hi = 1e9; out[0] = 0; pos[1] = 300; tgt[1] = 400; vmax[1] = 40; nset = 0;
    Servo_StopIfStuck(1, 400);
    CHECK(!strstr(out, "STUCK") && nset == 0, "StopIfStuck：还在动 -> 不管");
    vmax[1] = 1000;
    /* 8. 往负方向被挡住 */
    out[0] = 0; HAL_Delay(5000); pos[1] = tgt[1] = 300; block_lo = 260;
    Servo_Move(1, 232, 40, 0.3f, 400);
    CHECK(strstr(out, "SERVO1 STUCK at=2600 target=2320"), "往小的方向被挡在 260");
    block_lo = -1e9;
    /* 9. SV? 状态 */
    out[0] = 0; pos[1] = tgt[1] = 367.6;
    Servo_Report(1);
    CHECK(strstr(out, "SV 1 ANG=367.6 V=7412mV I=850mA P=6300mW T=") && strstr(out, "ST=0x05 STALL"), "SV? 打印电压电流功率温度和堵转标志");
    printf("%s\n", out);
    /* 10. 负角度显示 */
    out[0] = 0; pos[2] = tgt[2] = -862.3;
    Servo_Report(2);
    CHECK(strstr(out, "SV 2 ANG=-862.3"), "负角度显示");
    printf("%s: %d 项检查，%d 项失败\n", fails ? "失败" : "通过", checks, fails);
    return fails ? 1 : 0;
}
