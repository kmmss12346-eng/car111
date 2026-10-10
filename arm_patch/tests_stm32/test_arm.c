/* arm.c 的主机回归测试：用假的 HAL / 舵机 / 升降，检查指令解析、动作顺序、保护、急停、扫码、串口屏触摸选区。
 * hwt101.c 整个包含进来(串口中断回调在那里)，触摸帧、出错重启都走真的回调。
 * 运行：bash run.sh      (有断言失败会返回非 0) */
#include "arm.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

UART_HandleTypeDef huart1, huart2, huart3, huart5;
TIM_HandleTypeDef htim2;
USART_TypeDef fake_usart[6];
volatile uint8_t car_abort = 0;
void HAL_NVIC_SetPriority(int irqn, uint32_t pre, uint32_t sub) { (void)irqn; (void)pre; (void)sub; }
void HAL_NVIC_EnableIRQ(int irqn) { (void)irqn; }
void Link_RxCplt(void) { }
void Link_RxRestart(void) { }
#include "../Core/Src/hwt101.c"

static uint32_t now = 0;
static int abort_at = -1;
static float ang[3] = {0, 323.0f, -862.0f};
static char ev[8192];                 /* 事件记录：每个动作追加一小段文字，测试里用 strstr / 下标比较顺序 */
static char pi[4096];                 /* STM32 发给树莓派的文字 */
static char scr[8192];                /* 发给串口屏的文字(每条后面加 |) */
static uint8_t *rxp = 0;
static uint8_t *scrp = 0;             /* USART2(屏)接收缓冲 */
static int scr_armed = 0;             /* 1 = USART2 正在等下一个字节(HAL_UART_Receive_IT 开着)；收到一个字节或出错就变 0，要重新开 */
static int scr_starts = 0;            /* HAL_UART_Receive_IT(&huart2) 调了几次 */
static int scr_rxfail = 0;            /* 1 = 假装 USART2 接收开不起来 */
static int scr_lost = 0;              /* 没开接收时屏发来、丢掉的字节数 */
static int fails = 0, checks = 0;

#define EV(...) do { char t_[96]; snprintf(t_, sizeof t_, __VA_ARGS__); strncat(ev, t_, sizeof ev - strlen(ev) - 1); } while (0)
#define CHECK(c, msg) do { checks++; if (!(c)) { fails++; printf("  FAIL: %s  (line %d)\n", msg, __LINE__); } } while (0)

uint32_t HAL_GetTick(void) { return now; }
void HAL_Delay(uint32_t ms) { now += ms; if (abort_at >= 0 && (int)now >= abort_at) car_abort = 1; }
void Delay_Report(uint32_t ms) { HAL_Delay(ms); }
int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *b, uint16_t n, uint32_t t) {
    (void)t;
    if (h == &huart3 && strncmp((char *)b, "YAW100", 6) != 0) strncat(pi, (char *)b, (size_t)n < sizeof pi - strlen(pi) - 1 ? (size_t)n : sizeof pi - strlen(pi) - 1);
    if (h == &huart2 && b[0] != 0xFF) { size_t room = sizeof scr - strlen(scr) - 2; strncat(scr, (char *)b, (size_t)n < room ? (size_t)n : room); strcat(scr, "|"); }
    return 0;
}
int HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *b, uint16_t n) {
    (void)n;
    if (h == &huart5) rxp = b;
    if (h == &huart2) { scr_starts++; if (scr_rxfail) return 1; scrp = b; scr_armed = 1; }
    return 0;
}
int HAL_UART_Init(UART_HandleTypeDef *h) { (void)h; return 0; }
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t c) { (void)h; EV("pwm%u;", c); return 0; }
void tim_set(uint32_t ch, uint32_t v) { (void)ch; (void)v; }
/* 夹爪/转盘的脉宽：是默认的张开/夹紧/1~3 号位就记成 O; C; tt1; …，方便看先后顺序 */
void Claw_Set(uint32_t p) { if (p == 2090u) EV("O;"); else if (p == 2910u) EV("C;"); else EV("claw%u;", p); }
void Turntable_Set(uint32_t p) { if (p == 2608u) EV("tt1;"); else if (p == 1708u) EV("tt2;"); else if (p == 808u) EV("tt3;"); else EV("ttus%u;", p); }
int Servo_ReadAngle(uint8_t id, float *a) { *a = ang[id]; return 1; }
static void away(void);
static float block1 = 1e9f;                              /* ID1 被挡住的角度(转不过去)，1e9 = 没挡 */
void Servo_StopIfStuck(uint8_t id, float t) { EV("stuck%u:%.1f;", id, t); }
void Servo_Report(uint8_t id) { EV("rep%u;", id); }
void Servo_SetPower(uint16_t mw) { EV("pow%u;", mw); }
void Servo_SetComp(uint8_t id, float deg) { EV("comp%u=%.1f;", id, deg); }
void Servo_Hold(uint8_t id, float a) { EV("H%u:%.1f;", id, a); }
static float fgoal[3];
int Servo_Arrived(uint8_t id, float a, float tol) { float e = a - fgoal[id]; return (e < 0 ? -e : e) <= tol; }
void Servo_Params(uint8_t id) { EV("svp%u;", id); }
int Servo_WriteParam(uint8_t id, const char *name, long v) {
    EV("svw%u:%s=%ld;", id, name, v);
    if (strcmp(name, "PMAX") != 0 && strcmp(name, "IMAX") != 0) return 0;
    if (v < 0 || v > 65535) return 2;
    return v == 4242 ? -1 : 1;
}
static float clampa(uint8_t id, float a) { float lo = id == 1 ? 232.0f : -1220.0f, hi = id == 1 ? 440.0f : -503.5f; return a < lo ? lo : (a > hi ? hi : a); }
uint32_t Servo_Start(uint8_t id, float *a, float spd) {
    float d; *a = clampa(id, *a); fgoal[id] = *a; d = *a - ang[id]; if (d < 0) d = -d; ang[id] = (id == 1 && *a > block1) ? block1 : *a;
    EV("S%u:%.1f@%.0f;", id, *a, spd); return (uint32_t)(d / spd * 1000) + 200;
}
void Servo_Move(uint8_t id, float a, float spd, float tol, uint32_t extra) {
    (void)extra; a = clampa(id, a); EV("M%u:%.1f@%.0f/%.2f;", id, a, spd, tol); ang[id] = a; now += 300;
}
static void away(void) { ang[1] = 300.0f; ang[2] = -700.0f; }   /* 手臂先摆到别处，下一个动作一定要转 */
void Emm_V5_En_Control(uint8_t a, bool s, bool f) { (void)a; (void)s; (void)f; }
void Emm_V5_Reset_CurPos_To_Zero(uint8_t a) { EV("zero%u;", a); }
/* 假升降：真实高度 phys_mm(LFDIR=0 时 d=0 往上)，编码器 = (enc_off + 每毫米 enc_cpm × 高度) 对一圈 enc_cpr 取余 */
static double phys_mm = 60.0, enc_off = 12345.0, enc_cpm = 1638.4, enc_cpr = 65536.0;
static int enc_ok = 1, enc_frozen = 0, enc31_zero = 0;   /* enc31_zero：像现场那样 0x31 总回 0，只有 0x36 实时位置能用 */
static double pos36_off = -70000.0;                      /* 0x36 实时位置(一圈 65536，多圈累计，可以是负数) */
uint32_t fake_flash[256];
static int erases = 0;
static void flash_blank(void) { int q; for (q = 0; q < 256; q++) fake_flash[q] = 0xFFFFFFFFu; }
static int flash_fail = 0;
int HAL_FLASH_Unlock(void) { return 0; }
int HAL_FLASH_Lock(void) { return 0; }
int HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *e, uint32_t *err) { (void)err; if (flash_fail || e->Sector != 7) return 1; flash_blank(); erases++; now += 1000; return 0; }
int HAL_FLASH_Program(uint32_t type, uintptr_t addr, uint64_t data) { (void)type; fake_flash[(addr - (uintptr_t)fake_flash) / 4] = (uint32_t)data; return 0; }
int Car_Motor_Query(uint8_t addr, uint8_t func, uint8_t *out, uint8_t len) {
    double e; uint16_t v;
    if (!enc_ok || addr != 5) return 0;
    if (func == 0x36 && len == 8) {
        long p = (long)floor(pos36_off + (enc_frozen ? 0.0 : 65536.0 / 40.0 * phys_mm) + 0.5); unsigned long u = (unsigned long)(p < 0 ? -p : p);
        out[0] = 5; out[1] = 0x36; out[2] = p < 0 ? 1 : 0; out[3] = (uint8_t)(u >> 24); out[4] = (uint8_t)(u >> 16);
        out[5] = (uint8_t)(u >> 8); out[6] = (uint8_t)u; out[7] = 0x6B; return 1;
    }
    if (func != 0x31 || len != 5) return 0;
    if (enc31_zero) { out[0] = 5; out[1] = 0x31; out[2] = 0; out[3] = 0; out[4] = 0x6B; return 1; }
    e = fmod(enc_off + (enc_frozen ? 0.0 : enc_cpm * phys_mm), enc_cpr); if (e < 0) e += enc_cpr;
    v = (uint16_t)(e + 0.5) % (uint16_t)(enc_cpr > 65535 ? 65535 : enc_cpr);
    if (enc_cpr > 65535) v = (uint16_t)((long)(e + 0.5) % 65536);
    out[0] = 5; out[1] = 0x31; out[2] = (uint8_t)(v >> 8); out[3] = (uint8_t)v; out[4] = 0x6B; return 1;
}
void Emm_V5_Pos_Control(uint8_t a, uint8_t d, uint16_t v, uint8_t acc, uint32_t clk, bool r, bool f) {
    (void)v; (void)acc; (void)r; (void)f; EV("L%s%u;", d ? "-" : "+", clk);
    if (a == 5) phys_mm += (d ? -1.0 : 1.0) * (double)clk / 80.0;
}
void Emm_V5_Stop_Now(uint8_t a, bool f) { (void)f; EV("stop%u;", a); }
void Emm_V5_Origin_Trigger_Return(uint8_t a, uint8_t m, bool f) { (void)f; EV("home%u/%u;", a, m); }
void Emm_V5_Origin_Interrupt(uint8_t a) { EV("homeint%u;", a); }

static int run(const char *c) {
    char err[24] = "";
    int r = Arm_Command(c, err, sizeof err);
    if (r < 0) { strncat(pi, err, 20); strcat(pi, "!"); }
    car_abort = 0;
    return r;
}
static void clear(void) { ev[0] = 0; pi[0] = 0; scr[0] = 0; }
static int at(const char *needle) { const char *p = strstr(ev, needle); return p ? (int)(p - ev) : -1; }
static int has(const char *s) { return strstr(ev, s) != 0; }
static int last(const char *needle) { int r = -1; const char *p = ev; while ((p = strstr(p, needle)) != 0) { r = (int)(p - ev); p++; } return r; }
static int pis(const char *s) { return strstr(pi, s) != 0; }
static int scrs(const char *s) { return strstr(scr, s) != 0; }
/* 屏发来字节：和真的串口一样，开着接收(HAL_UART_Receive_IT)才收得到，收到一个以后要回调里重新开 */
static void scr_in(const uint8_t *f, int n) {
    int i;
    for (i = 0; i < n; i++) {
        if (!scr_armed) { scr_lost++; continue; }
        scr_armed = 0; *scrp = f[i]; HAL_UART_RxCpltCallback(&huart2);
    }
}
/* 触摸帧：67 XH XL YH YL 事件 FF FF FF(事件 1 = 按下，0 = 松开) */
static void touch(int x, int y, int ev) {
    uint8_t f[9];
    f[0] = 0x67; f[1] = (uint8_t)(x >> 8); f[2] = (uint8_t)x; f[3] = (uint8_t)(y >> 8); f[4] = (uint8_t)y; f[5] = (uint8_t)ev;
    f[6] = f[7] = f[8] = 0xFF;
    scr_in(f, 9);
}
static void tap(int x, int y) { touch(x, y, 1); touch(x, y, 0); Arm_Poll(); }   /* 按下、松开在同一点，再让主循环跑一次 */
static int zone_is(const char *want) { pi[0] = 0; run("ZONE?"); return strstr(pi, want) != 0; }
/* 扫码模块发来字节(走 hwt101.c 里真的串口回调) */
static void qr_feed(const char *s) { for (; *s; s++) { *rxp = (uint8_t)*s; HAL_UART_RxCpltCallback(&huart5); now += 1; } }

int main(void) {
    int i;
    huart1.Instance = USART1; huart2.Instance = USART2; huart3.Instance = USART3; huart5.Instance = UART5;
    flash_blank();
    Arm_Init();
    CHECK(has("pwm3;") && has("pwm2;"), "Arm_Init 启动了夹爪/转盘 PWM");
    CHECK(scr_armed && scr_starts == 1, "开机打开 USART2 接收(收屏的触摸)");
    CHECK(scrs("sendxy=1|cls 0|") && scrs("xstr 0,0,240,48,0,65535,0,1,1,1,\"START ZONE 1 OR 2\"")
          && scrs("xstr 0,52,236,268,1,65535,31,1,1,1,\"1\"") && scrs("xstr 244,52,236,268,1,65535,63488,1,1,1,\"2\"") && !scrs("READY"),
          "开机显示选区页：先发 sendxy=1、清屏，左边大按钮 1(蓝)、右边大按钮 2(红)，不写 READY");
    clear();
    CHECK(run("ZONE?") == 1 && pis("ZONE 0 0"), "开机还没选区：ZONE 0 0");

    /* ---- 保护 ---- */
    clear();
    CHECK(run("GRAB 1") == -1 && pis("ERR NOCAL"), "ARMOK=0 时拒绝");
    CHECK(run("OBS RAW O") == -1, "ARMOK=0 时 OBS 拒绝");
    Arm_Param_Set("ARMOK", 1);
    clear();
    CHECK(run("LIFT?") == 1 && pis("LIFT 1 60") && pis("BOOT=NOCAL"), "开机没标定：当作 60mm，LIFT? 回 BOOT=NOCAL");
    clear();
    CHECK(run("LIFT 100") == 1 && has("L+3200;"), "开机不用 LIFT ZERO：从 60 走到 100 = 往上 40mm(3200 脉冲)");
    clear();
    CHECK(run("LIFT 0") == 1 && has("L-8000;"), "LIFT 0 = 回到最低点(往下 100mm)");
    CHECK(run("LIFT ZERO") == 1 && has("zero5;"), "LIFT ZERO");
    /* 下面的顺序测试用固定的一组高度(和默认值无关)，都在最高点 100mm 以内 */
    Arm_Param_Set("ZHI", 0); Arm_Param_Set("ZGRAB", 80); Arm_Param_Set("ZDROP", 60); Arm_Param_Set("ZPLC", 80);
    Arm_Param_Set("ZSTK", 40); Arm_Param_Set("ZOBRAW", 0); Arm_Param_Set("ZOBRNG", 0);
    CHECK(Arm_Param_Set("LFMAX", 150) == 2, "最高点写死 105mm：SET LFMAX 150 被拒绝");
    CHECK(Arm_Param_Set("ZPLC", 1000) == 2 && Arm_Param_Set("NOPE", 1) == 0 && Arm_Param_Set("ZPLC", 90) == 1, "参数范围/名字检查");
    Arm_Param_Set("ZPLC", 80);

    /* ---- 参数错误 ---- */
    CHECK(run("GRAB 4 H") == -1 && run("GRAB 1 Z") == -1 && run("TAKE 0") == -1 && run("DROP X") == -1 && run("OBS FOO") == -1
          && run("PLACE 2 Q") == -1 && run("CLAW 100") == -1 && run("TT 9") == -1 && run("AD 3 1") == -1
          && run("AD 1 x") == -1 && run("AD 1 100") == -1 && run("LIFT x") == -1, "各种错误参数都被拒绝");
    CHECK(run("FOO") == 0 && run("") == 0 && run("CLAW") == 0, "不认识的指令返回 0(交给后面)");

    /* ---- OBS ---- */
    away(); clear();
    CHECK(run("OBS RAW O") == 1, "OBS RAW O");
    CHECK(at("O;") >= 0 && at("O;") < at("S1:") && at("S1:") >= 0 && at("S2:") >= 0, "OBS：先张爪，再 ID1/ID2 一起转");
    CHECK(has("S1:323.0@90;") && has("S2:-862.0@90;"), "OBS RAW 的姿态和大动作速度");

    /* ---- GRAB n H 的完整顺序 ---- */
    away(); clear();
    CHECK(run("GRAB 2 H") == 1, "GRAB 2 H");
    {
        int tt = at("tt2;"), open0 = at("O;"), down = at("L+6400;"), close = at("C;"), up = at("L-6400;"), sw = at("S1:"), drop_dn = at("L+4800;"), open1 = -1, up2 = -1;
        const char *p = strstr(ev, "L+4800;");
        if (p) { const char *q = strstr(p, "O;"); if (q) open1 = (int)(q - ev); q = strstr(p, "L-4800;"); if (q) up2 = (int)(q - ev); }
        CHECK(tt >= 0 && tt < open0 && open0 < down && down < close && close < up && up < sw && sw < drop_dn && drop_dn < open1 && open1 < up2,
              "GRAB H：转盘→张爪→下降→夹紧→抬起→缩回转向→下降→松开→抬起");
    }

    /* ---- PICK 用地面高度 ZPLC ---- */
    clear();
    CHECK(run("PICK 1 H") == 1 && has("L+6400;"), "PICK H 走到 ZPLC(80mm=6400 脉冲)");
    Arm_Param_Set("ZGRAB", 90);
    clear(); run("GRAB 1 H");
    CHECK(has("L+7200;"), "GRAB H 走到 ZGRAB(90mm=7200 脉冲)");
    Arm_Param_Set("ZGRAB", 80);

    /* ---- TAKE ---- */
    clear();
    CHECK(run("TAKE 3") == 1, "TAKE 3");
    CHECK(at("tt3;") < at("L+4800;") && at("L+4800;") < at("C;") && at("C;") < at("L-4800;"), "TAKE：转盘→下降到 ZDROP→夹紧→抬起");

    /* ---- DROP / 码垛 / 缩回 ID2 ---- */
    clear();
    CHECK(run("DROP") == 1 && has("L+6400;") && at("O;") > at("L+6400;") && has("M2:-862.0@90/1.00;"), "DROP：走到 ZPLC→松开→回到 ZHI→ID2 缩回");
    clear();
    CHECK(run("DROP S") == 1 && has("L+3200;"), "DROP S 下降到 ZSTK(40mm=3200 脉冲)");

    /* ---- PLACE = TAKE + OBS RING + DROP ---- */
    Arm_Param_Set("A1P", 350);                     /* 圆环姿态和转盘姿态不一样，ID1 才会转 */
    away(); clear();
    CHECK(run("PLACE 1") == 1 && at("C;") >= 0 && at("C;") < last("S1:") && last("S1:") < last("O;"), "PLACE：先取(夹紧)，再转到圆环上方，再放(松开)");
    Arm_Param_Set("A1P", 323);

    /* ---- STOW ---- */
    away(); clear();
    CHECK(run("STOW") == 1 && has("S1:") && has("S2:"), "STOW");

    /* ---- 精确控制 ---- */
    clear();
    CHECK(run("A? 1") == 1 && pis("ANG 1 "), "A? 回角度");
    clear();
    ang[1] = 323.0f;
    CHECK(run("AD 1 0.5") == 1 && has("M1:323.5@40/0.30;") && pis("ANG 1 323.5000"), "AD 小增量用微调速度/精度");
    clear();
    CHECK(run("AD 2 -20") == 1 && has("@90/0.30;"), "AD 大增量(>8°)用大动作速度");
    clear();
    CHECK(run("AF 2 -860.5") == 1 && has("M2:-860.5"), "AF 绝对角度");
    clear();
    ang[1] = 323.0f; ang[2] = -862.0f;
    CHECK(run("AP 325 -863") == 1 && has("S1:325.0@80;") && has("S2:-863.0@80;"), "AP 小调整：微调速度的 2 倍(80)");
    clear();
    CHECK(run("AP 340 -900") == 1 && has("@90;"), "AP 大角度变化：改用大动作速度(90)");
    CHECK(pis("ANG 1") && pis("ANG 2"), "AP 回读两个角度");

    /* ---- 舵机被挡住：一起转等到时间还没到位，交给 Servo_StopIfStuck(停在原地，不再顶着发热) ---- */
    ang[1] = 323.0f; ang[2] = -862.0f; block1 = 340.0f; clear();
    CHECK(run("AP 400 -862") == 1 && has("stuck1:400.0;") && !has("stuck2") && !has("S2:") && !has("H1"), "ID1 被挡在 340：到时间没到位，叫 Servo_StopIfStuck(1)；ID2 本来就在 -862，不发指令");
    ang[1] = 323.0f; ang[2] = -700.0f; clear();
    CHECK(run("AP 323 -862") == 1 && !has("S1:") && has("H1:323.0;") && has("S2:-862.0") && has("H2:-862.0;"), "AP 只动 ID2：ID1 已经在目标上不发转动指令(不会白动一下)，只在原地保持；ID2 到了马上停住");
    block1 = 1e9f; clear();
    CHECK(run("AP 330 -860") == 1 && !has("stuck") && has("H1:330.0;") && has("H2:-860.0;"), "都到位：不叫 Servo_StopIfStuck，两个到了都马上停住");
    ang[1] = 330.1f; ang[2] = -850.6f; clear();
    CHECK(run("AP 330.1 -850.6") == 1 && !has("S1:") && !has("S2:") && has("H1:330.1;") && has("H2:-850.6;"),
          "手松开后摆好，AP 到现在的角度：不转，但两个舵机都重新上力保持(标定向导靠这个)");
    clear();
    CHECK(run("SV? 1") == 1 && has("rep1;") && run("SV? 3") == -1, "SV? 1 读舵机状态；ID 不对 ERR ARG");
    /* 舵机功率 SPOW、舵机内部设置 SVP? / SVW */
    clear();
    CHECK(Arm_Param_Set("SPOW", 12000) == 1 && has("pow12000;") && Arm_Param_Set("SPOW", 500) == 2, "SET SPOW 马上换舵机功率，太小拒绝");
    clear(); Arm_Init();
    CHECK(has("pow12000;"), "开机把 SPOW 交给 main.c");
    CHECK(has("comp1=5.0;") && has("comp2=0.0;"), "开机把补偿 S1COMP=5 / S2COMP=0 交给 main.c");
    clear();
    CHECK(Arm_Param_Set("S1COMP", 4.5f) == 1 && has("comp1=4.5;") && Arm_Param_Set("S2COMP", 3) == 1 && has("comp2=3.0;") && Arm_Param_Set("S1COMP", 20) == 2,
          "SET S1COMP / S2COMP 马上换补偿角度，超范围拒绝");
    Arm_Param_Set("S1COMP", 5); Arm_Param_Set("S2COMP", 0);
    Arm_Param_Set("SPOW", 30000);
    clear();
    CHECK(run("SVP? 2") == 1 && has("svp2;") && run("SVP? 0") == -1, "SVP? 读舵机内部设置");
    clear();
    CHECK(run("SVW 1 PMAX 15000") == 1 && has("svw1:PMAX=15000;") && has("svp1;"), "SVW 改完读回来");
    CHECK(run("SVW 1 ID 3") == -1 && pis("ERR NAME"), "SVW 不认识的名字(比如舵机 ID)拒绝");
    CHECK(run("SVW 1 PMAX 70000") == -1 && pis("ERR RANGE"), "SVW 数值超范围拒绝");
    CHECK(run("SVW 1 PMAX 4242") == -1 && pis("ERR WRITE"), "SVW 写失败报 ERR WRITE");
    CHECK(run("SVW 1 PMAX x") == -1 && pis("ERR ARG"), "SVW 数值不是数字");

    /* ---- 非并行模式的先后顺序 ---- */
    Arm_Param_Set("PARA", 0);
    clear(); run("OBS RING");
    CHECK(at("M1:") >= 0 && at("M1:") < at("M2:"), "PARA=0：伸出时先转 ID1 再伸 ID2");
    clear(); run("STOW");
    CHECK(at("M2:") >= 0 && at("M2:") < at("M1:"), "PARA=0：缩回时先缩 ID2 再转 ID1");
    /* AEXT=1：ID1 管伸缩、ID2 管旋转，先后顺序跟着反过来 */
    CHECK(Arm_Param_Set("AEXT", 1) == 1 && Arm_Param_Set("AEXT", 3) == 2, "AEXT 只能是 1 或 2");
    clear(); run("STOW");
    CHECK(at("M1:") >= 0 && at("M1:") < at("M2:"), "AEXT=1、PARA=0：收起时先缩 ID1 再转 ID2");
    clear(); run("OBS RING");
    CHECK(at("M2:") >= 0 && at("M2:") < at("M1:"), "AEXT=1、PARA=0：伸出时先转 ID2 再伸 ID1");
    Arm_Param_Set("PARA", 1);
    clear(); run("DROP");
    CHECK(has("M1:323.0@90/1.00;") && !has("M2:"), "AEXT=1：DROP 最后只把 ID1 缩到 A1H，不动 ID2");
    Arm_Param_Set("AEXT", 2);
    clear(); run("DROP");
    CHECK(has("M2:-862.0@90/1.00;") && !has("M1:"), "AEXT=2：DROP 最后只把 ID2 缩到 A2R");
    /* 夹爪张开/夹紧、转盘三个位置都是参数 */
    CHECK(Arm_Param_Set("CLWO", 1999) == 1 && Arm_Param_Set("CLWC", 2777) == 1 && Arm_Param_Set("CLWC", 3000) == 2, "CLWO/CLWC 可以改，超出夹爪限位拒绝");
    clear(); run("CLAW O"); run("CLAW C");
    CHECK(has("claw1999;") && has("claw2777;"), "CLAW O / CLAW C 用 CLWO / CLWC");
    CHECK(Arm_Param_Set("TT2", 1650) == 1 && Arm_Param_Set("TT3", 400) == 2, "TT1~3 可以改，超出转盘限位拒绝");
    clear(); run("TT 2"); run("TT 1");
    CHECK(has("ttus1650;") && has("tt1;"), "TT 2 用 TT2 的脉宽");
    Arm_Param_Set("CLWO", 2090); Arm_Param_Set("CLWC", 2910); Arm_Param_Set("TT2", 1708);

    /* ---- 急停：升降停下并要求重新回零 ---- */
    clear();
    abort_at = (int)now + 800;
    run("GRAB 1 H");
    abort_at = -1;
    CHECK(has("stop5;"), "急停时停升降");
    clear();
    CHECK(run("OBS RAW") == -1 && pis("ERR NOZERO"), "急停打断升降后要求重新回零");
    clear();
    CHECK(run("PARK") == -1 && pis("ERR NOZERO") && !has("L+") && !has("L-") && !has("S1:") && !has("S2:"),
          "升降位置不知道(急停打断过)：PARK 什么都不动，回 ERR NOZERO");
    run("LIFT ZERO");

    /* ---- 升降限幅 ---- */
    clear();
    run("LIFT 0"); clear();
    run("LIFT 999");
    CHECK(has("L+8400;"), "LIFT 999 被限制在最高点 105mm(8400 脉冲)");
    run("LIFT 0");

    /* ---- PARK：收臂 + 升降停到 60mm(下次开机认高度最稳) ---- */
    Arm_Param_Set("ZHI", 100);
    away(); clear();
    CHECK(run("PARK") == 1 && has("L+8000;") && has("S1:323.0@90;") && has("S2:-862.0@90;") && has("L-3200;")
          && at("L+8000;") < at("S1:") && at("L+8000;") < at("S2:") && at("S1:") < at("L-3200;") && at("S2:") < at("L-3200;"),
          "PARK(ARMOK=1)：先升到 ZHI，再像 STOW 那样收臂(A1H、A2R)，最后降到 60mm");
    clear(); run("LIFT?");
    CHECK(pis("LIFT 1 60.0"), "PARK 以后升降在 60mm");
    Arm_Param_Set("ARMOK", 0);
    run("LIFT 0"); away(); clear();
    CHECK(run("PARK") == 1 && has("L+4800;") && !has("L+8000") && !has("S1:") && !has("S2:") && !has("M1:") && !has("M2:") && !has("H1") && !has("H2"),
          "PARK(ARMOK=0，姿态还没标定)：舵机不动，只把升降走到 60mm");
    clear(); run("LIFT?");
    CHECK(pis("LIFT 1 60.0"), "ARMOK=0 的 PARK 以后升降也在 60mm");
    Arm_Param_Set("ARMOK", 1);
    run("LIFT 0"); clear();
    abort_at = (int)now + 500;
    i = run("PARK");
    abort_at = -1;
    CHECK(i == 1 && has("stop5;") && !has("S1:"), "PARK 途中急停：升降马上停，不再往下做(main.c 回 ERR ABORT)");
    run("LIFT ZERO");
    Arm_Param_Set("ZHI", 0);
    run("LIFT 0");

    /* ---- 选启停区：屏上触摸(USART2 收 67 帧)、ZONE 指令 ---- */
    clear(); Arm_Init();                            /* 重新开机：选区页 */
    CHECK(zone_is("ZONE 0 0"), "开机：ZONE 0 0");
    clear(); tap(100, 200);
    CHECK(zone_is("ZONE 1 0") && scrs("cls 0|") && scrs("xstr 0,0,320,80,1,65535,0,1,1,1,\"ZONE 1\"")
          && scrs("xstr 336,8,136,64,0,65535,33808,1,1,1,\"BACK\"") && scrs("xstr 0,128,480,192,1,0,2016,1,1,1,\"START\"")
          && scrs("xstr 0,84,480,40,0,65504,0,1,1,1,\"\"") && !scrs("sendxy"),
          "按左边按钮(按下、松开都在按钮 1 里)：选 1 区，画准备页(ZONE 1、状态行、大 START、小 BACK)");
    clear(); tap(400, 30);
    CHECK(zone_is("ZONE 0 0") && scrs("START ZONE 1 OR 2"), "准备页按 BACK：回选区页，选的区清掉");
    clear(); touch(100, 200, 1); touch(400, 200, 0); Arm_Poll();
    CHECK(zone_is("ZONE 0 0") && scr[0] == 0, "在按钮 1 按下、滑到按钮 2 松开：不算");
    touch(100, 200, 0); Arm_Poll();
    CHECK(zone_is("ZONE 0 0") && scr[0] == 0, "只有松开、没有按下：不算");
    tap(240, 200); tap(100, 20); tap(600, 200);
    CHECK(zone_is("ZONE 0 0") && scr[0] == 0, "按在两个按钮中间的缝上、标题行、屏外的坐标：都不算");
    clear(); tap(470, 315);
    CHECK(zone_is("ZONE 2 0") && scrs("\"ZONE 2\""), "按右边按钮：选 2 区");
    clear(); touch(240, 250, 1); Arm_Poll();
    CHECK(zone_is("ZONE 2 0") && scr[0] == 0, "START 只按下、还没松开：不算");
    touch(20, 310, 0); Arm_Poll();
    CHECK(zone_is("ZONE 2 1") && scrs("xstr 0,128,480,192,1,0,65504,1,1,1,\"GO\"") && scrs("xstr 0,84,480,40,0,65504,0,1,1,1,\"GO\"")
          && scrs("xstr 336,8,136,64,0,0,0,1,1,1,\"\"") && !scrs("cls"),
          "在 START 里按下、松开：s=1，START 换成 GO、状态行 GO、BACK 擦掉(不整页重画)");
    clear(); tap(400, 30); tap(240, 250); tap(100, 200);
    CHECK(zone_is("ZONE 2 1") && scr[0] == 0, "按了 START 以后不再收触摸(BACK、START、别处都不算)");
    clear();
    CHECK(run("ZONE ASK") == 1 && zone_is("ZONE 0 0") && scrs("sendxy=1|cls 0|") && scrs("START ZONE 1 OR 2"),
          "ZONE ASK：发 sendxy=1、重画选区页，清掉选的区和 START");
    clear();
    CHECK(run("ZONE 1") == 1 && zone_is("ZONE 1 0") && scrs("sendxy=1|cls 0|") && scrs("\"ZONE 1\"") && scrs("\"START\""),
          "ZONE 1：和在屏上按 1 一样，画准备页");
    tap(240, 250);
    CHECK(zone_is("ZONE 1 1"), "ZONE 1 以后屏上按 START：s=1");
    CHECK(run("ZONE 2") == 1 && zone_is("ZONE 2 0"), "ZONE 2：START 清掉(s=0)");
    touch(240, 250, 1); run("ZONE 2"); touch(240, 250, 0); Arm_Poll();
    CHECK(zone_is("ZONE 2 0"), "按下 START 以后页面重画了(换页)，再松开不算(按下的那一下属于上一页)");
    clear();
    CHECK(run("ZONE MSG SCAN 1/2") == 1 && scrs("xstr 0,84,480,40,0,65504,0,1,1,1,\"SCAN 1/2\"") && !scrs("cls"),
          "ZONE MSG：只写准备页的状态行(黄字)");
    clear(); run("ZONE MSG  a\"bcdefghijklmnopqrstuvwxyz");
    CHECK(scrs(",\"abcdefghijklmnopqrst\""), "ZONE MSG：去掉双引号和前面多的空格，最多 20 个字");
    clear(); run("ZONE MSG");
    CHECK(scrs("xstr 0,84,480,40,0,65504,0,1,1,1,\"\""), "ZONE MSG 不带字：清空状态行");
    run("ZONE MSG PLAN OK"); clear(); tap(400, 30);
    CHECK(scrs("START ZONE 1 OR 2") && scrs("xstr 240,0,240,48,0,65504,0,1,1,1,\"\""), "BACK 回选区页：状态行清空(在右上角)");
    clear(); run("ZONE MSG SCAN");
    CHECK(scrs("xstr 240,0,240,48,0,65504,0,1,1,1,\"SCAN\""), "选区页的状态行在右上角");
    clear();
    CHECK(run("ZONE LOCK") == 1 && scrs("cls 0|") && !scrs("xstr") && zone_is("ZONE 0 0"), "ZONE LOCK：清屏回到比赛布局");
    clear(); tap(100, 200); tap(400, 300);
    CHECK(zone_is("ZONE 0 0") && scr[0] == 0, "ZONE LOCK 以后屏上按什么都不算");
    clear(); run("ZONE MSG X");
    CHECK(scr[0] == 0, "比赛布局时 ZONE MSG 只记下，不画(不盖掉比赛的字)");
    run("ZONE 2"); tap(240, 250); run("ZONE LOCK");
    CHECK(zone_is("ZONE 2 1"), "ZONE LOCK 以后 ZONE? 还查得到选的区和 START");
    clear();
    CHECK(run("ZONE 3") == -1 && pis("ERR ARG") && run("ZONE") == -1 && run("ZONE ASK X") == -1 && run("ZONE FOO") == -1 && run("ZONE 1 2") == -1,
          "ZONE 参数不对：ERR ARG");
    /* 坏帧、半帧 */
    run("ZONE ASK");
    {
        static const uint8_t half[] = { 0x67, 0x00, 0x64, 0x00, 0xC8, 0x01, 0xFF, 0xFF };          /* 少一个 FF */
        static const uint8_t comp[] = { 0x65, 0x00, 0x01, 0x01, 0xFF, 0xFF, 0xFF };              /* 控件事件帧(不是坐标) */
        static const uint8_t err[]  = { 0x1A, 0xFF, 0xFF, 0xFF };                                /* 屏回的错误码 */
        static const uint8_t lng[]  = { 0x67, 0x00, 0x64, 0x00, 0xC8, 0x01, 0x00, 0xFF, 0xFF, 0xFF };   /* 长了一个字节 */
        static const uint8_t ff4[]  = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
        scr_in(half, (int)sizeof half); touch(100, 200, 0); Arm_Poll();
        CHECK(zone_is("ZONE 0 0"), "半帧(少一个 FF)：和后面的帧拼起来长度不对，一起丢掉");
        scr_in(comp, (int)sizeof comp); scr_in(err, (int)sizeof err); scr_in(lng, (int)sizeof lng); touch(100, 200, 0); Arm_Poll();
        CHECK(zone_is("ZONE 0 0"), "控件事件帧、错误码、长度不对的帧：都不当成触摸");
        scr_in(ff4, (int)sizeof ff4); tap(100, 200);
        CHECK(zone_is("ZONE 1 0"), "多余的 FF 以后：下一帧照样认得出(重新对上帧)");
    }
    {
        static const uint8_t sleepf[] = { 0x68, 0x01, 0xC0, 0x00, 0x96, 0x01, 0xFF, 0xFF, 0xFF };   /* 屏睡眠时的触摸帧 68，x=448 y=150 */
        static const uint8_t sleepr[] = { 0x68, 0x01, 0xC0, 0x00, 0x96, 0x00, 0xFF, 0xFF, 0xFF };
        run("ZONE ASK");
        scr_in(sleepf, 9); scr_in(sleepr, 9); Arm_Poll();
        CHECK(zone_is("ZONE 2 0"), "屏睡眠时的 68 帧也认(x=448 在右半边 = 2 区)");
    }
    /* USART2 出错：一定要重新开始接收 */
    run("ZONE ASK");
    {
        static const uint8_t part[] = { 0x67, 0x00, 0x64, 0x00 };
        int s0;
        scr_in(part, 4);
        s0 = scr_starts;
        scr_armed = 0;                              /* 溢出：HAL 停了接收，再调错误回调 */
        HAL_UART_ErrorCallback(&huart2);
        CHECK(scr_starts == s0 + 1 && scr_armed, "USART2 出错后错误回调里重新开始接收(不然以后再也按不动)");
        tap(100, 200);
        CHECK(zone_is("ZONE 1 0"), "出错时收了一半的帧丢掉，接着按照样能选");
        s0 = scr_lost;
        scr_armed = 0; touch(400, 200, 1);          /* 没开接收时来的字节收不到(假串口和真的一样) */
        CHECK(scr_lost == s0 + 9, "假串口：没开接收时收不到");
        HAL_UART_ErrorCallback(&huart2);
    }
    /* 屏重新上电(发来 88 FF FF FF)：sendxy 忘了、画面也没了 */
    {
        static const uint8_t boot[] = { 0x88, 0xFF, 0xFF, 0xFF };
        run("ZONE 1"); clear();
        scr_in(boot, 4); Arm_Poll();
        CHECK(scrs("sendxy=1|cls 0|") && scrs("\"ZONE 1\"") && scrs("\"START\"") && zone_is("ZONE 1 0"), "屏重新上电：再发 sendxy=1，整页重画(选的区不变)");
        run("ZONE LOCK"); clear();
        scr_in(boot, 4); Arm_Poll();
        CHECK(scr[0] == 0, "比赛布局时屏重新上电：不画选区页");
    }
    /* 旧的树莓派程序不发 ZONE LOCK：写 SCR 时自动回到比赛布局 */
    run("ZONE ASK"); clear();
    CHECK(run("SCR t1 QR SCAN") == 1 && scrs("cls 0|xstr 292,82,188,38,0,65535,0,0,1,1,\"QR SCAN\""),
          "选区页还在显示时写 SCR：先清屏回到比赛布局，再写字");
    clear(); tap(100, 200);
    CHECK(zone_is("ZONE 0 0") && scr[0] == 0, "SCR 自动回到比赛布局以后，触摸也关了");
    clear(); run("SCR t1 QR OK");
    CHECK(!scrs("cls"), "已经是比赛布局：SCR 不清屏");
    /* 选区页还在显示时扫到码：告诉树莓派，不写屏(不盖掉按钮) */
    run("ZONE ASK"); clear();
    qr_feed("999+888+777+666\r\n"); Arm_Poll();
    CHECK(pis("QR 999+888+777+666") && !scrs("999+888"), "选区页还在显示时扫到码：发给树莓派，但不写屏(不盖掉按钮)");
    run("QR CLR"); run("ZONE LOCK");
    /* USART2 接收开不起来：和以前一样显示 READY，终端里照样能选区 */
    scr_rxfail = 1; clear(); Arm_Init(); scr_rxfail = 0;
    CHECK(scrs("cls 0|") && scrs("\"READY\"") && !scrs("START ZONE") && !scrs("sendxy"), "收不到触摸(USART2 接收开不起来)：开机和以前一样清屏、显示 READY");
    scr_rxfail = 1;
    CHECK(zone_is("ZONE 0 0") && run("ZONE 2") == 1 && zone_is("ZONE 2 0"), "收不到触摸时终端里 ZONE 2 照样能选");
    scr_rxfail = 0;
    CHECK(run("ZONE ASK") == 1 && scr_armed, "开机没开起来的接收：ZONE ASK 时再试一次");
    tap(100, 200);
    CHECK(zone_is("ZONE 1 0"), "再试开起来以后屏上能选");
    clear(); Arm_Init(); run("ZONE LOCK");          /* 恢复：接收开着，比赛布局 */

    /* ---- 二维码在中断里锁存：长指令阻塞(主循环不跑 Arm_Poll)时扫到的码不丢 ---- */
    run("QR CLR"); clear();
    qr_feed("xx156+123+516+231\r\n");
    for (i = 0; i < 300; i++) qr_feed("x");          /* 后面一直来字节(缓冲满了好几次) */
    CHECK(run("QR?") == 1 && pis("QR 156+123+516+231"), "Arm_Poll 没跑时扫到的码在中断里已经锁存：后面的字节挤满缓冲也不丢，QR? 读得到");
    clear(); Arm_Poll();
    CHECK(pis("QR 156+123+516+231") && scrs("\"156+123+\"") && scrs("\"516+231\""), "主循环再写屏(t0 带 +)、告诉树莓派");
    clear(); Arm_Poll(); qr_feed("156+123+516+231\r\n"); Arm_Poll();
    CHECK(pi[0] == 0 && scr[0] == 0, "同一个码再扫到：不重复发、不重写屏");
    run("QR CLR"); clear();
    for (i = 0; i < 6; i++) qr_feed("452+321+254+312");   /* 不发换行的模块连着发(中间没有停 60 毫秒) */
    CHECK(run("QR?") == 1 && pis("QR 452+321+254+312"), "不发换行、连着发：缓冲满了清空前也在中断里找一遍");
    run("QR CLR"); clear();
    qr_feed("111+222+333+444\r\n");
    run("QR CLR"); Arm_Poll(); run("QR?");
    CHECK(pis("QR NONE") && !scrs("111+222"), "QR CLR 把中断里锁存了还没取走的码也清掉");
    clear(); qr_feed("12+345+678+90+1\r\n"); qr_feed("123+456+789+01\r\n"); Arm_Poll(); run("QR?");
    CHECK(pis("QR NONE"), "格式不对的行(组的位数不对)不锁存");

    /* ---- 二维码(画模式：屏工程里不用放控件) ---- */
    clear();
    {
        const char *code = "777+111+222+333\r\n";
        extern void Arm_Poll(void);
        const char *p;
        for (p = code; *p; p++) { *rxp = (uint8_t)*p; Arm_QR_RxCplt(); now += 1; }
        Arm_Poll();
    }
    CHECK(strstr(scr, "xstr 0,0,322,80,1,65535,0,1,1,1,\"777+111+\"") && strstr(scr, "xstr 0,80,288,80,1,65535,0,1,1,1,\"222+333\""),
          "画模式：任务码用大字(字库1)画在左上两行，组之间的 + 都在(t0 = 777+111+ 共 8 个大字 320 像素，框 322 宽)");
    clear(); run("SCR t1 RAW 1/3");
    CHECK(strstr(scr, "xstr 292,82,188,38,0,65535,0,0,1,1,\"RAW 1/3\""), "画模式：t1 阶段小字画在 t7 右边(188 宽，长的阶段文字放得下)");
    clear(); run("SCR t5 B1 R1 G2 B3");
    CHECK(strstr(scr, "xstr 326,2,154,38,0,65535,0,0,1,1,\"B1 R1 G2 B3\""), "画模式：t5 第一批小字画在右上(t0 右边)");
    clear(); run("SCR t9 X");
    CHECK(scr[0] == 0, "画模式：表里没有的名字不发");
    Arm_Param_Set("SCRMODE", 0);                    /* 下面测"写控件"模式 */

    /* ---- 二维码 ---- */
    clear();
    {
        const char *code = "xx452+321+254+312\r\n";
        extern void Arm_Poll(void);
        const char *p;
        for (p = code; *p; p++) { *rxp = (uint8_t)*p; Arm_QR_RxCplt(); now += 1; }
        Arm_Poll();
    }
    CHECK(pis("QR 452+321+254+312") && strstr(scr, "t0.txt=\"452+321+\"") && strstr(scr, "t7.txt=\"254+312\""), "扫到任务码：发给树莓派并分两行显示到串口屏 t0(带 +)、t7");
    clear();
    {
        const char *again = "452+321+254+312\r\n";
        const char *p;
        for (p = again; *p; p++) { *rxp = (uint8_t)*p; Arm_QR_RxCplt(); now += 1; }
        Arm_Poll();
    }
    CHECK(!pis("QR 452") && scr[0] == 0, "连续模式反复扫到同一个码：不重复发给树莓派、不重写屏");
    clear();
    {
        const char *other = "111+222+333+123\r\n";
        const char *p;
        for (p = other; *p; p++) { *rxp = (uint8_t)*p; Arm_QR_RxCplt(); now += 1; }
        Arm_Poll();
    }
    CHECK(pis("QR 111+222+333+123") && strstr(scr, "t0.txt=\"111+222+\""), "换了一个码：重新发、重新显示");
    clear(); {
        const char *back = "452+321+254+312\r\n"; const char *p;
        for (p = back; *p; p++) { *rxp = (uint8_t)*p; Arm_QR_RxCplt(); now += 1; }
        Arm_Poll();
    }
    clear();
    run("QR?");
    CHECK(pis("QR 452+321+254+312"), "QR? 回任务码");
    clear(); run("QR CLR"); run("QR?");
    CHECK(pis("QR NONE"), "QR CLR 清掉");
    clear();
    { const char *bad = "12+34\r\n"; const char *p; for (p = bad; *p; p++) { *rxp = (uint8_t)*p; Arm_QR_RxCplt(); } now += 100; Arm_Poll(); }
    run("QR?");
    CHECK(pis("QR NONE"), "格式不对的不当成任务码");
    clear();
    { const char *nt = "111+222+333+123"; const char *p; for (p = nt; *p; p++) { *rxp = (uint8_t)*p; Arm_QR_RxCplt(); now += 1; } now += 100; Arm_Poll(); }
    CHECK(pis("QR 111+222+333+123"), "没有换行的模块：60ms 没新字节也算收完");

    /* ---- 串口屏 ---- */
    clear();
    run("SCR t1 RAW GRAB 1/3 RED");
    CHECK(strstr(scr, "t1.txt=\"RAW GRAB 1/3 RED\""), "SCR");
    clear(); run("SCR t1 a\"b");
    CHECK(strstr(scr, "t1.txt=\"ab\""), "SCR 去掉双引号");
    clear(); run("SCMD page 1");
    CHECK(strstr(scr, "page 1"), "SCMD");
    CHECK(run("SCR") == 0 && run("SCR t1") == -1, "SCR 参数检查");
    run("ZONE 1"); clear();
    CHECK(run("ZONE LOCK") == 1 && scrs("page 0|") && !scrs("cls"), "写控件模式：ZONE LOCK 重新载入 page 0(控件都回来)");

    /* ---- 升降编码器：标定、开机自动找高度 ---- */
    run("LIFT 60"); phys_mm = 60.0;                      /* 让"程序以为的高度"和真实高度一致 */
    clear();
    CHECK(run("LIFT ENC?") == 1 && pis("LIFTENC ") && pis("NOCAL"), "没标定时 LIFT ENC? 只回原始读数");
    clear();
    CHECK(run("LIFT CAL 60") == 1 && pis("LIFTCAL OK") && pis("CPR=65536") && pis("SRC=31") && fake_flash[0] == 0x4C465432u, "LIFT CAL 60：测出编码器关系并存进 Flash");
    CHECK(fabs(phys_mm - 60.0) < 0.05, "标定完回到原来的高度");
    clear();
    phys_mm = 47.3;                                       /* 关机时被人推到 47.3mm */
    Arm_Init();
    CHECK((pis("LIFTBOOT 47.3") || pis("LIFTBOOT 47.29")) && has("L+1016;") && fabs(phys_mm - 60.0) < 0.05, "开机：编码器算出在 47.3mm，往上走 12.7mm 到正好 60");
    clear(); run("LIFT?");
    CHECK(pis("LIFT 1 60.0") && pis("BOOT=POS"), "开机用编码器、以 Flash 里记的上次位置为中心找到的：LIFT? 回 BOOT=POS");
    clear();
    phys_mm = 75.6;
    Arm_Init();
    CHECK((pis("LIFTBOOT 75.6") || pis("LIFTBOOT 75.59")) && has("L-1248;") && fabs(phys_mm - 60.0) < 0.05, "开机：在 75.6mm，往下走 15.6mm 到 60");
    clear();
    CHECK(run("LIFT ENC?") == 1 && (pis("Z=60.0") || pis("Z=59.99")) && pis("NOW=60.0"), "LIFT ENC? 算出的高度和程序记的一致");
    /* 编码器一圈是 16384 的驱动器、方向也反过来 */
    enc_cpr = 16384.0; enc_cpm = -409.6; enc_off = 3000.0; phys_mm = 30.0;
    run("LIFT 30"); phys_mm = 30.0;
    clear();
    CHECK(run("LIFT CAL 30") == 1 && pis("CPR=16384"), "一圈 16384、方向相反的编码器也能标定");
    run("LIFT 60");                                      /* 断电前停在 60(程序记下了) */
    clear();
    phys_mm = 68.0;
    Arm_Init();
    CHECK((pis("LIFTBOOT 68.0") || pis("LIFTBOOT 67.99")) && has("L-640;") && fabs(phys_mm - 60.0) < 0.05, "16384 编码器：开机在 68mm，往下 8mm 到 60");
    /* 读不到编码器：当作在 60，不动 */
    enc_ok = 0; clear(); phys_mm = 52.0;
    Arm_Init();
    CHECK(pis("LIFTBOOT NOENC") && !has("L+") && !has("L-"), "读不到编码器：不动，当作在 60");
    clear(); run("LIFT?");
    CHECK(pis("LIFT 1 60.0") && pis("BOOT=NOENC"), "读不到编码器：LIFT? 回 BOOT=NOENC(高度是假设的)");
    CHECK(run("LIFT CAL 60") == -1 && pis("ERR NOENC"), "读不到编码器时 LIFT CAL 报 ERR NOENC");
    enc_ok = 1; enc_frozen = 1; enc31_zero = 1; pos36_off = 0.0; clear();
    { double save = phys_mm; phys_mm = 0.0; run("LIFT 0"); phys_mm = 0.0; (void)save; }
    flash_blank();
    Arm_Init(); clear();                                  /* 重新上电：Flash 空了，程序里也变成没标定 */
    CHECK(run("LIFT ENC?") == 1 && pis("31=0 36=0 NOCAL"), "没标定时 LIFT ENC? 两种读数都打出来");
    /* 现场情况：0x31 一直回 0。程序自动改用 0x36 实时位置；在最低点标定只往上走、不往下撞 */
    enc_frozen = 0; pos36_off = -70000.0; clear();
    CHECK(run("LIFT CAL 0") == 1 && pis("LIFTCAL OK") && pis("SRC=36") && pis("CPR=65536"), "0x31 总是 0：自动改用 0x36 标定成功");
    CHECK(has("L+800;") && !has("L-800;L-") && fabs(phys_mm) < 0.05, "最低点标定：先往上 10mm 再回到 0，不往下走");
    run("LIFT 60"); phys_mm = 55.5; clear(); Arm_Init();
    CHECK((pis("LIFTBOOT 55.5") || pis("LIFTBOOT 55.49")) && fabs(phys_mm - 60.0) < 0.05, "用 0x36 标定后开机：从 55.5 自动走到 60");
    phys_mm = 79.0; clear(); Arm_Init();
    CHECK(fabs(phys_mm - 60.0) < 0.05 && has("L-1520;"), "0x36：开机在 79mm 往下走到 60");
    clear();
    CHECK(run("LIFT ENC?") == 1 && pis("SRC=36") && (pis("Z=60.0") || pis("Z=59.99")), "LIFT ENC? 显示用的是 0x36");
    pos36_off = -1e9; phys_mm = 60.0; enc_cpr = 65536.0; enc_cpm = 1638.4;
    enc31_zero = 0; enc_frozen = 1; clear();
    CHECK(run("LIFT CAL 60") == -1 && pis("ERR ENCMOVE") && pis("LIFTCAL RAW"), "两种读数都对不上：ERR ENCMOVE，打印原始读数，不存");
    enc_frozen = 0; pos36_off = -70000.0;
    enc_frozen = 0; flash_fail = 1; enc_cpr = 65536.0; enc_cpm = 1638.4; clear();
    run("LIFT 60"); phys_mm = 60.0;
    CHECK(run("LIFT CAL 60") == -1 && pis("ERR FLASH"), "Flash 写失败报 ERR FLASH");
    flash_fail = 0;
    CHECK(run("LIFT CAL 120") == -1 && pis("ERR RANGE"), "LIFT CAL 超出 0~LFMAX 拒绝");
    /* 没标定(Flash 是空的)时开机：当作在 60，不动 */
    flash_blank();
    clear(); Arm_Init();
    CHECK(pis("LIFTBOOT NOCAL") && !has("L+") && !has("L-"), "没标定：开机不动，当作在 60");
    clear(); run("LIFT?");
    CHECK(pis("LIFT 1 60.0") && pis("BOOT=NOCAL"), "没标定：LIFT? 回 BOOT=NOCAL(高度是假设的)");
    run("LIFT 60");

    /* ---- 记住上次停下的位置：断电时停在哪都行 ---- */
    enc_cpr = 65536.0; enc_cpm = 1638.4; enc_off = 12345.0; enc31_zero = 0; enc_frozen = 0;
    flash_blank(); Arm_Init(); run("LIFT CAL 60"); phys_mm = 60.0;
    CHECK(pis("LIFTCAL OK"), "重新标定");
    run("LIFT 10");
    CHECK(fabs(phys_mm - 10.0) < 0.05, "走到 10mm");
    clear(); Arm_Init();                                  /* 断电时停在 10mm(离 60 有 50mm，以前找不回来) */
    CHECK(pis("LIFTBOOT 10.0") && pis("last 10.0") && fabs(phys_mm - 60.0) < 0.05, "上次停在 10mm：开机从 10 找到并走到 60");
    run("LIFT 95"); clear(); Arm_Init();
    CHECK(pis("LIFTBOOT 95.0") && fabs(phys_mm - 60.0) < 0.05, "上次停在 95mm：开机走到 60");
    run("LIFT 20"); phys_mm = 33.0; clear(); Arm_Init();   /* 断电后被人往上推了 13mm */
    CHECK((pis("LIFTBOOT 33.0") || pis("LIFTBOOT 32.99")) && fabs(phys_mm - 60.0) < 0.05, "断电后被推了 13mm 也能找回");
    run("LIFT 5"); phys_mm = 32.0; clear(); Arm_Init();    /* 推了 27mm：按上次位置算出 -8mm，在行程外，挪一整圈到 32 */
    CHECK((pis("LIFTBOOT 32.0") || pis("LIFTBOOT 31.99")) && fabs(phys_mm - 60.0) < 0.05, "算到最低点以下时自动挪一圈");
    /* 急停打断升降：用编码器把高度找回来，不用重新回零 */
    run("LIFT 60"); car_abort = 1; clear();
    run("LIFT 80"); clear(); run("LIFT?");
    CHECK(pis("LIFT 1 80.0"), "急停打断后用编码器找回高度");
    /* 记录快满了：开机时擦掉重来，标定还在 */
    for (i = 0; i < 40; i++) { run("LIFT 50"); run("LIFT 70"); }
    erases = 0; clear(); Arm_Init();
    CHECK(erases == 1 && fake_flash[0] == 0x4C465432u && fabs(phys_mm - 60.0) < 0.05, "位置记录快满：开机擦一次扇区，标定保留");
    run("LIFT 30"); clear(); Arm_Init();
    CHECK(pis("LIFTBOOT 30.0") && fabs(phys_mm - 60.0) < 0.05, "擦完以后照样记位置");
    run("LIFT 60");
    /* Flash 里只有标定、没有位置记录：在 60 上下找 */
    for (i = 64; i < 256; i++) fake_flash[i] = 0xFFFFFFFFu;   /* 位置记录从 0x100 字节(第 64 个字)开始 */
    phys_mm = 70.0; clear(); Arm_Init();
    CHECK((pis("LIFTBOOT 70.0") || pis("LIFTBOOT 69.99")) && pis("last 60.0") && fabs(phys_mm - 60.0) < 0.05, "没有位置记录：在 60 上下找到 70，走回 60");
    clear(); run("LIFT?");
    CHECK(pis("LIFT 1 60.0") && pis("BOOT=ENC"), "没有位置记录、编码器找到的：LIFT? 回 BOOT=ENC");
    clear(); run("LIFT ZERO"); run("LIFT?");
    CHECK(pis("LIFT 1 0.0") && pis("BOOT=ENC"), "BOOT= 只说开机时怎么认的，之后 LIFT ZERO 也不变");
    run("LIFT 60"); phys_mm = 60.0;

    /* ---- GET/SET 用的接口 ---- */
    clear();
    Arm_Param_Dump();
    CHECK(pis("P ZPLC=80.0000") && pis("P LFMAX=105.0000") && pis("P ARMOK=1.0000") && pis("P ASPD=90.0000"), "参数全部打印");

    (void)i;
    printf("%s: %d 项检查，%d 项失败\n", fails ? "失败" : "通过", checks, fails);
    return fails ? 1 : 0;
}
