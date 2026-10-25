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
static uint32_t t_up1 = 0, t_s1 = 0, t_s2 = 0;   /* (10-24)clear() 以后第一条往上的升降指令、第一次转 ID1 / ID2 的时刻(0 = 还没有) */
static int abort_at = -1;
static float ang[3] = {0, 323.0f, -862.0f};
/* (10-23)slow = 1：舵机按发的速度慢慢转(时间在 HAL_Delay 里走)，测 PRE 发完指令就回、不等转完；0 = 一发就到(以前的测试) */
static int slow = 0;
static float sgoal[3], sspd[3];
static void servo_tick(uint32_t ms) {
    int id;
    if (!slow) return;
    for (id = 1; id <= 2; id++) {
        float d = sgoal[id] - ang[id], st = sspd[id] * (float)ms / 1000.0f;
        if (d > st) ang[id] += st; else if (d < -st) ang[id] -= st; else ang[id] = sgoal[id];
    }
}
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
static void lift_tick(uint32_t ms);
void HAL_Delay(uint32_t ms) { now += ms; if (abort_at >= 0 && (int)now >= abort_at) car_abort = 1; lift_tick(ms); servo_tick(ms); }
void Delay_Report(uint32_t ms) { HAL_Delay(ms); }
static char qrtx[2048];                /* 发给扫码模块(UART5)的字节，十六进制 "7E 00 07 ..." 每条后面加 | */
int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *b, uint16_t n, uint32_t t) {
    (void)t;
    if (h == &huart5) { uint16_t k; for (k = 0; k < n && strlen(qrtx) < sizeof qrtx - 8; k++) { char x[4]; snprintf(x, sizeof x, k ? " %02X" : "%02X", b[k]); strcat(qrtx, x); } strcat(qrtx, "|"); }
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
void tim_set(uint32_t ch, uint32_t v) { if (ch == TIM_CHANNEL_3) EV("pwm3=%u;", (unsigned)v); }   /* 开机时夹爪的初始脉宽 */
/* 夹爪/转盘的脉宽：是默认的张开/夹紧/1~3 号位就记成 O; C; tt1; …，方便看先后顺序 */
static uint32_t t_tt = 0, t_close = 0;                   /* 最近一次转盘换脉宽、夹爪夹紧(2910)的时刻 */
void Claw_Set(uint32_t p) { if (p == 2090u) EV("O;"); else if (p == 2910u) { EV("C;"); t_close = now; } else EV("claw%u;", p); }
void Turntable_Set(uint32_t p) { static uint32_t last = 0; if (p != last) t_tt = now; last = p;
    if (p == 2608u) EV("tt1;"); else if (p == 1708u) EV("tt2;"); else if (p == 808u) EV("tt3;"); else EV("ttus%u;", p); }
int Servo_ReadAngle(uint8_t id, float *a) { *a = ang[id]; return 1; }
static void away(void);
static float block1 = 1e9f;                              /* ID1 被挡住的角度(转不过去)，1e9 = 没挡 */
void Servo_StopIfStuck(uint8_t id, float t) { EV("stuck%u:%.1f;", id, t); }
void Servo_Report(uint8_t id) { EV("rep%u;", id); }
void Servo_SetPower(uint16_t mw) { EV("pow%u;", mw); }
void Servo_SetComp(uint8_t id, float deg) { EV("comp%u=%.1f;", id, deg); }
void Servo_SetLimits(uint8_t id, float lo, float hi) { EV("lim%u=%.0f,%.0f;", id, lo, hi); }
void Servo_Hold(uint8_t id, float a) { EV("H%u:%.1f;", id, a); if (slow) sgoal[id] = a; }
void Servo_Track(uint8_t id, float a, float spd) { EV("T%u:%.1f@%.0f;", id, a, spd); }
static float fgoal[3];
int Servo_Arrived(uint8_t id, float a, float tol) { float e = a - fgoal[id]; return (e < 0 ? -e : e) <= tol; }
void Servo_Params(uint8_t id) { EV("svp%u;", id); }
int Servo_WriteParam(uint8_t id, const char *name, long v) {
    EV("svw%u:%s=%ld;", id, name, v);
    if (strcmp(name, "PMAX") != 0 && strcmp(name, "IMAX") != 0) return 0;
    if (v < 0 || v > 65535) return 2;
    return v == 4242 ? -1 : 1;
}
static float clampa(uint8_t id, float a) { float lo = id == 1 ? 44.0f : -1220.0f, hi = id == 1 ? 440.0f : -300.0f; return a < lo ? lo : (a > hi ? hi : a); }
uint32_t Servo_Start(uint8_t id, float *a, float spd) {
    float d; *a = clampa(id, *a); fgoal[id] = *a; d = *a - ang[id]; if (d < 0) d = -d;
    if (id == 1 && !t_s1) t_s1 = now ? now : 1;
    if (id == 2 && !t_s2) t_s2 = now ? now : 1;
    if (slow) { sgoal[id] = *a; sspd[id] = spd; } else ang[id] = (id == 1 && *a > block1) ? block1 : *a;
    EV("S%u:%.1f@%.0f;", id, *a, spd); return (uint32_t)(d / spd * 1000) + 200;
}
void Servo_Move(uint8_t id, float a, float spd, float tol, uint32_t extra) {
    (void)extra; a = clampa(id, a); EV("M%u:%.1f@%.0f/%.2f;", id, a, spd, tol); ang[id] = a; sgoal[id] = a; now += 300;
}
static void away(void) { ang[1] = 300.0f; ang[2] = -700.0f; }   /* 手臂先摆到别处，下一个动作一定要转 */
void Emm_V5_En_Control(uint8_t a, bool s, bool f) { (void)a; (void)s; (void)f; }
void Emm_V5_Reset_CurPos_To_Zero(uint8_t a) { EV("zero%u;", a); }
/* 假升降：真实高度 phys_mm(LFDIR=0 时 d=0 往上)，编码器 = (enc_off + 每毫米 enc_cpm × 高度) 对一圈 enc_cpr 取余 */
static double phys_mm = 60.0, enc_off = 12345.0, enc_cpm = 1638.4, enc_cpr = 65536.0;
static int enc_ok = 1, enc_frozen = 0, enc31_zero = 0;   /* enc31_zero：像现场那样 0x31 总回 0，只有 0x36 实时位置能用 */
static double pos36_off = -70000.0;                      /* 0x36 实时位置(一圈 65536，多圈累计，可以是负数) */
/* (10-21)机构真正能到的最高/最低：要去的高度超出去就停在那(丢步)；late_mm：走得比估的慢，等完时还差这么多没走，之后每 80ms 走掉 1/late_n */
static double phys_top = 1e9, phys_bot = -1e9, late_mm = 0.0, late_left = 0.0;
static int lift_dead = 0;                 /* 1 = 升降驱动器收到指令也不动(堵转保护、丢了帧)：编码器照旧读 */
static int late_n = 3, late_steps = 0;
/* (10-25)升降按发的速度慢慢走(转/分 × 40mm 一圈)，时间在 HAL_Delay 里走：Lift_Wait_Watch 一路读编码器，要能看到它在走、碰到底停住 */
static double mv_goal = 60.0, mv_v = 0.0;                /* 正在去的高度、速度 mm/s(0 = 没在走) */
static double mv_slow = 1.0;                             /* 实际速度是发的几倍(< 1 = 比估的慢) */
static int mv_clamped = 0;                               /* 1 = 要去的地方超出机构能到的(会顶住) */
static uint32_t late_acc = 0, t_block = 0, t_stop = 0;   /* 顶住(碰到底/顶)的时刻、清零以后第一次叫升降停住的时刻 */
static void lift_tick(uint32_t ms) {
    double q, st;
    if (mv_v > 0.0) {
        st = mv_v * (double)ms / 1000.0;
        if (fabs(mv_goal - phys_mm) <= st) { phys_mm = mv_goal; mv_v = 0.0; if (mv_clamped) t_block = now; }
        else phys_mm += (mv_goal > phys_mm) ? st : -st;
    }
    if (mv_v > 0.0 || late_steps <= 0) { late_acc = 0; return; }
    for (late_acc += ms; late_acc >= 80 && late_steps > 0; late_acc -= 80) { q = late_left / late_steps; phys_mm += q; late_left -= q; late_steps--; }
}
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
    double t, from;
    (void)acc; (void)r; (void)f; EV("L%s%u;", d ? "-" : "+", clk);
    if (!d && !t_up1) t_up1 = now ? now : 1;
    if (a != 5 || lift_dead) return;
    from = (mv_v > 0.0) ? mv_goal : phys_mm;              /* 相对上一次要去的地方(Emm 的相对运动) */
    t = from + (d ? -1.0 : 1.0) * (double)clk / 80.0;
    mv_clamped = (t > phys_top || t < phys_bot);
    if (t > phys_top) t = phys_top;
    if (t < phys_bot) t = phys_bot;
    if (late_mm > 0.0) { late_left = (t > phys_mm) ? late_mm : -late_mm; late_steps = late_n; late_acc = 0; t -= late_left; }
    mv_goal = t; mv_v = (double)v / 60.0 * 40.0 * mv_slow;
    if (mv_v <= 0.0) { phys_mm = t; mv_v = 0.0; }
}
void Emm_V5_Stop_Now(uint8_t a, bool f) { (void)f; EV("stop%u;", a); if (a == 5) { mv_v = 0.0; late_steps = 0; late_left = 0.0; if (!t_stop) t_stop = now; } }
void Emm_V5_Origin_Trigger_Return(uint8_t a, uint8_t m, bool f) { (void)f; EV("home%u/%u;", a, m); }
void Emm_V5_Origin_Interrupt(uint8_t a) { EV("homeint%u;", a); }

static int run(const char *c) {
    char err[24] = "";
    int r = Arm_Command(c, err, sizeof err);
    if (r < 0) { strncat(pi, err, 20); strcat(pi, "!"); }
    car_abort = 0;
    return r;
}
static void clear(void) { ev[0] = 0; pi[0] = 0; scr[0] = 0; t_up1 = t_s1 = t_s2 = 0; }
static int at(const char *needle) { const char *p = strstr(ev, needle); return p ? (int)(p - ev) : -1; }
static int has(const char *s) { return strstr(ev, s) != 0; }
static int last(const char *needle) { int r = -1; const char *p = ev; while ((p = strstr(p, needle)) != 0) { r = (int)(p - ev); p++; } return r; }
static int pis(const char *s) { return strstr(pi, s) != 0; }
static double pinum(const char *key) { const char *p = strstr(pi, key); return p ? atof(p + strlen(key)) : -1e9; }   /* 回复里 key 后面的数 */
static int scrs(const char *s) { return strstr(scr, s) != 0; }
/* 屏发来字节：和真的串口一样，开着接收(HAL_UART_Receive_IT)才收得到，收到一个以后要回调里重新开 */
static void scr_in(const uint8_t *f, int n) {
    int i;
    for (i = 0; i < n; i++) {
        if (!scr_armed) { scr_lost++; continue; }
        scr_armed = 0; *scrp = f[i]; HAL_UART_RxCpltCallback(&huart2);
    }
}
/* 触摸帧：67 XH XL YH YL 事件 FF FF FF(事件 1 = 按下，0 = 松开)。
 * 按下之前先过 touch_gap 毫秒(默认 600，比换页后的保护时间 0.5 秒长：人正常地看一眼再按) */
static uint32_t touch_gap = 600;
static void touch(int x, int y, int ev) {
    uint8_t f[9];
    if (ev == 1) now += touch_gap;
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
    CHECK(scrs("cls 0|") && !scrs("xstr") && !scrs("START ZONE") && !scrs("READY") && !scrs("sendxy"),
          "(10-23 用户\"不需要显示启停区1还是2\")开机只清屏：不再显示选区页、不写 READY");
    clear();
    CHECK(run("ZONE?") == 1 && pis("ZONE 0 0"), "开机还没选区：ZONE 0 0");

    /* ---- 保护 ---- */
    clear();
    CHECK(run("GRAB 1") == -1 && pis("ERR NOCAL"), "ARMOK=0 时拒绝");
    CHECK(run("OBS RAW O") == -1, "ARMOK=0 时 OBS 拒绝");
    Arm_Param_Set("ARMOK", 1);
    Arm_Param_Set("OVLAP", 0);                         /* 下面老的测试按"一步一步做"写的；边升边转的单独测(文件末尾) */
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
    CHECK(Arm_Param_Set("LFMAX", 150) == 2 && Arm_Param_Set("LFMAX", 120) == 1, "最高点上限 130mm：SET LFMAX 150 拒绝、120 可以");
    Arm_Param_Set("LFMAX", 115);
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

    /* ---- (10-19 用户：电池盒挡着)转去转盘：转到 A1T1 -> 伸到 A2T -> 转到 A1T2 -> 缩回 A2R -> 转到 A1D；离开反过来 ---- */
    Arm_Param_Set("A1T1", 150); Arm_Param_Set("A2T", -800); Arm_Param_Set("A1T2", 280); Arm_Param_Set("A2SPD", 400);
    away(); clear();
    CHECK(run("GRAB 2 H") == 1, "A2T：GRAB 2 H");
    {
        int td = at("S1:323.0"), ext = at("S2:-800.0@400"), back = at("S2:-862.0@400");
        CHECK(td >= 0 && ext > td && back > ext && !has("M1:150.0") && !has("M1:280.0"),
              "ID1 一口气转到转盘(不停)，路上伸到 A2T、再缩回 A2R(伸缩按 A2SPD)");
    }
    Arm_Param_Set("A1G", 100);                     /* 原料盘在另一边 */
    clear();
    CHECK(run("OBS RAW") == 1, "A2T：OBS RAW");
    {
        int sw = at("S1:100.0"), to = at("S2:-862.0");
        CHECK(sw >= 0 && to > sw && !has("S2:-800.0"), "回原料区：ID1 一直转，路上伸缩直接到 A2E(不先伸到 A2T)");
    }
    clear();
    CHECK(run("AFS 2 -900 500") == 1 && has("M2:-900.0@500") && run("AFS 2 -900 0") == -1, "AFS：按给定速度转");
    Arm_Param_Set("A2T", 0); Arm_Param_Set("A1T1", 0); Arm_Param_Set("A1T2", 0); Arm_Param_Set("A2SPD", 0); Arm_Param_Set("A1G", 323);
    away(); clear(); run("GRAB 2 H");
    CHECK(!has("S2:-800.0") && !has("M2:-800.0"), "A2T=0：照旧");

    /* ---- (10-23 用户"原料区域抓取物料之后再新加上一个往回缩的动作，怕爪子挡住其他物料")A2GB：
     *      原料盘上夹紧以后先抬起、缩回到 A2GB(ID1 不转)，再去车上转盘；圆环那边(PICK)不缩；A2GB=0 照旧 ---- */
    Arm_Param_Set("A2GB", -1100);
    away(); clear();
    CHECK(run("GRAB 1 H") == 1, "A2GB：GRAB 1 H");
    {
        int close = at("C;"), hold = at("H1:300.0"), back = at("S2:-1100.0"), turn = at("S1:");
        CHECK(close >= 0 && hold > close && back > close && turn > back && turn > hold,
              "A2GB：夹紧 -> ID1 不转、伸缩缩回到 A2GB -> 才转去车上转盘");
        CHECK(at("L-") > close && at("L-") < back, "A2GB：先直着抬起离开原料盘，再缩回");
    }
    away(); clear(); run("PICK 1 H");
    CHECK(!has("S2:-1100.0") && !has("M2:-1100.0"), "A2GB：圆环那边(PICK)不缩");
    Arm_Param_Set("A2GB", 0);
    away(); clear(); run("GRAB 1 H");
    CHECK(!has("S2:-1100.0") && !has("M2:-1100.0"), "A2GB=0：照旧");

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
    Arm_Param_Set("A1P", 200);                     /* 圆环姿态和转盘姿态不一样，ID1 才会转 */
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
    clear(); CHECK(run("APN 330 -850") == 1 && has("T1:330.0@90;") && has("T2:-850.0@90;") && !has("S1:") && !has("H1"), "APN：只发目标不等(默认 ASPD)，不用 Servo_Start(不补偿)");
    CHECK(pis("ANG 1") && pis("ANG 2"), "APN 回报现在的两个角度");
    ang[1] = 320.0f; ang[2] = -800.0f;
    clear(); CHECK(run("APN 330 -850 500") == 1 && has("T1:330.0@20;") && has("T2:-850.0@100;"), "APN 500ms 到：按离目标多远算转速(10°→20°/s，50°→100°/s)");
    clear(); CHECK(run("APN 320.1 -100 1") == 1 && has("T1:320.1@100;") && has("T2:-100.0@900;"), "APN 转速限在 1~900");
    CHECK(run("APN 330") != 1 && run("APN 330 -850 0") != 1, "APN 参数不对：不执行");

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
    /* (10-18)舵机限位 S1MIN/S1MAX/S2MIN/S2MAX：开机交给 main.c；SET 马上换；下限不能比上限大 */
    clear(); Arm_Init();
    CHECK(has("lim1=44,244;") && has("lim2=-1210,-300;"), "开机把限位 S1MIN/S1MAX、S2MIN/S2MAX 交给 main.c");
    clear();
    CHECK(Arm_Param_Set("S2MAX", -150) == 1 && has("lim2=-1210,-150;"), "SET S2MAX 马上换 ID2 限位");
    CHECK(Arm_Param_Set("S2MAX", -1300) == 2 && Arm_Param_Set("S2MIN", -100) == 2 && Arm_Param_Set("S1MIN", 300) == 2,
          "下限比上限大(或上限比下限小)就拒绝");
    Arm_Param_Set("S2MAX", -300);
    /* (10-18 用户：齿条重装、ID1 放开)姿态角度不再限在 -1210~-300 / 44~244，S1MIN 可以是负的(真转的时候 main.c 按限位截住) */
    CHECK(Arm_Param_Set("A2E", -1400) == 1 && Arm_Param_Set("A2R", -250) == 1 && Arm_Param_Set("A2P", -1500) == 1 &&
          Arm_Param_Set("A1G", 30) == 1 && Arm_Param_Set("A1D", 260) == 1 && Arm_Param_Set("S1MIN", -20) == 1 && has("lim1=-20,244;"),
          "A2E/A2R/A2P、A1G/A1D 超出以前的范围也能设，S1MIN 能设负数");
    CHECK(Arm_Param_Set("A2E", -6000) == 2 && Arm_Param_Set("A1G", 800) == 2, "太离谱的数还是拒绝");
    Arm_Param_Set("S1MIN", 44); Arm_Param_Set("A2E", -862); Arm_Param_Set("A2R", -862); Arm_Param_Set("A2P", -862);
    Arm_Param_Set("A1G", 323); Arm_Param_Set("A1D", 323);
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
    {   /* CLSPD：夹爪慢慢转过去(1999 -> 2777 差 778 微秒，1000 微秒/秒 约 0.78 秒，中间经过好几个位置)；CLSPD 0 一下子到 */
        uint32_t t0;
        CHECK(Arm_Param_Set("CLSPD", 1000) == 1, "CLSPD 可以设");
        run("CLAW O"); clear(); t0 = now; run("CLAW C");
        CHECK(has("claw2009;") && has("claw2777;") && now - t0 >= 700u + (uint32_t)400, "CLSPD 1000：夹爪一点点转过去，约 0.8 秒(再加 CLWAIT)");
        Arm_Param_Set("CLSPD", 0);
        Arm_Param_Set("CLSPO", 0);
        run("CLAW O"); clear(); t0 = now; run("CLAW C");
        CHECK(!has("claw2009;") && has("claw2777;") && now - t0 < 500u, "CLSPD 0：和以前一样一下子到");
        /* 张开用 CLSPO、合上用 CLSPD：张开慢(1100)、合上一下子(0) */
        Arm_Param_Set("CLSPO", 1100);
        clear(); t0 = now; run("CLAW O");
        CHECK(has("claw2766;") && has("claw1999;") && now - t0 >= 600u + 400u, "CLSPO 1100：张开慢慢转(约 0.7 秒)");
        clear(); t0 = now; run("CLAW C");
        CHECK(!has("claw2009;") && has("claw2777;") && now - t0 < 500u, "合上用 CLSPD(0)：一下子到");
        Arm_Param_Set("CLSPO", 0);
    }
    CHECK(Arm_Param_Set("TT2", 1650) == 1 && Arm_Param_Set("TT3", 400) == 2, "TT1~3 可以改，超出转盘限位拒绝");
    clear(); run("TT 2"); run("TT 1");
    CHECK(has("ttus1650;") && has("tt1;"), "TT 2 用 TT2 的脉宽");
    Arm_Param_Set("CLWO", 2090); Arm_Param_Set("CLWC", 2910); Arm_Param_Set("TT2", 1708);
    /* (10-21 用户)夹爪张开只两种：原料盘上方 CLWOR，其他所有地方都和车上转盘上方一样用 CLWOT；CLWOT = 0 用 CLWO，CLWOR = 0 和其他地方一样 */
    CHECK(Arm_Param_Set("CLWOR", 300) == 2 && Arm_Param_Set("CLWOT", 3000) == 2 && Arm_Param_Set("CLWOR", 0) == 1,
          "CLWOR/CLWOT：0 = 用 CLWO，其他要在夹爪限位 500~2910 里面");
    away(); clear(); run("GRAB 2 H"); run("CLAW OR"); run("CLAW OT");
    CHECK(!has("claw") && at("O;") >= 0, "CLWOR/CLWOT 没调(0)：GRAB 张开、松开、CLAW OR/OT 都和以前一样用 CLWO");
    CHECK(Arm_Param_Set("CLWOR", 1500) == 1 && Arm_Param_Set("CLWOT", 1700) == 1, "CLWOR/CLWOT 可以设");
    clear(); run("CLAW OR"); run("CLAW OT"); run("CLAW O");
    CHECK(at("claw1500;") >= 0 && at("claw1500;") < at("claw1700;") && !has("O;"), "CLAW OR 用 CLWOR；CLAW OT、CLAW O 都用 CLWOT");
    away(); clear(); run("OBS RAW O");
    CHECK(has("claw1500;") && !has("O;"), "OBS RAW O：原料盘上方张开用 CLWOR");
    away(); clear(); run("OBS RING O");
    CHECK(has("claw1700;") && !has("claw1500;") && !has("O;"), "OBS RING O：圆环那边张开也用 CLWOT(和车上转盘一样)");
    away(); clear(); run("GRAB 2 H");
    {
        int open0 = at("claw1500;"), close = at("C;"), rel = at("claw1700;");
        CHECK(open0 >= 0 && open0 < close && close < rel && !has("O;"), "GRAB H：下爪前张开用 CLWOR，松开放进转盘用 CLWOT");
    }
    away(); clear(); run("PICK 1 H");
    CHECK(at("C;") >= 0 && !has("O;") && !has("claw1500;"), "PICK H(圆环取回)：下爪前、放进转盘都用 CLWOT");
    away(); clear(); run("TAKE 3");
    {
        const char *q = strstr(ev, "claw1700;");
        CHECK(q && at("C;") > (int)(q - ev) && at("L+4800;") > (int)(q - ev), "TAKE：到转盘上方换成 CLWOT 再下去夹");
    }
    clear(); run("DROP");
    CHECK(has("claw1700;") && !has("O;") && !has("claw1500;"), "DROP：地上圆环松开也用 CLWOT");
    Arm_Param_Set("CLWOR", 0); Arm_Param_Set("CLWOT", 0);
    /* (10-21 用户：伸缩的舵机要最快、旋转的慢一点)大动作转速分开：伸缩 EXTSPD、旋转 ROTSPD，0 = ASPD */
    CHECK(Arm_Param_Set("ROTSPD", 5) == 2 && Arm_Param_Set("EXTSPD", 2000) == 2, "ROTSPD/EXTSPD：0 或 10 以上，EXTSPD 最多 1500");
    CHECK(Arm_Param_Set("ROTSPD", 30) == 1 && Arm_Param_Set("EXTSPD", 1500) == 1, "ROTSPD/EXTSPD 可以设");
    away(); clear(); run("OBS RING");
    CHECK(has("S1:323.0@30;") && has("S2:-862.0@1500;"), "OBS：ID1(旋转)按 ROTSPD、ID2(伸缩)按 EXTSPD");
    clear(); run("DROP");
    CHECK(has("M2:-862.0@1500/1.00;"), "DROP 缩回伸缩舵机按 EXTSPD");
    Arm_Param_Set("PARA", 0);
    away(); clear(); run("STOW");
    CHECK(has("M2:-862.0@1500/") && has("M1:323.0@30/"), "PARA=0 一个一个转也各按各的速度");
    Arm_Param_Set("PARA", 1);
    Arm_Param_Set("AEXT", 1);
    away(); clear(); run("OBS RING");
    CHECK(has("S1:323.0@1500;") && has("S2:-862.0@30;"), "AEXT=1：ID1 是伸缩舵机，按 EXTSPD");
    Arm_Param_Set("AEXT", 2);
    Arm_Param_Set("ROTSPD", 0); Arm_Param_Set("EXTSPD", 0);
    /* (10-21 用户：从转盘里取了物料，要先转一个角度再伸长，不然挡到电池盒)LVDEG：夹着物料离开车上转盘先只转 ID1 */
    Arm_Param_Set("A1P", 200);
    CHECK(Arm_Param_Set("LVDEG", 25) == 1 && Arm_Param_Set("LVDEG", 200) == 2, "LVDEG 可以设，最多 120");
    away(); clear(); run("TAKE 1"); clear();
    CHECK(run("APZ 200 -700 60") == 1, "APZ");
    CHECK(at("M1:298.0") >= 0 && at("M1:298.0") < at("S2:") && !has("M2:"), "夹着物料离开转盘：ID1 先只转 25°(伸缩不动)，再一起转、伸缩");
    run("CLAW O"); away(); ang[1] = 323.0f; ang[2] = -862.0f; clear();
    run("APZ 200 -700 60");
    CHECK(!has("M1:298.0"), "爪子张着(没夹物料)：不先转");
    Arm_Param_Set("LVDEG", 0);
    away(); clear(); run("TAKE 1"); clear(); run("APZ 200 -700 60");
    CHECK(!has("M1:298.0") && !has("M1:303.0"), "LVDEG=0：和以前一样");
    Arm_Param_Set("LVDEG", 0);
    /* (10-21)圆环那组分段走(R2STG=1；默认 0 = ID1 一直转，用户"ID1 整个转动过程要连续") */
    Arm_Param_Set("R1T1", 230); Arm_Param_Set("R2T", -650); Arm_Param_Set("R1T2", 300); Arm_Param_Set("R2SPD", 500);
    Arm_Param_Set("R2STG", 1);
    run("SIDE RING");
    run("CLAW O");                                        /* (10-21)上一个 TAKE 夹着的放下(夹着物料时不许再 TAKE) */
    away(); clear(); run("TAKE 1"); clear();
    CHECK(run("APZ 200 -800 60") == 1, "分段：APZ");
    {
        int r1 = at("M1:300.0"), e1 = at("M2:-650.0@500"), r2 = at("M1:230.0"), e2 = at("M2:-800.0@500"), r3 = at("M1:200.0");
        CHECK(r1 >= 0 && r1 < e1 && e1 < r2 && r2 < e2 && e2 < r3,
              "离开车上转盘：只转到 R1T2 -> 只伸到 R2T(按 R2SPD) -> 只转到 R1T1 -> 只缩到要的长度 -> 再转过去");
    }
    clear();
    CHECK(run("TRAY") == 1, "TRAY：去车上转盘上方");
    {
        int r1 = at("M1:230.0"), e1 = at("M2:-650.0@500"), r2 = at("M1:300.0"), e2 = at("M2:-862.0@500"), r3 = at("M1:323.0");
        CHECK(r1 >= 0 && r1 < e1 && e1 < r2 && r2 < e2 && e2 < r3 && !has("L+"),
              "去车上转盘：只转到 R1T1 -> 只伸到 R2T -> 只转到 R1T2 -> 只缩回 A2R -> 只转到 A1D，不下降");
    }
    Arm_Param_Set("R2STG", 0);
    run("CLAW O");
    away(); clear(); run("TAKE 1"); clear(); run("APZ 200 -800 60");
    {
        int s1 = at("S1:200.0"), e1 = at("S2:-650.0@500"), e2 = at("S2:-800.0@500");
        CHECK(s1 >= 0 && s1 < e1 && e1 < e2 && !has("M1:300.0") && !has("M1:230.0"),
              "R2STG=0(默认)：ID1 一口气转过去不停，转过 R1T2 伸到 R2T、转过 R1T1 缩到要的长度");
    }
    clear(); run("TRAY");
    {
        int s1 = at("S1:323.0"), e1 = at("S2:-650.0@500"), e2 = at("S2:-862.0@500");
        CHECK(s1 >= 0 && s1 < e1 && e1 < e2 && !has("M1:230.0") && !has("M1:300.0"), "R2STG=0：去车上转盘也是 ID1 一直转");
    }
    Arm_Param_Set("R1T1", 0); Arm_Param_Set("R2T", 0); Arm_Param_Set("R1T2", 0); Arm_Param_Set("R2SPD", 0);
    run("SIDE RAW");
    /* (10-21 用户：arm_calib 里伸缩的速度要和 run 里统一)AF 大动作按 EXTSPD/ROTSPD(没设按 ASPD)，小调整照旧 ASPDF */
    Arm_Param_Set("EXTSPD", 1000);
    ang[2] = -862.0f; clear(); run("AF 2 -700");
    CHECK(has("M2:-700.0@1000/"), "AF 2 大动作按 EXTSPD");
    clear(); run("AF 2 -704");
    CHECK(has("M2:-704.0@40/"), "AF 小调整照旧按 ASPDF");
    Arm_Param_Set("EXTSPD", 0);
    Arm_Param_Set("A1P", 323);
    away(); clear(); run("OBS RING");
    CHECK(has("S1:323.0@90;") && has("S2:-862.0@90;"), "ROTSPD/EXTSPD = 0：和以前一样用 ASPD");

    /* ---- 急停：升降停下并要求重新回零 ---- */
    run("CLAW O");
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
    CHECK(has("L+9200;"), "LIFT 999 被限制在最高点 115mm(9200 脉冲)");
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
    clear(); Arm_Init();                            /* 重新开机：(10-23)只清屏 */
    CHECK(zone_is("ZONE 0 0"), "开机：ZONE 0 0");
    clear(); tap(100, 200); tap(240, 250);
    CHECK(zone_is("ZONE 0 0") && scr[0] == 0, "(10-23)开机只清屏、不收触摸：屏上按哪里都不算");
    run("ZONE ASK");                                /* 旧流程的选区页(ZONE ASK)还留着 */
    clear(); tap(100, 200);
    CHECK(zone_is("ZONE 1 0") && scrs("cls 0|") && scrs("xstr 0,128,480,192,0,0,2016,1,1,1,\"START\"")
          && !scrs("ZONE 1") && !scrs("BACK") && !scrs("xstr 0,84") && !scrs("sendxy"),
          "按左边按钮(按下、松开都在按钮 1 里)：选 1 区，画准备页：(10-23)只有大 START(不写 ZONE 1、没有状态行和 BACK)");
    clear(); tap(400, 30);
    CHECK(zone_is("ZONE 1 0") && scr[0] == 0, "(10-23)准备页右上角(以前的 BACK)按了不算");
    run("ZONE ASK");
    clear(); touch(100, 200, 1); touch(400, 200, 0); Arm_Poll();
    CHECK(zone_is("ZONE 0 0") && scr[0] == 0, "在按钮 1 按下、滑到按钮 2 松开：不算");
    touch(100, 200, 0); Arm_Poll();
    CHECK(zone_is("ZONE 0 0") && scr[0] == 0, "只有松开、没有按下：不算");
    tap(240, 200); tap(100, 20); tap(600, 200);
    CHECK(zone_is("ZONE 0 0") && scr[0] == 0, "按在两个按钮中间的缝上、标题行、屏外的坐标：都不算");
    clear(); tap(470, 315);
    CHECK(zone_is("ZONE 2 0") && scrs("\"START\"") && !scrs("\"ZONE 2\""), "按右边按钮：选 2 区(准备页不写 ZONE 2)");
    clear(); touch(240, 250, 1); Arm_Poll();
    CHECK(zone_is("ZONE 2 0") && scr[0] == 0, "START 只按下、还没松开：不算");
    touch(20, 310, 0); Arm_Poll();
    CHECK(zone_is("ZONE 2 1") && scrs("xstr 0,128,480,192,0,0,65504,1,1,1,\"GO\"") && !scrs("xstr 0,84") && !scrs("xstr 336")
          && !scrs("cls"),
          "在 START 里按下、松开：s=1，START 换成 GO(不整页重画；(10-23)没有状态行、没有 BACK)");
    clear(); tap(400, 30); tap(240, 250); tap(100, 200);
    CHECK(zone_is("ZONE 2 1") && scr[0] == 0, "按了 START 以后不再收触摸(START、别处都不算)");
    clear();
    CHECK(run("ZONE ASK") == 1 && zone_is("ZONE 0 0") && scrs("sendxy=1|cls 0|") && scrs("START ZONE 1 OR 2"),
          "ZONE ASK：发 sendxy=1、重画选区页，清掉选的区和 START");
    clear();
    CHECK(run("ZONE 1") == 1 && zone_is("ZONE 1 0") && scrs("sendxy=1|cls 0|") && !scrs("\"ZONE 1\"") && scrs("\"START\""),
          "ZONE 1(树莓派 race 1)：画准备页，(10-23)只有 START");
    tap(240, 250);
    CHECK(zone_is("ZONE 1 1"), "ZONE 1 以后屏上按 START：s=1");
    /* 换页以后 0.5 秒内按下的不算：选区时手指抖一下/连按两下，不会顺手按到 START(START 就在按钮 1、2 的下半部分) */
    run("ZONE ASK");
    touch_gap = 50; tap(100, 250);
    CHECK(zone_is("ZONE 0 0"), "ZONE ASK 刚画完 50ms 就按：不算");
    touch_gap = 600; tap(100, 250);
    CHECK(zone_is("ZONE 1 0"), "过了保护时间再按：选 1 区");
    touch_gap = 100; tap(100, 250); touch_gap = 300; tap(100, 250);
    CHECK(zone_is("ZONE 1 0"), "选好区以后 0.1 秒、0.3 秒又在同一个地方按了两下(手指抖)：都不算 START");
    touch_gap = 50; touch(100, 250, 1); touch_gap = 600; touch(100, 250, 0); Arm_Poll();
    CHECK(zone_is("ZONE 1 0"), "保护时间内按下、过了保护时间才松开：也不算");
    touch_gap = 600; tap(100, 250);
    CHECK(zone_is("ZONE 1 1"), "正常再按一次 START：s=1");
    CHECK(run("ZONE 2") == 1 && zone_is("ZONE 2 0"), "ZONE 2：START 清掉(s=0)");
    touch(240, 250, 1); run("ZONE 2"); touch(240, 250, 0); Arm_Poll();
    CHECK(zone_is("ZONE 2 0"), "按下 START 以后页面重画了(换页)，再松开不算(按下的那一下属于上一页)");
    clear();
    CHECK(run("ZONE MSG SCAN 1/2") == 1 && scr[0] == 0, "(10-23)准备页不显示状态行：ZONE MSG 只记下，不画");
    run("ZONE ASK");
    clear(); run("ZONE MSG  a\"bcdefghijklmnopqrstuvwxyz");
    CHECK(scrs(",\"abcdefghijklmnopqrst\""), "ZONE MSG：去掉双引号和前面多的空格，最多 20 个字");
    clear(); run("ZONE MSG");
    CHECK(scrs("xstr 240,0,240,48,1,65504,0,1,1,1,\"\""), "ZONE MSG 不带字：清空状态行(选区页右上角)");
    run("ZONE MSG PLAN OK"); clear(); run("ZONE ASK");
    CHECK(scrs("START ZONE 1 OR 2") && scrs("xstr 240,0,240,48,1,65504,0,1,1,1,\"\""), "重新 ZONE ASK：选区页的状态行清空(在右上角)");
    clear(); run("ZONE MSG SCAN");
    CHECK(scrs("xstr 240,0,240,48,1,65504,0,1,1,1,\"SCAN\""), "选区页的状态行在右上角");
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
        run("ZONE ASK"); now += 600;
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
        CHECK(scrs("sendxy=1|cls 0|") && !scrs("\"ZONE 1\"") && scrs("\"START\"") && zone_is("ZONE 1 0"), "屏重新上电：再发 sendxy=1，整页重画(选的区不变；(10-23)只有 START)");
        run("ZONE LOCK"); run("QR CLR"); clear();
        scr_in(boot, 4); Arm_Poll();
        CHECK(scr[0] == 0, "比赛布局时屏重新上电(还没读到码)：不画选区页，什么都不写");
        qr_feed("321+654+987+123\r\n"); Arm_Poll(); clear();
        scr_in(boot, 4); Arm_Poll();
        CHECK(scrs("\"321+654+\"") && scrs("\"987+123\"") && !scrs("START") && pi[0] == 0,
              "比赛布局时屏重新上电：把任务码重新写上(赛规要一直显示)，不再发给树莓派");
        run("QR CLR");
    }
    /* 旧的树莓派程序不发 ZONE LOCK：写 SCR 时自动回到比赛布局 */
    run("ZONE ASK"); clear();
    CHECK(run("SCR t2 GRAB 0") == 1 && scrs("cls 0|xstr 0,240,240,80,1,65535,0,1,1,1,\"\xD7\xA5\xC8\xA1" "0\""),
          "选区页还在显示时写 SCR：先清屏回到比赛布局，再写字");
    clear(); tap(100, 200);
    CHECK(zone_is("ZONE 0 0") && scr[0] == 0, "SCR 自动回到比赛布局以后，触摸也关了");
    clear(); run("SCR t2 GRAB 1");
    CHECK(!scrs("cls"), "已经是比赛布局：SCR 不清屏");
    /* 选区页还在显示时扫到码：告诉树莓派，不写屏(不盖掉按钮) */
    run("ZONE ASK"); clear();
    qr_feed("999+888+777+666\r\n"); Arm_Poll();
    CHECK(pis("QR 999+888+777+666") && !scrs("999+888"), "选区页还在显示时扫到码：发给树莓派，但不写屏(不盖掉按钮)");
    run("QR CLR"); run("ZONE LOCK");
    /* USART2 接收开不起来：开机照样只清屏，终端里照样能选区 */
    scr_rxfail = 1; clear(); Arm_Init(); scr_rxfail = 0;
    CHECK(scrs("cls 0|") && !scrs("READY") && !scrs("START ZONE") && !scrs("sendxy"), "收不到触摸(USART2 接收开不起来)：开机只清屏((10-23)不写 READY)");
    scr_rxfail = 1;
    CHECK(zone_is("ZONE 0 0") && run("ZONE 2") == 1 && zone_is("ZONE 2 0"), "收不到触摸时终端里 ZONE 2 照样能选");
    scr_rxfail = 0;
    CHECK(run("ZONE ASK") == 1 && scr_armed, "开机没开起来的接收：ZONE ASK 时再试一次");
    tap(100, 200);
    CHECK(zone_is("ZONE 1 0"), "再试开起来以后屏上能选");
    clear(); Arm_Init(); run("ZONE LOCK");          /* 恢复：接收开着，比赛布局 */

    /* ---- QR RAW 诊断：扫码器发来的内容不是任务码格式时也看得到 ---- */
    run("QR CLR"); clear();
    qr_feed("ABC 12\x01\r\n");
    CHECK(run("QR RAW") == 1 && pis("last=ABC 12<01>"), "QR RAW：显示最后一条原样内容(看不见的字符写成十六进制)");
    CHECK(pis("QRRAW n="), "QR RAW：带收到的字节数");
    run("QR?");
    CHECK(pis("QR NONE"), "不是任务码格式的内容不当成任务码");

    /* ---- 二维码在中断里锁存：长指令阻塞(主循环不跑 Arm_Poll)时扫到的码不丢 ---- */
    run("QR CLR"); clear();
    qr_feed("xx156+123+516+231\r\n");
    for (i = 0; i < 300; i++) qr_feed("x");          /* 后面一直来字节(缓冲满了好几次) */
    CHECK(run("QR?") == 1 && pis("QR 156+123+516+231"), "Arm_Poll 没跑时扫到的码在中断里已经锁存：后面的字节挤满缓冲也不丢，QR? 读得到");
    clear(); Arm_Poll();
    CHECK(!pis("QR ") && scrs("\"156+123+\"") && scrs("\"516+231\""), "主循环再写屏(t0 带 +)；QR? 已经回过这个码了，不再主动发一遍");
    run("QR CLR"); clear();
    qr_feed("156+123+516+231\r\n"); Arm_Poll();
    CHECK(pis("QR 156+123+516+231") && scrs("\"156+123+\""), "没被 QR? 取走的新码：主循环写屏、主动告诉树莓派");
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
    CHECK(strstr(scr, "xstr 0,0,480,120,0,65535,0,1,1,1,\"777+111+\"") && strstr(scr, "xstr 0,120,480,120,0,65535,0,1,1,1,\"222+333\""),
          "(10-23)画模式：任务码用大字(字库 0，字高 120)整个宽度居中画两行，组之间的 + 都在(777+111+ 共 8 个大字 480 像素)");
    clear(); run("SCR t2 GRAB 3");
    CHECK(strstr(scr, "xstr 0,240,240,80,1,65535,0,1,1,1,\"\xD7\xA5\xC8\xA1" "3\""), "(10-23)抓取数：中字(字库 1)画在下面一行左半边，前面加中文\"抓取\"(GB2312)");
    clear(); run("SCR t3 PLACE 12");
    CHECK(strstr(scr, "xstr 240,240,240,80,1,65535,0,1,1,1,\"\xB7\xC5\xD6\xC3" "12\""), "(10-23)放置数：右半边，\"放置\" + 两位数");
    clear(); run("SCR t3 5");
    CHECK(strstr(scr, "\"\xB7\xC5\xD6\xC3" "5\""), "(10-23)树莓派只发数字也行");
    clear(); run("SCR t1 RAW 1/3"); run("SCR t5 B1 R1 G2 B3");
    CHECK(scr[0] == 0, "(10-23)以前的阶段、批次小字不再画(屏上只有赛规要的字)");
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

    /* ---- 扫到码以后关掉扫码模块(GM65 串口指令)，QR CLR 时再打开 ---- */
    {
        static const uint8_t r_on[] = {0x02, 0x00, 0x00, 0x01, 0xD6, 0x12, 0x34};    /* 读 0x0000 的回复：D6 = 连续扫(最低两位 10) */
        static const uint8_t r_off[] = {0x02, 0x00, 0x00, 0x01, 0xD5, 0x56, 0x78};   /* D5 = 等指令才扫(01) */
        static const uint8_t ack[] = {0x02, 0x00, 0x00, 0x01, 0x00, 0x33, 0x31};     /* 写成功 */
#define QRB(a) do { size_t k_; for (k_ = 0; k_ < sizeof(a); k_++) { *rxp = (a)[k_]; Arm_QR_RxCplt(); now += 1; } } while (0)
        const char *code = "452+321+254+312\r\n", *p;
        run("QR CLR"); now += 1000; Arm_Poll();                               /* 前面的开/关都超时放弃，从干净的状态开始 */
        now += 1000; Arm_Poll();
        clear(); qrtx[0] = 0;
        for (p = code; *p; p++) { *rxp = (uint8_t)*p; Arm_QR_RxCplt(); now += 1; }
        Arm_Poll();
        CHECK(pis("QR 452+321+254+312"), "扫到码照样发给树莓派");
        CHECK(strstr(qrtx, "7E 00 07 01 00 00 01 AB CD|") != 0, "扫到码：先读扫码模块 0x0000 区(工作模式)");
        qrtx[0] = 0; QRB(r_on); Arm_Poll();
        CHECK(strcmp(qrtx, "7E 00 08 01 00 00 D5 AB CD|") == 0, "只改最低两位写回去：D6 -> D5(等指令才扫 = 关掉，不再一直响)");
        qrtx[0] = 0; QRB(ack); Arm_Poll(); run("QR SCAN?");
        CHECK(pis("QRSCAN OFF auto=1") && qrtx[0] == 0, "写成功：QR SCAN? 回 OFF");
        clear(); run("QR?");
        CHECK(pis("QR 452+321+254+312"), "关掉以后码还在，QR? 照样读得到");
        for (p = code; *p; p++) { *rxp = (uint8_t)*p; Arm_QR_RxCplt(); now += 1; }
        qrtx[0] = 0; Arm_Poll();
        CHECK(qrtx[0] == 0, "同一个码再来一次：不再发关闭指令");
        clear(); qrtx[0] = 0; run("QR CLR");
        CHECK(strstr(qrtx, "7E 00 07 01 00 00 01 AB CD|") != 0, "QR CLR(开跑前清码)：重新打开扫码模块，先读");
        qrtx[0] = 0; QRB(r_off); Arm_Poll();
        CHECK(strcmp(qrtx, "7E 00 08 01 00 00 D6 AB CD|") == 0, "D5 -> D6(连续扫)");
        QRB(ack); Arm_Poll(); clear(); run("QR SCAN?");
        CHECK(pis("QRSCAN ON"), "打开了：QR SCAN? 回 ON");
        /* 本来就是要的模式：只读不写 */
        qrtx[0] = 0; run("QR ON"); QRB(r_on); Arm_Poll();
        CHECK(strcmp(qrtx, "7E 00 07 01 00 00 01 AB CD|") == 0, "本来就开着：只读、不写");
        /* 模块不回话：每 0.3 秒重试，3 次都没回就放弃，记下 noreply，主循环不卡 */
        qrtx[0] = 0; run("QR OFF");
        for (i = 0; i < 6; i++) { now += 310; Arm_Poll(); }
        clear(); run("QR SCAN?");
        CHECK(pis("noreply") && !pis("busy"), "扫码模块不回话：重试后放弃，QR SCAN? 带 noreply");
        CHECK(strstr(qrtx, "7E 00 07 01 00 00 01 AB CD|7E 00 07 01 00 00 01 AB CD|7E 00 07 01 00 00 01 AB CD|") != 0
              && strstr(qrtx, "7E 00 08") == 0, "不回话：读了 3 次，没写");
        /* QR AUTO 0：扫到码不关 */
        run("QR AUTO 0"); run("QR CLR"); now += 2000; Arm_Poll();
        qrtx[0] = 0;
        for (p = code; *p; p++) { *rxp = (uint8_t)*p; Arm_QR_RxCplt(); now += 1; }
        Arm_Poll();
        CHECK(qrtx[0] == 0, "QR AUTO 0：扫到码不关扫码模块");
        run("QR AUTO 1"); run("QR CLR"); now += 2000; Arm_Poll();
#undef QRB
    }

    /* ---- 10-14 边升边转(OVLAP=1)：先直着升 OVCLR，再一边升到 ZHI 一边转，转过一半开始往目标上方降，最后直着降 ---- */
    {
        int c_, up1, sw, last_dn;
        const char *q;
        Arm_Param_Set("OVLAP", 1); Arm_Param_Set("OVCLR", 20);
        Arm_Param_Set("ZHI", 100); Arm_Param_Set("ZOBRAW", 100); Arm_Param_Set("ZGRAB", 20); Arm_Param_Set("ZDROP", 60);
        Arm_Param_Set("A1D", 200);                 /* 转盘姿态和原料盘姿态不一样，ID1 才会转 */
        run("OBS RAW O"); clear();
        CHECK(run("GRAB 2 H") == 1, "OVLAP：GRAB 2 H");
        c_ = at("C;"); up1 = at("L+6400;"); sw = at("S1:");
        q = strstr(ev + (up1 >= 0 ? up1 + 1 : 0), "L");
        CHECK(c_ >= 0 && c_ < up1 && up1 < sw && (!q || (int)(q - ev) > sw), "OVLAP：夹紧以后一条指令升到 ZHI(中途不停、不再发第二条)，升过 20mm 才开始转");
        CHECK(t_s1 >= t_up1 + 500 && t_s1 <= t_up1 + 650,
              "(10-24 用户\"上升的过程中突然停顿一下\")直着升的 20mm 不单独停下核对：按 LFRPM 60 算 0.5 秒升过 20mm(加 LFMRG 一半)就开始转");
        /* (10-24 用户"原料区抓住物料上升的过程中突然停顿一下，原料盘一转把物料甩飞")设了 A2GB(夹起以后往回缩)：同样一条指令升到 ZHI 不停 */
        Arm_Param_Set("A2GB", -1100);
        run("OBS RAW O"); clear();
        CHECK(run("GRAB 1 H") == 1, "OVLAP + A2GB：GRAB 1 H");
        {
            int cl = at("C;"), up = at("L+6400;"), bk = at("S2:-1100.0");
            const char *nx = strstr(ev + (up >= 0 ? up + 1 : 0), "L");
            CHECK(cl >= 0 && cl < up && up < bk && (!nx || (int)(nx - ev) > bk) && t_s2 >= t_up1 + 500 && t_s2 <= t_up1 + 650,
                  "OVLAP + A2GB：夹紧以后一条指令升到 ZHI(中途不停)，升过 OVCLR(0.5 秒)就开始缩回，还在接着升");
        }
        Arm_Param_Set("A2GB", 0);
        run("OBS RAW O"); clear();
        CHECK(run("GRAB 2 H") == 1, "OVLAP：再 GRAB 2 H");
        sw = at("S1:"); c_ = at("C;");
        q = strstr(ev + (sw > 0 ? sw : 0), "L-");
        last_dn = q ? (int)(q - ev) : -1;
        q = strstr(ev + (c_ > 0 ? c_ : 0), "O;");
        CHECK(last_dn > sw && q && (int)(q - ev) > last_dn, "OVLAP：转到转盘上方降下去以后才松爪");
        clear(); CHECK(run("TAKE 1") == 1, "OVLAP：TAKE 1");
        q = strstr(ev, "L-1600;");
        CHECK(q && strstr(q + 1, "L-1600;") && at("C;") > (int)(strstr(q + 1, "L-1600;") - ev), "OVLAP：TAKE 先降到槽口上方 20mm，转盘到位后再直着降 20mm 夹");
        clear(); run("OBS RING"); clear();
        CHECK(run("DROP") == 1, "OVLAP：DROP");
        q = strstr(ev, "O;");
        CHECK(q && strstr(q, "L+") && !strstr(strstr(q, "L+") + 1, "L+") && has("M2:") == 0, "OVLAP：放下松爪后一条指令升到 ZHI(中途不停)，边升边缩回");
        clear(); CHECK(run("APZ 340 -862 80") == 1 && has("S1:340.0") && at("L-") > at("S1:"), "APZ：转到 (340,-862) 并降到 80mm");
        CHECK(run("APZ 340 -862") != 1, "APZ 少参数：不执行");
        Arm_Param_Set("OVLAP", 0); Arm_Param_Set("A1D", 323);
        Arm_Param_Set("ZHI", 0); Arm_Param_Set("ZOBRAW", 0); Arm_Param_Set("ZGRAB", 80); Arm_Param_Set("ZDROP", 60);
    }

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
    /* 10-15 待机姿态：STBY SAVE 记下现在的高度/手臂/夹爪，存进 Flash；开机、PARK 都回到这里；STBY CLR 恢复原样 */
    clear(); run("STBY?");
    CHECK(pis("STBY NONE"), "没存过待机姿态：STBY? 回 STBY NONE");
    run("LIFT 80"); ang[1] = 250.0f; ang[2] = -600.0f; run("CLAW 1300");
    clear();
    CHECK(run("STBY SAVE") == 1 && pis("STBY Z=80.0") && pis("A1=250.0") && pis("A2=-600.0") && pis("CLAW=1300")
          && fake_flash[0] == 0x4C465432u && fake_flash[16] == 0x53544259u, "STBY SAVE：记下 80mm、ID1 250、ID2 -600、夹爪 1300，存进 Flash(升降标定还在)");
    away(); phys_mm = 83.0; clear();
    Arm_Init();
    CHECK(fabs(phys_mm - 80.0) < 0.05 && has("S1:250.0") && has("S2:-600.0") && has("claw1300;")
          && at("S1:250.0") < at("claw1300;"), "开机：编码器找准高度后自动回到待机姿态(手臂、夹爪、升降 80mm)，不再停 60");
    away(); run("LIFT 40"); clear();
    CHECK(run("PARK") == 1 && fabs(phys_mm - 80.0) < 0.05 && has("S1:250.0") && has("claw1300;"), "PARK：存了待机姿态就回到它");
    away(); run("CLAW 1500"); run("LIFT 40"); clear();
    CHECK(run("STOW") == 1 && fabs(phys_mm - 80.0) < 0.05 && has("S1:250.0") && has("S2:-600.0") && !has("claw1300;"),
          "STOW：存了待机姿态就收到它的手臂角度和高度(只有一个默认位置)，夹爪不动(可能夹着物料)");
    clear();
    CHECK(run("STBY CLR") == 1 && fake_flash[16] != 0x53544259u && fake_flash[0] == 0x4C465432u, "STBY CLR：删掉待机姿态，升降标定还在");
    clear(); phys_mm = 75.6;
    Arm_Init();
    CHECK(fabs(phys_mm - 60.0) < 0.05 && !has("claw1300;"), "删掉以后开机照旧停 60");
    /* 编码器一圈是 16384 的驱动器、方向也反过来 */
    enc_cpr = 16384.0; enc_cpm = -409.6; enc_off = 3000.0; phys_mm = 30.0;
    run("LIFT 30"); phys_mm = 30.0;
    clear();
    CHECK(run("LIFT CAL 30") == 1 && pis("CPR=16384") && !pis("LIFTSLIP"), "一圈 16384、方向相反的编码器也能标定(标定时上下走不按旧标定核对)");
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
    clear(); run("LIFT CAL 77"); run("LIFT?");
    CHECK(pis("ERR NOENC") && pis("LIFT 1 60.0"), "LIFT CAL 失败(读不到编码器)：程序记的高度不改(以前被改成填的数)");
    enc_ok = 1; clear(); phys_mm = 60.0; Arm_Init(); run("LIFT 85");    /* 停在 85 关电 */
    enc_ok = 0; clear(); phys_mm = 85.0;
    Arm_Init();
    CHECK(pis("LIFTBOOT NOENC (assume last 85.0") && !has("L+") && !has("L-"), "读不到编码器：当作在上次停下的 85(Flash 里记的)，不是一律 60");
    clear(); run("LIFT?");
    CHECK(pis("LIFT 1 85.0") && pis("BOOT=NOENC"), "读不到编码器：LIFT? 回上次的 85");
    /* (10-21)开机没读到编码器(高度是猜的)：之后一读到就认准(差 17mm 也改，以前超过 15mm 不改) */
    enc_ok = 1; phys_mm = 68.0; clear();                  /* 其实在 68(差 17mm) */
    CHECK(run("LIFT SYNC") == 1 && (pis("LIFTSYNC 68.0") || pis("LIFTSYNC 67.99")) && !has("L+") && !has("L-"),
          "LIFT SYNC：按编码器认出在 68(不动升降)");
    clear(); run("LIFT?");
    CHECK((pis("LIFT 1 68.0") || pis("LIFT 1 67.99")) && pis("BOOT=SYNC"), "认准以后 LIFT? 回 68、BOOT=SYNC");
    run("LIFT 85"); enc_ok = 0; phys_mm = 85.0; clear(); Arm_Init();      /* 又一次开机没读到 */
    enc_ok = 1; phys_mm = 102.0; clear();                /* 其实在 102 */
    run("LIFT 90");
    CHECK(pis("LIFTSYNC") && fabs(phys_mm - 107.0) < 0.05, "猜的高度：第一次走完读到编码器就认准(差 17mm)，报 LIFTSYNC");
    clear(); run("LIFT 90");
    CHECK(fabs(phys_mm - 90.0) < 0.05 && !pis("LIFTSLIP"), "认准以后走到 90 就是 90");
    enc_ok = 0; clear(); phys_mm = 90.0;
    CHECK(run("LIFT SYNC") == -1 && pis("ERR NOENC"), "读不到编码器：LIFT SYNC 回 ERR NOENC");
    enc_ok = 1;
    run("LIFT 60"); phys_mm = 60.0;
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
    run("LIFT 60"); abort_at = (int)now + 250; clear();
    run("LIFT 80"); abort_at = -1; clear(); run("LIFT?");
    CHECK(phys_mm > 62.0 && phys_mm < 78.0 && fabs(pinum("LIFT 1 ") - phys_mm) < 0.1, "急停打断：升降停在半路，用编码器找回真高度");
    run("LIFT 80");
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

    /* ---- (10-21)升降走完用编码器核对：要去的高度机构到不了(顶住、丢步)就按编码器改过来，报 LIFTSLIP ---- */
    enc_cpr = 65536.0; enc_cpm = 1638.4; enc_off = 12345.0; enc31_zero = 0; enc_frozen = 0;
    phys_mm = 60.0; flash_blank(); Arm_Init(); run("LIFT CAL 60"); phys_mm = 60.0;
    Arm_Param_Set("LFMAX", 125); phys_top = 115.0; phys_bot = 0.0;    /* 机构最高只能到 115、最低 0 */
    clear();
    CHECK(run("LIFT 80") == 1 && !pis("LIFTSLIP") && !has("stop5;") && fabs(phys_mm - 80.0) < 0.05, "正常走到 80：核对对得上，不报、不停");
    clear();
    CHECK(run("LIFT 120") == 1 && fabs(pinum("LIFTSLIP want=") - 120.0) < 0.01 && fabs(pinum(" got=") - 115.0) < 0.05 && !pis("too far")
          && has("stop5;") && fabs(phys_mm - 115.0) < 0.05, "LIFT 120，机构最高只到 115：走完核对发现差 5mm，报 LIFTSLIP，叫驱动器停住(不再顶着)");
    clear(); run("LIFT?");
    CHECK(fabs(pinum("LIFT 1 ") - 115.0) < 0.05, "高度按编码器改成 115(以前还当作 120，差的 5mm 下次往下全顶到最低点下面)");
    clear();
    CHECK(run("LIFT 2") == 1 && fabs(phys_mm - 2.0) < 0.05 && !pis("LIFTSLIP"), "再降到 2mm(像 ZPLC)：正好到 2，不往最低点下面顶");
    Arm_Param_Set("LFCHK", 0);                            /* 走完不核对(以前的样子) */
    clear();
    CHECK(run("LIFT 120") == 1 && fabs(pinum("LIFTSLIP want=") - 120.0) < 0.01 && fabs(pinum(" got=") - 115.0) < 0.05 && has("stop5;"),
          "(10-25)LFCHK 0 也一样：走的时候顶到最高点马上叫停、按编码器记成 115(以前 LFCHK 0 一直顶着，顶到驱动器堵转保护)");
    clear(); run("LIFT 2");
    CHECK(fabs(phys_mm - 2.0) < 0.05 && !pis("LIFTSLIP"), "LFCHK 0：之后下到 2mm 就是 2mm(以前顶住少走的 5mm 没人知道，下到 2mm 时往最低点下面顶)");
    Arm_Param_Set("LFCHK", 1.5f); clear(); run("LIFT 20");
    CHECK(fabs(phys_mm - 20.0) < 0.05 && !pis("LIFTSLIP"), "改过来以后再走就准了");
    run("LIFT 60"); phys_mm = 57.0; clear();              /* 被碰下去 3mm(程序还当在 60) */
    CHECK(run("LIFT 2") == 1 && fabs(pinum("LIFTSLIP want=") - 2.0) < 0.01 && fabs(pinum(" got=")) < 0.05 && has("stop5;") && fabs(phys_mm) < 0.05,
          "要去的比机构最低点还低(顶住)：报 LIFTSLIP，叫驱动器别再往下顶，高度改成 0");
    clear(); run("LIFT 2");
    CHECK(fabs(phys_mm - 2.0) < 0.05, "之后下到 2mm 就是 2mm");
    run("LIFT 60"); phys_mm = 78.0; clear();              /* 差了 18mm：编码器一圈 40mm，差太多可能认错圈 */
    CHECK(run("LIFT 70") == -1 && pis("LIFTSLIP") && pis("(too far, kept)") && pis("ERR LIFT") && has("stop5;"),
          "差 15mm 以上：报(too far, kept)，不改，回 ERR LIFT(GRAB/DROP 就停在这里，不在半空夹/松爪子)；"
          "(10-25)不动了也叫驱动器停住(以前不叫停，顶着的话一直顶到堵转保护)");
    clear(); run("LIFT?");
    CHECK(fabs(pinum("LIFT 1 ") - 70.0) < 0.05, "差太多时高度照旧(70)");
    phys_mm = 70.0;
    late_mm = 3.0; late_n = 3; clear();                   /* 升降比估的慢：等完时还差 3mm 在走 */
    CHECK(run("LIFT 90") == 1 && !pis("LIFTSLIP") && fabs(phys_mm - 90.0) < 0.05, "还在走：等它停下再核对，不当成丢步");
    clear(); run("LIFT?");
    CHECK(fabs(pinum("LIFT 1 ") - 90.0) < 0.05, "还在走的那次高度照样是 90");
    late_mm = 0.0;
    /* 边升边转(OVLAP=1，Lift_Start + Lift_Done_Now)也核对 */
    Arm_Param_Set("OVLAP", 1); Arm_Param_Set("OVCLR", 20); Arm_Param_Set("ZHI", 120);
    run("LIFT 60"); away(); clear();
    CHECK(run("APZ 340 -862 60") == 1 && fabs(pinum("LIFTSLIP want=") - 120.0) < 0.01 && fabs(pinum(" got=") - 115.0) < 0.05 && fabs(phys_mm - 60.0) < 0.05,
          "边升边转：ZHI 120 到不了，升完核对出只到 115，改过来，最后正好降到 60");
    Arm_Param_Set("OVLAP", 0); Arm_Param_Set("ZHI", 0);
    /* 开机认高度：停在 118 以上关电，开机时 LFMAX 还是程序里写的 115，以前会认错一整圈 */
    phys_top = 1e9; run("LIFT 120");
    Arm_Param_Set("LFMAX", 115); clear(); Arm_Init();
    CHECK(fabs(pinum("LIFTBOOT ") - 120.0) < 0.05 && fabs(phys_mm - 60.0) < 0.05, "停在 120 关电：开机认出 120 走回 60(以前认成 80，'往下 20' 其实停在 100)");
    /* LIFT ZERO：编码器的 0 也改到这里(存进 Flash)，核对和开机都按新的 0 算 */
    run("LIFT 0"); erases = 0; clear();
    CHECK(run("LIFT ZERO") == 1 && erases == 0 && !pis("LIFTZERO"), "就在编码器的 0 上 LIFT ZERO：本来就对得上，不擦 Flash");
    enc_off += 4.0 * 1638.4;                              /* 编码器的 0 和真的最低点差了 4mm(标定时量错了) */
    erases = 0; clear();
    CHECK(run("LIFT ZERO") == 1 && erases == 1 && fake_flash[0] == 0x4C465432u && fabs(pinum("LIFTZERO ENC ") - 4.0) < 0.05,
          "LIFT ZERO：编码器说在 4mm，把编码器的 0 改到这里，存进 Flash，回 LIFTZERO ENC 4.0 -> 0");
    clear(); run("LIFT ENC?");
    CHECK(fabs(pinum("Z=")) < 0.05, "LIFT ENC? 现在算出 0");
    clear(); run("LIFT 60");
    CHECK(!pis("LIFTSLIP") && fabs(phys_mm - 60.0) < 0.05, "之后走到 60 核对对得上(不会按旧的 0 又改回去)");
    clear(); Arm_Init();
    CHECK(fabs(pinum("LIFTBOOT ") - 60.0) < 0.05 && fabs(phys_mm - 60.0) < 0.05, "重新开机也按新的 0 认出 60");
    phys_top = 1e9; phys_bot = -1e9;
    /* (10-21 实车"下降的时候不夹取，抬起来了再夹取")差一点没到(爪子顶到下面了)：叫驱动器别再往下顶，报 LIFTPUSH */
    run("LIFT 60"); phys_bot = 21.0; clear();
    CHECK(run("LIFT 20") == 1 && pis("LIFTPUSH want=20") && !pis("LIFTSLIP") && has("stop5;"), "差 1mm 没到(顶住了)：LIFTPUSH，叫驱动器停住不再顶");
    clear(); run("LIFT?");
    CHECK(fabs(pinum("LIFT 1 ") - 21.0) < 0.05, "高度记成真的 21");
    phys_bot = -1e9;
    /* ---- (10-25 用户"降到最低设置 z=5 留出这么多冗余，还是会往下面压触发过载保护")走的时候一直盯着编码器 ---- */
    {
        uint32_t t0;
        run("LIFT 60"); t0 = now; clear();
        CHECK(run("LIFT 80") == 1 && fabs(phys_mm - 80.0) < 0.05 && !has("stop5;") && now - t0 < 620u && now - t0 > 480u,
              "正常走 20mm(0.5 秒)：一到就往下做，不再白等 LFMRG 200ms");
        run("LIFT 100"); phys_bot = 12.0; t0 = now; t_block = t_stop = 0; clear();   /* 物料比 ZPLC 早 7mm 碰到地面 */
        CHECK(run("LIFT 5") == 1 && fabs(pinum("LIFTSLIP want=") - 5.0) < 0.01 && fabs(pinum(" got=") - 12.0) < 0.05 && !pis("blocked")
              && has("stop5;") && t_block > 0 && t_stop >= t_block && t_stop - t_block <= 150u && fabs(phys_mm - 12.0) < 0.05,
              "放物料：降到 5，12mm 就碰到地面：碰到后 0.15 秒内叫驱动器停住(以前一直顶到算好的时间 + LFMRG + 核对完)，按编码器记成 12，照常往下做(松爪子)");
        CHECK(now - t0 < 2200u + 400u, "碰到地面以后不再白等");
        clear(); run("LIFT?");
        CHECK(fabs(pinum("LIFT 1 ") - 12.0) < 0.05, "高度记成真的 12");
        run("LIFT 100"); phys_bot = 30.0; t_block = t_stop = 0; clear();              /* 碰地的地方比要去的高 25mm(差太多) */
        CHECK(run("LIFT 5") == -1 && pis("(blocked)") && pis("ERR LIFT") && has("stop5;") && t_stop - t_block <= 150u && fabs(phys_mm - 30.0) < 0.05,
              "碰到底的地方比要去的高 15mm 以上：马上叫停(以前这种不叫停，一直顶到驱动器堵转保护)，回 ERR LIFT 不松爪子");
        clear(); run("LIFT?");
        CHECK(fabs(pinum("LIFT 1 ") - 30.0) < 0.05, "差 25mm 也记成真的 30(一路盯着数的，不会认错圈)");
        phys_bot = -1e9; run("LIFT 60");
        mv_slow = 0.6; t0 = now; clear();                                              /* 升降比算的慢(LFACC 加减速) */
        CHECK(run("LIFT 20") == 1 && fabs(phys_mm - 20.0) < 0.05 && !pis("LIFTSLIP") && !pis("LIFTPUSH") && !has("stop5;") && now - t0 > 1500u,
              "升降比算的慢：到点了还在走就接着等它走完，不在半路叫停、不当成顶住");
        mv_slow = 1.0; run("LIFT 60");
    }
    /* (10-21)升降根本没动(堵转保护/丢帧)：编码器一圈 40mm，以前会把 100 认成 76 当"丢了 7mm"，在半空夹下去 */
    Arm_Param_Set("ZGRAB", 69.0f);
    run("LIFT 100"); run("CLAW O"); away(); lift_dead = 1; clear();
    CHECK(run("GRAB 2 H") == -1 && pis("ERR LIFT") && pis("(blocked)") && has("stop5;") && !has("C;") && fabs(phys_mm - 100.0) < 0.05,
          "GRAB 时升降没下去：(10-25)一路盯着编码器，一直不动就叫停，报 (blocked)，回 ERR LIFT，不在半空夹");
    clear(); run("LIFT?");
    CHECK(fabs(pinum("LIFT 1 ") - 100.0) < 0.05, "没下去：高度记成真的 100(一路盯着数的，不会认错圈)");
    lift_dead = 0;
    run("LIFT 60");

    /* ---- (10-21 用户"放 1、2 号环的时候爪子莫名其妙松开""运输过程中一定要按第三步 CLWC 夹紧")
     *      夹爪每动一次都回 CLAWSET；夹着物料时只有放下(DROP、松进转盘、CLAW O/OR/OT)能张开 ---- */
    Arm_Param_Set("CLWO", 2090); Arm_Param_Set("CLWC", 2910); Arm_Param_Set("CLWOR", 0); Arm_Param_Set("CLWOT", 0);
    run("LIFT 60"); run("CLAW O"); clear();
    CHECK(run("CLAW C") == 1 && pis("CLAWSET 2910 C"), "CLAW C：回 CLAWSET 2910 C(夹爪每动一次都说是哪一步)");
    clear(); run("CLAW C");
    CHECK(!pis("CLAWSET"), "没动就不说");
    run("CLAW O"); away(); clear();
    CHECK(run("TAKE 2") == 1 && pis("CLAWSET 2910 C"), "TAKE：夹紧时回 CLAWSET 2910 C");
    clear();
    CHECK(run("TAKE 3") == -1 && pis("ERR HOLD") && !has("O;") && !has("L+") && !has("L-") && !has("S1:") && !has("S2:"),
          "夹着物料又来 TAKE：回 ERR HOLD，爪子不张开、手臂升降都不动(以前一开头就张开，物料掉在半路)");
    clear();
    CHECK(run("GRAB 1 H") == -1 && pis("ERR HOLD") && !has("O;"), "夹着物料 GRAB：同样 ERR HOLD");
    away(); clear();
    CHECK(run("OBS RING O") == 1 && pis("CLAWKEEP OBS") && !has("O;") && !pis("CLAWSET") && has("S1:"),
          "夹着物料 OBS RING O：手臂照样摆过去，爪子保持夹紧(回 CLAWKEEP OBS)");
    clear();
    CHECK(run("APZ 340 -700 100") == 1 && !has("O;") && !pis("CLAWSET"), "夹着物料转来转去(APZ)：爪子一直夹紧，一次都不动");
    clear();
    CHECK(run("DROP") == 1 && pis("CLAWSET 2090 O") && at("O;") > at("L-"), "DROP：降到放的高度才张开(CLAWSET 2090 O)");
    away(); clear();
    CHECK(run("TAKE 3") == 1 && has("O;") && has("C;"), "放下以后又能 TAKE");
    clear();
    CHECK(run("CLAW O") == 1 && pis("CLAWSET 2090 O"), "树莓派明说张开(放下 / 放回转盘)：CLAW O 照样张开");
    away(); clear();
    CHECK(run("TAKE 1") == 1, "CLAW O 以后不算夹着：能 TAKE");
    clear();
    CHECK(run("CLAW OT") == 1 && pis("CLAWSET 2090 OT"), "放回车上转盘：CLAW OT 张开");
    away(); clear();
    CHECK(run("GRAB 1 H") == 1 && at("C;") < last("O;"), "GRAB H：夹紧 -> 送进车上转盘松开");
    /* (10-21 实车"下降的时候不夹取，抬起来了再夹取")转盘换槽要 TTWAIT 才转完，下降只要半秒：等转盘停了才夹紧 */
    Arm_Param_Set("TTWAIT", 4000.0f);                        /* 比下降久得多：没等转盘的话夹紧一定早于它 */
    away(); clear();
    CHECK(run("GRAB 3 H") == 1 && t_close >= t_tt + 4000u, "GRAB 换槽：车上转盘转完(TTWAIT)才夹紧");
    away(); clear();
    run("CLAW O");
    CHECK(run("PICK 1 H") == 1 && t_close >= t_tt + 4000u, "PICK 换槽：车上转盘转完(TTWAIT)才夹紧");
    run("CLAW O");
    /* TT <微秒>(arm_calib 调转盘时发的)以后再 TT_Go 到原来那个号位：也要等转盘转到 */
    run("TT 1"); run("TT 1500"); away(); clear(); run("CLAW O");
    CHECK(run("GRAB 1 H") == 1 && t_close >= t_tt + 4000u, "TT 1500 以后 GRAB 1：车上转盘转回 1 号位(TTWAIT)才夹紧");
    run("CLAW O");
    Arm_Param_Set("TTWAIT", 800.0f);
    CHECK(Arm_Param_Set("LFMAX", NAN) == 2 && Arm_Param_Set("CLWC", NAN) == 2, "SET 参数给 nan：不认(以前范围检查挡不住)");
    clear();
    CHECK(run("AP nan 100") == -1, "AP nan：不认");
    away(); clear();
    CHECK(run("TAKE 2") == 1, "GRAB 松进车上转盘以后不算夹着：能 TAKE");
    /* 存了待机姿态：夹着物料时 PARK 回待机姿态，爪子不换成待机姿态的 */
    run("CLAW O"); run("LIFT 80"); ang[1] = 250.0f; ang[2] = -600.0f; run("CLAW 1300");
    CHECK(run("STBY SAVE") == 1, "存一个夹爪 1300 的待机姿态");
    away(); clear();
    CHECK(run("TAKE 2") == 1, "夹起物料");
    away(); clear();
    CHECK(run("PARK") == 1 && pis("CLAWKEEP STBY") && !has("claw1300;") && has("S1:250.0"), "夹着物料 PARK：手臂回待机姿态，爪子保持夹紧(CLAWKEEP STBY)");
    run("CLAW O"); away(); clear();
    CHECK(run("PARK") == 1 && has("claw1300;") && pis("CLAWSET 1300 STBY"), "空爪 PARK：夹爪照常到待机姿态的 1300");
    /* 开机：存了待机姿态就直接用它的夹爪脉宽，不先张开到 2090(STM32 万一中途重启，夹着的物料不会先掉) */
    clear(); Arm_Init();
    CHECK(has("pwm3=1300;") && !has("pwm3=2090;") && !has("O;"), "开机：夹爪直接是待机姿态的 1300，不先张开");
    run("STBY CLR"); clear(); Arm_Init();
    CHECK(has("pwm3=2090;"), "没存待机姿态：开机照旧 2090");
    clear();
    CHECK(run("TAKE 1") == 1, "开机以后不算夹着");
    run("CLAW O"); run("LIFT 60");

    /* ---- (10-23 用户"到原料区、粗加工区、暂存区之前机械臂提前转到需要的姿态，节省时间")PRE RAW / PRE RING：
     *      车开往工位的最后一段直线之前发；照 OBS 的路线转(绕电池盒)，要看着角度发的指令发完就回，不等转完；不降到观察高度 ---- */
    {
        uint32_t t0;

        Arm_Param_Set("OVLAP", 1); Arm_Param_Set("PARA", 1); Arm_Param_Set("ZHI", 100); Arm_Param_Set("ZOBRAW", 90); Arm_Param_Set("ZOBRNG", 90);
        Arm_Param_Set("ASPD", 90); Arm_Param_Set("ROTSPD", 0); Arm_Param_Set("EXTSPD", 0); Arm_Param_Set("A1D", 323); Arm_Param_Set("A2R", -862);
        Arm_Param_Set("A1G", 100); Arm_Param_Set("A2E", -950); Arm_Param_Set("A1P", 200); Arm_Param_Set("A2P", -800);
        Arm_Param_Set("A1T1", 150); Arm_Param_Set("A2T", -700); Arm_Param_Set("A1T2", 280); Arm_Param_Set("A2SPD", 400); Arm_Param_Set("A2STG", 0);
        Arm_Param_Set("R1T1", 230); Arm_Param_Set("R2T", -650); Arm_Param_Set("R1T2", 300); Arm_Param_Set("R2SPD", 500); Arm_Param_Set("R2STG", 0);
        run("CLAW O"); run("LIFT 100");
        slow = 1; ang[1] = sgoal[1] = 323.0f; ang[2] = sgoal[2] = -862.0f;
        clear(); t0 = now;
        CHECK(run("PRE RAW") == 1, "PRE RAW：回 DONE");
        CHECK(at("S1:100.0") >= 0 && at("S2:-950.0@400") > at("S1:100.0") && !has("S2:-700.0"),
              "PRE RAW 在车上转盘上方：和 OBS 一样 ID1 一直转，转过 A1T2 伸缩直接到 A2E");
        CHECK(ang[1] > 200.0f && now - t0 < 1500u && !has("H1:"), "PRE RAW：伸缩指令发了就回，不等 ID1 转完(还在半路，没停住)");
        CHECK(has("O;") && !has("L+") && !has("L-"), "PRE RAW：张开夹爪；本来在 ZHI，升降不动、不降到观察高度");
        HAL_Delay(3000);                                  /* 车开过去的这段时间舵机自己转完 */
        CHECK(ang[1] == 100.0f && ang[2] == -950.0f, "车开过去的路上手臂转到了观察姿态");
        clear();
        CHECK(run("OBS RAW O") == 1 && !has("S1:") && !has("S2:") && has("H1:") && has("H2:") && has("L-800;"),
              "到了再 OBS RAW：已经在姿态上，舵机只原地保持，只降到观察高度");
        /* 圆环那组：转过 R1T2 伸到 R2T、转过 R1T1 缩到 A2P，这一条发了就回 */
        run("LIFT 100"); ang[1] = sgoal[1] = 323.0f; ang[2] = sgoal[2] = -862.0f;
        clear(); t0 = now;
        CHECK(run("PRE RING") == 1 && at("S1:200.0") >= 0 && at("S2:-650.0@500") > at("S1:200.0") && at("S2:-800.0@500") > at("S2:-650.0@500"),
              "PRE RING 在车上转盘上方：ID1 一直转，转过 R1T2 伸到 R2T、转过 R1T1 缩到 A2P");
        CHECK(ang[1] > 205.0f && ang[1] <= 231.0f && !has("H1:"), "PRE RING：转过 R1T1 发完最后一条伸缩就回(ID1 还没到 A1P)");
        /* 不在车上转盘上方：两个一起发，马上回 */
        HAL_Delay(3000); ang[1] = sgoal[1] = 120.0f; ang[2] = sgoal[2] = -862.0f;
        clear(); t0 = now;
        CHECK(run("PRE RING") == 1 && has("S1:200.0@90") && has("S2:-800.0@90") && now - t0 < 100u && ang[1] < 125.0f,
              "PRE RING 不在车上转盘上方：ID1、ID2 一起发，不等就回");
        /* 已经在姿态上的那个只原地保持 */
        HAL_Delay(3000); clear();
        CHECK(run("PRE RING") == 1 && !has("S1:") && !has("S2:") && has("H1:200.0") && has("H2:-800.0"), "PRE：已经在姿态上就只原地保持");
        /* 升降不在最高处：先升到 ZHI 再转 */
        run("LIFT 60"); ang[1] = sgoal[1] = 120.0f; clear();
        CHECK(run("PRE RAW") == 1 && at("L+3200;") >= 0 && at("S1:100.0") > at("L+3200;"), "PRE：升降不在最高处，先升到 ZHI 再转");
        /* 分段走(A2STG=1)：前面几段照样一段一段等，最后一转不等 */
        HAL_Delay(3000); Arm_Param_Set("A2STG", 1); ang[1] = sgoal[1] = 323.0f; ang[2] = sgoal[2] = -862.0f; clear();
        CHECK(run("PRE RAW") == 1 && at("M1:280.0") >= 0 && at("M2:-950.0@400") > at("M1:280.0") && at("S1:100.0") > at("M2:-950.0@400")
              && !has("M1:100.0") && ang[1] > 250.0f, "PRE 分段走：只转到 A1T2 -> 伸缩 -> 最后转到 A1G 这一下不等");
        Arm_Param_Set("A2STG", 0);
        /* PARA=0：先转好再伸缩(伸缩不等) */
        HAL_Delay(3000); Arm_Param_Set("PARA", 0); ang[1] = sgoal[1] = 120.0f; ang[2] = sgoal[2] = -862.0f; clear();
        CHECK(run("PRE RING") == 1 && at("M1:200.0") >= 0 && at("S2:-800.0") > at("M1:200.0") && !has("M2:"), "PRE PARA=0：先转 ID1(等)，再发伸缩(不等)");
        Arm_Param_Set("PARA", 1);
        HAL_Delay(3000); slow = 0;
        /* 不做的情况 */
        clear();
        CHECK(run("PRE FOO") == -1 && pis("ERR ARG") && run("PRE") == 0, "PRE 参数不对：ERR ARG；PRE 不带参数不是机械臂指令");
        Arm_Param_Set("ARMOK", 0); clear();
        CHECK(run("PRE RAW") == -1 && pis("ERR NOCAL") && !has("S1:"), "ARMOK=0：PRE 拒绝");
        Arm_Param_Set("ARMOK", 1);
        away(); run("TAKE 1"); clear();
        CHECK(run("PRE RAW") == -1 && pis("ERR HOLD") && !has("S1:") && !has("O;"), "爪子里夹着物料：PRE 不做(不张开、不转)");
        run("CLAW O");
        Arm_Param_Set("OVLAP", 0); Arm_Param_Set("ZHI", 0); Arm_Param_Set("ZOBRAW", 0); Arm_Param_Set("ZOBRNG", 0);
        Arm_Param_Set("A1G", 323); Arm_Param_Set("A2E", -862); Arm_Param_Set("A1P", 323); Arm_Param_Set("A2P", -862);
        Arm_Param_Set("A1T1", 0); Arm_Param_Set("A2T", 0); Arm_Param_Set("A1T2", 0); Arm_Param_Set("A2SPD", 0);
        Arm_Param_Set("R1T1", 0); Arm_Param_Set("R2T", 0); Arm_Param_Set("R1T2", 0); Arm_Param_Set("R2SPD", 0);
        run("LIFT 60");
    }

    /* ---- (10-24 用户"把这个上下动的这个程序删掉")上电以后升降不再隔几秒上下动(以前的上电保活删了) ---- */
    {
        int k;

        run("LIFT 60"); clear(); Arm_Init(); clear();
        for (k = 0; k < 100; k++) { now += 1000; Arm_Poll(); }
        CHECK(!has("L+") && !has("L-"), "上电以后没有树莓派指令：升降一直不动(上电保活删了)");
    }

    /* ---- GET/SET 用的接口 ---- */
    clear();
    Arm_Param_Dump();
    CHECK(pis("P ZPLC=80.0000") && pis("P LFMAX=115.0000") && pis("P ARMOK=1.0000") && pis("P ASPD=90.0000"), "参数全部打印");

    (void)i;
    printf("%s: %d 项检查，%d 项失败\n", fails ? "失败" : "通过", checks, fails);
    return fails ? 1 : 0;
}
