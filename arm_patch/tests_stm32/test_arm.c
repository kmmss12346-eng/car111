/* arm.c 的主机回归测试：用假的 HAL / 舵机 / 升降，检查指令解析、动作顺序、保护、急停、扫码。
 * 运行：bash run.sh      (有断言失败会返回非 0) */
#include "arm.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

UART_HandleTypeDef huart2, huart3, huart5;
TIM_HandleTypeDef htim2;
volatile uint8_t car_abort = 0;

static uint32_t now = 0;
static int abort_at = -1;
static float ang[3] = {0, 495.0f, -862.0f};
static char ev[8192];                 /* 事件记录：每个动作追加一小段文字，测试里用 strstr / 下标比较顺序 */
static char pi[4096];                 /* STM32 发给树莓派的文字 */
static char scr[1024];                /* 发给串口屏的文字 */
static uint8_t *rxp = 0;
static int fails = 0, checks = 0;

#define EV(...) do { char t_[96]; snprintf(t_, sizeof t_, __VA_ARGS__); strncat(ev, t_, sizeof ev - strlen(ev) - 1); } while (0)
#define CHECK(c, msg) do { checks++; if (!(c)) { fails++; printf("  FAIL: %s  (line %d)\n", msg, __LINE__); } } while (0)

uint32_t HAL_GetTick(void) { return now; }
void HAL_Delay(uint32_t ms) { now += ms; if (abort_at >= 0 && (int)now >= abort_at) car_abort = 1; }
void Delay_Report(uint32_t ms) { HAL_Delay(ms); }
int HAL_UART_Transmit(UART_HandleTypeDef *h, uint8_t *b, uint16_t n, uint32_t t) {
    (void)t;
    if (h == &huart3 && strncmp((char *)b, "YAW100", 6) != 0) strncat(pi, (char *)b, (size_t)n < sizeof pi - strlen(pi) - 1 ? (size_t)n : sizeof pi - strlen(pi) - 1);
    if (h == &huart2 && b[0] != 0xFF) { strncat(scr, (char *)b, (size_t)n); strcat(scr, "|"); }
    return 0;
}
int HAL_UART_Receive_IT(UART_HandleTypeDef *h, uint8_t *b, uint16_t n) { (void)n; if (h == &huart5) rxp = b; return 0; }
int HAL_UART_Init(UART_HandleTypeDef *h) { (void)h; return 0; }
int HAL_TIM_PWM_Start(TIM_HandleTypeDef *h, uint32_t c) { (void)h; EV("pwm%u;", c); return 0; }
void tim_set(uint32_t ch, uint32_t v) { (void)ch; (void)v; }
void Claw_Set(uint32_t p) { EV("claw%u;", p); }
void Claw_Open(void) { EV("O;"); }
void Claw_Close(void) { EV("C;"); }
void Turntable_Set(uint32_t p) { EV("ttus%u;", p); }
void Turntable_GoTo(uint8_t s) { EV("tt%u;", s); }
int Servo_ReadAngle(uint8_t id, float *a) { *a = ang[id]; return 1; }
static float clampa(uint8_t id, float a) { float lo = id == 1 ? 390.0f : -1220.0f, hi = id == 1 ? 600.0f : -503.5f; return a < lo ? lo : (a > hi ? hi : a); }
uint32_t Servo_Start(uint8_t id, float *a, float spd) {
    float d; *a = clampa(id, *a); d = *a - ang[id]; if (d < 0) d = -d; ang[id] = *a;
    EV("S%u:%.1f@%.0f;", id, *a, spd); return (uint32_t)(d / spd * 1000) + 200;
}
void Servo_Move(uint8_t id, float a, float spd, float tol, uint32_t extra) {
    (void)extra; a = clampa(id, a); EV("M%u:%.1f@%.0f/%.2f;", id, a, spd, tol); ang[id] = a; now += 300;
}
void Emm_V5_En_Control(uint8_t a, bool s, bool f) { (void)a; (void)s; (void)f; }
void Emm_V5_Reset_CurPos_To_Zero(uint8_t a) { EV("zero%u;", a); }
void Emm_V5_Pos_Control(uint8_t a, uint8_t d, uint16_t v, uint8_t acc, uint32_t clk, bool r, bool f) { (void)a; (void)v; (void)acc; (void)r; (void)f; EV("L%s%u;", d ? "-" : "+", clk); }
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

int main(void) {
    int i;
    Arm_Init();
    CHECK(has("pwm3;") && has("pwm2;"), "Arm_Init 启动了夹爪/转盘 PWM");
    CHECK(strstr(scr, "cls 0") && strstr(scr, "\"READY\""), "画模式开机清屏并显示 READY");

    /* ---- 保护 ---- */
    clear();
    CHECK(run("GRAB 1") == -1 && pis("ERR NOCAL"), "ARMOK=0 时拒绝");
    CHECK(run("OBS RAW O") == -1, "ARMOK=0 时 OBS 拒绝");
    Arm_Param_Set("ARMOK", 1);
    clear();
    CHECK(run("LIFT?") == 1 && pis("LIFT 1 60"), "开机位置 = 离最低点 60mm");
    clear();
    CHECK(run("LIFT 100") == 1 && has("L+3200;"), "开机不用 LIFT ZERO：从 60 走到 100 = 往上 40mm(3200 脉冲)");
    clear();
    CHECK(run("LIFT 0") == 1 && has("L-8000;"), "LIFT 0 = 回到最低点(往下 100mm)");
    CHECK(run("LIFT ZERO") == 1 && has("zero5;"), "LIFT ZERO");
    /* 下面的顺序测试用固定的一组高度(和默认值无关)，都在最高点 100mm 以内 */
    Arm_Param_Set("ZHI", 0); Arm_Param_Set("ZGRAB", 80); Arm_Param_Set("ZDROP", 60); Arm_Param_Set("ZPLC", 80);
    Arm_Param_Set("ZSTK", 40); Arm_Param_Set("ZOBRAW", 0); Arm_Param_Set("ZOBRNG", 0);
    CHECK(Arm_Param_Set("LFMAX", 150) == 2, "最高点写死 100mm：SET LFMAX 150 被拒绝");
    CHECK(Arm_Param_Set("ZPLC", 1000) == 2 && Arm_Param_Set("NOPE", 1) == 0 && Arm_Param_Set("ZPLC", 90) == 1, "参数范围/名字检查");
    Arm_Param_Set("ZPLC", 80);

    /* ---- 参数错误 ---- */
    CHECK(run("GRAB 4 H") == -1 && run("GRAB 1 Z") == -1 && run("TAKE 0") == -1 && run("DROP X") == -1 && run("OBS FOO") == -1
          && run("PLACE 2 Q") == -1 && run("CLAW 100") == -1 && run("TT 9") == -1 && run("AD 3 1") == -1
          && run("AD 1 x") == -1 && run("AD 1 100") == -1 && run("LIFT x") == -1, "各种错误参数都被拒绝");
    CHECK(run("FOO") == 0 && run("") == 0 && run("CLAW") == 0, "不认识的指令返回 0(交给后面)");

    /* ---- OBS ---- */
    clear();
    CHECK(run("OBS RAW O") == 1, "OBS RAW O");
    CHECK(at("O;") >= 0 && at("O;") < at("S1:") && at("S1:") >= 0 && at("S2:") >= 0, "OBS：先张爪，再 ID1/ID2 一起转");
    CHECK(has("S1:495.0@90;") && has("S2:-862.0@90;"), "OBS RAW 的姿态和大动作速度");

    /* ---- GRAB n H 的完整顺序 ---- */
    clear();
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
    clear();
    CHECK(run("PLACE 1") == 1 && at("C;") >= 0 && at("C;") < last("S1:") && last("S1:") < last("O;"), "PLACE：先取(夹紧)，再转到圆环上方，再放(松开)");

    /* ---- STOW ---- */
    clear();
    CHECK(run("STOW") == 1 && has("S1:") && has("S2:"), "STOW");

    /* ---- 精确控制 ---- */
    clear();
    CHECK(run("A? 1") == 1 && pis("ANG 1 "), "A? 回角度");
    clear();
    ang[1] = 495.0f;
    CHECK(run("AD 1 0.5") == 1 && has("M1:495.5@40/0.30;") && pis("ANG 1 495.5000"), "AD 小增量用微调速度/精度");
    clear();
    CHECK(run("AD 2 -20") == 1 && has("@90/0.30;"), "AD 大增量(>8°)用大动作速度");
    clear();
    CHECK(run("AF 2 -860.5") == 1 && has("M2:-860.5"), "AF 绝对角度");
    clear();
    ang[1] = 495.0f; ang[2] = -862.0f;
    CHECK(run("AP 497 -863") == 1 && has("S1:497.0@80;") && has("S2:-863.0@80;"), "AP 小调整：微调速度的 2 倍(80)");
    clear();
    CHECK(run("AP 500 -900") == 1 && has("@90;"), "AP 大角度变化：改用大动作速度(90)");
    CHECK(pis("ANG 1") && pis("ANG 2"), "AP 回读两个角度");

    /* ---- 非并行模式的先后顺序 ---- */
    Arm_Param_Set("PARA", 0);
    clear(); run("OBS RING");
    CHECK(at("M1:") >= 0 && at("M1:") < at("M2:"), "PARA=0：伸出时先转 ID1 再伸 ID2");
    clear(); run("STOW");
    CHECK(at("M2:") >= 0 && at("M2:") < at("M1:"), "PARA=0：缩回时先缩 ID2 再转 ID1");
    Arm_Param_Set("PARA", 1);

    /* ---- 急停：升降停下并要求重新回零 ---- */
    clear();
    abort_at = (int)now + 800;
    run("GRAB 1 H");
    abort_at = -1;
    CHECK(has("stop5;"), "急停时停升降");
    clear();
    CHECK(run("OBS RAW") == -1 && pis("ERR NOZERO"), "急停打断升降后要求重新回零");
    run("LIFT ZERO");

    /* ---- 升降限幅 ---- */
    clear();
    run("LIFT 0"); clear();
    run("LIFT 999");
    CHECK(has("L+8000;"), "LIFT 999 被限制在最高点 100mm(8000 脉冲)");
    run("LIFT 0");

    /* ---- 二维码(画模式：屏工程里不用放控件) ---- */
    clear();
    {
        const char *code = "777+111+222+333\r\n";
        extern void Arm_Poll(void);
        const char *p;
        for (p = code; *p; p++) { *rxp = (uint8_t)*p; Arm_QR_RxCplt(); now += 1; }
        Arm_Poll();
    }
    CHECK(strstr(scr, "xstr 0,0,288,80,1,65535,0,1,1,1,\"777+111\"") && strstr(scr, "xstr 0,80,288,80,1,65535,0,1,1,1,\"222+333\""),
          "画模式：任务码用大字(字库1)画在左上两行");
    clear(); run("SCR t1 RAW 1/3");
    CHECK(strstr(scr, "xstr 292,2,188,38,0,65535,0,0,1,1,\"RAW 1/3\""), "画模式：t1 小字画在右上");
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
    CHECK(pis("QR 452+321+254+312") && strstr(scr, "t0.txt=\"452+321\"") && strstr(scr, "t7.txt=\"254+312\""), "扫到任务码：发给树莓派并分两行显示到串口屏 t0、t7");
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
    CHECK(pis("QR 111+222+333+123") && strstr(scr, "t0.txt=\"111+222\""), "换了一个码：重新发、重新显示");
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

    /* ---- GET/SET 用的接口 ---- */
    clear();
    Arm_Param_Dump();
    CHECK(pis("P ZPLC=80.0000") && pis("P LFMAX=100.0000") && pis("P ARMOK=1.0000") && pis("P ASPD=90.0000"), "参数全部打印");

    (void)i;
    printf("%s: %d 项检查，%d 项失败\n", fails ? "失败" : "通过", checks, fails);
    return fails ? 1 : 0;
}
