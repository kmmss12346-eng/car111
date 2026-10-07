/* 5 号(升降)电机单独测试程序 —— 只用来测试，测完要烧回正式程序！
 *
 * 不用 HAL、不用机械臂代码，只给地址 5 发最基本的张大头指令，每两条之间都留 20ms：
 *   1. 解除堵转保护、使能(这时用手拧电机应该拧不动)
 *   2. 速度模式：30 转/分 正转 0.5 秒 → 停 → 反转 0.5 秒 → 停   (各约转 1/4 圈)
 *   3. 位置模式：正转 800 脉冲(1/4 圈) → 反转 800 脉冲
 *   之后每 4 秒重复一遍。升降先放在中间位置，免得往一头顶。
 * 过程从 USART3 打印，树莓派上看：stty -F /dev/serial0 115200 raw -echo; cat /dev/serial0
 */
#include <stdint.h>

#define REG(a)        (*(volatile uint32_t *)(a))
#define RCC_AHB1ENR   REG(0x40023830)
#define RCC_APB1ENR   REG(0x40023840)
#define GPIOB         0x40020400u
#define GPIOC         0x40020800u
#define MODER(g)      REG((g) + 0x00)
#define PUPDR(g)      REG((g) + 0x0C)
#define IDR(g)        REG((g) + 0x10)
#define BSRR(g)       REG((g) + 0x18)
#define AFRH(g)       REG((g) + 0x24)
#define USART3        0x40004800u
#define UART4         0x40004C00u
#define SR(u)         REG((u) + 0x00)
#define DR(u)         REG((u) + 0x04)
#define BRR(u)        REG((u) + 0x08)
#define CR1(u)        REG((u) + 0x0C)
#define SYST_CSR      REG(0xE000E010)
#define SYST_RVR      REG(0xE000E014)
#define SYST_CVR      REG(0xE000E018)

static volatile uint32_t ms = 0;
void SysTick_Handler(void) { ms++; }
static void delay(uint32_t t) { uint32_t s = ms; while (ms - s < t) { } }

/* ---------- 打印(USART3) ---------- */
static void putc3(char c) { while (!(SR(USART3) & (1u << 7))) { } DR(USART3) = (uint8_t)c; }
static void puts3(const char *s) { while (*s) putc3(*s++); }
static void hex2(uint8_t v) { const char *h = "0123456789ABCDEF"; putc3(h[v >> 4]); putc3(h[v & 15]); }
static void dec(uint32_t v) { char b[11]; int i = 0; do { b[i++] = (char)('0' + v % 10); v /= 10; } while (v); while (i) putc3(b[--i]); }

/* ---------- 电机串口(UART4) ---------- */
static void tx4(const uint8_t *p, int n)
{
    int i;
    for (i = 0; i < n; i++) { while (!(SR(UART4) & (1u << 7))) { } DR(UART4) = p[i]; }
    while (!(SR(UART4) & (1u << 6))) { }                 /* 等最后一个字节发完 */
}
static void flush4(void) { int n = 0; while ((SR(UART4) & (1u << 5)) && n++ < 64) (void)DR(UART4); (void)SR(UART4); (void)DR(UART4); }
static int rx4(uint8_t *buf, int max, uint32_t wait_ms)  /* 收 wait_ms 毫秒内到的所有字节 */
{
    int n = 0;
    uint32_t s = ms;
    while (ms - s < wait_ms)
    {
        if (SR(UART4) & (1u << 5)) { uint8_t b = (uint8_t)DR(UART4); if (n < max) buf[n++] = b; }
    }
    return n;
}

static void uart_init(uint32_t u)
{
    CR1(u) = 0;
    BRR(u) = 139;                                         /* 16MHz / 115200 ≈ 138.9 */
    CR1(u) = (1u << 13) | (1u << 3) | (1u << 2);          /* UE TE RE */
}

/* 引脚 pin(10/11) 设成：0 输入，1 输出，2 复用(af) */
static void pin_mode(uint32_t g, int pin, int mode, int af)
{
    MODER(g) = (MODER(g) & ~(3u << (pin * 2))) | ((uint32_t)mode << (pin * 2));
    if (mode == 2) AFRH(g) = (AFRH(g) & ~(15u << ((pin - 8) * 4))) | ((uint32_t)af << ((pin - 8) * 4));
}
static void pin_pull(uint32_t g, int pin, int pull)       /* 0 无，1 上拉，2 下拉 */
{
    PUPDR(g) = (PUPDR(g) & ~(3u << (pin * 2))) | ((uint32_t)pull << (pin * 2));
}
static int pin_read(uint32_t g, int pin) { return (IDR(g) >> pin) & 1u; }


static void emm(const uint8_t *c, int n, const char *what)
{
    flush4();
    tx4(c, n);
    puts3("  发: "); puts3(what); puts3("  [");
    { int i; for (i = 0; i < n; i++) { hex2(c[i]); if (i + 1 < n) putc3(' '); } }
    puts3("]");
    {
        uint8_t r[16];
        int k, m = rx4(r, sizeof(r), 20);            /* 顺便看驱动器回没回(PC11 没接就收不到，不影响) */
        if (m) { puts3("  回: "); for (k = 0; k < m; k++) { hex2(r[k]); putc3(' '); } }
    }
    puts3("\r\n");
}

static void vel(uint8_t dir, uint16_t rpm)
{
    uint8_t c[8];
    c[0] = 5; c[1] = 0xF6; c[2] = dir; c[3] = (uint8_t)(rpm >> 8); c[4] = (uint8_t)rpm; c[5] = 0; c[6] = 0; c[7] = 0x6B;
    emm(c, 8, dir ? "速度 反转 30转/分" : "速度 正转 30转/分");
}

static void stop5(void)
{
    uint8_t c[5] = { 5, 0xFE, 0x98, 0, 0x6B };
    emm(c, 5, "停");
}

static void pos(uint8_t dir, uint32_t clk)
{
    uint8_t c[13];
    c[0] = 5; c[1] = 0xFD; c[2] = dir; c[3] = 0; c[4] = 60; c[5] = 0;         /* 60 转/分，加速度 0(直接启动) */
    c[6] = (uint8_t)(clk >> 24); c[7] = (uint8_t)(clk >> 16); c[8] = (uint8_t)(clk >> 8); c[9] = (uint8_t)clk;
    c[10] = 0; c[11] = 0; c[12] = 0x6B;                                       /* 相对运动，不等同步 */
    emm(c, 13, dir ? "位置 反转 800 脉冲" : "位置 正转 800 脉冲");
}

int main(void)
{
    uint32_t round = 0;

    SYST_RVR = 16000 - 1; SYST_CVR = 0; SYST_CSR = 7;
    RCC_AHB1ENR |= (1u << 1) | (1u << 2);
    RCC_APB1ENR |= (1u << 18) | (1u << 19);
    (void)RCC_APB1ENR;
    pin_mode(GPIOB, 10, 2, 7); pin_mode(GPIOB, 11, 2, 7);
    pin_mode(GPIOC, 10, 2, 8); pin_mode(GPIOC, 11, 2, 8);
    pin_pull(GPIOC, 11, 1);
    uart_init(USART3);
    uart_init(UART4);
    delay(1500);
    puts3("\r\n\r\n===== 5 号(升降)电机测试 =====  (测完记得烧回正式程序)\r\n");

    for (;;)
    {
        uint8_t c[6];
        round++;
        puts3("\r\n第 "); dec(round); puts3(" 轮\r\n");
        c[0] = 5; c[1] = 0x0E; c[2] = 0x52; c[3] = 0x6B; emm(c, 4, "解除堵转保护"); delay(20);
        c[0] = 5; c[1] = 0xF3; c[2] = 0xAB; c[3] = 1; c[4] = 0; c[5] = 0x6B; emm(c, 6, "使能(现在拧电机应该拧不动)"); delay(500);
        vel(0, 30); delay(500); stop5(); delay(600);
        vel(1, 30); delay(500); stop5(); delay(600);
        pos(0, 800); delay(800);
        pos(1, 800); delay(800);
        puts3("  这一轮做完。电机动了吗？\r\n");
        delay(4000);
    }
}

/* ---------- 启动 ---------- */
extern uint32_t _estack, _etext, _sdata, _edata, _sbss, _ebss;
void Reset_Handler(void)
{
    uint32_t *s = &_etext, *d = &_sdata;
    while (d < &_edata) *d++ = *s++;
    for (d = &_sbss; d < &_ebss; ) *d++ = 0;
    main();
    for (;;) { }
}
void Default_Handler(void) { for (;;) { } }

__attribute__((section(".isr_vector"), used))
void (*const vectors[16])(void) = {
    (void (*)(void))&_estack, Reset_Handler,
    Default_Handler, Default_Handler, Default_Handler, Default_Handler, Default_Handler,
    0, 0, 0, 0,
    Default_Handler, Default_Handler, 0, Default_Handler,
    SysTick_Handler,
};
