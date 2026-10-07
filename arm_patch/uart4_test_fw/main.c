/* UART4(PC10/PC11) 和电机驱动器通讯测试程序 —— 只用来测试，测完要烧回正式程序！
 *
 * 不用 HAL，直接操作寄存器，内部 16MHz 时钟。上电后：
 *   1. 测引脚电平：PC10 能不能拉高拉低(被别的设备顶住?)，PC11 上有没有驱动器的 TX 接着
 *   2. 每个驱动器(地址 1~5)读电压、状态，打印收到的原始字节
 *   3. 4 个轮子依次慢转 1 秒(1 号→2 号→3 号→4 号)，看哪个转了 = PC10 发的指令哪个驱动器收到了
 *   之后每 3 秒重复第 2 步
 * 结果从 USART3(PB10/PB11，和树莓派通信的那路，115200)打印出来，树莓派上看：
 *   stty -F /dev/serial0 115200 raw -echo; cat /dev/serial0
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

static void pin_test(void)
{
    int pd, pu, r0, r1;

    puts3("\r\n[1] 引脚电平测试\r\n");
    /* PC11(RX)：驱动器的 TX 空闲时是高电平。接着驱动器 => 上拉下拉都读到 1 */
    pin_mode(GPIOC, 11, 0, 0);
    pin_pull(GPIOC, 11, 2); delay(5); pd = pin_read(GPIOC, 11);
    pin_pull(GPIOC, 11, 1); delay(5); pu = pin_read(GPIOC, 11);
    pin_pull(GPIOC, 11, 0);
    puts3("  PC11: 下拉时="); dec(pd); puts3(" 上拉时="); dec(pu);
    if (pd && pu)        puts3("  -> 有设备把它拉高：驱动器的 TX 接上了且有电 (好)\r\n");
    else if (!pd && pu)  puts3("  -> 悬空：PC11 上什么都没接(驱动器 TX 没接到 PC11，或驱动器没电)\r\n");
    else if (!pd && !pu) puts3("  -> 被拉到低电平：PC11 和 GND 短路，或接的设备没上电\r\n");
    else                 puts3("  -> 读数异常\r\n");

    /* PC10(TX)：先看外面有没有东西拉着它，再自己输出 0/1 读回来 */
    pin_mode(GPIOC, 10, 0, 0);
    pin_pull(GPIOC, 10, 2); delay(5); pd = pin_read(GPIOC, 10);
    pin_pull(GPIOC, 10, 0);
    pin_mode(GPIOC, 10, 1, 0);
    BSRR(GPIOC) = 1u << (10 + 16); delay(2); r0 = pin_read(GPIOC, 10);
    BSRR(GPIOC) = 1u << 10;        delay(2); r1 = pin_read(GPIOC, 10);
    puts3("  PC10: 下拉输入时="); dec(pd); puts3(" 输出0读回="); dec(r0); puts3(" 输出1读回="); dec(r1); puts3("\r\n");
    if (r0 != 0)      puts3("  -> ★输出 0 却读到 1：PC10 被别的设备的输出顶住(比如某个驱动器的 TX 错接在 PC10 上)，或接到了 3.3V/5V\r\n");
    else if (r1 != 1) puts3("  -> ★输出 1 却读到 0：PC10 和 GND 短路\r\n");
    else if (pd)      puts3("  -> 能正常拉高拉低，外面有驱动器 RX 把它上拉 (好)\r\n");
    else              puts3("  -> 能正常拉高拉低；但外面没有上拉，可能驱动器 RX 没接到 PC10(有的驱动器 RX 不带上拉，这条仅供参考)\r\n");

    pin_mode(GPIOC, 10, 2, 8);                            /* 恢复成 UART4 */
    pin_mode(GPIOC, 11, 2, 8);
    pin_pull(GPIOC, 11, 1);                               /* RX 加上拉，没接线时不乱收 */
}

static void query(uint8_t addr)
{
    uint8_t q[3], r[32];
    int n, i, okv = 0, okf = 0;
    uint32_t mv = 0;
    uint8_t fl = 0;

    puts3("  地址 "); dec(addr); puts3(": ");
    flush4();
    q[0] = addr; q[1] = 0x24; q[2] = 0x6B; tx4(q, 3);
    n = rx4(r, sizeof(r), 30);
    for (i = 0; i + 5 <= n; i++) if (r[i] == addr && r[i + 1] == 0x24 && r[i + 4] == 0x6B) { okv = 1; mv = ((uint32_t)r[i + 2] << 8) | r[i + 3]; }
    puts3("电压回复["); for (i = 0; i < n; i++) { hex2(r[i]); if (i + 1 < n) putc3(' '); } puts3("] ");

    flush4();
    q[1] = 0x3A; tx4(q, 3);
    n = rx4(r, sizeof(r), 30);
    for (i = 0; i + 4 <= n; i++) if (r[i] == addr && r[i + 1] == 0x3A && r[i + 3] == 0x6B) { okf = 1; fl = r[i + 2]; }
    puts3("状态回复["); for (i = 0; i < n; i++) { hex2(r[i]); if (i + 1 < n) putc3(' '); } puts3("] ");

    if (okv) { puts3(" 电压="); dec(mv / 1000); putc3('.'); dec((mv % 1000) / 100); dec((mv % 100) / 10); puts3("V"); }
    if (okf) { puts3(fl & 1 ? " 已使能" : " 没使能"); if (fl & 8) puts3(" ★堵转保护"); if (fl & 4) puts3(" ★堵转"); }
    if (!okv && !okf) puts3(n ? " -> 收到了字节但格式不对" : " -> 没回复");
    puts3("\r\n");
}

static void wheel_spin(uint8_t addr)
{
    uint8_t c[8];
    c[0] = addr; c[1] = 0x0E; c[2] = 0x52; c[3] = 0x6B; tx4(c, 4); delay(5);                         /* 解除堵转保护 */
    c[0] = addr; c[1] = 0xF3; c[2] = 0xAB; c[3] = 1; c[4] = 0; c[5] = 0x6B; tx4(c, 6); delay(5);      /* 使能 */
    c[0] = addr; c[1] = 0xF6; c[2] = 0; c[3] = 0; c[4] = 30; c[5] = 10; c[6] = 0; c[7] = 0x6B; tx4(c, 8);   /* 30 转/分 */
    delay(1000);
    c[0] = addr; c[1] = 0xFE; c[2] = 0x98; c[3] = 0; c[4] = 0x6B; tx4(c, 5);                         /* 停 */
    delay(300);
}

int main(void)
{
    uint8_t a;
    uint32_t round = 0;

    SYST_RVR = 16000 - 1; SYST_CVR = 0; SYST_CSR = 7;    /* 1ms 中断 */
    RCC_AHB1ENR |= (1u << 1) | (1u << 2);                 /* GPIOB GPIOC */
    RCC_APB1ENR |= (1u << 18) | (1u << 19);               /* USART3 UART4 */
    (void)RCC_APB1ENR;                                    /* 开时钟后等一下再动外设 */
    pin_mode(GPIOB, 10, 2, 7); pin_mode(GPIOB, 11, 2, 7);
    uart_init(USART3);
    delay(1500);                                          /* 等驱动器上电启动 */

    puts3("\r\n\r\n===== UART4(PC10/PC11) 电机通讯测试 =====  (测完记得烧回正式程序)\r\n");
    pin_mode(GPIOC, 10, 2, 8); pin_mode(GPIOC, 11, 2, 8);
    uart_init(UART4);
    pin_test();

    puts3("\r\n[2] 读每个驱动器(PC10 发问，PC11 收回复)\r\n");
    for (a = 1; a <= 5; a++) query(a);

    puts3("\r\n[3] 4 个轮子依次慢转 1 秒(车要架空！)：看哪个轮子转了\r\n");
    for (a = 1; a <= 4; a++)
    {
        puts3("  现在转 "); dec(a); puts3(a == 1 ? " 号(右前)\r\n" : a == 2 ? " 号(左前)\r\n" : a == 3 ? " 号(右后)\r\n" : " 号(左后)\r\n");
        wheel_spin(a);
    }
    puts3("  转完了。哪个轮子没转，就是那个驱动器没收到 PC10 的指令(没电/线断/地址不对)\r\n");

    for (;;)
    {
        delay(3000);
        round++;
        puts3("\r\n[2] 第 "); dec(round); puts3(" 次重新读驱动器\r\n");
        for (a = 1; a <= 5; a++) query(a);
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
