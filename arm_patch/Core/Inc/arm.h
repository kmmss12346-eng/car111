/* arm.h  机械臂总控：升降(Emm ID5)、夹爪、转盘、夹取/放置流程、二维码读取(UART5)、串口屏(USART2)
 *
 * 树莓派 -> STM32 新增指令(每条一行，和原来的 F/S/R/A/U 一样，做完回 DONE 或 ERR xxx)：
 *
 *  夹爪 / 转盘 / 升降
 *   CLAW O | CLAW C | CLAW <微秒>         夹爪张开(参数 CLWO) / 夹紧(参数 CLWC) / 直接给脉宽
 *   TT <1~3> | TT <微秒>                   车上转盘转到 1/2/3 号位(参数 TT1/TT2/TT3) / 直接给脉宽
 *   LIFT ZERO                              把"现在的高度"记为 0 (升降最低点)
 *   LIFT HOME                              让 ID5 驱动器自己回零(需先在驱动器里配好回零方式)
 *   LIFT <mm>                              升降到离零点(最低点)往上 mm 毫米(只能在 ZERO/HOME 之后)
 *   LIFT?                                  回 "LIFT <是否已回零> <当前mm> BOOT=<开机怎么认的高度>"
 *                                          BOOT=ENC / POS：编码器找准的(POS = 以 Flash 里记的上次停下的位置为中心找的)；
 *                                          BOOT=NOCAL / NOENC：没标定 / 读不到编码器，开机时直接当作在 60mm(可能不准)
 *
 *  ID1 / ID2 两个舵机的精确控制，用于摄像头闭环对准(哪个管伸缩看参数 AEXT，默认 ID2 伸缩、ID1 旋转)
 *   A? <1|2>                               读角度，回 "ANG <id> <度>"
 *   SV? <1|2>                              读舵机状态，回 "SV <id> ANG=.. V=..mV I=..mA P=..mW T=..C ST=0x.. [STALL…]"(查发热)
 *   U <1|2>                                舵机松开(可以用手摆)，下一条转动指令会自动重新出力
 *   SVP? <1|2>                             读舵机内部的保护设置，回 "SVP <id> RESP=.. STALLM=.. STALLP=.. … PMAX=.. IMAX=.. …"
 *   SVW <1|2> <名字> <数值>                改一项舵机内部设置(名字见 SVP?，例如 SVW 1 PMAX 15000)，改完读回来
 *   舵机转动时允许的最大功率是参数 SPOW(mW，默认 30000)：转不动(停住时电流大、ST 有 0x40)就 SET SPOW 加大
 *   舵机没转到位又停住不动了：电流大(在顶着东西)就让它停在原地、不再顶着发热，打印 "SERVOn STUCK, hold here at=.. target=.. I=..mA ST=..";
 *   电流小(只是停顿)就重发一次目标，打印 "SERVOn PAUSED, resend …"
 *   补偿 S1COMP / S2COMP(度，默认 ID1 5、ID2 0)：手臂有摩擦，舵机总停在离目标差几度的地方，发给舵机的目标就往转的方向多给这么多，
 *   一到真正的目标(或者刚转过一点)马上让它停在那里，不再往前使劲。总差一点就加大，总转过头就减小
 *   小步(< 4.5°)自动慢转(15°/s)、到位前每 8ms 查一次角度，0.5° 这样的小调整也能停准；AP 和整套动作里已经在目标附近(ATOL 内)的舵机
 *   不再转、只在原地保持(松开过的也重新上力)；AF/AD 照常转
 *   AF <1|2> <度>                          转到绝对角度(支持小数，到位误差 ATOL)
 *   AD <1|2> <增量度>                      在现在的角度上再转(支持小数)，回 "ANG <id> <度>"
 *   AP <ID1度> <ID2度>                     ID1、ID2 一起转到指定角度(回到记下来的对准姿态用)
 *
 *  夹取 / 放置分步指令(摄像头闭环用)
 *   OBS RAW [O] | OBS RING [O]             手臂摆到"观察姿态"：原料盘上方 / 地上圆环上方(O=同时张开夹爪)
 *   GRAB <n> H | PICK <n> H                手臂已经对准：夹起物料放进转盘 n 号位(GRAB 用原料盘夹取高度，PICK 用地面圆环高度)
 *   TAKE <n>                               从转盘 n 号位取出物料，夹着停在转盘上方
 *   DROP [S]                               手臂已经对准：下降、松开、抬起、伸缩舵机缩回(S=码垛，放在已有物料上)
 *   STOW                                   收臂待命(升到最高、缩回)
 *   PARK                                   停车待机：像 STOW 那样收臂，再把升降降到 60mm(下次开机认高度最稳)。
 *                                          ARMOK=0 时只降升降；升降位置不知道时回 ERR NOZERO
 *
 *  不用摄像头的完整流程(调试、应急)
 *   GRAB <n>        = OBS RAW O  + GRAB <n> H
 *   PICK <n>        = OBS RING O + PICK <n> H
 *   PLACE <n> [S]   = TAKE <n> + OBS RING + DROP [S]
 *
 *  二维码 / 串口屏
 *   QR?                                    回 "QR <任务码>" 或 "QR NONE"(车在走的时候扫到的码也记着：中断里就认好、存下来)
 *   QR CLR                                 清掉已读到的任务码
 *   SCR <控件名> <文字>                    往串口屏某个文本控件写字，例如 SCR t0 452+321+254+312
 *                                          (选区页还在显示时，先自动回到比赛布局、关掉触摸，和 ZONE LOCK 一样)
 *   SCMD <淘晶驰指令>                      直接发一条串口屏指令，例如 SCMD page 1
 *
 *  选启停区(屏上触摸。开机屏上就是选区页：左边大按钮 1、右边大按钮 2；选好进准备页：ZONE n、状态行、START、BACK)
 *   ZONE?                                  回 "ZONE <区> <s>"：区 = 选了几区(0 = 还没选)，s = 1 表示选区以后按过 START
 *   ZONE ASK                               画选区页，清掉选的区和 START，打开触摸
 *   ZONE 1 | ZONE 2                        直接选区(和在屏上按一样)，画准备页，START 清掉
 *   ZONE MSG <文字>                        写选区页/准备页的状态行(只要 ASCII，最多 20 个字)
 *   ZONE LOCK                              关掉触摸，清屏回到比赛布局(选的区和 START 还记着，ZONE? 照样能查)
 *
 * 另外 SET / GET 可以改/看机械臂参数(名字见 arm.c 的 tun 表)。
 * 夹爪、转盘、两个舵机的姿态、升降各个高度，用树莓派上的 arm_calib.py 一步一步标定最方便。
 *
 * STM32 -> 树莓派 额外会主动发：QR <任务码>   (扫码模块读到合格的任务码时发一次；同时自动把任务码分两行写到串口屏的 t0 和 t7：
 *                                            t0 = 前两组带后面的 +，如 452+321+；t7 = 后两组，如 254+312)
 */
#ifndef __ARM_H
#define __ARM_H

#include "main.h"

#define SERVO_FINE_DEG  4.5f   /* 离目标这么近的小动作(摄像头微调，一步最多 4°)：转慢一点、查得勤一点，到了马上停，停得准 */

void Arm_Init(void);                 /* 开机调用一次：启动夹爪/转盘 PWM，开始接收二维码 */
void Arm_Poll(void);                 /* 主循环里反复调用：处理扫码模块收到的任务码、屏上的触摸 */

/* 返回 0 = 这条不是机械臂的指令，请继续往下判断
 *      1 = 做完了(调用者再回 DONE)
 *     -1 = 出错，err 里已经填好 "ERR xxx" */
int  Arm_Command(const char *cmd, char *err, int errlen);

int  Arm_Param_Set(const char *name, float v);   /* 0=没有这个名字 1=成功 2=超出范围 */
void Arm_Param_Dump(void);

void Arm_QR_RxCplt(void);            /* UART5 收到 1 字节(在串口中断回调里调用) */
void Arm_QR_RxRestart(void);         /* UART5 出错后重新开始接收 */
void Arm_Scr_RxCplt(void);           /* USART2 收到串口屏 1 字节(触摸坐标；在串口中断回调里调用) */
void Arm_Scr_RxRestart(void);        /* USART2 出错后重新开始接收 */

void Screen_Text(const char *obj, const char *text);

#endif
