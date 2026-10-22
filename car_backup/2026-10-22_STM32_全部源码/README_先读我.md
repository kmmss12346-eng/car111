# 用户电脑上的 STM32 工程(2026-10-22 用户发来的 car_2027_-_zhengshiban7.zip)

用户电脑 `E:\CubeMXcode\car_2027 - zhengshiban` 的源码，原样放在这里，**只是备份**。
去掉了 `Drivers/`(CubeMX 自带的 CMSIS/HAL)、编译中间文件和 `car_2027.uvguix.Lenovo`；留了编译出来的 `car_2027.hex` 和调用栈报告 `car_2027.htm`。

## 和仓库 `arm_patch/` 比(固件 = 1ad6130，之后没改过)
- **一样：** arm.c、chassis.c、chassis.h、hwt101.c、car_2027.uvprojx。
- **main.c：** 少了 1ad6130 的"急停时舵机马上停"(`Servo_Wait` 里见 `car_abort` 就 `Servo_Hold`，5 行)。用户只拷了 arm.c。
- **arm.h：** 只差注释。
- **栈还是 1KB：** `startup_stm32f407xx.s` 的 `Stack_Size` 和 `car_2027.ioc` 的 `ProjectManager.StackSize` 都是 0x400；仓库从 10-07(5b9abcb)起是 0x1000。
  `car_2027.htm` 里最深调用要 1360 字节，还没算中断。超出 1KB 的部分会写进下面 512 字节的堆，而 `fashion_star_uart_servo.c` 的
  `FSUS_RingBuffer2Package` / `FSUS_WriteData` 用 malloc(舵机收发包就在堆里)。RAM 只用了 4.75KB，改成 4KB 没问题。
- **car_2027.hex：** 里面有 LIFTPUSH / NOZERO，是用 1ad6130 的 arm.c 编的；main.c 是旧的、栈是 1KB。
- **仓库 arm_patch 没跟踪、用户工程里改过的**(和 9/30 上传的 `car_2027 - zhengshiban(3).zip` 比)：
  `Core/Inc/hwt101.h`、`Core/Inc/stm32f4xx_it.h`、`Core/Src/stm32f4xx_hal_msp.c`、`Core/Src/stm32f4xx_it.c`。
  要在电脑外编译整套固件，用这里的 Core/ 加上 CubeMX 的 Drivers/。
