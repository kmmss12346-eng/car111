# car111

STM32F407VETx 机器人小车固件（Keil MDK-ARM + STM32CubeMX / HAL 库）。仓库包含两个独立工程：

| 目录 | 来源压缩包 | 说明 |
| --- | --- | --- |
| `car_2027/` | `car_2027 - zhengshiban(3).zip` | 2027 正式版工程（CubeMX 工程 `car_2027.ioc`，Keil 工程 `MDK-ARM/car_2027.uvprojx`）。源码在 `Core/Src`，包含底盘（`chassis.c`）、偏航控制（`yaw_control.c`）、HWT101 陀螺仪（`hwt101.c`）、Emm_V5 步进电机驱动、飞特串口舵机驱动（`fashion_star_uart_servo.c`）、环形缓冲区等。 |
| `finals_2024/` | `决赛代码(1).zip` | 2024 决赛代码（CubeMX 工程 `chuankongping.ioc`，Keil 工程 `MDK-ARM/chuankongping.uvprojx`）。应用代码分目录存放：`Hardware/`（步进电机、OLED、RGB、电机、传感器、任务调度等）、`Fashion/`（舵机与串口）、`Software/`（软件 I2C、灰度传感器、DataScope 上位机）。 |

根目录下的两个 `.zip` 为原始压缩包，保留作备份。

## 构建

- 目标芯片：STM32F407VETx。
- 用 Keil MDK-ARM 打开各自 `MDK-ARM/*.uvprojx` 编译、下载；或用 STM32CubeMX 打开 `.ioc` 重新生成外设代码。
- 注意：本仓库是在云端整理的，没有硬件也没有编译验证，代码内容与压缩包原样一致。

## 整理说明

- 两个工程都保留了随包附带的 `Drivers/`（STM32F4xx HAL 驱动与 CMSIS），这样克隆后可直接打开编译，代价是仓库较大。如需瘦身，可删除后让 CubeMX 重新生成。
- `.gitignore` 排除了 Keil / CubeMX 的构建产物（`.o`、`.d`、`.crf`、`.axf`、`.map`、`.lst`、`.htm`、`MDK-ARM/<工程名>/` 输出目录等）以及 Keil 的个人界面状态（`*.uvguix.*`、`*.uvoptx`）。压缩包里原有的这些文件没有提交。
