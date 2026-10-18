# car111 小车项目：给下一个 AI 的完整接手说明（最新：2026-10-18 → 先看第 12 节；1~11 节是 10-12 ~ 10-15 写的，部分已过时）

> 用法：先读完这份，再读同一个包里的 `docs/HANDOFF.md`（更细的历史和规矩）、`docs/integration/*.md`（6 份系统分析）。
> 代码在包里 `code/` 下，和 GitHub 仓库 `kmmss12346-eng/car111` 分支 `claude/tender-heisenberg-2t1awl` 的提交 HEAD 完全一样。

---

## 1. 项目是什么

- 工程训练类比赛小车（场地约 2400×2400mm）：**启停区出发 → QR 点扫二维码得任务码 → 原料区(转盘)按任务码夹 3 个物料放进车上转盘 → 粗加工区放到 3 个圆环再取回 → 暂存区放下(第二批码垛在同色上) → 第二批再来一遍 → 回启停区**。每轮 3 分钟。
- 任务码格式 `ddd+ddd+ddd+ddd`：第 1、3 组是两批物料的颜色和夹取顺序（1红 2黄 3蓝 4绿 5黑 6浅蓝），第 2、4 组是放到几号环（1/2/3 各一个）。解析在 `task_plan.py`。
- 硬件：
  - **STM32F407**（Keil 工程，用户电脑 `E:\CubeMXcode\car_2027 - zhengshiban`）：麦轮底盘(4 个张大头 Emm 驱动，UART4)、升降(5 号 Emm)、ID1/ID2 两个串口舵机(机械臂转动/伸缩)、夹爪 PA2、车上转盘 PB3、HWT101 陀螺仪、串口屏 TJC(USART2，屏上选启停区)、和树莓派通信 USART3(`/dev/serial0`)。UART5 原本接 GM65 扫码器，**现在扫码器插在树莓派 USB 上**(名字 "USBKey Chip USBKey Module"，USB 键盘模式)。
  - **树莓派**（用户 zstu，IP 10.109.173.12，代码目录 `~/lidar_map_north`）：雷达(`/dev/ttyACM0`)建图/定位/规划路线/开车，机械臂上的 USB 摄像头(`/dev/video0`)做视觉对准，跑主程序 `run_live.sh`（= `map_merge_live.py`）。
- 用户：中文交流，在车旁边边跑边改，经常很急；日志用截图发。说话口语化（如"往前"实际指"往原料盘那边/车的右边"，要确认方向）。

## 2. 用户的规矩（必须遵守）

1. 每次改完**都推回 GitHub**（分支 `claude/tender-heisenberg-2t1awl`；好几个对话同时在推：先 `git fetch` + `git merge`，不 rebase、不 force push；不主动开 PR）。
2. 提交信息结尾加：
   ```
   Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>
   Claude-Session: <当前会话链接>
   ```
3. 交付 = zip 包 `car111_update_<月日><字母>.zip`（例如 `car111_update_1012c.zip`），里面 `car111_update_XXX/树莓派/lidar_map_north/*`（仓库 `arm_patch/pi/lidar_map_north` 下所有 git 跟踪的文件，**不放 *.json**），有 STM32 改动时加 `STM32/Core/Src/*.c`。
4. **车上独有的文件不能覆盖**：`map_config_*.json`(地图+任务配置)、`place_adj.json`、`servo_cal.json`、`vision_cal.json`、`claw_mask.png`，以及车上独有模块 lidar_map_live、ld14p_scan、pose_fix、merge_stations、reference_correction、stm32_link、home_fix。
5. 改配置给**一行 python 命令**（`run_live.sh` 没有 `--config` 参数，配置文件是 `~/lidar_map_north/map_config_start2_roi.json`），例如：
   ```
   cd ~/lidar_map_north && python3 -c "import json;p='map_config_start2_roi.json';c=json.load(open(p));m=c.setdefault('mission_cfg',{});m['raw_track_s']=40;json.dump(c,open(p,'w'),indent=1,ensure_ascii=False);print('已改好')"
   ```
   改默认值时同时在 `apply_mission_config.py` 的 `OLD_DEFAULTS` 加"旧默认 → 新默认"（车上配置里存着全部旧默认值，不升级就不生效；每条只升级一次，记在 `mission_cfg._migrated`）。
6. 回复要给详细的传输/安装步骤（见第 4 节）。用户会拒绝很耗时的工具调用，动作要快。

## 3. 代码结构（树莓派 `arm_patch/pi/lidar_map_north/`）

| 文件 | 作用 |
|---|---|
| `map_merge_live.py` | 主程序（run_live.sh 启动）：终端命令、扫描、画地图、`go`/`race`、`park`、`sync`、`set/get` |
| `auto_run.py` | 按路线开车、每个停车点雷达定位修正、到站调任务钩子、回家对准、race 流程(屏上选区+START) |
| `route_plan.py` | 路线规划（黄区/工位/原料转盘/边线余量，障碍物） |
| `mission_hooks.py` | 各停车点的任务：QR 读码、RAW 原料区夹取、ROUGH/TEMP 放置/取回/码垛、START 显示统计；`DEFAULTS` 是所有任务参数 |
| `mission_cli.py` | 调试命令：`mtest`、`gtest`、`rtest`、`vcal`、`vclaw`、`vmask`、`vdbg`、`pcal`、`mcode`… |
| `visual_servo.py` | 闭环对准（手臂/车轮，J 矩阵在 servo_cal.json） |
| `vision.py` / `matdet.py` / `ringdet.py` | 摄像头、物料颜色识别(HSV)、圆环识别 |
| `arm_link.py` | 和 STM32 机械臂指令通信；`qr()` 先问 USB 扫码器再问 STM32 |
| `qr_usb.py` | USB 键盘模式扫码器读取（**独立子进程**读 /dev/input，见 6.4） |
| `arm_calib.py` | 机械臂一次性标定向导：`python3 arm_calib.py --from <步骤名>` |
| `apply_mission_config.py` | 把 DEFAULTS 写进车上配置、升级旧默认值 |
| `sim_mission.py` + `test_*.py` | 电脑上的模拟和测试 |

STM32：`arm_patch/Core/Src/arm.c`(机械臂、升降、夹爪、转盘、扫码 UART5、串口屏、指令解析)、`chassis.c`(底盘运动)、`main.c`(舵机)、`hwt101.c`(陀螺仪/串口中断)。主机测试 `bash arm_patch/tests_stm32/run.sh`。

测试：`cd arm_patch/pi/lidar_map_north && ls test_*.py | sed 's/.py//' | xargs python3 -m unittest sim_mission`（10-12 是 479 个，全过，约 2 分钟）。

## 4. 部署步骤（照抄给用户）

**树莓派：**
1. Windows PowerShell：`scp "$HOME\Downloads\car111_update_XXX.zip" zstu@10.109.173.12:/home/zstu/`
2. 树莓派（先在 run_live 里 `q` 退出）：
   ```
   cd ~ && unzip -o car111_update_XXX.zip && cp -r car111_update_XXX/树莓派/lidar_map_north/* ~/lidar_map_north/ && cd ~/lidar_map_north && python3 apply_mission_config.py && bash run_live.sh
   ```
3. 开跑：`zone 2`（或 1）再 `go`；比赛模式 `race`（屏上选区、按 START；**屏上 START 触摸曾经不灵**，用终端：`abort` → `reset` → `zone 2` → `go`）。

**STM32 烧录：** `arm.c` 等拷到 `E:\CubeMXcode\car_2027 - zhengshiban\Core\Src\` → Keil F7 → `scp "E:\CubeMXcode\car_2027 - zhengshiban\MDK-ARM\car_2027\car_2027.hex" zstu@10.109.173.12:/home/zstu/` → run_live 里 `arm LIFT 60` 再 `q` → `openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program /home/zstu/car_2027.hex verify reset exit"`。
**烧录后 STM32 参数回到默认(ARMOK=0)**：先 `sync` 再动手臂（10-11 后的版本 `park` 会自动 sync）。

## 5. 常用命令速查（run_live 里）

- `park`（收臂、升降停 60mm）、`sync`、`get` / `set 名字 值`（存进配置）、`abort`、`q`
- `arm XXX`：直接给 STM32 发指令，如 `arm STOW`、`arm OBS RAW O`、`arm AP <ID1角度> <ID2角度>`、`arm LIFT 60`、`arm QR?`、`arm QR RAW`、`arm QR SCAN?`
- `mtest RAW 1 force`（车停原料区不动，按任务码测第 1 批夹取；force=转盘已拿空）、`mtest ROUGH/TEMP 1 force [nogo]`、`gtest 颜色号 [nogo]`
- `vdbg RAW 颜色号`（存 vdebug.png 看识别）、`vmask`、`vclaw RAW 颜色号`、`vclaw RING 外径`、`vcal RAW 颜色号`(不加 arm=测车轮)、`vcal RING`
- `pcal 1/2`、`pcal show/clear`（放置补偿 place_adj.json）
- 直接输入任务码 `652+312+526+231` = `mcode`
- 另开终端 `python3 vview.py` 看摄像头画面；`python3 qr_diag.py`（run_live 关着时）诊断扫码器

## 6. 用户定下的做法 / 当前状态（别改回去）

### 6.1 路线与停车点（在配置 `stops` 里，`[x, y, 车头角度]`，180=朝西，90=朝北）
- QR `[2100, 1350, 90]`；RAW 原料区 `[1200, 2170, 180]`（用户坚持 1200；10-11 数据：设 1250 时物料离爪子约 10mm，1200 时 35~57mm——要上车看）。
- 原料区停车点离原料转盘只有 15mm（要求 30mm），规划只在这一段放宽，会打 ⚠ 提示，正常。
- 启停区 1：和区 2 上下对称、车头都朝北；第二站动作在区 1 是"左 42、后 83、逆 60"。
- 地图边缘的障碍物忽略（boundary_guard_mm）。**行驶时不理地图障碍物**（只用于规划；`exec_obstacles=true` 恢复）。停车点旁有障碍物也停准（10-12）。
- 工位停车点雷达定位失败时，按推算把"离区域太近少走的那段"补上，停到规定点（回家那段仍宁短不长）。

### 6.2 原料区
- **按任务码顺序夹**（`raw_any_order=False`），不随机夹。
- **车到停车点就不挪车找物料**（`RAW_CAR_FIXED=True` 写死，`raw_seek` 等不管）。10-12 起对准方式 `align_by.RAW='wheels'`：手臂停在观察/夹取姿态不动，只微调车轮；`'arm'`=只动手臂(10-11 做法)。用车轮前要先在原料盘停转时做 `vcal RAW 颜色号`（不加 arm）。
- 修 1 次就下爪（`raw_max_iter=1`），30mm 内下爪（`raw_grab_max_mm=30`），强制下爪到 60mm（`raw_force_grab`、`raw_force_max_mm=60`）；每个物料最多等 `raw_track_s=40` 秒，夹不到跳过做下一个。
- 第一次夹到后记下姿态，后面从这个姿态等（`raw_keep_pose`）。
- 蓝色/浅蓝物料必须先 `vmask`（爪子是蓝色的，不然物料挨着爪子会被当成爪子去掉）。浅蓝在阴影里 V 只有 140 左右（matdet 浅蓝 V 下限 160，有一份把它降到 120 的改动在本地 stash 没推，未完成）。

### 6.3 工位（粗加工 ROUGH / 暂存 TEMP）
- 放置顺路 3→2→1（`place_order='sweep'`）。
- 放置故意偏一点：`place_adj.json`，RING(第一层)用户调好的是 `[4.6, -86]`（dID1, dID2 度；ID1 正=往 3 号环那边，约 19mm/度；ID2 负=往车近）。STACK 还要调。
- 10-12 起工位也默认只动车轮对准。

### 6.4 扫码
- 扫码器插树莓派 USB（键盘模式），`qr_usb.py` 在**独立子进程**里读（run_live 很忙时后台线程抢不到 GIL，内核只缓冲 64 个按键事件会丢键——这是 10-11 "手拿能读、跑的时候读不到"的原因；还没上车确认修好）。run_live 一启动就接管扫码器，读到码打印"扫码器读到任务码：…"。
- USB 口松过一次导致完全认不到（lsusb 里没有）；建议胶带固定。
- 用户 10-12：**不要备用任务码**（qr_fallback_code 已删）；`qr_fixed_code`(写死任务码，不扫)保留，平时应为 None。
- QR 点读不到码会前后挪着找（1200/1260/1140/1320/1080），`qr_first_wait_s=3`、`qr_dwell_s=2`（用户车上配置）。
- STM32 端加过"扫到码后用 GM65 串口命令关扫码器、QR CLR 时打开"（`QR ON/OFF/AUTO/SCAN?`），但扫码器已不接 STM32，这个功能对现在的扫码器无效（`QR SCAN?` 回 UNKNOWN noreply 是正常的）。USB 键盘模式下程序没法让扫码器停止"嘀嘀"响，只能用 GM65 设置码改。

### 6.5 跑全程
- `full_course=True`（默认）：不因时间/没任务码/没活提前回家，每个停车点都去；某个任务没做完就接着做下一个。比赛要 3 分钟内回家时改 `full_course=False, home_when_idle=True, time_limit_s=150, round_s=180`。
- 用户车上配置可能还有 `time_limit_s=10000, round_s=10000, home_when_idle=False`（测试用）。

### 6.6 其它
- 待机/行驶姿态 = STOW：升降 ZHI、ID1=A1H、ID2=A2R。改待机位置只改 A1H（`set A1H 角度`）；A2R 和转盘放料共用别动。
- 跳到某个标定步骤：`python3 arm_calib.py --from A1G`（原料盘上方姿态）/ `ZGRAB` / `ZOBRAW` / `CLWO` / `PL1` 等。

## 7. 下一步：用户要改机械结构（最重要）

用户 10-12 说：**舵机、电机都不变，只改夹子和摄像头位置**：
- 摄像头仍装在机械臂上，**摄像头中心和夹子中心完全对准**（镜头正对夹子中心），"摄像头、夹子、物料中心三点一线就夹"，夹取位置写成固定姿态。
- 这和现有代码一致（对准 = 把物料中心对到画面里的爪子点 claw_px；固定姿态 + 只动车轮对准），**预计不用重写，主要是重新标定**。已经告诉用户：
  1. 装之前备份 `map_config_*.json place_adj.json servo_cal.json vision_cal.json claw_mask.png`；装好后把 `servo_cal.json vision_cal.json claw_mask.png` 挪走（旧标定作废）。
  2. 镜头要垂直朝下（否则物料高度变化会偏）、夹子张开时镜头要能看到物料大半个圆、装牢。
  3. 标定顺序：`arm_calib.py --from CLWO` 一路调（夹爪、转盘、A1G/A2E、ZGRAB、**ZOBRAW**、A1P/A2P、ZPLC、ZSTK、**ZOBRNG**）→ `vmask` → `vclaw RAW 颜色号` → `vclaw RING 外径` → `vcal RAW 颜色号`(车轮) 和 `vcal RING` → `pcal clear` 后 `pcal 1`、`pcal 2`。
  4. 先让用户发一张 `arm OBS RAW O` 姿态下夹子张开、下面放物料的 `vview` 截图，确认遮挡和大小再标定。
- 待确认：**新夹子的颜色**（程序假设爪子是蓝色：matdet 找蓝/浅蓝物料时去掉蓝色爪子区域；新夹子不是蓝色要改 `matdet.py` 的爪子识别）；夹法是否仍是两边夹。

## 8. 还没解决 / 要留意

- 原料区：10-11 实测物料停下时离爪子点 30~60mm、修正后 v 方向始终差 ~18mm(-35px)（多半是 claw_px.RAW 不准，换夹子后重新 vclaw 会一起解决）。
- 雷达定位经常失败（"对上的点只有 7~31%"），停车点精度靠推算。
- 启停区 1 新算法只在模拟里验证过。
- 屏幕 START 触摸不灵（用终端开跑）。
- 浅蓝物料阴影里识别不出（见 6.2，改动在 stash 未推）。
- 本地 stash `raw seek-in-view + lblue V (paused)`（原料区"画面里一个物料都没有就挪车"+浅蓝 V 下限 120）：车不挪的规定下前一半已无意义；浅蓝那一行可以单独拿出来用。

## 9. 包里有什么

```
接手说明_给下一个AI.md     ← 本文件
docs/HANDOFF.md            ← 仓库里的详细交接(历史、规矩、最近改动)
docs/integration/*.md      ← 系统分析(导航执行/规划/STM32 底盘/IO/任务/比赛规则)和接口约定 spec.md
code/树莓派/lidar_map_north/   ← 树莓派代码(= 最新部署包 car111_update_1012c 的内容)
code/STM32/Core/{Src,Inc}/     ← STM32 源码(arm.c chassis.c main.c hwt101.c arm.h chassis.h)
code/STM32/tests_stm32/        ← STM32 主机测试
git_log.txt                ← 分支最近 120 条提交记录
```

## 10. 10-14 新增（最新）

- **STM32 边升边转（OVLAP=1，OVCLR=20mm）**：`arm.c` 的 `Move_Over`：先直着升 OVCLR，再一边升到 ZHI 一边转，转过一半开始往目标上方降，最后 OVCLR 直着降。用在 OBS、GRAB/PICK(夹起放进转盘)、TAKE(转到槽口上方)、DROP 后缩回；新指令 `APZ a1 a2 z`（树莓派 `ArmLink.apz`，旧固件回 ERR CMD 时自动改成 AP + LIFT）。`set OVLAP 0` 回到一步一步。
- **工位只对准 2 号环一次（zone_reach='arm'，默认）**：车在 2 号环前对准一次(车轮)，拍一张照片，1、3 号环按示教的角度直接转手臂去放/取，不再一个一个对准；照片里 1、3 号环和示教时比偏了多少像素，按 RING 的手臂 J 换成角度补上(补车停偏、车身斜；teach_fix_max_deg 3°)。
  - 示教命令（车停在该区 2 号环正前方）：`rteach ROUGH 2` → `arm AD 1 ±8` 转到 1 号环上方 → `rteach ROUGH 1` → 同样 `rteach ROUGH 3`；TEMP 一样。`rteach show/clear`。存在 `ring_teach.json`（**车上独有，不能覆盖**）。
  - 没示教过就自动照旧一个环一个环对准。模拟：车身斜 ±3° 时放置误差 ≤2mm（画面看得到 3 个环时）。
- 用户的新夹子/摄像头（摄像头中心对准夹子中心）还没装；装好后先标定（见第 7 节）再示教 rteach。

## 11. 10-15 现在的状态（下一个接手的先看这里）

- **最新部署包：car111_update_1014b**（含 1014a）。代码 = 本分支 HEAD。全部在电脑上模拟跑通，**还没上车验证**。
- **用户接下来要在车上做（等他发结果再改代码）**：
  1. 装 1014b（STM32 烧 arm.c、树莓派装包、`python3 apply_mission_config.py`）；
  2. 新夹子/摄像头装好后重新标定（第 7 节顺序）；
  3. 示教 `rteach ROUGH 2/1/3`、`rteach TEMP 2/1/3`（第 10 节）；
  4. 架空 `mtest ROUGH 1 force`、`mtest TEMP 1 force`；
  5. 跑一次全程并留日志：`bash run_live.sh 2>&1 | tee ~/run_$(date +%m%d_%H%M).log`，连同 `map_captures` 最新文件发来——用来修"雷达定位经常失败"（第 8 节）。
- **夹爪速度（10-10 另一个对话加的，已在 arm.c 里）**：`CLSPD` = 合上(夹取)速度，默认 0 = 一下子合上；`CLSPO` = 张开(放下)速度，默认 800（微秒/秒，越小越慢）。用户定的：**夹取照原来一下子，张开 800**。用户车上配置里可能存过 `CLSPD=800`，要 `set CLSPD 0`。`arm_calib.py` 里有这两步（`--from CLSPD`）。旧固件 `set CLSPO` 会回 ERR NAME = 没烧新 arm.c。
- 夹爪大小：`CLWO`(张开，用户车上 2350)/`CLWC`(合上，用户改成过 1920)；用户车上张开值 > 合上值（数大=张得大）。
- 用户输入习惯：在 run_live 里一次只输一条、等回复再输下一条（连着打会粘成一行回 ERR CMD）。
- 对话记录：`docs/对话记录/`；更早的过程在 `docs/HANDOFF.md`。

## 12. 10-18 现在的状态（最新，下一个接手的先看这里）

### 12.0 一句话
原料区改成用户 10-18 定的 **"最近那格"做法**：手臂固定在看原料盘的姿态。用 arm_calib 的"原料盘上方的姿态"把爪子摆到原料盘上**离车最近那一格**正上方，车轮只用 V 慢慢对准。原料盘转一次停一次，**按任务码顺序**，要的颜色转到这一格就直接下爪。
代码写完、仿真跑通（提交 `9e36ebd`），部署包 `near_1018k.zip` 已经发给用户，**还没上车试**。

对话原文：`docs/对话记录/2026-10-15_18_对话记录.md`（纯文字，里面有几段"上下文压缩"摘要，折叠着）。

### 12.1 规矩（这个阶段用户/系统定的，接着照做）
- **回复：**
  - 回复用简体中文。
  - **每条命令单独放一个代码块**，并提醒用户命令里用英文的 `-`（用户常打成中文的破折号）。
  - run_live 里用户一次输一条、等回复再输下一条。
- **先复述再动手：** 用户常说"你先说说我什么意思、你准备怎么干"。
  - 先用自己的话复述加计划，等用户说"对 / 开始改"再改代码。
  - 用户理解错时会很急（10-18："你是傻逼吗定位圆心都乱走"），先复述能避免。
  - 用户会打断很长的工具调用（例如一次跑全部测试）：先回答用户的问题，测试分开跑。
- **Git：**
  - 在 `claude/tender-heisenberg-2t1awl` 上提交，同时 `git push origin HEAD:claude/claim-credit-qb8b1c`。
  - 推之前 `git fetch` + `git merge`；不 rebase、不 force push、不开 PR。
  - 提交、文件里不写模型名；提交信息结尾两行：`Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`、`Claude-Session: <当前会话链接>`。
- **这个阶段的交付方式：**
  - 只把改了的文件打成一个名字唯一的 zip，例如 `near_1018k.zip`，文件直接放在 zip 根目录。用户解压到 `下载\<zip名>\`。
  - 每个文件一条命令：`scp C:\Users\Lenovo\Downloads\<zip名>\mission_hooks.py zstu@10.109.173.12:/home/zstu/lidar_map_north/`，传完重启 run_live。
- **固件烧录：**
  1. 改了的 arm.c / main.c 拷到 `E:\CubeMXcode\car_2027 - zhengshiban\Core\Src\`，Keil Rebuild all。
  2. `scp "E:\CubeMXcode\car_2027 - zhengshiban\MDK-ARM\car_2027\car_2027.hex" zstu@10.109.173.12:/home/zstu/`
  3. 在 run_live 里 `arm LIFT 60`，再 `q`。
  4. `openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program /home/zstu/car_2027.hex verify reset exit"`
  5. 烧之前用 12.6 的一行 python 检查 hex 是不是新的。
- **车上独有、不能放进包、不能覆盖：**
  - 模块：lidar_map_live、ld14p_scan、pose_fix、merge_stations、reference_correction、stm32_link、home_fix。
  - 数据：map_config_*.json、place_adj.json、servo_cal.json、vision_cal.json、claw_mask.png、**ring_teach.json**（原料区示教 RAW_CT 也存在这个文件里）。
- **CRLF 文件：** STM32 的 arm.c、main.c、chassis.c、chassis.h 是 CRLF，改的时候保持 `\r\n`（用 Python 按字节替换）。

### 12.2 机构和实车上量到的数
- **机械臂：**
  - ID1 = 旋转，多圈角度。限位 S1MIN/S1MAX，默认 44 / 244。
  - ID2 = 伸缩齿条，约 0.114mm/度，负的往里缩、正的往外伸。限位 S2MIN/S2MAX，用户 10-18 设成 -1240 / -278。
  - AEXT=2（ID2 管伸缩）。限位参数存在树莓派配置 stm32_params，每次启动发给 STM32。
- **摄像头：**
  - 装在爪子上（爪子后上方往下看），跟着 ID1/ID2 一起动。
  - 画面里的"爪子点"（claw_px.RAW）= 物料在爪子正下方时的像素位置。
  - 原料区画面约 1.97 像素/毫米，斜着看，远处被压扁。
  - 看原料盘中心时，靠车那一格在画面上方、被爪子挡住；所以按另外两格推它的位置不准。
- **ID1 的精度：**
  - 在原料盘处每度约 4.7~6.9mm。
  - 有齿轮间隙：同一个角度画面差 8mm。所以转到看的姿态都从小往大转，先到 a1-2° 再到 a1（raw_backlash_deg）。
  - OBS 到位误差约 1°（ATOLC），AP 是 0.3°。
  - ID1 摩擦补偿 S1COMP=5°，靠近限位时补偿被截掉，会停在离限位 4~5° 的地方（raw_a1_margin_deg=5）。
- **原料盘：** 物料在半径约 87mm 的圆上，三格隔 120°（边长约 151mm）。每次停约 6 秒、转约 3 秒、转 120°。
  - 转盘外圈半径约 240 像素，用 turntable_detector 认。plate['c'] 是 3 帧中值，车在走时落后；c_now 是这一帧的。
- **识别和 V 指令的速度：**
  - 实车认一帧（外圈加物料）约 0.8 秒，约 1.2 帧/秒。
  - STM32 的 V 速度指令 400ms 收不到新的就停。所以识别放后台线程，主线程每 0.15 秒续一次速度。
  - `V?` 回 STM32 自己积分的走过距离（0.1mm）。
- **原料区只用 V，不发 S/F：**
  - S/F 走完 STM32 会把车头纠回 yaw_target（开机、HOME、`R 0`、转弯时定的）。
  - STM32 复位过、车被手搬过以后，车头会突然转一大圈（10-18 实车转了 90°）。
  - 单站调试前先 `send R 0`。
- **舵机读角度：** 偶尔 ERR READ，或回 OK 不带 ANG 行。read_angles 重读 6 次，STBY SAVE 每个舵机读 3 次。
- **最后一次 `get`（10-17 12:39 UTC）：**
  - 升降：LFMAX 115，ZHI 114，ZGRAB 88，ZDROP 95，ZOBRAW 114。
  - 姿态：A1G 58.2，A1D 240.4，A1H 242，A1P 50.3；A2E -576.6，A2R -1065.6，A2P -786。
  - 其它：ASPD 180，CLWO 1600，CLWC 1280，AEXT 2。
  - 10-18 又 set 了 S2MAX -278、S2MIN -1240。
- **待机姿态 STBY**（存在 STM32 Flash，10-18 11:5x UTC 的 `arm STBY?`）：Z=114，A1=231.1，A2=-1099.7，CLAW=1600。
  - 用户在按新齿条调待机位置（当时 ID2 在 -898 左右），可能已经 `arm STBY SAVE` 过。
  - 存了 STBY 以后，开机（`arm LIFT?` 显示 BOOT=ENC/POS 才动）、`arm STBY`、PARK、STOW（不动夹爪）都回到它。**第 6.6 节"待机 = A1H/A2R"已经过时。**
- **10-18 用户把 ID2 齿条重新装过**（想多出 3~4cm），同一个 ID2 角度下爪子的位置可能变了。
  - 两头限位 S2MIN/S2MAX、A2R（放车上转盘，最要紧）、A2P 和圆环示教、STBY 都可能要重新量 / 重调。
  - 还不知道用户量没量。量两头的办法见 12.7。
- **车上 ring_teach.json 里现在的 RAW_CT**（三个停靠位那套，做完 shift 以后）：
  - shift_mm ≈ [0, 55]，圆心像素 (260, 275)。
  - 3 号位（绿）ID1 51.01、ID2 -331.67；2 号位（蓝）是推出来的；1 号位（红）够不着。
  - 用户做 `rteach RAW near` 会整个覆盖掉。

### 12.3 原料区这几天的经过（为什么变成"最近那格"）
1. **10-15 ~ 10-16：** 先后做了跟随抓取（raw_follow）、按示教停靠位等着抓、照开源方案用转盘外圈认圆心、车轮对准圆心加固定抓取位（`_raw_grabpos_*`）。
2. **10-18 用户定：** 对准圆心，三个停靠位各示教一套手臂角度，按任务码顺序抓（raw_ct：`rteach RAW center` / `rteach RAW 1~3 颜色号`）。实车上遇到的问题和修法：
   - **对准圆心时车乱走、转了 90°。** 原因是 3 帧中值落后，一步一步挪时用了 S/F，S/F 会纠车头。重写成只用 V 连续走，用 c_now，测出 J 先检查，测不出来就挪回原处停下，不再乱挪。
   - **示教两步之间圆心漂了 23mm。** 每次示教前先对回圆心（用户："每一次从头开始"）。
   - **OBS 不准，又有齿轮间隙。** AP 从同一边转过去，再按手臂 J 补画面偏差（comp）；示教完按比赛的走法验证一次（raw_teach_verify）。
   - **ID2 被限位挡住。** STM32 加了参数 S1MIN/S1MAX/S2MIN/S2MAX，可以在线改；用户电动推着量出 -1240 / -278。
   - **三个位够不全。**
     - 红、蓝两格沿伸缩方向差约 134mm，ID2 行程只有约 110mm。
     - 加了 `rteach RAW shift`，算车停在哪够得着的格最多。
     - 但被挡住的蓝色格按另外两格推出来的位置不准，shift +55mm 以后蓝色反而差 45mm 以上，红色也够不着，只剩绿色。
3. **用户想过的办法：** 先顾蓝色（离车最近）让车往后退、放开 ID1 去够绿色、把齿条重新装多出 3~4cm。最后 10-18 定："**老老实实在最近的那个地方等着合适转过来夹取就行了**"。
   - 用户先以为 vview.py 里能改爪子停的位置，后来确认是在 **arm_calib.py 的"原料盘上方的姿态"** 改 A1G/A2E：直接把爪子摆到最近那格上方，车就不用往后挪太多。
   - 用户确认按任务码顺序一个一个来；"缓慢移动" = 车轮慢慢挪过去对位。

### 12.4 "最近那格"怎么实现的（提交 9e36ebd）
- **示教 `rteach RAW near 颜色号`**（mission_cli `_rteach_raw` → MissionHooks `raw_ct_teach_near`）：
  1. OBS RAW，再 AP 从同一边转到 A1G/A2E。
  2. 认这个颜色的物料（`_raw_near_material`：3 帧一致取中值）和原料盘中心（`_raw_center_still`）。
  3. 原料盘中心的目标 = 中心 − 物料离爪子点的偏差。车轮对过去（`_raw_center_align`）：第一轮 fresh=True，现测车轮 J；车轮最多挪 raw_near_teach_s_mm / raw_near_teach_f_mm = 100mm。
  4. 再看物料还差多少，最多对 raw_near_iters=3 轮。物料离爪子点不超过 raw_near_tol_mm+1 才保存。
- **存到 ring_teach.json 的 RAW_CT：**
  - home：手臂角度；c：原料盘中心像素，比赛时对准的目标；r：半径。
  - slots：'1' = 最近那格，a = home；'2'、'3' 是绕圆心转 ±120° 推出来的或看到的位置，a=None。
  - near='1'；shift_mm：示教时车轮从停车点挪了多少。
- **比赛 `_raw_ct_body`**（near_k = D['near']）：
  1. 对准原料盘中心到 c+comp：tol = raw_near_tol_mm = 2mm；rng_c = shift_mm，车轮能挪的 ±raw_center_s_mm(40) / ±raw_center_f_mm(60) 以示教时挪到的地方为中心。
  2. 对准以后车不动，按任务码顺序等。每次停下只看最近那一格（`_raw_ct_watch(focus=(P[near], 0.5×格间距))`），画面边上一会儿认得出、一会儿认不出的物料不算"转了"。
  3. 是要的颜色、这次停下剩的时间也够，就直接 GRAB。`_raw_ct_grab` 下爪前拍一帧，物料离爪子点超过 raw_slot_accept_mm(25) 就不下爪。
  4. 同一次停下不重复算（两次停下的时间差小于 0.75 个周期）。
  5. 夹完用 `_raw_ct_pose` 回到看的姿态，再 `_raw_near_recheck`：认 3 帧圆心，偏了超过 raw_near_redo_mm(4) 就车轮再对回去。
  6. 连着 raw_ct_skip_stops(4) 次停下都没看到要的颜色转到这一格，就跳过它。
- **near 模式下：**
  - `rteach RAW 1~3 颜色号`、`rteach RAW shift` 会拒绝并说明原因。
  - 换回三个停靠位的做法：`rteach RAW center`。
  - 原料区做完，离停车点 raw_vel_back_mm(160) 以内只用 V 挪回去。
- **固件：**
  - A1G/A1D/A1H/A1P 范围改成 -720~720，A2E/A2R/A2P 改成 -5000~5000，S1MIN/S1MAX 改成 -720~720。真转的时候 main.c 还是按 S1/S2 限位截住。
  - 这样 arm_calib 才能存更往里缩的 A2E。
- **树莓派：** 放圆环时的 ID2 范围（`_a2_env`）和 ID1 的默认范围，改成按 STM32 的 S1/S2 限位，不再写死 -1210~-300 / 44~244。
- **arm_calib：** "原料盘上方的姿态"的提示改成"爪子摆到离车最近那一格正上方"。
- **仿真 RawNearSlotTests（7 个）：**
  - 示教加比赛，按顺序 3 个都夹到；示教时车要挪 60mm 也行；夹完车被碰偏 7mm 会对回去。
  - 和实车一样慢帧、V 取整、车头会转，也能示教。
  - near 下别的示教命令会拒绝；颜色不对不保存。
  - 整场两次原料区：夹 6/6、放 12/12。
- **仿真里的数：**
  - 对准容差 3mm 时，爪子离物料最多 7~9mm（仿真里超过 8mm 算夹空）；2mm 时最多约 6.6mm；1.5mm 更慢，也不更准。
  - 原料区一次约 110~140 秒：按顺序、只在一格夹，夹一个约 12 秒，夹完回来常常错过下一次停。

### 12.5 用户接下来要做（等结果再改）。near_1018k.zip 已经发给用户
1. 烧固件（这次只有 arm.c 变了），烧之前用 12.6 的命令检查 hex。
2. scp 三个树莓派文件 mission_hooks.py、mission_cli.py、arm_calib.py，然后重启 run_live。
3. 在 arm_calib 里摆爪子：
   - 车停在原料区比赛时停的位置，原料盘停着。run_live 里先 `q` 退出。
   - 在树莓派桌面（VNC）的终端里运行（会自动打开摄像头）：`cd ~/lidar_map_north`，`python3 arm_calib.py --from A1G`。
   - 到"原料盘上方的姿态"：用 `2 -50` / `2 +50`、`1 +1` / `1 -1` 把爪子摆到最近那格物料正上方，回车保存，再 `q` 退出。
   - 齿条缩到限位也没关系，车轮会补（最多 10cm）。
4. 重启 run_live，车别动、原料盘别转：`rteach RAW near 颜色号`（红 1、蓝 3、绿 4；另开终端 `python3 vview.py` 看画面）。
5. 原料盘转起来：`mcode 134+123+413+231`（颜色要和盘上的一样），然后 `mtest RAW 1`，把日志发来。
- 另外，待机位置可能要按新齿条重新存：`arm STBY` → `arm AD 2 ±10` 等摆好 → `arm STBY SAVE` → `arm STBY?`。

### 12.6 要留意的风险
- **near 的示教和对准还没在实车上试过。** 爪子在最近那格上方时，原料盘中心在画面下边缘附近，外圈只看得到上半圈。外圈认不认得出、对得准不准，要看实车日志；认不出时看 vview 里的黄圈。
- **23 秒规则：**
  - 只在一格夹又按顺序，一个颜色最多等 3 次停（约 27 秒）。等的时候 ID1 每 12 秒摆 1°（raw_keepalive_s / raw_keepalive_deg）。
  - 裁判如果严格按"等转盘超过 23 秒"算，可能会被判。已经告诉用户，要他按规则确认。
- **时间：** 仿真里原料区一次约 2 分钟，整场时间要留意。
- **示教时车可能要挪很远**（爪子摆不到最近那格时）。比赛时以示教时挪到的地方为中心，只能再挪 ±40 / ±60mm。导航停车点和示教时车放的位置差太多，就会对不上。
- **齿条重装的影响**，见 12.2。
- **检查 hex 是不是新固件**（看 hex 里有没有 -720 / 720 这一对浮点数）：
  ```
  python3 -c "import binascii,struct;d=b''.join(binascii.unhexlify(l[9:-2]) for l in open('/home/zstu/car_2027.hex').read().split() if l[7:9]=='00');print('新固件(A2E范围已放宽)' if struct.pack('<ff',-720.0,720.0) in d else '旧固件(A2E还限在-1210~-300)')"
  ```
  上一版（有 S2MAX 的固件）的检查：把 `struct.pack('<ff',-720.0,720.0)` 换成 `b'S2MAX'`。

### 12.7 常用命令（run_live 里，一次只输一条）
- **参数：**
  - `get`：看全部参数。
  - `set 名字 数值`：存进配置，每次启动发给 STM32。
  - `sync`：把配置里的参数重新发给 STM32。
- **手臂：**
  - 读角度：`arm A? 1`、`arm A? 2`。
  - 转：`arm AD 2 30`（相对转，一次最多 AMAXD=60°）、`arm AF 2 -950`（直接转到这个角度）、`arm AP a1 a2`。
  - 姿态：`arm OBS RAW O`。
  - 升降：`arm LIFT 90`；`arm LIFT?`（显示 BOOT=ENC/POS 才是开机认准了高度）。
  - 夹爪：`arm CLAW 1600` / `arm CLAW O` / `arm CLAW C`。
  - 待机：`arm STBY`、`arm STBY SAVE`、`arm STBY?`、`arm STBY CLR`、`arm STOW`、`arm PARK`。
- **量 ID2 两头**（齿条重装以后要做）：
  1. `arm OBS RAW O`；伸出去会碰到东西就先 `arm LIFT 110`。
  2. `set S2MAX 1000`，然后 `arm AD 2 30` 一步步往外推，快到头改成 `arm AD 2 10`。顶住了就 `arm A? 2`，记为 A。
  3. `set S2MIN -3000`，然后 `arm AF 2 <A-700>`，再 `arm AD 2 -60` / `arm AD 2 -10` 往里推。到头 `arm A? 2`，记为 B。
  4. `arm OBS RAW O`，两头各往里留 30°：`set S2MAX <A-30>`、`set S2MIN <B+30>`。这一步一定要做，不然限位一直是放开的。
- **原料区示教：**
  - 现在用的：`rteach RAW near 颜色号`。
  - 三个停靠位那套：`rteach RAW center [here]`、`rteach RAW 1~3 颜色号`、`rteach RAW n save` / `rteach RAW n go`、`rteach RAW shift`、`rteach RAW clear`。
  - 看示教数据：`rteach show`。
- **看画面：**
  - 另开桌面终端：`cd ~/lidar_map_north`，`python3 vview.py`。绿十字 = 爪子点，黄圈 = 转盘外圈，紫菱形 = 要对到的点，红叉 = 目标。
  - `vwatch RAW 颜色号`：一直认这个颜色，配合 arm AD 手动调；`vwatch off` 停。
- **标定：**
  - `vcal RAW 颜色号 arm`：手臂 J；`vcal RAW 颜色号`：车轮 J。
  - `python3 arm_calib.py --from A1G`。
- **测试：** `mcode 任务码`、`mtest RAW 1`、`mtest ROUGH 1 force`、`mtest TEMP 1 force`。
- **单站调试前重设车头基准：** `send R 0`。

### 12.8 代码地图（原料区）
- **mission_hooks.py：**
  - 入口：`_raw` → `_raw_body`。有 RAW_CT 就走 `_raw_ct_body`（near 或三个停靠位）；没有就走以前的做法（`_raw_center_setup` / `_raw_grabpos_loop` / 跟随）。
  - 示教：`raw_ct_teach_near`、`raw_ct_teach_center`、`raw_ct_teach_slot`、`_raw_ct_verify`、`raw_ct_shift`、`_raw_ct_margins`、`raw_ct_save_slot`、`raw_ct_goto_slot`。
  - 读示教数据和摆姿态：
    - `_raw_ct`：读 ring_teach.json 的 RAW_CT，带 near。
    - `_raw_ct_pose`：OBS 加同一边 AP，再算 comp。
    - `_raw_ct_approach`；`_arm_limits` / `_a2_limits` / `_raw_a1_range`。
  - 对准：
    - `_raw_center_align` → `_raw_center_align_smooth`：只用 V，后台线程续速度，probe 测 J，`_raw_J_bad` 检查，测不出就 go_back。
    - `_raw_center_frame`：用 plate_scan 外圈的 c_now。
    - `_raw_center_still`、`_vel_goto`。
  - 比赛时等和夹：`_raw_ct_watch`（focus）、`_raw_ct_predict`、`_raw_ct_keepalive`、`_raw_ct_grab`、`_raw_near_recheck`；做完 `_raw_seek_end`。
  - 配置默认值都在 DEFAULTS：raw_ct_*、raw_near_*、raw_center_*、raw_backlash_deg、raw_a1_margin_deg、raw_vel_back_mm 等。
- **mission_cli.py：** `_rteach_raw`（rteach RAW …）、`vwatch`；帮助文字在文件开头。
- **其它树莓派文件：**
  - vision.py：`plate_scan`（turntable_detector 外圈加物料，写给 vview 看）、`overlay`（vview 的画面）。
  - visual_servo.py：手臂 J 探测（列归一化以后算 cond）。
  - arm_link.py：read_angles 重试、vel / vel_read、params 缓存。
- **固件：**
  - arm.c：参数表 `tun[]`（Arm_Param_Set 检查范围）、`Seq_Standby` / `Seq_Stow` / `Seq_Park`、`Lift_Boot`、STBY 指令。
  - main.c：`Servo_SetLimits` / `Servo_Clamp`，SERVO1/2_MIN/MAX 是开机默认值。
- **仿真 sim_mission.py：** SimWorld（plate_stop_s 等）、SimVision（plate_scan 3 帧中值、material_errors）、RawCenterTeachTests、RawNearSlotTests、RawFrameThreadTests。

### 12.9 测试（10-18 全部通过）
- `bash arm_patch/tests_stm32/run.sh`：arm 225、chassis 222、servo 41。
- `cd arm_patch/pi/lidar_map_north` 以后：
  - `python3 -m unittest sim_mission`：94 个，约 25 秒。
  - `python3 -m unittest discover -p "test_*.py"`：430 个，跳过 3 个，约 90 秒。
- 两组分开跑（用户会打断很长的命令）。

### 12.10 这个阶段的提交（10-15 ~ 10-18 共 50 个，详见 `git log af5d7da..9e36ebd`）
- **10-15：**
  - STBY 待机姿态存进 Flash，开机、PARK 回到它。
  - 夹爪下限 500us。
  - 原料区跟随抓取、示教停靠位。
- **10-16：**
  - 用户手量的限位：ID1 44~244、ID2 -1210~-300；升降 LFMAX 115。
  - STOW 也回到 STBY。
  - APN 平滑跟随、V 速度指令。
  - 原料区车轮对准圆心、用转盘外圈认圆心（开源方案）、固定抓取位。
- **10-17：**
  - 底盘直行补偿和记录：SCDF/SCDB、TRACE?、SCCM、SCDEC、SCIM、SCBL*。
  - 轮子接线映射：WMAP/WADR/WINV/WROL/WCHK；fix SF/SB。
- **10-18：**
  - 原料区"对准圆心 + 示教三个停靠位"（84213ab 起）。
  - 读角度重试；对准圆心按实车重写（只用 V）；推算被挡住的格；shift。
  - 限位在线改（S1MIN 等）；消齿轮间隙、按比赛走法验证。
  - 最后是 9e36ebd "最近那格"。
