# car111 小车项目：给下一个 AI 的完整接手说明（2026-10-12 晚）

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
