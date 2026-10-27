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
- **按任务码顺序夹**（`raw_any_order=False`），不随机夹。**例外(10-21 夜用户"对准了之后就应该立刻识别颜色然后夹取")**：最近那格做法 `raw_near_any_order=True`，这一格上停的是这一批要的哪个就先夹哪个；规则按任务码顺序算抓取分，已告诉用户，要按顺序改 false。
- **车到停车点就不挪车找物料**（`RAW_CAR_FIXED=True` 写死，`raw_seek` 等不管）。10-12 起对准方式 `align_by.RAW='wheels'`：手臂停在观察/夹取姿态不动，只微调车轮；`'arm'`=只动手臂(10-11 做法)。用车轮前要先在原料盘停转时做 `vcal RAW 颜色号`（不加 arm）。
- 修 1 次就下爪（`raw_max_iter=1`），30mm 内下爪（`raw_grab_max_mm=30`），强制下爪到 60mm（`raw_force_grab`、`raw_force_max_mm=60`）；每个物料最多等 `raw_track_s=40` 秒，夹不到跳过做下一个。
- 第一次夹到后记下姿态，后面从这个姿态等（`raw_keep_pose`）。
- 蓝色/浅蓝物料必须先 `vmask`（爪子是蓝色的，不然物料挨着爪子会被当成爪子去掉）。浅蓝在阴影里 V 只有 140 左右（matdet 浅蓝 V 下限 160，有一份把它降到 120 的改动在本地 stash 没推，未完成）。

### 6.3 工位（粗加工 ROUGH / 暂存 TEMP）
- ~~放置顺路 3→2→1（`place_order='sweep'`）~~ **10-22 起一律按任务码顺序放、按任务码顺序取回**(规则：必须按照任务码的顺序和位置放置；见文末 10-22)。
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

### 12.11 10-19 实车第一次 `rteach RAW near 3`
- **出的问题：** 报"画面里没看到 BLUE 物料"，日志里写"原料盘上认出的颜色：LIGHT_BLUE"。
  - 原因：matdet 里蓝(3)和浅蓝(6)的颜色范围有重叠，按饱和度中值分（S ≤ 180 算浅蓝）。蓝色物料在爪子正下方显得淡，被判成了浅蓝。
  - 那一步认物料用的是 material_errors，不往 vview 发画面，所以 vview 一直显示 waiting。
- **用户定：** "直接把浅蓝色和黑色去掉，省赛根本用不到"。
- **改了：**
  - matdet 新增 `no_lblue=True`（默认）：浅蓝的范围并进蓝色，蓝色不再和浅蓝按饱和度分开。
  - 示教时没有任务码就只认红、黄、蓝、绿（`RAW_COLORS`）。
  - `rteach RAW near` 和 `rteach RAW n 颜色号` 只收 1~4。
  - near 示教认物料改用 plate_scan，画面会发给 vview。
  - 认不出时，报错里列出认到了哪些颜色、离爪子点多远。
- **还留着：** 任务码解析还是允许 1~6；比赛时只认这一批要的颜色。
- **部署包：** `blue_1019a.zip`（matdet.py、mission_hooks.py、mission_cli.py）。

### 12.12 10-19 run_live 里 USB 扫码器读不到码（扫码器会"嘀"）
- **现象：** `mtest QR`、`qr` 都说没码；`qr` 显示「USB 扫码器在 /dev/input/event4，还没读到码」。扫码器每次都"嘀"。
- **qr_diag.py（不独占）每次都读到：** 按键是 `CAPSLOCK 1 3 4 RSHIFT = … 2 3 1 ENTER`，打完再按一下 `CAPSLOCK`，整条 0.065 秒，没有丢键。
  把这串键喂给 qr_usb.Reader 也认得出来，所以解析没问题。
- **判断：** 扫码器每扫一次先按 CapsLock，要看到电脑把它的大写灯翻过来才接着打码（打完再按一下恢复）。
  run_live 的读码进程独占（EVIOCGRAB）扫码器以后，系统收不到这个键、不翻灯，扫码器就卡住：只"嘀"不打码。
  佐证：10-11 第一次能读到，是在还没独占的时候；之后 run_live 里一直读不到（当时以为是没对准码板、或 GIL 丢键）。
  **10-19 实车证实：** 装了 qrled_1019h 以后，run_live 一启动扫码就出现「扫码器读到任务码：431+231+134+213」，`qr` 设上了任务码。
- **改了（qr_usb.py）：**
  - 设备改成读写打开（没写权限就只读）。
  - 独占着的设备按 CapsLock/NumLock 时，先用 EVIOCGLED 读现在的灯，再往设备写 EV_LED 把灯翻过来，和系统做的一样。
    没独占的设备不管，系统自己会翻。
  - `QR_USB_GRAB=0` 环境变量：不独占。码会打进终端，灯由系统翻；备用办法。
  - 新提示（同一种不刷屏）：
    - 按了 4 次 CapsLock 还没出码；
    - 翻灯失败；
    - SYN_DROPPED 丢键；
    - 扫码器打出来的一行不是任务码。
  - 读码子进程退出时日志会说一声，并自动重开，最多 3 次。
- **qr_diag.py：**
  - `python3 qr_diag.py 30 grab`：像新的 run_live 那样独占，并替它翻灯。
  - `python3 qr_diag.py 30 grab noled`：独占但不翻灯，和以前的 run_live 一样。
  - 每个键后面标出隔了多久（超过 5ms 才标），每行最后写出拼出来的任务码。
- **如果翻灯还是不行：** 先跑上面两个 grab 对比；再试 `QR_USB_GRAB=0 bash run_live.sh`。
- **部署包：** `qrled_1019h.zip`（qr_usb.py、qr_diag.py）。

### 12.13 10-19 摄像头掉线换编号、雷达只认出 1 个圆柱
- **摄像头：**
  - 实车上 USB 摄像头掉线又连上，`/dev/video0` 没了，变成 `/dev/video1`（取画面）和 `/dev/video2`（元数据，没画面）。
  - 程序写死 `/dev/video0`，所以 `mtest RAW` 报「打不开摄像头」。当时拔插摄像头就好了。
  - 改了 vision.py：
    - `Camera.open` 先开配置的节点，打不开（或读不到画面）就找插着的 USB 摄像头。先看 `/dev/v4l/by-id/usb-*-video-index0`，没有再扫 `/sys/class/video4linux`，只认 USB、index 0 的节点。video10 以上是树莓派自己的编解码器，不会选到。
    - 用着用着连着 2 秒读不到画面（配置 `camera.reopen_s`，0 = 不重开），就关掉重开，最多每 3 秒试一次，不抛错。
    - 按「连着读不到多久」算，不按「上次读到画面」算：开车时几分钟不要帧，回来第一次没等到不会误重开。
    - 打不开时，报错里写找到了哪些 USB 摄像头、哪些程序开着 `/dev/video*`。
  - 测试在 test_vision.py 的 UsbCameraTests（假 cv2，加临时目录里的假 by-id/sysfs）。
- **雷达只认出 1 个圆柱**（场上 3 个）：
  - `pts`：第一次扫描场地里只有 C1 (1194,1585) 和原料转盘那团；第二次扫描（车转 60° 以后）场地里 0 个点。
  - 新工具 `lidar_blind.py`：读 map_captures 里的扫描原始帧，按方向统计。结果：
    - 雷达在车左边（前 22.5、左 134.8mm，只比车身宽 5mm），没有大片被 35cm 内的东西挡住。
    - 第二次扫描时，左边 1.5~2.2 米的圆柱、转盘都没有回波，可同方向 3 米外的墙、1 米处的 QR 码板有回波。判断是激光高过了 10cm 圆柱顶：雷达偏高，或往左翘。
    - 第一次扫描车头正前方 0°~+10° 一个回波都没有。照片里雷达在前左轮正后面，判断是被轮子挡住。S1 下面那个圆柱当时正好在 +7°。
  - 已让用户量雷达出光窗口离地高度、用手机水平仪看左右/前后倾斜。**等结果再改**（降低雷达或垫平）。
  - 右下角那个圆柱是倒着的（5cm 高），扫不到。比赛障碍物是立着的 φ50×100。


### 12.14 10-19 工位(粗加工区/暂存区)：车只对准 2 号环，1、3 号环只动手臂（提交 0a98660）
- **用户要求：** 整车先对准 2 号环；1、3 号环按已知环距(规则：1-2、2-3 各 150mm，在 580×150 的条上)算位置，只动手臂不动车，按任务码放。工位那边手臂够长。
- **做法：**`zone_reach='arm'` 时 `_p2_align` 让车轮对准 2 号环(记下 p2 角度和车的位置 `_p2_at`)；1、3 号环的角度优先级：
  这次第一批对准时量到的(`_zone_learn`) > 示教的(`rteach`) > 按几何算(`_zone_geom`：`J_arm`(RING 'arm' 列 [ID2, ID1]) + `J_ch` 的前进方向 + `vision.scale('RING')`，按手臂绕转轴转的极坐标模型)。
  算出来超出手臂范围(`_zone_limits`：S1MIN/S1MAX 等收进 `zone_a1_margin_deg`/`zone_a2_margin_deg`)就照旧挪车对准。
  只动手臂到了以后再用画面细对一次(`_zone_arm_fine`，`fixed_j=True`、`allow_chassis=False`)；码垛对下面那个物料。
  车挪过(>1mm)就回 2 号环重新对准；取回时只有"只动手臂放的、车没动"的才按放下时的角度直接取。
- **前提：**做过 `vcal RING arm`(手臂和画面的对应关系)，否则日志说"没量过手臂和画面的对应关系"，照旧每个环挪车对准。
- **实车风险：**10-17 A1P≈50.3°、S1MIN 44：1、3 号环要 ID1 ±27° 左右，可能有一个环超出 ID1 范围 → 那个环照旧挪车。
- 仿真：ZoneArmGeomTests(5 个)；模拟器 `SimWorld(arm_r_mm=...)` 按真车手臂(转轴 + 齿条)算、摄像头跟着 ID1 转。

### 12.15 10-20 原料区(最近那格)：车停偏了也一直找、按偏差直接算（提交 450c89e）
- **用户：**"到原料区偏差稍微大一点就不识别了，偏差大一点也要一直识别对准"；"识别的时候是不是写死了先向左再向右挪着看，应该根据偏差的位置来算"；"原料区耗费时间太久"。
- **根本原因：**最近那格示教时原料盘圆心在画面 (340, 471)，就在下边上(画面 640×480)。turntable_detector 的粗找(Hough)要求圆心在画面里，
  车往外停偏几毫米圆心就出画面 → 认不出 → 以前等 12 秒再按 `raw_center_search_mm` 写死的点挪着找；找到了车轮也只能挪 ±40/±60mm → 对不上就整批不抓。
- **vision.py：**`plate_scan(colors, hints=None)`：照原来的办法认不出外圈、又给了 hints(哪怕是 [])：
  - `_plate_vote`：缩小的画面上每个边缘点沿梯度往两边各走一个外圈半径投票，票都投在圆心上(投票的格子比画面大，圆心在画面外也算得出)，几毫秒；
  - `_plate_seek`：在投票的峰和 hints 上沿径向找边、鲁棒拟合(和 turntable_detector 局部跟踪一样)，要求露出 ≥90°、残差 ≤4、边缘强度 ≥12(防地面纹理)、半径和猜的差 ≤25%；
    找到就 `tt._accepted` 当跟踪种子(以后每帧局部跟，圆心在画面外也跟得住)，外圆 dict 里 `seek=True`；
  - 露出太少拟合不准、只有投票明显：返回 `coarse=True` 的大概圆心(只用来知道往哪挪)；`last_seek` 记最像的那段差在哪；`last_scan_ms` 记每帧物料/外圈耗时。
  - 外圈半径：Vision 记上次认出的 `_rim_r`；mission 给 hints 第 3 个数(示教时存的 `RAW_CT.rim`，没有按 物料圆半径/0.77 估)。
- **mission_hooks.py `_raw_center_align_smooth(search=...)`**(只有最近那格比赛时 `_raw_near_search(D)` 给；示教、三个停靠位做法不变)：
  - ① `seek()`：每帧 hints=[最可能的圆心(车对准时在 rc)、上次认出的]；认出外圈(停下连 3 帧一致)就返回；`coarse` 或"只看到一个物料且原料盘停着"
    (按示教时最近那格→圆心的向量 `near_vec` 推)就直接往算出的位置挪；外圈物料都看不到才按 `raw_near_step_mm` 在范围里从近到远一处一处看(往原料盘那边的优先)。
  - 范围 `_raw_near_box`：以示教时停的地方(rc=D.shift)为中心，往原料盘那边 `raw_near_toward_mm`(40)、往外 `raw_near_away_mm`(80)、前后 `raw_near_along_mm`(±120)；
    哪边是原料盘按 J⁻¹(爪子点−示教圆心) 定，没有 J 按地图(停车点→`raw_center_mm`)；有地图位姿就 `_body_clear` 查车身出不出场地、离黄色区/工位/地图障碍物(ctx.obstacles，不管 exec_obstacles)，那边少挪。
  - 认出圆心后 `widen()`：对准的地方就是示教时车停的地方(相对原料盘一样，不会碰原料盘)，在范围外也把范围放大到包住它，最多 `raw_near_max_mm`(150)。
    **防认错撞原料盘：**按猜的圆心/投票找到的外圈(vision 一直标 `seek=True`，跟着它的帧也是，直到跟丢或粗找重新认出)要画面里的物料在物料圆上
    (`_rc_conf`)才算确认；没确认时往原料盘那边只放大到原来的 `raw_near_toward_mm`，挪近了确认了(卡在范围边上 3 秒时再看)再放大。
  - ② 存过 RAW 'ch' 就直接用(`_raw_J_stored`，比赛时不再挪着测)。
  - ③ 对准时 hints=[预测的圆心]；连着 `raw_near_lost_s`(4)秒认不出 → 停下回 ① 接着找；车轮被范围卡住 3 秒还差就停；①②③ 一共 `raw_near_find_s`(90)秒。
    认出外圈后每 `raw_center_mats_every`(5)帧才认一次物料。**只有真认出圆心的帧才算对准**(以前光靠推算也能"对准了"，修掉了)。
  - `_raw_center_frame(colors, claw, near, hints, mats)`：按 seek 找到的外圈，画面里的物料要在物料圆上(±30%)才算；coarse 放 `self._rc_coarse`。
- **省时间：**夹完一个以前要停下来专门认 3 帧看车偏没偏(`_raw_near_recheck`)，回来常常差 1 秒赶不上下一次停、要多等一整圈(实车日志"只剩 0.3 秒(下爪要 1.4 秒)")。
  现在 `_raw_ct_watch(centers=[])` 等原料盘时用 plate_scan 顺便认外圈，`_raw_near_redo` 判断要不要对回去。
- **示教**(rteach RAW near/center)现在多存 `RAW_CT.rim`(外圈半径像素)。没重新示教也能用(按物料圆半径估)，重新示教一次更稳。
- **模拟器：**`SimVision.plate_scan(colors, hints)` 按真检测器建模：粗找要圆心在画面里、认出过就跟(45 像素内)、hints/投票能找回(半径差 ≤30 像素)、露出 <90° 只给 coarse。
  新测试(RawNearSlotTests)：停远 5/10cm、前后偏 10cm、对准中途挡住 6 秒、什么都看不到、只看到一个物料、地图查范围；test_vision.PlateSeekTests(合成画面)。
  模拟里停远 10cm：一来就认出、13 秒对准、三个都夹到。
- **没解决：**原料区总时间主要是按顺序等颜色转到最近那格(原料盘停 ~5 秒、转 ~3 秒，按任务码顺序最坏每个等两圈)。规则按顺序算抓取分，`raw_any_order` 默认不开。

### 12.16 10-20 雷达：每个停车点定位帧数 8 → 4
- 用户嫌"每一步都用雷达太花时间"。`reloc_frames` 默认 8 → 4(auto_run、map_merge_live 站点扫描复测)；路线中间的定位一直是 4 帧。一帧 600 多点，4 帧也比 ICP 用的 `reloc_points`(600)多。
- 停车点定位日志多了"一共用时 X 秒"、"测了两次"(修正量 >50mm 会再测一次)。下次看实车日志再决定还能省哪(比如 QR 站点扫描 12 帧)。
- 想过"到工位时雷达在后台拍、和手臂摆姿势同时做"：工位车轮横移最多 ±20mm(`ring_strafe_max_mm`)，必须先定位修正再做任务；只有原料区能并行，一次省 1 秒左右，没做。
- **车上配置里如果写了 `reloc_frames` 就按配置的**(map_config_*.json 是车上的文件，不能覆盖)。

### 12.17 10-20 工位(粗加工区/暂存区)：2 号环车轮连续对准一次，之后车不动；1、3 号环按 arm_calib 调的角度只转手臂
- **用户：**"对准二号环之后车保持不动……只动机械臂去放置物料"；"对准 1 号 3 号我可以自己去对准，加到 arm_calib.py 的校准过程中"；
  "粗加工区和暂存区的位置是一样的，不需要重复对准"；"从车上转盘放到工位，手上有物料，转过去的位置应该和原料区不一样，怕挡到电池盒"；
  "怎么实现车缓慢对准 2 号环，可以借鉴原料区的代码"；"一定要一次改好"。用户的照片 + 路线：两个区都是 1 号环在车头前方(同一边)，一份标定两个区共用。
- **固件 arm.c(提交 1f7d247)：**新参数 R1T1/R2T/R1T2/R2SPD = 车上转盘 <-> 地上圆环这条路单独一组绕电池盒(意思同原料区 A1T1/A2T/A1T2/A2SPD)。
  `g_side`(OBS RAW/RING、GRAB=原料盘、PICK=圆环、PLACE=圆环、新指令 `SIDE RAW|RING` 设)；`Detour_Get`：在圆环那边且 R2T≠0 用圆环那组，否则原料区那组；
  `Leave_Tray` 圆环那组：转过 R1T2 伸到 R2T(要去的比 R2T 还长就直接到)，转过 R1T1 再到目标长度；`Sweep` 多一个 ID2 速度参数。arm.h 加了 SIDE 的说明。
- **arm_calib.py：**A1P/A2P 那步改成"2 号环"(车停在比赛的停车点远近)；后面加 R1、R3(kind='ringpose'：爪子摆到 1/3 号环正上方，存"比 A1P/A2P 多转多少"
  到 ring_teach.json 的 ROUGH 和 TEMP，`src='arm_calib'`，删掉旧的 e1/e3；以前存过就从存过的位置开始)；再加 R1T1/R2T/R1T2/R2SPD(夹着物料调，做完按原路回 2 号环上方)。
  旧固件没有 R1T1 等参数就跳过那四步。test_arm_calib 37 个。
- **mission_hooks.py：**
  - `_teach(zone)` 多返回 `cal`(src=arm_calib)。没调过：★★ 大声提醒去 arm_calib 调，照旧一个环一个环挪车对准(`zone_arm_geom` 默认改成 False，不再按环距算)；rteach 示教的照旧(转过去还用摄像头再对)。
  - `_p2_align`：align_by 是 wheels 且 `zone_smooth`(默认开)：`_ring2_smooth` —— 手臂先精确回 A1P/A2P(`zone_obs_fine`，OBS 大动作只到 1° 内，差 1° 到 1、3 号环偏 2mm 多)，
    车轮只发 V(新的 `_VelDrive`：V? 量走了多少、到范围边上不再往外走、识别放后台线程每 0.15 秒续 V)；每帧按车走了多少推 2 号环在画面哪(看三个环时记的 `ring_uv`)，
    只认推算位置 0.4 个环距以内的；偏多少 mm 就每秒走多少(最快 `zone_vel_speed` 15、最慢 5)；到容差停下，停稳 `zone_settle_s` 后拍 3 帧取平均再决定(容差 = tol_mm.RING)；
    横移限 ±`ring_strafe_max_mm`(相对停车点)、沿那一排 `ring_f[2]`±`zone_f_mm`(60)；到边上了/到时间了还差：只动手臂补(`servo.run allow_chassis=False`，`_arm_only_kw`)；
    跟丢 `zone_lost_s`(3)秒停下重新找；挪了 `zone_stall_s`(6)秒偏差都没小 0.5mm(打滑、没走起来)也交给手臂补；一开始停着找 `zone_look_s`(10)秒找不到：暂存区第二批 2 号环被盖住就改 `_align_covered`(物料顶面/外圈)一步一步对，
    否则这一趟这个区 `_p2_failed`(不再试)，照旧一个环一个环对准。没有 RING 'ch' J、或只测过前后(synth)、或越走越偏：返回 'noj' 照旧一步一步(servo.run，会测 J)；不认 V：'nofw' 同上。
  - `_place_item_arm`：cal 时 1、3 号环 = p2 + 调的角度(+ PL1/PL2 补偿)，**不再用摄像头对、不挪车**；`_cal_target` 按真限位(不收余量)查，超出 `zone_cal_over_deg`(ID1 2°、ID2 40°)以内按限位放，再多这个环不放(物料留车上)。
  - 取回：放下时记了 `arm=True` 且车没动 → 直接按放下的角度取(以前就有)。
  - `_zone_start` 发 `SIDE RING`，`_raw` 发 `SIDE RAW`(旧程序 ERR CMD 以后不再发)；`_put_back`(夹着物料放回转盘)按绕电池盒的角度分段 AP(`_to_tray_detour`)。
  - `_zone_back`：这一趟车只用 V 动过(`_zone_v_only`)就只用 V 挪回停车点；发过 S/F(照旧挪车对准过)就照旧 S/F。
  - `ring_offset_mm.TEMP` 改成和 ROUGH 一样(1 号 +150)：只在没有 RING ch J 时用。
- **mission_cli.py：**mtest 开始前说这个区 1、3 号环调没调、2 号环能不能连续对；`rteach ROUGH|TEMP` 写 `src='rteach'`(不会顶着旧的 arm_calib 标记)。
- **模拟器：**`SIDE` 指令、`take_sides`；ZoneCalTests(12 个：整场两批车只对 2 号环一次之后不动、停偏 45mm+离远 30mm 手臂补、跟丢再找、旧固件、没调过、超限位、
  没 vcal RING、第二批 2 号环盖住、OBS 误差、车轮打滑、SIDE、放回转盘绕电池盒)；RawFrameThreadTests 加 2 号环真实时间后台线程续 V。模拟误差平均 0.55mm/最大 0.98mm。
- **测试：**466 单元(3 跳过) + 119 模拟全过。
- **要留意：**① R1/R3 是相对 A1P/A2P 的，停车点离圆环那一排的远近要和调的时候一样(横移到头手臂补伸缩时，1、3 号环会偏 0.46×补的毫米数)；
  ② ID1 有齿轮间隙：调 R1/R3 时从 2 号环转过去，比赛时从车上转盘转过去，方向不一样可能差一点——实车放偏了先看是不是总偏同一边；
  ③ R1T1 要选在从 2 号环往车上转盘转、1 号/3 号环那边都碰不到电池盒的地方。

### 12.18 10-20 晚：四个任务(2 号环认环、夹爪三个张开角度、去掉 PL1/PL2、停车点自己输入)
- **用户：**"工位就是类似于这个圆台去定位这个 2 这个点就好了，你不要让车乱跑了，定位好之后我就按照这个相对位置去夹取放置"；
  "找不到环不要后退，要往前慢慢去找"；"原料盘上方张开一个角度、车上物料盘上方另一个角度、其他位置另一个角度"(从转盘里取之前张开也用转盘上方那个——用户确认"对的")；
  "把 28、29 步的补偿去掉"；"这四个停靠位置我需要自己输入选定合适位置"；"改了点的位置路线容易规划不出来报错"；"障碍物离这四个点太近车跑不到指定位置"。
- **任务一 2 号环(提交 5eb6dfd，mission_hooks / mission_cli)：**
  - `_ring2_smooth` 认环：半径要在到工位时看到的三个环中位数(`_ring_rmed`)的 0.75~1.33 倍；车按推算走了 ≥25px 但画面里的圈 ≤8px 没动：
    `fixed_or_slip` 看别的环动没动——别的环动了 = 车上的东西(拉黑，以后不认)，都没动 = 车轮打滑(交给手臂补)。
  - 边走边修车轮和画面的对应：拿看到的环位置和 J0·车走的距离做相似变换拟合(比例 0.7~1.4、转角 ≤35°、残差 ≤8px)，修正明显、对准成功就存下。
  - 横移：目标 = 让偏差为 0 的 S(就是 arm_calib 调 2 号环时车离那一排的远近)，范围 [目标 ± ring_strafe_max_mm] ∩ ±`zone_s_max_mm`(70)，再按地图(`_body_clear`)收。
  - 开始时画面里不到两个环：只往前 `zone_seek_speed`(10mm/s)慢慢挪，最多 `zone_seek_mm`(200)，看到两个环就停，从后往前第二个是 2 号环；一开始就看到两个环不挪。
  - 调过 1、3 号环(src=arm_calib)的区：2 号环没对准就这个区不放(物料留车上)，**不再一个环一个环挪车**；存的 1、3 号环 ID1 比 2 号环差不到 `zone_cal_min_deg`(5°)当没调(多半是直接回车存了 0)。
    配置 zone_reach 不是 arm 但 arm_calib 调过：照样按新做法(log 说一次)。mtest 开跑前的说明和这个判断一致。
  - `place_comp_on`(默认 False)：place_adj.json(PL1/PL2、pcal 量的)和 place_shift_mm 一律不加；pcal save 时提示。
- **任务三 去掉 PL1/PL2(提交 2cafdaf)：**arm_calib 删了 PL1/PL2 两步和它们专用的摄像头对准(_WizArm/_cam_align/_open_vision/_open_store/_setup_place)，
  Wizard 不再有 vision/store 参数；`--from PL1` 说已去掉、改从 A1P 调。
- **任务二 夹爪三个张开角度(提交 b376de6)：**
  - 固件 arm.c：新参数 CLWOR(原料盘上方)、CLWOT(车上转盘上方)，0 = 和 CLWO 一样(1~499 拒绝)。`Claw_OAt(CLAW_AT_RAW/TRAY)`；
    OBS RAW O 和 GRAB 下爪前用 CLWOR；GRAB/PICK 松开放进转盘用 CLWOT；TAKE 先照旧 CLWO 转过去，到转盘上方再换 CLWOT 下去夹；DROP、PICK 下爪前、OBS RING O 用 CLWO。
    新指令 `CLAW OR` / `CLAW OT`。`Seq_PickHere` 多一个 ring 参数。STM32 主机测试 241 项。
  - arm_calib：ZDROP 后加 CLWOT(手臂在转盘 1 号格上方降到 ZDROP 调)、ZGRAB 后加 CLWOR(原料盘上方降到 ZGRAB 调)，needs_fw(旧固件跳过)；
    调夹爪时 `self.tune` 记正在调的值，c 夹住看看、o 回到它不会丢；其他步 o 按这一步在哪张开(`open_us`)；CLWOR/CLWOT 没改就不存(还是 0)。
  - 树莓派：`ArmLink.claw(True, where='RAW'|'TRAY')` 发 CLAW OR/OT，旧固件回 ERR ARG 就退回 CLAW O(说一次)；`_put_back` 用 TRAY，vclaw RAW 用 RAW。
- **任务四 停车点(提交 c766eb4)：**
  - run_live 新命令 `stop`(看四个停车点能不能停)/`stop 名字 X Y [车头] [force]`：`route_plan.check_stop` 按真实车身(含雷达)查黄色区/工位/原料转盘/场地边(硬余量)，
    扫描过再查障碍物(真实大小、最少 goal_obstacle_min_margin_mm)；不能停不改，列出最近能停的(直线最近、只沿车头、只横着)；工位/原料盘在车左边、比原来的停车点远 40mm 以上、
    沿那一排偏离工位中间 60mm(原料盘 100mm)以上只提醒。改了写配置(备份 .json.bak)，清掉旧路线/预先规划，扫描过马上重新规划。
  - `Planner.leg`：停车点本身压到区域/出场地，改停 `_fix_goal`(5mm 一格最近、goal_fix_max_mm 200 内)，`_reshift` 把 goal_shift/stop_pose 按规定的点算、note 写清楚。
  - `plan_mission(cfg 带 goal_near_fallback=True)`：所有车道封死档都失败后，最后一档(不封车道)开不过去的停车点用 `_near_leg` 在附近找(沿车头 ±300、横着 ±75，25mm 一格，
    横着按两倍算距离，最多 8 秒)。map_merge_live 的 attempt() 按最小障碍物余量也失败才用它。auto_run 到站时照旧把 goal_shift 交给 `hooks.stop_offset`。
  - mission_hooks：`stop_offset` 记下，`task()` 取出成 `_cur_shift`；`_stop_pose` 按真正停的位置算；`_ring2_smooth` 横移上限 +|往左|、往前找 +max(0,-往前)；`_raw_near_box` 范围放宽。
    模拟：横着挪 100mm 不告诉 → 只挪 70mm、2 个掉空 2 次碰倒；告诉了 → 三个都准。
- **测试：**单元 474 个(3 跳过) + 模拟 124 个全过 + STM32 主机 241/222/41 项全过。更新包 tasks_1020c.zip(mission_hooks / mission_cli / arm_calib / arm_link / route_plan / map_merge_live / arm.c)。
- **要留意：**① 用户 RAW 设过 y=2210(压原料转盘)，最多 2155(按 raw_margin 30)；还没回答 RAW_CT.shift_mm 是多少。
  ② CLWOT 要比转盘格子窄得下去：用户调的时候注意张开不能碰格子边。③ 停车点挪得太远(开不过去停附近)时，原料区找圆心按地图只放宽了范围，真实能不能够着要看实车。
- **10-21 补充(提交见 git log)：**用户"伸缩的舵机速度加到最快、ID1 旋转慢一点"：固件加 ROTSPD(旋转舵机大动作转速，0 或 10~300)、EXTSPD(伸缩舵机，0 或 10~1500)，
  0 = 用 ASPD；哪个是伸缩按 AEXT。`Big_Spd(id)`；Servos_To 的 speed 传 0 = 各按 Big_Spd；Move_Over/Sweep/DROP 缩回/AD 大动作/AP 大变化/APN 默认都按它。
  A2SPD/R2SPD 是 0 时绕电池盒那几段伸缩也用 EXTSPD。STM32 主机测试 248 项。
- **10-21 2 号环改成"停着算好再挪"(用户："不要一开始就往前运动，要根据实际情况来计算位置然后修正")：**实车暂存区日志：边走边对 1.3 帧/秒，
  14mm → 27mm 越走越偏，整个区不放(J 横移那列可能是反的，或者 V 和 S 的横移方向不一致)。`zone_calc`(默认 True)：找到 2 号环以后 `calc()`：
  停着 ring_list 取中值 → J 算要挪多少 → `clamp_to`(横移 ±zone_s_max_mm、沿那一排 fc±zone_f_mm、地图) → chassis_move 一次挪过去 → 停 zone_calc_settle_s 再拍 →
  `match_ring2` 按所有环一起平移认出 2 号环(不靠 J 准不准；没跟着动的是车上的东西) → `fit_j`(两个方向都挪过：整个 J 直接解；只有一个方向：J0 / 横移反了 / 前进反了
  三种加转角+比例，挑转得最少的) → 再算再挪，最多 zone_calc_moves(4) 次；只差 zone_calc_arm_mm(2.5) 以内交给手臂补(小挪动车轮差 1mm 左右)；
  车挪了画面没动 = 打滑，交给手臂补。修正过的 J 存下(note['jsave'])。用 S/F 动过，`_zone_v_only=False`(挪回停车点用 S/F)。zone_calc=False = 以前的 V 边走边对。
  模拟：J 横移反了、转 16°、停偏 57mm、打滑都能对上；整场 3 组放置平均 0.94mm / 最大 1.92mm。
- **10-21 再补：**用户实车(算好再挪)对准了但"不丝滑、时间太长"：`zone_calc_smooth`(默认 True)算好的那一下用 `_vel_goto`(V 斜着一起走，最快 `zone_calc_speed` 20mm/s，
  快到了减速，按 V? 走到)，STM32 不认 V 才 S/F；只用 V 时挪回停车点也用 V。模拟 3 整场放置平均 0.62 / 最大 1.40mm。
  用户"从转盘里取了物料要先转一个角度再伸长，不然挡到电池盒"：固件 `LVDEG`(默认 20°，0~120)：Leave_Tray 里爪子夹着(claw_now≈CLWC)、ID1 在 A1D 附近、AEXT=2 时
  先只转 ID1 这么多度(伸缩不动)，再按原来的绕电池盒(R1T2/R1T1)或直接转过去。APZ、OBS、PLACE 离开转盘都走这里。STM32 主机测试 253 项。
- **10-21 车上转盘 <-> 圆环分段绕电池盒(用户："从车上原料盘到暂存区也要先转动一个角度再伸长再转动一个角度再缩回，让我自由调试"；"arm_calib 里 ID2 伸缩速度要和 run 统一")：**
  固件 `R2STG`(默认 1)：圆环那组(R1T1/R2T/R1T2/R2SPD)分段走 `Stages()`：离开车上转盘 只转到 R1T2 -> 只伸到 R2T -> 只转到 R1T1 -> 只缩到要的长度 -> 只转过去；
  去车上转盘反过来(R1T1 -> R2T -> R1T2 -> A2R -> A1D)。`A2STG`(默认 0)原料区那组也能分段。`LVDEG` 默认改 0(用户说"不是我的意思")。
  新指令 `TRAY`(按去车上转盘的路走到上方、不下降，要 ARMOK=1)。`AF` 大动作(>8°)按 Big_Spd(EXTSPD/ROTSPD/ASPD)，arm_calib 里伸缩和比赛时一样快。STM32 主机测试 260 项。
  arm_calib：圆环那组按出去的顺序调 R1T2(rtray0：TRAY 到车上转盘上方开始)-> R2T -> R1T1 -> R2SPD(演示缩到 A2P 再伸回 R2T)，做完缩到 A2P、转到 2 号环；
  这四步里输入 `t`：TRAY -> APZ 2 号环 -> TRAY 真走一遍(和比赛一样)，再按分段回到这一步开始的地方。ArmLink 加 TRAY 超时。
- **10-21 最后：**用户"ID1 整个转动过程要连续，不要伸缩一下就停一下"(位置已经调好)：R2STG 默认 0(ID1 一直转，转过 R1T2 伸到 R2T、转过 R1T1 缩到要的长度，Sweep)；
  分段只在 set R2STG 1 时用。车上已经烧了上一版(R2STG 默认 1)的话，run_live 里 set R2STG 0 就行(存进配置，每次启动发给 STM32)。arm_calib 说明改成连续的说法。
- **10-21 升降"下到最低了还往下动"(用户："你检查一下升降的过程这一块有 bug 吗")：**找到的问题：
  ① 升降(Emm ID5)只按发了多少脉冲、等多久当作到了，走完不核对。要去的高度机构到不了(实车开机编码器读出 115，ZHI 118 / STBY 120 比它高)就顶住丢步，
  程序还当作到了 118/120，差的几毫米留到下一次：下到 ZPLC 这种低处时实际要去最低点下面，一直往下顶。
  ② 开机认高度时"算出来比最高点高就减一整圈"按 g_lmax 算，开机时还是程序里写的 115(树莓派的 LFMAX 还没发过来)，停在 118 以上关电会认成低 40mm。
  修法(arm.c)：`Lift_Check(mm)` 在 Lift_Goto / Lift_Finish / Lift_Done_Now 走完后读编码器(还在走就每 80ms 再读，停稳为止，最多约 1 秒)，
  差 `LFCHK`(默认 1.5mm，0 = 不核对)以上：Emm_V5_Stop_Now(闭环驱动器别再顶着)、lift_mm 改成编码器的、回 `LIFTSLIP want=.. got=..`；
  差 15mm 以上只报(末尾 ` (too far, kept)`，编码器一圈 40mm 可能认错圈)。`LIFT_TOP_MM` 130：Lift_EncPos 挪整圈和 LFMAX 上限都按它。
  LIFT CAL 上下走那两步不核对(`lift_nochk`，旧标定可能不准)。LIFT ZERO 时标定过编码器就把编码器的 0 也改到这里存 Flash(`Lift_Rebase0`，回 `LIFTZERO ENC .. -> 0`；
  差不到 0.3mm 不改)，不然核对/开机又按旧的 0 改回去。LIFT HOME 没改(默认 lift_init='skip' 不用；以编码器为准)。
  树莓派：`arm_link.lift_slip_text()` 把 LIFTSLIP 翻成中文(要去的 ≥60 没到：ZHI/STBY/LFMAX 改到实际-2 以下；要去的 ≤30 停在更高：ZPLC/ZGRAB/ZDROP 改到实际+1 以上)，
  ArmLink.request 打印到日志，arm_calib 的 do() 也打印。STM32 主机测试 281 项(桩里加了 phys_top/phys_bot 顶住、late_mm 走得慢)；树莓派 603 个全过。
  **让用户做：**量升降最高能到多少(arm LIFT ENC? / 看 LIFTSLIP 的 got)，ZHI、STBY(重新 STBY SAVE)、LFMAX 都设到它以下；看到 LIFTSLIP 就按提示改参数。
- **10-21 晚(两个问题："放 1、2 号环的时候爪子莫名其妙松开"、"对准时不要用机械臂修正，就用车轮，修正距离放开")：**
  ① 2 号环对准只用车轮(mission_hooks `zone_wheels_only=True`)：`_ring2_smooth` 的 calc/tail 里"只差一点/打滑/到边上/次数用完交给手臂补"都去掉，
  车轮接着修(最多 `zone_wheels_moves` 8 次)，最后差不到 accept_mm 照样放，不然这个区不放；V 慢走打滑就改 S/F 再挪一次。知道停车点在地图上
  (`_stop_pose` 有)时范围放开到横移 `zone_wheels_s_mm` 150、沿那一排 `zone_wheels_f_mm` 150(都按 `_body_clear` 查地图)；没有停车点照旧 70/60。
  ② 夹爪：程序里运输途中没有任何张开指令(查遍了 STM32 和树莓派)。固件加 `claw_hold`：GRAB/PICK/TAKE 夹紧后到放下前，TAKE/GRAB/PICK 回 ERR HOLD，
  OBS … O、待机姿态回 CLAWKEEP 不张开；`Claw_Move(target, why)` 每次动回 `CLAWSET <us> <why>`；开机夹爪直接用待机姿态的脉宽(不先 2090)。
  树莓派 `arm_link.claw_line_text` 把 CLAWSET/CLAWKEEP/READY/LIFTBOOT(中途重启)打印成中文；`_recover` 见 ERR HOLD 就 disabled + maybe_holding。
  用户不要限速(说和速度无关)。STM32 主机测试 302 项、树莓派 606 个全过。
- **10-21 晚再补(用户确认"可以一起改")：**① 夹爪张开只两种：`Open_Us()` = CLWOT(0 用 CLWO)，Claw_O/Claw_OAt(TRAY) 都用它，只有 Claw_OAt(RAW) 用 CLWOR；
  arm_calib 的 open_us 除原料盘那几步外都用 CLWOT。② 对准 2 号环故意对偏：mission_cfg `ring2_off_mm`{F,S}(±`ring2_off_max_mm` 20)，
  `_ring2_off_px(J0)` 按车轮 J 换成像素加到 goalp(vview 的 live_goal 也跟着)；run_live `r2off 前后 左右` 改并存配置(map_merge_live)。
  ③ 升降：开机等驱动器 6 秒；NOENC 时 `lift_guess`，Lift_Check 第一次读到编码器 ±20mm 内就认准(LIFTSYNC，BOOT=SYNC)；新指令 `LIFT SYNC`；
  LIFT CAL 读不到编码器不再改高度。树莓派初始化见 BOOT=NOENC 先发 LIFT SYNC，读不到就 disabled(本轮不夹不放)。
  STM32 主机测试 308 项、树莓派 607 个全过。原料区"放下去了不夹"推测是升降高度差 17mm(BOOT=NOENC)，没有日志确认。
- **10-21 夜(用户实车 mtest RAW 1："一个都没夹上来"、"突然抽搐一下"、"对齐不够快不够准")：**日志里开机 BOOT=POS(升降高度是编码器认的)，
  三次 GRAB 都有 `CLAWSET 1150 C`(STM32 确实发了夹紧)。改：① keepalive / 等原料盘的 `_raw_ct_keepalive` 不再摆 ID1，`_lift_bob`：升降往上
  `keepalive_lift_mm` 4mm 再回来(靠近 LFMAX 就往下)；0 = 以前摆 ID1。② 最近那格 `_raw_ct_grab(near=True)` 下爪前 `_raw_grab_fix`：拍两帧
  material_errors，差 `raw_grab_fix_mm` 1.5 以上按 store RAW ch 的车轮 J 用 V(_vel_goto，不行 S/F)挪(手臂不动，最多 2 次、每次 ≤15mm，
  原料盘这次停下剩的时间不够就不挪)，夹完 `_raw_wheels_back` 挪回去；调用时 need 多留 `raw_grab_fix_s`。模拟加 `plate_jitter_deg`(原料盘每次停的
  角度差几度)：±6° 时新做法 9/9 夹到，关掉会夹空。树莓派 609 个全过。
  **没解决、要用户查：**如果下到底爪子根本没合上(日志里有 CLAWSET 1150 C)，是夹爪舵机没执行：CLWC 1150 可能比爪子合拢的位置还紧，空爪合上就一直顶着
  发烫、过热保护(会"松一下再夹紧"、"不夹")；或者夹爪舵机的线/电源。让用户空爪 arm CLAW C 听有没有嗡嗡顶着、摸烫不烫，arm_calib 第 3 步重调 CLWC。
- **10-21 夜再改(用户实车第二次 mtest RAW 1："对准的时间太长"、"对准了之后就应该立刻识别颜色然后夹取，为什么要等那么久"、"夹子一会儿夹取一会儿不夹取")：**
  日志：BLUE 补位车轮挪了 10.3mm，再看还差 9.3mm(认的位置没跟着车动)，夹空了，之后 BLUE 两次又转到这一格；YELLOW/RED 差 16mm 没补反而夹上了；
  BLUE 补完挪回去以后"车被带偏 4.1mm"又对一遍；按任务码顺序等颜色。改了(全在 mission_hooks.py)：
  ① `raw_near_any_order=True`：`_raw_near_any` 这一格上是这一批还没夹的哪个就马上夹(放进它自己的槽)；False 照旧走原来按顺序的循环。
  ② 对准原料盘中心时 `_raw_center_frame` 每帧顺便记物料绕圆心的角度(`_rc_note`)，`_rc_plate_stop` 看出原料盘在对准时停下的时刻，
     `_raw_ct_watch(known_stop=)` 对准完还停着就这一次停下马上夹(不再等它转一下再停)。
  ③ `_raw_grab_fix`：挪完等 `raw_grab_fix_settle_s` 0.3 秒、两帧中值再看；画面里物料跟着动不到 `raw_grab_fix_trust` 50% = 认的位置不对
     (被爪子挡住/和蓝色爪子连在一起)，车轮挪回去按原来的位置夹(来不及就这次不夹)；离对准的地方一共最多 `raw_grab_fix_max_mm` 25mm、最多 3 次；
     日志写"物料在爪子前/后、左/右多少 mm"(看爪子点是不是整体偏)。V 没到位不再补发 S/F(以前会多走一遍)。
  ④ 夹完不挪回去(`raw_grab_fix_back=False`)；等原料盘时停靠位画面位置、"车被带偏"按车轮补位挪过的量算(`fix_shift`，`_raw_near_redo(shift=)`)。
  ⑤ 没夹上：夹过的颜色又出现在原料盘上(最近那格要旁边没别的物料)就把抓取数减回去，`raw_near_regrab` 1 次等它转过来再夹；转一圈(3 次停)没来就不等；
     下完爪回来原料盘还停在这一次、它还在这一格：马上再夹(时间够的话)；都夹过以后再看 `raw_near_check_s` 3 秒(`_raw_near_leftover`)。
     真车画面多半只看得到最近那一格，最后一个没夹上又转走了的看不出来(要等一圈太费时间，没等)。
  仿真：any-order 平均 86 秒 vs 按顺序 108 秒(原料区)。没解决/要用户查的：夹爪到底有没有合上(日志每次都发了 1150 夹紧)、爪子点 vclaw RAW 是不是偏了。
  **同一晚再改(用户实车，用的还是上一版："对准一次之后就应该不动了啊后面为什么还是乱动"、"下爪了但是不夹取")：**`raw_grab_fix` 默认改成 False
  (对准以后车轮一下都不动，`_raw_ct_grab` 只记"物料离爪子点多少、在爪子前后左右")；最近那格示教以后改了 A1G/A2E：不再改用以前开着车找圆心的做法，
  `give_up` 叫重新 rteach RAW near。实车日志里 BLUE 一次认成离爪子点 122mm、一次 60mm(蓝色爪子被当成物料)；用户把 CLWOR 改成了 1700(张得更小)。
- **10-21 夜最后一轮(用户列的 6 件事："两个区域爪子都不夹取""把整个代码查一遍有没有 bug""所有东西备份到 GitHub""小车运动太慢""暂存区 2 号环
  放不下去""故意对偏"，1 个半小时后有定时任务再查一遍)：**
  ① 爪子：固件 GRAB/PICK(Seq_PickHere)以前下降(约 0.5 秒)的同时车上转盘在换槽(TTWAIT 800ms)，夹紧时转盘舵机还在转；用户看到"下降的时候
     不夹取，抬起来了再夹取"、以前"运输时爪子张开一下再紧"都像夹爪舵机供电被拉低。改成 `TT_WaitReady()` 以后才 `Claw_C()`(TAKE 本来就这样)，
     tests_stm32 加了检查(没改的固件会失败)。夹爪 PWM 本身没问题(PA2 只给 TIM2，USART2 在 PD5/PD6；1150 不被限幅；树莓派没有在夹着时发张开)。
     还要用户在车上查：`arm CLAW C`/`arm CLAW 1700` 来回几次反应快不快、`arm SV? 1` 看电压、舵机烫不烫、CLWC 是不是夹得太死(舵机一直顶着发热)。
  ④ 速度：`_vel_goto` 重写(实车串口来回 0.1~0.2 秒、1 转/分约 4mm/s：以前快到了来回摆，10mm 要 9 秒；现在按"新指令生效前还会走的那段"预判停车，
     最后一两毫米按 1 转/分 走算好的时间，停下再量，1~4 秒)；zone_calc_speed 20→30、raw_center_search_speed 20→30(raw_center_speed 留 15：
     边走边看太快了存的对应关系反了时来不及发现，test_center_bad_stored_wheel_j)。仿真加了 `vel_latency_s`、VelGotoTests。
  修到的 bug：`_ring2_smooth` 的 `fit_j` 以前连挪一两毫米的那几下也拿来修车轮和画面的对应关系(认圆环差一两个像素，转角算成多少都有可能，
     还会存下来)：改成只用挪了 `zone_fit_min_mm` 8mm 以上的。
  ⑤ `zone_map_free_mm` 60：离停车点 60mm 以内挪车不按地图查(test_short_move_not_clamped_by_map_check 复现了实车"限成 30mm、差 16mm 不放")；横着(往工位那边/往外)只有 `zone_map_free_s_mm` 10mm 以内不查(横着压进工位这一轮就结束了)。
  ⑥ 故意对偏 r2off：ROUGH/TEMP 方向都对(test_ring2_deliberate_offset_in_temp_and_direction)；对应关系 J 修过以后故意对偏的像素按新 J 重算；
     一步一步对准(`_p2_align`)和码垛认圆环(`_align_covered(ring_off=)`)也偏(认物料顶面对 PICK 点不偏，PICK 点是量过的)。
  ② 查 bug(固件 + 树莓派各查了一遍)改了的：
     - 固件升降：`Lift_Check` 以前在 Move_Over 15ms 一次的舵机循环里(Lift_Done_Now)停下来等编码器最多 1 秒，ID1 会转过头：改成置 `lift_chk_due`，
       手臂转完在 Lift_Finish 里补。差 0.5mm~LFCHK = 爪子/物料顶到原料盘、地面、转盘槽底(闭环驱动器一直往下顶，爪子被压住合不上，
       很像"下降的时候不夹取，抬起来了再夹取")：Stop_Now 不再顶、`LIFTPUSH want=.. got=..`(树莓派提示把 ZGRAB/ZPLC/ZDROP 调高 1~2mm)。
       编码器一圈 40mm 会认错圈：认出的高度不在上次核对以后走过的范围(`lift_span_lo/hi` ±2mm)里、或差 15mm 以上：不改高度，Lift_Goto/Lift_Finish
       回 `ERR LIFT`(GRAB/PICK/TAKE/DROP 不在半空夹/松爪子)。开机擦 Flash 后马上 Pos_Save；`LIFT SYNC` 没回零回 ERR NOZERO；`TT <微秒>` 后 tt_slot=0；
       ParseF / Arm_Param_Set 挡住 nan。main.c `Servo_Wait` 读到角度时见急停就 Servo_Hold(以前急停了还要等它转完)。STM32 主机测试 316/222/42 项。
     - 树莓派 GRAB 出错(`_grab_failed`)：按 STM32 回的最后一次夹爪动作(`ArmLink.claw_last`，CLAWSET 的 C/OT…)：还没夹紧 = 物料还在原料盘上，收臂当没夹到；
       夹紧了 = 放回它的转盘槽(张开以后 STM32 也不再当夹着，不然后面都 ERR HOLD)，当夹到了；已经松进转盘 = 当夹到了；ERR LIFT = 本轮不再夹放，叫 arm LIFT ZERO。
     - 原料区最近那格：没夹上减抓取数时转盘那一格也清掉；等 3 次停下都没转来就当已经夹到了(多半看错)；下爪前一看"离爪子点 122mm"但别的物料都没动
       = 认错了，照样下爪(`_raw_plate_still`)；都夹过以后再看原料盘只算停在停靠位上的(蓝色爪子不算)；`_rc_plate_stop` 两个以上颜色要都往同一边转才算转、
       只看到一个颜色要转得更多；`_raw_close_delay` = max(升降下降, TTWAIT) + 合上(和新固件一样)；等原料盘动升降出错不停；
       动升降(keepalive)那段时间里停下的按动之前算(保守)、这个时刻**不记进周期推算**(记进去会把周期算错：模拟里在同一次停下里又按"刚停下"去夹，
       下爪时原料盘已经转了)；回不到看原料盘的姿态这个原料区就不接着夹(收着的手臂直接转过去会刮电池盒，剩下的记没夹到)。
     - 2 号环：车挪了画面没动(打滑)时记的位移按画面实际改回来(不然挪回停车点会差出没走的那段)。
     - arm_link：屏幕文字里的 '!' 换成 '.'(STM32 把 '!' 当急停)；LIFTPUSH、新的 too far 翻成中文。arm_calib：CLWOR=0 按 CLWOT 算(和固件一样)。
     - 仿真跟着新固件：GRAB/PICK 等车上转盘转完才夹紧；`claw_hold`(GRAB/PICK/TAKE 回 ERR HOLD、OBS … O 回 CLAWKEEP)；GRAB/PICK 回 CLAWSET 行；
       `fail_cmd=('GRAB', n, 'closed')` = 夹紧以后才出错(test_grab_error_after_closing_puts_it_back)。
  ③ 备份：tag `backup-1021`；对话记录 docs/对话记录/2026-10-18_22_对话记录.md。车上才有的文件(lidar_map_live、ld14p_scan、pose_fix、merge_stations、
     reference_correction、stm32_link、home_fix 这几个模块和 map_config_*.json、place_adj.json、servo_cal.json、vision_cal.json、claw_mask.png、ring_teach.json)
     仓库里没有，要用户从车上发过来才能备份。更新包 fix_1021j.zip(arm.c、main.c、mission_hooks.py、arm_link.py、arm_calib.py)。
  测试：STM32 主机 316/222/42 项、树莓派 620 个(3 跳过)全过。转盘等完才夹紧以后(下爪到夹住多约 0.4 秒)：用户用的"最近那格"(不按顺序)
  模拟 30 组 90/90 全进转盘、没夹空，12 组总用时只多 3 秒；没示教时的旧做法(_raw_stop_pick：一回来看到原料盘停着、不知道停了多久就马上下爪)
  本来就有约三成的组会夹空一次(改之前 20 组 8 组、改之后 6 组)，3 个测试换了没碰上的种子(测试里写了原因)。
  `_raw_predict_stop_start` 还没量到完整周期时：离上次看到它停下(或上一个是在哪次停下里夹的，raw_cycle['grab_stop'])不到"停一次+转一次"，就按那次停下算。
- **10-22(用户实车 mtest TEMP 1 force，码 243+231："按照任务码不应该是先放2环再放3环再放1环吗……这个也要按顺序来")：**
  `_sweep` 改成 `_code_order`：粗加工区、暂存区(含码垛)一律按任务码顺序放；车上配置里 apply_mission_config.py 写进去的 `place_order: sweep`
  不再用(日志说一次)。`_pickback_sequence` 的 code 模式按 item.index 排(以前跟着放的顺序，放是 3→2→1 时取回也是 3→2→1)。
  `PlaceOrderTests`。树莓派 620 个全过。原料区"最近那格"还是不按顺序夹(raw_near_any_order=True)，已问用户要不要改回按顺序。
  同一天用户说"从车上转盘转到 3 号环、2 号环的途中爪子在到圆环正上方之前就松开了"：查过程序只在 LIFT ZPLC 以后发 CLAW O(日志也只有这一次张开)；
  找到的问题是连续绕电池盒(R2STG=0)时 ID1 转到 R1T2 才开始伸、ID1 不等，按用户参数(ROTSPD 180、R1T2 194.2→R1T1 163.7 只有 30°=0.17 秒，
  伸到 R2T 要 327°≥0.42 秒)经过电池盒时伸缩不到一半，2 号环还会在 R1T1 全速反向缩回。让用户先 `set R2STG 1` 试，等结果再决定改不改固件
  (建议：转到 R1T2 时没伸到位 ID1 就等它伸到位，伸缩不在半路反向)。
- **10-22 再改(用户"就是要全部按照这个任务码的顺序来放置和夹取"、"爪子的问题已经修好了不用管"、"只需要把这12次夹取和放置的顺序改对")：**
  `_grab_any_order()`：原料区一律按任务码顺序夹，配置里的 raw_any_order / raw_near_any_order 写着 true 也不用(日志说一次)，
  只有 mtest RAW n any(raw_any_once，只这一次)才不按顺序。最近那格一律走 `_raw_near_any(in_order=True)`：只等还没夹的里面最靠前的那个
  (没夹上的放回去又是最靠前的，先把它再夹上；连着 raw_ct_skip_stops 次停下都不是它就记没夹到、等下一个)，"没夹上再夹/都夹完再看一眼"照样有。
  `_raw_any`(跟随/停靠位/等停到爪子附近)按顺序只给下一个颜色当候选，等 raw_track_s 没夹到就记没夹到、接着下一个；`_raw_grabpos_loop` 同样。
  missed() 把 grab_total 也减回去(再夹到/没夹到时只记一次)。暂存区途中松爪用户说已经修好了，固件没改。
  测试：`PlaceOrderTests.test_whole_run_two_batches_all_in_task_code_order`(两批 6 次夹 + 粗加工放/取回 + 暂存放/码垛全按顺序)、
  `RawNearSlotTests.test_task_order_by_default_any_only_with_mtest_any`。树莓派 621 个(3 跳过)全过。
- **10-22 备份时车上应该是的版本(下一个接手的先对一下)：** 已发给用户的更新包和对应提交：
  fix_1021j = 1ad6130(arm.c、main.c 烧 STM32；mission_hooks.py、arm_link.py、arm_calib.py)；
  order_1022a = 71bf450(mission_hooks.py：按任务码顺序放/取回)；order_1022b = a06ca37(mission_hooks.py、mission_cli.py：夹取也按顺序)。
  所以车上固件 = 1ad6130 的 arm.c/main.c(用户烧没烧要问)，树莓派 = a06ca37 的 mission_hooks.py/mission_cli.py + 1ad6130 的 arm_link.py/arm_calib.py。
  用户 10-21 12:40 发的 STM32 参数(ZHI 115、ZGRAB 85、ZDROP 95、ZPLC 0、ZOBRNG 115、A1D 237.6、A1P 49.5、A2P -985.5、R1T2 194.2、R2T -702.3、
  R1T1 163.7、R2SPD/EXTSPD 1500、ROTSPD 180、S1COMP 5、CLWC 1150、CLWO 1650、CLWOR 1700、CLWOT 1450、TTWAIT 800、SPOW 30000)在对话记录里。
  还没做 vclaw RING(日志一直提示)；用户说用 r2off 手动对偏。车上才有的配置/标定文件仍然不在仓库里(要用户发过来)。
- **10-22 下午 用户发来电脑上的 STM32 工程(car_2027_-_zhengshiban7.zip，备份在 `car_backup/2026-10-22_STM32_全部源码/`，先读那里的 README)：**
  arm.c 和 1ad6130 一样，hex 里有 LIFTPUSH(新 arm.c 烧了)；**main.c 还是旧的**(少了急停时舵机马上停)；**栈还是 1KB**(startup .s 和 .ioc 都是 0x400，
  仓库 10-07 起 0x1000；htm 里最深调用 1360 字节 + 中断，超出的部分写进舵机库 malloc 用的堆)。已建议用户拷 1ad6130 的 main.c、仓库的
  startup_stm32f407xx.s 和 car_2027.ioc(和用户的只差栈那一行)重新编译烧录，包 stm32_1022c.zip，用户还没回。
  车上树莓派(car_backup 里的 map_config_start2_roi.json)：比赛程序和仓库 a06ca37 一样；apply_mission_config.py 是 10-12 以前的旧版(没有 _migrated)，
  OLD_DEFAULTS 里只有 raw_track_s 还是旧的 150(新默认 40，只影响没有 rteach RAW near 时的老做法)；qr_fallback_code 还在配置里但程序不用；
  place_order 还是 sweep(10-22 起不用)。车上才有的程序和配置/标定文件已经备份在 `car_backup/2026-10-22_树莓派*`。
- **10-23 用户齿条重装、arm_calib 全部重调(对话里贴了整段过程)：** 存的 A1D=236.6 A2R=-1212.9 TT1=2380 TT2=1600 TT3=750 ZDROP=86 CLWOT=1450
  A1G=54 A2E=-946.8 A1T1=54 A2T=-846.9 A1T2=185.5 ZGRAB=85 CLWOR=1800 ZOBRAW=116 A1P=53.7 A2P=-1023.5 R1=-43.4/+288.2 R3=+39/+597.7
  R1T2=237.9 R2T=-712 R1T1=177.4 R2SPD=1500 ZPLC=0 ZSTK=53 ZOBRNG=115 A1H=241.7；run_live 里 set S2MIN -1300、S2MAX -250(放得比实际还宽)。
  用户说 ID2 缩到最短 -1230.7、伸到最长 -530.3，但调 3 号环时伸到了 -425.7(没报"没到")，最长是多少要再问。
  用户这阶段的话："只改码垛的时候的动作，其他时候的动作我慢慢调"、"不要把其他的东西改乱了"。停车点(stop 命令)、速度(route_speed_rpm /
  strafe_speed_rpm 一行 python、set TCV / SCACC)、粗加工区取回流程只是解释和给命令，没改程序。
- **10-23 码垛不扫倒两层的垛(用户"先放好三环我再去放1环的时候容易把三环放好的东西给撞掉")：** 原因是几何：ID1 从车上转盘 236.6 往下转，
  3 号环 92.7、2 号环 53.7、1 号环 10.3，去 1 号环要扫过 3、2 号环，去 2 号环扫过 3 号环；以前过了 R1T1 就按目标环的长度一边转一边扫，
  升降最高 115，夹着的物料底比两层垛顶(53+60)只高一两毫米。按三个环的角度拟合(fit_arm_polar)：爪子离转轴 = 276.4 + 0.0857×ID2 毫米
  (齿条重装后每度 0.086mm，不是以前说的 0.114)，三个环离转轴 213/189/240mm。mission_hooks：`_stack_carry` + `stack_carry_plan`，只在码垛、
  路上要经过已经叠了两层的环时换走法：APZ 到第一个要躲的环前面 20°(过了 R1T1，伸缩改成躲它的长度)，每段只转或只伸缩，在两个环中间改长度，
  最后 _return_to_pose 转到目标。缩着转用 A2R(stack_carry_a2)；缩到 A2R 只离 2 号环中心 16mm，所以 2 号环从外侧绕(ID2 约 -440，离它 50mm)，
  3 号环缩着(67mm)。配置 stack_carry / stack_carry_clear_mm(50) / stack_carry_a2 / stack_carry_outer / stack_carry_pre_deg。
  仿真加了 `_sweep_check`(夹着物料转的路上离两层垛中心不到 40mm 算撞，真车伸缩比转快：按先伸缩再转查)，TAKE 改成转回转盘以后才算夹着。
  StackCarryTests：车上的角度算出的路线、整场以前扫到 1 次现在 0 次、2 号环太近从外侧绕、其他指令(粗加工区、暂存区第一批、前两次码垛)和以前一模一样。
  **没改、要留意：** 放完以后 `_drop` 不缩回(drop_retract=False)，下一个 TAKE 空爪按圆环那边的长度转回车上转盘，也会经过别的环(空爪比夹着物料时高一截)；
  外侧绕要伸到 -440 左右，用户说的最长 -530.3 如果是真的就够不着(会停在 -530，离 2 号环约 42mm)。更新包 stack_1023a.zip(只有 mission_hooks.py)。
- **10-23 下午：** 升降最高只到 113.9：让用户 set LFMAX/ZHI/ZOBRNG/ZOBRAW 113、待机姿态重新 STBY SAVE(STBY SAVE 存的是按下那一刻的 lift_mm)。
  升降往下顶触发过载：碰地时编码器读数一次比一次高(5.8→9.8→11.1→13.4→16.0，LIFT ZERO 时 ENC 24.98→0)，原因还没查清(用户说皮带没打滑)；
  给了只动升降/反复碰地两个小测试，还在等结果；提过"放到地上时最后一段慢降、碰地就停并自动校零"的固件改法，用户还没说要。
  注意：10-23 一度让用户 set LFCHK 0、LFACC 220、LFMRG 400，后来都改回 LFCHK 1.5、LFACC 0、LFMRG 200(LFCHK 关掉顶住了不停，更容易过载)；ZPLC 建议 6。
  r2off 每个方向上限 20 → 60(用户"这个20远远不够你放宽到60")：mission_hooks DEFAULTS ring2_off_max_mm、map_merge_live 的 r2off 检查。
  车上配置里没有 ring2_off_max_mm 这一项，给了用户一行 python 先写进配置(不用换程序就生效)。偏这么多多半是爪子点不准(还没做 vclaw RING)。
- **10-23 arm_calib 画 r2off 红十字(用户"arm_calib 里也出现故意对偏的那个偏心十字，我手动把绿色的和红色十字对齐，再按对齐的位置调 1、3 号环")：**
  vlive.py `r2off_px(cfg_path)`：和 mission_hooks._ring2_off_px 一样按 servo_cal.json 的 RING ch 把 r2off(F, S)换成像素；找圆环时在
  绿十字 + 这个偏移画红十字(标 r2off)，左上角偏差改成圆环离红十字多少(选离红十字最近的环)。arm_calib 开 vlive 时带 --config(和向导同一份配置)，
  看圆环那几步说一句红十字是什么；A1P、R1、R3 的说明加了"把圆环中心对到红十字上，用红十字对就别降低高度"。
  红十字是爪子相对摄像头的真实位置，手臂转到 1、3 号环也照样准(摄像头装在爪子上)，但要在 ZOBRNG 的高度看(降低了有视差)。
  车上的 J、r2off -15/30：红十字在绿十字 (-1, +34) 像素。更新包 calib_1023b.zip(arm_calib.py、vlive.py)。
- **10-23 屏幕只显示赛规要的字(用户"不需要显示那个什么启停区域1还是2了，按要求帮我显示好必要的字")：** 赛规(附件2-1/2-2，用户发的 PDF 和
  docs/rules 里的一样)：读到码后一直显示任务码(字高≥12mm)，回家后显示正确抓取数、正确放置数；一键启动要有明确标识。
  固件 arm.c：开机只清屏(不再显示选区页、不写 READY)；准备页(ZONE 1|2)只画大 START(不写 ZONE n、没有状态行和 BACK，右上角按了不算)；
  ZONE MSG 只记不画(选区页 ZONE ASK 留着给旧流程)。树莓派：`race 1` / `race 2` 在终端选区(不写用配置 start_zone；race_auto 也用它)，
  race_flow 发 ZONE n 不再发 ZONE ASK，ZONE? 回 0 区(STM32 重启)每 2 秒再发 ZONE n；mission_hooks `screen_show`(默认 code/code2/grab/place)
  之外的 _ui 都不写屏，_show_stats 只写做成的个数(GRAB 3、PLACE 5)，prepare 先 ZONE LOCK 清屏(不再写 ---)。
  用户说一键启动是"整车电闸"：已告诉用户按"15 秒不动本轮结束"树莓派开机来不及，建议电一直开着用屏上 START 或另接按键；用户说先别管线，先改屏幕。
  测试：STM32 主机 317/222/42，树莓派 634 个(3 跳过)只有 test_ringdet 那一项(环境)。更新包 screen_1023c.zip(arm.c 要烧录；auto_run.py、
  map_merge_live.py、mission_hooks.py)。


- **10-23 晚 按 docs/任务清单_1023.md 做完程序第 1~6 项(更新包 tasks_1023d.zip：arm.c 要烧录；arm_calib.py、arm_link.py、auto_run.py、
  map_merge_live.py、mission_hooks.py、vlive.py)：**
  1. arm_calib 看原料画面的几步输入 `g 左右 上下`(像素)挪绿十字(vision_cal.json 的 claw_px.RAW)，vlive 发现文件变了就重读。挪完要重新 `rteach RAW near 颜色号`。
  2. 固件参数 A2GB：原料盘上夹紧、直着抬 OVCLR 以后一边升到 ZHI 一边缩回到 A2GB(ID1 不转)，再去车上转盘；0 = 不缩。arm_calib 在 ZGRAB 后面加了 A2GB 一步。
  3. 原料区只靠摄像头：外圈半径和示教时差 ±15%、物料圆差 ±20% 的圆不认；认准了才放大挪车范围；V? 每 0.5 秒才读一次；
     找圆心加对准最多 raw_near_find_s=30 秒(用户说 20，仿真里画面里什么都看不到时 20 秒 6 次有 4 次找不到，改成 30，要告诉用户)。
  4. 到站不用雷达定位(reloc_stops 默认 ['PREHOME','START'])，QR 站点扫描只找障碍物；转弯前离东西近时的雷达定位保留(仿真里全关会压进禁区 29~63mm)。
     测"到站雷达修正"机制的测试(test_nav_drive、test_route_goal_obstacle)用 OLD_RELOC 照以前打开。
  5. `go step` 到站停下时终端 `F 数`、`S 数`、`R 数` 挪车(F/S 最多 300mm、R 最多 30°)，`save` 存成这一站新的停车点(下一次 go 才用)，
     后面的路线按挪过的量补。
  6. 新固件指令 `PRE RAW|RING`(Seq_Pre)：开往原料区/粗加工区/暂存区的最后一段直线(到停车点不再转弯)之前，auto_run._pre_arm 调
     hooks.pre_arrive：手臂升到 ZHI、照 OBS 的路线转向观察姿态，绕电池盒那几步照样看着 ID1 角度发(Sweep/Stages 里 pre_nowait)，
     指令发完就回 DONE，剩下的舵机自己转完；不降到观察高度。到了照常 OBS(已经在姿态上只 Hold)。这一站用不上手臂不发；
     分段调试 skip 这一站调 pre_cancel 收臂；旧固件 ERR CMD 就这一轮不再发；mission_cfg pre_arm=false 可关。底盘代码没改。
     急停时已经发出去的转动不会停(最多再转一秒左右)。
  测试：STM32 主机 338/222/42；树莓派 649 个(3 跳过)只有 test_ringdet 那一项(环境)。
  检查 hex 是不是这一版：`python3 -c "import binascii;d=b''.join(binascii.unhexlify(l[9:-2]) for l in open('/home/zstu/car_2027.hex').read().split() if l[7:9]=='00');print('新固件(有 A2GB、PRE)' if b'A2GB\x00' in d else '旧固件')"`
- **10-23 晚 屏幕和一键启动(任务清单第 16~18 项，更新包 tasks_1023e.zip 包含 1023d 的全部内容，arm.c 要烧录)：**
  - 屏(用户"字太小、屏原来自带的东西容易点到、抓取放置要中文、排版要整齐、一开始不要绿色大字")：用户自己用 USART HMI 做新工程替换屏里的出厂演示：
    横屏 480x320、page0 黑底不放控件；字库 0 = 字高 120(指定字符 0123456789+)，字库 1 = 字高 72 GB2312(指定字符 抓取放置0123456789)，
    program.s 里 baud 改成 9600(新版软件默认 115200)。固件 scr_box：t0/t7 整宽两行大字，t2/t3 最下一行左右两半中字，
    Screen_Text 取文字里最后一个数，前面加 GB2312 "抓取"(D7A5C8A1)/"放置"(B7C5D6C3)；t1/t4/t5/t6 不再画。
    树莓派 _show_stats 跑的时候不写(screen_stats_live=false)，_start 回到启停区才写 GRAB n / PLACE n。竖屏没做(放不下 8 个 120 像素的大字)。
    (10-27 晚改成读到码以后一直显示抓取数、放置数：screen_counts_live，见后面"10-27 晚 屏上抓取/放置数")
  - race：race_start 默认 'now' = ZONE LOCK 清屏、不出 START，切区直接开跑(和 zone n + go 一样，开跑后扫描、规划)；'screen' = 以前屏上按 START。
  - 一键启动 = 整车电闸(用户"一键启动是一个物理的上电开关""这个电就是整车的电闸")：固件 Boot_Keepalive(上电后、第一条机械臂指令之前每 6 秒升降上下 4mm，
    最多 90 秒)；树莓派 `bootrace 1|2|off` 存配置 boot_race/start_zone(start_zone 改 zone_snap 才写得进文件)，install_boot_race.sh 装
    ~/.config/autostart/car_boot_race.desktop -> boot_race.sh -> run_live.sh --boot；--boot 时 boot_race 没开就马上退出，开了就先关掉(只用一次)再 race。
    **风险：** 树莓派开机 + 程序启动要几十秒(没实测)，3 分钟里少了这一段；"15 秒不动"靠升降上下保活，裁判认不认要用户确认。
    更好的接法是树莓派单独供电一直开着、电闸只管 STM32 和电机(还没做：要等 STM32 上电的 READY 才开跑)。
- **10-23 晚 第 19 项(查整个项目)第一轮：**
  - 仿真扫了一遍：原料区最近那格 30 个随机种子都 3 个全夹到、没夹空(仿真里一次原料区平均 114 秒，主要在等转盘：按任务码顺序、只在最近那格夹；
    夹完回来"这次停下只剩不到 1.7 秒"就要等下一圈)；粗加工区/暂存区 8 个种子 x 3 种停车偏差(0、+30、-40mm)都 12 个放对、最大误差 1.4mm。
  - 真路线(route_plan.plan_mission)每段都是"转完弯再走一段约 900mm 直线到站"：PRE 在这段直线开始前发，能生效。
  - pyflakes：比赛用的模块没有未定义名字之类的错，只有没用到的变量。
  - 比赛前要和用户确认：mission_cfg full_course 现在是 True(用户 10-18 要求一直跑完)，比赛要 3 分钟内回家拿"按时返回、显示统计"6 分就改 false。
  - 上车验证清单(新功能都还没上车试)：①烧固件后 hex 检查显示"新固件"；②arm_calib --from A2GB 调夹起后缩回；③arm_calib 原料几步 g 左右 上下 挪绿十字，
    再 rteach RAW near；④go step 每站 F/S/R 挪车、save；⑤正常 go：日志里"手臂提前转向看原料盘的姿态"，到站 OBS 很快；
    ⑥屏：新 HMI 工程烧进屏，开机全黑，读码两行大字，回家"抓取n 放置n"；⑦bootrace：install_boot_race.sh、bootrace 2、关电、开电闸自己跑，量开机到车动几秒。
- **10-23 晚 第 19 项第二轮(部署包 tasks_1023g = 1023f 全部 + 下面两处)：**
  - 命题解析 PPT 原话"机器人 15 秒没有移动，本轮比赛结束"、"启动按钮必须明确标记'启动'且只能有一个，不能被遮挡"。
    电闸上电到树莓派程序接手要几十秒，这段只有 STM32 的上电保活在动：改成升降往上 15mm(原来 4mm)、每 5 秒一次(bb7cc5b，要烧固件)。
    上面不够 15mm 少走一点，不够 4mm 才往下 4mm。跑的过程中的保活(keepalive_lift_mm=4，mission_cfg 里可调)没改。
  - `_bail_out`(停车点里意外出错)清掉提前摆臂标记(d2b76b1)：爪子里可能有东西不收臂时，以前后面每一站都不再提前摆臂。
  - 任务命题文档第五版交给用户(第四版第 1 部分 290 字超了 200 字限制，作废)。字数按正文+表题+表格逐字数。
  - 配置、示教文件防断电写坏(74c7f97)：一键启动 = 整车电闸，bootrace / save / 示教完可能马上关电。配置(map_merge_live、arm_calib)、
    place_adj.json(mission_hooks.save_json、mission_cli)改成临时文件 + fsync + 改名；vision_cal.json、servo_cal.json 加 fsync。
    boot_race.sh 的日志同时存 logs/boot_月日_时分秒.log。部署包 tasks_1023h(= 1023g + 这些)。
  - **注意：车上的 apply_mission_config.py(10-22 备份)比仓库旧**(没有 _migrated 那套"每条升级只做一次"、qr_fallback_code 删除)。
    仓库版会按迁移表改 mission_cfg 里的值(比如 raw_track_s)，所以 1023h 没带它；要换先和用户确认。
- **10-23 夜 第 19 项第三轮：STM32 偶尔一次没回话不再丢分。** 全场仿真里让每条指令第 n 次出错一次(sim_glitch_fuzz.py，
  160 种 + 原料区最近那格 50 多种)，以前会：开头 GET/LIFT? 一次没回话整轮不夹不放；松手以后抬起来出错当成还夹着放回转盘
  (程序以为环是空的，后面砸上去)；取回夹紧以后出错整轮 ERR HOLD；原料区下爪出错这一批后面全没夹到；TAKE 出错这个不放了。
  修法见提交说明(DroppedError、_take、PICK 按夹爪最后动作处理、原料区没夹上给它再转一圈的时间、ArmLink.do 对绝对位置指令重发一次)。
  测试 SingleGlitchTests。**没做的：**底盘 F/S/R 的 DONE 丢了(超时)整条路线停(Abort)；实车记录里超时都是 STM32 没开，
  没见过跑着跑着丢一行，所以没改协议(要改就在固件里给运动指令编号、加 MV? 查最后一条做完没有)。
  - 同一条指令连着两次出错：都只少一个物料、不撞不掉；开头 GET/LIFT? 连着两次没回话以前整轮不夹不放，现在到下一站再初始化一次(8e75e84)。
  - 终端 `full on` / `full off`(582cdc5)：比赛前选一直跑完全程 / 3 分钟内回家，存进配置；`docs/比赛当天操作清单.md` 写了比赛当天每一步。
    部署包 tasks_1023j(= 1023i + 这两处)。**full 怎么选要用户按实测定**：3 分钟一到裁判就停，之后做的不算分。
  - 终端 `check`(66eb3cb，self_check.py)：比赛前自检，只发 PING / GET / LIFT? / PRE X(新固件回 ERR ARG，不动)，再看配置和示教文件、
    摄像头/雷达设备、开机自启动、SD 卡空间，每项 ✓/★。部署包 tasks_1024a(= 1023j + self_check.py + 比赛当天清单)。
- **10-24 黄色物料认不出来：** vlive 里按 2(黄) 再按 k，把物料放在绿十字下面现场标定颜色(matdet.calibrate_color，存 vision_cal.json 的
  color_ranges)，0 = 恢复配置里的范围。比赛场地灯光不一样，到了场地要重新标。部署包 tasks_1024b。
- **10-24 码垛躲避改成"先缩 -> 缩着转到位 -> 再伸"(用户"往里面缩去躲避然后到位置了再伸过去不要一会缩一会伸"，要在 arm_calib 里能调)：**
  10-23 的 stack_carry_plan 按几何走：3 号环缩着过、2 号环太近从外侧绕(伸到 -440)、两个环中间改长度，用户看着就是一会缩一会伸。
  现在默认 `stack_carry_mode='in'`(mission_hooks `_stack_in` + `stack_in_plan`)：路上要经过已经叠了两层的环时，APZ 从车上转盘出发
  (STM32 照旧按圆环那组绕电池盒：R1T2 伸到 R2T、转到 R1T1)停在往里缩的地方 `stack_carry_a1`(没调 = R1T1)，AP 只往里缩到
  `stack_carry_a2`(没调 = A2R)，AP 缩着只转到目标环的 ID1，最后 _return_to_pose 只伸出去；一路在 ZHI。往里缩的地方不在 R1T1 和第一个
  两层垛中间(调错了)就改在 R1T1 缩(没绕电池盒就在车上转盘和那个垛正中间)，日志 ★。路上出错要放回转盘：`_to_tray_detour(dodge=)`
  先只缩、缩着转回往里缩的地方，再照常绕电池盒回去。`stack_carry_mode='plan'` 还能换回 10-23 的走法。
  arm_calib 在 ZSTK 后面加两步 STKA2(往里缩到多短，在 2 号环上方调，屏上按 fit_arm_polar 算爪子离 3、2 号环中心多远)、
  STKA1(在哪里往里缩，绕过电池盒以后；R1T1 前面或者到了第一个环不存)；这两个值存配置的 mission_cfg(stack_carry_a2/stack_carry_a1)，
  不发给 STM32(STORE_KEYS)；`t` 按正在调的值走一遍码垛的路(TRAY -> APZ 到 STKA1 -> 缩 -> 转到最远的环 -> 伸 -> 原路缩着回来 -> TRAY)。
  check 自检多一行说码垛躲避用的什么值。不用烧固件。
  **要和用户说清楚：** 按 10-23 存的角度，2 号环离转轴 189mm，ID2 缩到最短(-1230)爪子中心也只离 2 号环中心 18mm 左右：
  2 号环的垛还是要靠升降够高从垛顶上面过(ZHI 120 时夹着的物料底比两层垛顶高 7mm 左右)；3 号环缩着能离 70mm 左右。
  出车上转盘先伸到 R2T 是绕电池盒(用户自己调的那组)，躲垛这一段只缩一次、伸一次。
  测试：StackCarryTests(新加 in 的路线、往里缩的地方、整场每步只动一个舵机且伸缩只换一次方向、调短了不扫到、路上出错放回转盘先缩)、
  test_arm_calib.StackDodgeTests、test_self_check。test_arm_calib.py 里 `if __name__` 挪到最后(以前 python3 test_arm_calib.py 跑不到后面几个类)。
- **10-24 原料区：夹起上升途中停一下被原料盘甩飞、认到以后下爪慢(用户原话"上升的过程中突然停顿一下……转盘开始转动就给物料甩飞出去了"、
  "识别到了之后下达抓取命令或者识别的时候能不能快速一点")：** 原因是固件 Move_Over(OVLAP=1)先 Lift_Goto(OVCLR) 走完一段，
  等 LFMRG(200ms) + 编码器核对，再 Lift_Start 升到 ZHI；设了 A2GB(夹起往回缩)的原料区夹取就走这里，停在离原料盘 20mm 的地方。
  改成 Lift_Start(ZHI) 一条指令升到顶，按 Lift_Rise_Ms(OVCLR)(LFRPM 匀速 + LFMRG/2)的时刻再开始转/缩(要烧固件；test_arm 352 项)。
  树莓派最近那格：已经在抓取的角度(最近那格就是看的姿态)不再发 AP；刚看着原料盘停下的那一帧(raw_seen_fresh_s 0.5 秒内)就按它查物料，
  不再另拍一帧；下爪前算时间加上 _raw_clear_s(CLWAIT - raw_claw_close_s + 升 OVCLR 的时间，raw_need_clear)，剩的时间不够抬离原料盘就等下一次停。
  其他提速只能用户自己权衡：LFRPM(现在 60，"实测 60 稳")、LFMRG 200、CLWAIT。仿真 12 个种子：每次夹少一条 AP，偶尔多等一次停(仿真不模拟甩飞)。
  另外 10-24：raw_keepalive_s 12 -> 0(用户"把每隔 15s 动的动作去掉")；raw_near_recheck 默认关(用户"夹取过程中车不要再对准")；
  用户重装齿条，ID2 限位让用户 set S2MIN -2000 / S2MAX 500 先放宽，调完再量两头设准；所有 ID2 姿态、rteach RAW near 要重调。
- **10-24 用户"把这个上下动的这个程序直接给我删了"：** 固件上电保活(Boot_Keepalive，上电后每 5 秒升降往上 15mm)整个删掉(要烧固件)。
  已经提醒过用户：电闸一键启动时树莓派开机那几十秒车完全不动，有"15 秒不动本轮结束"的风险，用户坚持删。test_arm 345 项。
- **10-24 圆环十字单独可调、去掉 r2off(用户"粗加工区域和暂存区域的十字中心做成可调节的，做个颜色区分……两个互不干扰的独立的十字，
  故意对偏的那个红色十字去掉")：** 原料的绿十字(claw_px.RAW)不动。vlive 看圆环时爪子点(claw_px.RING)画红十字(看物料时绿十字)，
  Vision.overlay(vview)同样按 kind 上色；arm_calib 看圆环的那几步(A1P/R1/R3/ZPLC/ZOBRNG)`g 左右 上下` 挪 claw_px.RING(vlive.claw_shift
  kind='RING')，看原料的那几步挪 RAW，两个分开存在 vision_cal.json。r2off 去掉：mission_hooks 没有 ring2_off_mm/_ring2_off_px/_offset_measure，
  2 号环直接对到 claw_px.RING；run_live 的 r2off 只说去掉了；vlive.r2off_px 删了。用户去掉前 r2off 已经是 0 0。
  先说明过：原料的 g 一直只改 RAW，圆环十字不准是因为从来没量过(没做 vclaw RING，用的是配置估计值)。测试 RingCrossTests、
  GreenCrossTests、sim test_ring_cross_sets_where_ring2_is_aligned。部署包 tasks_1024i(arm.c 和 1024h 一样)。
- **10-25 放物料时升降还往下顶、触发过载(用户"降到最低的时候设置 z=5 留出这么多冗余还是会往下面压触发过载保护，好好检查一下到底是哪里的代码出问题了")：**
  查到的代码问题：① 固件 Lift_Goto 只按时间等(距离/速度 + LFMRG)，等完才用编码器核对一次：物料一碰到地面驱动器就一直使劲往下顶，碰得越早顶得越久；
  ② Lift_Check 差 15mm 以上(too far)只回 ERR LIFT、不叫停：碰地的地方比 ZPLC 高 15mm 以上(10-23 漂到过 25mm)驱动器一直顶到堵转保护；
  ③ LFCHK=0 时整个不核对，顶住了也不叫停(10-23 让用户 set 过 LFCHK 0，第二次就过载)；
  ④ arm_calib 的 ZPLC/ZSTK 让人用手把物料塞进爪子量，比赛时物料是从车上转盘格子里按 ZDROP 夹的，物料在爪子里的位置不一样
  (ZDROP 那一步让物料底离格子 2~5mm，从格子里夹的就往下多出这么多，手塞得深更多)：量出的 ZPLC 偏低，比赛时每次都往地上顶。
  改法：固件 `Lift_Wait_Watch`(Lift_Goto 等的时候每 40ms 读编码器，在上一次读到的附近找，不会认错圈)：到了(差 <0.3mm)马上往下做(不再白等 LFMRG)；
  离目标 1mm 以上、连着两次几乎没动 → Stop_Now(碰到后约 0.1 秒就不顶了)，lift_stall/lift_stall_z 交给 Lift_Check：LFCHK=0 也核对、
  在停下的高度附近找；差 15mm 以上回 `LIFTSLIP … (blocked)`、记真高度、ERR LIFT。到点了还在走(加减速比估的慢)接着等，最多 2 倍时间。
  出发前读一次编码器(先等 5ms，Emm 两条指令要隔开)，和 lift_mm 差 5mm 以上或开机高度是猜的(lift_guess)就不盯、照旧按时间等。
  Lift_Check 的 too far：不动了也 Stop_Now(还在走的不停，免得停在半路)。Lift_Finish(Move_Over 里边转边升降)没改，最后一段 Lift_Goto 是盯着的。
  arm_calib ZPLC/ZSTK 加 `take`(要 ARMOK=1)：TRAY → TT1、CLWOT → ZDROP → CLWC → ZHI → APZ 回 2 号环上方，再往下降量。
  arm_link.lift_slip_text：往下碰到底改成"在 xx mm 就碰到底了…改到 xx 以上；放地上的 ZPLC 用 arm_calib --from ZPLC 输入 take 重新量"，加 (blocked)。
  测试桩：假升降按转速随时间走(mv_goal/mv_v，Stop_Now 停在原地，相对运动接着上一次的目标)；test_arm 353 项；TakeLikeRaceTests。部署包 tasks_1025a(要烧固件)。
  **让用户做：** 烧固件；`get` 看 LFCHK 不是 0、LFRPM(10-22 车上配置是 150、LFACC 0)；arm_calib --from ZPLC 用 take 重新量 ZPLC、ZSTK。
  碰地高度越来越高(5.8→25)的机械原因还是没查清(LFRPM 150 + LFACC 0 硬起硬停可能让带轮/皮带一点点滑)：新固件碰到就停，顶的力和时间都小很多。
- **10-25 主程序里挪十字(用户"可不可以直接在主程序改")：** run_live 加 `g`(mission_cli._cross_shift，map_merge_live 转给 handle_cli)：
  `g 13 1` = 红十字(claw_px.RING)往右 13、往下 1 像素，`g RAW 13 1` = 绿十字，`g` = 看两个十字在哪；存进 vision_cal.json，
  同时改 h.vision.cfg(不用重开)。用户那次：车停的时候爪子本来对着 2 号环，日志 2 号环 (289,229)、红十字 (276,228)，车往后挪了 11mm，
  物料偏车尾 → `g 13 1`。这辆车画面左边是车头方向(环1 在左要前进)，物料偏车尾 = 红十字往右挪，每毫米约 1.24 像素(ZOBRNG 高度)。
  码垛第二层"不管先放哪个环，都在经过 3 号环之前缩到 STKA2、缩着转到目标环再伸出去"(现在只有路上有两层垛才这样)：已向用户复述，等用户确认
  ①放完空爪回转盘要不要也先缩 ②第一层/粗加工区不变，再改 mission_hooks._stack_carry(tall 为空也走 stack_in_plan，first 用 3 号环)。
- **10-25 停车点随便改(用户"我想随便改这四个停靠点的位置而不要报错")：** run_live 的 `stop 名字 X Y [车头]` 输入什么就存什么，
  不再"★ 不能停 / 没改 / force"：route_plan.check_stop 多返回 `moves`(跑的时候 Planner.leg 会不会挪走：压到区域/出场地 -> _fix_goal；
  碰到障碍物 -> _shift_goal)。只是比硬余量近(moves=False)：说"照样停在这里"，Planner.leg 本来就只把那个区域放宽到它的距离、准确停在输入的点；
  moves=True：说"跑的时候自动停到最近停得下的 (x,y)"。只有车头不是横平竖直、或者压得太深周围 200mm 都停不下才不改(force 照旧能硬存)。
  用户那次：QR (2000,1350) 会压到黄色区，(2000,1250) 离黄色区 21mm，最后存了 (2010,1250)。
  同时 map_merge_live 画图前过滤 `Glyph … missing from` 警告(地图标题"停车点改了：route 重新规划"的中文，DejaVu Sans 没有中文，每个字刷一行)，
  有中文字体(CJK/WenQuanYi/Droid Sans Fallback…)就让 matplotlib 用它。测试 test_nav_live test_stop_command_saves_what_was_typed、
  test_route_goal_obstacle test_check_stop_says_whether_the_race_moves_it。部署包 tasks_1025c。
- **10-25 一起改的(用户确认"对你改吧")：**
  ① 码垛第二层一律缩→转→伸(用户"不管是先放哪个环……在经过环3之前就缩回到指定位置然后到指定的环的位置然后再伸长出来")：
  mission_hooks DEFAULTS `stack_carry_always=True`；_stack_carry 'in' 不再要求路上有两层的垛；stack_in_plan(always=True) 往里缩的地方
  在路上第一个环(on_way 离车上转盘最近的，一般 3 号环；去的就是它就是目标环)前面，返回多了 first_ring。放完空爪回转盘、第一层、粗加工区不变；
  'plan' 模式照旧只有两层的垛才躲。日志"码垛(第二层一律这样走)"；放回转盘那句改成"还在码垛躲避的那一段"。
  ② 原料区"看原料盘的姿态和示教时不一样……这次不抓"(实车：重装齿条后 arm_calib 把 A2E 从 -1299.5 改成 -1190，最近那格示教作废，整批不抓)：
  `raw_pose_follow=True`：最近那格、ID1 差不超过 `raw_pose_a1_tol_deg`(3°)就照样做——D.home 换成现在的姿态、每个停靠位的抓取角度
  跟着挪同样多、comp 清零(摄像头装在爪子上，车轮按画面把原料盘中心对到示教的位置，爪子就在最近那格上方，伸缩差的长度车轮补)。
  ID1 差得多(画面转了)照旧不做。sim RawNearSlotTests.test_id2_renumbered_still_grabs。
  ③ 识别时间放长(用户"把摄像头的识别时间改长啊每一个地方都要做完")：raw_near_find_s 30→60、zone_find_s 40→60、zone_look_s 10→15
  (10-22 车上配置 mission_cfg 没写这几项，用程序默认；raw_track_s 车上配置是 150)。
  ④ 跑的时候 q 退出崩溃(terminate called without an active exception / Aborted)：以前只给 STM32 发急停，流程线程不知道要停(Ctx.aborted 看
  state['abort'])，6 秒没等到就收尾，Python 拆还在被读帧线程用着的摄像头。map_merge_live finally：跑着就 state['abort']=True、join 10 秒、
  hooks.close()(停读帧线程、关摄像头)、close_stream()；流程线程还活着(非 sim)就 atexit._run_exitfuncs() 后 os._exit(0)。
  ⑤ 屏幕白底黑字(用户"屏幕最好是白底字是黑的现在字也太小了顺序也排的不对")：arm.c 加参数 SCRFG(默认 0 黑)/SCRBG(默认 65535 白)，
  画字模式 cls 和 xstr 都按它们(要烧固件；以前的黑底白字 set SCRBG 0、set SCRFG 65535)。字小：字大小是屏工程里字库定的
  (字库 0 字高 120、字库 1 字高 72，见 10-23 那条)，程序改不了；顺序不对：还不知道用户要什么顺序，等用户发屏的照片。test_arm 354 项。
- **10-26 暂存区 2 号环对不准(用户发了三次 mtest TEMP 1 的日志，都是"2 号环对不准：这个区这次不放")：**
  原因：开机后第一次到圆环那里只认出一个"圆"(半径 259 像素，圆心在画面外 (245,-26))，_survey 量不出圆环这里每毫米多少像素(要看到两个以上环)，
  vision.scale('RING') 还是配置默认的 2.96；_ring2_smooth 拿它查 vcal RING 量的车轮和画面的对应关系(横移 1.12、前进 1.16 像素/mm)，判"不合理"，
  退回一步一步挪(servo.run + _wheels_kw，横移只许 ring_strafe_max_mm 20mm)，对不上就整个区不放。10-23 对得准是因为那几次 _survey 看到三个环，
  把 px_per_mm.RING 改成了 1.27(只在内存里)。按 2.96 算 _ring_rmax_range 是 49~281 像素，259 的大圆也被当成圆环(按 1.14 算是 19~108)。
  改：① `_ring_scale_seed`：没做 vclaw RING、这次运行还没看清三个环时，px_per_mm.RING 按 servo_cal.json 的 RING.px_mm_seen(_survey 看清三个环时存的)，
  没有就按 vcal RING 的 J(一次运行只定一次；vc['_ring_scale_src'] = seen/J/cfg/survey)。② `_ring2_smooth`：J 只是大小不对(两列还垂直)就 `_j_rescaled`
  方向照用、大小按 sc，不再退回一步一步挪；大小没核对过(src=J)或改过时，fit_j/jfix 能修的比例放宽到 0.3~3。③ `_jac_f(fresh=True)`(看清三个环、
  存的前进那一列和环距对不上时重测)：前进方向没变就保留实测的横移那一列方向，不再标 synth(标了以后 2 号环只能一步一步挪)。
  ④ `_ring_odd_size`：按大小认不出圆环时不限大小看一帧，说出那个圆多大、圆环该多大。测试 test_ring_scale.py、
  sim ZoneCalTests.test_stale_j_size_still_aligns_smoothly。用户那几次的真实情况还不清楚：认出的"圆环"半径 259 像素(10-23 是 55)，
  要么车没停在圆环前面(认的是别的圆)，要么摄像头离地近了很多(ZOBRNG/升降高度不对；10-24 开机读不到升降编码器 BOOT=NOENC)。新包的日志会说清楚。
- **10-26 原料区慢、物料在下面却不抓(用户"识别对准抓取何下爪子的时间都太长……有时候下一个已经在摄像头下面了但是还是一直不抓取错过了又要再等一轮")：**
  ① 不抓的原因：夹完一个回到看的姿态时原料盘多半已经停着(夹一个再回来 ≈ 原料盘转一圈 9 秒)，要推算这一次停了多久才知道够不够下爪；
  `_raw_ct_predict` 以前要亲眼看到两次以上停下才推，不够就不说话，一直等它转一下再停 → 错过这一次，要的颜色再转一圈才回来。
  改：只看到过一次停下也推下一次(离那一次 0.9~1.6 个周期内，按最早可能停下的时刻算：周期打 `raw_ct_period_tol`=0.1 折)；推不出来说一句
  "原料盘停着(这一格上是 X)，不知道这一次已经停了多久……"。② 等着的时候量出原料盘每次停多久、转多久(_raw_ct_watch 的 prev_stop：手臂、车、
  升降都没动过才量；在配置值的 0.6~1.6 倍 / 0.5~2 倍以内才记)，存进 raw_stops/raw_moves，_raw_ct_stop_len、`_raw_ct_period` 按量到的算。
  ③ 每夹一个日志多一行"(下爪到放进转盘 x 秒、回到看原料盘 y 秒；从它停下算 z 秒，原料盘转一圈约 w 秒)"：看时间花在哪
  (GRAB 里升降、夹爪的等待是固件参数 LFRPM/CLWAIT/CLSPO/TTWAIT，等用户发这行再说)。
  ④ 识别提速，认出来的结果完全一样：matdet.detect_many(一帧认几个颜色：HSV、自动找爪子、不能用的边缘只算一次)，Vision.material_errors/plate_scan 用它；
  RANSAC 一次算完(`_ransac_best`)；三个点一次抽完(`_triples_fast`：照 numpy Generator.choice 的 Floyd + Lemire 抽法，第一次用时和 rng.choice 比对，
  对不上就退回一组一组抽)；auto_claw_mask 的 31×31 闭运算只在爪子那一块做。这台机器三个颜色一帧 108ms → 39ms。
  ⑤ _raw_ct_pose：AP 回报的角度直接用，不再另读(每次回到看的姿态省两次串口来回)。测试 test_raw_timing.py、test_matdet.SameButFasterTests。
- **10-26 路线选最近的 + 走在车道中间、离黄色区多留余量(用户"路径规划的时候肯定要选择最近的路线啊不能随机选择啊"、
  "车在移动的老是容易压到这个黄色的区域……尽量让车走在中间一点，多留点冗余出来一定不要碰到黄色区域")：**
  原因：车上配置 yellow_margin_mm=50、clearance_penalty=5、clearance_pref_mm=150、strafe_cost_factor=3。中间 400 宽的车道(车宽含雷达 300)
  走在正中间两边也只有 45~55，每走一格都按"比希望的 50 近"加 yellow_soft_cost_s(0.5 秒/格)、净空倍数 4.5 倍，穿过去要多算 20 多秒，
  A* 宁可绕外圈(原料区→粗加工区 14.9 秒，直着往下 11.4 秒)；strafe_cost_factor=3 连每段横移的起停也乘 2，60~80mm 的短横移按 3 秒算。
  route_plan 改：① `route_center`(默认 True)：净空倍数和"比希望余量近"的代价都按横截面(走向的垂直方向前后 center_window_mm=300)里
  停得下的位置净空最大的那条算(`_slide`)，窄车道走在正中间不罚，偏了才罚(宽车道贴边也罚)；窄车道另外最多多算 narrow_lane_penalty=0.3 倍
  行驶时间(用时差不多才选宽的)。② 黄色区硬余量 `yellow_floor_mm`=35(配置 yellow_min_margin_mm 写 30 也按 35；35 是 400 宽车道按 25 栅格
  一定还有一条线开得过去的最大值)，默认 yellow_margin_mm 50、yellow_min_margin_mm 35。③ `strafe_seg_factor`(默认 1.25)：横移每段起停
  最多乘这么多(长横移照样每格乘 strafe_cost_factor)。④ `_direct`：停车点在车头正前/正后、一条直行按硬余量扫得过就只走这一条
  (PREHOME→START；不然 A* 为了躲启停区旁边贴边的软代价横移出去再回来)。⑤ `_search`：最近 4 个终点栅格点都要补 8~10mm 小移动时，
  再试 4 个"补的差值每个方向是 0 或 ≥20mm"的；还不行就留着最后一条小移动(auto_run flatten 到停车点慢速补)。test_route_safety 只禁中间的小移动。
  ⑥ note 里"只留了 xx mm"只在离黄色区不到 YELLOW_NOTE_MM=40 时写(居中走窄车道是正常的)。⑦ (前面就改的)lane_block='soft' 默认：
  有障碍物的车道不整段封死，穿过去多算 lane_avoid_s=2 秒。车上配置 12 个随机障碍物布局：平均行驶 110.6→103.2 秒，最差 134→104 秒，
  路线上离黄色区不到 40mm 的比例 8%→1%，规划 1.4→1.8 秒(这台电脑)。离黄色区最近的 30mm 是用户的 QR 停车点 (2010,1250) 本身
  (车角对黄色区角)，建议 stop QR 2015 1250。比较脚本见 scratchpad rp_cmp/cmp_center.py(旧版 rp_head.py)。
- **10-26 原料区最后一个夹完直接收臂(用户"最后一个夹取完了把爪子缩回来就完事了啊还把爪子转过去干嘛呢")：**
  `raw_near_check_last` 默认 False(True = 以前的这一批都夹过以后再转回原料盘上方看 3 秒，夹过的还在上面就再夹)；
  _raw_near_any 和 _raw_ct_body(非最近那格)最后一个夹到就 break，不再 _raw_ct_pose 转回看的姿态，_stow_quiet 收臂。
  10-27 用户又问了一次("夹取最后一个物料完成之后机械臂不应该再转到原料区域那边去了吧应该就直接回到待机状态走了吧")：
  查了全部原料区做法，固定抓取位(_raw_grabpos_loop，raw_center + raw_grab_fixed)夹完最后一个还 OBS RAW 一次，也改成直接收臂；
  _raw_any / _grab_item 本来就是每个开始前才 OBS，最后一个以后不转。sim：after_last_grab(最后一条 GRAB 以后第一条转手臂的指令)
  RawNearSlotTests.test_last_grab_stows_without_swinging_back、RawFollowTests.test_fixed_grab_spot_last_grab_stows 都是 STOW。
  收臂 STOW：存了待机姿态(STBY SAVE)就回到它，没存回 A1H/A2R。用户车上用的是最近那格(rteach RAW near)，1026b 起就是这样。
- **10-27 原料区最近那格认出就直接下爪(用户"黄色的认偏了，明明就是在正下方，只要第一个在爪子的正下方，后面的就不要想了，识别出来直接下抓")：**
  实车 YELLOW 每次停在最近那一格都认在离爪子点 37~42mm(GREEN 同一格 3.8mm)，raw_ct_check 超过 raw_slot_accept_mm(25)不下爪，
  一直等到时间到，后面的 RED 也没夹。`raw_near_trust_slot`(默认 True)：_raw_near_any 调 _raw_ct_grab(trust=True)，原料盘中心对准过、
  这一格(occupancy，raw_slot_match 以内)认出是要的颜色就直接下爪，离得远只记一行日志、也不按它挪车轮(raw_grab_fix)。
  用户明确说"不能放弃夹取任何一个物料"：不要加"夹不了就跳过"的做法(10-27 加过又撤了)。
  sim RawNearSlotTests.test_mis_seen_color_in_the_near_slot_is_still_grabbed(SimVision.color_bias_px 让 BLACK 认偏 40mm)。
  同一天 vview 在原料区等物料时也实时更新(vision.material_errors 写画面，标出每个颜色、离爪子点多少 mm)。
  暂存区离黄色区近：到站对准 2 号环时车轮把车挪到"A1P/A2P 下 2 号环在十字上"的远近，手臂调长了车就往黄色区退(_body_clear 只要 30mm，
  两块黄色区中间那条缝有 55mm 就挪进去了)；建议用户 stop TEMP 320 1200、stop ROUGH 1200 320，车摆在停车点重调 arm_calib --from A1P，程序没改。
- **10-27 工位 2 号环车轮被"离工位 30mm"拦住、整个区不放(用户"这是为什么没有对准就停了"；"不能拿机械臂补啊，对齐第二个圆环是写死的啊，
  轮子不是可以自由移动吗为什么要动机械臂")：**
  实车粗加工区：用户分段调试挪车后 save 的停车点 (1270,300) 车头 0°，车身离粗加工区只剩 20mm；2 号环要再往区那边横移 23mm，
  _ring2_smooth 的 clamp_to 按 _body_clear(导航的 qr_zone_margin_mm 30)只许挪到 zone_map_free_s_mm(10) 里的 -9mm，还差 12.6mm，
  只用车轮(zone_wheels_only)就整个区不放。**不许用手臂补 2 号环**(手臂停在 A1P/A2P 一下都不动；加过 zone_arm_rescue 又撤了，别再加)。
  改：`zone_align_margin_mm`(默认 5)：车轮对准 2 号环(clamp_to、s_range、creep)时 _body_clear(work=(区, 余量)) 离这个工位只留这么多，
  离黄色区、场地边、别的区照旧 30mm；规则是车身(机械臂除外)压进工位这一轮就结束，所以不能小于 0。clamp_to 被拦时前后照走、只二分少挪横移
  (以前两个方向一起按 10% 一档少挪)。日志写清楚被什么拦住；车轮到头还差超过 accept_mm 时提示 2 号环的手臂姿态是车贴着工位调的，
  车停在停车点 arm_calib --from A1P 重调(根本解决：2 号环正下方离工位 30~60mm)。这次的情况改完能横移 15mm、还差约 8mm(10 以内照样放)。
  主程序 `zmargin 毫米`(0~30，存进 mission_cfg.zone_align_margin_mm；只输入 zmargin 看现在多少)，test_nav_live.test_zmargin_command_saves。
- **10-27 转弯停留(用户"为什么每一次转弯的时候停留这么久……到了转弯的地方不应该是立马转过去吗")：**
  实车日志里转弯前雷达定位(_before_turn，离黄色区/工位/场地边近、推算误差可能比余量大时)每次都先"车不动：太久没动，升降往下 4mm
  再回来"：make_tick 按"离上次摆多久"算，车开完 3~5 秒的直线以后第一次 tick 就摆。改：tick.moved()(_Drive._send / _turn / 停车点做完调 `_moved`)，
  按"车多久没动"算，刚开完一段定位不摆(test_nav_drive.test_no_keepalive_twitch_right_after_driving)。
  转弯前定位本身不能去掉：test_nav_route 那两条路线、误差 1.2~2% 时关掉它车角扫进黄色区 47~90mm，开着最近 9.5~13mm；实车一次定位到
  横 35mm、前后约 67mm 的偏差。但实车 5 次里 4 次没对上(28%、28%、0%、点太少 61 个；要 45%)，没对上照推算转。主程序 `turnloc on/off`
  存 reloc_before_turn(默认 on，off 时提醒风险；test_nav_live.test_turnloc_command_saves)。转弯本身 R 90 实车 2.0 秒(speed fast 以后；以前 3.2~3.7)。
  sim ZoneCalTests.test_stop_close_to_zone_wheels_reach_ring2_without_arm(停车点 [1270,345,180] 雷达那边朝区，余量 30 就是实车那一次)。
- **10-27 1、3 号环摄像头微调一直调到对准为止(用户"把微调时间放长直到调好为止"；同一天用户说三个环都只偏一点、偏得各不一样，
  每个环用自己的十字 g / g R1 / g R3 挪)：** `zone_cal_fine_s` 2 → 10 秒、`zone_cal_fine_iter` 3 → 12 次(都只是防卡死的上限，
  偏差到 tol_mm.RING 以内就停；以前实车舵机慢，2 秒、3 次常常半路就停、按半路的角度放)。日志"微调好了"只在真对准(tol 以内)时说，
  到上限还差的说"到上限了(还差的在 accept_mm 以内，按现在的放)"(以前到时间、差 3.5mm 以内也说"好了")。
  sim ZoneCalTests.test_ring13_fine_tune_keeps_going_until_aligned(对应关系偏大一倍 + 每修一下多等 0.8 秒：以前 2 个都半路停、
  放偏 1.8/2.7mm，现在都对准、0.5/0.9mm)。同一天还提给用户、**用户说"先别管"没做的：**识别不到就一直认(不再到时间就停)、
  2 号环车轮挪到头还差的也照样放(不再整个区不放)；一直认的时候车不动，规则 15 秒不动本轮结束，要不要轻动升降没定。
  另外用户这两天 ID2 齿条跳齿：set 了 S2MIN -1195、S2MAX -500(是不是量的两头没回答)，A2E/A2GB(-1299.x)、待机姿态 A2(-1198.5)
  都比 S2MIN 小(转的时候被限位截住)；让用户在车上转盘那里量偏了多少再把所有 ID2 值一起挪(还没量完)。
- **10-27 下午 1、3 号环十字和 ID1 齿轮间隙(用户发了 4 次 mtest TEMP 1 force 的日志和照片)：**
  ① 用户把橙十字挪到 (284,356)、蓝十字到 (385,225)(当成绝对位置输了 g R1/g R3)，离圆环 100 多像素，超过 ring_gate_px(约 86)：
  1、3 号环"看不到目标"。让用户 g R1 5 -129、g R3 -96 2 挪回红十字 (289,227)。
  ② 接着微调越调越偏(14→20→33mm)：servo_cal.json 里手臂的对应关系是齿条重装以前量的，方向反了，让用户 vcal RING arm 重量(量完就收敛了)。
  ③ 十字放在红十字上，微调收敛到 0.9/1.4mm，可是放下去 1 号环(红)偏得多：1、3 号环的十字要各自标(arm_calib --from A1P 的 R1、R3 两步，
  爪子摆到环心正上方以后 g + off)，用户标了。
  ④ 标完 3 号环一开始只差 3.6mm，可是调满 10 秒手臂基本不动、放下去蓝色还偏：**ID1 齿轮间隙约 2°**(10-18 量到过)。小修正和刚转过来的方向
  相反，全被间隙吃掉(舵机读数转了 -1°，手臂没动)；而且微调时从 2 号环那边转过去，调完去车上转盘夹物料、再从转盘那边转回来，两次手臂
  停的地方差一个间隙(3 号环那里 2° ≈ 8mm)。改：`zone_backlash_deg`(默认 2)，`_id1_backlash`/`_ap_from_tray_side`：转到圆环上方的角度时
  ID1 最后一下总是从车上转盘那一边(A1D 那边)到位，要往转盘那边转就先多转一个间隙再转回来——1、3 号环微调开始时、微调的每一下
  (ArmActuators.id1_backlash/id1_from，只在 _cal_fine 里设)、对准 2 号环前回到 A1P(zone_obs_fine)、_return_to_pose(from_tray)的"再转一次"。
  sim：SimWorld(id1_backlash_deg) 舵机角度 a1 / 手臂实际 a1_act；ZoneCalTests.test_ring13_fine_tune_with_id1_backlash(以前 11.6mm，现在 1.1mm)。
  ZoneArmGeomTests.test_ring_out_of_reach_moves_the_car_and_realigns_ring2 种子 3 → 0：那条老退路(够不着就挪车)本来就约 1/6 种子会在
  "回到 2 号环重新对准"时认错环(zone_backlash_deg=0 也是种子 4、6)，用户的车不走这条路；以后有空可以查。
- **10-27 晚 1、3 号环的修正真的转得动(用户"红1蓝2红3还是这样有偏差尤其是1号环偏差最大"；我提了"夹着物料放之前再对一次"和
  "把修正改好"两条，用户"再对一次没有必要吧把修正改好就行了"——只做了后一条，别加"夹着物料再对一次")：**
  实车日志 1 号环微调 7.4 → 7.6 → 3.4 → 3.35 → 3.30mm(后面修了 3 次手臂一下不动，到 10 秒上限按差 3.3mm 放)。两个原因：
  ① STM32 的 AP(arm.c Servos_To，tol = ATOL 0.3°)：本来就离目标 ATOL 以内的只 Hold 不转；要转的(小动作 15°/s、8ms 查一次)
  离目标还差 ATOL 以内就 Hold，停在差 0.15~0.3° 的地方。零点几度的小修正等于没转(visual_servo 的 min_step 0.35° 也被它放大/归零)。
  ② 06b6022 的"先多转 zone_backlash_deg(2°)再转回来"不够：实测间隙 2.3°，多转的那一下本身也提前 0.3° 停，舵机走不完间隙、手臂不动，
  转回来那一下也碰不到手臂——往转盘那边的小修正全被吃掉；手臂还停在间隙的另一边，微调好的位置和夹着物料从转盘转回来放的位置
  差零点几度(1 号环离转轴最远，加上 return_tol_deg 0.4° ≈ 1.7mm，一共偏 3~6mm)。改：
  `zone_backlash_extra_deg`(默认 1)：`_id1_backlash` 返回的"先多转"= 间隙 + 它(一共 3°)；`zone_atol_comp`(None = 0.7×ATOL = 0.21°，
  `_id1_comp`)：ID1 最后一下往转的方向多给这么多(`_cal_fine` 开始那下、ArmActuators.id1_comp = arm_move 的每一下、`_ap_from_tray_side`)；
  `_cal_fine` 给 servo.run 传 min_step={'id1': ATOL − comp + 0.05}(visual_servo.run 新参数，只这一次)；`return_tol_fine_deg`(0.15)：
  `_return_to_pose(from_tray)` ID1 差 0.15° 以上就再转(以前 0.4°)，再转两次还差、但在 return_tol_deg 以内不打 ★。
  sim：SimWorld(ap_atol=True，默认开)：AP 照 Servos_To 停(`_ap_atol_stop`：差 ATOL 以内不转；4.5° 以内的小动作停在差 ATOL − U(0, 0.15) 的地方；
  大动作照旧)，ap_atol=False = 以前一下转到目标(补偿以后会转过头，别再用它测补偿)。ZoneCalTests.test_ring13_small_corrections_really_move
  (间隙 2.3°、和实车一样 1 号环离转盘角度最远(ring_tan 1 号 -150)：以前 1/2/3 号环最大 6.1/2.7/1.3mm，现在 1.4/0.5/1.5mm)、
  test_sim_ap_stops_within_atol_like_stm32；test_ring13_camera_fine_tune_fixes_a_bad_calibration 的"每个环 ≤ 2.6 秒"改成平均 ≤ 2 秒、
  最长 ≤ 4 秒(ap_atol 以后个别环在 tol 附近被画面噪声多拖一两次，不补偿也一样，最长 3.7 秒)。
  ap_atol 默认开以后有 5 个**旧做法**(车上现在不用)的测试超阈值，改前的代码(zone_atol_comp=0、zone_backlash_extra_deg=0)也一模一样超，
  不是这次改坏的：MissionSimTests.test_live_defaults_whatever_stops_under_the_claw_and_wheels_first(10-10 的每环对准，放置 2.98mm > 2.5)、
  ZoneFlowTests.test_next_ring_starts_at_the_reach_of_the_last_one / test_wheels_move_closer_when_the_arm_cannot_reach(一个环一个环对准)、
  RawFollowTests.test_taught_slots_predict_and_grab_on_the_stop(跟随 + 三个停靠位，只夹到 1 个)、
  RawCenterTeachTests.test_slot_far_beyond_the_range_does_not_move_the_arm(多发了一条 AP)。这 5 个照旧用理想舵机(ap_atol=False，有注释)；
  以后要用这些做法，先在 ap_atol=True 下查手臂小修正(它们的 id1_comp 是 0、min_step 0.35)。
- **10-27 晚 2 三个环剩下的"一点点偏差"(用户发 4 次 mtest TEMP 1 force 日志 + 照片："最后每一个环还是有一点点偏差有什么办法给他消除吗"，
  接着"直接改好吧")：** 日志里三个原因：
  ① **对准容差 3mm**：车上配置 tol_mm.RING = 3.0(10-22 备份里看到的；accept_mm.RING 8、return_tol_deg 1.5、return_bias false)，
  差 2.9mm 也"微调好了"、2 号环也算对准(4 次：2 号环 0.3~2.9mm、3 号环 0.3~2.9mm、1 号环 1.6~2.3mm)。
  ② **ID1 有摩擦**：往同一个方向修的零点几度一小下，舵机读数转了、画面一点没动，下一下才一起跳过去(1 号环 6.1 → 6.0 → 7.3(跳到另一边) → 2.3mm)。
  ③ STM32 一下转 4.5° 以上(main.c SERVO_FINE_DEG)不限速(AP 160°/s)、8ms 查一次，停的地方差 1° 多(日志"回到对准姿态差了 ID1 -1.00°")：
  "先多转再转回来"那一下要在 4.5° 以内才慢慢转、停得准，所以多转的不能直接加大。改：
  `zone_cal_fine_tol_mm`(1.0)、`zone_vel_tol_mm`(1.5，以前 None)：1、3 号环微调 / 2 号环车轮对到多少毫米，都和 tol_mm.RING 取小的(`_ring_tol`)；
  主程序 `rtol 1 1.5`(map_merge_live，存进配置，也改 mission_cli 已经建好的钩子；只输入 rtol 看现在多少)。
  `zone_same_approach`(True)：ID1 每次同一个动作到位——`arm_link.approach_id1(arm, pre, fin, a2)` 先到 pre(往车上转盘那边多转
  间隙 + `zone_backlash_extra_deg`，现在 2 + 3 = 5°，上限 8°)，没转过去就再转一次，再从 pre 分几下(每下 < SLOW_STEP_DEG 3.5°)转回 fin(多给了 comp)：
  ArmActuators.arm_move 的每一下(id1_always，不管往哪边修)、`_ap_from_tray_side`(always 默认按 zone_same_approach)、`_cal_fine` 开始、
  `_return_to_pose(from_tray)`(大动作先转到 a1 + side×多转，再 approach；这时不学 _ret_bias、不用 return_preload_deg)、zone_obs_fine 回 A1P
  (读数对上也照样做)、码垛 `_carry_rest`(最后转到目标环那一下，接着伸出去时 ID1 用最后一下发的角度，只动伸缩)。
  sim：SimWorld(id1_stick_deg：顶过间隙还要再多转这么多手臂才动；ap_fast_err：ID1 一下 4.5° 以上停在目标往前 -0.3~+1.2°，默认关)。
  ZoneCalTests.test_rings_left_with_small_errors_same_approach_and_tighter_tol(实车容差 + 间隙 2.3° + 摩擦 0.8° + 快速转停偏：
  以前最大 5.7mm 平均 2.3mm，现在最大 1.1mm 平均 0.7mm，微调平均 2 秒)、test_ring_tol_is_the_smaller_one、test_approach_id1_slow_steps_from_the_tray_side、
  test_nav_live.test_rtol_command_saves；test_ring13_camera_fine_tune_fixes_a_bad_calibration 时间改成平均 ≤ 3 秒、最长 ≤ 5 秒。
  扫过的间隙/摩擦组合(2.3/0~1.2、3.0/0.8、1.5/0.5、0/0)新做法最大都在 1.7mm 以内；只多转 4°(一下转回来)在间隙 + 摩擦 ≥ 3.5° 时还会偏 4mm，
  多转 5°/6° 但一下转回来(超过 4.5°)更差——所以是"多转 5°、分两下慢慢转回来"。代价：每次到位多 0.3~0.5 秒。
  有 3 个测试改成 zone_same_approach=False(有注释)：RawFrameThreadTests.test_ring2_keeps_moving_while_a_frame_is_slow(假手臂不许转，只测 V 续着)、
  ZoneFlowTests.test_return_from_the_tray_learns_its_bias(同一个动作时不学偏差、也不用再转一次，学偏差只在关掉时有用)、
  ZoneFlowTests.test_drives_straight_to_each_ring_in_code_order(一个环一个环挪车的旧做法，车上不用：同一个动作时 60 个种子底盘平均多修 0.3 下
  (0.87 -> 1.20)、种子 0 修了 5 下超过 ≤4，放置一样准；多半是模拟随机数用得不一样，没细查)。
- **10-27 晚 粗加工区取回不再花时间"对准"(用户"车在粗加工区域……放置完了还要夹取回车上，夹取回车上的时候不要再花时间对准了直接夹取就行了")：**
  车没动、只转手臂放的取回(direct)本来就不用摄像头，但 _goto_recorded → _return_to_pose(from_tray) 走的是 f97944d 的"同一个动作到位"
  (先往转盘那边多转约 5°、再分两下慢慢转回来)，回读差 0.15° 以上再转、最多两次——看着就是在对准，每个多 1~2 秒。
  改：`pickback_quick`(默认 True)：direct 的取回 `_quick_to_pose` 一条 APZ 转到放下时的角度(边转边降到 ZOBRNG)，回读一次，
  ID1 差 `pickback_tol_deg`(1°)以上才 _ap_from_tray_side 补一下，接着 PICK。挪过车放的(direct=False，老做法)照旧回到记录位置 + 摄像头对准
  (sim 里那条老路一律直接夹会夹空 4 个、碰倒 5 个，没改)。sim ZoneCalTests.test_pickback_goes_straight_to_the_placed_angles
  (6 个取回 86.2 秒，以前 90.4 秒，没有"再转一次")。同一天用户选了"两处都是"，我把原料区也改成第二次来不对准，
  用户纠正"我的意思是粗加工区……夹取回车上"，原料区那个撤回了、没提交，别再加。
- **10-27 晚 屏上抓取/放置数(用户"这个屏幕只显示数字抓取和放置的个数没有显示了")：**
  10-23 起抓取数、放置数只在回到启停区(_start)才写：没回到启停区(停掉、分段调试、出错)屏上就只有任务码。回家以后也可能只剩任务码：
  ①回家收臂(PARK，升降停 60)时屏要是掉电重启，固件只重画 STM32 自己读到的任务码；②树莓派同样的字不重发(arm_link._screen_last)。
  改：树莓派 `screen_counts_live`(默认 True，新键；旧键 screen_stats_live 不再读，apply_mission_config 写进车上配置的旧默认值 false 会挡住它)：
  读到任务码以后 _show_stats 写 GRAB 0 / PLACE 0，做成一个改一次(读到码以前不写，开跑前屏上照旧什么都没有)；
  _start 收臂以后 screen_reset，任务码、抓取数、放置数再写一遍，没写成(arm_link.screen 现在返回 True/False)歇 0.2 秒再写一次。
  固件 arm.c(要烧录)：画模式下每个框最后画的字记在 scr_last(Screen_Box 画、Screen_Text 记、Screen_Clear 清)，屏重新上电(88 FF FF FF)时
  Screen_Redraw 全部重画(任务码、抓取数、放置数，不管是谁写的)；t0 没画过、STM32 又读到了码才 QR_Draw。
  测试：test_arm 370 项(新加 5 项，旧固件有 3 项不过)；sim MissionSimTests.test_screen_counts_shown_while_running、
  test_screen_rewritten_after_screen_reboot_at_home(旧代码两项都不过)。每写一次屏约 50ms(9600 波特率)，一轮最多多写 18 次(约 1 秒)。
- **10-26 第二轮停在和第一轮同样的地方(用户"这个车在这四个停靠点停稳了但是后面第二轮运行……还能在同样的地方吗，这是一个大问题")：**
  10-23 起路上只在 PREHOME/START 用雷达定位(用户要求)，车轮打滑的误差一路累计，第二轮停的地方会差几厘米(手臂照样准：每次到工位都用
  摄像头对准 2 号环/原料盘中心)。改：`zone_anchor`(默认 True)。对准以后车和工位/原料盘的相对位置每次一样：`_anchor_mark`(在 _p2_align 对准
  2 号环、_raw_ct_body / _raw_center_setup 对准原料盘中心以后)记下车轮挪的 + 规划挪过的停车点(_cur_shift)；第一次的当基准(_anchor0)。
  以后再来，差多少这一次就停偏了多少；`_zone_back_target` 让 _zone_back / _raw_seek_end 挪回第一次停的地方(V 或 S/F)，
  路线下一段照样按停车点算，误差不再一轮轮累计。差 < zone_anchor_min_mm(8) 不挪，> zone_anchor_max_mm(60) 不信(多半认错)。
  一个环一个环挪车对准的旧做法(align_by wheels、zone_reach 不是 arm)不记，照旧挪回这一次停的地方。
  sim ZoneCalTests.test_second_round_goes_back_to_where_the_first_round_stopped、RawNearSlotTests.test_second_visit_goes_back_to_where_the_first_visit_stopped。
- **10-26 1、3 号环转过去以后用摄像头只动手臂微调再放(用户选的方案 A："到指定位置之后再通过摄像头微调机械臂的位置……车的位置不动，
  对准之间必须控制在两秒左右"；"这新加的两个也要是在主程序可调节的")：**
  mission_hooks `zone_cal_fine`(默认 True)：arm_calib 调过 1、3 号环的区(T['cal'])，第一次放 1/3 号环前空爪先转到调的角度上方
  (apz 到 ZOBRNG)，`_cal_fine` 用 servo.run('RING', _ring_measure('RING1'/'RING3'), fixed_j, allow_chassis=False, timeout_s=zone_cal_fine_s=2,
  max_iter 3) 只动手臂把这个环的中心对到它自己的十字上，再 _take 夹物料、回到对准的角度放。对上了 / 到时间但比开始准(err 比 history[0] 小 0.3mm 以上)
  就用现在的角度；一开始就差 > zone_cal_fine_max_mm(30) 不信。结果存 `_zone_learn[区][环]`(相对 p2)，这个区第二轮(含码垛)直接用、不再对。
  十字：vision claw_px.RING1 / RING3(没有就用 RING；`RING_CROSS`、`cross_color`：1 号橙、3 号蓝)，ring_px/ring_error 加 kind；
  vlive `--ringno 1/3`；arm_calib R1、R3 两步开 vlive --ringno、g 挪这个环自己的十字(_step_ring)；主程序 `g R1 dx dy` / `g R3 dx dy`(mission_cli._cross_shift)，
  只输入 g 看四个十字。sim ZoneCalTests.test_ring13_camera_fine_tune_fixes_a_bad_calibration(调差 1.5°：微调后最大 2.8mm、每个环最长 1.7 秒；
  不微调 10mm)、test_ring1_cross_is_where_ring1_goes；test_obs_error_is_removed_before_aligning 改成 zone_cal_fine=False 跑。
  mtest(手放车、hooks 一直用同一个)：RAW/ROUGH/TEMP 开始前清 _anchor0/_anchor_now(不按"第一次停的地方"挪)；_clear_zone_records
  清 _zone_learn[区](1、3 号环重新微调，g R1/g R3 挪过十字马上看得到)。比赛 go 每次新建 MissionHooks，本来就清。
- **10-26 整车提速 + 车开着摆臂(用户"这整个车运动和转向都太慢了帮我加速，还有就是每一个动作的衔接也要运动丝滑一点……在三分钟内尽量多抓取"、
  "尤其是转向的时候还有就是提前开这个机械臂的时候要丝滑运动车动车的不要停")：**
  - 固件 arm.c：PRE 在后台走(`PREBG`，默认 1)。Sweep 在 pre_bg 时 Bg_Begin 记下状态、只发 ID1 的转动就回 DONE；Bg_Step 做 Sweep 循环里
    每一圈的事(转过 T1/T2 发伸缩、到了 Hold、超时 StopIfStuck、急停两个都 Hold)；`Arm_Bg_Poll`(最多 15ms 看一次，急停马上处理)在 chassis.c
    Car_Turn_Closed / Car_Straight_Closed / Strafe_Run 每个周期 HAL_Delay 之后叫、Arm_Poll(主循环，车停着时)也叫；Arm_Command 开头
    `Arm_Bg_Finish` 走完再做。PREBG=0 回到以前(等伸缩都发了才回)。只在车上转盘上方绕电池盒那种 PRE 用后台(别的 PRE 本来就不等)。
    tests_stm32 全过(test_arm 365、test_chassis 225、test_servo 42)：后台伸缩晚不过 2°、车停着主循环接着走、急停马上停且之后不再转。
    急停的已知小缝：底盘最后一个周期(走完那一下)里来的 '!'，Link_Done 先清了标志，后台会接着转完(和以前 pre_nowait 一样)。
  - 转弯慢的原因：Car_Turn_Closed 最后几度按 TCKP×差的度数(转/分)靠上去，TCKP=3 是按 PPD 25.4 调的；车上 PPD=47，同样的转/分车身只转一半快。
    底盘仿真(test_chassis 的假车，K_ROT 换成 PPD 47；脚本 scratchpad turnsweep/tc_sweep.c)R 90 用 2.6~3.1 秒(最后 20° 爬 1.5 秒)，
    和实车日志 R +90 2.5 秒、R +60 3.2 秒一样。TCLAG(控制器认为的车身延迟，0.16)比真车大：提前刹车、慢慢爬；比真车小：冲过头再回。
  - `speed_profile.py` + 主程序 `speed fast / speed max / speed normal / speed`：fast = 直行 190、横移 160 转/分，SCACC 150、SCDEC 110、
    SSACC 180，TCV 120、TCA 600、TCD 300、TCMIN 2、TCKP = 0.128×PPD(47 → 6，限 3~8)，turn90_s 2.0 / turn180_s 2.8；max = 220/170、SCACC 200、
    SCDEC 150、SSACC 200、TCV 150、TCA 800。第一次换档把原来的配置值(没有的记 None)和 STM32 当时的值(GET)存进配置 `speed_saved`，
    normal 原样还原(test_speed_profile)。仿真：R 90 平均 2.6 → 2.0 秒，小角度修车头 1.0 → 0.7 秒，停下误差 ≤ 0.5°。
  - `turntune`：HOME 后 TCLAG 0.16/0.13/0.10/0.08 各转 +90/-90 两遍(共 16 次，车转回原来的车头)，选每次误差都 ≤ 0.6° 里平均最快的
    (差 0.05 秒以内选大的)，存 TCLAG，按实测平均用时 +0.1 更新 turn90_s(turn180_s = 1.4 倍)。出错/卡住/abort 保持原来的。
  - route_plan.Motion：直行刹车按 SCDEC 算(`_seg(d, v, a, b)`)：F +1117(150 转/分、SCACC 100、SCDEC 70)估 3.6 秒，实车 3.8 秒(以前估 3.5)。
  - auto_run._before_turn：定位以后转弯前要修的两份偏差里，转 90° 以后正好在下一条 F/S 方向上的那一份由下一条补上(Nav 每条按 ref 算长短)；
    只修另一份、按那里转也够安全(moves_clearance ≥ reloc_check_margin_mm)就不单独挪(`reloc_fold`，默认 True；`_fold_axis`)。
    test_nav_route 两个区：转弯前修正小动作 8 → 5 条，离禁区最近 9.5 / 14.3mm(要求 ≥ 5)。test_nav_drive.test_reloc_before_turn_folds_side_error_into_next_leg。
  - 整轮用时大头是手臂：sim_mission 用用户的手臂参数估操作 316 秒(两批全做)+ 行驶约 90 秒。底盘这几样估计每轮省 30~40 秒；
    再快要调手臂速度(LFRPM、ASPD、ROTSPD)。
- 用户 10-26 发的分段调试截图：ROUGH 2 号环偏 84.5mm，要横移 +72(往黄色区那边)、前进 +48 才对得上，_body_clear(qr_zone_margin_mm 30)只让挪 +7/+5，
  整个区不放。和 35mm 黄色区余量无关(那里用的是 30)。多半是 arm_calib 调 A1P 时车离粗加工区比停车点 (1200,340) 远了约 7cm(或停车点改过)，
  要在比赛停车点上重调 A1P/R1/R3；等用户发前面"圆环在画面里/看到 3 个圆环/夹角"那几行确认。
- 已知：test_ringdet.SchoolTargetTests.test_empty_target_near_and_under_claw 在这台电脑上早就不过(0d408cf 也不过)：R=90 (337,260) 误差 0.602 像素，
  R=130 (337,283) 最近的结果是个半径 25 的小圆(离真圆心 15 像素)。是校赛靶(粗黑环 + 白心 + 数字)的合成图，和今天的改动无关，没改。
