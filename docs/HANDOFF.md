# 接手说明（最新：2026-10-10 晚，全流程整合，还没开始改代码）

分支 `claude/tender-heisenberg-2t1awl`。**先读完这一节，再读 `docs/integration/spec.md`（分工和接口）和 `docs/integration/analysis_*.md`（6 份分析报告）。**
文末"附：上一位留下的交接"是更早的工位对准交接，仍然有效。

## 一、现在的状态

**代码：** 和 44b740d 一样（工位对准提速的 WIP，详见文末附录）。分析时跑过一遍树莓派全套测试，231 个全过；STM32 主机测试（`bash arm_patch/tests_stm32/run.sh`）arm 105、chassis 32、servo 38，全过。

**这次做了什么：**
- 没改代码。
- 做了全面分析，写成 6 份报告：导航执行、路线规划、STM32 底盘、屏幕/扫码/升降、整场任务、比赛规则。
- 定好了分 5 块并行实现的分工和接口，写在 spec.md。
- 刚开始实现，用户就叫停了。改了一半的东西没有保存。

**用户的车上有、仓库里没有的模块：** `lidar_map_live.py`、`ld14p_scan.py`、`pose_fix.py`、`merge_stations.py`、`reference_correction.py`、`stm32_link.py`、`home_fix.py`，还有地图配置。
- **不能**把它们放进更新包，也不能替换车上的版本。
- `docs/pi_snapshot_1006/` 里有 10 月 6 日的配置，以及 `lidar_map_live.py`、`home_fix.py` 的旧副本，用来看函数签名和几何。
- 仓库里的 `route_plan.py`、`auto_run.py`、`map_merge_live.py` 就是车上正在跑的版本，和上一个更新包 1010i 逐字节相同。
- 分析报告里提到的 `scratchpad/...` 脚本是上一位的临时文件，**接手的人看不到**，要用就照报告的描述重写。

## 二、用户这次的要求（10-10 原话整理）

1. **工位放置太慢。**
   - 用户原话："先第一次校准再第二次放太浪费时间"。
   - 调整的幅度要小、要慢。
   - 用户希望到了附近先慢速小幅动轮子去找，实在不行再动爪子。
   - 注意：分析结论是 wheels_first 在模拟里更慢、失败更多（见 analysis_mission.md 第 3 节）。默认先保持关闭，再加 `mtest … wheels` 让用户上车对比；回复用户时要解释原因。
2. **扫码读不到时，前后挪一点再读。**
   - 扫码器装在车的**右后方**，离车后边约 20 mm、离右边约 40 mm，也就是在车中心后 125 mm、右 90 mm。
   - 车停在 QR 点时，扫码器离码中心约 125 mm（偏 42°），而且码板位置每场随机 ±100 mm。
3. **车不能碰到原料盘、各工位和障碍物。** 压到黄色区域或工位区，本轮结束、0 分。转弯要留出余量。
4. **待机时升降自动停到 60 mm。** 开机自动认高度要求升降在 60 附近。
5. **屏幕选启停区。**
   - 屏上显示选择启停区 1 还是 2；
   - 选完把车放好，车自己对准；
   - 比赛开始后不能再碰电脑（不能再敲 `y`）。
6. **回启停区要非常准。** 雷达定位效果不错，但有时还是会跑出地图（0 分）。
7. **把所有任务连起来完整跑一次。** 启停区 1 从来没测过。
8. **底盘更平稳。** 少抖动、动作衔接更顺；出错要有补救办法；用户没想到的也要考虑到。

## 三、分析查出的重点（细节和行号在各报告里）

1. **【高，已复现】回家前定位中途失败后，会接着执行原路线剩下的指令，车冲出场地。**
   - 位置：`auto_run.py:181-186` 加 `map_merge_live.py:336-360`。
   - 见 analysis_nav-exec.md 第 0 节。
2. **【高】路线中途完全没有雷达修正。**
   - `relocalize` 导入了但从没调用。
   - 配置里有一组 v16 的键（`reloc_*`、`plan_footprint_mm`、`zone_margin_mm`、`cam_side_targets`），代码一个都不读。
   - 蒙特卡洛估计常规误差下整场压线概率约 0.69；每个停车点加每次转弯后都用雷达修正，降到约 0。
3. **【高，规则】`go` 以后还要在电脑上敲 `y`**（`auto_run.py:279`）。另外规划在 x86 上就要 13～15 s，加上第二次扫描，会超过"15 s 不动本轮结束"。
4. **【高】时间到了不会提前回家。** `drive` 里没有时间检查；计时从创建 MissionHooks 时就开始了。
5. **【高】扫码。**
   - 开跑前从不发 `QR CLR`，可能读到上一轮的旧码；
   - QR 停车点对不准码；
   - 读不到就整轮不夹放。
6. **【高】启停区 1 照现在的配置会冲出北边界。**
   - 车头改朝西（START2 绕场地中心转 +90°：`(x,y,h)->(2400-y, x, h+90)`）就和区 2 等价，第二站动作可以原样用。
   - START1 停车点的车头也要改成 180。
7. **【中】`flatten` 悄悄丢掉小于 20 mm 的移动**（`auto_run.py:42`），误差会累积。ROUGH y=340、TEMP x=340、第二站都不在 25 mm 栅格上。
8. **【中】规划余量。**
   - 暂存区、粗加工区、转盘、场地边都没有硬余量；
   - 黄区余量会降到 15 甚至 0；
   - `ring_strafe_max_mm`=40 时，工位里车离白区只剩 5～20 mm。
9. **【中】底盘抖动，来源已定位（chassis.c）。**
   - 转弯末段在 0 和 ±8 rpm 之间来回切；
   - 转弯刹车的减速度超过驱动器斜坡；
   - 横移前馈从 CA 切到 CD 时跳一下；
   - 横移航向环用硬死区、不滤波。
   - R 只接受整数度，ALTOL 1.5°。
10. **【中】STM32 完全不收屏发回来的数据**（USART2 没开接收）。触摸选区要改固件：`sendxy=1` 后解析 0x67 帧，不用改屏的工程。
11. **【中】升降只有正常跑完才停 60。** abort、出错、mtest、退出程序时都不停。建议在 STM32 加 `PARK` 指令；abort 后不要自动动。
12. **【低】其他。**
    - 第二批在暂存区找不到同色下层时会平放（0 分，还多算放置数）；
    - 屏上任务码缺"+"；
    - 噪声跳变会被当成发散，把 servo_cal.json 里的 J 删掉；
    - 车宽 260 加雷达 40 正好 300，没有余量；
    - 转盘 x 位置也随机 ±100（RAW 停车点固定）。

## 四、建议的做法

按 `docs/integration/spec.md` 分 5 块做。文件归属互不重叠，可以并行；接口按 spec 的"Interface contract"来。每块要做的事，上一位写的详细任务清单如下（spec 里的各块范围是它的简版）：

- **pi-nav**（auto_run.py、map_merge_live.py、新 reloc.py）
  - 修回家重放 bug，回家每一步都按真实车身限幅，保证不出场地；
  - 快速 ICP（reloc.py），每个停车点先重定位再修正；
  - flatten 把余数带到下一条，不再丢；
  - 时间不够时从当前点直接回家（预先算好回家路线）；
  - 启停区配置切换（`zone 1|2`）；
  - 屏幕一键启动流程：查 `ZONE?` → 选区 → 开跑前先扫第一次并预规划 → 按 START → 不再问 `y`；
  - 车长时间不动时调用 `hooks.keepalive()`；
  - 退出时停 60。
- **pi-plan**（route_plan.py）
  - 加硬余量（`zone_margin_mm`、`edge_margin_mm`），黄区余量最低到 15 为止；
  - 规划提速 3 倍以上；
  - 新增 `pose_clear`、`moves_clear`、`plan_leg`；
  - 每段都精确停到停车点；
  - 测试两个启停区。
- **pi-mission**（mission_hooks.py 等）
  - hooks 的接口：`prepare`、`start_clock`、`go_home_now`、`set_home_eta`、`keepalive`、`park`；
  - 扫码前后挪着找（避开障碍物，最后回到停车点）；
  - 时间管理；
  - 第二批找不到同色就跳过；
  - 屏幕显示；
  - 发散判断；
  - `mtest wheels/nofilt`；
  - 车身斜 3° 以上提醒并提前补偿；
  - `ring_strafe_max_mm` 改成 20；
  - apply_mission_config 加上新键。
- **stm32-chassis**（chassis.c、main.c）
  - 转弯末段平滑；
  - 横移前馈平滑，航向环照直行环的做法改；
  - 精确模式：速度 ≤ PSPD（默认 70）的移动加减速更柔、航向修正阈值降到 0.5°；
  - R 支持小数；
  - 补主机测试。
- **stm32-io**（arm.c、hwt101.c）
  - 屏幕触摸；
  - `ZONE?`、`ZONE ASK`、`ZONE 1|2`、`ZONE MSG`、`ZONE LOCK`；
  - `PARK`；
  - `LIFT?` 回复里加 `BOOT=`；
  - 二维码在中断里锁存；
  - 补主机测试。

做完：
1. 合并，全部测试跑通；
2. 再做一轮对抗审查；
3. 打更新包，带上 e6b5c72 和 44b740d 以后的全部改动：`树莓派/lidar_map_north/` 加 STM32 的 `Core/Src`、`Core/Inc` 文件；
4. 给用户写中文的传输、烧录、上车测试步骤。

## 五、用户的习惯（回复和交付）

**回复：** 用中文，操作步骤要写详细。

**树莓派：**
- 地址 `zstu@10.109.173.12`，程序在 `/home/zstu/lidar_map_north/`。
- 更新包放 E 盘，用 PowerShell 传：
  ```
  scp "E:\car111_update_XXXX.zip" zstu@10.109.173.12:/home/zstu/
  ```
- 然后在树莓派上（先在 run_live 里输入 `q` 退出）：
  ```
  cd ~
  unzip -o car111_update_XXXX.zip
  cp -r ~/car111_update_XXXX/树莓派/lidar_map_north/* ~/lidar_map_north/
  cd ~/lidar_map_north
  python3 apply_mission_config.py
  bash run_live.sh
  ```

**STM32：**
- Keil 工程在 `E:\CubeMXcode\car_2027 - zhengshiban`（路径不要改）。
- 烧录步骤：
  1. Keil 里 Rebuild（F7）；
  2. `scp "...\MDK-ARM\car_2027\car_2027.hex" zstu@10.109.173.12:/home/zstu/`；
  3. run_live 里先 `arm LIFT 60`，再 `q`；
  4. `openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "program /home/zstu/car_2027.hex verify reset exit"`。

**提交：**
- 先 fetch 再 merge，不 rebase、不 force，因为别的对话也往这个分支推。
- 提交信息结尾带上 attribution 行。
- 不写模型名。
- 没有要求就不开 PR。

---

# 附：上一位留下的交接：粗加工区 / 暂存区对准(接着 3bf4a39 做的)

分支 `claude/tender-heisenberg-2t1awl`。这次提交是**做到一半停下的**：
`bash arm_patch/run_tests.sh` 在最后几处改动之后**没有完整跑过**，接手先跑一遍。

## 一、已经做完的

### 1. 3bf4a39 的测试补齐(任务 1)
- `sim_mission.py`
  - `_frames_dt(n)`：一次测量拍 n 帧的用时(5 帧 = 0.35 秒，2 帧 = 0.17 秒)。
  - `pixel_error(..., n)`：噪声按帧数算(`noise_px` 是 5 帧的噪声，n 帧乘 √(5/n))。
  - `SimVision.ring_error / ring_list / pick_px` 都接受 `n`，按 n 算时间和噪声。
  - `SimWorld(tilt={'ROUGH': 5.0})`：车身和圆环那一排斜几度(手臂、底盘、摄像头一起转)，给任务 4 用，还没有测试用到。
  - `SimWorld(ap_bias=(0.9, 1.6))`：大幅 AP 总是多转一点(测 return_bias)。
  - 新测试：`test_two_frame_measurements_are_faster`、`test_next_ring_starts_at_the_reach_of_the_last_one`、`test_return_from_the_tray_learns_its_bias`。
- `test_vision.py`：圆环只在爪子附近一块里找(结果和整幅一样)、小块没找到退回整块、2 帧测量认到 1 帧就算。
- `test_visual_servo.py`：偏差明显在容差里不再复测。
- `mission_hooks.py`：`_at_obs`——刚看完三个环(survey)手臂还在观察姿态，第一个环不再 OBS 一遍(每个工位省一次 OBS)。

### 2. mtest 的圆环记录(任务 2)
- `mission_cli._clear_zone_records`：`mtest ROUGH`(两批都是)和 `mtest TEMP 1` 开始时清掉这个工位"圆环上有物料"的记录和放下时的位置(用户会用手把圆环拿空)，日志里说清楚清掉了什么。
- `mtest TEMP 2 force`：先清掉暂存区记录，再只记第一批各一个(以前会重复记)。
- `mission_hooks._ring_taken_why`：返回 `'record'`(按记录)/`'vision'`(摄像头看到)/`None`。
  按记录占着的环：**不开过去、不对准**，直接跳过，提示"按记录环N 上还放着 X(…不是摄像头看到的)"。
- 测试：`test_mtest_zone_batch1_starts_with_empty_rings`、`test_mtest_temp2_force_records_batch1_once`、`test_ring_taken_by_record_says_so`。

### 3. 码垛环号显示(任务 3)
- `mcode`：第二批显示"粗加工区去环X，暂存区码垛到环Y"(Y = 第一批同色物料的环)。
- `mtest TEMP 2` 开头：显示"码垛到环Y(叠在第一批的 X 上)"；没加 force 又没有第一批记录时提醒。
- 测试：`test_mcode_lists_slots`(改过)。

### 4. 测量滤波(新加，默认开)
- `visual_servo`：`run(..., filt=True)`。每动一下按 J 推算偏差该变成多少，和这次测到的按可信程度合起来(卡尔曼式)；
  同一位置连测两次时自动估计测量噪声(`meas_var`)；测到的和推算的差太多(`filter_gate`)就只信这次测的。
  不开滤波时和以前完全一样(原料区没开)。
- `mission_hooks`：`zone_filter=True`(粗加工区/暂存区对准用)。
- 模拟结果(10 组随机 × 两批，只看放圆环 tol 1.2mm)：真实误差平均 0.69 → 0.72mm(差不多)，修正次数 1.97 → 1.28，测量次数 3.6 → 2.7；
  噪声 1.5px 时放不成的从 38 次降到 3 次。`test_noisy_camera_still_places_within_ring2` 以前因为 2 帧噪声大偶尔失败，开滤波后通过。

### 5. 先动车轮(任务 5，用户 14:34 的要求，做成开关，默认关)
- `visual_servo`：`run(..., wheels_first=True)`，只在 'F'/'Fs'(工位里)有用：沿圆环那一排还差 `wheels_min_mm`(1.5)以上就让车轮前后挪，
  每次最多 `wheels_step_mm`(15)，速度 `wheels_rpm`(60)；剩下的和离圆环的远近交给手臂。车轮不会因为它横着挪(不压线)。
- `arm_link.ArmActuators.chassis_move(s, f, speed=None)`：可以指定这次的速度。
- `mission_hooks`：`wheels_first=False`、`wheels_step_mm=15`、`wheels_min_mm=1.5`、`wheels_rpm=60`。
- 模拟：比默认慢约 26 秒/两批(模拟里底盘每挪一次按 1 秒 + 0.35 秒等稳算；手臂一次约 0.45 秒)，准度差不多。
  真车上底盘小步挪要多久不知道，要上车对比才能定默认值。

## 二、剩下的(按顺序)

1. **跑 `bash arm_patch/run_tests.sh`**，修好出问题的。最后一次确认：sim_mission 38 个通过(滤波已开)；
   test_mission_cli 61 个是在接上滤波/先动车轮之前通过的；test_visual_servo 18 个在接上之前通过。
2. 补测试：
   - test_visual_servo：滤波在噪声大时修正次数少、误差不变大；J 错了(符号反)时 gate 能让它恢复；不开滤波行为不变。
   - sim_mission：`wheels_first=True` 全放对、工位里没有 S 指令、每次 F ≤ wheels_step_mm、用 wheels_rpm。
3. mtest 加一个选项方便上车对比：`mtest TEMP 1 force wheels` = 这次先动车轮(现在只能改配置)。
4. **任务 4：车身斜(2~6°)的提前补偿 + 3° 以上提醒**(还没做)。思路：
   - survey 时：`axis` = 三个环连线在画面里的方向；`jf` = 底盘前进 1mm 画面动多少(存着的 J['ch'] 第二列或 `_jac_f` 现测)；
     斜的角度 = jf 和 axis 的夹角(带正负)；`nrm` = axis 的法向；`perp0` = 圆环到爪子点在 nrm 上的距离(像素，三个环一样)。
     挪到环 k(前后挪 f_k)以后：`perp_k = perp0 + (jf·nrm)·f_k`。
   - 换成手臂：`du = -solve(J_arm, nrm*perp_k)`，取 ID2(相对 A2P 的度数)，必要时 ID1 也补。
   - `_predict_d2`：工位里还没对准过 → 用 survey 算的 `d2_0 + b·(F-F0)`(b = 每毫米 F 的 ID2 度数)；
     对准过一个 → 那一次的 d2 + b·(F-F那次)；两个以上隔 60mm → 现在的直线拟合。第一个环也用(就算 `_at_obs` 也发一次 AP)。
   - 日志里一直报角度；|角度| ≥ 3° 时 ★ 提醒"把车摆正"；survey 日志的"横向差"改成挪到每个环以后的预计值；
     `ring_gate_px` 和 ">40mm 要横移" 的提醒也按预计值的最大值算。
   - 注意 jf 的方向只准到 ±1~2°(20mm 探测 / Broyden)。可以用第一次换环(150~300mm)前后的画面再修一次。
   - 测试用 `SimWorld(tilt={'ROUGH': 5.0, 'TEMP': -4.0})`：到 1、3 号环第一次测到的偏差要小；≥3° 有提醒、<3° 没有。
5. 模拟里看到的另一个问题(没改)：噪声大时 1.6mm → 4.9mm 这种一次跳变会被当成"发散"，丢掉 J 重新探测，
   之后整个对准乱掉。发散判断可以改成看滤波后的值、或者要求连续两次变大。
6. **打更新包**(要带上 e6b5c72 原料区旧默认值自动升级 + 这次所有改动)：
   `car111_update_XXXX/树莓派/lidar_map_north/` + README(中文)。用户的传输步骤：
   ```
   scp "C:\Users\Lenovo\Downloads\<包名>.zip" zstu@10.109.173.12:/home/zstu/
   cd ~; unzip -o <包名>.zip
   cp -r ~/<目录>/树莓派/lidar_map_north/* ~/lidar_map_north/
   cd ~/lidar_map_north; python3 apply_mission_config.py; bash run_live.sh
   ```
   `apply_mission_config.py` 会自动补上新加的配置项(zone_filter、wheels_first、wheels_*)。
   上车测试建议：`mcode …` → `mtest TEMP 1 force`(看圆环记录清掉、对准次数) → `mtest TEMP 2 force`(看码垛环号) → `mtest ROUGH 1 force`。
