# 接手说明：粗加工区 / 暂存区对准(接着 3bf4a39 做的)

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
