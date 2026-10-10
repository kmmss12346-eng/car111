# 路线执行与定位审查：map_merge_live.py / auto_run.py / home_fix.py（快照）

我只读了代码，没有改任何文件。所有行号都对应仓库里的副本。

## 运行了什么
- `python3 -m unittest test_auto_run test_route_margin`：12 个测试全部通过。
- 用真实配置 `docs/pi_snapshot_1006/map_config_start2_roi.json` 加 `sim_obstacles`，在暂存目录（scratchpad）里跑了 `route_plan.plan_mission`、`auto_run.flatten` 和 `home_cut`。
- 用快照里的 `home_fix.icp_align` 在合成点云上计时。
- 写了一个假的 ctx，验证下面第 1 条 bug。

**机器说明**：所有计时都是在 2.8 GHz Xeon 上单线程测的，不是在树莓派上。树莓派大概率更慢，以日志里的"规划用时"和"(x 秒)"为准。

## 0. 最重要的结论
1. **【高，已复现】回家前定位失败后会重放原路线，车冲出启停区和场地。**
   - `do_home_fix` 已经移动过车（第 1–3 轮修正）之后，还可能返回 False。三种情况：后面某一轮 ICP 对不上（`map_merge_live.py:336-337`），修了 3 轮还没进误差范围（`:341-342`），或者某条指令失败（`:360`）。
   - `drive` 收到 False 后，从 cut 点接着执行原路线剩下的指令（`auto_run.py:181-186`）。
   - 复现：approach 先发了 `F -790, S -95` 然后返回 False，drive 接着又发了 `F -764, S -100`。车在启停区南面多走约 760 mm，出了场地。
   - 这时到达 START 后的 `home_fix` 没有 init，`icp_align` 的 `max_shift=200` 会拒绝修正（`home_fix.py:90-93`），车就停在场外。
2. **【高】时间到了路线也不会提前回家。**
   - `mission_hooks` 超过 `time_limit_s=150` 后只是不再夹放（`mission_hooks.py:23,44`）。
   - 但 `drive` 里没有任何时间检查（`auto_run.py:173-238`），车照样开完第二批的 RAW→ROUGH→TEMP（光行驶就约 30–40 s，还不算每个停车点的停车和扫描），才回 START。
3. **【高，规则】go 以后还要人在电脑上输入 `y`。**
   - `run_mission` 无条件调用 `ctx.ask`（`auto_run.py:279`），`Ctx.ask` 会一直等终端输入（`map_merge_live.py:397-402`）。
   - 规则 2)(2)：比赛开始后不能碰笔记本电脑。
4. **【高，规则】停车超过 15 秒的风险。**
   - 第二次扫描（36 帧）之后，`poll` 在 GUI 线程里同步执行 `plan_route`（`map_merge_live.py:271-274`）。
   - 用真实配置测：区 2 规划 13.0 s，区 1 规划 15.4 s。原因是每一段都先按黄区余量 30 mm 做 A*，搜完整个状态空间失败后，再降到 15 mm（`route_plan.py:476-481`）。
   - 再加上等人输入 `y`，很可能超过 15 秒。
   - QR 站点扫描如果发现新障碍物，还要再规划一次（`:663-668`），之后才读二维码。
5. **【中】ICP 很慢。**
   - 1500×1500 个点、3 个初始角度、45 次迭代，每次约 2.4–3.1 s（`home_fix.py:40-43,58-75`）。
   - 回家前定位最多量 4 次。树莓派上每轮可能接近 10 s，影响 15 秒规则，也影响 3 分钟总时间。
6. **路线中途完全没有位置修正。**
   - 文件开头的说明就写了这一点（`map_merge_live.py:3`）。
   - `relocalize` 和 `apply_fix` 导入了但从来没调用（`:16`）。
   - 配置里有一组 v16 才用的键（`reloc_*`、`plan_footprint_mm`、`zone_margin_mm`、`reloc_split_mm`、`cam_side_targets`），v14 代码一个都不读。

## 1. 一次完整 go 的流程（区 2）

| # | 位置 | 做什么 | 雷达扫描 / 运动指令 | 用时（估计或实测） |
|---|---|---|---|---|
| 0 | `map_merge_live.py:551-561` | 检查 link、`auto`、`busy`，并要求先 reset（`history.views` 为空）；`abort=False`、`auto=True`；启动 `mission_thread(True)` | – | – |
| 1 | `:419-431` | `mission_cfg.enabled` 时创建 `MissionHooks`，150 s 计时从这里开始（`mission_hooks.py:173`）；先调 `mission_cli.release()` 放掉摄像头 | – | – |
| 2 | `auto_run.py:246-255` | `ping()` → `sync_params(stm32_params)`（19 条 SET）→ `home()`（STM32 执行 `Car_Motor_Enable` 和 `Car_Home`，记下当前车头方向 `yaw_target`，`main.c:724-730`） | – | 约 1 s |
| 3 | `:257` → `Ctx.scan`（`map_merge_live.py:389-391`）→ commands 队列 → `poll`（100 ms 定时器）→ `command('scan')`（`:623`）→ `launch` → 工作线程 `acquire` | 第 1 次扫描：`scan_frames=36`，雷达刚打开时先等 1.0 s（`:91`）；结果在 `poll`（`:671-684`）里处理：记录 `yaw_scan1=gyro.value`、`history.commit`、`redraw`、`save()`（JSON 加 PNG） | 扫描 36 帧 | 约 5–8 s，车不动 |
| 4 | `auto_run.py:260-266` | `link.second_move(rel)`，`rel={'left_mm':42,'forward_mm':83,'cw_deg':60}`；车头误差超过 10° 就停止（Abort）；然后 `sleep(0.8)` | S 42、F 83、R -60（由 Pi 逐条发，或 STM32 的 P2：`main.c:666-676`） | 约 3–4 s |
| 5 | `:268` → `command('second')`（`map_merge_live.py:597-617`） | 用 `moved_config` 推算第二站位置；配置里有 `second_pose_mm` 就用它（用户配置里只有 `_second_pose_mm_old`，所以不生效）；用陀螺仪 YAW100 算车头（`second_use_gyro=true`，`:607-612`）；第 2 次扫描；`poll` 里调用 `refine_second`（`:674`），然后 commit、`redraw`；视图数到 2 时调用 `plan_route()`（`:271-274`），再 `print_route` 和 `save` | 扫描 36 帧 | 扫描约 4–6 s，规划 13 s（实测）以上 |
| 6 | `plan_route`（`:178-227`） | 规划起点是第二站转轴点；车头转回 `yaw0=90`，所以 `turn_back` 约等于 +60（`:187-196`）；mission 里的 `START` 映射成 `START2`（`:204`）；障碍物余量依次试 (60,+80)→(40,40)→(20,0)（`:210-216`） | – | – |
| 7 | `auto_run.py:269-276` | `ctx.plan()` 返回 `(turn_back, legs, reason)`；`flatten`（`:35-46`）丢掉小于 20 mm 的 F/S（`:42`）；用 `Motion` 估算用时 | – | – |
| 8 | `:279` | **`ctx.ask` 等人输入 `y`** | – | 人工 |
| 9 | `drive`（`:148-239`） | 开始前先 `home_cut`（`:165-166`）；之后逐条 `link.move(cmd,val,speeds)`，F 用 150、S 用 140、R 用 None；每条等 DONE 后做 `check_move`（`:227-237`）；每条之前检查 `ctx.aborted()`（`:175`） | 55 条指令 | 运动模型估计 81.7 s |
| 9a | 每个停车点标记（`:187-223`） | 顺序如下：<br>① `hooks.adjust`（空函数，`mission_hooks.py:202-204`）<br>② 停车点在 `rescan_stops`（默认 QR）里就做 `ctx.station_scan`（`:191-208`）：36 帧，**车位直接用计划的停车点**（`map_merge_live.py:502`）；离已知障碍物超过 200 mm 的算新障碍物，有新的就 commit、从这里重新规划、`save`，然后把新路线接进 `seq`（`auto_run.py:199-205`）<br>③ START 并且没做过 approach 时调用 `ctx.home_fix`（`:209-212`）<br>④ 调用 **`hooks.task(stop)`**（`:213-214`）；没有 hooks 时停 `stop_wait=3 s`（`:215-221`） | QR 处扫描 36 帧 | QR：扫描约 4 s，有新障碍物再加规划 13 s 以上，读码最多 6 s |
| 9b | `hooks.task`（`mission_hooks.py:206-216`） | 第一次调用时才初始化：`_ensure` 里做机械臂初始化和收臂（STOW）；`reset_disp`；按角色分发：QR 读码；RAW 抓取（默认车轮不动）；ROUGH/TEMP 看圆环，底盘用 F 和最多 ±40 mm 的 S 对准，最后 `_return_to_stop` 按**指令位移**退回（`:875,989,1623-1633`）；START 收臂、升降停到 60 mm、显示统计（`:1656-1672`） | 工位里有底盘小动作 | 每个工位几十秒 |
| 9c | cut 点（`auto_run.py:178-186`） | 调用 `ctx.home_approach`，内部是 `do_home_fix(init=计划车位相对起点的位置, order, max_move)`。每一轮：等 0.4 s，扫 `home_fix_frames=12` 帧，ICP；第一次需要修正前先发 `link.home()`；然后依次 R(-θ)、S(-ly)、F(-fx)，大于 150 mm 用 150/140 转/分，否则 60 转/分；最多修 3 轮再确认一次；成功后跳到 START 标记 | 每轮 12 帧，最多 4 轮 | 每轮约 2 s 加 ICP（Xeon 上约 3 s） |

区 2 用模拟障碍物时，每段的运动用时（`Motion` 模型）：QR 3.7 s，RAW 11.1 s，ROUGH 9.1 s，TEMP 12.2 s，START2 12.0 s。
- cut 点在 `(2150,944,90)`。原来的 `F -1025` 被拆成 `-231` 和 `-794` 两条，`max_move=1044`。
- 所有段都只能按黄区余量 15 mm 规划出来，每段都带警告。

## 2. 缺失模块被怎么调用（从调用处推断）

**`lidar_map_live`**（`:11-12`）
- `Mapping(cfg)` 返回的对象有 `.c`、`.latched`（字典列表：id、x、y、radius、seen、count、kind）、`.points`、`.next_id`、`.ingest(frame,count)`；调用方另外设置了 `.raw_frames`。用到的地方：`:83,103,52-60,638,656`。
- `validate_config(c)` 返回 c，出错抛异常（`:123,297`）。
- `roi_class(x,y,c)` 返回 `'interior'`、`'work_zone'` 或 `'boundary_uncertain'`。
- `rotate(xy,deg)` 返回 N×2 数组，逆时针为正。
- `to_map(xy,c)` 返回地图坐标。
- `accumulated_detections(frames,c,report=None)` 返回 `([{x,y,count,frames}], N×2 点)`（`:52,575`）。
- `body_corners`、`body_half_bound`、`FIELD_MM`，以及 `YELLOW_RECTS`、`TEMP_ZONE`、`ROUGH_ZONE`、`START_RECTS`、`POINTS`（只用来画图；`POINTS['QR_SCAN']` 在 `:171,564`）。
- `make_path` 导入了但没用。

**`ld14p_scan`**
- `Stream(port,model)` 有 `.proc.poll()`、`.snapshot()`（返回 `(frame{'points':[(角度,距离mm,…)],'seq'}, 收到时刻, count)`）和 `.close()`（`:73,98,101`）。
- `cartesian(polar)` 返回雷达坐标系下的 N×2 点（`home_fix.py:19`）。

**`reference_correction`**：`description(c)` 返回字符串（`:301`），`metadata()` 返回字典（`:290`），`applicable` 没用到；`apply(world,c)` 在 `lidar_map_live.field_points` 里用。

**`merge_stations`**
- `History()` 有：
  - `.views`：每项含 `config`、`raw_frames`、`field_returns`；
  - `.objects`：每项含 id、x、y、radius、seen、count、anchor_view、ambiguous；
  - `.merge_radius_mm`、`.set_radius()`（`:570`）、`.commit(m, source, seq)`（`:663,677`）。
- `moved_config(cfg, left, fwd, cw)` 返回新配置（`:600,626`）。
- `second_scan_config(cfg, x, y, 'cw'|'ccw'|'yaw', deg)`（`:619`）。

**`pose_fix`**
- `calibrate_mount(m.c, 扫到的点, 真实点, 雷达地图坐标)` 返回 `{lidar_yaw_deg, lidar_forward_mm, lidar_left_mm, residual_mm}`（`:647`）。
- `refine_second(m, history.objects, accumulated_detections, roi_class, to_map, trust_yaw=bool)` 返回 `(fix, msg)`，并且**直接修改 `m.c` 里的车位**（`:674-675`）。
- `relocalize` 和 `apply_fix` 没被调用。

**`stm32_link`**：`Stm32Link(port)` 和 `FakeLink()` 都有 `.ok`、`.err`，下面的方法都返回 `(ok, reply)`，例外已注明：
- `ping()`；`sync_params(dict, log)` 返回失败的参数名列表；`home()`；
- `second_move(rel|None, log)` 返回 `DONE t=毫秒 e=0.01度`；
- `move(cmd, val, speed|None)`；`abort()`（发 `!`）；
- `get_params()` 返回 `(dict|None, reply)`；`set_param(name, v)`；`cal(dist, spd)` 返回 `(dict|None, reply)`；
- `request(text, timeout, collect=True)` 返回 `(ok, reply, info行)`（`arm_link.py:57`、`mission_hooks.py:1083`）；
- 陀螺仪接口 `.value`（YAW100，单位 0.01°）和 `.fresh(max_age)`（`:607,676`）；
- `parse_done(reply)` 返回 `{'t': 秒, 'err': 度}`（`auto_run.py:7`）。

**`home_fix`**（只有快照，延迟导入，`:313`）
- `body_points(frames, c, rmin=150, rmax=4500, voxel=20, max_n=1500)`。
- `icp_align(cur, ref, init=(θ, 前, 左))` 返回 `{ok, why, theta_deg, tx, ty, rms, inlier, n}`。门槛：inlier ≥ 45%、rms ≤ 15 mm、离 init 不超过 200 mm 和 8°。

**`sim_lidar.SimStream`**：只在 `--sim` 时用。

## 3. 误差怎么累积，现有修正有哪些

**误差来源**（数量级是估计）：
1. **第二站车位**：只有 `refine_second` 按共同障碍物修一次。它的误差会整体平移整条路线。
2. **`flatten` 悄悄丢掉小于 20 mm 的移动**（`auto_run.py:42`），规划却当它们已经执行了。例如开始时吸附到栅格的那一步（模拟里是 `F -8`），每轴最多约 12 mm。
3. **转弯转轴补偿**：STM32 的 `PVB` 和 `PVL` 不在 `stm32_params` 里，所以用默认值 103/0（`chassis.c:130-131`）。配置里的备注写实测是 93，或者 109/12。偏 13 mm 时，每转 90° 车中心约偏 19 mm。全程有 14 次以上转弯（区 1 是 20 次左右）。
4. **车头残差**：每条直行或横移结束时，偏差不超过 1.5°（`ALTOL`）就留给下一条指令去纠偏。
5. **距离比例误差**：FPPM/BPPM 等约 0.5–1%，最长一段 1725 mm 会差约 10–17 mm。
6. **横移前后漂移**：DRL/DRR。
7. **工位里底盘小动作之后，按指令位移退回停车点**（`mission_hooks.py:1623`），每个工位误差几毫米。

**现有修正**：
- STM32 按累计的目标航向保持车头（`chassis.c:792-813`）。
- 出发前的 `refine_second`。
- 终点的 `home_approach` 和 `home_fix`。
- 摄像头在工位里量到的停车误差（`learn`、`ring_f`）每到一个新停车点就清零（`mission_hooks.py:213-214`），**不会反馈给路线**。
- QR 站点扫描只用来找障碍物，不改车位（`map_merge_live.py:502`）。

**home approach 的局限**：
- 只在最后一个 R 之后才开始，离目标不超过 `home_approach_mm=800`（`auto_run.py:102-131`）。
  - 区 2 是在 800 mm 处开始。README 说模拟只验证过 200–500 mm（`README_v14.md:22`）。
  - 区 1 在离目标 168 mm 处才开始，前面那段 F 950 完全靠航位推算。
- 只用 `views[0]` 做参考。点对点 ICP 遇到沿墙方向会退化，场外有人走动会被当成离群点。
- 推算误差超过 200 mm 或 8° 就直接拒绝。
- `max_move = 剩余平移 + 250`，而且按单轴比较（`:144`），等于没有门槛（区 2 是 1044）。
- 修正动作不做碰撞检查。
- 容差 6 mm / 1°，最多 3 轮，小动作不一定能收敛，收敛不了就触发第 1 条 bug。

## 4. 车为什么会停在启停区外、出场地、压黄区（证据）
1. **重放 bug**（第 0 节第 1 条）：直接把车送出场地。
2. **`home_fix` 没有 init 时**（`map_merge_live.py:363`）：
   - 偏差超过 200 mm 就拒绝修正，车停在场外。
   - 如果 ICP 在 200 mm 以内对错了位置（比如沿墙滑），会按错的结果走，而且没有 `max_move` 限制。
3. **规划余量太小**：真实配置加模拟障碍物时，**每一段都降到了黄区余量 15 mm**。
   - 车身外框是 300 mm 正方形（真车宽 260）加 40 mm 雷达突出，中间车道宽 400 mm，真车离黄区的余量大约只有 45–55 mm。
   - 全程约 14 m 都靠航位推算，横向偏 30–50 mm 并不奇怪。
4. **ROUGH/TEMP 停车点在 340 mm**：真车边到白区还有 60 mm，`ring_strafe_max_mm=40`（`mission_hooks.py:100`）用完只剩 20 mm，再减去停车误差。
5. **最后一段是朝场地角落开**：区 2 后退进角落，区 1 前进进角落。启停区 300 mm、车长 290 mm，长度方向只有 ±5 mm 的余量。只要稍微冲过头，就出启停区，同时出场地。
6. **QR 新障碍物登记错位**：车位用的是计划值，登记出来的障碍物位置是错的。离已知障碍物 200 mm 以上的会被当成新的（重复），可能把车道封死，结果"规划不出剩下的路线"，车停在 QR（`auto_run.py:197-198`）。

## 5. 改进方案

### 5a. 路线中途重定位（只用 map_merge_live 里已有的东西）
1. **参考点云**：
   - `refP0 = body_points(views[0].raw_frames, c0)`；
   - 再把 `views[1]` 的点按第二站相对 c0 的位姿变换过来，体素去重，最多留 2000 点。
   - 在第二次扫描之后算一次，缓存起来。
2. **`measure_pose(plan_pose)`**：
   - init 的换算和 `Ctx.home_approach` 一样（`:367-369`）；
   - `acquire(c0, frames=8)`，`cur = body_points(..., max_n=600)`，再 `icp_align(cur, ref, init=init)`；
   - 结果换回世界坐标：c0 位姿加上 (tx, ty, θ)；
   - **只测不动**，返回位姿。现在的 `do_home_fix(move=False)` 只返回 False，拿不到位姿。
3. **在哪里用**：
   - 每个停车点标记处，在 `adjust/task` 之前，这时车本来就停着；
   - **QR 处直接复用 station_scan 已经拍的 36 帧**，不用多扫。顺便用测出的车位来登记新障碍物。
   - 以后可以像 `home_cut` 那样，在长直行或转弯之后插入"重定位标记"。配置里的 v16 键 `reloc_min_travel_mm` 和 `reloc_split_mm` 就是这个意图。
4. **门槛**：
   - `res.ok`；
   - 偏差不超过 `reloc_max_shift_mm`（150）和 `reloc_max_yaw_deg`（5）。这两个键配置里已经有了。
   - 修正超过 50 mm 时，要求连续两次结果一致。
5. **怎么修**（不重新规划）：
   - 车头偏 ≥ 1° 时发 `R round(-eθ)`。STM32 的 R 是"目标航向 + delta"，同时也修正了陀螺仪基准。**不要调用 `link.home()`**，`do_home_fix` 里的 `:345-348` 不能直接复用。
   - 横向偏 ≥ 15 mm 时，用 60 转/分发 S；
   - 前后偏差并到下一条同方向的 F 里；
   - 修正动作先用 `route_plan.check_moves`（`:411-431`）检查，需要在 `plan_route` 里把 planner（`pl_`）存进 state。
6. **ICP 提速**（必须做）：
   - cur 降到 500–600 点：Xeon 上从 2.9 s 降到 0.36–0.66 s（实测）；
   - 有 init 时只试一个初始角度；
   - 或者给参考点云预先算一张 10 mm 格子的最近点表，每次查表 O(1)；
   - 或者如果树莓派上有 scipy，用 `cKDTree`。
   - 建议放在新的仓库模块里，不改 Pi 上的 `home_fix.py`。
7. **第二站也用 ICP**：第二次扫描对第一次扫描做 ICP，init = (-60°+陀螺仪, 83, 42)。两次几乎在同一位置，重叠非常高，可以拿来校验或替代 `refine_second`，不依赖障碍物。
8. **风险**：
   - 离起点 2 m 以上（RAW、TEMP）时，重叠率可能低于 45%，这种情况不修，是安全的；
   - 场地有没有围墙不确定，要在真实场地上测；
   - 二维码板和原料转盘的位置是规则允许随机摆放的，不能当作绝对路标。
   - 可选补充：按已知圆柱匹配，2 个圆柱做刚体拟合，1 个圆柱加陀螺仪车头只修平移。

### 5b. 最后回启停区：又准又安全
1. **修重放 bug**：
   - `do_home_fix` 返回三种状态：`ok`、`fail_unmoved`、`fail_moved`，同时带上最后一次成功测到的位姿；
   - 只有 `fail_unmoved` 才允许执行原路线剩下的指令；
   - `fail_moved` 时：没有 init 再测一次，或者按"最后一次成功的位姿减去之后发过的指令"补齐，否则原地停住。
2. **加 PREHOME 停车点**（只改配置）：
   - 区 2：`stops.PREHOME=[2250,400,90]`，mission 改成 `[..., "TEMP", "PREHOME", "START"]`；
   - 这样 `home_cut` 的 cut 点就在 PREHOME，离目标 250 mm，落在 README 验证过的范围内；`max_move=500`；最后一段只有一条直线 F；
   - `role_of('PREHOME')` 返回 None，hooks 会直接跳过，不受影响。
   - 区 1 对应 `[2250,2000,90]`。
3. **最后几步**：
   - 用 40–60 转/分；最多修 2 轮；
   - 每一条修正都先限幅：按 ICP 位姿推算，车身外框不能超出场地（留 3–5 mm），目标点朝场内偏 2–3 mm；
   - ICP 推算出车已经压到边，就只往里修；
   - 最后一次确认时，要求推算出的车身在启停区矩形里，否则不再动。
4. **定位一直失败时**：从 PREHOME 慢速执行"原距离减 15 mm"。宁可短一点，也不越过边界。
5. **时间保护**：把 `hooks.time_left()` 传给 approach，时间紧时只量 1 轮。
6. **提前回家**：在规划线程里预先算好"从每个停车点直接回 START"的路线。`time_left < 剩余预计` 时，`drive` 切换到回家路线。`replan_from` 的机制已经有了，但树莓派上现场规划太慢，所以要预先算。
7. 规则写的是"回到启停区"，没写车身必须完全在里面。只有 ±5 mm 的余量，所以最好的目标就是**手放的起点位置**，home_fix 对齐的参考本来就是这个位置。

### 5c. 支持启停区 1
抽签决定启停区，所以区 1 必须能用。

**配置**（建议加 `--zone` 参数或 `zone N` 命令，只在 history 为空时允许；在 `validate_config`（`:123`）之前或之后同步更新 `raw_cfg` 和 `state['config']`）：
- `start_zone=1`。START 到 START1 的映射已经支持（`:204`），`stops.START1=[2250,2250,90]` 也已经有了。
- `car_x_mm/car_y_mm/car_yaw_deg = 2250/2250/90`。**车头要保持朝北**：雷达在车左边，朝北时雷达面向场内；朝南时雷达对着东墙。
- `second_relative_moves = {left:42, forward:-83, cw:-60}`：
  - 现在的 forward +83 会让车头到 y=2478，出场地；
  - 转到车头 150°，雷达朝西南，正对场地最远的角；
  - 陀螺仪的合理性检查 `abs(d+cw)<=15` 正负号是对得上的（`:610`）。
- 去掉或按区分开 `second_pose_mm`。
- `second_use_p2=false`：STM32 的 P2 写死了区 2 的动作（`main.c:666-676`）。

**必须在车上验证**（源码不在仓库）：
- `stm32_link.second_move` 和 `merge_stations.moved_config` 能不能处理负数；
- `refine_second` 有没有假设区 2 的几何。
- 验证方法：先用 `p2` 命令，再手动 `second`。

**几何**：
- 第二站 (2208,2167) 原地转，扫过的圆半径约 195 mm，最右到 x=2403，和区 2 现在的情况一样（超出 3 mm）。
- 用模拟障碍物规划区 1：67 条指令、约 100 s；第一段要绕路；cut 点离目标只有 168 mm；最后是 `F 150, S -75`，**车头朝北墙冲**，更需要 5b 的 PREHOME。

## 6. 指令衔接
- **每条 F/S/R 之间都完全停车**。STM32 的顺序是：加速、匀速、减速、`Stop_Quick`、等 `SETTLE` 60 ms、`Yaw_Median5`（约 44 ms）、`Heading_Fix`（偏差大于 1.5° 时再转一次，约 1.2 s），然后回 DONE（`chassis.c:1063-1098`、`main.c:845-883`）。Pi 收到 DONE 才发下一条。
- 按 `Motion` 模型，55 条指令里约 40 s 花在起停和固定开销上（每条 v/a≈0.5 s 加 0.25 s），14 次转弯约 27 s。
- 停车点之间不能合并，因为要做任务。同一段里同方向的移动已经合并了（`route_plan.py:381-395,451-460`）。
- **可以省时间的地方**：
  - F 加 S 合成斜向移动：需要新的固件指令，规划也要按斜线检查，每对省约 0.7 s。
  - 去掉 ±25/±35 mm 的小动作：把停车点放在车道可用的栅格上，全程约 14 条，约 8 s。
  - 任务被跳过的停车点直接不停。
  - 规划改成黄区余量 15 mm 起步，或者先快速检查车道宽度：规划时间能从 13 s 降下来。
- Pi 的运动模型和 STM32 的加速度对不上：`route_acc_rpm_s=300` 对 `SCACC=200`，`strafe_acc_rpm_s=200` 对 `SSACC=140`。估计的用时偏乐观。

## 7. 其它风险和 bug
1. 第 0 节里的第 1–5 条。
2. 只要有一个新障碍物，站点扫描就把**这次看到的全部**障碍物 commit 进去（`:663`），而且用的是计划车位，可能产生重复障碍物。
3. `plan_route` 和 `save` 都在 matplotlib 的 GUI 线程里跑。终端输入的 `abort` 要排队等 `poll` 处理（`:686-694`），重新规划的时候急停会晚十几秒才发出（这时车本来是停着的）。
4. `__station` 调用 `launch` 时如果 busy 会被拒绝，`station_scan` 要干等 180 s 才超时（`:296,376,499-504`）。
5. go 在 QR 重新规划过以后，再用 `drive` 会执行 `state['legs']`，也就是从 QR 开始的那半段路线（`:396`）。
6. `do_home_fix` 的移动不经过 `check_move`，不检查车头误差（`:359`）。
7. `home_cut` 用的是 `stops[prev]`；如果那个停车点被挡住改停了别处，就和实际停车位置对不上（`auto_run.py:97`，对照 `route_plan.py:493-497`）。
8. 桌面模拟里不做回家对准（`:316-317`）。`test_auto_run` 没有覆盖 `home_cut`、approach 失败、站点重新规划、时间到了这些情况。建议先补一个"approach 动过车以后返回 False，不能再发原指令"的测试，可以参考我上面的复现。
9. 出发时转回车头的那一步 R（`flatten` 生成的第一条）不经过规划器的转弯空间检查。现在这一步的车角超出东边线 3 mm。

需要的话，我可以把这份报告整理成一个页面方便分享。