# car111 full-integration spec (2026-10-10)

Repo: /home/user/car111 (branch claude/tender-heisenberg-2t1awl).
Pi code: arm_patch/pi/lidar_map_north. STM32: arm_patch/Core/Src (+Inc), host tests arm_patch/tests_stm32 (bash run.sh).
Pi tests: cd arm_patch/pi/lidar_map_north && python3 -m unittest test_task_plan test_visual_servo test_vision test_matdet test_ringdet test_mission_cli test_auto_run test_route_margin test_arm_calib sim_mission  (231 pass at base).
Analysis reports (READ the ones for your area first): docs/integration/analysis_{nav-exec,nav-plan,stm32-chassis,stm32-io,mission,rules}.md
User's real map config (Oct 6): docs/pi_snapshot_1006/map_config_start2_roi.json. Pi-only modules (NOT in repo, must not be shipped or replaced; you may import them on the car only, with graceful fallback in tests): lidar_map_live, ld14p_scan, pose_fix, merge_stations, reference_correction, stm32_link, home_fix (snapshots of lidar_map_live/home_fix in docs/pi_snapshot_1006 show signatures).

## What the user asked (verbatim gist, Chinese user; all UI text/logs/comments in Chinese like the existing code)
1. Zone placing too slow ("first align, then go back and place wastes time"); small, slow corrections; "wheels first near the target, arm last".
2. QR: if not read, move a little forward/back and retry. Scanner is at the car's RIGHT-REAR, ~20 mm from the rear edge, ~40 mm from the right edge (car 290 x 260 → scanner ≈ 125 mm behind centre, 90 mm right of centre).
3. Never hit turntable / zones / obstacles; NEVER enter yellow areas or task areas (round ends); leave margin at turns.
4. Standby: lift goes to 60 mm automatically (power-on auto-height needs it near 60).
5. Screen: shows "choose start zone 1 or 2"; user chooses, places the car, it aligns by itself.
6. Returning into the start zone must be very precise; lidar localisation works well but sometimes the car still leaves the field (0 points).
7. Integrate everything and run the whole mission; start zone 1 never tested.
8. Smoother driving (less jitter), better chaining, fallbacks for failures. Think of what they missed.

## Rules that matter (see rules.md)
- One-key start with a clearly marked START button, only one start; after the start nobody may touch the car or the laptop (typing `y` is forbidden).
- Robot not moving for 15 s ends the round (23 s while waiting for the turntable). Planning/scanning stillness counts.
- Projection (arm excluded) entering any non-gray area (yellow, task zones; start zones excepted) ends the round. Touching counts.
- Start zone is drawn by lot (1 or 2), both must work. Car must fit a 300x300 square (260 wide + 40 lidar = 300!).
- QR board position random: board centre y in 1100..1300 on the east wall (code centre y 1098.5..1301.5); turntable centre x random 1100..1300.
- Placed materials must never be touched again. Second batch in temp zone only stacks on the same colour (flat fallback scores 0 → skip it).
- Round 180 s.

## Ownership (work ONLY in your files; other files belong to other agents)
- pi-nav: auto_run.py, map_merge_live.py, NEW reloc.py, test_auto_run.py, NEW test_reloc.py
- pi-plan: route_plan.py, test_route_margin.py, NEW test_route_safety.py (pure planning; no auto_run edits)
- pi-mission: mission_hooks.py, mission_cli.py, visual_servo.py, sim_mission.py, arm_link.py, apply_mission_config.py, test_mission_cli.py, test_visual_servo.py, task_plan.py, test_task_plan.py, README_mission.md
- stm32-chassis: Core/Src/chassis.c, Core/Inc/chassis.h, Core/Src/main.c, tests_stm32/test_chassis.c (+ its stubs), tests_stm32/run.sh
- stm32-io: Core/Src/arm.c, Core/Inc/arm.h, Core/Src/hwt101.c, tests_stm32/test_arm.c (+ its stubs)

## Interface contract (implement exactly; consumers must guard with hasattr/try so each side works alone)

### STM32 new/changed commands (stm32-io implements; Pi consumes)
- `ZONE?` → info line `ZONE <z> <s>` then DONE. z = chosen start zone 0/1/2 (0 = none yet), s = 1 if START was touched since the last ZONE ASK / ZONE n.
- `ZONE ASK` → draw the zone-choice page (two big buttons "1" | "2"), z=0, s=0, touch enabled. DONE.
- `ZONE 1` / `ZONE 2` → choose zone as if touched; draw the ready page (top "ZONE n", a status line, a big START button, a small BACK button); s=0. DONE.
- `ZONE MSG <ascii text ≤ 20>` → write the status line on the zone pages. DONE.
- `ZONE LOCK` → touch disabled, clear screen, back to the race layout (like Screen_Boot without READY). DONE.
- Touch: TJC screen on USART2 (9600). Send `sendxy=1` at boot and on ZONE ASK/ZONE n; parse 0x67/0x68 frames (9 bytes incl. FF FF FF) in the RX ISR; press+release inside the same button counts. Page 1: left half → zone 1, right half → zone 2 → ready page. Ready page: START → s=1 and status "GO"; BACK → page 1. Boot shows the zone page (touch enabled) unless ARM init fails.
- `PARK` → lift must be known (else ERR NOZERO). ARMOK=1: lift to ZHI, servos to (A1H, A2R) like STOW, then lift to 60 (LIFT_BOOT_MM). ARMOK=0: only lift to 60. DONE.
- `LIFT?` reply keeps the prefix `LIFT <known> <mm>` and appends ` BOOT=<ENC|NOCAL|NOENC|POS>`.
- QR: latch a valid code inside the RX ISR (so codes read while a motion command blocks are not lost); `QR?`/`QR CLR` unchanged in syntax.

### STM32 chassis (stm32-chassis)
- `R <deg>` accepts decimals (0.1 deg); `R 0` still = HOME. F/S/R syntax and the `DONE t=.. e=..` reply unchanged.
- Smoother turn end, strafe feed-forward without jumps, strafe heading loop like the straight one, precision mode for moves commanded at ≤ PSPD rpm (default 70): gentle accel, longer settle, heading fix at > 0.5 deg. New tunables must have SET names and safe defaults.

### Pi hooks API (pi-mission implements in mission_hooks.MissionHooks; pi-nav calls, always `if hasattr(hooks, name)`)
- `prepare(link, log)`: right after a race/go starts (before the second move): arm init + STOW if needed, sync arm params, `QR CLR`, write "---" to the code fields, open the camera. Idempotent, must not raise (log instead).
- `start_clock()`: competition timer t0 = now (the official start = START touched or terminal race/go confirmed).
- `time_left()`: existing.
- `go_home_now(next_role, est_leg_s, est_home_from_next_s)` → bool: True when doing the next stop's work would not leave time to get home (round_s=180, home_margin_s config); uses learned per-role work times.
- `keepalive()`: small visible arm motion (< 1 s, ID1 ±3° and back) to beat the 15 s rule during long still phases; no-op when the arm is not ready/disabled or called < 5 s ago. Only called from the mission thread.
- `park()`: STOW + lift 60 via `PARK` if supported else `STOW` + `LIFT 60`; skip if a material may be held. Used at the end of a run and when quitting (never automatically after abort).
- `task(stop, link, log)`: unchanged. QR wiggle uses `self.ctx.obstacles()` if available (list of (x_mm, y_mm, r_mm) world) and raw_cfg['stops']['QR'].

### Pi Ctx / auto_run (pi-nav implements; pi-mission consumes ctx.obstacles)
- `ctx.obstacles()` → [(x, y, r)] current map obstacles (world mm).
- `ctx.relocalize(plan_pose, frames=8)` → dict(ok, pose=(x,y,yaw_deg), dx, dy, dyaw, rms, inlier) using a fast ICP in reloc.py against the start reference cloud (scan 1 + scan 2 in the start body frame); gates: inlier ≥ 0.45, rms ≤ 15 mm, |shift| ≤ reloc_max_shift_mm (150), |dyaw| ≤ reloc_max_yaw_deg (5).

### route_plan helpers (pi-plan implements; pi-nav consumes, guarded)
- `pose_clear(cfg, obstacles, pose, margin_mm=None)` → (ok, min_clearance_mm, what): REAL footprint (car_length x car_width + lidar disk) vs yellow/task zones/turntable/field edge/obstacles.
- `moves_clear(cfg, obstacles, start_pose, cmds, margin_mm=None, pivot=None)` → (ok, min_clearance_mm, index, what): swept check of flattened commands [('F',mm)/('S',mm)/('R',deg)] (turns swept by angle steps around the configured pivot).
- `plan_leg(cfg, obstacles, start_pose, goal, start_pivot=None)` → leg dict like plan_mission legs (for first-leg replans and home legs), raising or returning None when impossible.
- `plan_mission(...)` keeps its signature; must become faster (target ≥3x on the user config) and never fall back to 0 mm yellow margin (minimum tier configurable, default 15 mm).
