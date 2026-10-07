# 树莓派端：机械臂任务 + 视觉闭环

总说明（赛规风险、接线、串口屏、STM32 安装、上车调试顺序）在上一级的 `../../README.md`，先读那份。这里只讲树莓派端。

## 文件

| 文件 | 作用 |
|---|---|
| `mission_hooks.py` | **整场任务**：挂在 `auto_run.drive` 每个停车点上，按停车点分发 QR / RAW / ROUGH / TEMP / START |
| `visual_servo.py` | **摄像头闭环对准**：测偏差 → 动 ID1/ID2(精调)或底盘(粗调) → 再测；自己探测动作和画面的对应关系 |
| `vision.py` | 摄像头(只开一次)、物料和圆环识别(复用您的 `wuliao`、`ring_detect`；圆环检测改成了亚像素) |
| `arm_link.py` | STM32 机械臂指令封装、给视觉闭环用的手臂/底盘动作(含底盘位移记账) |
| `task_plan.py` | 任务码解析、每批物料的颜色/圆环/转盘槽位、码垛目标(同色匹配) |
| `mission_cli.py` | 终端测试命令：`arm` `qr` `mcode` `vcal` `vdbg` `mtest` `mot` |
| `apply_mission_config.py` | 给配置文件加 `mission_cfg`(只补缺的，自动备份) |
| `route_plan.py` | v14 的路线规划，加了**离黄色区硬性余量** `yellow_margin_mm`(默认 30)：某段按 30mm 走不通(如 400mm 宽的中间车道)时这一段自动减半/放到 0 并在终端提示 |
| `auto_run.py` `map_merge_live.py` | v14 的文件，各加了几行(见 `../v14_integration.patch`)；v4：车没转到位(`ERR STALL`/车头还差 10° 以上)时停下并提示 |
| `sim_mission.py` | 整场模拟(假 STM32 + 假摄像头 + 物理世界)，也能估算用时 |
| `test_*.py` | 单元测试 |

## 安装

1. 把本文件夹里的文件复制到树莓派的 `lidar_map_north/`。`auto_run.py`、`map_merge_live.py` 会**覆盖**您现有的同名文件——它们基于 v14。如果您的版本比 v14 新，不要覆盖，用补丁(在 `lidar_map_north` 的**上一级目录**运行)：`patch -p0 < v14_integration.patch`，补丁文件是 `../v14_integration.patch`，一共 3 处小改动：
   - `auto_run.drive`：开头加 `hooks.ctx = ctx`（让任务钩子能检查 abort）
   - `map_merge_live.mission_thread`：配置里 `mission_cfg.enabled` 为 true 时创建 `MissionHooks` 并传给 `run_mission`
   - `map_merge_live.command`：加 `arm vcal mtest vdbg qr mcode mot` 几个命令
   - v4 `auto_run`：第二站或路线里某条指令回 `ERR STALL`、或做完车头还差 10° 以上，马上停下并提示怎么查(以前会带着错的车位继续扫描、走路线)
2. 把 `chengxu` 里的 **`wuliao.py`、`ring_detect.py`** 复制到这个文件夹（识别就是用它们的）。
3. `python3 apply_mission_config.py`。
4. 需要 `numpy` 和 `opencv`（树莓派上识别代码本来就要用）。
5. 跑测试确认环境没问题：`python3 -m unittest test_task_plan test_visual_servo test_vision test_mission_cli test_auto_run test_route_margin sim_mission`

## 每个停车点做什么

停车点名字按前缀匹配角色：`QR`、`RAW`、`ROUGH`、`TEMP`、`START*`（和配置里 `stops`/`mission` 的名字一致；名字不同就配 `stop_aliases`，如 `{"ROUGH": ["PROC"]}`）。同一个角色第几次到，就是第几批。

**QR**：反复发 `QR?`(最多 `qr_timeout_s` 秒)，读到后解析，屏上显示任务码。没读到/内容不对：本轮不夹放，路线照走。

**RAW**（每个物料）：
`OBS RAW O`(手臂摆到原料盘上方、张开夹爪) → 等原料盘停稳(`wait_still`，最多 `raw_wait_s` 秒) → 视觉闭环对准(物料顶面中心对爪子位置) → `GRAB n H`(下降夹紧、抬起、转到转盘上方放进 n 号槽)。底盘为了对准挪过的话，抓下一个前先挪回停车点。

**ROUGH**：
- 放(每个物料)：底盘沿车头挪到圆环旁 → `OBS RING O`(**空爪**到圆环上方) → 闭环对准圆环中心 → 记下 ID1/ID2 角度 → `TAKE n`(去转盘取) → `AP`(回到记下的角度) → `DROP`(下降、松开、抬起、ID2 缩回)
  先用空爪对准是因为圆环不会被手里的物料挡住，而且舵机回到同一个角度的重复精度很高。
- 取回(按 `pickback_order`)：底盘回到放下时的位置 → `AP` 回到放下时的角度 → 测一次(在 `tol_mm.PICK` 内就不再动) → `PICK n H`
- 做完底盘挪回停车点，`STOW`。

**TEMP**：第一批和 ROUGH 的放一样(平放)；第二批找同色的第一批物料，对准那个圆环的中心，`DROP S`(下降浅一个物料高度)。第一批那个没放成功(没有同色物料可叠)：改放到空着的圆环，没有空环就不放。

**START**：`STOW`，屏上显示 `DONE`、抓取数、放置数、用时；终端打印每次放置的对准误差。

## 配置 `mission_cfg`（在 `map_config_start2_roi.json` 里）

`apply_mission_config.py` 会把默认值全写进去，您改自己要改的：

| 键 | 默认 | 说明 |
|---|---|---|
| `enabled` | true | false = `go` 时不做夹放，只走路线 |
| `time_limit_s` | 150 | 超过这个时间不再夹放。计时从 `go` 开始，要给回家留时间 |
| `qr_timeout_s` / `raw_wait_s` | 6 / 10 | 等二维码 / 等原料盘停稳最多多久 |
| `lift_init` | `"zero"` | 升降零点：`zero`(现在就是最高点) / `home`(驱动器回零) / `skip` |
| `camera` | /dev/video0 640x480 | `flip` 可设 -1/0/1(cv2.flip)，None 不翻 |
| `claw_px` | RAW、RING 都是 [336.8, 282.9] | 爪子轴线在画面里的位置。**调它微调放置位置**，1 像素≈0.34mm(圆环)/0.23mm(原料盘) |
| `px_per_mm` | RAW 4.36 / RING 2.96 | 每毫米多少像素，只用来把像素换成毫米判断容差；`vcal` 会告诉您实测值 |
| `tol_mm` | RAW 3.0、RING 1.0、PICK 2.5、STACK 2.0 | 对准到多小算好。想拿 1 环就 ≤1.0 |
| `accept_mm` | RAW 6.0、RING 2.5、PICK 5.0、STACK 4.0 | 修正次数用完后，误差不超过这个仍然夹/放，超过就跳过这个物料 |
| `ring_offset_mm` | 见总 README | 圆环相对停车点沿车头的位置，**要现场核对编号方向** |
| `pickback_fast` | true | 取回时回到放下时的姿态，不重新对准 |
| `pickback_order` | `"code"` | `code` 按任务码 / `reverse` 倒序 / `near` 就近(最省底盘移动) |
| `return_preload_deg` | [0, 0] | 回到记下的姿态前，先从反方向多转这些度(ID1, ID2)再回来，消除齿轮间隙；放置有固定方向的偏差时试试 |
| `frames` | 5 | 每次测量取几帧 |
| `servo` | {} | 覆盖视觉闭环参数，见下 |
| `servo_cal_file` | servo_cal.json | 校准结果文件(相对启动目录) |
| `chassis_fine_rpm` | 60 | 视觉微调时底盘速度 |
| `stop_aliases` | null | 停车点别名 |
| `yellow_margin_mm`(配置顶层) | 30 | 路线规划时车身离黄色区至少留多少 mm。嫌近就加大(50)，嫌绕远费时间就减小；0 = 和 v14 原版一样 |
| `screen` | t0 t7 t1 t2 t3 t4 t5 t6 | 串口屏控件名 |

## 终端命令（在 `map_merge_live` 的命令行里）

| 命令 | 作用 |
|---|---|
| `arm <指令>` | 直接给 STM32 发机械臂指令并打印回复：`arm LIFT ZERO`、`arm OBS RING O`、`arm AD 1 0.5`、`arm GET`… |
| `qr` | 读 STM32 里存的任务码 |
| `mcode 156+123+516+231` | 手动设任务码(不扫码也能测) |
| `vcal RING` / `vcal RAW <颜色号>` | 视觉校准。加 `arm` 只校准手臂不动底盘：`vcal RING arm` |
| `vdbg [RING\|RAW <颜色号>]` | 存 `vdebug.png`：爪子位置(绿十字)、识别到的圆环(黄)/物料(红叉) |
| `mtest QR` / `RAW n` / `ROUGH n` / `TEMP n` / `START` | 单独测一个工位(n=批次)。转盘里没东西时加 `force` 假定有：`mtest ROUGH 1 force` |
| `mtest reset` | 清空任务码和记录 |
| `gtest <颜色号>` / `gtest <颜色号> nogo` | 夹取测试(不需要 ARMOK、不用转盘)：摄像头找这个颜色的物料 → 手臂对准 → 下降夹住 → 抬起来。nogo = 只对准不夹。要先标定 A1G A2E ZOBRAW ZGRAB ZHI，并 `arm LIFT ZERO` |
| `mot` | 看 5 个电机驱动器(1 右前、2 左前、3 右后、4 左后、5 升降)的电压、是否使能、是否触发堵转保护，并给出中文建议。**车不动时先用它** |
| `mot en` | 5 个驱动器解除堵转保护并使能，再看一次状态(开机和每次 `go` 时 STM32 也会自动做一次) |

原有的 `set 名字 数值`、`get` 也能用于机械臂参数（`set ZPLC 95` 会存进配置，以后每次启动自动发给 STM32）。

## 视觉闭环怎么工作的

`visual_servo.py`：`p = 目标像素 − 爪子像素`。

- **雅可比 J**：每个单位动作(ID2 转 1°、ID1 转 1°、底盘 S/F 1mm)让目标在画面里移动多少像素。第一次(或发现对不上时)用小动作探测：动一下、看画面变化、算出 J；存进 `servo_cal.json`，下次直接用。探测步长自动加大到画面变化够测准为止；目标靠近画面边缘时用更小的步长，保证不会探测到目标出画面。
- **修正**：`Δ动作 = −J⁻¹·p × 增益`。偏差 ≤ 18mm 用手臂(精度高)，更大或手臂行程不够用底盘。每次修正后用实际移动量在线微调 J。
- **保护**：误差反而变大 → 丢掉 J 重新探测，再发散就放弃；手臂相对观察姿态的总偏移有上限；每步动作量、总时间都有上限；对准失败的物料跳过，不乱夹。

`mission_cfg.servo` 可以覆盖：`max_iter`(6)、`timeout_s`(10)、`gain_arm`(0.85)、`gain_ch`(0.8)、`settle_arm_s`(0.2，手臂动完等多久再拍)、`settle_ch_s`(0.35)、`arm_limit_deg`({id2:150, id1:12})、`step_limit_deg`({id2:45, id1:4})、`arm_cover_mm`(18)、`chassis_min_mm`(6)、`confirm`(true，达标后再测一次确认)等，名字和含义见 `visual_servo.py` 开头的 `DEFAULTS`。

**怎么判断校准对不对**：`vcal RING` 会打印"ID2 每转 1° ≈ x mm、ID1 每转 1° ≈ y mm、实测 z 像素/毫米"。ID1 约等于(臂长 × π/180)毫米/度(臂长 150mm 时约 2.6)，ID2 约等于齿轮分度圆周长/360。数量级对不上，说明摄像头没拍到目标或者机构有问题。

## 出错时的行为

| 情况 | 行为 |
|---|---|
| `ARMOK=0` / 没回零 / STM32 里没有机械臂参数 | 整轮不做夹放，路线照走，屏上 `ARM OFF` |
| 没读到二维码 / 内容不对 | 整轮不做夹放，屏上 `QR FAIL` / `QR BAD` |
| 某个物料看不到、对不准、STM32 回错误 | 跳过这个物料，`STOW` 收臂，继续下一个 |
| 收臂也失败 | 抛 `Abort`，路线停止（带着伸出的手臂乱走不安全） |
| 终端输入 `abort` | STM32 停升降，当前动作回 `ERR ABORT`，钩子抛 `Abort`，路线停止 |
| 转弯时轮子没转起来(`ERR STALL`)，或者指令做完车头还差 10° 以上 | 路线马上停止，终端提示：`mot` 看驱动器、抬车 `send R 90`、充电、电机电源关了再开 |
| 超过 `time_limit_s` | 不再夹放，路线继续走完 |

## 测试

```
python3 -m unittest test_task_plan test_visual_servo test_vision test_mission_cli test_auto_run test_route_margin sim_mission
python3 sim_mission.py                   # 整场模拟，打印过程和结果
python3 sim_mission.py fast 1            # 提速参数、只做第一批，看用时估计
python3 sim_mission.py 1 LFRPM=300 ASPD=200 CLWAIT=250    # 用您自己的速度参数估算
```
模拟里摄像头朝向、镜像、ID1/ID2 每度多少毫米都是随机的，程序事先不知道，必须靠探测——和真车一样。
