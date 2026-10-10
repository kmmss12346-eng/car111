# 树莓派端：机械臂任务 + 视觉闭环

总说明（赛规风险、接线、串口屏、STM32 安装、上车调试顺序）在上一级的 `../../README.md`，先读那份。这里只讲树莓派端。

## 文件

| 文件 | 作用 |
|---|---|
| `mission_hooks.py` | **整场任务**：挂在 `auto_run.drive` 每个停车点上，按停车点分发 QR / RAW / ROUGH / TEMP / START |
| `visual_servo.py` | **摄像头闭环对准**：测偏差 → 动 ID1/ID2(精调)或底盘(粗调) → 再测；自己探测动作和画面的对应关系 |
| `vision.py` | 摄像头(只开一次，后台线程一直取最新一帧)、爪子点/物料半径标定文件 `vision_cal.json` |
| `matdet.py` | 物料识别(抗爪子遮挡)：只用没被挡住的那段圆边拟合整个圆，被挡住 80% 圆心也只差 1~2 像素；颜色范围用 `wuliao.py` 里的 |
| `ringdet.py` | 圆环识别(抗爪子遮挡)：每段边缘拟合圆弧，同心的合成一个圆环，被挡掉一截也能认，圆心误差零点几像素 |
| `vlive.py` | 摄像头实时预览(在树莓派桌面终端运行，自己开摄像头，map_merge_live 要先退出)：画出识别到的圆、爪子区域、离爪子点多少毫米 |
| `vview.py` | 实时看 map_merge_live 正在用的摄像头画面(rtest / gtest / mtest / vclaw / go 时)，不用退出 map_merge_live |
| `arm_link.py` | STM32 机械臂指令封装、给视觉闭环用的手臂/底盘动作(含底盘位移记账) |
| `task_plan.py` | 任务码解析、每批物料的颜色/圆环/转盘槽位、码垛目标(同色匹配) |
| `mission_cli.py` | 终端测试命令：`arm` `qr` `mcode` `vmask` `vclaw` `vcal` `vdbg` `gtest` `mtest` `mot` |
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
2. 把 `chengxu` 里的 **`wuliao.py`** 复制到这个文件夹(物料颜色范围用它里面调好的；没有就用 matdet.py 里一样的默认值)。
3. `python3 apply_mission_config.py`。
4. 需要 `numpy` 和 `opencv`（树莓派上识别代码本来就要用）。
5. 跑测试确认环境没问题：`python3 -m unittest test_task_plan test_visual_servo test_vision test_mission_cli test_auto_run test_route_margin sim_mission`

## 每个停车点做什么

停车点名字按前缀匹配角色：`QR`、`RAW`、`ROUGH`、`TEMP`、`START*`（和配置里 `stops`/`mission` 的名字一致；名字不同就配 `stop_aliases`，如 `{"ROUGH": ["PROC"]}`）。同一个角色第几次到，就是第几批。

**QR**：反复发 `QR?`(最多 `qr_timeout_s` 秒)，读到后解析，屏上显示任务码。没读到/内容不对：本轮不夹放，路线照走。

**RAW**（每个物料）：
`OBS RAW O`(手臂摆到原料盘上方、张开夹爪) → 等原料盘停稳(`wait_still`，最多 `raw_wait_s` 秒) → 视觉闭环对准(物料顶面中心对爪子位置) → `GRAB n H`(下降夹紧、抬起、转到转盘上方放进 n 号槽)。底盘为了对准挪过的话，抓下一个前先挪回停车点。

**ROUGH / TEMP 到工位先做的事**：
- (只在 `mtest` 里)`HOME`：把现在的车头方向记为要保持的方向。车是手搬过来的，不记的话底盘一前后挪就把车头往开机时的方向转。
- **看清三个圆环**(`ring_survey`)：`OBS RING O`，拍一次画面找出所有圆环，按排列认出 1、2、3 号(画面里从左到右的顺序见 `ring_order`；
  看到三个直接认；两个看哪一头没有环；一个当作 2 号)，算出"每个环正对爪子时底盘要前进/后退多少"。
  第一次还会底盘前进 20mm 再退回来，看画面往哪边动(存进 `servo_cal.json`，以后不再测)。顺便用圆环间距(`ring_spacing_mm`=150)算每毫米多少像素(没做 `vclaw RING` 时用它判断误差)。
  车要停在 **2 号环正对爪子、车身和圆环那一排平行**；停偏 75mm 以上(画面里只看到两个环、离爪子都远)会提示"认不准"。

**ROUGH**：
- 放(按任务码顺序，每个物料)：底盘沿圆环那一排前后挪，一条指令开到这个环(按看到的位置) → `OBS RING O`(**空爪**到圆环上方) → 闭环对准圆环中心
  (只认这个环，不去追旁边那个；**先用手臂**，沿这一排差 18mm 以上才前后挪底盘、最多 `ring_fix_max_mm`；
  离圆环远一点近一点靠 ID2 伸缩补，**伸缩到头还够不着，车轮才横着靠近/退远**，只挪够不着的那一段，离停车点累计不超过 `ring_strafe_max_mm`)
  → 白心里已经有东西就不放 → 记下 ID1/ID2 角度 → `TAKE n`(去转盘取) → `AP`(回到记下的角度)
  → 下降到 `ZPLC`、松开、抬到 `ZHI`(**不缩回**、不回去拍照，直接去下一个环；`drop_retract=true` 改回用 `DROP`)
  先用空爪对准是因为圆环不会被手里的物料挡住，而且舵机回到同一个角度的重复精度很高。
  取出物料以后出错(回不到姿态、下降失败)：先把物料放回它的转盘槽再收臂，不会夹着物料到处走。
- (`learn_pick=true` 时)第一次放下物料后回去量 PICK 点(物料在爪子正下方时它顶面圆心在画面里的位置)，取回时直接认物料。默认关(要多拍一次)，取回认圆环外圈。
- 取回(按 `pickback_order`)：底盘回到放下时的位置(只前后) → `AP` 回到放下时的角度 → 白心被物料盖住：量过 PICK 点就认物料顶面，否则认圆环外圈(放物料时记下了空圆环的大小)
  → 都认不到就回到放下时的位置直接夹 → `PICK n H`
- 做完先 `STOW`(收臂)，再把底盘挪回停车点。

**TEMP**：第一批和 ROUGH 的放一样(平放)；第二批找同色的第一批物料，先认它的顶面对到 PICK 点(认不到就认圆环外圈；都认不到不放，免得砸倒下面那个)，下降到 `ZSTK`(浅一个物料高度)。第一批那个没放成功(没有同色物料可叠)：改放到空着的圆环，没有空环就不放。

**转盘槽被占着**：前面没放出去的物料还在某个槽里，第二批同一个槽的物料就不夹(放进去会砸在上面)，也不会把它当成第二批的去放。

**START**：`STOW`，屏上显示 `DONE`、抓取数、放置数、用时；终端打印每次放置的对准误差。

## 配置 `mission_cfg`（在 `map_config_start2_roi.json` 里）

`apply_mission_config.py` 会把默认值全写进去，您改自己要改的：

| 键 | 默认 | 说明 |
|---|---|---|
| `enabled` | true | false = `go` 时不做夹放，只走路线 |
| `time_limit_s` | 150 | 超过这个时间不再夹放。计时从 `go` 开始，要给回家留时间 |
| `qr_timeout_s` / `raw_wait_s` | 6 / 10 | 等二维码 / 等原料盘停稳最多多久 |
| `lift_init` | `"zero"` | 升降零点：`zero`(现在就是最低点，数字越大越高) / `home`(驱动器回零) / `skip` |
| `camera` | /dev/video0 640x480 | `flip` 可设 -1/0/1(cv2.flip)，None 不翻 |
| `claw_px` | RAW、RING 都是 [336.8, 282.9] | 爪子点(物料夹正时圆心在画面里的位置)。**用 `vclaw RAW <颜色>` / `vclaw RING` 实测**，存在 `vision_cal.json`，比这里的优先 |
| `px_per_mm` | RAW 1.97 / RING 2.96 | 每毫米多少像素，只用来把像素换成毫米判断容差。RAW 认到物料后按物料半径和 `material_diam_mm`(50) 现算；RING 在 `vclaw RING` 量过圆环后按 `ring_outer_diam_mm`(95) 算，没量过就在到工位看清圆环时按圆环间距算(现场约 1.46) |
| `tol_mm` | RAW 2.0、RING 2.0、PICK 2.5、STACK 2.5 | 对准到多小算好。差不多就行，不为零点几毫米反复微调；想更准把 RING 改小(1.0)，会多调一两下 |
| `accept_mm` | RAW 4.0、RING 3.0、PICK 5.0、STACK 4.0 | 修正次数用完后，误差不超过这个仍然夹/放，超过就跳过这个物料(爪子每边只有约 5mm 余量) |
| `place_confirm` | true | 放物料对准到容差内后再拍一次确认(零点几秒，防止一次测量的噪声把偏了的当成对准了) |
| `align_max_iter` | 6 | 工位里对准最多修正几次 |
| `detector` / `ring_detector` | `circle` / `arcs` | 识别方法：换回原来的写 `wuliao` / `contour` |
| `matdet` / `ringdet` | {} | 识别参数(一般不用改)，比如 `matdet.claw_mask` 爪子区域文件名 |
| `vision_cal_file` | vision_cal.json | `vclaw` 实测结果文件 |
| `return_tol_deg` | 0.4 | 取完物料回到对准姿态后回读角度，差得比这个多就再转一次(最多 2 次) |
| `ring_survey` | true | 到粗加工区/暂存区先看清三个圆环，按看到的位置直接开到每个环(false = 按 `ring_offset_mm` 的名义位置走) |
| `ring_order` | ROUGH `lr`、TEMP `lr` | 摄像头画面里 1、2、3 号环的排列：`lr` = 从左到右 1 2 3；`rl` = 从左到右 3 2 1。**现场看 vview 核对**(mtest 加 `rev` 是临时反过来) |
| `ring_spacing_mm` | 150 | 相邻两个圆环中心的距离 |
| `chassis_strafe` | false | true = 工位里对准时底盘随便横移(会压进工位，不推荐)。false = 沿圆环那一排前后挪，远近先靠 ID2 伸缩补，补不过来才横移(见下一行) |
| `ring_strafe_max_mm` | 40 | 手臂伸缩够不着时车轮横着挪(靠近/退远圆环)，离停车点累计最多这么多毫米。车停的时候离工位线留够这个距离；0 = 绝不横移 |
| `ring_fix_max_mm` | 60 | 对准一个圆环时底盘最多再为对准前后挪这么多毫米，超过就停下(多半认错了环) |
| `ring_move_min_mm` | 15 | 去下一个环要挪的距离比这小就不动底盘(手臂够得着) |
| `drop_retract` | false | 放下物料后要不要缩回伸缩舵机(true = 用 STM32 的 `DROP`，会缩回) |
| `mtest_hold_heading` | true | mtest ROUGH/TEMP 开始时 `HOME`：以现在的车头方向为准 |
| `ring_offset_mm` | 见总 README | `ring_survey=false` 或看不清圆环时用：圆环相对停车点沿车头的位置 |
| `pickback_fast` | true | 取回时回到放下时的姿态，不重新对准 |
| `learn_pick` | false | true = 还没量过 PICK 点时，第一个物料放下后回去拍一次量出来(多几秒)；false = 取回认圆环外圈 |
| `check_ring_empty` | true | 平放前看一眼白心：已经放着东西(比如取回失败留下的)就不放 |
| `pickback_order` | `"code"` | `code` 按任务码 / `reverse` 倒序 / `near` 就近(最省底盘移动) |
| `return_preload_deg` | [0, 0] | 回到记下的姿态前，先从反方向多转这些度(ID1, ID2)再回来，消除齿轮间隙；放置有固定方向的偏差时试试 |
| `frames` | 5 | 每次测量取几帧 |
| `servo` | {} | 覆盖视觉闭环参数，见下 |
| `servo_cal_file` | servo_cal.json | 校准结果文件(相对启动目录) |
| `chassis_fine_rpm` | 100 | 视觉微调时底盘速度(60mm 以内的挪动) |
| `stop_aliases` | null | 停车点别名 |
| `yellow_margin_mm`(配置顶层) | 30 | 路线规划时车身离黄色区至少留多少 mm。嫌近就加大(50)，嫌绕远费时间就减小；0 = 和 v14 原版一样 |
| `screen` | t0 t7 t1 t2 t3 t4 t5 t6 | 串口屏控件名 |

## 终端命令（在 `map_merge_live` 的命令行里）

| 命令 | 作用 |
|---|---|
| `arm <指令>` | 直接给 STM32 发机械臂指令并打印回复：`arm LIFT ZERO`、`arm OBS RING O`、`arm AD 1 0.5`、`arm GET`… |
| `qr` | 读 STM32 里存的任务码 |
| `mcode 156+123+516+231` | 手动设任务码(不扫码也能测) |
| `vmask` | 标定爪子在画面里占的区域(存 `claw_mask.png`)：`arm OBS RAW O`、爪子附近没有物料时用。**蓝色/浅蓝物料必须先做** |
| `vclaw RAW <颜色号>` | 实测原料爪子点：原料盘停住、物料放在爪子正下方。会降下去夹正、松开、升回去测，两遍一致才存 |
| `vclaw RING [外径mm]` | 实测圆环爪子点：先 `arm DROP` 放一个物料、圆环纸挪到物料正好在中心、物料拿走，再输入。会自己回 OBS RING 再测；后面写上尺子量的黑环外径(如 `vclaw RING 100`)，像素才能换成准确的毫米 |
| `rtest [arm]` | 圆环识别+对准测试：OBS RING → 找圆环 → 手臂(够不着时底盘)对准，报告误差，不取不放。`arm` = 只动手臂。同时开 `python3 vview.py` 看画面 |
| `vcal RING` / `vcal RAW <颜色号>` | 视觉校准。加 `arm` 只校准手臂不动底盘：`vcal RING arm` |
| `vdbg [RING\|RAW <颜色号>]` | 存 `vdebug.png`：爪子位置(绿十字)、识别到的圆环(黄)/物料(红叉) |
| `mtest QR` / `RAW n` / `ROUGH n` / `TEMP n` / `START` | 单独测一个工位(n=批次)。转盘里没东西时加 `force` 假定有：`mtest ROUGH 1 force` |
| `mtest ROUGH n nogo` / `rev` | `nogo` = 只认圆环、对准，不取不放(先看对得准不准)；`rev` = 这次 1 号、3 号环反过来。mtest 每次重新计时，开始时 `HOME`(以现在的车头方向为准)，底盘位移从 0 算 |
| `mtest reset` | 清空任务码和记录 |
| `vclaw PICK` | 清掉物料顶面的爪子点(PICK)，下次放下物料时重新量。摄像头/爪子动过、改过 `ZOBRNG` 时用(改了 `ZOBRNG` 程序也会自己发现) |
| `vwatch RING` / `vwatch RAW <颜色号>` / `vwatch PICK <颜色号>` / `vwatch off` | 后台一直识别，配合另一个桌面终端的 `python3 vview.py` 实时看；这时照常用 `arm LIFT`、`arm AD` 调手臂 |
| `gtest <颜色号>` / `gtest <颜色号> nogo` | 夹取测试(不用转盘；要 ARMOK=1)：摄像头找这个颜色的物料 → 手臂对准 → 下降夹住 → 抬起来。nogo = 只对准不夹。要先标定 A1G A2E ZOBRAW ZGRAB ZHI，并 `arm LIFT ZERO` |
| `mot` | 看 5 个电机驱动器(1 右前、2 左前、3 右后、4 左后、5 升降)的电压、是否使能、是否触发堵转保护，并给出中文建议。**车不动时先用它** |
| `mot en` | 5 个驱动器解除堵转保护并使能，再看一次状态(开机和每次 `go` 时 STM32 也会自动做一次) |

原有的 `set 名字 数值`、`get` 也能用于机械臂参数（`set ZPLC 95` 会存进配置，以后每次启动自动发给 STM32）。

## 视觉闭环怎么工作的

`visual_servo.py`：`p = 目标像素 − 爪子像素`。

- **雅可比 J**：每个单位动作(ID2 转 1°、ID1 转 1°、底盘 S/F 1mm)让目标在画面里移动多少像素。第一次(或发现对不上时)用小动作探测：动一下、看画面变化、算出 J；存进 `servo_cal.json`，下次直接用。探测步长自动加大到画面变化够测准为止；目标靠近画面边缘时用更小的步长，保证不会探测到目标出画面。
- **修正**：`Δ动作 = −J⁻¹·p × 增益`。偏差 ≤ 18mm 用手臂(精度高)，更大或手臂行程不够用底盘。粗加工区/暂存区里：只有沿圆环那一排差 18mm 以上才前后挪底盘，横向(离圆环远近)的偏差先靠 ID2 伸缩，伸缩到头还差才让车轮横移那一段(`chassis_axes='Fs'`)。
  每次修正后用实际移动量在线微调 J(按每个轴让画面动了多少像素分摊，ID2 每度只动零点几像素，不会被测量噪声带偏)。
  离目标很近(容差的 2 倍以内)时先再测一次取平均再决定动不动，不去追测量噪声。
- **保护**：误差反而变大 → 丢掉 J 重新探测，再发散就放弃；手臂相对观察姿态的总偏移有上限；每步动作量、总时间都有上限；对准失败的物料跳过，不乱夹。

`mission_cfg.servo` 可以覆盖：`max_iter`(6)、`timeout_s`(10)、`gain_arm`(0.85)、`gain_ch`(0.8)、`settle_arm_s`(0.2，手臂动完等多久再拍)、`settle_ch_s`(0.35)、`arm_limit_deg`({id2:250, id1:12})、`step_limit_deg`({id2:90, id1:4})、`chassis_axes`('SF'；工位里自动用 'F')、`near_avg`(2.0)、`arm_cover_mm`(18)、`chassis_min_mm`(6)、`confirm`(true，达标后再测一次确认)等，名字和含义见 `visual_servo.py` 开头的 `DEFAULTS`。

**怎么判断校准对不对**：`vcal RING` 会打印"ID2 每转 1° ≈ x mm、ID1 每转 1° ≈ y mm、实测 z 像素/毫米"。ID1 约等于(臂长 × π/180)毫米/度(臂长 150mm 时约 2.6)，ID2 约等于齿轮分度圆周长/360。数量级对不上，说明摄像头没拍到目标或者机构有问题。

## 出错时的行为

| 情况 | 行为 |
|---|---|
| `ARMOK=0` / 没回零 / STM32 里没有机械臂参数 | 整轮不做夹放，路线照走，屏上 `ARM OFF` |
| 没读到二维码 / 内容不对 | 整轮不做夹放，屏上 `QR FAIL` / `QR BAD` |
| 某个物料看不到、对不准、STM32 回错误 | 跳过这个物料，`STOW` 收臂，继续下一个 |
| 已经从转盘取出物料以后出错 | 先把物料放回它的转盘槽再收臂；放回也失败：爪子保持夹紧收臂，本轮不再夹放 |
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
