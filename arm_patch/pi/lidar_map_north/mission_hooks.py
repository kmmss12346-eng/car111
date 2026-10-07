"""整场任务钩子：接在 auto_run.drive 每个停车点上(hooks.task)。

路线(配置里的 mission)： QR -> RAW -> ROUGH -> TEMP -> RAW -> ROUGH -> TEMP -> START
  QR     读任务码(STM32 的 GM65 扫码模块)，解析，显示到串口屏
  RAW    原料盘：按任务码的颜色顺序，用摄像头找到物料，闭环对准，夹起放进车上转盘 1/2/3 号槽
  ROUGH  粗加工区：把转盘里的物料依次放到指定圆环(摄像头对准圆环中心)，再按同样顺序取回转盘
  TEMP   暂存区：第一批平放在指定圆环；第二批码垛在同色物料上
  START  回到启停区：收臂，串口屏显示抓取/放置数量

放圆环的办法(为了准)：
  1 底盘沿车头方向挪到目标圆环旁边(圆环间隔 150mm 的整数倍，位置已知)
  2 手臂摆到观察姿态，爪子是空的，圆环不会被挡住；摄像头闭环对准圆环中心
  3 记下这时 ID1/ID2 的角度，去转盘取物料，再回到记下的角度(舵机回到同一角度的重复精度高)，下降、松手
码垛也是对准圆环中心(下面那个物料就放在这个圆环上)，只是下降高度浅一个物料。

出错处理：
  - 机械臂还没标定(ARMOK=0)、升降没回零：整轮不做夹放，路线照走，并在串口屏提示
  - 单个物料对不准或出错：跳过这个物料，收臂，继续下一个
  - 收到急停(abort)：立刻抛 Abort，路线停止
  - 超过 time_limit_s：不再做夹放，让车尽快回家
"""
import time

from arm_link import ArmLink, ArmActuators, ArmError, ArmAbort
from task_plan import parse_code, TaskError, role_of, Stats
from visual_servo import VisualServo, JacStore
from vision import Vision, VisionError

try:
    from auto_run import Abort
except Exception:                       # 单独测试时没有 auto_run
    class Abort(Exception):
        pass


DEFAULTS = dict(
    enabled=True,
    time_limit_s=150.0,                 # 每轮 3 分钟；超过它就不再做夹放，留出时间让车开回启停区(一次路线最后一段大约 20~30 秒)
    qr_timeout_s=6.0,                   # 到 QR 点最多等多久读码
    raw_wait_s=10.0,                    # 等原料盘停稳最多多久(规则：等转盘最多多停 8 秒)
    lift_init='skip',                   # 升降位置：STM32 开机就认为升降在离最低点 60mm(开机前放到这个高度)，一般不用管。'home'=让驱动器回零；'zero'/'skip'=不自动记零
    camera=dict(device='/dev/video0', width=640, height=480, fps=30, flip=None),
    frames=5,                                                    # 每次测量取几帧(多帧取中值；少一点快一点，噪声会大一点)
    claw_px=dict(RAW=[336.8, 282.9], RING=[336.8, 282.9]),      # 爪子轴线在画面里的位置
    px_per_mm=dict(RAW=4.36, RING=2.96),                         # 每毫米多少像素(只用来换算容差)
    tol_mm=dict(RAW=3.0, RING=1.0, PICK=2.5, STACK=2.0),         # 对准到多小算好
    accept_mm=dict(RAW=6.0, RING=2.5, PICK=5.0, STACK=4.0),      # 修正次数用完后，误差不超过这个也照常夹/放，超过就跳过
    servo=dict(),                                                # 覆盖 visual_servo.DEFAULTS
    servo_cal_file='servo_cal.json',
    chassis_fine_rpm=60,
    # 圆环相对停车点、沿车头方向的位置(毫米，正=在车头前方)。车头朝向见 map_config 的 stops：
    #   ROUGH 车头朝东，圆环板 3-2-1 从西到东 -> 1 号在前方 +150
    #   TEMP  车头朝南，板转了 90°           -> 1 号在后方 -150(如果实际相反，把正负号对调)
    ring_offset_mm=dict(ROUGH={'1': 150.0, '2': 0.0, '3': -150.0}, TEMP={'1': -150.0, '2': 0.0, '3': 150.0}),
    pickback_fast=True,                 # 粗加工区取回时直接回到放下时记下的底盘位置和手臂角度，只测一次确认，容差内就不重新对准(省 4~5 秒/个)
    pickback_order='code',              # 粗加工区取回的顺序：'code'=按任务码顺序(最符合规则)；'reverse'=倒序；'near'=就近(底盘走得最少，省时间)
    return_preload_deg=[0.0, 0.0],      # 回到记下的姿态前，先从反方向多转这些度(ID1, ID2)再回来，消除齿轮间隙；0=不用
    stop_aliases=None,                  # 停车点名字不叫 QR/RAW/ROUGH/TEMP/START 时，例如 {'ROUGH': ['PROC']}
    screen=dict(code='t0', code2='t7', stage='t1', grab='t2', place='t3', msg='t4', b1='t5', b2='t6'),   # 任务码分两行(字高 ≥12mm 一行放不下)：t0=前两组，t7=后两组
)


def deep_merge(base, extra):
    out = {}
    for k, v in base.items():
        out[k] = deep_merge(v, None) if isinstance(v, dict) else (list(v) if isinstance(v, list) else v)
    for k, v in (extra or {}).items():
        if isinstance(v, dict) and isinstance(out.get(k), dict):
            out[k] = deep_merge(out[k], v)
        else:
            out[k] = v
    return out


class MissionHooks:
    def __init__(self, raw_cfg=None, log=print, vision=None, store=None, now=time.monotonic, sleep=time.sleep, ctx=None):
        self.raw_cfg = raw_cfg or {}
        self.cfg = deep_merge(DEFAULTS, self.raw_cfg.get('mission_cfg'))
        self.log = log
        self.now = now
        self.sleep = sleep
        self.ctx = ctx                      # auto_run.drive 会把 ctx 填进来，用来检查 abort
        self.vision = vision
        self.store = store
        self.t0 = now()                     # 计时从创建开始(go 一开始就创建)
        self.arm = None
        self.act = None
        self.servo = None
        self.plan = None
        self.stats = Stats()
        self.visits = {}
        self.in_tray = {}                   # 槽位 -> Item(现在转盘里有的物料)
        self.on_ring = {}                   # (区, 圆环号) -> [Item, ...](从下到上)
        self.pose_at = {}                   # (区, 转盘槽位) -> 放下那一刻的 底盘位移 S/F 和手臂角度 a1/a2，取回时直接回到这里
        self.learn = {'S': 0.0, 'F': 0.0}   # 这个工位里前一个圆环对准时发现的停车误差(底盘位移里扣掉圆环间距后剩下的)，带到下一个圆环
        self.disabled = None                # 不为 None = 整轮不做夹放，原因在里面
        self._inited = False
        self.placements = []                # 记录每次放置：(区, 环, 是否码垛, 对准误差 mm)

    # ------------------------------------------------------------------ 给 auto_run 的接口
    def adjust(self, stop, link, log):
        """到停车点后调姿态：目前什么也不做(车位已经由雷达定位好；精确对准在 task 里用摄像头做)。"""
        return None

    def task(self, stop, link, log):
        self.log = log or self.log
        role = role_of(stop, self.cfg.get('stop_aliases'))
        if role is None:
            self.log(f'    (停车点 {stop} 没有对应任务，跳过)')
            return
        self._ensure(link)
        self.act.reset_disp()                       # 路线把车开到了新的停车点，位移从 0 重新算
        self.learn = {'S': 0.0, 'F': 0.0}
        self.visits[role] = self.visits.get(role, 0) + 1
        self.run_role(role, self.visits[role])

    # ------------------------------------------------------------------ 初始化
    def _ensure_arm_only(self, link):
        """只建立和 STM32 的指令通道(不初始化升降、不检查 ARMOK)：给 arm / qr 这类简单测试命令用。"""
        if self.arm is None:
            self.arm = ArmLink(link, log=self.log)

    def _ensure_vision(self):
        if self.vision is None:
            cfg = self.cfg
            self.vision = Vision(dict(camera=cfg['camera'], claw_px=cfg['claw_px'], px_per_mm=cfg['px_per_mm'], frames=cfg['frames']), log=self.log)

    def _ensure(self, link):
        if self._inited:
            return
        cfg = self.cfg
        self._ensure_arm_only(link)
        self._ensure_vision()
        if self.store is None:
            self.store = JacStore(cfg.get('servo_cal_file'))
        self.act = ArmActuators(self.arm, link, cfg['chassis_fine_rpm'], log=self.log)
        self.servo = VisualServo(self.act, self.store, cfg=cfg['servo'], log=self.log, sleep=self.sleep)
        self._inited = True
        self._init_arm()

    def _init_arm(self):
        try:
            params = self.arm.params()
            if not params or 'ARMOK' not in params:
                self.disabled = 'STM32 里没有机械臂参数(GET 里找不到 ARMOK)：STM32 程序是不是新版(arm.c)'
            elif params['ARMOK'] < 0.5:
                self.disabled = '机械臂参数还没标定(ARMOK=0)：标定好各姿态后 set ARMOK 1'
            else:
                known, mm = self.arm.lift_state()
                if known:
                    self.log(f'  升降已回零，现在 {mm:.1f}mm')
                elif self.cfg['lift_init'] == 'home':
                    self.arm.do('LIFT HOME')
                    self.log('  升降：驱动器回零完成')
                else:
                    self.disabled = '升降位置不知道了(急停打断过升降？)：把升降放到最低点，输入 arm LIFT ZERO'
        except ArmAbort as ex:
            raise Abort(str(ex))
        except (ArmError, Exception) as ex:
            self.disabled = f'初始化机械臂出错：{ex}'
        if self.disabled:
            self.log(f'★ 本轮不做夹取/放置：{self.disabled}')
            self._ui('msg', 'ARM OFF')
        else:
            self.log('  机械臂就绪。')

    def close(self):
        """释放摄像头。每次 go 结束要调用，否则下一次 go 打不开 /dev/video0。"""
        v = self.vision
        if v is not None and hasattr(v, 'close'):
            try:
                v.close()
            except Exception:
                pass

    # ------------------------------------------------------------------ 小工具
    def _ui(self, key, text):
        if self.arm is None:
            return
        obj = self.cfg['screen'].get(key)
        if obj:
            self.arm.screen(obj, text)

    def _check_abort(self):
        if self.ctx is not None and hasattr(self.ctx, 'aborted') and self.ctx.aborted():
            raise Abort('收到 abort，已停止')

    def elapsed(self):
        return self.now() - self.t0

    def time_left(self):
        return self.cfg['time_limit_s'] - self.elapsed()

    def _can_work(self):
        if self.disabled:
            return False
        if self.plan is None:
            self.log('    没有任务码，不能做夹取/放置')
            return False
        return True

    def _show_stats(self):
        self._ui('grab', self.stats.grab_text())
        self._ui('place', self.stats.place_text())

    def _recover(self, why):
        """单个物料出错后：尽量把手臂收好。收不好就停下整个路线(带着伸出的手臂乱走不安全)。"""
        self.log(f'    ★ {why}；收臂')
        try:
            self.arm.stow()
        except ArmAbort as ex:
            raise Abort(str(ex))
        except ArmError as ex:
            raise Abort(f'收臂也失败了({ex})，停止路线')

    # ------------------------------------------------------------------ 分发
    def run_role(self, role, visit):
        self._check_abort()
        h = {'QR': self._qr, 'RAW': self._raw, 'ROUGH': self._rough, 'TEMP': self._temp, 'START': self._start}[role]
        try:
            h(visit)
        except ArmAbort as ex:
            raise Abort(str(ex))

    # ------------------------------------------------------------------ QR
    def _qr(self, visit):
        self._ui('stage', 'QR SCAN')
        code = None
        t_end = self.now() + self.cfg['qr_timeout_s']
        while True:
            try:
                code = self.arm.qr()
            except ArmError as ex:
                self.log(f'    读二维码出错：{ex}')
                break
            if code or self.now() >= t_end:
                break
            self._check_abort()
            self.sleep(0.2)
        if not code:
            self.log('★ 没有读到二维码，本轮不做夹取/放置(路线照走)')
            self._ui('msg', 'QR FAIL')
            return
        try:
            self.plan = parse_code(code)
        except TaskError as ex:
            self.log(f'★ 二维码内容不对：{ex}')
            self._ui('msg', 'QR BAD')
            return
        self.log(f'    任务码 {self.plan.code}：{self.plan.describe()}')
        for w in self.plan.warnings:
            self.log(f'    注意：{w}')
        self._ui('code', self.plan.code[:7])
        self._ui('code2', self.plan.code[8:])
        l1, l2 = self.plan.screen_lines()
        self._ui('b1', l1)
        self._ui('b2', l2)
        self._ui('stage', 'QR OK')
        self._ui('msg', 'QR OK')
        self._show_stats()

    # ------------------------------------------------------------------ RAW：从原料盘抓进转盘
    def _raw(self, visit):
        if not self._can_work():
            return
        batch = 1 if visit <= 1 else 2
        for item in self.plan.items(batch):
            self._check_abort()
            if self.time_left() < 0:
                self.log('    ★ 时间到，不再抓取')
                break
            ok = False
            try:
                ok = self._grab_item(item)
            except (ArmError, VisionError) as ex:
                if isinstance(ex, ArmAbort):
                    raise
                self._recover(f'抓 {item.color_name} 出错：{ex}')
            self.stats.grab(ok)
            if ok:
                self.in_tray[item.slot] = item
            self._show_stats()
        self._stow_quiet()

    def _grab_item(self, item):
        cfg = self.cfg
        self.log(f'  ▶ 抓第{item.batch}批 #{item.index + 1}：{item.color_name} -> 转盘 {item.slot} 号槽')
        self._ui('stage', f'RAW GRAB {item.index + 1}/3 {item.color_short}')
        self._recenter(min_mm=20.0)                 # 上一个物料对准时底盘挪过的话先挪回停车点，免得这个物料出了视野
        self.arm.obs('RAW', open_claw=True)
        still, last = self.vision.wait_still(item.color, timeout_s=cfg['raw_wait_s'])
        if last is None and self._recenter(min_mm=3.0):
            self.log('    看不到，底盘挪回停车点再找一次')
            still, last = self.vision.wait_still(item.color, timeout_s=cfg['raw_wait_s'])
        if last is None:
            self.log(f'    ★ 看不到 {item.color_name} 物料，跳过')
            self._ui('msg', f'NO {item.color_short}')
            return False
        if not still:
            self.log('    原料盘还没停稳，按现在的位置先试(对准过程中会继续跟)')
        res = self.servo.run('RAW', lambda: self.vision.material_error(item.color), self.vision.scale('RAW'),
                             cfg['tol_mm']['RAW'], allow_chassis=True, label=f'抓{item.color_short}', bounds=self.vision.bounds('RAW'), confirm=False)
        self.log(f'    对准结果：{res}')
        if not res.ok and res.err_mm > cfg['accept_mm']['RAW']:
            self._recover(f'没对准({res.reason})，不夹，免得夹偏')
            return False
        self.arm.grab_here(item.slot)
        return True

    # ------------------------------------------------------------------ ROUGH：放下，再取回
    def _rough(self, visit):
        if not self._can_work():
            return
        batch = 1 if visit <= 1 else 2
        items = self.plan.items(batch)
        placed = []
        for item in items:
            self._check_abort()
            if self.time_left() < 0:
                self.log('    ★ 时间到，不再放置')
                break
            if item.slot not in self.in_tray:
                self.log(f'    转盘 {item.slot} 号槽是空的({item.color_name} 没抓到)，不放')
                continue
            if self._place_item(item, 'ROUGH', item.ring, stack=False, label=f'粗加工放{item.color_short}'):
                placed.append(item)
        for item in self._pickback_sequence('ROUGH', placed):
            self._check_abort()
            if self.time_left() < 0:
                self.log('    ★ 时间到，不再取回(物料留在粗加工区)')
                break
            self._pickback_item(item, 'ROUGH')
        self._return_to_stop('ROUGH')
        self._stow_quiet()

    def _goto_recorded(self, zone, item):
        """取回时直接回到放下那一刻的底盘位置和手臂角度(物料就在那儿)。成功回到返回 True；没有记录或不允许返回 False。
        回去以后下一步的 servo.run 会先测一次：偏差在容差内就不动，直接夹。"""
        rec = self.pose_at.get((zone, item.slot))
        if not rec or not self.cfg.get('pickback_fast', True):
            return False
        d = self.act.disp
        dS, dF = int(round(rec['S'] - d['S'])), int(round(rec['F'] - d['F']))
        if dS or dF:
            self.log(f'    底盘回到放下时的位置：前进 {dF:+d}mm、横移 {dS:+d}mm')
            self.act.chassis_move(dS, dF)
        self._return_to_pose(rec['a1'], rec['a2'])
        zobr = float(self.arm.params().get('ZOBRNG', 0.0))
        if zobr > 0.05:
            self.arm.lift(zobr)                              # 摄像头标定比例时用的高度
        return True

    def _pickback_sequence(self, zone, placed):
        mode = self.cfg.get('pickback_order', 'code')
        if mode == 'reverse':
            return list(reversed(placed))
        if mode == 'near':
            offs = self.cfg['ring_offset_mm'].get(zone) or {}
            left, out, pos = list(placed), [], self.act.disp['F'] - self.learn['F']
            while left:
                nxt = min(left, key=lambda it: abs(float(offs.get(str(it.ring), 0.0)) - pos))
                out.append(nxt)
                left.remove(nxt)
                pos = float(offs.get(str(nxt.ring), 0.0))
            return out
        return list(placed)

    def _pickback_item(self, item, zone):
        cfg = self.cfg
        self.log(f'  ▶ 从{zone}取回 {item.color_name}(环{item.ring}) -> 转盘 {item.slot} 号槽')
        self._ui('stage', f'{zone[:5]} PICK {item.index + 1}/3 R{item.ring}')
        try:
            if not self._goto_recorded(zone, item):
                self._goto_ring(zone, item.ring)
                self.arm.obs('RING', open_claw=True)
            res = self.servo.run('RING', self.vision.ring_error, self.vision.scale('RING'),
                                 cfg['tol_mm']['PICK'], allow_chassis=True, label=f'取回{item.color_short}', bounds=self.vision.bounds('RING'), confirm=False)
            self.log(f'    对准结果：{res}')
            if not res.ok and res.err_mm > cfg['accept_mm']['PICK']:
                self._recover(f'没对准({res.reason})，不取，免得夹歪碰倒')
                return False
            self._learn_from(zone, item.ring)
            self.arm.pick_here(item.slot)
        except ArmAbort:
            raise
        except (ArmError, VisionError) as ex:
            self._recover(f'取回出错：{ex}')
            return False
        self.in_tray[item.slot] = item
        stack = self.on_ring.get((zone, item.ring))
        if stack and item in stack:
            stack.remove(item)
        return True

    # ------------------------------------------------------------------ TEMP
    def _temp(self, visit):
        if not self._can_work():
            return
        batch = 1 if visit <= 1 else 2
        for item in self.plan.items(batch):
            self._check_abort()
            if self.time_left() < 0:
                self.log('    ★ 时间到，不再放置')
                break
            if item.slot not in self.in_tray:
                self.log(f'    转盘 {item.slot} 号槽是空的({item.color_name} 不在车上)，不放')
                continue
            ring, stack = item.ring, False
            if batch == 2:
                low = self.plan.lower_item(item)
                if low is not None and self.on_ring.get(('TEMP', low.ring)):
                    ring, stack = low.ring, True
                    self.log(f'    {item.color_name} 码垛到环{ring}(下面是第一批的 {low.color_name})')
                else:
                    # 没有同色的物料可叠(第一批那个没放成功)：赛题不允许叠在别的颜色上；改放到空着的圆环，没有空环就不放，免得撞倒
                    free = [r for r in (item.ring, 1, 2, 3) if not self.on_ring.get(('TEMP', r))]
                    if not free:
                        self.log(f'    {item.color_name} 没有同色物料可叠、暂存区也没有空环，不放(留在车上)')
                        continue
                    ring = free[0]
                    self.log(f'    {item.color_name} 找不到同色的第一批物料可叠，平放在空着的环{ring}')
            self._place_item(item, 'TEMP', ring, stack=stack, label=('码垛' if stack else '暂存放') + item.color_short)
        self._return_to_stop('TEMP')
        self._stow_quiet()

    # ------------------------------------------------------------------ 放到圆环(核心)
    def _place_item(self, item, zone, ring, stack, label):
        cfg = self.cfg
        key = 'STACK' if stack else 'RING'
        self.log(f'  ▶ {zone} 放 {item.color_name}(转盘{item.slot}号槽) 到环{ring}' + (' [码垛]' if stack else ''))
        self._ui('stage', f'{zone[:5]} PLACE {item.index + 1}/3 R{ring}' + (' S' if stack else ''))
        err_mm = None
        ok = False
        try:
            self._goto_ring(zone, ring)
            self.arm.obs('RING', open_claw=True)                 # 空爪到圆环上方：圆环不会被挡住
            res = self.servo.run('RING', self.vision.ring_error, self.vision.scale('RING'),
                                 cfg['tol_mm'][key], allow_chassis=True, label=label, bounds=self.vision.bounds('RING'))
            self.log(f'    对准结果：{res}')
            err_mm = res.err_mm
            if not res.ok and res.err_mm > cfg['accept_mm'][key]:
                self._recover(f'没对准({res.reason})，物料留在车上，不放')
            else:
                self._learn_from(zone, ring)                     # 底盘为了对准挪了多少，下一个圆环直接带上
                a1, a2 = self.arm.read_angles()                  # 记下对准时 ID1、ID2 的角度
                d = self.act.disp
                self.pose_at[(zone, item.slot)] = dict(S=d['S'], F=d['F'], a1=a1, a2=a2)
                self.arm.take(item.slot)                         # 去转盘取物料
                self._return_to_pose(a1, a2)                     # 回到记下的角度
                self.arm.drop(stack)                             # 下降、松手、抬起
                ok = True
        except ArmAbort:
            raise
        except (ArmError, VisionError) as ex:
            self._recover(f'放置出错：{ex}')
        self.stats.place(ok)
        if ok:
            self.in_tray.pop(item.slot, None)
            self.on_ring.setdefault((zone, ring), []).append(item)
            self.placements.append((zone, ring, bool(stack), err_mm))
        self._show_stats()
        return ok

    def _return_to_pose(self, a1, a2):
        pre = self.cfg.get('return_preload_deg') or [0.0, 0.0]
        if abs(pre[0]) > 1e-6 or abs(pre[1]) > 1e-6:
            self.arm.ap(a1 + pre[0], a2 + pre[1])                # 先从反方向靠近，再回来，消除齿轮间隙
        self.arm.ap(a1, a2)

    # ------------------------------------------------------------------ 底盘沿车头方向在圆环间挪动
    def _ring_nominal(self, zone, ring):
        offs = self.cfg['ring_offset_mm'].get(zone) or {}
        return float(offs.get(str(ring), 0.0))

    def _goto_ring(self, zone, ring):
        """底盘挪到这个圆环旁边：目标位置 = 圆环的名义位置 + 前面圆环对准时学到的停车误差；按累计位移算差多少。"""
        d = self.act.disp
        dF = int(round(self._ring_nominal(zone, ring) + self.learn['F'] - d['F']))
        dS = int(round(self.learn['S'] - d['S']))
        if dF == 0 and dS == 0:
            return
        self.log(f'    底盘挪到环{ring}：前进 {dF:+d}mm' + (f'、横移 {dS:+d}mm' if dS else ''))
        self.act.chassis_move(dS, dF)

    def _learn_from(self, zone, ring):
        d = self.act.disp
        self.learn = {'S': d['S'], 'F': d['F'] - self._ring_nominal(zone, ring)}

    def _return_to_stop(self, zone):
        d = self.act.disp
        s, f = -int(round(d['S'])), -int(round(d['F']))
        if s or f:
            self.log(f'    底盘挪回停车点：前进 {f:+d}mm、横移 {s:+d}mm')
            try:
                self.act.chassis_move(s, f)
            except ArmAbort:
                raise
            except ArmError as ex:
                raise Abort(f'底盘挪回停车点失败：{ex}')

    def _recenter(self, min_mm):
        """底盘离停车点超过 min_mm 就挪回去。返回是否挪了。"""
        d = self.act.disp
        if (d['S'] ** 2 + d['F'] ** 2) ** 0.5 < min_mm:
            return False
        s, f = -int(round(d['S'])), -int(round(d['F']))
        self.log(f'    底盘挪回停车点：前进 {f:+d}mm、横移 {s:+d}mm')
        self.act.chassis_move(s, f)
        return True

    def _stow_quiet(self):
        """每个工位做完收臂(开车前手臂要收好)。"""
        try:
            self.arm.stow()
        except ArmAbort:
            raise
        except ArmError as ex:
            raise Abort(f'收臂失败：{ex}')

    # ------------------------------------------------------------------ START：回到启停区
    def _start(self, visit):
        if self.arm is not None and not self.disabled:
            try:
                self.arm.stow()
            except ArmAbort:
                raise
            except ArmError as ex:
                self.log(f'    收臂失败：{ex}')
        self._ui('stage', 'DONE')
        self._show_stats()
        self._ui('msg', f'T={self.elapsed():.0f}s')
        s = self.stats
        self.log(f'  ■ 回到启停区。用时 {self.elapsed():.0f}s；{s.grab_text()}，{s.place_text()}')
        for zone, ring, stack, err in self.placements:
            self.log(f'      {zone} 环{ring}{" 码垛" if stack else ""}：对准误差 {err:.2f}mm' if err is not None else f'      {zone} 环{ring}')
