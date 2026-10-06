"""任务码解析与整场流程规划（纯逻辑，不碰硬件，电脑上可直接测试）。

任务码 "a+b+c+d"，四组三位数（赛题）：
  a  第一批三个物料的颜色和搬运顺序      颜色：1红 2黄 3蓝 4绿 5黑 6浅蓝
  b  第一批物料在粗加工区和暂存区的放置位置   圆环号 1~3
  c  第二批三个物料的颜色和搬运顺序
  d  第二批物料在粗加工区的放置位置
第二批在暂存区只能"码垛"在第一批物料上面，颜色要一致：
  第二批里颜色为 X 的物料，叠在第一批里颜色为 X 的那个物料所在的圆环上。

例：156+123+516+231
  第一批 红(1)、黑(5)、浅蓝(6) 依次抓，依次放在 1、2、3 号圆环(粗加工区和暂存区都是)
  第二批 黑(5)、红(1)、浅蓝(6) 依次抓，在粗加工区依次放在 2、3、1 号圆环；
         到暂存区叠在同色物料上：黑叠在 2 号环(第一批的黑在 2)，红叠在 1 号环，浅蓝叠在 3 号环
"""
import re

COLOR_NAMES = {1: 'RED', 2: 'YELLOW', 3: 'BLUE', 4: 'GREEN', 5: 'BLACK', 6: 'LIGHT_BLUE'}
COLOR_SHORT = {1: 'RED', 2: 'YEL', 3: 'BLU', 4: 'GRN', 5: 'BLK', 6: 'LBL'}      # 串口屏上用(短)


class TaskError(ValueError):
    pass


class Item:
    """一个要搬运的物料。slot = 车上转盘的槽位(按抓取顺序 1、2、3)。"""
    __slots__ = ('batch', 'index', 'color', 'ring')

    def __init__(self, batch, index, color, ring):
        self.batch, self.index, self.color, self.ring = batch, index, color, ring

    @property
    def slot(self):
        return self.index + 1

    @property
    def color_name(self):
        return COLOR_NAMES[self.color]

    @property
    def color_short(self):
        return COLOR_SHORT[self.color]

    def __repr__(self):
        return f'Item(b{self.batch}#{self.index + 1} {self.color_name} ring{self.ring} slot{self.slot})'


def clean_code(text):
    """去掉空格、引号、换行，全角加号换成半角。"""
    t = str(text or '')
    t = t.replace('＋', '+').replace('﹢', '+')
    return re.sub(r'[\s"\'\x00-\x1f]', '', t)


class Plan:
    def __init__(self, code, batches, warnings):
        self.code = code
        self.batches = batches            # {1: [Item, Item, Item], 2: [...]}
        self.warnings = warnings

    def items(self, batch):
        return list(self.batches[batch])

    def lower_item(self, item):
        """码垛时第二批的 item 要叠在哪个第一批物料上(颜色相同的)；找不到返回 None。"""
        if item.batch != 2:
            return None
        for it in self.batches[1]:
            if it.color == item.color:
                return it
        return None

    def stack_ring(self, item):
        """第二批的 item 在暂存区应叠放的圆环号(第一批同色物料所在的圆环)；找不到返回 None。"""
        low = self.lower_item(item)
        return low.ring if low is not None else None

    def describe(self):
        out = []
        for b in (1, 2):
            out.append(f'第{b}批: ' + ' '.join(f'{it.color_name}->环{it.ring}' for it in self.batches[b]))
        return '；'.join(out)

    def screen_lines(self):
        """给串口屏用的纯 ASCII 短文本。"""
        l1 = 'B1 ' + ' '.join(f'{it.color_short}{it.ring}' for it in self.batches[1])
        l2 = 'B2 ' + ' '.join(f'{it.color_short}{it.ring}' for it in self.batches[2])
        return l1, l2


def parse_code(text):
    """解析并检查任务码，返回 Plan。格式不对抛 TaskError。"""
    code = clean_code(text)
    parts = code.split('+')
    if len(parts) != 4:
        raise TaskError(f'任务码必须是 4 组三位数，用 + 连接：{code!r}')
    for p in parts:
        if len(p) != 3 or not p.isdigit():
            raise TaskError(f'每组必须是 3 位数字：{code!r}')
    warnings = []
    for gi in (0, 2):
        for ch in parts[gi]:
            if int(ch) not in COLOR_NAMES:
                raise TaskError(f'颜色编号必须是 1~6，发现 {ch}：{code!r}')
        if len(set(parts[gi])) != 3:
            warnings.append(f'第{gi // 2 + 1}批的三个颜色有重复：{parts[gi]}（赛题说三种不同颜色）')
    for gi in (1, 3):
        for ch in parts[gi]:
            if ch not in '123':
                raise TaskError(f'圆环编号必须是 1、2、3，发现 {ch}：{code!r}')
        if sorted(parts[gi]) != ['1', '2', '3']:
            warnings.append(f'第{gi // 2 + 1}批的放置位置 {parts[gi]} 不是 1、2、3 各一个')
    batches = {
        1: [Item(1, i, int(parts[0][i]), int(parts[1][i])) for i in range(3)],
        2: [Item(2, i, int(parts[2][i]), int(parts[3][i])) for i in range(3)],
    }
    return Plan(code, batches, warnings)


# ---------------------------------------------------------------------------
# 停车点名字 -> 角色
# ---------------------------------------------------------------------------
ROLES = ('QR', 'RAW', 'ROUGH', 'TEMP', 'START')


def role_of(stop, aliases=None):
    """路线里的停车点名字(如 'QR'、'RAW'、'START2')对应哪种任务。不认识返回 None。
    aliases：配置里的 {'角色': ['名字前缀', ...]}，没给就按 ROLES 本身的名字(不分大小写、按前缀)匹配。"""
    s = str(stop or '').upper()
    amap = {r: [r] for r in ROLES}
    for r, names in (aliases or {}).items():
        r = str(r).upper()
        if r in amap:
            amap[r] = [str(n).upper() for n in (names if isinstance(names, (list, tuple)) else [names])]
    for r in ROLES:
        for n in amap[r]:
            if s.startswith(n):
                return r
    return None


class Stats:
    """串口屏上显示的完成情况。

    grab_ok / grab_total      从原料盘抓进车里的物料数(最多 6)
    place_ok / place_total    放到圆环上的次数(粗加工区 6 次 + 暂存区 6 次，最多 12)
    注意：这里的"成功"是指令执行成功且对准误差在容许范围内，不是传感器确认的结果。
    """

    def __init__(self):
        self.grab_ok = 0
        self.grab_total = 0
        self.place_ok = 0
        self.place_total = 0

    def grab(self, ok):
        self.grab_total += 1
        self.grab_ok += 1 if ok else 0

    def place(self, ok):
        self.place_total += 1
        self.place_ok += 1 if ok else 0

    def grab_text(self):
        return f'GRAB {self.grab_ok}/{self.grab_total}'

    def place_text(self):
        return f'PLACE {self.place_ok}/{self.place_total}'
