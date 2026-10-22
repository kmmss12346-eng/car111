#!/usr/bin/env python3
"""桌面全流程模拟(不用雷达、不用 STM32、不用屏幕)：go → 两次扫描 → 规划 → 确认 → 按路线逐条“发”给模拟 STM32。
用法：python3 sim_run.py            (需要 numpy、matplotlib；会在当前目录下生成 map_captures)"""
import io
import os
import sys
import threading
import time

os.environ['MPLBACKEND'] = 'Agg'
import matplotlib
matplotlib.use('Agg')
import matplotlib.backend_bases as bb
import matplotlib.pyplot as plt


def _timer_start(self, interval=None):
    if interval is not None:
        self.interval = interval
    self._run = True
    def loop():
        while getattr(self, '_run', False):
            for func, a, k in list(self.callbacks):
                try:
                    func(*a, **k)
                except Exception as e:
                    print('timer cb error', repr(e))
            time.sleep(max(self.interval, 20) / 1000.0)
    threading.Thread(target=loop, daemon=True).start()


bb.TimerBase.start = _timer_start
bb.TimerBase.stop = lambda self: setattr(self, '_run', False)
plt.show = lambda *a, **k: [time.sleep(0.2) for _ in iter(lambda: bool(plt.get_fignums()), False)]


class ScriptedStdin:
    """前 N 秒每 1.5 秒发一个 y(确认行驶)，第一行是 go，最后是 q。"""
    def __init__(self, run_seconds):
        self.t0 = time.monotonic()
        self.n = 0
        self.run_seconds = run_seconds

    def readline(self):
        self.n += 1
        if self.n == 1:
            return 'go\n'
        if time.monotonic() - self.t0 > self.run_seconds:
            return 'q\n' if self.n % 2 == 0 else ''
        time.sleep(1.5)
        return 'y\n'

    def __iter__(self):
        return iter(self.readline, '')


if __name__ == '__main__':
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 45
    sys.stdin = ScriptedStdin(secs)
    import builtins
    builtins.input = lambda *a: (lambda l: (_ for _ in ()).throw(EOFError()) if l == '' else l.rstrip('\n'))(sys.stdin.readline())
    sys.argv = [sys.argv[0], '--sim', '--stm-port', 'none', '--stop-wait', '0']
    import map_merge_live
    raise SystemExit(map_merge_live.main())
