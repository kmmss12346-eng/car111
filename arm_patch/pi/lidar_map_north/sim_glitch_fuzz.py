"""(10-23)整场仿真：某条 STM32 指令第 n 次出错一次(或夹紧以后才出错)，看会不会崩、撞、掉、夹空，能不能 6 个全抓、12 个全放。
python3 sim_glitch_fuzz.py          全场(QR->RAW->ROUGH->TEMP 两批->回家)
python3 sim_glitch_fuzz.py raw      只跑原料区"最近那格"(真车用的做法)
不是 6/6、12/12(或原料区 3 个没全进转盘)的那几种打出来；回归测试见 sim_mission.SingleGlitchTests。"""
import sys

from sim_mission import SimWorld, make, run_mission, RawNearSlotTests

VERBS = ('OBS', 'AP', 'APZ', 'LIFT', 'TAKE', 'CLAW', 'STOW', 'SIDE', 'GRAB', 'PICK', 'GET', 'PARK', 'PRE', 'A?', 'QR?', 'LIFT?', 'SCR')


def full():
    bad = n_runs = 0
    for verb in VERBS:
        for n in (1, 2, 3, 5):
            for extra in ((), ('closed',)) if verb in ('GRAB', 'PICK', 'TAKE') else ((),):
                for seed in (1, 2):
                    n_runs += 1
                    w = SimWorld(seed=seed, code='156+123+516+231', fail_cmd=(verb, n) + extra)
                    h = make(w, dict(round_s=1e9), log=lambda m: None)
                    try:
                        run_mission(w, h, log=lambda m: None, nav=dict(pre=True))
                        exc = None
                    except Exception as ex:
                        exc = repr(ex)[:200]
                    st = h.stats
                    fired = w.count.get(verb, 0) >= n
                    perfect = (not exc and (w.air, w.loose, w.collisions) == (0, 0, 0)
                               and (st.grab_ok, st.place_ok) == (12, 12) and not h.disabled)   # (10-28)抓取含粗加工区取回：6 + 6
                    if fired and not perfect:
                        bad += 1
                        print(f'{verb:6s} n={n} {"/".join(extra) or "-":6s} seed={seed} grab {st.grab_ok}/{st.grab_total} '
                              f'place {st.place_ok}/{st.place_total} air/loose/coll={w.air},{w.loose},{w.collisions} '
                              f'disabled={h.disabled!r} {exc or ""}', flush=True)
    print(f'全场：{n_runs} 种，不完美 {bad} 种')


def raw():
    t = RawNearSlotTests()
    bad = n_runs = 0
    for verb in ('GRAB', 'AP', 'LIFT', 'OBS', 'A?', 'APZ', 'CLAW'):
        for n in (3, 4, 5):                                  # 前两次 A?/AP 在示教里(示教不是比赛流程)
            for extra in ((), ('closed',)) if verb == 'GRAB' else ((),):
                for seed in (1, 2, 5):
                    n_runs += 1
                    try:
                        w, h, text = t._near_run(seed, fail_cmd=(verb, n) + extra)
                    except Exception as ex:
                        bad += 1
                        print(f'{verb} n={n} seed={seed} 出错 {ex!r}', flush=True)
                        continue
                    tray = sum(1 for v in w.tray.values() if v)
                    if w.count.get(verb, 0) >= n and (tray < 3 or w.loose or w.collisions):
                        bad += 1
                        print(f'{verb:5s} n={n} {"/".join(extra) or "-":6s} seed={seed} 转盘里 {tray} 个 loose={w.loose} coll={w.collisions}',
                              flush=True)
    print(f'原料区最近那格：{n_runs} 种，不完美 {bad} 种')


if __name__ == '__main__':
    raw() if sys.argv[1:2] == ['raw'] else full()
