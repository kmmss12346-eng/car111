"""实时看 map_merge_live 里摄像头正在看的画面(带识别标注)。rtest / gtest / mtest / vclaw / vcal / go 的时候用：
map_merge_live 不用退出，摄像头还是它在用，这里只显示它每次识别时写出来的画面。

在树莓派桌面(VNC)里另开一个终端运行：

    cd ~/lidar_map_north
    python3 vview.py

窗口里按键(先用鼠标点一下画面窗口)：
    s     存一张当前画面到 vview.png
    q     退出

画面上：
    绿十字      爪子点(物料/圆环中心要对到这里)
    黄圈        认到的圆环(每一圈都画，被爪子挡住的部分也画)；灰圈 = 认到了但大小不像圆环，被过滤掉了
    青色圆+红叉  认到的物料(拟合出来的整个圆)和圆心
    变暗的区域  爪子(识别时不用这块)
    左上角      离爪子点多少像素/毫米、时间；NOT FOUND = 这一帧没认到
map_merge_live 这会儿没在用摄像头时画面不动，下面会显示多少秒没更新。

(不开 map_merge_live、单独看识别效果用 vlive.py，它自己开摄像头。)
"""
import os
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent
DEFAULT_PATH = '/tmp/vision_live.jpg'          # 和 vision.py 里 Vision.DEFAULT['live_view'] 一样


def main(argv=None):
    import cv2
    import numpy as np
    argv = sys.argv[1:] if argv is None else argv
    path = argv[0] if argv else DEFAULT_PATH
    win = 'vview  (s save, q quit)'
    img, last_m, t_upd = None, None, None
    print(f'显示 {path}(map_merge_live 识别时写的画面)。按 s 存图，q 退出(先用鼠标点一下画面窗口)。', flush=True)
    while True:
        try:
            m = os.stat(path).st_mtime
        except OSError:
            m = None
        if m is not None and m != last_m:
            fr = cv2.imread(path)
            if fr is not None:
                img, last_m, t_upd = fr, m, time.monotonic()
        show = img.copy() if img is not None else np.zeros((480, 640, 3), np.uint8)
        if img is None:
            msg = 'waiting... run rtest / gtest / mtest / vclaw in map_merge_live'
        else:
            age = time.monotonic() - t_upd
            msg = f'no new frame for {age:.0f}s (camera not in use right now)' if age > 1.5 else ''
        if msg:
            h = show.shape[0]
            cv2.putText(show, msg, (8, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 4)
            cv2.putText(show, msg, (8, h - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 255), 1)
        try:
            cv2.imshow(win, show)
        except cv2.error:
            print('显示不了窗口：要在树莓派桌面(VNC)里打开的终端运行，不能用 ssh。', flush=True)
            return 1
        key = cv2.waitKey(50) & 0xFF
        if key == ord('q'):
            break
        if key == ord('s') and img is not None:
            cv2.imwrite(str(ROOT / 'vview.png'), img)
            print(f'已存：{ROOT / "vview.png"}', flush=True)
    cv2.destroyAllWindows()
    return 0


if __name__ == '__main__':
    sys.exit(main())
