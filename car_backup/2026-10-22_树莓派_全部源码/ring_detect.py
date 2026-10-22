import cv2
import numpy as np
import math
import time


# ============================================================
# 摄像头设置
# ============================================================

CAMERA_DEVICE = "/dev/video0"

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30


# ============================================================
# 圆环定位参数
# ============================================================

IMAGE_CENTER_X = FRAME_WIDTH // 2
IMAGE_CENTER_Y = FRAME_HEIGHT // 2

# 三圆环整体中心允许误差
CENTER_TOLERANCE_X = 25
CENTER_TOLERANCE_Y = 25

# 连续稳定多少帧才认为已经停好
STABLE_FRAMES = 8

# 两次微调之间最短时间
ADJUST_INTERVAL = 0.45


# ============================================================
# 两点距离
# ============================================================

def point_distance(p1, p2):

    return math.sqrt(
        (p1[0] - p2[0]) ** 2 +
        (p1[1] - p2[1]) ** 2
    )


# ============================================================
# 检测圆环
# ============================================================

def detect_rings(frame):

    gray = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2GRAY
    )

    blur = cv2.GaussianBlur(
        gray,
        (5, 5),
        1.2
    )

    edge = cv2.Canny(
        blur,
        50,
        150
    )

    contours, _ = cv2.findContours(
        edge,
        cv2.RETR_LIST,
        cv2.CHAIN_APPROX_NONE
    )

    circles = []

    # --------------------------------------------------------
    # 找圆形轮廓
    # --------------------------------------------------------

    for cnt in contours:

        area = cv2.contourArea(cnt)

        perimeter = cv2.arcLength(
            cnt,
            True
        )

        if perimeter == 0:
            continue

        circularity = (
            4 *
            np.pi *
            area /
            (perimeter * perimeter)
        )

        if area < 100:
            continue

        if circularity < 0.75:
            continue

        (x, y), r = cv2.minEnclosingCircle(cnt)

        if r < 10:
            continue

        circles.append(
            (
                x,
                y,
                r
            )
        )

    # --------------------------------------------------------
    # 合并同一个圆环产生的多个圆
    # --------------------------------------------------------

    groups = []

    for circle in circles:

        x, y, r = circle

        merged = False

        for group in groups:

            gx = np.mean(
                [c[0] for c in group]
            )

            gy = np.mean(
                [c[1] for c in group]
            )

            if point_distance(
                (x, y),
                (gx, gy)
            ) < 8:

                group.append(circle)
                merged = True
                break

        if not merged:

            groups.append(
                [circle]
            )

    # --------------------------------------------------------
    # 得到圆环
    # --------------------------------------------------------

    rings = []

    for group in groups:

        if len(group) < 2:
            continue

        x = np.mean(
            [c[0] for c in group]
        )

        y = np.mean(
            [c[1] for c in group]
        )

        r = np.mean(
            [c[2] for c in group]
        )

        rings.append(
            (
                int(x),
                int(y),
                int(r)
            )
        )

    # --------------------------------------------------------
    # 从左到右排序
    # --------------------------------------------------------

    rings.sort(
        key=lambda ring: ring[0]
    )

    # 最多三个
    if len(rings) > 3:

        rings = rings[:3]

    return rings


# ============================================================
# 打开摄像头
# ============================================================

def open_camera():

    cap = cv2.VideoCapture(
        CAMERA_DEVICE,
        cv2.CAP_V4L2
    )

    if not cap.isOpened():

        print()
        print(
            "[圆环识别] 摄像头打开失败"
        )

        return None

    cap.set(
        cv2.CAP_PROP_FOURCC,
        cv2.VideoWriter_fourcc(*"MJPG")
    )

    cap.set(
        cv2.CAP_PROP_FRAME_WIDTH,
        FRAME_WIDTH
    )

    cap.set(
        cv2.CAP_PROP_FRAME_HEIGHT,
        FRAME_HEIGHT
    )

    cap.set(
        cv2.CAP_PROP_FPS,
        FPS
    )

    cap.set(
        cv2.CAP_PROP_BUFFERSIZE,
        1
    )

    # ========================================================
    # 注意：
    # 这里绝对不使用 cv2.flip()
    #
    # 保持你之前已经确认正确的原始画面方向。
    # ========================================================

    print(
        "[圆环识别] 摄像头打开成功"
    )

    return cap


# ============================================================
# 关闭摄像头
# ============================================================

def close_camera(cap):

    if cap is not None:

        cap.release()

        cv2.destroyAllWindows()

        print(
            "[圆环识别] 摄像头已关闭"
        )


# ============================================================
# 计算三个圆环整体中心
# ============================================================

def calculate_center(rings):

    if len(rings) < 3:

        return None

    cx = int(
        sum(r[0] for r in rings) / 3
    )

    cy = int(
        sum(r[1] for r in rings) / 3
    )

    return cx, cy


# ============================================================
# 绘制调试画面
# ============================================================

def draw_ring_display(
    frame,
    rings,
    stable_count,
    status
):

    display = frame.copy()

    # --------------------------------------------------------
    # 摄像头画面中心
    # --------------------------------------------------------

    cv2.drawMarker(
        display,
        (
            IMAGE_CENTER_X,
            IMAGE_CENTER_Y
        ),
        (255, 0, 0),
        cv2.MARKER_CROSS,
        25,
        2
    )

    # --------------------------------------------------------
    # 画三个圆环
    # --------------------------------------------------------

    for i, ring in enumerate(rings):

        x, y, r = ring

        cv2.circle(
            display,
            (x, y),
            int(r),
            (0, 255, 0),
            2
        )

        cv2.circle(
            display,
            (x, y),
            5,
            (0, 0, 255),
            -1
        )

        cv2.putText(
            display,
            f"Ring{i + 1}",
            (x + 10, y - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (0, 255, 0),
            2
        )

        cv2.putText(
            display,
            f"({x},{y})",
            (x + 10, y + 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            1
        )

    # --------------------------------------------------------
    # 三圆环整体中心
    # --------------------------------------------------------

    if len(rings) >= 3:

        center = calculate_center(rings)

        if center is not None:

            cx, cy = center

            cv2.circle(
                display,
                (cx, cy),
                8,
                (255, 0, 255),
                -1
            )

            cv2.line(
                display,
                (
                    IMAGE_CENTER_X,
                    IMAGE_CENTER_Y
                ),
                (cx, cy),
                (255, 0, 255),
                2
            )

            cv2.putText(
                display,
                f"Center: ({cx},{cy})",
                (20, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (255, 255, 255),
                2
            )

    # --------------------------------------------------------
    # 状态信息
    # --------------------------------------------------------

    cv2.putText(
        display,
        f"Rings: {len(rings)}",
        (20, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )

    cv2.putText(
        display,
        f"Stable: {stable_count}/{STABLE_FRAMES}",
        (20, 95),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (255, 255, 255),
        2
    )

    cv2.putText(
        display,
        status,
        (20, 125),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.65,
        (0, 255, 255),
        2
    )

    return display


# ============================================================
# 持续识别三圆环 + 小车微调
# ============================================================

def find_rings(
    cap,
    timeout=0,
    show_window=True,
    require_alignment=True,
    adjust_callback=None
):

    print()
    print(
        "[圆环识别] 开始持续扫描三个圆环..."
    )

    if timeout <= 0:

        print(
            "[圆环识别] 无限等待模式"
        )

    start_time = time.time()

    stable_count = 0

    last_center = None

    last_adjust_time = 0.0

    while True:

        # ----------------------------------------------------
        # timeout
        #
        # timeout=0：
        # 永不因为时间到了而结束。
        # ----------------------------------------------------

        if (
            timeout > 0
            and
            time.time() - start_time >= timeout
        ):

            print(
                "[圆环识别] 达到设定时间，继续扫描..."
            )

            start_time = time.time()

        # ----------------------------------------------------
        # 读取摄像头
        # ----------------------------------------------------

        ret, frame = cap.read()

        if not ret:

            print(
                "[圆环识别] 摄像头读取失败"
            )

            time.sleep(0.02)

            continue

        # ----------------------------------------------------
        # 识别
        # ----------------------------------------------------

        rings = detect_rings(frame)

        # ====================================================
        # 情况1：
        # 还没有三个圆环
        # ====================================================

        if len(rings) < 3:

            stable_count = 0
            last_center = None

            status = (
                f"SEARCHING {len(rings)}/3"
            )

            # ------------------------------------------------
            # 通知小车：
            # 继续向圆环方向低速接近
            # ------------------------------------------------

            if adjust_callback is not None:

                now = time.time()

                if (
                    now - last_adjust_time
                    >= ADJUST_INTERVAL
                ):

                    adjust_callback(
                        0,
                        0,
                        mode="approach",
                        stop=False
                    )

                    last_adjust_time = now

        # ====================================================
        # 情况2：
        # 已经找到三个圆环
        # ====================================================

        else:

            center = calculate_center(rings)

            if center is None:
                continue

            cx, cy = center

            # ------------------------------------------------
            # 计算整体中心与画面中心的误差
            # ------------------------------------------------

            error_x = (
                cx -
                IMAGE_CENTER_X
            )

            error_y = (
                cy -
                IMAGE_CENTER_Y
            )

            aligned = (
                abs(error_x)
                <= CENTER_TOLERANCE_X
                and
                abs(error_y)
                <= CENTER_TOLERANCE_Y
            )

            # ------------------------------------------------
            # 判断圆环中心是否稳定
            # ------------------------------------------------

            if last_center is not None:

                center_move = point_distance(
                    center,
                    last_center
                )

                if center_move < 15:

                    stable_count += 1

                else:

                    stable_count = 0

            else:

                stable_count = 0

            last_center = center

            # ------------------------------------------------
            # 需要继续微调
            # ------------------------------------------------

            if (
                require_alignment
                and
                not aligned
            ):

                stable_count = 0

                now = time.time()

                if (
                    adjust_callback is not None
                    and
                    now - last_adjust_time
                    >= ADJUST_INTERVAL
                ):

                    adjust_callback(
                        error_x,
                        error_y,
                        mode="align",
                        stop=False
                    )

                    last_adjust_time = now

                status = (
                    f"ALIGN "
                    f"X={error_x:+d} "
                    f"Y={error_y:+d}"
                )

            else:

                status = (
                    f"READY "
                    f"X={error_x:+d} "
                    f"Y={error_y:+d}"
                )

            # ------------------------------------------------
            # 连续稳定后停车
            # ------------------------------------------------

            if (
                aligned
                and
                stable_count >= STABLE_FRAMES
            ):

                print()
                print(
                    "[圆环识别] ★ 三圆环稳定定位完成 ★"
                )

                print(
                    f"[圆环识别] 三环整体中心："
                    f"({cx},{cy})"
                )

                # ------------------------------------------------
                # 停车
                # ------------------------------------------------

                if adjust_callback is not None:

                    adjust_callback(
                        0,
                        0,
                        mode="stop",
                        stop=True
                    )

                if show_window:

                    display = draw_ring_display(
                        frame,
                        rings,
                        stable_count,
                        "STOP - CENTER OK"
                    )

                    cv2.imshow(
                        "Ring Detection",
                        display
                    )

                    cv2.waitKey(800)

                # ------------------------------------------------
                # 返回三个圆环坐标
                # ------------------------------------------------

                ring_positions = {}

                for i, ring in enumerate(
                    rings[:3]
                ):

                    ring_positions[
                        str(i + 1)
                    ] = (
                        ring[0],
                        ring[1]
                    )

                return ring_positions

        # ----------------------------------------------------
        # 显示
        # ----------------------------------------------------

        if show_window:

            display = draw_ring_display(
                frame,
                rings,
                stable_count,
                status
            )

            cv2.imshow(
                "Ring Detection",
                display
            )

        # ----------------------------------------------------
        # ESC退出
        # ----------------------------------------------------

        key = cv2.waitKey(1) & 0xFF

        if key == 27:

            print()
            print(
                "[圆环识别] 用户按 ESC 退出"
            )

            return None
