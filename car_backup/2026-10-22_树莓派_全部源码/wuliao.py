import cv2
import numpy as np
import time
import math


# ============================================================
# Camera
# ============================================================

CAMERA_DEVICE = "/dev/video0"

FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FPS = 30

WINDOW_NAME = "Material Detect"


# ============================================================
# Detection parameters
# ============================================================

MIN_AREA = 500
MAX_AREA = 50000

MIN_WIDTH = 15
MIN_HEIGHT = 15

MAX_ASPECT_RATIO = 4.5

MIN_CIRCULARITY = 0.35

STABLE_FRAMES = 4
STABLE_RANGE = 3.0

LOST_TIMEOUT = 0.8

MIN_CONFIDENCE = 0.48
COLOR_MARGIN = 0.08


# ============================================================
# Camera center
# ============================================================

CAMERA_CENTER_X = FRAME_WIDTH / 2
CAMERA_CENTER_Y = FRAME_HEIGHT / 2


# ============================================================
# Color IDs
# ============================================================

COLOR_NAMES = {
    1: "RED",
    2: "YELLOW",
    3: "BLUE",
    4: "GREEN",
    5: "BLACK",
    6: "LIGHT_BLUE"
}


COLOR_TO_ID = {
    "RED": 1,
    "YELLOW": 2,
    "BLUE": 3,
    "GREEN": 4,
    "BLACK": 5,
    "LIGHT_BLUE": 6,

    "red": 1,
    "yellow": 2,
    "blue": 3,
    "green": 4,
    "black": 5,
    "light_blue": 6,
    "light blue": 6,

    "红": 1,
    "黄色": 2,
    "蓝": 3,
    "绿色": 4,
    "黑": 5,
    "浅蓝": 6,
}


# ============================================================
# HSV ranges
# ============================================================

COLOR_RANGES = {

    1: [
        ((0, 70, 40), (12, 255, 255)),
        ((168, 70, 40), (180, 255, 255))
    ],

    2: [
        ((15, 60, 50), (40, 255, 255))
    ],

    3: [
    ((85, 130, 40), (125, 255, 240))
    ],

    4: [
        ((38, 40, 30), (95, 255, 255))
    ],

    5: [
        ((0, 0, 0), (180, 255, 105))
    ],

    6: [
    ((85, 50, 160), (115, 180, 255))
    ]
}


# ============================================================
# Utility
# ============================================================

def clamp(value, low, high):
    return max(low, min(high, value))


def color_name(color):
    if isinstance(color, int):
        return COLOR_NAMES.get(color, "UNKNOWN")

    if isinstance(color, str):
        key = color.strip()

        if key in COLOR_TO_ID:
            return COLOR_NAMES[COLOR_TO_ID[key]]

        upper = key.upper()

        if upper in COLOR_TO_ID:
            return COLOR_NAMES[COLOR_TO_ID[upper]]

    return "UNKNOWN"


def color_id(color):

    if isinstance(color, (int, np.integer)):

        color = int(color)

        if color in COLOR_NAMES:
            return color

        return None

    if isinstance(color, str):

        key = color.strip()

        # "1" -> 1
        # "2" -> 2
        # "3" -> 3
        if key.isdigit():

            value = int(key)

            if value in COLOR_NAMES:
                return value

            return None

        if key in COLOR_TO_ID:
            return COLOR_TO_ID[key]

        upper = key.upper()

        if upper in COLOR_TO_ID:
            return COLOR_TO_ID[upper]

    return None

# ============================================================
# Camera
# ============================================================

def open_material_camera():

    print()
    print("========================================")
    print("Opening material camera")
    print("Device:", CAMERA_DEVICE)
    print("========================================")

    cap = cv2.VideoCapture(CAMERA_DEVICE, cv2.CAP_V4L2)

    if not cap.isOpened():
        print("ERROR: Cannot open material camera")
        return None

    cap.set(cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*"MJPG"))

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, FPS)

    try:
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    except Exception:
        pass

    time.sleep(0.5)

    valid_frames = 0

    for _ in range(10):

        ret, frame = cap.read()

        if ret and frame is not None:
            valid_frames += 1

        time.sleep(0.02)

    print("Camera warm-up frames:", valid_frames)

    if valid_frames == 0:

        print("ERROR: Camera opened but no valid frames")

        cap.release()

        return None

    print("Material camera ready")

    return cap


def close_material_camera(cap):

    if cap is not None:

        try:
            cap.release()
        except Exception:
            pass

    try:
        cv2.destroyWindow(WINDOW_NAME)
    except Exception:
        pass

    cv2.waitKey(1)

    print("Material camera closed")


# ============================================================
# Image preprocessing
# ============================================================

def preprocess(frame):

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)

    return hsv, lab


# ============================================================
# HSV mask
# ============================================================

def get_hsv_mask(hsv, color_id):

    ranges = COLOR_RANGES.get(color_id)

    if ranges is None:
        return np.zeros(hsv.shape[:2], dtype=np.uint8)

    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)

    for lower, upper in ranges:

        lower_np = np.array(lower, dtype=np.uint8)
        upper_np = np.array(upper, dtype=np.uint8)

        current = cv2.inRange(
            hsv,
            lower_np,
            upper_np
        )

        mask = cv2.bitwise_or(mask, current)

    return mask


# ============================================================
# Morphology
# ============================================================

def clean_mask(mask):

    kernel_small = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (5, 5)
    )

    kernel_large = cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE,
        (7, 7)
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel_small
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel_large
    )

    return mask


# ============================================================
# Blue / light blue scoring
# ============================================================

def calculate_blue_scores(frame, hsv, lab, mask):

    if cv2.countNonZero(mask) == 0:

        return 0.0, 0.0

    pixels_bgr = frame[mask > 0]

    pixels_hsv = hsv[mask > 0]

    pixels_lab = lab[mask > 0]

    if len(pixels_bgr) == 0:

        return 0.0, 0.0

    b = pixels_bgr[:, 0].astype(np.float32)
    g = pixels_bgr[:, 1].astype(np.float32)
    r = pixels_bgr[:, 2].astype(np.float32)

    h = pixels_hsv[:, 0].astype(np.float32)
    s = pixels_hsv[:, 1].astype(np.float32)
    v = pixels_hsv[:, 2].astype(np.float32)

    l_value = pixels_lab[:, 0].astype(np.float32)
    a_value = pixels_lab[:, 1].astype(np.float32)
    b_lab = pixels_lab[:, 2].astype(np.float32)

    # --------------------------------------------------------
    # RGB blue dominance
    # --------------------------------------------------------

    rgb_blue = b - (r + g) / 2.0

    blue_rgb_score = np.mean(
        np.clip((rgb_blue + 10.0) / 80.0, 0.0, 1.0)
    )

    # --------------------------------------------------------
    # HSV
    # --------------------------------------------------------

    hue_distance = np.abs(h - 110.0)

    hue_score = np.exp(
        -(hue_distance ** 2) / (2.0 * 22.0 ** 2)
    )

    sat_score = np.clip(
        (s - 20.0) / 180.0,
        0.0,
        1.0
    )

    blue_hsv_score = np.mean(
        hue_score * sat_score
    )

    # --------------------------------------------------------
    # Lab
    # --------------------------------------------------------

    blue_lab_strength = (
        b_lab - 128.0
    )

    blue_lab_score = np.mean(
        np.clip(
            (-blue_lab_strength + 8.0) / 45.0,
            0.0,
            1.0
        )
    )

    blue_score = (
        blue_hsv_score * 0.40 +
        blue_rgb_score * 0.25 +
        blue_lab_score * 0.35
    )

    # --------------------------------------------------------
    # Light blue
    # --------------------------------------------------------

    # Hue should be close to the measured light-blue hue
    light_blue_hue_score = np.exp(
        -((h - 99.0) ** 2) / (2.0 * 12.0 ** 2)
    )

    # Saturation should be around the measured value
    light_blue_sat_score = np.exp(
        -((s - 105.0) ** 2) / (2.0 * 45.0 ** 2)
    )

    # Light blue has high brightness
    light_blue_value_score = np.clip(
        (v - 170.0) / 70.0,
        0.0,
        1.0
    )

    # RGB blue dominance
    light_blue_rgb_score = np.clip(
        (b - r - 30.0) / 100.0,
        0.0,
        1.0
    )

    # Lab lightness
    light_blue_lab_lightness = np.clip(
        (l_value - 150.0) / 70.0,
        0.0,
        1.0
    )

    light_blue_score = (
        light_blue_hue_score * 0.20 +
        light_blue_sat_score * 0.30 +
        light_blue_value_score * 0.20 +
        light_blue_rgb_score * 0.15 +
        light_blue_lab_lightness * 0.15
    )

    return (
        float(blue_score),
        float(light_blue_score)
    )


# ============================================================
# Normal color score
# ============================================================

def calculate_normal_color_score(hsv, mask, target_id):

    pixels = hsv[mask > 0]

    if len(pixels) == 0:
        return 0.0

    h = pixels[:, 0].astype(np.float32)
    s = pixels[:, 1].astype(np.float32)
    v = pixels[:, 2].astype(np.float32)

    if target_id == 1:

        red1 = (
            ((h >= 0) & (h <= 12)) |
            ((h >= 168) & (h <= 180))
        )

        sat_ok = s >= 70
        value_ok = v >= 40

        score = np.mean(
            red1 & sat_ok & value_ok
        )

    elif target_id == 2:

        hue_ok = (
            (h >= 15) &
            (h <= 40)
        )

        sat_ok = s >= 60
        value_ok = v >= 50

        score = np.mean(
            hue_ok & sat_ok & value_ok
        )

    elif target_id == 4:

        hue_ok = (
            (h >= 38) &
            (h <= 95)
        )

        sat_ok = s >= 40
        value_ok = v >= 30

        score = np.mean(
            hue_ok & sat_ok & value_ok
        )

    elif target_id == 5:

        value_ok = v <= 105

        score = np.mean(value_ok)

    else:

        score = 0.0

    return float(score)


# ============================================================
# Target confidence
# ============================================================

def calculate_confidence(
    frame,
    hsv,
    lab,
    mask,
    target_id
):

    pixel_count = cv2.countNonZero(mask)

    if pixel_count < MIN_AREA:
        return 0.0

    if target_id in (3, 6):

        blue_score, light_blue_score = \
            calculate_blue_scores(
                frame,
                hsv,
                lab,
                mask
            )

        if target_id == 3:

            return blue_score

        else:

            return light_blue_score

    return calculate_normal_color_score(
        hsv,
        mask,
        target_id
    )


# ============================================================
# Contour circularity
# ============================================================

def calculate_circularity(contour):

    area = cv2.contourArea(contour)

    perimeter = cv2.arcLength(
        contour,
        True
    )

    if perimeter <= 0:
        return 0.0

    circularity = (
        4.0 *
        math.pi *
        area /
        (perimeter * perimeter)
    )

    return float(circularity)


# ============================================================
# Candidate detection
# ============================================================

def find_candidates(mask):

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )

    candidates = []

    for contour in contours:

        area = cv2.contourArea(contour)

        if area < MIN_AREA:
            continue

        if area > MAX_AREA:
            continue

        x, y, w, h = cv2.boundingRect(contour)

        if w < MIN_WIDTH or h < MIN_HEIGHT:
            continue

        aspect_ratio = max(w, h) / max(
            min(w, h),
            1
        )

        if aspect_ratio > MAX_ASPECT_RATIO:
            continue

        circularity = calculate_circularity(contour)

        if circularity < MIN_CIRCULARITY:
            continue

        # Reject contours touching the image boundary
        if (
            x <= 2 or
            y <= 2 or
            x + w >= FRAME_WIDTH - 2 or
            y + h >= FRAME_HEIGHT - 2
        ):
            continue

        candidates.append({
            "contour": contour,
            "area": area,
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "circularity": circularity
        })

    candidates.sort(
        key=lambda item: item["area"],
        reverse=True
    )

    return candidates


# ============================================================
# Top circle center
# ============================================================

def detect_top_center(frame, contour):

    x, y, w, h = cv2.boundingRect(contour)

    pad = 8

    x1 = max(0, x - pad)
    y1 = max(0, y - pad)

    x2 = min(
        frame.shape[1],
        x + w + pad
    )

    y2 = min(
        frame.shape[0],
        y + h + pad
    )

    roi = frame[y1:y2, x1:x2]

    if roi.size == 0:
        return None

    gray = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2GRAY
    )

    gray = cv2.GaussianBlur(
        gray,
        (5, 5),
        0
    )

    edges = cv2.Canny(
        gray,
        50,
        150
    )

    contours, _ = cv2.findContours(
        edges,
        cv2.RETR_LIST,
        cv2.CHAIN_APPROX_SIMPLE
    )

    best_center = None
    best_score = -1

    for c in contours:

        area = cv2.contourArea(c)

        if area < 20:
            continue

        if len(c) < 5:
            continue

        try:

            ellipse = cv2.fitEllipse(c)

        except cv2.error:
            continue

        (cx, cy), (major, minor), angle = ellipse

        if major <= 0 or minor <= 0:
            continue

        ratio = min(
            major,
            minor
        ) / max(
            major,
            minor
        )

        if ratio < 0.55:
            continue

        radius = (major + minor) / 4.0

        if radius < 3:
            continue

        # Prefer circles near the upper part
        # of the material.
        upper_score = 1.0 - (
            cy / max(roi.shape[0], 1)
        )

        circular_score = ratio

        score = (
            circular_score * 0.70 +
            upper_score * 0.30
        )

        if score > best_score:

            best_score = score

            best_center = (
                int(cx + x1),
                int(cy + y1)
            )

    if best_center is not None:
        return best_center

    # Fallback: contour center
    moments = cv2.moments(contour)

    if moments["m00"] != 0:

        cx = int(
            moments["m10"] /
            moments["m00"]
        )

        cy = int(
            moments["m01"] /
            moments["m00"]
        )

        return cx, cy

    return None


# ============================================================
# Find best target
# ============================================================

def find_best_target(
    frame,
    target_id
):

    hsv, lab = preprocess(frame)

    mask = get_hsv_mask(
        hsv,
        target_id
    )

    mask = clean_mask(mask)

    candidates = find_candidates(mask)

    if not candidates:

        return None, mask

    best = None
    best_score = -1

    for candidate in candidates:

        contour = candidate["contour"]

        single_mask = np.zeros(
            mask.shape,
            dtype=np.uint8
        )

        cv2.drawContours(
            single_mask,
            [contour],
            -1,
            255,
            thickness=-1
        )

        confidence = calculate_confidence(
            frame,
            hsv,
            lab,
            single_mask,
            target_id
        )

        if confidence < MIN_CONFIDENCE:
            continue

        area = candidate["area"]

        area_score = clamp(
            area / 8000.0,
            0.0,
            1.0
        )

        circularity = candidate["circularity"]

        final_score = (
            confidence * 0.60 +
            area_score * 0.20 +
            circularity * 0.20
        )

        if final_score > best_score:

            center = detect_top_center(
                frame,
                contour
            )

            if center is None:
                continue

            best_score = final_score

            best = {
                "id": target_id,
                "color": COLOR_NAMES[target_id],
                "center": center,
                "confidence": float(confidence),
                "score": float(final_score),
                "area": float(area),
                "bbox": (
                    candidate["x"],
                    candidate["y"],
                    candidate["w"],
                    candidate["h"]
                ),
                "circularity": float(circularity)
            }

    return best, mask


# ============================================================
# Stable tracker
# ============================================================

class StableTracker:

    def __init__(
        self,
        required_frames=STABLE_FRAMES,
        max_range=STABLE_RANGE
    ):

        self.required_frames = required_frames

        self.max_range = max_range

        self.points = []

        self.last_time = None

    def reset(self):

        self.points = []

        self.last_time = None

    def update(self, point):

        now = time.time()

        if point is None:

            self.points = []

            self.last_time = now

            return False, None

        if self.last_time is not None:

            if now - self.last_time > LOST_TIMEOUT:

                self.points = []

        self.last_time = now

        x, y = point

        self.points.append(
            (float(x), float(y))
        )

        if len(self.points) > self.required_frames:

            self.points.pop(0)

        if len(self.points) < self.required_frames:

            return False, None

        xs = [
            p[0]
            for p in self.points
        ]

        ys = [
            p[1]
            for p in self.points
        ]

        if (
            max(xs) - min(xs) > self.max_range
            or
            max(ys) - min(ys) > self.max_range
        ):

            return False, None

        center_x = int(
            round(
                sum(xs) / len(xs)
            )
        )

        center_y = int(
            round(
                sum(ys) / len(ys)
            )
        )

        return True, (
            center_x,
            center_y
        )


# ============================================================
# Draw
# ============================================================

def draw_result(
    frame,
    target_id,
    result,
    stable,
    stable_center,
    fps
):

    display = frame.copy()

    cx = int(CAMERA_CENTER_X)
    cy = int(CAMERA_CENTER_Y)

    # Camera center
    cv2.drawMarker(
        display,
        (cx, cy),
        (255, 255, 255),
        cv2.MARKER_CROSS,
        30,
        2
    )

    cv2.circle(
        display,
        (cx, cy),
        5,
        (255, 255, 255),
        -1
    )

    target_name = COLOR_NAMES[target_id]

    cv2.putText(
        display,
        "TARGET: " + target_name,
        (15, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (255, 255, 255),
        2
    )

    if result is not None:

        tx, ty = result["center"]

        x, y, w, h = result["bbox"]

        cv2.rectangle(
            display,
            (x, y),
            (x + w, y + h),
            (0, 255, 0),
            2
        )

        cv2.circle(
            display,
            (tx, ty),
            8,
            (0, 0, 255),
            -1
        )

        cv2.drawMarker(
            display,
            (tx, ty),
            (0, 255, 255),
            cv2.MARKER_CROSS,
            24,
            2
        )

        text = (
            "%s  %.2f"
            % (
                result["color"],
                result["confidence"]
            )
        )

        cv2.putText(
            display,
            text,
            (x, max(60, y - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

    if stable:

        cv2.putText(
            display,
            "STABLE - GRAB",
            (15, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        if stable_center is not None:

            cv2.circle(
                display,
                stable_center,
                12,
                (0, 255, 0),
                3
            )

    else:

        cv2.putText(
            display,
            "SEARCHING",
            (15, 65),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2
        )

    cv2.putText(
        display,
        "FPS: %.1f" % fps,
        (15, FRAME_HEIGHT - 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1
    )

    return display


# ============================================================
# Main material detection interface
# ============================================================

def find_material(
    cap,
    target_color,
    timeout=30,
    show_window=True
):

    target_id = color_id(target_color)

    if target_id is None:

        print(
            "ERROR: Unknown target color:",
            target_color
        )

        return None

    target_name = COLOR_NAMES[target_id]

    print()
    print("----------------------------------------")
    print("Looking for:", target_name)
    print("Target ID:", target_id)
    print("----------------------------------------")

    if cap is None:

        print("ERROR: Camera object is None")

        return None

    if not cap.isOpened():

        print("ERROR: Camera is not opened")

        return None

    if show_window:

        try:

            cv2.namedWindow(
                WINDOW_NAME,
                cv2.WINDOW_NORMAL
            )

            cv2.resizeWindow(
                WINDOW_NAME,
                FRAME_WIDTH,
                FRAME_HEIGHT
            )

        except cv2.error as e:

            print(
                "WARNING: Cannot create display window:",
                e
            )

    tracker = StableTracker()

    start_time = time.time()

    last_frame_time = time.time()

    fps = 0.0

    frame_count = 0

    last_result = None

    while True:

        elapsed = time.time() - start_time

        if elapsed >= timeout:

            print()
            print(
                "Material detection timeout:",
                target_name
            )

            if show_window:

                try:
                    cv2.destroyWindow(WINDOW_NAME)
                except Exception:
                    pass

            return None

        ret, frame = cap.read()

        if not ret or frame is None:

            print(
                "WARNING: Failed to read camera frame"
            )

            if show_window:

                try:

                    blank = np.zeros(
                        (
                            FRAME_HEIGHT,
                            FRAME_WIDTH,
                            3
                        ),
                        dtype=np.uint8
                    )

                    cv2.putText(
                        blank,
                        "CAMERA FRAME ERROR",
                        (120, 240),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 0, 255),
                        2
                    )

                    cv2.imshow(
                        WINDOW_NAME,
                        blank
                    )

                    key = cv2.waitKey(1)

                    if key == ord("q"):

                        return None

                except cv2.error:
                    pass

            time.sleep(0.03)

            continue

        frame_count += 1

        now = time.time()

        dt = now - last_frame_time

        if dt > 0:

            instant_fps = 1.0 / dt

            fps = (
                fps * 0.85 +
                instant_fps * 0.15
            )

        last_frame_time = now

        # ----------------------------------------------------
        # Detection
        # ----------------------------------------------------

        result, mask = find_best_target(
            frame,
            target_id
        )

        if result is not None:

            last_result = result

            stable, stable_center = tracker.update(
                result["center"]
            )

        else:

            stable, stable_center = tracker.update(
                None
            )

        # ----------------------------------------------------
        # Display
        # ----------------------------------------------------

        if show_window:

            display = draw_result(
                frame,
                target_id,
                result,
                stable,
                stable_center,
                fps
            )

            try:

                cv2.imshow(
                    WINDOW_NAME,
                    display
                )

                key = cv2.waitKey(1) & 0xFF

                if key == ord("q"):

                    print(
                        "User stopped material detection"
                    )

                    try:
                        cv2.destroyWindow(
                            WINDOW_NAME
                        )
                    except Exception:
                        pass

                    return None

            except cv2.error as e:

                if frame_count == 1:

                    print(
                        "WARNING: OpenCV display error:",
                        e
                    )

        # ----------------------------------------------------
        # Stable target
        # ----------------------------------------------------

        if stable:

            final_result = dict(last_result)

            final_result["center"] = stable_center

            print()
            print("========================================")
            print("MATERIAL FOUND")
            print("Color:", final_result["color"])
            print("ID:", final_result["id"])
            print("Center:", final_result["center"])
            print(
                "Confidence: %.3f"
                % final_result["confidence"]
            )
            print(
                "Area: %.1f"
                % final_result["area"]
            )
            print("========================================")

            # Hold image for a short time
            # so the user can see the successful detection.
            if show_window:

                hold_start = time.time()

                while time.time() - hold_start < 0.8:

                    try:

                        cv2.imshow(
                            WINDOW_NAME,
                            display
                        )

                        key = cv2.waitKey(1) & 0xFF

                        if key == ord("q"):
                            break

                    except cv2.error:
                        break

            if show_window:

                try:
                    cv2.destroyWindow(
                        WINDOW_NAME
                    )
                except Exception:
                    pass

                cv2.waitKey(1)

            return final_result


# ============================================================
# Simple camera preview test
# ============================================================

def camera_preview():

    cap = open_material_camera()

    if cap is None:
        return

    try:

        cv2.namedWindow(
            WINDOW_NAME,
            cv2.WINDOW_NORMAL
        )

        cv2.resizeWindow(
            WINDOW_NAME,
            FRAME_WIDTH,
            FRAME_HEIGHT
        )

    except cv2.error:
        pass

    print()
    print("Camera preview started")
    print("Press Q to exit")
    print()

    while True:

        ret, frame = cap.read()

        if not ret or frame is None:
            continue

        try:

            cv2.putText(
                frame,
                "CAMERA PREVIEW - PRESS Q",
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (255, 255, 255),
                2
            )

            cv2.drawMarker(
                frame,
                (
                    int(CAMERA_CENTER_X),
                    int(CAMERA_CENTER_Y)
                ),
                (255, 255, 255),
                cv2.MARKER_CROSS,
                30,
                2
            )

            cv2.imshow(
                WINDOW_NAME,
                frame
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

        except cv2.error:
            break

    close_material_camera(cap)


# ============================================================
# Test
# ============================================================

if __name__ == "__main__":

    camera_preview()
