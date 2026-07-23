# MaixCam vision module.
# Mode 0: idle. Mode 1: YOLO materials. Mode 2: YOLO green ring with HSV refinement.

from maix import camera, display, image, uart, pinmap, app, time, err, nn, gpio
import cv2
import numpy as np
import gc
import json
import os


# =========================
# UART0 and fill light
# =========================
UART_DEVICE = "/dev/ttyS0"
UART_BAUDRATE = 9600
UART_PINS = {"A16": "UART0_TX", "A17": "UART0_RX"}

for pin, func in UART_PINS.items():
    err.check_raise(pinmap.set_pin_function(pin, func), "Failed set pin {} to {}".format(pin, func))

serial = uart.UART(UART_DEVICE, UART_BAUDRATE)

fill_light = None
try:
    err.check_raise(pinmap.set_pin_function("B3", "GPIOB3"), "Failed set fill light pin")
    fill_light = gpio.GPIO("GPIOB3", gpio.Mode.OUT)
    fill_light.value(0)
except Exception as e:
    print("Fill light LED init failed: {}".format(e))


# =========================
# Runtime config
# =========================
IMG_W = 640
IMG_H = 480
CAM_FPS = 30
CAM_BUFF_NUM = 1
SHOW_IMAGE = True
DISPLAY_EVERY_N = 3
PRINT_FPS = True
GC_EVERY_N = 80

REPORT_INTERVAL_MS = 60
SERIAL_COORD_MAX = 240
FILTER_Q = 4.0
FILTER_R = 36.0
FILTER_MISS_LIMIT = 5

MODE_IDLE = 0
MODE_MATERIAL = 1
MODE_RING = 2
MODE_NAMES = {
    MODE_IDLE: "Idle",
    MODE_MATERIAL: "YOLO Material",
    MODE_RING: "YOLO+HSV Green Ring",
}

TARGET_NAMES = ["RED", "GREEN", "BLUE"]
TARGET_COLORS = [image.COLOR_RED, image.COLOR_GREEN, image.COLOR_BLUE]
MATERIAL_CLASS_MAP = {1: 1, 2: 3, 3: 2}
RING_CLASS_MAP = {1: 2, 2: 1, 3: 3}
RING_TARGET_CLASS = 2

FRAME_HEADER = 0x55
FRAME_MULTI_TARGET = 0x5B
FRAME_TAIL = 0xAA


# =========================
# Ring thresholds from color_line_det.py
# =========================
HSV_CONFIG_PATH = "/root/hsv_thresholds.json"
DEFAULT_HSV_THRESHOLDS = {
    "red1": [0, 10, 100, 255, 100, 255],
    "red2": [160, 180, 100, 255, 100, 255],
    "green": [22, 86, 29, 255, 59, 238],
    "blue": [92, 122, 71, 255, 88, 255],
}


def valid_hsv_values(values):
    if not isinstance(values, list) or len(values) != 6:
        return False
    try:
        values = [int(value) for value in values]
    except (TypeError, ValueError):
        return False
    return (
        0 <= values[0] <= values[1] <= 180
        and 0 <= values[2] <= values[3] <= 255
        and 0 <= values[4] <= values[5] <= 255
    )


def load_hsv_thresholds():
    thresholds = {name: values.copy() for name, values in DEFAULT_HSV_THRESHOLDS.items()}
    if not os.path.exists(HSV_CONFIG_PATH):
        print("HSV config not found, using defaults")
        return thresholds

    try:
        with open(HSV_CONFIG_PATH, "r") as file:
            saved = json.load(file)
        for name in thresholds:
            values = saved.get(name)
            if valid_hsv_values(values):
                thresholds[name] = [int(value) for value in values]
            else:
                print("Invalid HSV preset {}, using default".format(name))
        print("Loaded HSV thresholds: {}".format(thresholds))
    except Exception as exc:
        print("Load HSV config failed, using defaults: {}".format(exc))
    return thresholds


def hsv_bounds(values):
    lower = np.array([values[0], values[2], values[4]], dtype=np.uint8)
    upper = np.array([values[1], values[3], values[5]], dtype=np.uint8)
    return lower, upper


HSV_THRESHOLDS = load_hsv_thresholds()
RED_LOWER_1, RED_UPPER_1 = hsv_bounds(HSV_THRESHOLDS["red1"])
RED_LOWER_2, RED_UPPER_2 = hsv_bounds(HSV_THRESHOLDS["red2"])
GREEN_RING_LOWER, GREEN_RING_UPPER = hsv_bounds(HSV_THRESHOLDS["green"])
BLUE_RING_LOWER, BLUE_RING_UPPER = hsv_bounds(HSV_THRESHOLDS["blue"])

RING_KERNEL = np.ones((3, 3), dtype=np.uint8)
RING_MIN_CIRCULARITY = 0.3
RING_REFERENCE_AREA = 800 * 600
RING_IMAGE_AREA = IMG_W * IMG_H
RING_MIN_AREA = 800 * RING_IMAGE_AREA / RING_REFERENCE_AREA
RING_MAX_AREA = 90000 * RING_IMAGE_AREA / RING_REFERENCE_AREA
RING_ROI_PADDING = 0.20


# =========================
# Camera, display, and model
# =========================
cam = camera.Camera(IMG_W, IMG_H, fps=CAM_FPS, buff_num=CAM_BUFF_NUM)
cam.skip_frames(20)

try:
    cam.awb_mode(camera.AwbMode.Manual)
    cam.set_wb_gain([0.134, 0.0625, 0.0625, 0.1239])
except Exception:
    pass

try:
    cam.saturation(60)
except Exception:
    pass

disp = display.Display() if SHOW_IMAGE else None

material_detector = None
try:
    material_detector = nn.YOLOv5(model="/root/models/maixhub/Champion_material2/model_293122.mud")
    print("Material model loaded: {}x{}".format(
        material_detector.input_width(), material_detector.input_height()
    ))
except Exception as e:
    print("Material model load failed: {}".format(e))

ring_detector = None
try:
    ring_detector = nn.YOLOv5(model="/root/models/maixhub/Champion_ring2/model_293138.mud")
    print("Ring model loaded: {}x{}".format(
        ring_detector.input_width(), ring_detector.input_height()
    ))
except Exception as e:
    print("Ring model load failed: {}".format(e))


# =========================
# UART protocol
# =========================
def clamp_byte(value):
    return max(0, min(255, int(value)))


def scale_coord(x, y):
    sx = clamp_byte(x * SERIAL_COORD_MAX / IMG_W)
    sy = clamp_byte(y * SERIAL_COORD_MAX / IMG_H)
    return sx, sy


def set_fill_light(enabled):
    if fill_light is not None:
        fill_light.value(1 if enabled else 0)


def read_mode_command(current_mode):
    data = serial.read()
    if not data:
        return current_mode
    if isinstance(data, int):
        data = [data]

    new_mode = current_mode
    for value in data:
        if 0x30 <= value <= 0x32:
            new_mode = value - 0x30
        elif value == ord("L"):
            set_fill_light(True)
        elif value == ord("l"):
            set_fill_light(False)
    return new_mode


def send_multi_targets(mode, targets):
    payload = [FRAME_HEADER, FRAME_MULTI_TARGET, mode, len(targets)]
    for target in targets:
        payload.extend([target["cls"], target["sx"], target["sy"]])
    payload.append(FRAME_TAIL)
    serial.write(bytes(payload))


# =========================
# Target filtering
# =========================
def new_target_filter():
    return {
        "valid": False,
        "miss": 0,
        "x": 0.0,
        "y": 0.0,
        "px": FILTER_R,
        "py": FILTER_R,
        "last_target": None,
    }


material_filters = {cls: new_target_filter() for cls in range(1, 4)}
ring_filters = {cls: new_target_filter() for cls in range(1, 4)}


def reset_target_filters():
    for filters in (material_filters, ring_filters):
        for cls in filters:
            filters[cls] = new_target_filter()


def filter_targets(targets, filters):
    detected_by_cls = {target["cls"]: target for target in targets}
    filtered_targets = []

    for cls in range(1, 4):
        state = filters[cls]
        target = detected_by_cls.get(cls)
        if target is not None:
            mx = target["cx"]
            my = target["cy"]
            if not state["valid"]:
                state["x"] = mx
                state["y"] = my
                state["px"] = FILTER_R
                state["py"] = FILTER_R
                state["valid"] = True
            else:
                state["px"] += FILTER_Q
                state["py"] += FILTER_Q
                kx = state["px"] / (state["px"] + FILTER_R)
                ky = state["py"] / (state["py"] + FILTER_R)
                state["x"] += kx * (mx - state["x"])
                state["y"] += ky * (my - state["y"])
                state["px"] *= 1.0 - kx
                state["py"] *= 1.0 - ky

            state["miss"] = 0
            target["cx"] = int(state["x"])
            target["cy"] = int(state["y"])
            target["sx"], target["sy"] = scale_coord(target["cx"], target["cy"])
            state["last_target"] = target.copy()
            filtered_targets.append(target)
        elif state["valid"]:
            state["miss"] += 1
            if state["miss"] < FILTER_MISS_LIMIT:
                filtered_targets.append(state["last_target"].copy())
            else:
                state["valid"] = False
                state["last_target"] = None

    return filtered_targets


# =========================
# Material YOLO detection
# =========================
def detect_materials(img):
    if material_detector is None:
        return []

    model_w = material_detector.input_width()
    model_h = material_detector.input_height()
    img_small = img.resize(model_w, model_h)
    objs = material_detector.detect(img_small, conf_th=0.5, iou_th=0.45)
    scale_x = IMG_W / model_w
    scale_y = IMG_H / model_h
    best_by_cls = {}

    for obj in objs:
        raw_cls = int(obj.class_id) + 1
        cls = MATERIAL_CLASS_MAP.get(raw_cls, raw_cls)
        if cls < 1 or cls > 3:
            continue

        box_x = obj.x * scale_x
        box_y = obj.y * scale_y
        box_w = obj.w * scale_x
        box_h = obj.h * scale_y
        cx = box_x + box_w / 2.0
        cy = box_y + box_h / 2.0
        old = best_by_cls.get(cls)
        if old is not None and obj.score <= old["score"]:
            continue

        sx, sy = scale_coord(cx, cy)
        best_by_cls[cls] = {
            "cls": cls,
            "x": int(box_x),
            "y": int(box_y),
            "w": int(box_w),
            "h": int(box_h),
            "cx": int(cx),
            "cy": int(cy),
            "sx": sx,
            "sy": sy,
            "score": obj.score,
        }

    return [best_by_cls[cls] for cls in sorted(best_by_cls)]


# =========================
# Ring YOLO detection and HSV refinement
# =========================
def process_ring_mask(mask, dilate_iterations):
    mask = cv2.erode(mask, RING_KERNEL, iterations=1)
    return cv2.dilate(mask, RING_KERNEL, iterations=dilate_iterations)


def make_green_ring_mask(hsv):
    return process_ring_mask(cv2.inRange(hsv, GREEN_RING_LOWER, GREEN_RING_UPPER), 1)


def refine_ring_target(frame_rgb, target):
    padding_x = int(target["w"] * RING_ROI_PADDING)
    padding_y = int(target["h"] * RING_ROI_PADDING)
    roi_x0 = max(0, target["x"] - padding_x)
    roi_y0 = max(0, target["y"] - padding_y)
    roi_x1 = min(IMG_W, target["x"] + target["w"] + padding_x)
    roi_y1 = min(IMG_H, target["y"] + target["h"] + padding_y)
    if roi_x1 <= roi_x0 or roi_y1 <= roi_y0:
        return target

    roi_rgb = frame_rgb[roi_y0:roi_y1, roi_x0:roi_x1]
    hsv = cv2.cvtColor(roi_rgb, cv2.COLOR_RGB2HSV)
    mask = make_green_ring_mask(hsv)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best_contour = None
    best_area = 0.0

    for contour in contours:
        area = cv2.contourArea(contour)
        if area <= RING_MIN_AREA or area >= RING_MAX_AREA:
            continue
        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        circularity = 4.0 * np.pi * area / (perimeter * perimeter)
        if circularity <= RING_MIN_CIRCULARITY:
            continue

        moments = cv2.moments(contour)
        if moments["m00"] == 0:
            continue
        if area > best_area:
            best_contour = contour
            best_area = area

    if best_contour is None:
        return target

    moments = cv2.moments(best_contour)
    cx = int(roi_x0 + moments["m10"] / moments["m00"])
    cy = int(roi_y0 + moments["m01"] / moments["m00"])
    x, y, w, h = cv2.boundingRect(best_contour)
    _, radius = cv2.minEnclosingCircle(best_contour)
    sx, sy = scale_coord(cx, cy)
    target.update({
        "x": roi_x0 + x,
        "y": roi_y0 + y,
        "w": w,
        "h": h,
        "r": int(radius),
        "cx": cx,
        "cy": cy,
        "sx": sx,
        "sy": sy,
        "refined": True,
    })
    return target


def detect_rings(img):
    if ring_detector is None:
        return []

    model_w = ring_detector.input_width()
    model_h = ring_detector.input_height()
    img_small = img.resize(model_w, model_h)
    objs = ring_detector.detect(img_small, conf_th=0.5, iou_th=0.45)
    scale_x = IMG_W / model_w
    scale_y = IMG_H / model_h
    best_by_cls = {}

    for obj in objs:
        raw_cls = int(obj.class_id) + 1
        cls = RING_CLASS_MAP.get(raw_cls, raw_cls)
        if cls != RING_TARGET_CLASS:
            continue

        x = int(obj.x * scale_x)
        y = int(obj.y * scale_y)
        w = int(obj.w * scale_x)
        h = int(obj.h * scale_y)
        cx = x + w // 2
        cy = y + h // 2
        old = best_by_cls.get(cls)
        if old is not None and obj.score <= old["score"]:
            continue
        sx, sy = scale_coord(cx, cy)
        best_by_cls[cls] = {
            "cls": cls,
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "cx": cx,
            "cy": cy,
            "sx": sx,
            "sy": sy,
            "score": obj.score,
            "refined": False,
        }

    targets = [best_by_cls[cls] for cls in sorted(best_by_cls)]
    if not targets:
        return targets

    frame_rgb = image.image2cv(img, ensure_bgr=False, copy=False)
    return [refine_ring_target(frame_rgb, target) for target in targets]


# =========================
# Drawing
# =========================
def draw_targets(img, targets, title):
    img.draw_string(2, 2, title, image.COLOR_WHITE)
    for target in targets:
        color = TARGET_COLORS[target["cls"] - 1]
        name = TARGET_NAMES[target["cls"] - 1]
        if "r" in target:
            img.draw_circle(target["cx"], target["cy"], target["r"], color, 3)
            img.draw_circle(target["cx"], target["cy"], 5, image.COLOR_BLACK, -1)
            label_x = max(0, target["cx"] - target["r"])
            label_y = max(0, target["cy"] - target["r"] - 18)
        else:
            img.draw_rect(target["x"], target["y"], target["w"], target["h"], color, 3)
            img.draw_line(target["cx"] - 8, target["cy"], target["cx"] + 8, target["cy"], color, 2)
            img.draw_line(target["cx"], target["cy"] - 8, target["cx"], target["cy"] + 8, color, 2)
            label_x = target["x"]
            label_y = max(0, target["y"] - 18)
        img.draw_string(
            label_x,
            label_y,
            "{} ({},{})".format(name, target["sx"], target["sy"]),
            color,
        )


def draw_runtime_status(img, runtime_fps):
    img.draw_string(2, 22, "FPS:{}".format(runtime_fps), image.COLOR_WHITE)


# =========================
# Main loop
# =========================
mode_command = MODE_RING
last_mode = None
frame_id = 0
fps_count = 0
fps_t0 = time.ticks_ms()
runtime_fps = 0
last_report_ms = 0

while not app.need_exit():
    frame_id += 1
    fps_count += 1
    now = time.ticks_ms()

    mode_command = read_mode_command(mode_command)
    if mode_command != last_mode:
        set_fill_light(mode_command in (MODE_MATERIAL, MODE_RING))
        reset_target_filters()
        last_mode = mode_command
        print("Mode changed to {}".format(mode_command))

    img = cam.read()
    if mode_command == MODE_IDLE:
        targets = []
        serial_targets = []
    elif mode_command == MODE_MATERIAL:
        targets = detect_materials(img)
        serial_targets = filter_targets(targets, material_filters)
    elif mode_command == MODE_RING:
        targets = detect_rings(img)
        serial_targets = filter_targets(targets, ring_filters)
    else:
        mode_command = MODE_IDLE
        targets = []
        serial_targets = []

    if mode_command != MODE_IDLE and now - last_report_ms >= REPORT_INTERVAL_MS:
        send_multi_targets(mode_command, serial_targets)
        last_report_ms = now

    if SHOW_IMAGE and frame_id % DISPLAY_EVERY_N == 0:
        draw_targets(img, targets, "Mode{}:{}".format(mode_command, MODE_NAMES[mode_command]))
        draw_runtime_status(img, runtime_fps)
        if mode_command != MODE_IDLE and not targets:
            img.draw_string(2, 42, "No Target", image.COLOR_RED)
        if disp is not None:
            disp.show(img)

    if now - fps_t0 >= 1000:
        runtime_fps = fps_count
        if PRINT_FPS:
            print("fps:", runtime_fps, "cmd:", mode_command, "targets:", len(serial_targets))
        fps_count = 0
        fps_t0 = now

    if frame_id % GC_EVERY_N == 0:
        gc.collect()

    time.sleep_ms(1)

set_fill_light(False)
print("Fill light LED off")
