# MaixCam vision module.
# Mode 0: idle. Mode 1: YOLO materials. Mode 2: three black rings with pure CV.

from maix import camera, display, image, uart, pinmap, app, time, err, nn, gpio
import cv2
import numpy as np
import gc


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
    MODE_RING: "CV 3 Black Rings",
}

TARGET_NAMES = ["RED", "GREEN", "BLUE"]
TARGET_COLORS = [image.COLOR_RED, image.COLOR_GREEN, image.COLOR_BLUE]
MATERIAL_CLASS_MAP = {1: 1, 2: 3, 3: 2}

FRAME_HEADER = 0x55
FRAME_MULTI_TARGET = 0x5B
FRAME_TAIL = 0xAA


# =========================
# Black ring refinement
# =========================
BLACK_GRAY_MAX = 130
BLACK_PROCESS_W = 320
BLACK_PROCESS_H = 240
BLACK_CLOSE_ITERATIONS = 2
BLACK_MIN_OUTER_AREA = 150
BLACK_MAX_OUTER_AREA = 20000
BLACK_MIN_HOLE_RATIO = 0.15
BLACK_MAX_HOLE_RATIO = 0.95
BLACK_MIN_CIRCULARITY = 0.30
BLACK_MIN_ASPECT_RATIO = 0.50
BLACK_MAX_ASPECT_RATIO = 2.00
BLACK_MAX_TARGETS = 3
BLACK_KERNEL = np.ones((3, 3), dtype=np.uint8)


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
# Three black rings: dark threshold and nested contours
# =========================
def detect_rings(img):
    frame_rgb = image.image2cv(img, ensure_bgr=False, copy=False)
    frame_small = cv2.resize(
        frame_rgb,
        (BLACK_PROCESS_W, BLACK_PROCESS_H),
        interpolation=cv2.INTER_AREA,
    )
    gray = cv2.cvtColor(frame_small, cv2.COLOR_RGB2GRAY)
    gray = cv2.GaussianBlur(gray, (3, 3), 0)
    _, mask = cv2.threshold(gray, BLACK_GRAY_MAX, 255, cv2.THRESH_BINARY_INV)
    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        BLACK_KERNEL,
        iterations=BLACK_CLOSE_ITERATIONS,
    )
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if hierarchy is None:
        return []

    scale_x = IMG_W / BLACK_PROCESS_W
    scale_y = IMG_H / BLACK_PROCESS_H
    candidates = []

    for index, contour in enumerate(contours):
        child_index = hierarchy[0][index][2]
        if child_index < 0 or len(contour) < 5:
            continue

        outer_area = cv2.contourArea(contour)
        if not BLACK_MIN_OUTER_AREA <= outer_area <= BLACK_MAX_OUTER_AREA:
            continue

        perimeter = cv2.arcLength(contour, True)
        if perimeter <= 0:
            continue
        circularity = 4.0 * np.pi * outer_area / (perimeter * perimeter)
        if circularity < BLACK_MIN_CIRCULARITY:
            continue

        _, _, width, height = cv2.boundingRect(contour)
        if width <= 0 or height <= 0:
            continue
        aspect_ratio = width / float(height)
        if not BLACK_MIN_ASPECT_RATIO <= aspect_ratio <= BLACK_MAX_ASPECT_RATIO:
            continue

        largest_hole_area = 0.0
        current_child = child_index
        while current_child >= 0:
            largest_hole_area = max(
                largest_hole_area,
                cv2.contourArea(contours[current_child]),
            )
            current_child = hierarchy[0][current_child][0]

        hole_ratio = largest_hole_area / outer_area
        if not BLACK_MIN_HOLE_RATIO <= hole_ratio <= BLACK_MAX_HOLE_RATIO:
            continue

        (ellipse_x, ellipse_y), (ellipse_w, ellipse_h), _ = cv2.fitEllipse(contour)
        x, y, w, h = cv2.boundingRect(contour)
        cx = int(ellipse_x * scale_x)
        cy = int(ellipse_y * scale_y)
        radius = int((ellipse_w * scale_x + ellipse_h * scale_y) / 4.0)
        sx, sy = scale_coord(cx, cy)
        candidates.append({
            "cls": 0,
            "x": int(x * scale_x),
            "y": int(y * scale_y),
            "w": int(w * scale_x),
            "h": int(h * scale_y),
            "r": radius,
            "cx": cx,
            "cy": cy,
            "sx": sx,
            "sy": sy,
            "score": outer_area * hole_ratio,
            "black_ring": True,
        })

    candidates.sort(key=lambda target: target["score"], reverse=True)
    targets = candidates[:BLACK_MAX_TARGETS]
    targets.sort(key=lambda target: target["cx"])
    for index, target in enumerate(targets):
        target["cls"] = index + 1
    return targets


# =========================
# Drawing
# =========================
def draw_targets(img, targets, title):
    img.draw_string(2, 2, title, image.COLOR_WHITE)
    for target in targets:
        if target.get("black_ring"):
            color = image.COLOR_YELLOW
            name = "RING{}".format(target["cls"])
        else:
            color = TARGET_COLORS[target["cls"] - 1]
            name = TARGET_NAMES[target["cls"] - 1]
        if "r" in target:
            img.draw_circle(target["cx"], target["cy"], target["r"], color, 3)
            img.draw_circle(target["cx"], target["cy"], 5, image.COLOR_RED, -1)
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
