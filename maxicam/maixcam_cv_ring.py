# MaixCam YOLO material detection and lightweight Hough ring detection.
# Mode 0: idle. Mode 1: materials. Mode 2: three black rings.

from maix import camera, display, image, uart, pinmap, app, time, err, gpio, touchscreen, nn
import cv2
import numpy as np
import gc


# True: tap the screen twice to switch between material and ring modes.
# False: only UART commands from the controller can switch modes.
DEBUG_MODE = False


# =========================
# UART0 and fill light
# =========================
UART_DEVICE = "/dev/ttyS0"
UART_BAUDRATE = 9600
UART_PINS = {"A16": "UART0_TX", "A17": "UART0_RX"}

for pin, func in UART_PINS.items():
    err.check_raise(pinmap.set_pin_function(pin, func), "Failed set pin {} to {}".format(pin, func))

serial = uart.UART(UART_DEVICE, UART_BAUDRATE)
print("UART open: {}, baudrate {}".format(UART_DEVICE, UART_BAUDRATE))

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
RING_PROCESS_W = 240
RING_PROCESS_H = 180
CAM_FPS = 30
CAM_BUFF_NUM = 1
SHOW_IMAGE = True
DISPLAY_EVERY_N = 3
PRINT_FPS = DEBUG_MODE
GC_EVERY_N = 80

REPORT_INTERVAL_MS = 60
SERIAL_COORD_MAX = 240
MAX_TARGET_COUNT = 3
RING_ROI_SIZE = 140

RING_ROI_X0 = (RING_PROCESS_W - RING_ROI_SIZE) // 2
RING_ROI_Y0 = (RING_PROCESS_H - RING_ROI_SIZE) // 2
RING_ROI_X1 = RING_ROI_X0 + RING_ROI_SIZE
RING_ROI_Y1 = RING_ROI_Y0 + RING_ROI_SIZE

MODE_IDLE = 0
MODE_MATERIAL = 1
MODE_RING = 2
MODE_NAMES = {
    MODE_IDLE: "Idle",
    MODE_MATERIAL: "YOLO Material",
    MODE_RING: "Hough Black Rings",
}

TARGET_NAMES = ["UNKNOWN", "RED", "GREEN", "BLUE"]
TARGET_COLORS = [image.COLOR_WHITE, image.COLOR_RED, image.COLOR_GREEN, image.COLOR_BLUE]

FRAME_HEADER = 0x55
FRAME_MULTI_TARGET = 0x5B
FRAME_TAIL = 0xAA


# =========================
# Material model and ring parameters
# =========================
MATERIAL_MODEL_PATH = "/root/models/maixhub/Champion_material2/model_293122.mud"
# Champion_material2 labels: red, blue, green.
# Controller protocol classes: 1=red, 2=green, 3=blue.
MATERIAL_CLASS_MAP = {1: 1, 2: 3, 3: 2}
MATERIAL_CONFIDENCE = 0.5
MATERIAL_IOU = 0.45

RING_MIN_RADIUS = 6
RING_MAX_RADIUS = 65
RING_MIN_DISTANCE = 20


# =========================
# Camera, display, and touchscreen
# =========================
cam = camera.Camera(
    IMG_W,
    IMG_H,
    image.Format.FMT_RGB888,
    fps=CAM_FPS,
    buff_num=CAM_BUFF_NUM,
)
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
ts = touchscreen.TouchScreen() if DEBUG_MODE else None

material_detector = None
try:
    material_detector = nn.YOLOv5(model=MATERIAL_MODEL_PATH)
    print(
        "Material model loaded: {} ({}x{})".format(
            MATERIAL_MODEL_PATH,
            material_detector.input_width(),
            material_detector.input_height(),
        )
    )
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
# YOLO material detection
# =========================
def detect_materials(img):
    if material_detector is None:
        return []

    model_w = material_detector.input_width()
    model_h = material_detector.input_height()
    model_img = img.resize(model_w, model_h)
    objects = material_detector.detect(
        model_img,
        conf_th=MATERIAL_CONFIDENCE,
        iou_th=MATERIAL_IOU,
    )
    scale_x = IMG_W / model_w
    scale_y = IMG_H / model_h
    best_by_class = {}

    for obj in objects:
        raw_cls = int(obj.class_id) + 1
        cls = MATERIAL_CLASS_MAP.get(raw_cls)
        if cls is None:
            continue

        x = int(obj.x * scale_x)
        y = int(obj.y * scale_y)
        w = int(obj.w * scale_x)
        h = int(obj.h * scale_y)
        cx = x + w // 2
        cy = y + h // 2
        sx, sy = scale_coord(cx, cy)
        target = {
            "cls": cls,
            "x": x,
            "y": y,
            "w": w,
            "h": h,
            "cx": cx,
            "cy": cy,
            "sx": sx,
            "sy": sy,
            "score": float(obj.score),
        }
        old_target = best_by_class.get(cls)
        if old_target is None or target["score"] > old_target["score"]:
            best_by_class[cls] = target

    targets = list(best_by_class.values())
    targets.sort(key=lambda target: target["cx"])
    return targets[:MAX_TARGET_COUNT]


# =========================
# Lightweight Hough ring detection
# =========================
def detect_rings(img):
    frame_rgb = image.image2cv(img, ensure_bgr=False, copy=False)
    frame_small = cv2.resize(
        frame_rgb,
        (RING_PROCESS_W, RING_PROCESS_H),
        interpolation=cv2.INTER_AREA,
    )
    ring_roi = frame_small[RING_ROI_Y0:RING_ROI_Y1, RING_ROI_X0:RING_ROI_X1]
    gray = cv2.cvtColor(ring_roi, cv2.COLOR_RGB2GRAY)
    equalized = cv2.equalizeHist(gray)
    blurred = cv2.GaussianBlur(equalized, (5, 5), sigmaX=1.2, sigmaY=1.2)

    circles = cv2.HoughCircles(
        blurred,
        cv2.HOUGH_GRADIENT_ALT,
        1.5,
        RING_MIN_DISTANCE,
        param1=120,
        param2=0.85,
        minRadius=RING_MIN_RADIUS,
        maxRadius=min(RING_MAX_RADIUS, min(ring_roi.shape[0], ring_roi.shape[1]) // 2),
    )
    if circles is None:
        return []

    scale_x = IMG_W / RING_PROCESS_W
    scale_y = IMG_H / RING_PROCESS_H
    radius_scale = min(scale_x, scale_y)
    detected = []
    for cx, cy, radius in np.rint(circles[0, :, :3]).astype(np.int32).tolist():
        cx += RING_ROI_X0
        cy += RING_ROI_Y0
        image_cx = int(max(0, min(RING_PROCESS_W - 1, cx)) * scale_x)
        image_cy = int(max(0, min(RING_PROCESS_H - 1, cy)) * scale_y)
        image_radius = int(radius * radius_scale)
        sx, sy = scale_coord(image_cx, image_cy)
        detected.append({
            "cls": 0,
            "cx": image_cx,
            "cy": image_cy,
            "r": image_radius,
            "sx": sx,
            "sy": sy,
        })

    targets = detected[:MAX_TARGET_COUNT]
    targets.sort(key=lambda target: target["cx"])
    if len(targets) == 1:
        targets[0]["cls"] = 2
    else:
        for index, target in enumerate(targets):
            target["cls"] = index + 1
    return targets


# =========================
# Drawing
# =========================
def draw_runtime_status(img, runtime_fps, touch_count):
    run_name = "DEBUG" if DEBUG_MODE else "CAR"
    status = "{} FPS:{}".format(run_name, runtime_fps)
    if DEBUG_MODE:
        status += " TAP:{}/2".format(touch_count)
    img.draw_string(2, 22, status, image.COLOR_WHITE)


def draw_materials(img, targets):
    for target in targets:
        color = TARGET_COLORS[target["cls"]]
        img.draw_rect(target["x"], target["y"], target["w"], target["h"], color, 3)
        img.draw_string(
            target["x"],
            max(0, target["y"] - 18),
            "{} ({},{})".format(TARGET_NAMES[target["cls"]], target["sx"], target["sy"]),
            color,
        )


def draw_rings(img, targets):
    for target in targets:
        color = image.COLOR_YELLOW
        cx = target["cx"]
        cy = target["cy"]
        img.draw_circle(cx, cy, target["r"], color, 3)
        img.draw_line(cx - 8, cy, cx + 8, cy, color, 2)
        img.draw_line(cx, cy - 8, cx, cy + 8, color, 2)
        img.draw_string(
            max(0, cx - target["r"]),
            max(0, cy - target["r"] - 18),
            "RING{} ({},{})".format(target["cls"], target["sx"], target["sy"]),
            color,
        )


# =========================
# Main loop
# =========================
mode_command = MODE_MATERIAL
last_mode = None
last_touch_pressed = False
debug_touch_count = 0
frame_id = 0
fps_count = 0
fps_t0 = time.ticks_ms()
runtime_fps = 0
last_report_ms = 0

print("Run mode: {}".format("DEBUG" if DEBUG_MODE else "CAR"))

while not app.need_exit():
    frame_id += 1
    fps_count += 1
    now = time.ticks_ms()

    touch_x = 0
    touch_y = 0
    touch_pressed = False
    if DEBUG_MODE:
        touch_x, touch_y, touch_pressed = ts.read()
        if touch_pressed and not last_touch_pressed:
            debug_touch_count += 1
            print("Debug tap: {}/2".format(debug_touch_count))
            if debug_touch_count >= 2:
                mode_command = MODE_RING if mode_command != MODE_RING else MODE_MATERIAL
                debug_touch_count = 0
        last_touch_pressed = touch_pressed
    else:
        mode_command = read_mode_command(mode_command)

    if mode_command != last_mode:
        set_fill_light(mode_command in (MODE_MATERIAL, MODE_RING))
        last_mode = mode_command
        print("Mode changed to {}".format(mode_command))

    img = cam.read()

    if mode_command == MODE_IDLE:
        targets = []
    elif mode_command == MODE_MATERIAL:
        targets = detect_materials(img)
    elif mode_command == MODE_RING:
        targets = detect_rings(img)
    else:
        mode_command = MODE_IDLE
        targets = []

    if mode_command != MODE_IDLE and now - last_report_ms >= REPORT_INTERVAL_MS:
        send_multi_targets(mode_command, targets)
        last_report_ms = now

    if SHOW_IMAGE and frame_id % DISPLAY_EVERY_N == 0:
        img.draw_string(2, 2, "Mode{}:{}".format(mode_command, MODE_NAMES[mode_command]), image.COLOR_WHITE)
        draw_runtime_status(img, runtime_fps, debug_touch_count)
        if mode_command == MODE_MATERIAL:
            draw_materials(img, targets)
        elif mode_command == MODE_RING:
            roi_x = int(RING_ROI_X0 * IMG_W / RING_PROCESS_W)
            roi_y = int(RING_ROI_Y0 * IMG_H / RING_PROCESS_H)
            roi_w = int(RING_ROI_SIZE * IMG_W / RING_PROCESS_W)
            roi_h = int(RING_ROI_SIZE * IMG_H / RING_PROCESS_H)
            img.draw_rect(roi_x, roi_y, roi_w, roi_h, image.COLOR_BLUE, 2)
            draw_rings(img, targets)
        if mode_command != MODE_IDLE and not targets:
            img.draw_string(2, 42, "No Target", image.COLOR_RED)
        if DEBUG_MODE and touch_pressed:
            img.draw_circle(touch_x, touch_y, 8, image.COLOR_WHITE, 2)
        if disp is not None:
            disp.show(img)

    if now - fps_t0 >= 1000:
        runtime_fps = fps_count
        if PRINT_FPS:
            print("fps:", runtime_fps, "cmd:", mode_command, "targets:", len(targets))
        fps_count = 0
        fps_t0 = now

    if frame_id % GC_EVERY_N == 0:
        gc.collect()

    time.sleep_ms(1)

set_fill_light(False)
print("Fill light LED off")
