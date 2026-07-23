# maixcam_vision_multi_target.py
# MaixCam Pro vision module.
# Mode 0: idle, fill light off.
# Mode 1: detect red/green/blue materials.
# Mode 2: detect red/green/blue rings.

from maix import camera, display, image, uart, pinmap, app, time, err, nn, gpio
import gc


# =========================
# UART0 config
# =========================
UART_DEVICE = "/dev/ttyS0"
UART_BAUDRATE = 9600
UART_PINS = {"A16": "UART0_TX", "A17": "UART0_RX"}

for pin, func in UART_PINS.items():
    err.check_raise(pinmap.set_pin_function(pin, func), "Failed set pin {} to {}".format(pin, func))

serial = uart.UART(UART_DEVICE, UART_BAUDRATE)
print("UART open: {}, baudrate {}".format(UART_DEVICE, UART_BAUDRATE))


# =========================
# Fill light
# =========================
fill_light = None
try:
    err.check_raise(pinmap.set_pin_function("B3", "GPIOB3"), "Failed set fill light pin")
    fill_light = gpio.GPIO("GPIOB3", gpio.Mode.OUT)
    fill_light.value(0)
    print("Fill light LED off")
except Exception as e:
    print("Fill light LED init failed: {}".format(e))


# =========================
# Camera and display config
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


def scale_roi(x, y, w, h):
    sx = IMG_W / 640.0
    sy = IMG_H / 480.0
    return [int(x * sx), int(y * sy), int(w * sx), int(h * sy)]


ROI_MATERIAL = scale_roi(0, 0, 640, 480)
ROI_RING = scale_roi(0, 0, 640, 480)

MODE_IDLE = 0
MODE_MATERIAL = 1
MODE_RING = 2
MODE_NAMES = {
    MODE_IDLE: "Idle",
    MODE_MATERIAL: "Material",
    MODE_RING: "Ring",
}

TARGET_NAMES = ["RED", "GREEN", "BLUE"]
TARGET_COLORS = [image.COLOR_RED, image.COLOR_GREEN, image.COLOR_BLUE]
MATERIAL_CLASS_MAP = {1: 1, 2: 3, 3: 2}
RING_CLASS_MAP = {1: 2, 2: 1, 3: 3}

FRAME_HEADER = 0x55
FRAME_MULTI_TARGET = 0x5B
FRAME_TAIL = 0xAA


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


ring_filters = {cls: new_target_filter() for cls in range(1, 4)}
material_filters = {cls: new_target_filter() for cls in range(1, 4)}


# =========================
# Camera init
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

if SHOW_IMAGE:
    disp = display.Display()
else:
    disp = None


# =========================
# YOLOv5 models
# =========================
ring_detector = None
try:
    ring_detector = nn.YOLOv5(model="/root/models/maixhub/Champion_ring2/model_293138.mud")
    print("Ring model loaded: {}x{}".format(ring_detector.input_width(), ring_detector.input_height()))
except Exception as e:
    print("Ring model load failed: {}".format(e))

material_detector = None
try:
    material_detector = nn.YOLOv5(model="/root/models/maixhub/Champion_material2/model_293122.mud")
    print("Material model loaded: {}x{}".format(material_detector.input_width(), material_detector.input_height()))
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


def read_mode_command(current_mode):
    data = serial.read()
    if not data:
        return current_mode

    if isinstance(data, int):
        data = [data]

    new_mode = current_mode
    for b in data:
        if 0x30 <= b <= 0x32:
            new_mode = b - 0x30
            set_fill_light(new_mode in (MODE_MATERIAL, MODE_RING))
        elif b == ord("L"):
            set_fill_light(True)
        elif b == ord("l"):
            set_fill_light(False)
    return new_mode


def send_multi_targets(mode, targets):
    payload = [FRAME_HEADER, FRAME_MULTI_TARGET, mode, len(targets)]
    for target in targets:
        payload.extend([target["cls"], target["sx"], target["sy"]])
    payload.append(FRAME_TAIL)
    serial.write(bytes(payload))


def set_fill_light(enabled):
    if fill_light is None:
        return
    fill_light.value(1 if enabled else 0)


def reset_target_filters():
    for filters in (ring_filters, material_filters):
        for cls in filters:
            filters[cls] = new_target_filter()


def targets_at_same_position(a, b):
    limit_x = max(a["w"], b["w"]) * 0.6
    limit_y = max(a["h"], b["h"]) * 0.6
    return abs(a["cx"] - b["cx"]) <= limit_x and abs(a["cy"] - b["cy"]) <= limit_y


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
            color_replaced = any(
                current["cls"] != cls and targets_at_same_position(current, state["last_target"])
                for current in targets
            )
            if color_replaced:
                state["valid"] = False
                state["miss"] = 0
                state["last_target"] = None
                continue

            state["miss"] += 1
            if state["miss"] < FILTER_MISS_LIMIT:
                filtered_targets.append(state["last_target"].copy())
            else:
                state["valid"] = False
                state["last_target"] = None

    return filtered_targets


# =========================
# Detection and drawing
# =========================
def detect_three_targets(img, model, roi, class_map=None):
    if model is None:
        return []

    model_w = model.input_width()
    model_h = model.input_height()
    img_small = img.resize(model_w, model_h)
    objs = model.detect(img_small, conf_th=0.5, iou_th=0.45)

    scale_x = IMG_W / model_w
    scale_y = IMG_H / model_h
    best_by_cls = {}

    for obj in objs:
        raw_cls = int(obj.class_id) + 1
        cls = class_map.get(raw_cls, raw_cls) if class_map else raw_cls
        if cls < 1 or cls > 3:
            continue

        box_x = obj.x * scale_x
        box_y = obj.y * scale_y
        box_w = obj.w * scale_x
        box_h = obj.h * scale_y
        cx = box_x + box_w / 2.0
        cy = box_y + box_h / 2.0

        if cx < roi[0] or cx > roi[0] + roi[2] or cy < roi[1] or cy > roi[1] + roi[3]:
            continue

        area = box_w * box_h
        score = obj.score
        old = best_by_cls.get(cls)
        if old is not None:
            if score < old["score"]:
                continue
            if score == old["score"] and area <= old["area"]:
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
            "score": score,
            "area": area,
        }

    return [best_by_cls[cls] for cls in sorted(best_by_cls.keys())]


def draw_targets(img, targets, title):
    img.draw_string(2, 2, title, image.COLOR_WHITE)

    for target in targets:
        idx = target["cls"] - 1
        color = TARGET_COLORS[idx]
        name = TARGET_NAMES[idx]
        x = target["x"]
        y = target["y"]
        w = target["w"]
        h = target["h"]
        cx = target["cx"]
        cy = target["cy"]

        img.draw_rect(x, y, w, h, color, 3)
        img.draw_line(cx - 8, cy, cx + 8, cy, color, 2)
        img.draw_line(cx, cy - 8, cx, cy + 8, color, 2)
        img.draw_string(x, max(0, y - 18), "{} ({},{})".format(name, target["sx"], target["sy"]), color)


def draw_runtime_status(img):
    if hasattr(gc, "mem_alloc") and hasattr(gc, "mem_free"):
        status = "FPS:{} MEM:{}/{}K".format(runtime_fps, gc.mem_alloc() // 1024, gc.mem_free() // 1024)
    else:
        status = "FPS:{}".format(runtime_fps)
    img.draw_string(2, 22, status, image.COLOR_WHITE)


def show_idle(img):
    img.draw_string(2, 2, "Mode0:Idle", image.COLOR_GREEN)
    draw_runtime_status(img)
    if disp is not None:
        disp.show(img)


def show_targets(img, roi, targets, mode_name):
    img.draw_rect(roi[0], roi[1], roi[2], roi[3], image.COLOR_BLUE, 2)
    draw_targets(img, targets, mode_name)
    draw_runtime_status(img)
    if not targets:
        img.draw_string(2, 42, "No Target", image.COLOR_RED)
    if disp is not None:
        disp.show(img)


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
        if SHOW_IMAGE and frame_id % DISPLAY_EVERY_N == 0:
            show_idle(img)
        time.sleep_ms(1)
        continue

    if mode_command == MODE_MATERIAL:
        model = material_detector
        roi = ROI_MATERIAL
    elif mode_command == MODE_RING:
        model = ring_detector
        roi = ROI_RING
    else:
        mode_command = MODE_IDLE
        time.sleep_ms(1)
        continue

    if mode_command == MODE_MATERIAL:
        targets = detect_three_targets(img, model, roi, MATERIAL_CLASS_MAP)
    else:
        targets = detect_three_targets(img, model, roi, RING_CLASS_MAP)
    if mode_command == MODE_RING:
        serial_targets = filter_targets(targets, ring_filters)
    else:
        serial_targets = filter_targets(targets, material_filters)

    if now - last_report_ms >= REPORT_INTERVAL_MS:
        send_multi_targets(mode_command, serial_targets)
        last_report_ms = now

    if SHOW_IMAGE and frame_id % DISPLAY_EVERY_N == 0:
        title = "Mode{}:{}".format(mode_command, MODE_NAMES.get(mode_command, "Unknown"))
        show_targets(img, roi, targets, title)

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
