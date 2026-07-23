# MaixCam touchscreen HSV threshold tuning tool.

from maix import camera, display, image, touchscreen, app, time, pinmap, err, gpio
import cv2
import numpy as np
import json
import os


SAVE_PATH = "/root/hsv_thresholds.json"
STEP = 1
TOP_BAR_H = 66
BOTTOM_BAR_H = 64
SIDE_BAR_W = 112
SAVE_MESSAGE_MS = 1200

PRESET_KEYS = ["red1", "red2", "green", "blue"]
PRESET_NAMES = ["RED1", "RED2", "GREEN", "BLUE"]
DEFAULT_PRESETS = {
    "red1": [0, 10, 100, 255, 100, 255],
    "red2": [160, 180, 100, 255, 100, 255],
    "green": [0, 255, 0, 255, 0, 30],
    "blue": [92, 122, 71, 255, 88, 255],
}
PARAM_NAMES = ["H min", "H max", "S min", "S max", "V min", "V max"]

COLOR_DARK = image.Color.from_rgb(28, 30, 34)
COLOR_GRAY = image.Color.from_rgb(72, 76, 84)
COLOR_BLUE = image.Color.from_rgb(20, 100, 220)
COLOR_GREEN = image.Color.from_rgb(20, 165, 70)
COLOR_RED = image.Color.from_rgb(220, 45, 45)
COLOR_WHITE = image.Color.from_rgb(255, 255, 255)
COLOR_YELLOW = image.Color.from_rgb(255, 220, 40)


def valid_values(values):
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


def load_presets():
    presets = {name: values.copy() for name, values in DEFAULT_PRESETS.items()}
    if not os.path.exists(SAVE_PATH):
        return presets
    try:
        with open(SAVE_PATH, "r") as file:
            saved = json.load(file)
        for name in presets:
            values = saved.get(name)
            if valid_values(values):
                presets[name] = [int(value) for value in values]
    except Exception as exc:
        print("Load threshold failed: {}".format(exc))
    return presets


def save_presets(presets):
    with open(SAVE_PATH, "w") as file:
        json.dump(presets, file)
    print("Saved HSV thresholds: {}".format(presets))


def init_fill_light():
    try:
        err.check_raise(pinmap.set_pin_function("B3", "GPIOB3"), "Failed set fill light pin")
        light = gpio.GPIO("GPIOB3", gpio.Mode.OUT)
        light.value(1)
        return light
    except Exception as exc:
        print("Fill light init failed: {}".format(exc))
        return None


def point_in_rect(x, y, rect):
    rx, ry, rw, rh = rect
    return rx <= x < rx + rw and ry <= y < ry + rh


def draw_button(img, rect, text, fill_color, selected=False):
    x, y, w, h = rect
    border = COLOR_YELLOW if selected else COLOR_WHITE
    img.draw_rect(x, y, w, h, fill_color, -1)
    img.draw_rect(x, y, w, h, border, 2)
    img.draw_string(x + 6, y + max(2, (h - 16) // 2), text, COLOR_WHITE)


def adjust_value(values, selected, delta):
    limit = 180 if selected < 2 else 255
    new_value = values[selected] + delta * STEP
    if selected % 2 == 0:
        values[selected] = max(0, min(values[selected + 1], new_value))
    else:
        values[selected] = max(values[selected - 1], min(limit, new_value))


disp = display.Display()
screen_w = disp.width()
screen_h = disp.height()
preview_w = screen_w - SIDE_BAR_W
preview_h = screen_h - BOTTOM_BAR_H

cam = camera.Camera(
    screen_w,
    screen_h,
    image.Format.FMT_BGR888,
    fps=30,
    buff_num=1,
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

ts = touchscreen.TouchScreen()
fill_light = init_fill_light()
presets = load_presets()
selected_preset = 0
selected_param = 0
show_binary = False
last_pressed = False
last_saved_ms = -SAVE_MESSAGE_MS

param_button_h = preview_h // 6
param_rects = []
for index in range(6):
    param_rects.append((preview_w + 2, index * param_button_h + 2, SIDE_BAR_W - 4, param_button_h - 4))

preset_button_w = preview_w // 4
preset_rects = []
for index in range(4):
    x = index * preset_button_w + 2
    width = preset_button_w - 4 if index < 3 else preview_w - x - 2
    preset_rects.append((x, 30, width, TOP_BAR_H - 32))

bottom_button_w = preview_w // 4
bottom_rects = []
for index in range(4):
    x = index * bottom_button_w + 2
    width = bottom_button_w - 4 if index < 3 else preview_w - x - 2
    bottom_rects.append((x, preview_h + 2, width, BOTTOM_BAR_H - 4))

print("HSV threshold tool: {}x{}".format(screen_w, screen_h))
print("Save path: {}".format(SAVE_PATH))

while not app.need_exit():
    camera_img = cam.read()
    values = presets[PRESET_KEYS[selected_preset]]

    if show_binary:
        frame_bgr = image.image2cv(camera_img, ensure_bgr=False, copy=False)
        hsv_img = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        lower = np.array([values[0], values[2], values[4]], dtype=np.uint8)
        upper = np.array([values[1], values[3], values[5]], dtype=np.uint8)
        binary_mask = cv2.inRange(hsv_img, lower, upper)
        binary_bgr = cv2.cvtColor(binary_mask, cv2.COLOR_GRAY2BGR)
        ui_img = image.cv2image(binary_bgr, bgr=True, copy=True)
    else:
        ui_img = camera_img

    touch_x, touch_y, pressed = ts.read()
    now = time.ticks_ms()
    if pressed and not last_pressed:
        handled = False
        for index, rect in enumerate(preset_rects):
            if point_in_rect(touch_x, touch_y, rect):
                selected_preset = index
                handled = True
                break

        if not handled:
            for index, rect in enumerate(param_rects):
                if point_in_rect(touch_x, touch_y, rect):
                    selected_param = index
                    handled = True
                    break

        values = presets[PRESET_KEYS[selected_preset]]
        if not handled and point_in_rect(touch_x, touch_y, bottom_rects[0]):
            adjust_value(values, selected_param, 1)
        elif not handled and point_in_rect(touch_x, touch_y, bottom_rects[1]):
            adjust_value(values, selected_param, -1)
        elif not handled and point_in_rect(touch_x, touch_y, bottom_rects[2]):
            show_binary = not show_binary
        elif not handled and point_in_rect(touch_x, touch_y, bottom_rects[3]):
            try:
                save_presets(presets)
                last_saved_ms = now
            except Exception as exc:
                print("Save threshold failed: {}".format(exc))

    values = presets[PRESET_KEYS[selected_preset]]
    ui_img.draw_rect(0, 0, preview_w, TOP_BAR_H, COLOR_DARK, -1)
    ui_img.draw_string(
        4,
        5,
        "HSV {},{},{},{},{},{}".format(*values),
        COLOR_YELLOW,
    )

    for index, rect in enumerate(preset_rects):
        fill = COLOR_BLUE if index == selected_preset else COLOR_GRAY
        draw_button(ui_img, rect, PRESET_NAMES[index], fill, index == selected_preset)

    for index, rect in enumerate(param_rects):
        label = "{} {}".format(PARAM_NAMES[index], values[index])
        fill = COLOR_BLUE if index == selected_param else COLOR_GRAY
        draw_button(ui_img, rect, label, fill, index == selected_param)

    draw_button(ui_img, bottom_rects[0], "+", COLOR_GREEN)
    draw_button(ui_img, bottom_rects[1], "-", COLOR_RED)
    draw_button(ui_img, bottom_rects[2], "RGB" if show_binary else "BIN", COLOR_BLUE)
    draw_button(ui_img, bottom_rects[3], "SAVE", COLOR_GRAY)

    if now - last_saved_ms < SAVE_MESSAGE_MS:
        ui_img.draw_rect(4, TOP_BAR_H + 4, 128, 24, COLOR_GREEN, -1)
        ui_img.draw_string(9, TOP_BAR_H + 8, "SAVED", COLOR_WHITE)

    if pressed:
        ui_img.draw_circle(touch_x, touch_y, 6, COLOR_YELLOW, 2)

    disp.show(ui_img)
    last_pressed = pressed
    time.sleep_ms(1)

if fill_light is not None:
    fill_light.value(0)
print("Fill light LED off")
