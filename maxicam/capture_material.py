from maix import camera, display, image, touchscreen, app, time, pinmap, err, gpio
import os


IMG_W = 640
IMG_H = 480
CAM_FPS = 30
SAVE_DIR = "/root/ring_photos"
NAME_PREFIX = "ring"
START_INDEX = 366
JPEG_QUALITY = 95
SHOW_SAVE_MS = 1000


def init_fill_light():
    try:
        err.check_raise(pinmap.set_pin_function("B3", "GPIOB3"), "Failed set fill light pin")
        light = gpio.GPIO("GPIOB3", gpio.Mode.OUT)
        light.value(1)
        print("Fill light LED on")
        return light
    except Exception as e:
        print("Fill light LED init failed: {}".format(e))
        return None


def set_fill_light(light, enabled):
    if light is not None:
        light.value(1 if enabled else 0)


def ensure_dir(path):
    if not os.path.exists(path):
        os.mkdir(path)


def next_index(path):
    index = 1
    for name in os.listdir(path):
        if not name.startswith(NAME_PREFIX + "_") or not name.endswith(".jpg"):
            continue
        try:
            value = int(name[len(NAME_PREFIX) + 1:-4])
        except Exception:
            continue
        if value >= index:
            index = value + 1
    return index


ensure_dir(SAVE_DIR)
fill_light = init_fill_light()

cam = camera.Camera(IMG_W, IMG_H, fps=CAM_FPS)
cam.skip_frames(20)
disp = display.Display()
ts = touchscreen.TouchScreen()

photo_index = max(START_INDEX, next_index(SAVE_DIR))
last_pressed = False
last_saved_name = ""
last_saved_path = ""
last_saved_ms = 0

print("Tap screen to save ring photos to {}".format(SAVE_DIR))

while not app.need_exit():
    img = cam.read()
    x, y, pressed = ts.read()
    now = time.ticks_ms()

    if pressed and not last_pressed:
        last_saved_name = "{}_{:04d}.jpg".format(NAME_PREFIX, photo_index)
        last_saved_path = "{}/{}".format(SAVE_DIR, last_saved_name)
        err = img.save(last_saved_path, quality=JPEG_QUALITY)
        print("save:", last_saved_path, "err:", err)
        photo_index += 1
        last_saved_ms = now

    img.draw_string(2, 2, "Tap screen to capture", image.COLOR_WHITE)
    img.draw_string(2, 22, "Saved: {}".format(photo_index - 1), image.COLOR_GREEN)
    if now - last_saved_ms < SHOW_SAVE_MS and last_saved_path:
        img.draw_string(2, 42, "OK: {}".format(last_saved_name), image.COLOR_GREEN)
    if pressed:
        img.draw_circle(x, y, 8, image.COLOR_RED, 2)

    disp.show(img)
    last_pressed = pressed
    time.sleep_ms(1)

set_fill_light(fill_light, False)
print("Fill light LED off")
