Based on OpenMV line follower example
Run this code on an OpenMV H7 plus camera. It probably works on the RT1060 too..

# OBSTACLE CHALLENGE

```   
import sensor
import time
from pupremote import PUPRemoteSensor
from pyb import Pin, Timer

# ===========================================================================
# Tunables
# ===========================================================================
DEBUG = True             # prints FPS to the OpenMV IDE; does NOT touch PUP data.
FPS_PRINT_EVERY = 30     # print fps once every N frames when DEBUG is on.

LED_DUTY = 30            # LED brightness %.

# Lock exposure / gain to fixed values to fight MOTION BLUR.
EXPOSURE_US = None
GAIN_DB = None

# --- IMAGE PROCESSING LAYER ---
USE_CLAHE = True         # Normalizes local lighting spots caused by your LEDs.
CLAHE_CLIP_LIMIT = 3     # High numbers = more contrast, lower numbers = less noise.

AREA_THRESHOLD = 150     #minimum number of pixels needed to detect block.

# Gamma / contrast applied to the analysis frame before find_blobs.
GAMMA = 1.9
CONTRAST = 1.1
BRIGHTNESS = -0.1

# ===========================================================================
# Colour thresholds (LAB) -- BLOCK AND CORNER DETECTION
# ===========================================================================
threshold_pink  = (30, 95, 40, 127, -127, -10)   # bright magenta / pink
threshold_red   = (20, 90, 30, 127, 15, 127)     # red
threshold_red2 = (0, 37, 11, 55, 6, 127)
threshold_green = (20, 90, -128, -29, -1, 44)    # green
# threshold_green2 = (19, 39, -92, -19, 6, 38)
threshold_black = (0, 37, -25, 0, -19, 8)   # black (corner marker)
black2 = (0, 20, -7, -2, -14, 4)

# ===========================================================================
# ROIs FOR CORNER AND BLOCK DETECTION
# ===========================================================================
img_center = (160, 120)
img_roi = (0, 110, 320, 130)
roi_rect = (img_roi[0], img_roi[1], img_roi[2], img_roi[3])
roi_left_bottom = (5, 201, 70, 35)
roi_right_bottom = (245, 201, 70, 35)
img_roi_corner = (155, 110, 10, 80)


# ===========================================================================
# PUPRemote Command Callback
# ===========================================================================
def msg(txt):
    # Callback handler for PUPRemote 'msg' command received from the Hub.
    # PUPRemote requires a return value matching the 'repr' format.
    print(txt)
    return txt + txt


# ===========================================================================
# LED Control
# ===========================================================================
light = Timer(2, freq=50000).channel(1, Timer.PWM, pin=Pin("P6"))
light.pulse_width_percent(LED_DUTY)

# ===========================================================================
# Camera Sensor Initialization
# ===========================================================================
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(False)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False)
sensor.skip_frames(time=2000)

# Optional explicit locks, applied AFTER skip_frames so they actually stick.
if GAIN_DB is not None:
    sensor.set_auto_gain(False, gain_db=GAIN_DB)
if EXPOSURE_US is not None:
    sensor.set_auto_exposure(False, exposure_us=EXPOSURE_US)

# ===========================================================================
# PUPRemote Communication
# ===========================================================================
p = PUPRemoteSensor(power=True)
p.add_command('msg', "repr", "repr")
p.add_channel('cam', to_hub_fmt='bhhb')


def _bbox_center(b):
    return (b.x() + (b.w() // 2), b.y() + (b.h() // 2))


def find_block(img_proc, img_debug):
    # Detect the nearest red / green / pink block.

    # img_proc  : Shared gamma-corrected and CLAHE analysis frame.
    # img_debug : Raw overlay frame displayed in OpenMV IDE.

    nearestRed = None
    nearestGreen = None
    nearestPink = None
    red_val = 0
    green_val = 0
    pink_val = 0

    img_debug.draw_rectangle(img_roi, color=(0, 0, 255))

    red = img_proc.find_blobs([threshold_red, threshold_red2], area_threshold=AREA_THRESHOLD, roi=img_roi, merge=True)
    green = img_proc.find_blobs([threshold_green], area_threshold=AREA_THRESHOLD, roi=img_roi, merge=True)
    pink = img_proc.find_blobs([threshold_pink], area_threshold=AREA_THRESHOLD, roi=img_roi, merge=True)

    if red:
        for b in red:
            cx, cy = _bbox_center(b)
            img_debug.draw_rectangle(b.rect(), color=(255, 0, 0))
            img_debug.draw_cross(cx, cy, color=(255, 0, 0))

            # Proximity calculation: Y + H equals the bottom edge of the blob.
            # Higher pixel value = lower down in the frame = closer to the robot camera.
            val = b.y() + b.h()
            if val > red_val:
                red_val = val
                nearestRed = b

    if green:
        for b in green:
            cx, cy = _bbox_center(b)
            img_debug.draw_rectangle(b.rect(), color=(0, 255, 0))
            img_debug.draw_cross(cx, cy, color=(0, 255, 0))

            # Proximity calculation: Higher pixel value = closer to the robot camera.
            val = b.y() + b.h()
            if val > green_val:
                green_val = val
                nearestGreen = b

    if pink:
        for b in pink:
            cx, cy = _bbox_center(b)
            img_debug.draw_rectangle(b.rect(), color=(255, 0, 255))
            img_debug.draw_cross(cx, cy, color=(255, 0, 255))

            # Pink is actively tracked to isolate it from red, preventing false red positives.
            val = b.y() + b.h()
            if val > pink_val:
                pink_val = val
                nearestPink = b

    # Pick dominant colour based on closest block (lowest in frame)
    # COLOR MAPPING KEY FOR HUB: 0 = None, 1 = Green, 2 = Red
    if nearestRed and nearestGreen:
        color = 2 if red_val >= green_val else 1
    elif nearestRed:
        color = 2
    elif nearestGreen:
        color = 1
    else:
        color = 0

    if color == 2:
        center_x, center_y = _bbox_center(nearestRed)
    elif color == 1:
        center_x, center_y = _bbox_center(nearestGreen)
    else:
        center_x, center_y = 0, 0

    if nearestPink is not None:
        p_center_x, p_center_y = _bbox_center(nearestPink)
    else:
        p_center_x, p_center_y = 0, 0

    return {
        "center_x": center_x,
        "center_y": center_y,
        "color": color,
        "p_center_x": p_center_x,
        "p_center_y": p_center_y,
    }


def check_corner_roi(img_proc, img_debug):
    img_debug.draw_rectangle(img_roi_corner, color=(255, 255, 0))
    black = img_proc.find_blobs([threshold_black], area_threshold=100, roi=img_roi_corner, merge=True)
    return {"black": 1 if black else 0}


# ===========================================================================
# Main Execution Loop
# ===========================================================================
clock = time.clock()
_frame = 0

while True:
    clock.tick()
    img_debug = sensor.snapshot()

    # Image preparation layer
    img_proc = img_debug.copy()
    img_proc.gamma_corr(gamma=GAMMA, contrast=CONTRAST, brightness=BRIGHTNESS)

    if USE_CLAHE:
        img_proc.histeq(adaptive=True, clip_limit=CLAHE_CLIP_LIMIT)

    img_debug.draw_cross(160, 120, color=(0, 0, 0)) # place cross on the center of the camera

    block = find_block(img_proc, img_debug)
    corner = check_corner_roi(img_proc, img_debug)

    # NOTE: Pink block data is purposely omitted from 'data' tuple because the
    # 'cam' PUPRemote channel format ('bhhb') is only configured to send 4 values.
    data = (block["color"], block["center_x"], block["center_y"], corner["black"])
    p.update_channel('cam', *data)
    p.process()

    if DEBUG:
        _frame += 1
        if _frame >= FPS_PRINT_EVERY:
            print("fps:", clock.fps())
            _frame = 0

    # Give the hardware background handler a tiny window to service the USB link
    time.sleep_ms(1)
```

# OPEN CHALLENGE

```
import sensor
from pupremote import PUPRemoteSensor
from pyb import Pin, Timer

def msg(txt):
    print(txt)
    return txt+txt

light = Timer(2, freq=50000).channel(1, Timer.PWM, pin=Pin("P6"))
light.pulse_width_percent(100)

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_hmirror(False)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
sensor.set_auto_exposure(False)
sensor.skip_frames(time=2000)

p = PUPRemoteSensor(power=False)
p.add_command('msg', "repr", "repr")
p.add_channel('cam', to_hub_fmt='bhhb')

img_center = (160, 120)
img_roi = (5, 110, 310, 125)
roi_rect = (img_roi[0], img_roi[1], img_roi[2], img_roi[3])
roi_left_bottom = (5, 201, 70, 35)
roi_right_bottom = (245, 201, 70, 35)
img_roi_corner = (155, 80, 10,40)

def find_block(img, img_debug, distance_cap):
    img_contrast = img.copy()
    img_contrast.gamma_corr(gamma=1.9, contrast=1.1, brightness=-0.1)
    color = 0
    nearestRed = None
    nearestGreen = None
    nearestPink = None
    red_val = 0
    green_val = 0
    pink_val = 0
    blocks = 0

    # Bright Magenta / Pink
    threshold_pink = (30, 95, 40, 127, -127, -10)

    # Just Red
    threshold_red = (20, 90, 30, 127, 15, 127)

    # Standard Green
    threshold_green = (20, 90, -127, -20, -10, 80)

    img_debug.draw_rectangle(img_roi, color=(0, 0, 255))
    red = img_contrast.find_blobs([threshold_red], area_threshold=150, roi=img_roi, merge=True)
    green = img_contrast.find_blobs([threshold_green], area_threshold=150, roi=img_roi, merge=True)
    pink = img_contrast.find_blobs([threshold_pink], area_threshold=150, roi=img_roi, merge=True)

    center_x = 0
    center_y = 0
    p_center_x = 0
    p_center_y = 0

    if red:
        for b in red:
            img_debug.draw_rectangle(b.rect(), color=(255, 0, 0))
            center_x = b.x() + (b.w() // 2)
            center_y = b.y() + (b.h() // 2)
            img_debug.draw_cross(center_x, center_y, color=(255, 0, 0))
            val = b.y() + b.h()
            if val > distance_cap:
                blocks += 1
            if val > red_val:
                nearestRed = b
                red_val = val

    if green:
        for b in green:
            img_debug.draw_rectangle(b.rect(), color=(0, 255, 0))
            center_x = b.x() + (b.w() // 2)
            center_y = b.y() + (b.h() // 2)
            img_debug.draw_cross(center_x, center_y, color=(0, 255, 0))
            val = b.y() + b.h()
            if val > distance_cap:
                blocks += 1
            if val > green_val:
                nearestGreen = b
                green_val = val

    if pink:
        for b in pink:
            img_debug.draw_rectangle(b.rect(), color=(255, 0, 255))
            p_center_x = b.x() + (b.w() // 2)
            p_center_y = b.y() + (b.h() // 2)
            img_debug.draw_cross(p_center_x, p_center_y, color=(255, 0, 255))
            val = b.y() + b.h()
            if val > distance_cap:
                blocks += 1
            if val > pink_val:
                nearestPink = b
                pink_val = val

    if red_val == 0 and green_val == 0:
        block = {"center_x": center_x, "center_y": center_y, "color": 0, "p_center_x": p_center_x, "p_center_y": p_center_y}
        return block

    if nearestRed and nearestGreen:
        if red_val >= green_val:
            color = 2
        else:
            color = 1
    elif nearestRed:
        color = 2
    elif nearestGreen:
        color = 1
    else:
        color = 0

    block = {"center_x": center_x, "center_y": center_y, "color": color, "p_center_x": p_center_x, "p_center_y": p_center_y}
    return block

def check_corner_roi(img, img_debug):
    img_contrast = img.copy()
    img_contrast.gamma_corr(gamma=1.9, contrast=1.1, brightness=-0.1)
    threshold_black = (0, 37, -128, 15, -54, 37)
    img_debug.draw_rectangle(img_roi, color=(0, 0, 255))
    img_debug.draw_rectangle(img_roi_corner, color=(255, 255, 0))
    black = img_contrast.find_blobs([threshold_black], area_threshold=150, roi=img_roi_corner, merge=True)
    if black:
        black_val = 1
    else:
        black_val = 0
    corner = {"black": black_val}
    return corner

while True:
    img_debug = sensor.snapshot()
    img = img_debug.copy()
    img_debug.draw_cross(160, 120, color=(0, 0, 0))
    block = find_block(img, img_debug, 30)
    corner = check_corner_roi(img, img_debug)
    data = (block["color"], block["center_x"], block["center_y"], corner["black"])
    p.update_channel('cam', *data)
    p.process()
```
