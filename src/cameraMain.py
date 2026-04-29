# Based on OpenMV line follower example
# Run this code on an OpenMV H7 plus camera. It probably works on the RT1060 too..

import sensor
from pupremote import PUPRemoteSensor
from pyb import Pin, Timer


def msg(txt):
    print(txt)
    return txt+txt


# 50kHz pin6 timer2 channel1
light = Timer(2, freq=50000).channel(1, Timer. PWM, pin=Pin("P6"))
light.pulse_width_percent(100)  # adjust light 0~100

sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
# sensor.set_vflip(True)
sensor.set_hmirror(False)
sensor.set_auto_gain(False)  # must be turned off for color tracking
sensor.set_auto_whitebal(False)  # must be turned off for color tracking
sensor.set_auto_exposure(False)
sensor.skip_frames(time=2000)

p = PUPRemoteSensor(power=True)

# Send and receive any object with 'repr' encoding
p.add_command('msg', "repr", "repr")
p.add_channel('color', to_hub_fmt='f')  # integer (0: none, 1: green, 2: red)
p.add_channel('bl_x', to_hub_fmt='f')  # integer center x of rectangle
p.add_channel('bl_y', to_hub_fmt='f')  # integer center y of rectangle

img_center = (160, 120)  # Image center pixel (for alignment)
img_roi = (5, 90, 310, 90)  # ROI for color block detection
roi_rect = (img_roi[0], img_roi[1], img_roi[2], img_roi[3])


def find_block(img, img_debug, distance_cap):
    img_contrast = img.copy()
    img_contrast.gamma_corr(gamma=1.9, contrast=1.1, brightness=-0.1)
    color = 0
    # blob = None
    nearestRed = None
    nearestGreen = None
    red_val = 0
    green_val = 0
    blocks = 0

    # thresholds in LAB format for color filtering
    threshold_red = (17, 60, 22, 54, -2, 35)
    # (23, 61, 16, 54, 0, 22)
    # (0, 47, 38, 54, 22, 55)
    # (0, 45, 23, 127, 5, 31)
    # (0, 32, 10, 127, 9, 127)
    # (0, 31, 10, 127, 9, 127)
    # (20, 45, 25, 40, 10, 127)
    # (0, 45, 25, 49, 13, 127)
    # (0, 46, 18, 49, 13, 127)
    # (0, 43, 9, 127, 4, 127)
    # (0, 35, 10, 127, -10, 127)
    # (0, 43, 10, 127, -12, 127)

    threshold_green = (0, 100, -98, -25, -10, 127)
    # (22, 49, -128, -23, -128, 127)
    # (25, 50, -65, -25, 21, 79)
    # (25, 50, -65, -25, 21, 79)
    # (20, 45, -40, -15, 4, 57)
    # (21, 43, -128, -19, 16, 127)
    # (0, 100, -56, -25, 17, 50)
    # (28, 46, -56, -25, 17, 50)
    # (20, 45, -40, -15, 4, 57)
    # (20, 54, -40, -17, 0, 57)
    # (0, 100, -98, -25, -10, 127)

    # detect blobs with minimum size; no merging to prevent combining distant blocks
    img_debug.draw_rectangle(img_roi, color=(0, 0, 255))
    red = img_contrast.find_blobs([threshold_red], area_threshold=150, roi=img_roi, merge=True)
    green = img_contrast.find_blobs([threshold_green], area_threshold=150, roi=img_roi, merge=True)

    center_x = 0
    center_y = 0

    if red:  # evaluate all red blobs
        for b in red:
            img_debug.draw_rectangle(b.rect(), color=(255, 0, 0))

            center_x = b.x() + (b.w() // 2)
            center_y = b.y() + (b.h() // 2)

            img_debug.draw_cross(center_x, center_y, color=(255, 0, 0))

            val = b.y() + b.h()
            if val > distance_cap:
                blocks += 1
            if val > red_val:  # update nearest red
                nearestRed = b
                red_val = val

    if green:  # valuate all green blobs
        for b in green:
            img_debug.draw_rectangle(b.rect(), color=(0, 255, 0))

            center_x = b.x() + (b.w() // 2)
            center_y = b.y() + (b.h() // 2)

            img_debug.draw_cross(center_x, center_y, color=(0, 255, 0))

            val = b.y() + b.h()   # (2*b.pixels())-b.area()
            if val > distance_cap:
                blocks += 1
            if val > green_val:  # update nearest green
                nearestGreen = b
                green_val = val

    if red_val == 0 and green_val == 0:
        block = {"center_x": center_x, "center_y": center_y, "color": 0}
        return block

    # choose the blob with the higher score
    if nearestRed and nearestGreen:
        if red_val >= green_val:
            color = 2
            # blob = nearestRed
        else:
            color = 1
            # blob = nearestGreen
    elif nearestRed:
        color = 2
        # blob = nearestRed
    elif nearestGreen:
        color = 1
        # blob = nearestGreen
    else:
        color = 0
        # blob = None

    block = {"center_x": center_x, "center_y": center_y, "color": color}

    return block  # return block info and color if found


def find_rect(img, img_debug, img_roi):
    nearest_dist = 0
    nearestBlock = None

    global roi_rect

    center = {"center_x": -100, "center_y": -100}

    for rect in img_debug.find_rects(roi=img_roi, threshold=10000):
        dist = rect.y() + rect.h()
        # print("dist: ", dist)

        if dist > 0 and dist > nearest_dist and insideROI(roi_rect, rect):  # update nearest block
            nearestBlock = rect
            nearest_dist = dist

    if nearestBlock is not None:
        img_debug.draw_rectangle(nearestBlock.rect(), color=(255, 255, 0))

        center_x = nearestBlock.x() + (nearestBlock.w() // 2)
        center_y = nearestBlock.y() + (nearestBlock.h() // 2)

        # Draw the center point
        img_debug.draw_cross(center_x, center_y, color=(255, 255, 0))
        center = {"center_x": center_x, "center_y": center_y}

    return center


def insideROI(roi, rect):

    inside = False

    roi_x1 = roi[0]
    roi_y1 = roi[1]
    roi_x2 = roi[0] + roi[2]
    roi_y2 = roi[1] + roi[3]

    rect_x1 = rect.x()
    rect_y1 = rect.y()
    rect_x2 = rect.x() + rect.w()
    rect_y2 = rect.y() + rect.h()

    if rect_x1 >= roi_x1 and rect_y1 >= roi_y1 and rect_x2 <= roi_x2 and rect_y2 <= roi_y2:
        inside = True

    return inside


while True:
    img_debug = sensor.snapshot()
    img = img_debug.copy()  # clean image

    img_debug.draw_cross(160, 120, color=(0, 0, 0))

    # center = find_rect(img, img_debug, img_roi)
    block = find_block(img, img_debug, 30)

    print("block_x: ", block["center_x"])

    p.update_channel('color', block["color"])
    p.process()
    p.update_channel('bl_x', block["center_x"])
    p.process()
    p.update_channel('bl_y', block["center_y"])
    p.process()

