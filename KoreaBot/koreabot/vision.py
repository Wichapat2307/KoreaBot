"""Colour detection shared by line_follow and color_params.

Both use these exact functions, so what the colour window highlights is
exactly what the robot follows.
"""

import math

import cv2
import numpy as np


WHITE = "white"
YELLOW = "yellow"

# ปิดรูขอบขรุขระของแสงสะท้อนให้เป็นก้อนเดียวก่อนวัดความกลม (วิธีเดียวกับ ex.py)
LIGHT_KERNEL = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (9, 9))


def line_mask(image, c, colour):
    """Pixels of that line colour, using colour settings c.

    white:  any hue, pale (S <= s_max) and bright (V >= v_min)
    yellow: hue h_low - h_high, S >= s_low, V >= v_low
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    if colour == YELLOW:
        return cv2.inRange(hsv, np.array((c["h_low"], c["s_low"], c["v_low"])),
                           np.array((c["h_high"], 255, 255)))
    return cv2.inRange(hsv, np.array((0, 0, c["v_min"])),
                       np.array((179, c["s_max"], 255)))


def remove_lights(mask, min_circularity, margin):
    """Erase round blobs (ceiling light reflections) from a white mask in place.

    Same method as remove_lights() in ex.py: a blob whose convex hull is at
    least min_circularity round (1 = perfect circle) is a light, and a circle
    margin times its radius is blacked out so its glow goes too.
    Returns (center, radius) of each one erased.
    """
    closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, LIGHT_KERNEL)
    contours, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    lights = []
    for c in contours:
        hull = cv2.convexHull(c)
        perimeter = cv2.arcLength(hull, True)
        if perimeter == 0:
            continue
        circularity = 4 * math.pi * cv2.contourArea(hull) / perimeter ** 2
        if circularity < min_circularity:
            continue
        (x, y), r = cv2.minEnclosingCircle(hull)
        center, radius = (int(x), int(y)), int(r * margin)
        cv2.circle(mask, center, radius, 0, -1)
        lights.append((center, radius))
    return lights


def clean_mask(image, c, colour):
    """line_mask plus the light filter for white.  Returns (mask, lights)."""
    mask = line_mask(image, c, colour)
    lights = []
    if colour == WHITE and c["ignore_round"]:
        lights = remove_lights(mask, c["light_min_circularity"], c["light_margin"])
    return mask, lights


def find_runs(hot):
    """(start, end) of every run of True in a 1-D bool array, end exclusive."""
    edges = np.diff(np.concatenate(([0], hot.astype(np.int8), [0])))
    return list(zip(np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)))


def decode(msg):
    """sensor_msgs/CompressedImage -> BGR image, or None."""
    return cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
