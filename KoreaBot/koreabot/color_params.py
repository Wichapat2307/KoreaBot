#!/usr/bin/env python3

"""KoreaBot file 2: the colour settings window.

Shows the live camera with what counts as each line colour tinted on top
(blue = white line, green = yellow line, magenta circles = ceiling lights that
get ignored), and sliders for the colour settings:

    left click on the yellow tape    pick the yellow colour from it
    right click on the white tape    pick the white colour from it
    d  back to defaults    q  close (Esc or closing the window too)

Every change goes straight to line_follow on /Linefollow/colors and is saved
to config/colors.py, so it is remembered next time.  Editing that file by hand
works too - this window picks it up and sends it on.

    ros2 run koreabot color_params
"""

import json
import signal
import time

import cv2
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from koreabot import topics
from koreabot.settings import COLORS
from koreabot.vision import WHITE, YELLOW, clean_mask, decode


WINDOW = "KoreaBot colours"
VIEW_W = 480
FONT = cv2.FONT_HERSHEY_SIMPLEX

SAVE_AFTER = 0.5          # วิ หลังเลื่อนแถบครั้งสุดท้ายค่อยเขียนไฟล์
FILE_CHECK_EVERY = 0.5

# (ชื่อแถบ, ชื่อค่า, ค่าสูงสุดของแถบ, คูณ) แถบรับแค่จำนวนเต็ม ค่าทศนิยมคูณก่อนแล้วหารกลับ
SLIDERS = [
    ("white S max", "s_max", 255, 1),
    ("white V min", "v_min", 255, 1),
    ("yellow H low", "h_low", 179, 1),
    ("yellow H high", "h_high", 179, 1),
    ("yellow S low", "s_low", 255, 1),
    ("yellow V low", "v_low", 255, 1),
    ("ignore lights", "ignore_round", 1, 1),
    ("light round %", "light_min_circularity", 100, 100),
    ("light margin x10", "light_margin", 50, 10),
]


class ColorParams(Node):

    def __init__(self):
        super().__init__("color_params")

        self.values, errors = COLORS.load()
        for error in errors:
            self.get_logger().warn(f"colors.py: {error}")
        self.file_mtime = COLORS.mtime()
        self.next_file_check = 0.0
        self.changed_at = None          # เลื่อนแถบแล้วยังไม่ได้เซฟ

        self.pub = self.create_publisher(String, topics.COLORS, topics.LATCHED)

        self.frame = None
        self.image_topic = None
        self.image_sub = None
        self.subscribe_camera(topics.CAMERA)
        self.next_topic_scan = time.monotonic() + 3.0

        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
        self.slider_last = {}
        for label, name, top, scale in SLIDERS:
            pos = self.to_slider(name, scale)
            cv2.createTrackbar(label, WINDOW, pos, top, lambda _: None)
            self.slider_last[label] = pos
        cv2.setMouseCallback(WINDOW, self.on_mouse)

        self.publish()
        self.get_logger().info(
            f"Colour window open - sending on {topics.COLORS}, saving to {COLORS.path}")

    ########################################################

    def subscribe_camera(self, topic):
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
        self.image_topic = topic
        self.image_sub = self.create_subscription(
            CompressedImage, topic, self.on_image, qos_profile_sensor_data)

    def on_image(self, msg):
        image = decode(msg)
        if image is not None:
            self.frame = image

    def find_camera(self, now):
        if self.frame is not None or now < self.next_topic_scan:
            return
        self.next_topic_scan = now + 2.0
        if self.count_publishers(self.image_topic) > 0:
            return
        names = sorted(
            (name for name, types in self.get_topic_names_and_types()
             if "sensor_msgs/msg/CompressedImage" in types and not name.startswith("/Linefollow")),
            key=lambda n: "image_raw" not in n)
        if names and names[0] != self.image_topic:
            self.subscribe_camera(names[0])

    ########################################################

    def to_slider(self, name, scale):
        return int(round(float(self.values[name]) * scale))

    def publish(self):
        self.pub.publish(String(data=json.dumps(self.values)))

    def set_values(self, raw, source):
        """Validate, apply, send to line_follow; the file is saved a moment later."""

        raw = dict(raw)
        # h_low เลยเกิน h_high = ดันอีกตัวตามไป ไม่งั้นโดนปฏิเสธ แถบเด้งไปมา
        h_low = raw.get("h_low", self.values["h_low"])
        h_high = raw.get("h_high", self.values["h_high"])
        if h_low > h_high:
            if "h_low" in raw:
                raw["h_high"] = h_low
            else:
                raw["h_low"] = h_high

        values, errors = COLORS.validate(raw, self.values)
        for error in errors:
            self.get_logger().warn(f"Refused: {error}")
        if values == self.values:
            return
        changed = [f"{k} {self.values[k]} -> {values[k]}" for k in values if values[k] != self.values[k]]
        self.values = values
        self.publish()
        self.changed_at = time.monotonic()
        self.get_logger().info(f"{source}: " + ", ".join(changed))

    def save(self):
        if self.changed_at is None:
            return
        self.changed_at = None
        COLORS.save(self.values)
        self.file_mtime = COLORS.mtime()     # เราเขียนเอง ไม่ต้องอ่านซ้ำ

    def on_mouse(self, event, x, y, flags, param):
        """Left click = pick yellow, right click = pick white, from the clicked tape."""

        if event not in (cv2.EVENT_LBUTTONDOWN, cv2.EVENT_RBUTTONDOWN) or self.frame is None:
            return
        h, w = self.frame.shape[:2]
        fx, fy = int(x * w / VIEW_W), int(y * w / VIEW_W)
        if not (0 <= fx < w and 0 <= fy < h):
            return

        patch = self.frame[max(0, fy - 4):fy + 5, max(0, fx - 4):fx + 5]
        hue, sat, val = np.median(
            cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3), axis=0).astype(int)

        # เผื่อช่วงรอบสีที่คลิกเป็นสัดส่วน เทปสีจางจะได้ไม่ได้ s_low 0 ที่นับเทา/ขาวเป็นเหลือง
        if event == cv2.EVENT_LBUTTONDOWN:
            raw = {"h_low": max(0, hue - 8), "h_high": min(179, hue + 8),
                   "s_low": sat // 2, "v_low": val // 2}
            colour = "yellow"
        else:
            raw = {"s_max": min(255, sat + 40), "v_min": val * 3 // 4}
            colour = "white"
        self.set_values(raw, f"Picked {colour} from H{hue} S{sat} V{val}")

    ########################################################

    def step(self):
        now = time.monotonic()
        self.find_camera(now)

        # ไฟล์ถูกแก้ด้วยมือ = โหลดแล้วส่งต่อ
        if now >= self.next_file_check:
            self.next_file_check = now + FILE_CHECK_EVERY
            mtime = COLORS.mtime()
            if mtime != self.file_mtime:
                self.file_mtime = mtime
                values, errors = COLORS.load(base=self.values)
                for error in errors:
                    self.get_logger().warn(f"colors.py: {error}")
                if values != self.values:
                    self.values = values
                    self.publish()
                    self.get_logger().info("colors.py edited by hand - sent to line_follow")

        # แถบ: คนเลื่อน = ใช้ค่านั้น / ค่าเปลี่ยนจากที่อื่น (คลิก ไฟล์) = ขยับแถบตาม
        moved = {}
        for label, name, top, scale in SLIDERS:
            pos = cv2.getTrackbarPos(label, WINDOW)
            if pos != self.slider_last[label]:
                moved[name] = bool(pos) if name == "ignore_round" else pos / scale
            self.slider_last[label] = pos
        if moved:
            self.set_values(moved, "Slider")
        for label, name, top, scale in SLIDERS:
            pos = self.to_slider(name, scale)
            if pos != self.slider_last[label]:
                cv2.setTrackbarPos(label, WINDOW, pos)
                self.slider_last[label] = pos

        if self.changed_at is not None and now - self.changed_at > SAVE_AFTER:
            self.save()

        self.show()

    def show(self):
        if self.frame is None:
            view = np.zeros((270, VIEW_W, 3), np.uint8)
            cv2.putText(view, "Waiting for the camera...", (15, 120), FONT, 0.5, (255, 255, 255), 1)
            cv2.putText(view, "sliders work already", (15, 145), FONT, 0.42, (0, 165, 255), 1)
        else:
            h, w = self.frame.shape[:2]
            view = cv2.resize(self.frame, (VIEW_W, max(1, h * VIEW_W // w)),
                              interpolation=cv2.INTER_AREA)
            scale = VIEW_W / w
            white, lights = clean_mask(view, self.values, WHITE)
            yellow, _ = clean_mask(view, self.values, YELLOW)
            view[white > 0] = (0.35 * view[white > 0] + (160, 80, 0)).astype(np.uint8)
            view[yellow > 0] = (0.35 * view[yellow > 0] + (0, 165, 0)).astype(np.uint8)
            for center, radius in lights:
                cv2.circle(view, center, radius, (255, 0, 255), 1)
            cv2.putText(view, f"blue = white {100 * (white > 0).mean():.1f}%   "
                        f"green = yellow {100 * (yellow > 0).mean():.1f}%",
                        (5, 16), FONT, 0.42, (255, 255, 255), 1)
            cv2.putText(view, "left click = pick yellow   right click = pick white",
                        (5, view.shape[0] - 8), FONT, 0.42, (255, 255, 0), 1)

        cv2.imshow(WINDOW, view)
        key = cv2.waitKey(1) & 0xFF
        closed = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1
        if key in (ord('q'), 27) or closed:
            raise KeyboardInterrupt
        if key == ord('d'):
            self.set_values(COLORS.defaults(), "Defaults")


def main(args=None):
    topics.setup_network()
    rclpy.init(args=args)
    signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(KeyboardInterrupt))
    node = None
    try:
        node = ColorParams()
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            node.step()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.save()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
