#!/usr/bin/env python3

"""KoreaBot file 1: follow the line - camera in, /cmd_vel out.

It does what it is told on the /Linefollow topic (see topics.py):

    Line follower:True  / True     drive              (starts False: robot still)
    Line follower:False / False    stop
    White:True  / White:False      follow the white line on the right or not   (starts True)
    Yellow:True / Yellow:False     follow the yellow line on the left or not   (starts False)

    only White   ->  keep the white line in its ring on the right
    only Yellow  ->  keep the yellow line in its ring on the left
    both         ->  drive in the middle between them (center_x); if one
                     drops out, the middle is guessed from the other one and
                     the last lane width seen
    neither      ->  stand still, even with Line follower:True

Each line is tracked by a dot that locks onto it and slides along with it.
The robot steers to bring the dot (or the middle) onto its target, slowing
down the harder it turns.  A lost line: keep going for lost_grace, then turn
towards the line's side to look for it (search_time), back the other way,
then give up - stopped, and Line follower goes back to False.

Settings arrive on /Linefollow/colors (color_params window) and
/Linefollow/params (robot_params window).  Until they do, it uses the saved
files config/colors.py and config/robot_params.py - and it re-reads those if
they are edited by hand.  What it is doing goes out on /Linefollow/status.

The window shows the camera: the top crop_top is darkened (tracking ignores
it), each dot (green = locked on, red = lost) and its target ring.
    click near a line   snap that line's dot onto it
    m  camera / line mask    c  save a snapshot    q  quit (Esc, closing the window, Ctrl+C)

    ros2 run koreabot line_follow
    ros2 run koreabot line_follow --preview     # window only, never drives
"""

import json
import os
import signal
import sys
import time

import cv2
import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from koreabot import topics
from koreabot.cmd_vel import VelocityPublisher
from koreabot.settings import COLORS, ROBOT
from koreabot.vision import WHITE, YELLOW, clean_mask, decode, find_runs, line_mask


# error = 1.0 เมื่อห่างเป้าเท่านี้ (สัดส่วนความกว้างภาพ) kp คิดจากสเกลนี้
ERROR_SPAN = 0.25

FILE_CHECK_EVERY = 0.5    # วิ ดูว่าไฟล์ค่าถูกแก้ด้วยมือไหม
STATUS_EVERY = 0.25       # วิ ส่ง /Linefollow/status
CMD_EVERY = 0.1           # วิ ส่ง /cmd_vel ซ้ำอย่างน้อยทุกเท่านี้

WINDOW = "KoreaBot line follow"
VIEW_W = 480              # ภาพกล้องบนจอย่อให้กว้างเท่านี้ การมองยังใช้ภาพเต็ม
STALE_WARN = 1.0
FONT = cv2.FONT_HERSHEY_SIMPLEX

DRIVING = ("FOLLOW", "LOST", "SEARCH")


class LineFollow(Node):

    def __init__(self, preview=False):
        super().__init__("line_follow")
        self.preview = preview

        # สวิตช์ที่สั่งผ่าน /Linefollow  เริ่มต้น: หยุดนิ่ง ตามเส้นขาว
        self.declare_parameter("white", True)
        self.declare_parameter("yellow", False)
        self.switches = {
            "line_follower": False,
            "white": bool(self.get_parameter("white").value),
            "yellow": bool(self.get_parameter("yellow").value),
        }

        # ค่าตั้ง: จากไฟล์ก่อน แล้วค่าจากหน้าต่างปรับค่า (topic) ทับทีหลัง
        self.colors, errors = COLORS.load()
        self.robot, more = ROBOT.load()
        for error in errors + more:
            self.get_logger().warn(f"Settings file: {error}")
        self.file_mtimes = (COLORS.mtime(), ROBOT.mtime())
        self.next_file_check = 0.0

        self.cmd_pub = VelocityPublisher(self)
        self.status_pub = self.create_publisher(String, topics.STATUS, 10)
        self.debug_pub = self.create_publisher(CompressedImage, topics.DEBUG_IMAGE, 1)

        self.create_subscription(String, topics.LINEFOLLOW, self.on_command, 10)
        self.create_subscription(String, topics.COLORS, self.on_colors, topics.LATCHED)
        self.create_subscription(String, topics.PARAMS, self.on_params, topics.LATCHED)

        self.image_topic = None
        self.image_sub = None
        self.subscribe_camera(topics.CAMERA)
        self.next_topic_scan = time.monotonic() + 3.0

        self.frame = None
        self.frame_time = 0.0
        self.frame_seq = 0
        self.seen_seq = -1
        self.fps = 0.0

        # การตามเส้น
        self.dots = {WHITE: None, YELLOW: None}   # ตำแหน่งจุดแต่ละสี (พิกเซลภาพเต็ม)
        self.lane_width = None                    # ระยะเหลือง-ขาวล่าสุด สัดส่วนความกว้าง
        self.last_alone = None                    # เห็นแค่สีไหนเป็นสีสุดท้าย ใช้เลือกทางกวาดหา
        self.reading = None
        self.debug = None

        # การขับ
        self.state = "WAITING"
        self.prev_error = None
        self.prev_error_time = 0.0
        self.error_rate = 0.0
        self.last_seen = 0.0
        self.search_start = None
        self.cmd = (0.0, 0.0)
        self.last_cmd_time = 0.0

        self.show_mask = False
        self.view_size = (VIEW_W, 1)
        self.start_time = time.monotonic()
        self.next_report = self.start_time + 5.0
        self.next_status = 0.0

        cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_GUI_NORMAL)
        cv2.setMouseCallback(WINDOW, self.on_mouse)

        self.get_logger().info(
            f"{'PREVIEW - will not drive. ' if preview else ''}"
            f"Waiting for commands on {topics.LINEFOLLOW}: "
            f"{topics.command_text(**self.switches)}")

    ########################################################
    # topics เข้า
    ########################################################

    def on_command(self, msg):
        switches, errors = topics.parse_command(msg.data)
        for error in errors:
            self.get_logger().warn(f"{topics.LINEFOLLOW}: {error}")
        if not switches:
            return

        old = dict(self.switches)
        self.switches.update(switches)

        if self.switches["line_follower"] and not old["line_follower"]:
            # เริ่มใหม่ทุกครั้ง ไม่ให้ค้างสถานะกวาดหาจากรอบก่อน
            self.reset_tracking()
        if not self.switches["line_follower"] and old["line_follower"] and not self.preview:
            self.cmd_pub.stop()
        if (self.switches["white"], self.switches["yellow"]) != (old["white"], old["yellow"]):
            self.reset_tracking()

        self.get_logger().info(f"Command '{msg.data}' -> {topics.command_text(**self.switches)}")

    def on_colors(self, msg):
        self.apply(msg.data, COLORS, "colors", "Colours")

    def on_params(self, msg):
        self.apply(msg.data, ROBOT, "robot", "Robot settings")

    def apply(self, text, group, attr, label):
        try:
            raw = json.loads(text)
        except json.JSONDecodeError as error:
            self.get_logger().warn(f"{label}: not JSON ({error})")
            return
        old = getattr(self, attr)
        new, errors = group.validate(raw, old)
        for error in errors:
            self.get_logger().warn(f"{label} refused: {error}")
        changed = [f"{k} {old[k]} -> {new[k]}" for k in new if new[k] != old[k]]
        setattr(self, attr, new)
        self.seen_seq = -1      # วัดเฟรมเดิมใหม่ด้วยค่าใหม่ทันที
        if changed:
            self.get_logger().info(f"{label}: " + ", ".join(changed))

    def check_files(self):
        """Settings files edited by hand (and no window running) - use them too."""
        mtimes = (COLORS.mtime(), ROBOT.mtime())
        if mtimes == self.file_mtimes:
            return
        self.file_mtimes = mtimes
        for group, attr, label in ((COLORS, "colors", "Colours"), (ROBOT, "robot", "Robot settings")):
            values, errors = group.load(base=getattr(self, attr))
            for error in errors:
                self.get_logger().warn(f"{os.path.basename(group.path)}: {error}")
            self.apply(json.dumps(values), group, attr, label)

    ########################################################
    # กล้อง
    ########################################################

    def subscribe_camera(self, topic):
        # sensor QoS = best effort เฟรมหายก็ข้ามไป ห้าม reliable จะไล่ส่งซ้ำจนภาพค้างยาว
        if self.image_sub is not None:
            self.destroy_subscription(self.image_sub)
        self.image_topic = topic
        self.image_sub = self.create_subscription(
            CompressedImage, topic, self.on_image, qos_profile_sensor_data)
        self.get_logger().info(f"Camera topic: {topic}")

    def on_image(self, msg):
        image = decode(msg)
        if image is None:
            self.get_logger().warn("Could not decode a camera frame", throttle_duration_sec=5.0)
            return
        now = time.monotonic()
        if self.frame is not None and now > self.frame_time:
            self.fps = 0.8 * self.fps + 0.2 / (now - self.frame_time)
        self.frame = image
        self.frame_time = now
        self.frame_seq += 1

    def find_camera(self, now):
        """No frames yet: try another compressed image topic that exists."""
        if now < self.next_topic_scan:
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
    # การมอง
    ########################################################

    def reset_tracking(self):
        self.dots = {WHITE: None, YELLOW: None}
        self.prev_error = None
        self.last_seen = time.monotonic()
        self.search_start = None
        self.seen_seq = -1

    def following(self):
        """"white", "yellow", "both" or None - from the White / Yellow switches."""
        white, yellow = self.switches["white"], self.switches["yellow"]
        if white and yellow:
            return "both"
        return WHITE if white else YELLOW if yellow else None

    def track(self, work, colour, cut, mask):
        """Slide colour's dot onto its line in the strip at dot_y."""

        r, c = self.robot, self.colors
        h, w = work.shape[:2]

        if colour == WHITE:
            x0, x1, ring = int(r["scan_from"] * w), w, r["target_x"] * w
        else:
            x0, x1, ring = 0, int(r["yellow_scan_to"] * w), r["yellow_target_x"] * w
        x0 = max(0, min(w - 2, x0))
        x1 = max(x0 + 2, min(w, x1))

        yc = int(r["dot_y"] * h)
        rad = r["dot_size"]
        y0 = max(0, min(h - 2, yc - rad))
        y1 = min(h, max(y0 + 2, yc + rad + 1))

        cols = mask[y0:y1, x0:x1].mean(axis=0) / 255.0
        runs = [(a, b) for a, b in find_runs(cols >= c["col_ratio"]) if b - a >= c["min_width_px"]]

        spot = {"strip": (x0, y0 + cut, x1, y1 + cut), "y": yc + cut, "ring": ring,
                "found": False, "x": self.dots[colour]}
        if not runs:
            return spot     # ไม่เห็นเส้น จุดค้างที่เดิม เฟรมหน้าหาใกล้ ๆ ที่เดิมก่อน

        def centre(run):
            a, b = run
            weights = cols[a:b]
            return x0 + a + float(np.dot(np.arange(b - a), weights) / weights.sum())

        centres = [centre(run) for run in runs]
        old = self.dots[colour]
        # หาใกล้ที่เดิมก่อน (ภายใน dot_reach) จะได้เกาะเส้นเดิม ไม่กระโดดไปก้อนอื่น
        near = [x for x in centres if old is not None and abs(x - old) <= r["dot_reach"] * w]
        if near:
            seen = min(near, key=lambda x: abs(x - old))
            dot = r["dot_smooth"] * old + (1.0 - r["dot_smooth"]) * seen
        else:
            # หลุดเส้น = คว้าก้อนที่ใกล้วงเป้าของสีนี้ที่สุด (เส้นควรอยู่ตรงนั้น)
            dot = min(centres, key=lambda x: abs(x - ring))

        self.dots[colour] = dot
        spot["found"] = True
        spot["x"] = dot
        return spot

    def sense(self, frame):
        """Track the selected lines; work out where the robot is vs where it should be."""

        r = self.robot
        mode = self.following()

        # การตามเส้นไม่ดูส่วนบน crop_top เลย พิกัด y ที่ส่งออกไปบวก cut กลับเป็นของภาพเต็ม
        cut = int(r["crop_top"] * frame.shape[0])
        work = frame[cut:]
        w = work.shape[1]

        reading = {"cut": cut, "mode": mode, "spots": {}, "lights": [],
                   "found": False, "pos": None, "target": None, "error": None}

        for colour in (YELLOW, WHITE):
            if mode not in (colour, "both"):
                continue
            mask, lights = clean_mask(work, self.colors, colour)
            reading["lights"] += [((x, y + cut), rad) for (x, y), rad in lights]
            reading["spots"][colour] = self.track(work, colour, cut, mask)

        spots = reading["spots"]
        if mode == WHITE:
            reading["target"] = r["target_x"] * w
            pos = spots[WHITE]["x"] if spots[WHITE]["found"] else None
        elif mode == YELLOW:
            reading["target"] = r["yellow_target_x"] * w
            pos = spots[YELLOW]["x"] if spots[YELLOW]["found"] else None
        elif mode == "both":
            reading["target"] = r["center_x"] * w
            yellow, white = spots[YELLOW], spots[WHITE]
            lane = self.lane_width if self.lane_width is not None else r["lane_width"]
            if yellow["found"] and white["found"] and white["x"] > yellow["x"]:
                self.lane_width = (white["x"] - yellow["x"]) / w
                pos = (yellow["x"] + white["x"]) / 2
            elif yellow["found"]:
                pos = yellow["x"] + lane * w / 2      # เห็นแค่เหลือง เดากลางเลนจากความกว้างเลนเดิม
                self.last_alone = YELLOW
            elif white["found"]:
                pos = white["x"] - lane * w / 2
                self.last_alone = WHITE
            else:
                pos = None
        else:
            pos = None

        if pos is not None:
            reading["found"] = True
            reading["pos"] = pos
            reading["error"] = (pos - reading["target"]) / (ERROR_SPAN * w)
        return reading

    ########################################################
    # การขับ
    ########################################################

    def steer(self, error, now):
        """(forward m/s, turn rad/s) to bring the line back to its target."""

        r = self.robot

        # ลูปหมุน ~50 รอบ/วิ แต่ภาพมา ~11 เฟรม/วิ เฟรมเดิมใช้อัตราเดิม ไม่งั้น kd ไม่มีผล
        if self.prev_error is None:
            self.error_rate = 0.0
        elif now > self.prev_error_time:
            self.error_rate = (error - self.prev_error) / (now - self.prev_error_time)
        self.prev_error = error
        self.prev_error_time = now

        # อยู่ขวาเป้า (error บวก) = เลี้ยวขวา angular.z ลบ
        turn = -(r["kp"] * error + r["kd"] * self.error_rate)
        if r["invert_steering"]:
            turn = -turn
        turn = max(-r["max_turn"], min(r["max_turn"], turn))

        # ยิ่งเลี้ยวแรง ยิ่งช้า (เต็ม max_turn = ลด slow_down เต็ม) แต่ไม่หยุด
        turning = abs(turn) / r["max_turn"] if r["max_turn"] > 0 else 0.0
        speed = r["forward_speed"] * (1.0 - r["slow_down"] * turning)
        speed = max(min(r["min_speed"], r["forward_speed"]), speed)
        return speed, turn

    def search_dir(self):
        """+1 = turn left, -1 = turn right: always towards the line's side."""
        mode = self.following()
        if mode == YELLOW or (mode == "both" and self.last_alone == YELLOW):
            direction = 1.0
        else:
            direction = -1.0
        return -direction if self.robot["invert_steering"] else direction

    def decide(self, now):
        r = self.robot
        reading = self.reading

        if self.preview:
            return "PREVIEW", 0.0, 0.0
        if not self.switches["line_follower"]:
            return "STOPPED", 0.0, 0.0
        if reading["mode"] is None:
            return "NO LINE SELECTED", 0.0, 0.0
        if now - self.frame_time > r["image_timeout"]:
            return "CAMERA LOST", 0.0, 0.0

        if reading["found"]:
            self.last_seen = now
            self.search_start = None
            speed, turn = self.steer(reading["error"], self.frame_time)
            return "FOLLOW", speed, turn

        # เส้นหายแวบเดียว (เฟรมเบลอ รอยต่อเทป) วิ่งคำสั่งเดิมต่อไปก่อน
        if now - self.last_seen < r["lost_grace"]:
            return "LOST", self.cmd[0], self.cmd[1]

        if r["search_enabled"]:
            if self.search_start is None:
                self.search_start = now
                self.get_logger().info(
                    f"Line lost - searching {'left' if self.search_dir() > 0 else 'right'} first")
            t = now - self.search_start
            leg = r["search_time"]
            # ขาแรกไปฝั่งเส้น ขาสองกลับ 2 เท่า (เลยจุดเริ่มไปอีกฝั่ง) แล้วจบ
            if t < 3.0 * leg:
                direction = self.search_dir() if t < leg else -self.search_dir()
                return "SEARCH", r["search_speed"], direction * r["search_rate"]

        # หาไม่เจอ หยุด แล้วรอคำสั่ง Line follower:True ใหม่
        self.switches["line_follower"] = False
        self.search_start = None
        self.get_logger().error(
            "Could not find the line - stopped. Line follower is now False; "
            "put it back on the line and send Line follower:True.")
        return "GAVE UP", 0.0, 0.0

    def send(self, state, speed, turn, now):
        """Publish at CMD_EVERY, and immediately whenever the command changes."""
        if self.preview:
            return
        driving = state in DRIVING
        if not driving:
            speed, turn = 0.0, 0.0
        changed = (speed, turn) != self.cmd or state != self.state
        if changed or now - self.last_cmd_time >= CMD_EVERY:
            if not driving and changed:
                self.cmd_pub.stop()           # หยุดส่งซ้ำหลายครั้ง กัน WiFi ทำหาย
            elif driving or self.switches["line_follower"]:
                cmd = Twist()
                cmd.linear.x = float(speed)
                cmd.angular.z = float(turn)
                self.cmd_pub.publish(cmd)
            self.last_cmd_time = now
        self.cmd = (speed, turn)

    ########################################################
    # ลูปหลัก - main() เรียกทุกรอบหลัง spin_once
    ########################################################

    def step(self):
        now = time.monotonic()

        if now >= self.next_file_check:
            self.next_file_check = now + FILE_CHECK_EVERY
            self.check_files()

        if now >= self.next_status:
            self.next_status = now + STATUS_EVERY
            self.publish_status(now)

        if self.frame is None:
            self.find_camera(now)
            self.wait_for_camera(now)
            return

        if self.frame_seq != self.seen_seq:
            self.seen_seq = self.frame_seq
            self.reading = self.sense(self.frame)
            self.debug = self.draw(self.frame, self.reading)
            self.publish_debug()

        state, speed, turn = self.decide(now)
        if state != "FOLLOW":
            self.prev_error = None    # กลับมา FOLLOW แล้ว kd จะได้ไม่กระตุก
        if state != self.state:
            self.get_logger().info(f"State: {self.state} -> {state}")
        self.send(state, speed, turn, now)
        self.state = state

        self.show(now)

    def publish_status(self, now):
        r = self.reading
        spots = {} if r is None else r["spots"]
        status = {
            "state": self.state,
            "line_follower": self.switches["line_follower"],
            "white": self.switches["white"],
            "yellow": self.switches["yellow"],
            "following": self.following(),
            "white_found": bool(spots.get(WHITE, {}).get("found")),
            "yellow_found": bool(spots.get(YELLOW, {}).get("found")),
            "error": None if not (r and r["found"]) else round(r["error"], 3),
            "cmd_forward": round(self.cmd[0], 3),
            "cmd_turn": round(self.cmd[1], 3),
            "camera_fps": round(self.fps, 1),
            "frame_age_s": None if self.frame is None else round(now - self.frame_time, 2),
            "camera_topic": self.image_topic,
        }
        self.status_pub.publish(String(data=json.dumps(status)))

    def publish_debug(self):
        if self.debug_pub.get_subscription_count() == 0:
            return
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = cv2.imencode(".jpg", self.debug)[1].tobytes()
        self.debug_pub.publish(msg)

    ########################################################
    # หน้าต่าง
    ########################################################

    def draw(self, frame, r):
        debug = frame.copy()

        # ภาพเต็มถูกย่อลงจอ วาดให้หนาขึ้นตามอัตราย่อ เส้นจะได้ไม่หายตอนย่อ
        k = max(1.0, debug.shape[1] / VIEW_W)

        def px(n):
            return max(1, int(round(n * k)))

        cut = r["cut"]
        if cut:
            debug[:cut] = debug[:cut] // 2
            for x in range(0, debug.shape[1], px(16)):
                cv2.line(debug, (x, cut), (x + px(8), cut), (0, 165, 255), px(1))

        for center, radius in r["lights"]:
            cv2.circle(debug, center, radius, (255, 0, 255), px(2))

        dot_r = max(self.robot["dot_size"], px(7))
        ring_r = dot_r + px(6)
        both = r["mode"] == "both"

        for colour, spot in r["spots"].items():
            x0, _, x1, _ = spot["strip"]
            y = spot["y"]
            for x in range(x0, x1, px(12)):
                cv2.line(debug, (x, y), (min(x + px(5), x1), y), (160, 160, 160), px(1))
            if not both:
                cv2.circle(debug, (int(spot["ring"]), y), ring_r, (255, 255, 0), px(2))
            fill = (0, 200, 0) if colour == WHITE else (0, 200, 255)
            if spot["found"]:
                dx = int(spot["x"])
                cv2.circle(debug, (dx, y), dot_r + px(2), (255, 255, 255), -1)
                cv2.circle(debug, (dx, y), dot_r, fill, -1)
            elif spot["x"] is not None:
                dx = int(spot["x"])
                cv2.circle(debug, (dx, y), dot_r, (0, 0, 255), px(2))

        if both and r["spots"]:
            y = next(iter(r["spots"].values()))["y"]
            cv2.circle(debug, (int(r["target"]), y), ring_r, (255, 255, 0), px(2))
            if r["found"]:
                cv2.drawMarker(debug, (int(r["pos"]), y), (255, 255, 0),
                               cv2.MARKER_CROSS, px(18), px(2))
        return debug

    def show(self, now):
        base = self.debug
        if self.show_mask:
            work = np.zeros(self.frame.shape[:2], np.uint8)
            for colour in (WHITE, YELLOW):
                if self.switches[colour]:
                    work |= line_mask(self.frame, self.colors, colour)
            view = cv2.cvtColor(work, cv2.COLOR_GRAY2BGR)
            drawn = np.any(self.debug != self.frame, axis=2)
            view[drawn] = self.debug[drawn]
            base = view

        h, w = base.shape[:2]
        view = cv2.resize(base, (VIEW_W, max(1, h * VIEW_W // w)), interpolation=cv2.INTER_AREA)
        self.view_size = (view.shape[1], view.shape[0])
        vh = view.shape[0]

        r = self.reading
        if r["mode"] is None:
            text, colour = "NO LINE SELECTED (White / Yellow)", (0, 0, 255)
        elif r["found"]:
            text, colour = f"LOCKED  err {r['error']:+.2f}", (0, 255, 0)
        else:
            text, colour = "LOST - click the line", (0, 0, 255)
        cv2.putText(view, text, (5, 18), FONT, 0.5, colour, 1)

        label = {WHITE: "WHITE / RIGHT", YELLOW: "YELLOW / LEFT", "both": "BOTH / MIDDLE",
                 None: "NO LINE"}[r["mode"]]
        (tw, _), _ = cv2.getTextSize(label, FONT, 0.5, 1)
        cv2.putText(view, label, (VIEW_W - tw - 5, 18), FONT, 0.5, (255, 255, 255), 1)

        stale = now - self.frame_time
        if stale > STALE_WARN:
            cv2.putText(view, f"NO NEW FRAMES {stale:.0f}s", (5, 40), FONT, 0.55, (0, 0, 255), 2)

        # แถบล่าง: สถานะสวิตช์จาก /Linefollow
        bar = np.full((28, VIEW_W, 3), 40, np.uint8)
        on = self.switches["line_follower"]
        cv2.putText(bar, f"{self.state}", (5, 19), FONT, 0.5,
                    (0, 220, 0) if self.state in DRIVING else (0, 200, 255), 1)
        switches = (f"LF:{'ON' if on else 'off'}  W:{'on' if self.switches['white'] else 'off'}"
                    f"  Y:{'on' if self.switches['yellow'] else 'off'}  {self.fps:.0f}fps")
        (tw, _), _ = cv2.getTextSize(switches, FONT, 0.45, 1)
        cv2.putText(bar, switches, (VIEW_W - tw - 5, 19), FONT, 0.45, (255, 255, 255), 1)

        self.present(np.vstack([view, bar]))

    def wait_for_camera(self, now):
        view = np.zeros((270, VIEW_W, 3), np.uint8)
        cv2.putText(view, f"Waiting {now - self.start_time:.0f}s for camera", (15, 120),
                    FONT, 0.5, (255, 255, 255), 1)
        cv2.putText(view, "Robot camera running? ROS_DOMAIN_ID right?", (15, 145),
                    FONT, 0.42, (0, 165, 255), 1)
        self.present(view)

        if now < self.next_report:
            return
        self.next_report = now + 5.0
        seen = {name for name, _ in self.get_topic_names_and_types()}
        if "/odom" in seen or "/scan" in seen:
            self.get_logger().warn(
                f"Still waiting for {self.image_topic} - the robot is on the ROS graph but "
                "sends no camera. Start the Pi camera: ros2 run camera_ros camera_node")
        else:
            self.get_logger().warn(
                "Nothing from the robot on the ROS graph. "
                f"ROS_DOMAIN_ID={os.environ.get('ROS_DOMAIN_ID', '(unset)')} "
                f"ROS_STATIC_PEERS={os.environ.get('ROS_STATIC_PEERS') or '(unset)'}")

    def present(self, image):
        cv2.imshow(WINDOW, image)
        key = cv2.waitKey(1) & 0xFF
        closed = cv2.getWindowProperty(WINDOW, cv2.WND_PROP_VISIBLE) < 1
        if key in (ord('q'), 27) or closed:
            raise KeyboardInterrupt
        if key == ord('m'):
            self.show_mask = not self.show_mask
        if key == ord('c') and self.debug is not None:
            stamp = time.strftime("%Y%m%d-%H%M%S")
            path = os.path.expanduser(f"~/koreabot_{stamp}.png")
            cv2.imwrite(path, self.debug)
            cv2.imwrite(path.replace(".png", "_raw.png"), self.frame)
            self.get_logger().info(f"Saved {path}")

    def on_mouse(self, event, x, y, flags, param):
        """Click near a line: snap that line's dot onto it."""
        if event != cv2.EVENT_LBUTTONDOWN or self.frame is None or self.reading is None:
            return
        fx = x * self.frame.shape[1] / self.view_size[0]
        spots = self.reading["spots"]
        if not spots:
            return
        # สองเส้น: คลิกซ้ายของกลางเลน = เหลือง ขวา = ขาว
        if len(spots) == 2:
            colour = YELLOW if fx < self.robot["center_x"] * self.frame.shape[1] else WHITE
        else:
            colour = next(iter(spots))
        self.dots[colour] = fx
        self.seen_seq = -1


############################################################

def main(args=None):
    topics.setup_network()
    rclpy.init(args=args)

    # rclpy ดัก SIGTERM แล้วปิด context ทิ้ง พอถึง finally จะสั่งหยุดไม่ได้ หุ่นวิ่งค้าง
    # เปลี่ยนเป็น KeyboardInterrupt จะได้ลงทางเดิมที่หยุดหุ่นเป็น
    signal.signal(signal.SIGTERM, lambda signum, frame: (_ for _ in ()).throw(KeyboardInterrupt))

    node = None
    try:
        node = LineFollow(preview="--preview" in sys.argv)
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.02)
            node.step()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            if not node.preview:
                try:
                    node.cmd_pub.stop()
                except Exception:
                    pass
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
