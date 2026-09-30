#!/usr/bin/env python3

"""KoreaBot file 3: the robot settings window.

A slider for every robot setting - speed, kp, kd, turn speed, where the lines
should be, ... - grouped the same way as config/robot_params.py.  Every change
goes straight to line_follow on /Linefollow/params and is saved to
config/robot_params.py, so it is remembered next time.  Editing that file by
hand works too - this window picks it up and sends it on.

The strip at the top drives the robot through the /Linefollow topic:

    START / STOP        send "Line follower:True" / "Line follower:False"
    White / Yellow      send "White:True/False", "Yellow:True/False"

and the line under it shows what line_follow reports on /Linefollow/status.

    ros2 run koreabot robot_params
"""

import json
import math
import signal
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from koreabot import topics
from koreabot.settings import ROBOT


TICK_MS = 50              # ปั๊ม ROS + อัปเดตหน้าต่างทุกเท่านี้
SEND_EVERY = 0.1          # วิ ส่งค่าให้ line_follow ไม่เกินนี้ระหว่างลากแถบ
SAVE_AFTER = 0.5          # วิ หลังลากแถบครั้งสุดท้ายค่อยเขียนไฟล์
FILE_CHECK_EVERY = 0.5
STATUS_STALE = 2.0        # วิ ไม่ได้ยินสถานะนานเกินนี้ = line_follow ไม่ได้รัน

RED, GREEN, GREY, BLUE = "#c62828", "#2e7d32", "#666666", "#1565c0"
SMALL = ("TkDefaultFont", 8)


def step_for(low, high, is_int):
    """Slider step: whole numbers for ints, about 1/200 of the range otherwise."""
    if is_int:
        return 1
    return 10 ** math.floor(math.log10((high - low) / 200.0))


class RobotParams(Node):

    def __init__(self, root):
        super().__init__("robot_params")
        self.root = root

        self.values, errors = ROBOT.load()
        for error in errors:
            self.get_logger().warn(f"robot_params.py: {error}")
        self.file_mtime = ROBOT.mtime()
        self.next_file_check = 0.0
        self.dirty_send = False
        self.changed_at = None
        self.last_send = 0.0

        self.params_pub = self.create_publisher(String, topics.PARAMS, topics.LATCHED)
        self.cmd_pub = self.create_publisher(String, topics.LINEFOLLOW, 10)
        self.create_subscription(String, topics.STATUS, self.on_status, 10)
        self.status = None
        self.status_time = 0.0

        self.build()
        self.publish()
        self.get_logger().info(
            f"Robot settings window open - sending on {topics.PARAMS}, saving to {ROBOT.path}")

    ########################################################
    # หน้าต่าง
    ########################################################

    def build(self):
        root = self.root
        root.title("KoreaBot robot")
        root.resizable(False, True)

        # ---- ปุ่มสั่งงานผ่าน /Linefollow ----
        top = tk.Frame(root, padx=4, pady=4)
        top.pack(fill="x")
        big = ("TkDefaultFont", 10, "bold")
        tk.Button(top, text="START", font=big, bg=GREEN, fg="white", width=5, pady=1,
                  activebackground="#43a047", activeforeground="white",
                  command=lambda: self.command(line_follower=True)).pack(side="left")
        tk.Button(top, text="STOP", font=big, bg=RED, fg="white", width=5, pady=1,
                  activebackground="#e53935", activeforeground="white",
                  command=lambda: self.command(line_follower=False)).pack(side="left", padx=3)
        self.white_var = tk.BooleanVar(value=True)
        self.yellow_var = tk.BooleanVar(value=False)
        tk.Checkbutton(top, text="White", variable=self.white_var, font=SMALL,
                       command=lambda: self.command(white=self.white_var.get())).pack(side="left")
        tk.Checkbutton(top, text="Yellow", variable=self.yellow_var, font=SMALL,
                       command=lambda: self.command(yellow=self.yellow_var.get())).pack(side="left")

        self.status_text = tk.StringVar(value="...")
        self.status_label = tk.Label(root, textvariable=self.status_text, anchor="w",
                                     font=SMALL, padx=4)
        self.status_label.pack(fill="x")

        # ---- แถบค่า: รายการเลื่อนได้ ----
        area = tk.Frame(root, padx=4)
        area.pack(fill="both", expand=True)
        canvas = tk.Canvas(area, height=330, highlightthickness=0)
        bar = ttk.Scrollbar(area, orient="vertical", command=canvas.yview)
        canvas.configure(yscrollcommand=bar.set)
        bar.pack(side="right", fill="y")
        canvas.pack(side="left", fill="both", expand=True)
        form = tk.Frame(canvas)
        canvas.create_window((0, 0), window=form, anchor="nw")
        form.bind("<Configure>", lambda _e: canvas.configure(
            scrollregion=canvas.bbox("all"), width=form.winfo_reqwidth()))

        def wheel(event):
            canvas.yview_scroll(-1 if event.num == 4 or getattr(event, "delta", 0) > 0 else 1, "units")
        for seq in ("<Button-4>", "<Button-5>", "<MouseWheel>"):
            canvas.bind_all(seq, wheel)

        self.hint = tk.StringVar(value="Point at a setting for help")
        tk.Label(root, textvariable=self.hint, fg=GREY, font=SMALL, anchor="w", justify="left",
                 wraplength=300, padx=4).pack(fill="x")

        self.vars = {}
        self.value_labels = {}
        self.steps = {}
        row = 0
        for title, names in ROBOT.sections:
            tk.Label(form, text=title.split("  (")[0], font=("TkDefaultFont", 8, "bold"),
                     fg=BLUE, anchor="w").grid(row=row, column=0, columnspan=3, sticky="w", pady=(6, 0))
            row += 1
            for name in names:
                self.add_row(form, row, name)
                row += 1

        bottom = tk.Frame(root, padx=4, pady=4)
        bottom.pack(fill="x")
        tk.Button(bottom, text="Defaults", font=SMALL, padx=4, pady=0,
                  command=lambda: self.set_values(ROBOT.defaults(), "Defaults")).pack(side="left")
        self.saved_text = tk.StringVar(value="")
        tk.Label(bottom, textvariable=self.saved_text, fg=GREY, font=SMALL).pack(side="left", padx=6)

    def add_row(self, form, row, name):
        default, low, high, meaning = ROBOT.schema[name]
        label = tk.Label(form, text=name, font=SMALL, anchor="w")
        label.grid(row=row, column=0, sticky="w")

        if isinstance(default, bool):
            var = tk.BooleanVar(value=self.values[name])
            widget = tk.Checkbutton(form, variable=var, pady=0,
                                    command=lambda n=name, v=var: self.set_values({n: v.get()}, "Changed"))
            widget.grid(row=row, column=1, sticky="w")
            text = f"{name}: {meaning}"
        else:
            step = step_for(low, high, isinstance(default, int))
            self.steps[name] = step
            var = tk.DoubleVar(value=float(self.values[name]))
            widget = ttk.Scale(form, from_=low, to=high, variable=var, length=150,
                               command=lambda _v, n=name: self.on_slide(n))
            widget.grid(row=row, column=1, sticky="w", padx=2)
            shown = tk.Label(form, text=self.fmt(name, self.values[name]), font=SMALL, width=6, anchor="e")
            shown.grid(row=row, column=2, sticky="e")
            self.value_labels[name] = shown
            text = f"{name} [{low} - {high}]: {meaning}"

        for w in (label, widget):
            w.bind("<Enter>", lambda _e, t=text: self.hint.set(t), add="+")
        self.vars[name] = var

    def fmt(self, name, value):
        if isinstance(value, bool):
            return str(value)
        step = self.steps.get(name, 1)
        digits = max(0, -int(math.floor(math.log10(step)))) if step < 1 else 0
        return f"{value:.{digits}f}"

    ########################################################
    # ค่า
    ########################################################

    def on_slide(self, name):
        step = self.steps[name]
        raw = round(round(self.vars[name].get() / step) * step, 6)
        if isinstance(ROBOT.schema[name][0], int):
            raw = int(raw)
        if raw != self.values[name]:
            self.set_values({name: raw}, None)

    def set_values(self, raw, source):
        values, errors = ROBOT.validate(raw, self.values)
        for error in errors:
            self.get_logger().warn(f"Refused: {error}")
            self.saved_text.set(f"refused: {error}")
        if values != self.values:
            if source:
                changed = [f"{k} {self.values[k]} -> {values[k]}" for k in values if values[k] != self.values[k]]
                self.get_logger().info(f"{source}: " + ", ".join(changed))
            self.values = values
            self.dirty_send = True
            self.changed_at = time.monotonic()
            self.saved_text.set("sending...")
        self.refresh_widgets()

    def refresh_widgets(self):
        for name, var in self.vars.items():
            value = self.values[name]
            if isinstance(value, bool):
                if var.get() != value:
                    var.set(value)
            else:
                # ไม่ขยับแถบที่คนกำลังลากอยู่เกินจำเป็น ขยับเมื่อค่าต่างจริงเท่านั้น
                if abs(var.get() - value) > self.steps[name] / 2:
                    var.set(float(value))
                self.value_labels[name].config(text=self.fmt(name, value))

    def publish(self):
        self.params_pub.publish(String(data=json.dumps(self.values)))
        self.last_send = time.monotonic()

    def command(self, **switches):
        text = topics.command_text(**switches)
        self.cmd_pub.publish(String(data=text))
        self.get_logger().info(f"Sent '{text}' on {topics.LINEFOLLOW}")

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
            self.status_time = time.monotonic()
        except json.JSONDecodeError:
            pass

    ########################################################
    # ลูป
    ########################################################

    def tick(self):
        rclpy.spin_once(self, timeout_sec=0)
        now = time.monotonic()

        if self.dirty_send and now - self.last_send >= SEND_EVERY:
            self.dirty_send = False
            self.publish()
        if self.changed_at is not None and now - self.changed_at > SAVE_AFTER:
            self.changed_at = None
            ROBOT.save(self.values)
            self.file_mtime = ROBOT.mtime()      # เราเขียนเอง ไม่ต้องอ่านซ้ำ
            self.saved_text.set("saved")

        # ไฟล์ถูกแก้ด้วยมือ = โหลดแล้วส่งต่อ
        if now >= self.next_file_check:
            self.next_file_check = now + FILE_CHECK_EVERY
            mtime = ROBOT.mtime()
            if mtime != self.file_mtime and self.changed_at is None:
                self.file_mtime = mtime
                values, errors = ROBOT.load(base=self.values)
                for error in errors:
                    self.get_logger().warn(f"robot_params.py: {error}")
                if values != self.values:
                    self.values = values
                    self.refresh_widgets()
                    self.publish()
                    self.saved_text.set("file edited - sent")

        s = self.status
        if s is None or now - self.status_time > STATUS_STALE:
            self.status_text.set("line_follow is NOT running")
            self.status_label.config(fg=RED)
        else:
            line = f"err {s['error']:+.2f}" if s["error"] is not None else "no line"
            self.status_text.set(f"{s['state']} · {line} · {s['cmd_forward']:.2f} m/s")
            self.status_label.config(fg=GREEN if s["state"] in ("FOLLOW", "LOST", "SEARCH") else "black")
            # ติ๊ก White/Yellow ตามที่ line_follow ใช้อยู่จริง
            if self.white_var.get() != s["white"]:
                self.white_var.set(s["white"])
            if self.yellow_var.get() != s["yellow"]:
                self.yellow_var.set(s["yellow"])

        self.root.after(TICK_MS, self.tick)

    def close(self):
        if self.changed_at is not None:
            ROBOT.save(self.values)
        if self.dirty_send:
            self.publish()


def main(args=None):
    topics.setup_network()
    rclpy.init(args=args)
    root = tk.Tk()
    node = RobotParams(root)

    def quit_all(*_):
        root.quit()
    signal.signal(signal.SIGTERM, quit_all)
    signal.signal(signal.SIGINT, quit_all)
    root.protocol("WM_DELETE_WINDOW", quit_all)

    root.after(TICK_MS, node.tick)
    try:
        root.mainloop()
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        try:
            root.destroy()
        except tk.TclError:
            pass


if __name__ == "__main__":
    main()
