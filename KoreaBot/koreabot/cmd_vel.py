#!/usr/bin/env python3

"""Send drive commands to the robot on /cmd_vel.

line_follow.py builds a plain Twist and hands it to publish(); this wraps it the
way the robot wants it.  TurtleBot3 on Jazzy listens for TwistStamped (same as
navi.py uses) - set USE_STAMPED = False for a robot that still takes Twist.

    cmd_pub = VelocityPublisher(node)
    cmd_pub.publish(twist)
    cmd_pub.stop()
"""

from geometry_msgs.msg import Twist, TwistStamped


CMD_TOPIC = "/cmd_vel"

# Jazzy = TwistStamped ถ้าหุ่นเป็น Humble หรือเก่ากว่า ใช้ Twist ธรรมดา
# ส่งผิดชนิด = หุ่นเงียบ ไม่ขยับ ไม่มี error อะไรบอกเลย
USE_STAMPED = True

# ขีดจำกัดของ burger กันพิมพ์ค่าผิดแล้วหุ่นพุ่ง
MAX_LINEAR = 0.22     # m/s
MAX_ANGULAR = 2.84    # rad/s

# สั่งหยุดซ้ำหลายครั้ง WiFi หายไปครั้งเดียวหุ่นจะวิ่งค้างคำสั่งเดิม
STOP_REPEAT = 3


def _clamp(value, limit):
    return max(-limit, min(limit, value))


class VelocityPublisher:

    def __init__(self, node, topic=CMD_TOPIC):
        self.node = node
        msg_type = TwistStamped if USE_STAMPED else Twist
        self.pub = node.create_publisher(msg_type, topic, 10)

    ########################################################

    def publish(self, twist):
        """Send one Twist, clamped to what the robot can do."""

        cmd = Twist()
        cmd.linear.x = _clamp(float(twist.linear.x), MAX_LINEAR)
        cmd.angular.z = _clamp(float(twist.angular.z), MAX_ANGULAR)

        if not USE_STAMPED:
            self.pub.publish(cmd)
            return

        msg = TwistStamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist = cmd

        self.pub.publish(msg)

    ########################################################

    def stop(self):
        """Zero speed, sent a few times so a dropped packet cannot miss it."""

        for _ in range(STOP_REPEAT):
            self.publish(Twist())
