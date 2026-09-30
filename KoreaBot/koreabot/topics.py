"""Every topic KoreaBot uses, in one place - like setup.py lists the programs.

The three programs talk only through these topics:

    LINEFOLLOW  ->  line_follow         commands, plain text (std_msgs/String)
                                          "Line follower:True"  / "True"   drive
                                          "Line follower:False" / "False"  stop
                                          "White:True"  / "White:False"    follow the white line or not
                                          "Yellow:True" / "Yellow:False"   follow the yellow line or not
                                        several at once is fine: "White:True, Yellow:True"

    COLORS      ->  line_follow         colour settings from color_params, JSON text
    PARAMS      ->  line_follow         robot settings from robot_params, JSON text
    STATUS      <-  line_follow         what the robot is doing, JSON text, 4 times a second

Send a command by hand:

    ros2 topic pub --once /Linefollow std_msgs/msg/String "data: 'Line follower:True'"
"""

import os

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


LINEFOLLOW = "/Linefollow"
COLORS = "/Linefollow/colors"
PARAMS = "/Linefollow/params"
STATUS = "/Linefollow/status"

CMD_VEL = "/cmd_vel"
DEBUG_IMAGE = "/Linefollow/debug_image"

# กล้อง Pi (camera_ros) ส่งชื่อนี้ ถ้าไม่มีจะหาเองจาก ROS graph (ดู line_follow.py)
CAMERA = "/camera/image_raw/compressed"

# ค่าตั้งส่งแบบ "latched": คนที่มาทีหลัง (line_follow เปิดทีหลังหน้าต่างปรับค่า) ได้ค่าล่าสุดทันที
LATCHED = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)

# ชื่อคำสั่งที่รับ (ตัวเล็ก ไม่มีช่องว่าง) -> ชื่อสวิตช์ใน line_follow
COMMAND_NAMES = {
    "linefollower": "line_follower",
    "linefollow": "line_follower",
    "white": "white",
    "yellow": "yellow",
}
TRUE_WORDS = ("true", "1", "on", "yes")
FALSE_WORDS = ("false", "0", "off", "no")


def parse_command(text):
    """"Line follower:True, Yellow:False" -> ({"line_follower": True, "yellow": False}, errors).

    Case and spaces do not matter.  A bare "True" / "False" means Line follower.
    """

    switches, errors = {}, []

    for part in text.replace(";", ",").replace("\n", ",").split(","):
        part = part.strip()
        if not part:
            continue

        if ":" in part:
            name, value = part.split(":", 1)
        else:
            name, value = "line follower", part

        key = COMMAND_NAMES.get(name.strip().lower().replace(" ", "").replace("_", ""))
        word = value.strip().lower()

        if key is None:
            errors.append(f"unknown switch '{name.strip()}' (use Line follower, White, Yellow)")
        elif word in TRUE_WORDS:
            switches[key] = True
        elif word in FALSE_WORDS:
            switches[key] = False
        else:
            errors.append(f"'{part}': value must be True or False")

    return switches, errors


def command_text(line_follower=None, white=None, yellow=None):
    """The command string for these switches, e.g. "Line follower:True, White:False"."""

    parts = []
    for label, value in (("Line follower", line_follower), ("White", white), ("Yellow", yellow)):
        if value is not None:
            parts.append(f"{label}:{value}")
    return ", ".join(parts)


# IP หุ่น ใช้เป็น ROS_STATIC_PEERS ตอนเชลล์ไม่ได้ตั้งมาให้
ROBOT_IP = "192.168.1.54"
ROBOT_DOMAIN = "54"
PEERS_PROFILE = os.path.expanduser("~/.config/ros/fastdds_peers.xml")


def setup_network():
    """Point DDS discovery at the robot.  Call before rclpy.init().

    เครื่องนี้เป็น VirtualBox บริดจ์ multicast ข้ามไม่ได้ ไม่มี ROS_STATIC_PEERS = ไม่เห็นหุ่นเลย
    ไฟล์ peers ไล่พอร์ตของ participant id 0-15 เพราะกล้องสตาร์ตทีหลังได้ id สูง

    Only on the robot's domain: the peers file lists domain 54's ports
    directly, so loading it on any other ROS_DOMAIN_ID would still reach the
    real robot - a test on another domain must stay away from it.
    """
    if os.environ.get("ROS_DOMAIN_ID", "0") != ROBOT_DOMAIN:
        return
    if not os.environ.get("ROS_STATIC_PEERS"):
        os.environ["ROS_STATIC_PEERS"] = ROBOT_IP
    if not os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE") and os.path.exists(PEERS_PROFILE):
        os.environ["FASTRTPS_DEFAULT_PROFILES_FILE"] = PEERS_PROFILE
