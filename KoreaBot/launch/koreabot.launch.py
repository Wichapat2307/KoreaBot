"""Start all three KoreaBot programs at once:

    ros2 launch koreabot koreabot.launch.py

line_follow stays still until it gets "Line follower:True" on /Linefollow
(the START button in the robot window sends it).
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="koreabot", executable="line_follow", output="screen"),
        Node(package="koreabot", executable="color_params", output="screen"),
        Node(package="koreabot", executable="robot_params", output="screen"),
    ])
