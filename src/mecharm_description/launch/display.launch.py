"""
display.launch.py
=================
URDF 만 RViz 에서 확인할 때 사용 (MoveIt 없이).

Usage:
  ros2 launch mecharm_description display.launch.py
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    urdf_path = os.path.join(
        get_package_share_directory("mecharm_description"),
        "urdf",
        "mecharm.urdf",
    )
    with open(urdf_path) as f:
        robot_description = f.read()

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", os.path.join(
                get_package_share_directory("mecharm_moveit_config"),
                "config", "moveit.rviz"
            )],
        ),
    ])
