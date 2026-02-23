"""
demo.launch.py
==============
robot_state_publisher + move_group + rviz2 를 한 번에 실행합니다.
실제 로봇 없이 RViz 에서 MoveIt 플래닝을 테스트할 때 사용합니다.

Usage:
  ros2 launch mecharm_moveit_config demo.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def load_file(package_name, file_path):
    pkg = get_package_share_directory(package_name)
    with open(os.path.join(pkg, file_path)) as f:
        return f.read()


def load_yaml(package_name, file_path):
    pkg = get_package_share_directory(package_name)
    with open(os.path.join(pkg, file_path)) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    # ── descriptions ──────────────────────────────────────────────
    robot_description = {
        "robot_description": load_file("mecharm_description", "urdf/mecharm.urdf")
    }
    robot_description_semantic = {
        "robot_description_semantic": load_file(
            "mecharm_moveit_config", "config/mecharm.srdf"
        )
    }

    kinematics_yaml       = load_yaml("mecharm_moveit_config", "config/kinematics.yaml")
    ompl_planning_yaml    = load_yaml("mecharm_moveit_config", "config/ompl_planning.yaml")
    joint_limits_yaml     = load_yaml("mecharm_moveit_config", "config/joint_limits.yaml")
    moveit_controllers    = load_yaml("mecharm_moveit_config", "config/moveit_controllers.yaml")

    # ── robot_state_publisher ─────────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ── joint_state_publisher (demo 용 가짜 상태) ─────────────────
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[robot_description],
    )

    # ── move_group ────────────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_planning_yaml,
            joint_limits_yaml,
            moveit_controllers,
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True},
            {"allow_trajectory_execution": True},
            {"publish_planning_scene": True},
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
        ],
    )

    # ── RViz2 ─────────────────────────────────────────────────────
    rviz_config = os.path.join(
        get_package_share_directory("mecharm_moveit_config"), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
        ],
    )

    return LaunchDescription([
        rsp_node,
        jsp_node,
        move_group_node,
        rviz_node,
    ])
