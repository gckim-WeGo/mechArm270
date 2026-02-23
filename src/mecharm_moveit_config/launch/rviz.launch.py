import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import yaml


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path) as f:
        return f.read()


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path) as f:
        return yaml.safe_load(f)


def generate_launch_description():
    robot_description_content = load_file(
        "mecharm_description", "urdf/mecharm.urdf"
    )
    robot_description = {"robot_description": robot_description_content}

    robot_description_semantic_content = load_file(
        "mecharm_moveit_config", "config/mecharm.srdf"
    )
    robot_description_semantic = {
        "robot_description_semantic": robot_description_semantic_content
    }

    kinematics_yaml = load_yaml("mecharm_moveit_config", "config/kinematics.yaml")

    rviz_config = os.path.join(
        get_package_share_directory("mecharm_moveit_config"),
        "config",
        "moveit.rviz",
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

    return LaunchDescription([rviz_node])
