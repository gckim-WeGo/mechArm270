import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import yaml


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path) as f:
        return yaml.safe_load(f)


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    with open(absolute_file_path) as f:
        return f.read()


def generate_launch_description():
    # ── URDF / SRDF ──────────────────────────────────────────────
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

    # ── MoveIt configs ────────────────────────────────────────────
    kinematics_yaml = load_yaml("mecharm_moveit_config", "config/kinematics.yaml")
    ompl_planning_yaml = load_yaml("mecharm_moveit_config", "config/ompl_planning.yaml")
    joint_limits_yaml = load_yaml("mecharm_moveit_config", "config/joint_limits.yaml")
    moveit_controllers_yaml = load_yaml(
        "mecharm_moveit_config", "config/moveit_controllers.yaml"
    )

    move_group_capabilities = {
        "capabilities": "move_group/MoveGroupCartesianPathService"
        " move_group/MoveGroupExecuteTrajectoryAction"
        " move_group/MoveGroupKinematicsService"
        " move_group/MoveGroupMoveAction"
        " move_group/MoveGroupPickPlaceAction"
        " move_group/MoveGroupPlanService"
        " move_group/MoveGroupQueryPlannersService"
        " move_group/MoveGroupStateValidationService"
        " move_group/MoveGroupGetPlanningSceneService"
        " move_group/ApplyPlanningSceneService"
        " move_group/ClearOctomapService"
    }

    # ── move_group node ───────────────────────────────────────────
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
            moveit_controllers_yaml,
            move_group_capabilities,
            {"use_sim_time": False},
            {"publish_robot_description_semantic": True},
        ],
    )

    return LaunchDescription([move_group_node])
