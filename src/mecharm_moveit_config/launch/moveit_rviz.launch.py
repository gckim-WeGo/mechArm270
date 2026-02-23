"""
moveit_rviz.launch.py
=====================
로봇 드라이버 + MoveIt + RViz만 실행하는 런치파일.
pick_and_place 노드는 포함하지 않음.

실행:
  ros2 launch mecharm_moveit_config moveit_rviz.launch.py
  ros2 launch mecharm_moveit_config moveit_rviz.launch.py port:=/dev/ttyACM0 baud:=115200
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    port_arg = DeclareLaunchArgument("port", default_value="/dev/ttyACM0")
    baud_arg = DeclareLaunchArgument("baud", default_value="115200")

    robot_description = {
        "robot_description": load_file("mecharm_description", "urdf/mecharm.urdf")
    }
    robot_description_semantic = {
        "robot_description_semantic": load_file(
            "mecharm_moveit_config", "config/mecharm.srdf"
        )
    }
    kinematics_yaml   = load_yaml("mecharm_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("mecharm_moveit_config", "config/joint_limits.yaml")

    ompl_planning_pipeline_config = {
        "planning_pipelines": ["ompl"],
        "default_planning_pipeline": "ompl",
        "ompl": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": " ".join([
                "default_planner_request_adapters/AddTimeOptimalParameterization",
                "default_planner_request_adapters/ResolveConstraintFrames",
                "default_planner_request_adapters/FixWorkspaceBounds",
                "default_planner_request_adapters/FixStartStateBounds",
                "default_planner_request_adapters/FixStartStateCollision",
                "default_planner_request_adapters/FixStartStatePathConstraints",
            ]),
            "start_state_max_bounds_error": 0.1,
        },
    }
    ompl_planning_yaml_raw = load_yaml("mecharm_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["ompl"].update(ompl_planning_yaml_raw)

    moveit_controllers = load_yaml("mecharm_moveit_config", "config/moveit_controllers.yaml")

    # ── robot_state_publisher ─────────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # ── Hardware driver ───────────────────────────────────────────
    hardware_node = Node(
        package="mecharm_hardware",
        executable="mecharm_driver",
        name="mecharm_driver",
        output="screen",
        parameters=[
            {"port": LaunchConfiguration("port")},
            {"baud": LaunchConfiguration("baud")},
            {"publish_rate": 10.0},
            {"move_speed": 50},
        ],
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
            ompl_planning_pipeline_config,
            joint_limits_yaml,
            moveit_controllers,
            {"moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager"},
            {"trajectory_execution.allowed_execution_duration_scaling": 10.0},
            {"trajectory_execution.allowed_goal_duration_margin": 10.0},
            {"trajectory_execution.execution_duration_monitoring": True},
            {"use_sim_time": True},
            {"publish_robot_description_semantic": True},
            {"allow_trajectory_execution": True},
            {"publish_planning_scene": True},
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
            {"monitor_dynamics": True},
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
            ompl_planning_pipeline_config,
            {"use_sim_time": False},
        ],
    )

    # ── AprilTag TF ───────────────────────────────────────────────
    apriltag_node = Node(
        package="mecharm_hardware",
        executable="apriltag_tf",
        name="apriltag_tf",
        output="screen",
        parameters=[
            {"camera_index": 0},
            {"tag_size": 0.025},
            {"tag_family": "tag36h11"},
        ],
    )

    return LaunchDescription([
        port_arg,
        baud_arg,
        rsp_node,
        hardware_node,
        move_group_node,
        rviz_node,
        # apriltag_node,
    ])
