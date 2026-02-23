"""
moveit_mech.launch.py
=====================
mecharm.urdf 를 사용하는 MoveIt2 + RViz2 런치 파일.
moveit2_tutorials 의 demo.launch.py 구조를 참고하여 작성.
mock_components 를 사용해 실제 하드웨어 없이 ros2_control + 컨트롤러를 구동합니다.

Usage:
  ros2 launch mecharm_moveit_config moveit_mech.launch.py
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=os.path.join(
                get_package_share_directory("mecharm_moveit_config"),
                "config",
                "moveit.rviz",
            ),
            description="RViz configuration file (full path)",
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )


def launch_setup(context, *args, **kwargs):

    urdf_path = os.path.join(
        get_package_share_directory("mecharm_description"),
        "urdf",
        "mecharm.urdf",
    )

    # mecharm.urdf 를 읽어서 ros2_control 태그(mock_components)를 삽입
    with open(urdf_path, "r") as f:
        urdf_content = f.read()

    ros2_control_tag = """
  <ros2_control name="MechArmFakeSystem" type="system">
    <hardware>
      <plugin>mock_components/GenericSystem</plugin>
    </hardware>
    <joint name="joint1_to_base">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint2_to_joint1">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint3_to_joint2">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint4_to_joint3">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint5_to_joint4">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="joint6_to_joint5">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
    <joint name="gripper_controller">
      <command_interface name="position"/>
      <state_interface name="position"><param name="initial_value">0.0</param></state_interface>
      <state_interface name="velocity"/>
    </joint>
  </ros2_control>
"""

    robot_description_content = urdf_content.replace(
        "</robot>", ros2_control_tag + "</robot>"
    )

    moveit_config = (
        MoveItConfigsBuilder("mecharm", package_name="mecharm_moveit_config")
        .robot_description(file_path=urdf_path)
        .robot_description_semantic(file_path="config/mecharm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True, publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    # ros2_control 이 포함된 robot_description (mock hardware용)
    robot_description_with_control = {
        "robot_description": robot_description_content
    }

    # move_group 노드
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": False},
            {"allow_trajectory_execution": True},
            {"publish_planning_scene": True},
            {"publish_geometry_updates": True},
            {"publish_state_updates": True},
            {"publish_transforms_updates": True},
        ],
    )

    # RViz2
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Static TF: world → base
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base"],
    )

    # robot_state_publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description_with_control],
    )

    # ros2_control_node (mock_components 사용)
    ros2_controllers_path = os.path.join(
        get_package_share_directory("mecharm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description_with_control, ros2_controllers_path],
        output="both",
    )

    # joint_state_broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # arm_controller spawner
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "-c", "/controller_manager"],
    )

    # gripper_controller spawner
    gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gripper_controller", "-c", "/controller_manager"],
    )

    nodes_to_start = [
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        gripper_controller_spawner,
        rviz_node,
    ]

    return nodes_to_start
