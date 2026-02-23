#!/usr/bin/env python3
"""
mecharm_moveit_example.py
=========================
MoveIt 2 Python API (moveit_commander / MoveGroupInterface) 를 사용한
mechArm 270 M5 제어 예제.

Usage:
  # 터미널 1: 드라이버 + MoveIt 실행
  ros2 launch mecharm_moveit_config real_robot.launch.py

  # 터미널 2: 예제 실행
  python3 mecharm_moveit_example.py
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, WorkspaceParameters,
    RobotState, Constraints, JointConstraint, BoundingVolume
)
from geometry_msgs.msg import PoseStamped, Vector3
from shape_msgs.msg import SolidPrimitive
from rclpy.action import ActionClient
import math


class MechArmMoveItExample(Node):
    """
    MoveIt 2 의 /move_group 액션을 직접 호출하는 간단한 예제.
    실제 프로젝트에서는 moveit_py 또는 MoveItPy 바인딩 사용 권장.
    """

    def __init__(self):
        super().__init__("mecharm_moveit_example")
        self._client = ActionClient(self, MoveGroup, "/move_group")
        self.get_logger().info("Waiting for move_group action server...")
        self._client.wait_for_server()
        self.get_logger().info("Connected to move_group!")

    # ── 조인트 각도로 이동 ─────────────────────────────────────────
    def move_to_joint_goal(self, joint_values: list, group_name: str = "arm"):
        """
        joint_values: 6개 각도 [rad] 리스트
        """
        joint_names = [
            "joint1_to_base",
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
        ]

        # 조인트 제약 조건 생성
        constraints = Constraints()
        for name, value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = value
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        # 플래닝 요청
        request = MotionPlanRequest()
        request.group_name          = group_name
        request.num_planning_attempts = 10
        request.allowed_planning_time = 5.0
        request.max_velocity_scaling_factor     = 0.1
        request.max_acceleration_scaling_factor = 0.1
        request.goal_constraints.append(constraints)

        goal_msg = MoveGroup.Goal()
        goal_msg.request   = request
        goal_msg.planning_options.plan_only           = False
        goal_msg.planning_options.replan              = True
        goal_msg.planning_options.replan_attempts     = 3
        goal_msg.planning_options.replan_delay        = 2.0

        self.get_logger().info(f"Sending joint goal to group '{group_name}'...")
        future = self._client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result().result
        self.get_logger().info(f"Result error code: {result.error_code.val}")
        return result.error_code.val == 1  # MoveItErrorCodes.SUCCESS

    # ── 포즈(위치+자세)로 이동 ────────────────────────────────────
    def move_to_pose_goal(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        """
        TCP 를 지정한 Cartesian pose 로 이동.
        """
        pose = PoseStamped()
        pose.header.frame_id = "base"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        from moveit_msgs.msg import PositionIKRequest
        # NOTE: Cartesian pose goal 은 moveit_py 또는 move_group_interface 를
        #       사용하는 것이 더 편리합니다. 여기서는 개념만 보여줍니다.
        self.get_logger().warn(
            "Cartesian pose goals require moveit_py bindings. "
            "Use move_to_joint_goal() for direct joint control."
        )

    # ── 그리퍼 제어 ──────────────────────────────────────────────
    def set_gripper(self, open_ratio: float):
        """
        open_ratio: 0.0 (완전 닫힘) ~ 1.0 (완전 열림)
        """
        from control_msgs.action import GripperCommand
        from rclpy.action import ActionClient as AC

        GRIPPER_RAD_OPEN  =  0.15
        GRIPPER_RAD_CLOSE = -0.74
        target = GRIPPER_RAD_CLOSE + open_ratio * (GRIPPER_RAD_OPEN - GRIPPER_RAD_CLOSE)

        client = AC(self, GripperCommand, "/gripper_controller/gripper_action")
        client.wait_for_server(timeout_sec=3.0)

        goal = GripperCommand.Goal()
        goal.command.position    = target
        goal.command.max_effort  = 50.0

        self.get_logger().info(f"Setting gripper to {open_ratio:.0%} open ({target:.3f} rad)")
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)
        result_future = future.result().get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        return result_future.result().result.reached_goal


# ── 메인 시퀀스 ──────────────────────────────────────────────────────────────
def main():
    rclpy.init()
    node = MechArmMoveItExample()

    # 1. Home 포지션으로 이동
    print("\n[1] Moving to home position...")
    node.move_to_joint_goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    import time; time.sleep(1.0)

    # 2. 그리퍼 열기
    print("\n[2] Opening gripper...")
    node.set_gripper(1.0)
    time.sleep(0.5)

    # 3. 작업 포지션으로 이동 (예시: 팔을 앞으로 뻗기)
    print("\n[3] Moving to task position...")
    node.move_to_joint_goal([
        0.0,               # joint1: 정면
        -math.pi / 6,      # joint2: 약간 위로
        -math.pi / 3,      # joint3: 앞으로
        0.0,               # joint4
        math.pi / 6,       # joint5: 손목 아래로
        0.0,               # joint6
    ])
    time.sleep(1.0)

    # 4. 그리퍼 닫기 (파지)
    print("\n[4] Closing gripper...")
    node.set_gripper(0.0)
    time.sleep(1.0)

    # 5. Home으로 복귀
    print("\n[5] Returning to home...")
    node.move_to_joint_goal([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # 6. 그리퍼 열기
    print("\n[6] Opening gripper...")
    node.set_gripper(1.0)

    print("\nDemo complete!")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
