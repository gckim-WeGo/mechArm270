#!/usr/bin/env python3
"""
mecharm_driver.py
=================
pymycobot 라이브러리를 사용해 실제 mechArm 270 M5 와 통신하는 ROS 2 노드.

기능:
  - /joint_states 퍼블리시  (현재 각도 -> radian)
  - /arm_controller/follow_joint_trajectory 액션 서버
  - /gripper_controller/gripper_action 액션 서버

Topic / Action:
  Published : /joint_states  [sensor_msgs/JointState]
  Action    : /arm_controller/follow_joint_trajectory
              [control_msgs/action/FollowJointTrajectory]
  Action    : /gripper_controller/gripper_action
              [control_msgs/action/GripperCommand]

Parameters:
  port  (str)  : 시리얼 포트 (default: /dev/ttyUSB0)
  baud  (int)  : 보드레이트  (default: 115200)
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectoryPoint

try:
    from pymycobot import MechArm270
except ImportError:
    raise RuntimeError(
        "pymycobot 가 설치되어 있지 않습니다.\n"
        "설치 방법: pip install pymycobot"
    )


# ── 상수 ────────────────────────────────────────────────────────────────────
ARM_JOINT_NAMES = [
    "joint1_to_base",
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
]
GRIPPER_JOINT_NAME = "gripper_controller"

DEG2RAD = math.pi / 180.0
RAD2DEG = 180.0 / math.pi

# 그리퍼: MoveIt radian <-> pymycobot 0~100 값 변환
# gripper_controller: lower=-0.74 (닫힘), upper=0.15 (열림)
GRIPPER_RAD_OPEN  =  0.15
GRIPPER_RAD_CLOSE = -0.74
GRIPPER_VAL_OPEN  = 100
GRIPPER_VAL_CLOSE = 0

def gripper_rad_to_val(rad: float) -> int:
    """MoveIt radian -> pymycobot 0~100"""
    ratio = (rad - GRIPPER_RAD_CLOSE) / (GRIPPER_RAD_OPEN - GRIPPER_RAD_CLOSE)
    return int(max(0, min(100, ratio * 100)))

def gripper_val_to_rad(val: int) -> float:
    """pymycobot 0~100 -> MoveIt radian"""
    ratio = val / 100.0
    return GRIPPER_RAD_CLOSE + ratio * (GRIPPER_RAD_OPEN - GRIPPER_RAD_CLOSE)


# ── 드라이버 노드 ─────────────────────────────────────────────────────────────
class MechArmDriver(Node):

    def __init__(self):
        super().__init__("mecharm_driver")

        # ── 파라미터 ──────────────────────────────────────────────
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baud", 115200)
        self.declare_parameter("publish_rate", 10.0)   # Hz
        self.declare_parameter("move_speed", 50)       # 1~100, pymycobot speed

        port = self.get_parameter("port").get_parameter_value().string_value
        baud = self.get_parameter("baud").get_parameter_value().integer_value
        self.publish_rate = self.get_parameter("publish_rate").get_parameter_value().double_value
        self.move_speed   = self.get_parameter("move_speed").get_parameter_value().integer_value

        # ── pymycobot 연결 ────────────────────────────────────────
        self.get_logger().info(f"Connecting to mechArm on {port} @ {baud}...")
        self.arm = MechArm270(port, baud)
        time.sleep(0.5)
        self.get_logger().info("Connected!")

        # ── 내부 상태 ─────────────────────────────────────────────
        self._lock = threading.Lock()
        self._current_joint_rad = [0.0] * 6
        self._current_gripper_rad = GRIPPER_RAD_OPEN

        # ── 퍼블리셔 ──────────────────────────────────────────────
        self._js_pub = self.create_publisher(JointState, "joint_states", 10)
        self._timer  = self.create_timer(1.0 / self.publish_rate, self._publish_joint_states)

        # ── 액션 서버 ─────────────────────────────────────────────
        cb_group = ReentrantCallbackGroup()

        self._arm_action_server = ActionServer(
            self,
            FollowJointTrajectory,
            "/arm_controller/follow_joint_trajectory",
            execute_callback=self._arm_execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        self._gripper_action_server = ActionServer(
            self,
            GripperCommand,
            "/gripper_controller/gripper_action",
            execute_callback=self._gripper_execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

        self.get_logger().info("mechArm driver ready.")

    # ── 콜백: 상태 퍼블리시 ───────────────────────────────────────
    def _publish_joint_states(self):
        # 1) 6축 각도 읽기 (degree → radian)
        try:
            angles_deg = self.arm.get_angles()
            if angles_deg and len(angles_deg) == 6:
                with self._lock:
                    self._current_joint_rad = [a * DEG2RAD for a in angles_deg]
        except Exception as e:
            self.get_logger().warn(f"Failed to read arm angles: {e}", throttle_duration_sec=5.0)

        # 2) 그리퍼 값 읽기 (0~100 → radian)
        try:
            gval = self.arm.get_gripper_value()
            if gval is not None:
                with self._lock:
                    self._current_gripper_rad = gripper_val_to_rad(int(gval))
        except Exception as e:
            self.get_logger().warn(f"Failed to read gripper value: {e}", throttle_duration_sec=5.0)

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        with self._lock:
            js.name     = ARM_JOINT_NAMES + [GRIPPER_JOINT_NAME]
            js.position = self._current_joint_rad + [self._current_gripper_rad]
        self._js_pub.publish(js)

    # ── 액션 공통 콜백 ────────────────────────────────────────────
    def _goal_callback(self, goal_request):
        self.get_logger().info("Goal received")
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info("Cancel requested")
        return CancelResponse.ACCEPT

    # ── 팔 궤적 실행 ─────────────────────────────────────────────
    def _arm_execute_callback(self, goal_handle):
        self.get_logger().info("Executing arm trajectory...")
        trajectory = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()

        # 조인트 순서 매핑
        traj_joint_names = trajectory.joint_names
        indices = []
        for arm_jn in ARM_JOINT_NAMES:
            if arm_jn in traj_joint_names:
                indices.append(traj_joint_names.index(arm_jn))
            else:
                indices.append(None)

        points = trajectory.points
        if not points:
            goal_handle.succeed()
            return result

        try:
            # pymycobot의 send_angles()는 비동기 명령이므로
            # 중간 waypoint를 연속 전송하면 이전 동작을 중단시켜 삐걱거림 발생.
            # 마지막 포인트(최종 목표)만 전송 후 도달 대기.
            last_point = points[-1]
            target_deg = []
            for idx in indices:
                if idx is not None:
                    target_deg.append(last_point.positions[idx] * RAD2DEG)
                else:
                    target_deg.append(0.0)

            total_time = (last_point.time_from_start.sec +
                          last_point.time_from_start.nanosec * 1e-9)

            self.arm.send_angles(target_deg, self.move_speed)

            # 실제 도달 여부를 polling으로 확인 (tolerance: 2도)
            TOLERANCE_DEG = 2.0
            POLL_INTERVAL = 0.1   # 100ms
            # 여유 시간: OMPL 계획 시간 + 실제 로봇 여유 (로봇이 느릴 수 있음)
            deadline = max(total_time * 2.0, total_time + 5.0)
            elapsed = 0.0
            reached = False

            while elapsed < deadline:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.get_logger().info("Trajectory canceled")
                    return result

                time.sleep(POLL_INTERVAL)
                elapsed += POLL_INTERVAL

                try:
                    current_deg = self.arm.get_angles()
                except Exception:
                    continue

                if current_deg and len(current_deg) == 6:
                    errors = [abs(current_deg[i] - target_deg[i]) for i in range(6)]
                    if all(e < TOLERANCE_DEG for e in errors):
                        reached = True
                        break

            if not reached:
                self.get_logger().warn(
                    f"Arm did not reach goal within {deadline:.1f}s, "
                    f"but marking succeeded anyway."
                )

        except Exception as e:
            self.get_logger().error(f"Arm trajectory execution failed: {e}")
            goal_handle.abort()
            return result

        goal_handle.succeed()
        self.get_logger().info("Arm trajectory completed")
        return result

    # ── 그리퍼 실행 ──────────────────────────────────────────────
    def _gripper_execute_callback(self, goal_handle):
        self.get_logger().info("Executing gripper command...")
        command = goal_handle.request.command
        result = GripperCommand.Result()

        target_rad = command.position
        target_val = gripper_rad_to_val(target_rad)

        try:
            self.arm.set_gripper_value(target_val, self.move_speed, gripper_type=1)
            time.sleep(1.0)  # 그리퍼 동작 대기

            # 현재 값 확인
            gval = self.arm.get_gripper_value()
            if gval is not None:
                result.position  = gripper_val_to_rad(int(gval))
                result.reached_goal = abs(result.position - target_rad) < 0.05
            else:
                result.position  = target_rad
                result.reached_goal = True

        except Exception as e:
            self.get_logger().error(f"Gripper command failed: {e}")
            goal_handle.abort()
            return result

        goal_handle.succeed()
        self.get_logger().info(f"Gripper done (val={target_val})")
        return result


# ── main ─────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = MechArmDriver()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
