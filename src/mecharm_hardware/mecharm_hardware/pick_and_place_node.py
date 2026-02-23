#!/usr/bin/env python3
"""
pick_and_place_node.py
======================
AprilTag Vision 창의 버튼으로 3가지 상태를 전환하는 pick & place 노드.

버튼:
  [SCAN]  : 태그 위치 확인 & RViz 박스 표시 (토글)
  [PICK]  : 그리퍼 열기 → 태그 이동 → 그리퍼 닫기 → 홈 복귀
  [PLACE] : 고정 위치 이동 → 그리퍼 열기 → 홈 복귀

실행:
  ros2 launch mecharm_moveit_config real_robot.launch.py port:=/dev/ttyACM0 baud:=115200
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import math
import time
import threading
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import PoseStamped, TransformStamped
from std_msgs.msg import String
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    MotionPlanRequest, Constraints, JointConstraint,
    CollisionObject, PlanningScene,
    PositionConstraint, OrientationConstraint, BoundingVolume,
)
from control_msgs.action import GripperCommand
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from tf2_ros import TransformBroadcaster, Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401

try:
    from pupil_apriltags import Detector
except ImportError:
    raise RuntimeError("pupil_apriltags 가 설치되어 있지 않습니다.\npip install pupil-apriltags")


# ── 회전행렬 → 쿼터니언 ───────────────────────────────────────────────────────
def rotation_matrix_to_quaternion(R):
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


# ── 상수 ──────────────────────────────────────────────────────────────────────
ARM_JOINT_NAMES = [
    "joint1_to_base", "joint2_to_joint1", "joint3_to_joint2",
    "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5",
]
HOME_JOINTS       = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
GRIPPER_RAD_OPEN  =  0.15
GRIPPER_RAD_CLOSE = -0.74
TAG_BOX_ID        = "apriltag_box"
BOX_SIZE_M        = 0.05

# ── 버튼 레이아웃 ──────────────────────────────────────────────────────────────
BTN_H      = 60    # 버튼 영역 높이 (px)
BTN_PAD    = 8     # 버튼 사이 간격 (px)
BTN_FONT   = cv2.FONT_HERSHEY_SIMPLEX
BTN_FSCALE = 0.7
BTN_THICK  = 2

# 버튼 정의: (label, cmd_key)
BUTTONS = [
    ("SCAN",  "scan"),
    ("PICK",  "pick"),
    ("PLACE", "place"),
]

# 상태 색상
COLOR_IDLE    = (60,  60,  60)   # 어두운 회색
COLOR_ACTIVE  = (0,  160,   0)   # 초록
COLOR_BUSY    = (0,   80, 200)   # 파랑
COLOR_TEXT    = (255, 255, 255)
COLOR_STATUS  = (30,  30,  30)   # 상태바 배경


class PickAndPlaceNode(Node):

    def __init__(self):
        super().__init__("pick_and_place")

        # ── 파라미터 ──────────────────────────────────────────────
        self.declare_parameter("place_x",            0.20)
        self.declare_parameter("place_y",            0.15)
        self.declare_parameter("place_z",            0.15)
        self.declare_parameter("pre_grasp_z_offset", 0.10)
        self.declare_parameter("lift_z_offset",      0.10)
        self.declare_parameter("tag_size",           0.025)

        self.place_pos = [
            self.get_parameter("place_x").value,
            self.get_parameter("place_y").value,
            self.get_parameter("place_z").value,
        ]
        self.pre_grasp_z_offset = self.get_parameter("pre_grasp_z_offset").value
        self.lift_z_offset      = self.get_parameter("lift_z_offset").value
        tag_size_param          = self.get_parameter("tag_size").value

        # ── TF ───────────────────────────────────────────────────
        self._tf_broadcaster = TransformBroadcaster(self)
        self._tf_buffer      = Buffer()
        self._tf_listener    = TransformListener(self._tf_buffer, self)

        # ── 공유 상태 ─────────────────────────────────────────────
        self._lock          = threading.Lock()
        self._tag_detected  = False
        self._latest_tag_id = None
        self._latest_pose_t = None
        self._latest_pose_R = None

        self._running       = True
        self._scanning      = False   # SCAN 모드 활성 여부
        self._busy          = False   # PICK/PLACE 실행 중 여부
        self._status_msg    = "대기 중"

        self._cmd_queue     = []
        self._cmd_lock      = threading.Lock()

        # 버튼 rect 저장 (vision_loop에서 계산)
        self._btn_rects     = []   # [(x1,y1,x2,y2), ...]

        # ── Action 클라이언트 ──────────────────────────────────────
        cb = ReentrantCallbackGroup()
        self._move_client = ActionClient(
            self, MoveGroup, "/move_group", callback_group=cb)
        self._gripper_client = ActionClient(
            self, GripperCommand, "/gripper_controller/gripper_action",
            callback_group=cb)

        # launch 시 move_group보다 늦게 뜰 수 있으므로 미리 대기
        self.get_logger().info("/move_group 서버 대기 중...")
        self._move_client.wait_for_server(timeout_sec=60.0)
        self.get_logger().info("/move_group 서버 연결됨")

        # ── Planning Scene 퍼블리셔 ───────────────────────────────
        self._scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)

        # ── /pick_and_place/cmd 토픽 수신 ─────────────────────────
        self.create_subscription(
            String, "/pick_and_place/cmd", self._cmd_topic_cb, 10)

        # ── 카메라 / AprilTag ─────────────────────────────────────
        self._cap = cv2.VideoCapture(0)
        if not self._cap.isOpened():
            self.get_logger().error("카메라를 열 수 없습니다!")
        ret, frame = self._cap.read()
        if ret:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            h, w = frame.shape[:2]
        else:
            w, h = 640, 480
        fx = fy = float(w)
        cx_cam, cy_cam = w / 2.0, h / 2.0
        self._camera_matrix = np.array(
            [[fx, 0, cx_cam], [0, fy, cy_cam], [0, 0, 1]], dtype=np.float64)
        self._dist_coeffs   = np.zeros((5, 1), dtype=np.float64)
        self._camera_params = (fx, fy, cx_cam, cy_cam)
        self._tag_size      = tag_size_param

        self._detector = Detector(
            families="tag36h11",
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
        )

        self.get_logger().info("PickAndPlaceNode 초기화 완료 – AprilTag Vision 창의 버튼을 클릭하세요.")

    # =========================================================================
    # 버튼 UI 그리기
    # =========================================================================
    def _draw_ui(self, frame: np.ndarray) -> np.ndarray:
        """카메라 프레임 아래에 버튼 영역을 붙여 반환한다."""
        h, w = frame.shape[:2]
        n = len(BUTTONS)
        btn_w = (w - BTN_PAD * (n + 1)) // n

        # 버튼 패널 (단색 배경)
        panel = np.zeros((BTN_H, w, 3), dtype=np.uint8)
        panel[:] = COLOR_STATUS

        rects = []
        for i, (label, cmd) in enumerate(BUTTONS):
            x1 = BTN_PAD + i * (btn_w + BTN_PAD)
            y1 = BTN_PAD
            x2 = x1 + btn_w
            y2 = BTN_H - BTN_PAD

            # 버튼 색상 결정
            if self._busy:
                color = COLOR_BUSY if (
                    (cmd == "pick"  and self._status_msg.startswith("[PICK]")) or
                    (cmd == "place" and self._status_msg.startswith("[PLACE]"))
                ) else COLOR_IDLE
            elif cmd == "scan" and self._scanning:
                color = COLOR_ACTIVE
            else:
                color = COLOR_IDLE

            cv2.rectangle(panel, (x1, y1), (x2, y2), color, -1)
            cv2.rectangle(panel, (x1, y1), (x2, y2), (120, 120, 120), 1)

            # 텍스트 중앙 정렬
            (tw, th), _ = cv2.getTextSize(label, BTN_FONT, BTN_FSCALE, BTN_THICK)
            tx = x1 + (btn_w - tw) // 2
            ty = y1 + (y2 - y1 + th) // 2
            cv2.putText(panel, label, (tx, ty), BTN_FONT, BTN_FSCALE,
                        COLOR_TEXT, BTN_THICK, cv2.LINE_AA)

            # panel 좌표 → 합성 이미지 좌표 변환: y offset = h
            rects.append((x1, h + y1, x2, h + y2))

        self._btn_rects = rects

        # 상태 메시지 (패널 우측)
        with self._lock:
            detected = self._tag_detected
            tag_id   = self._latest_tag_id
        if detected:
            info = f"TAG ID={tag_id} 감지"
            info_color = (0, 230, 0)
        else:
            info = "태그 없음"
            info_color = (100, 100, 100)

        cv2.putText(panel, info,
                    (w - 200, BTN_H // 2 + 6),
                    BTN_FONT, 0.55, info_color, 1, cv2.LINE_AA)

        # 상태 메시지 (프레임 상단 오버레이)
        cv2.rectangle(frame, (0, 0), (w, 28), (0, 0, 0), -1)
        cv2.putText(frame, self._status_msg, (6, 20),
                    BTN_FONT, 0.55, (200, 200, 200), 1, cv2.LINE_AA)

        return np.vstack([frame, panel])

    # =========================================================================
    # 마우스 콜백
    # =========================================================================
    def _on_mouse(self, event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        for i, (x1, y1, x2, y2) in enumerate(self._btn_rects):
            if x1 <= x <= x2 and y1 <= y <= y2:
                cmd = BUTTONS[i][1]
                self._push_cmd(cmd)
                break

    # =========================================================================
    # 비전 루프 (별도 스레드)
    # =========================================================================
    def vision_loop(self):
        win = "AprilTag Vision"
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(win, self._on_mouse)

        while self._running:
            ret, frame = self._cap.read()
            if not ret:
                # 카메라 없을 때 빈 프레임
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "No Camera", (200, 240),
                            BTN_FONT, 1.2, (0, 0, 200), 2)
            else:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

            h, w = frame.shape[:2]
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            tags = self._detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=self._camera_params,
                tag_size=self._tag_size,
            )

            # 화면 중심에 가장 가까운 태그 선택
            selected = None
            min_dist = float("inf")
            fc = (w // 2, h // 2)
            for tag in tags:
                cx, cy = tag.center
                d = (cx - fc[0])**2 + (cy - fc[1])**2
                if d < min_dist:
                    min_dist = d
                    selected = tag

                for pt in tag.corners:
                    cv2.circle(frame, (int(pt[0]), int(pt[1])), 4, (0, 0, 255), -1)
                rvec, _ = cv2.Rodrigues(tag.pose_R)
                cv2.drawFrameAxes(
                    frame, self._camera_matrix, self._dist_coeffs,
                    rvec, tag.pose_t.reshape(3, 1), self._tag_size / 2)
                cv2.putText(frame, f"ID:{tag.tag_id}",
                            (int(tag.center[0]), int(tag.center[1]) - 15),
                            BTN_FONT, 0.5, (0, 255, 0), 2)

            with self._lock:
                if selected is not None:
                    self._tag_detected  = True
                    self._latest_tag_id = selected.tag_id
                    self._latest_pose_t = selected.pose_t.flatten()
                    self._latest_pose_R = selected.pose_R.copy()
                    self._publish_tag_tf(selected.tag_id,
                                         selected.pose_t.flatten(),
                                         selected.pose_R)
                    if self._scanning:
                        self._update_scene_from_camera(selected.tag_id)
                else:
                    self._tag_detected  = False
                    self._latest_pose_t = None

            # UI 합성 후 표시
            display = self._draw_ui(frame)
            cv2.imshow(win, display)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                self._running = False
                break
            elif key == ord('1'):
                self._push_cmd("scan")
            elif key == ord('2'):
                self._push_cmd("pick")
            elif key == ord('3'):
                self._push_cmd("place")

        cv2.destroyAllWindows()

    # =========================================================================
    # TF 발행
    # =========================================================================
    def _publish_tag_tf(self, tag_id, pose_t, pose_R):
        qx, qy, qz, qw = rotation_matrix_to_quaternion(pose_R)
        ts = TransformStamped()
        ts.header.stamp    = self.get_clock().now().to_msg()
        ts.header.frame_id = "camera_link"
        ts.child_frame_id  = f"apriltag_{tag_id}"
        x, y, z = pose_t.flatten()
        ts.transform.translation.x = float(x)
        ts.transform.translation.y = float(y)
        ts.transform.translation.z = float(z)
        ts.transform.rotation.x    = float(qx)
        ts.transform.rotation.y    = float(qy)
        ts.transform.rotation.z    = float(qz)
        ts.transform.rotation.w    = float(qw)
        self._tf_broadcaster.sendTransform(ts)

    # =========================================================================
    # Planning Scene
    # =========================================================================
    def _update_scene_from_camera(self, tag_id: int):
        tag_pose = self._get_tag_pose_in_base(tag_id, timeout=0.1)
        if tag_pose is not None:
            self._update_planning_scene_box(tag_pose)

    def _update_planning_scene_box(self, tag_pose_base: PoseStamped):
        co = CollisionObject()
        co.header.frame_id = "base"
        co.id              = TAG_BOX_ID
        co.operation       = CollisionObject.ADD
        box = SolidPrimitive()
        box.type           = SolidPrimitive.BOX
        box.dimensions     = [BOX_SIZE_M, BOX_SIZE_M, BOX_SIZE_M]
        co.primitives      = [box]
        co.primitive_poses = [tag_pose_base.pose]
        scene = PlanningScene()
        scene.world.collision_objects = [co]
        scene.is_diff = True
        self._scene_pub.publish(scene)

    def _remove_planning_scene_box(self):
        co = CollisionObject()
        co.header.frame_id = "base"
        co.id              = TAG_BOX_ID
        co.operation       = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.world.collision_objects = [co]
        scene.is_diff = True
        self._scene_pub.publish(scene)

    # =========================================================================
    # TF 조회
    # =========================================================================
    def _get_tag_pose_in_base(self, tag_id: int, timeout: float = 2.0):
        try:
            tf = self._tf_buffer.lookup_transform(
                "base", f"apriltag_{tag_id}",
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=timeout),
            )
            pose = PoseStamped()
            pose.header.frame_id  = "base"
            pose.header.stamp     = tf.header.stamp
            pose.pose.position.x  = tf.transform.translation.x
            pose.pose.position.y  = tf.transform.translation.y
            pose.pose.position.z  = tf.transform.translation.z
            pose.pose.orientation = tf.transform.rotation
            return pose
        except Exception as e:
            self.get_logger().warn(f"TF 조회 실패: {e}")
            return None

    # =========================================================================
    # future 폴링 헬퍼 (MultiThreadedExecutor와 충돌 없이 대기)
    # =========================================================================
    def _wait_future(self, future, timeout: float) -> bool:
        deadline = time.time() + timeout
        while not future.done():
            if time.time() > deadline:
                return False
            time.sleep(0.05)
        return True

    # =========================================================================
    # MoveIt 이동
    # =========================================================================
    def move_to_joints(self, joint_values: list, timeout: float = 60.0) -> bool:
        constraints = Constraints()
        for name, value in zip(ARM_JOINT_NAMES, joint_values):
            jc = JointConstraint()
            jc.joint_name      = name
            jc.position        = float(value)
            jc.tolerance_above = 0.05
            jc.tolerance_below = 0.05
            jc.weight          = 1.0
            constraints.joint_constraints.append(jc)

        plan_req = MotionPlanRequest()
        plan_req.group_name                      = "arm"
        plan_req.num_planning_attempts           = 10
        plan_req.allowed_planning_time           = 10.0
        plan_req.max_velocity_scaling_factor     = 0.3
        plan_req.max_acceleration_scaling_factor = 0.3
        plan_req.goal_constraints.append(constraints)

        goal_msg = MoveGroup.Goal()
        goal_msg.request                          = plan_req
        goal_msg.planning_options.plan_only       = False
        goal_msg.planning_options.replan          = True
        goal_msg.planning_options.replan_attempts = 3
        goal_msg.planning_options.replan_delay    = 1.0

        future = self._move_client.send_goal_async(goal_msg)
        if not self._wait_future(future, timeout=15.0):
            self.get_logger().error("joint goal 응답 타임아웃")
            return False
        if not future.result().accepted:
            self.get_logger().error("joint goal 거부됨")
            return False

        result_future = future.result().get_result_async()
        if not self._wait_future(result_future, timeout=timeout):
            self.get_logger().error("joint goal 실행 타임아웃")
            return False

        ok = result_future.result().result.error_code.val == 1
        if not ok:
            self.get_logger().warn(
                f"joint goal 실패 (err={result_future.result().result.error_code.val})")
        return ok

    def move_to_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0,
                     pos_tol=0.01, timeout: float = 60.0) -> bool:
        pc = PositionConstraint()
        pc.header.frame_id = "base"
        pc.link_name       = "link6"
        pc.weight          = 1.0
        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type       = SolidPrimitive.SPHERE
        sphere.dimensions = [pos_tol]
        center = Pose()
        center.position.x    = float(x)
        center.position.y    = float(y)
        center.position.z    = float(z)
        center.orientation.w = 1.0
        region.primitives      = [sphere]
        region.primitive_poses = [center]
        pc.constraint_region   = region

        oc = OrientationConstraint()
        oc.header.frame_id           = "base"
        oc.link_name                 = "link6"
        oc.orientation.x             = float(qx)
        oc.orientation.y             = float(qy)
        oc.orientation.z             = float(qz)
        oc.orientation.w             = float(qw)
        oc.absolute_x_axis_tolerance = 0.3
        oc.absolute_y_axis_tolerance = 0.3
        oc.absolute_z_axis_tolerance = 0.3
        oc.weight                    = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        constraints.orientation_constraints.append(oc)

        plan_req = MotionPlanRequest()
        plan_req.group_name                      = "arm"
        plan_req.num_planning_attempts           = 10
        plan_req.allowed_planning_time           = 10.0
        plan_req.max_velocity_scaling_factor     = 0.3
        plan_req.max_acceleration_scaling_factor = 0.3
        plan_req.goal_constraints.append(constraints)

        goal_msg = MoveGroup.Goal()
        goal_msg.request                          = plan_req
        goal_msg.planning_options.plan_only       = False
        goal_msg.planning_options.replan          = True
        goal_msg.planning_options.replan_attempts = 3
        goal_msg.planning_options.replan_delay    = 1.0

        future = self._move_client.send_goal_async(goal_msg)
        if not self._wait_future(future, timeout=15.0):
            self.get_logger().error("pose goal 응답 타임아웃")
            return False
        if not future.result().accepted:
            self.get_logger().error("pose goal 거부됨")
            return False

        result_future = future.result().get_result_async()
        if not self._wait_future(result_future, timeout=timeout):
            self.get_logger().error("pose goal 실행 타임아웃")
            return False

        ok = result_future.result().result.error_code.val == 1
        if not ok:
            self.get_logger().warn(
                f"pose goal 실패 (err={result_future.result().result.error_code.val})")
        return ok

    # =========================================================================
    # 그리퍼
    # =========================================================================
    def set_gripper(self, open_ratio: float) -> bool:
        if not self._gripper_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("/gripper_controller/gripper_action 없음")
            return False

        target = GRIPPER_RAD_CLOSE + open_ratio * (GRIPPER_RAD_OPEN - GRIPPER_RAD_CLOSE)
        goal = GripperCommand.Goal()
        goal.command.position   = target
        goal.command.max_effort = 50.0

        future = self._gripper_client.send_goal_async(goal)
        if not self._wait_future(future, timeout=5.0):
            return False
        if not future.result().accepted:
            return False

        result_future = future.result().get_result_async()
        self._wait_future(result_future, timeout=5.0)
        return result_future.done()

    # =========================================================================
    # 명령 큐
    # =========================================================================
    def _push_cmd(self, cmd: str):
        with self._cmd_lock:
            self._cmd_queue.append(cmd.strip())

    def _pop_cmd(self):
        with self._cmd_lock:
            return self._cmd_queue.pop(0) if self._cmd_queue else None

    def _cmd_topic_cb(self, msg: String):
        self._push_cmd(msg.data)

    def _set_status(self, msg: str):
        self._status_msg = msg
        self.get_logger().info(msg)

    # =========================================================================
    # 상태 핸들러
    # =========================================================================
    def _do_scan_toggle(self):
        """SCAN 버튼: 토글 방식 (한번 누르면 ON, 다시 누르면 OFF)."""
        with self._lock:
            self._scanning = not self._scanning
            now = self._scanning

        if now:
            self._set_status("[SCAN] 태그 탐지 중... RViz 박스 갱신")
        else:
            self._set_status("[SCAN] 중지")
            self._remove_planning_scene_box()

    def _do_pick(self):
        with self._lock:
            detected = self._tag_detected
            tag_id   = self._latest_tag_id

        if not detected or tag_id is None:
            self._set_status("[PICK] 태그 없음 – 먼저 SCAN으로 태그를 확인하세요")
            return

        self._set_status(f"[PICK] 태그 ID={tag_id} TF 조회 중...")
        tag_pose = self._get_tag_pose_in_base(tag_id, timeout=2.0)
        if tag_pose is None:
            self._set_status("[PICK] TF 조회 실패 – 재시도하세요")
            return

        px = tag_pose.pose.position.x
        py = tag_pose.pose.position.y
        pz = tag_pose.pose.position.z
        self._set_status(f"[PICK] 태그 위치 x={px:.3f} y={py:.3f} z={pz:.3f}")

        self._set_status("[PICK] 그리퍼 열기")
        self.set_gripper(1.0)
        time.sleep(0.3)

        pre_z = pz + self.pre_grasp_z_offset
        self._set_status(f"[PICK] Pre-grasp z={pre_z:.3f}")
        if not self.move_to_pose(px, py, pre_z):
            self._set_status("[PICK] Pre-grasp 실패 – 홈 복귀")
            self._go_home()
            return

        self._set_status(f"[PICK] Grasp z={pz:.3f}")
        if not self.move_to_pose(px, py, pz):
            self._set_status("[PICK] Grasp 실패 – 홈 복귀")
            self._go_home()
            return

        self._set_status("[PICK] 그리퍼 닫기")
        self.set_gripper(0.0)
        time.sleep(0.5)
        self._remove_planning_scene_box()

        lift_z = pz + self.lift_z_offset
        self._set_status(f"[PICK] 들어올리기 z={lift_z:.3f}")
        self.move_to_pose(px, py, lift_z)

        self._set_status("[PICK] 홈 복귀")
        self._go_home()
        self._set_status("[PICK] 완료! PLACE 버튼으로 물체를 배치하세요.")

    def _do_place(self):
        px, py, pz = self.place_pos
        self._set_status(f"[PLACE] 목표 x={px} y={py} z={pz}")

        if not self.move_to_pose(px, py, pz):
            self._set_status("[PLACE] 이동 실패")
        else:
            self._set_status("[PLACE] 목표 도달")

        self._set_status("[PLACE] 그리퍼 열기")
        self.set_gripper(1.0)
        time.sleep(0.3)

        self._set_status("[PLACE] 홈 복귀")
        self._go_home()
        self._set_status("[PLACE] 완료!")

    def _go_home(self):
        self.move_to_joints(HOME_JOINTS)

    # =========================================================================
    # 메인 루프
    # =========================================================================
    def run(self):
        while self._running:
            cmd = self._pop_cmd()
            if cmd is None:
                time.sleep(0.05)
                continue

            if self._busy:
                self.get_logger().warn(f"작업 중 – 명령 '{cmd}' 무시됨")
                continue

            if cmd == "scan":
                self._do_scan_toggle()

            elif cmd == "pick":
                self._busy = True
                self._set_status("[PICK] 시작")
                try:
                    self._do_pick()
                finally:
                    self._busy = False

            elif cmd == "place":
                self._busy = True
                self._set_status("[PLACE] 시작")
                try:
                    self._do_place()
                finally:
                    self._busy = False

            elif cmd in ("q", "quit"):
                self._running = False
                break

            else:
                self.get_logger().warn(f"알 수 없는 명령: '{cmd}'")

        self._cap.release()


# ── main ──────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # 비전 루프는 메인 스레드에서 실행 (OpenCV GUI 요구사항)
    vision_thread = threading.Thread(target=node.vision_loop, daemon=True)
    vision_thread.start()

    # 명령 디스패처: 별도 스레드
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node._running = False
        vision_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
