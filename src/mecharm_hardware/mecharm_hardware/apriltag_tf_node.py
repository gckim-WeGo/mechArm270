#!/usr/bin/env python3
"""
apriltag_tf_node.py
===================
카메라로 AprilTag를 감지하고 TF를 발행하는 독립 노드.

- 감지된 태그의 pose를 camera_link 프레임 기준으로 TF 발행
- TF buffer를 통해 base 프레임 기준 pose도 계산 가능
- /apriltag/detections 토픽으로 감지 정보 발행 (선택적 구독용)

실행:
  ros2 run mecharm_hardware apriltag_tf
"""

import os
os.environ.setdefault("QT_QPA_PLATFORM", "xcb")

import math
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import TransformStamped, PoseStamped
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster, Buffer, TransformListener

try:
    from pupil_apriltags import Detector
except ImportError:
    raise RuntimeError("pupil_apriltags 미설치: pip install pupil-apriltags")


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


class AprilTagTFNode(Node):

    def __init__(self):
        super().__init__("apriltag_tf")

        self.declare_parameter("camera_index", 0)
        self.declare_parameter("tag_size",     0.025)   # 태그 실물 크기 (m)
        self.declare_parameter("tag_family",   "tag36h11")
        self.declare_parameter("parent_frame", "world")  # TF 부모 프레임

        cam_idx      = self.get_parameter("camera_index").value
        self._tag_size    = self.get_parameter("tag_size").value
        tag_family   = self.get_parameter("tag_family").value
        self._parent = self.get_parameter("parent_frame").value

        # ── TF ───────────────────────────────────────────────────
        self._broadcaster = TransformBroadcaster(self)
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── 감지 ID 목록 퍼블리셔 ──────────────────────────────────
        self._ids_pub = self.create_publisher(
            Int32MultiArray, "/apriltag/detected_ids", 10)

        # ── 카메라 초기화 ──────────────────────────────────────────
        self._cap = cv2.VideoCapture(cam_idx)
        if not self._cap.isOpened():
            self.get_logger().error(f"카메라 {cam_idx} 열기 실패!")

        ret, frame = self._cap.read()
        if ret:
            frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)
            h, w = frame.shape[:2]
        else:
            w, h = 640, 480

        fx = fy = float(w)
        self._camera_params = (fx, fy, w / 2.0, h / 2.0)
        self._camera_matrix = np.array(
            [[fx, 0, w / 2.0], [0, fy, h / 2.0], [0, 0, 1]], dtype=np.float64)
        self._dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # ── AprilTag 검출기 ────────────────────────────────────────
        self._detector = Detector(
            families=tag_family,
            nthreads=2,
            quad_decimate=1.0,
            quad_sigma=0.0,
            refine_edges=1,
            decode_sharpening=0.25,
        )

        self.get_logger().info(
            f"AprilTagTFNode 시작 (카메라={cam_idx}, 태그크기={self._tag_size}m, "
            f"부모프레임={self._parent})"
        )

    def run(self):
        """메인 루프: 프레임 읽기 → 태그 감지 → TF 발행 → 화면 표시"""
        cv2.namedWindow("AprilTag", cv2.WINDOW_NORMAL)

        while rclpy.ok():
            ret, frame = self._cap.read()
            if not ret:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "No Camera", (200, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0, 0, 200), 2)
            else:
                frame = cv2.rotate(frame, cv2.ROTATE_90_COUNTERCLOCKWISE)

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags = self._detector.detect(
                gray,
                estimate_tag_pose=True,
                camera_params=self._camera_params,
                tag_size=self._tag_size,
            )

            detected_ids = []
            for tag in tags:
                self._publish_tf(tag)
                detected_ids.append(tag.tag_id)

                # 시각화
                for pt in tag.corners:
                    cv2.circle(frame, (int(pt[0]), int(pt[1])), 4, (0, 0, 255), -1)
                rvec, _ = cv2.Rodrigues(tag.pose_R)
                cv2.drawFrameAxes(
                    frame, self._camera_matrix, self._dist_coeffs,
                    rvec, tag.pose_t.reshape(3, 1), self._tag_size / 2)

                t = tag.pose_t.flatten()
                label = f"ID:{tag.tag_id} ({t[0]:.2f},{t[1]:.2f},{t[2]:.2f})m"
                cv2.putText(frame, label,
                            (int(tag.center[0]) - 40, int(tag.center[1]) - 15),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 1)

            # 감지 ID 목록 발행
            msg = Int32MultiArray()
            msg.data = detected_ids
            self._ids_pub.publish(msg)

            # 상태 표시
            status = f"Tags: {detected_ids}" if detected_ids else "No tag"
            cv2.putText(frame, status, (6, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (200, 200, 200), 1)

            cv2.imshow("AprilTag", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cv2.destroyAllWindows()
        self._cap.release()

    def _publish_tf(self, tag):
        """
        태그의 world 기준 절대 TF 발행.

        robot_state_publisher가 /joint_states → world→...→camera_link TF를
        이미 발행하고 있으므로, TF buffer에서 world→camera_link를 조회하고
        camera_link 기준 태그 상대 좌표를 더해 world 기준 절대 TF를 발행한다.
        """
        now = self.get_clock().now().to_msg()

        # ── 1단계: camera_link 기준 임시 TF 발행 ─────────────────
        qx, qy, qz, qw = rotation_matrix_to_quaternion(tag.pose_R)
        x, y, z = tag.pose_t.flatten()

        tmp_frame = f"_cam_tag_{tag.tag_id}"
        ts_tmp = TransformStamped()
        ts_tmp.header.stamp    = now
        ts_tmp.header.frame_id = "camera_link"
        ts_tmp.child_frame_id  = tmp_frame
        ts_tmp.transform.translation.x = float(x)
        ts_tmp.transform.translation.y = float(y)
        ts_tmp.transform.translation.z = float(z)
        ts_tmp.transform.rotation.x    = float(qx)
        ts_tmp.transform.rotation.y    = float(qy)
        ts_tmp.transform.rotation.z    = float(qz)
        ts_tmp.transform.rotation.w    = float(qw)
        self._broadcaster.sendTransform(ts_tmp)

        # ── 2단계: TF buffer로 world 기준 절대 좌표 계산 ─────────
        # robot_state_publisher가 world→...→camera_link를 이미 발행 중이므로
        # world → _cam_tag_<id> = world→camera_link + camera_link→tag
        try:
            tf_abs = self._tf_buffer.lookup_transform(
                "world",
                tmp_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1),
            )
        except Exception:
            return  # TF 아직 미준비, 다음 프레임에 재시도

        # ── 3단계: world → apriltag_<id> 최종 TF 발행 ───────────
        ts_world = TransformStamped()
        ts_world.header.stamp    = now
        ts_world.header.frame_id = "world"
        ts_world.child_frame_id  = f"apriltag_{tag.tag_id}"
        ts_world.transform       = tf_abs.transform
        self._broadcaster.sendTransform(ts_world)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagTFNode()

    import threading
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
