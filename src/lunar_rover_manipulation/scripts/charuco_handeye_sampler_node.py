#!/usr/bin/env python3
"""ChArUco overlay and hand-eye sample recorder.

Press ``p`` in the OpenCV window to save one sample containing:

- color image timestamp;
- camera intrinsics;
- estimated camera -> ChArUco-board pose;
- latest arm joint states.
"""

from __future__ import annotations

import json
import math
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, JointState
from std_msgs.msg import String


def stamp_to_float(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def rotation_vector_to_quaternion(rvec):
    rotation, _ = cv2.Rodrigues(rvec)
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (rotation[2, 1] - rotation[1, 2]) / s
        qy = (rotation[0, 2] - rotation[2, 0]) / s
        qz = (rotation[1, 0] - rotation[0, 1]) / s
    else:
        diagonal = np.diag(rotation)
        index = int(np.argmax(diagonal))
        if index == 0:
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            qw = (rotation[2, 1] - rotation[1, 2]) / s
            qx = 0.25 * s
            qy = (rotation[0, 1] + rotation[1, 0]) / s
            qz = (rotation[0, 2] + rotation[2, 0]) / s
        elif index == 1:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            qw = (rotation[0, 2] - rotation[2, 0]) / s
            qx = (rotation[0, 1] + rotation[1, 0]) / s
            qy = 0.25 * s
            qz = (rotation[1, 2] + rotation[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            qw = (rotation[1, 0] - rotation[0, 1]) / s
            qx = (rotation[0, 2] + rotation[2, 0]) / s
            qy = (rotation[1, 2] + rotation[2, 1]) / s
            qz = 0.25 * s
    return [float(qx), float(qy), float(qz), float(qw)]


class CharucoHandeyeSampler(Node):
    def __init__(self):
        super().__init__("charuco_handeye_sampler")
        self.bridge = CvBridge()

        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/color/image_raw"
        ).value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/color/camera_info"
        ).value
        self.joint_state_topic = self.declare_parameter(
            "joint_state_topic", "/joint_states"
        ).value
        self.output_dir = Path(
            self.declare_parameter(
                "output_dir", "generated/calibration/handeye_samples"
            ).value
        )
        self.squares_x = int(self.declare_parameter("squares_x", 7).value)
        self.squares_y = int(self.declare_parameter("squares_y", 10).value)
        self.square_length_m = float(
            self.declare_parameter("square_length_m", 0.025).value
        )
        self.marker_length_m = float(
            self.declare_parameter("marker_length_m", 0.018).value
        )
        self.min_charuco_corners = int(
            self.declare_parameter("min_charuco_corners", 8).value
        )
        self.window_name = self.declare_parameter(
            "window_name", "ChArUco hand-eye sampler"
        ).value
        self.command_topic = self.declare_parameter(
            "command_topic", "/handeye/calibration_command"
        ).value
        self.status_topic = self.declare_parameter(
            "status_topic", "/handeye/calibration_status"
        ).value

        self.dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)
        self.board = cv2.aruco.CharucoBoard_create(
            self.squares_x,
            self.squares_y,
            self.square_length_m,
            self.marker_length_m,
            self.dictionary,
        )
        self.detector_params = cv2.aruco.DetectorParameters_create()

        self.camera_matrix = None
        self.dist_coeffs = None
        self.camera_info_msg = None
        self.latest_joint_state = None
        self.latest_detection = None
        self.latest_frame = None
        self.latest_image_message = None
        self.latest_overlay = None
        self.sample_count = 0
        self.session_dir = self.output_dir / datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.samples_jsonl = self.session_dir / "samples.jsonl"

        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_info, 10)
        self.create_subscription(JointState, self.joint_state_topic, self.on_joint_state, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.create_subscription(String, self.command_topic, self.on_command, 10)
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, status_qos
        )

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, 960, 600)
        self.get_logger().info(
            "ChArUco sampler ready: press 'p' to save, 'q' to quit. Output: %s"
            % self.session_dir
        )
        self.publish_status("ready")

    def publish_status(self, state):
        message = String()
        message.data = json.dumps(
            {
                "state": str(state),
                "samples": int(self.sample_count),
                "output_dir": str(self.session_dir),
            },
            ensure_ascii=False,
        )
        self.status_publisher.publish(message)

    def on_command(self, message):
        command = message.data.strip().lower()
        if command == "p":
            self.save_current_sample()
        elif command == "q":
            self.publish_status("stopping")
            self.get_logger().info("quit requested from web command")
            rclpy.shutdown()

    def on_info(self, message):
        self.camera_info_msg = message
        self.camera_matrix = np.array(message.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(message.d, dtype=np.float64).reshape(-1, 1)

    def on_joint_state(self, message):
        self.latest_joint_state = message

    def on_image(self, message):
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        overlay, detection = self.detect_and_draw(frame.copy())
        self.latest_detection = detection
        self.latest_frame = frame
        self.latest_image_message = message
        self.latest_overlay = overlay

        self.draw_status(overlay, detection)
        cv2.imshow(self.window_name, overlay)
        key = cv2.waitKey(1) & 0xFF
        if key == ord("p"):
            self.save_sample(message, detection, overlay)
        elif key == ord("q"):
            self.publish_status("stopping")
            self.get_logger().info("quit requested from sampler window")
            rclpy.shutdown()

    def save_current_sample(self):
        if (
            self.latest_image_message is None
            or self.latest_detection is None
            or self.latest_overlay is None
        ):
            self.get_logger().warning("sample rejected: no camera frame available")
            self.publish_status("waiting_for_frame")
            return
        self.save_sample(
            self.latest_image_message, self.latest_detection, self.latest_overlay
        )

    def detect_and_draw(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.detector_params
        )
        marker_count = 0 if ids is None else len(ids)
        detection = {
            "ok": False,
            "marker_count": marker_count,
            "charuco_count": 0,
        }
        if ids is None or marker_count == 0:
            return frame, detection

        cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        ret, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners,
            ids,
            gray,
            self.board,
            cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs,
        )
        charuco_count = int(ret) if ret is not None else 0
        detection["charuco_count"] = charuco_count
        if charuco_corners is not None and charuco_ids is not None and charuco_count > 0:
            cv2.aruco.drawDetectedCornersCharuco(
                frame, charuco_corners, charuco_ids, (0, 255, 255)
            )

        if (
            self.camera_matrix is None
            or self.dist_coeffs is None
            or charuco_corners is None
            or charuco_ids is None
            or charuco_count < self.min_charuco_corners
        ):
            return frame, detection

        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners,
            charuco_ids,
            self.board,
            self.camera_matrix,
            self.dist_coeffs,
            None,
            None,
        )
        if not ok:
            return frame, detection

        cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.08)
        detection.update(
            {
                "ok": True,
                "rvec": rvec.reshape(3).astype(float).tolist(),
                "tvec": tvec.reshape(3).astype(float).tolist(),
                "quaternion_xyzw": rotation_vector_to_quaternion(rvec),
            }
        )
        return frame, detection

    def draw_status(self, frame, detection):
        pose_ok = bool(detection.get("ok", False))
        marker_count = int(detection.get("marker_count", 0))
        charuco_count = int(detection.get("charuco_count", 0))
        status = "POSE OK" if pose_ok else ("BOARD PARTIAL" if marker_count else "NO BOARD")
        color = (0, 255, 0) if pose_ok else ((0, 255, 255) if marker_count else (0, 0, 255))
        cv2.rectangle(frame, (8, 8), (560, 96), (0, 0, 0), -1)
        cv2.putText(frame, status, (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
        cv2.putText(
            frame,
            f"markers={marker_count} charuco_corners={charuco_count} samples={self.sample_count}",
            (16, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )
        cv2.putText(
            frame,
            "press p to sample, q to quit",
            (16, 88),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
        )
        if pose_ok:
            tvec = detection["tvec"]
            cv2.putText(
                frame,
                f"camera->board xyz: {tvec[0]:+.3f}, {tvec[1]:+.3f}, {tvec[2]:+.3f} m",
                (16, frame.shape[0] - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )

    def save_sample(self, image_msg, detection, overlay):
        if not detection or not detection.get("ok"):
            self.get_logger().warning("sample rejected: ChArUco pose is not available")
            return
        if self.latest_joint_state is None:
            self.get_logger().warning("sample rejected: no /joint_states received yet")
            return
        if self.camera_info_msg is None:
            self.get_logger().warning("sample rejected: no camera_info received yet")
            return

        self.sample_count += 1
        index = self.sample_count
        image_name = f"sample_{index:03d}_overlay.png"
        cv2.imwrite(str(self.session_dir / image_name), overlay)

        sample = {
            "index": index,
            "image_topic": self.image_topic,
            "image_stamp": stamp_to_float(image_msg.header.stamp),
            "image_frame_id": image_msg.header.frame_id,
            "camera_info_topic": self.camera_info_topic,
            "camera_frame_id": self.camera_info_msg.header.frame_id,
            "camera_matrix": list(self.camera_info_msg.k),
            "distortion": list(self.camera_info_msg.d),
            "board": {
                "type": "charuco",
                "squares_x": self.squares_x,
                "squares_y": self.squares_y,
                "square_length_m": self.square_length_m,
                "marker_length_m": self.marker_length_m,
                "dictionary": "DICT_5X5_1000",
            },
            "camera_T_board": {
                "translation_xyz_m": detection["tvec"],
                "rotation_vector": detection["rvec"],
                "quaternion_xyzw": detection["quaternion_xyzw"],
            },
            "marker_count": detection["marker_count"],
            "charuco_corner_count": detection["charuco_count"],
            "joint_state": {
                "stamp": stamp_to_float(self.latest_joint_state.header.stamp),
                "names": list(self.latest_joint_state.name),
                "positions": [float(x) for x in self.latest_joint_state.position],
            },
            "overlay_image": image_name,
        }
        with self.samples_jsonl.open("a", encoding="utf-8") as handle:
            handle.write(json.dumps(sample, ensure_ascii=False) + "\n")
        self.get_logger().info(
            "saved sample %03d: board z=%.3fm, corners=%d, joints=%s"
            % (
                index,
                detection["tvec"][2],
                detection["charuco_count"],
                ",".join(sample["joint_state"]["names"][:4]),
            )
        )
        self.publish_status("sampling")


def main():
    rclpy.init()
    node = CharucoHandeyeSampler()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
