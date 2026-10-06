#!/usr/bin/env python3
"""Web-console ChArUco overlay and automatic hand-eye sample recorder.

The node has no GUI. It publishes the annotated camera image for the unified
web console and saves a sample after the board and arm have remained stable:

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
import time
import zipfile

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, JointState
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
        self.overlay_topic = self.declare_parameter(
            "overlay_topic", "/handeye/calibration_overlay/compressed"
        ).value
        self.auto_sample_enabled = bool(
            self.declare_parameter("auto_sample_enabled", True).value
        )
        self.auto_sample_stable_sec = float(
            self.declare_parameter("auto_sample_stable_sec", 0.8).value
        )
        self.auto_sample_min_interval_sec = float(
            self.declare_parameter("auto_sample_min_interval_sec", 1.5).value
        )
        self.auto_sample_joint_stable_deg = float(
            self.declare_parameter("auto_sample_joint_stable_deg", 0.6).value
        )
        self.auto_sample_min_joint_change_deg = float(
            self.declare_parameter("auto_sample_min_joint_change_deg", 2.0).value
        )
        self.auto_sample_min_charuco_corners = int(
            self.declare_parameter("auto_sample_min_charuco_corners", 35).value
        )
        self.auto_sample_pose_stable_translation_m = float(
            self.declare_parameter("auto_sample_pose_stable_translation_m", 0.004).value
        )
        self.auto_sample_pose_stable_rotation_deg = float(
            self.declare_parameter("auto_sample_pose_stable_rotation_deg", 1.5).value
        )
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
        self.received_first_frame = False
        self.quit_requested = False
        self.sample_count = 0
        self.last_status_emit = 0.0
        self.last_status_state = "waiting_for_frame"
        self.last_joint_vector = None
        self.joint_stable_since = None
        self.last_detection_for_stability = None
        self.pose_stable_since = None
        self.last_sample_monotonic = None
        self.last_sample_joint_vector = None
        self.archive_path = None
        self.session_dir = self.output_dir / datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.samples_jsonl = self.session_dir / "samples.jsonl"

        # Camera images may be published with sensor-data BEST_EFFORT QoS.
        # A RELIABLE subscriber cannot receive those images; BEST_EFFORT accepts
        # both BEST_EFFORT and RELIABLE camera publishers.
        camera_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_info, camera_qos)
        self.create_subscription(JointState, self.joint_state_topic, self.on_joint_state, 10)
        self.create_subscription(Image, self.image_topic, self.on_image, camera_qos)
        self.create_subscription(String, self.command_topic, self.on_command, 10)
        self.overlay_publisher = self.create_publisher(
            CompressedImage, self.overlay_topic, camera_qos
        )
        status_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            String, self.status_topic, status_qos
        )

        self.get_logger().info(
            "Web ChArUco sampler ready: auto sampling enabled=%s, output=%s"
            % (self.auto_sample_enabled, self.session_dir)
        )
        self.publish_status("waiting_for_frame", force=True)

    def publish_status(self, state=None, force=False, reason=""):
        state = str(state or self.last_status_state)
        now = time.monotonic()
        if not force and now - self.last_status_emit < 0.3:
            return
        self.last_status_emit = now
        self.last_status_state = state
        detection = self.latest_detection or {}
        joint_stable = bool(
            self.joint_stable_since is not None
            and now - self.joint_stable_since >= self.auto_sample_stable_sec
        )
        pose_stable = bool(
            self.pose_stable_since is not None
            and now - self.pose_stable_since >= self.auto_sample_stable_sec
        )
        message = String()
        message.data = json.dumps(
            {
                "state": state,
                "samples": int(self.sample_count),
                "output_dir": str(self.session_dir),
                "archive_path": str(self.archive_path or ""),
                "download_ready": bool(self.archive_path and self.archive_path.exists()),
                "marker_count": int(detection.get("marker_count", 0)),
                "charuco_corners": int(detection.get("charuco_count", 0)),
                "pose_ok": bool(detection.get("ok", False)),
                "joint_stable": joint_stable,
                "pose_stable": pose_stable,
                "auto_sample_enabled": self.auto_sample_enabled,
                "auto_ready": bool(
                    detection.get("ok", False)
                    and int(detection.get("charuco_count", 0))
                    >= self.auto_sample_min_charuco_corners
                    and joint_stable
                    and pose_stable
                ),
                "reason": reason,
            },
            ensure_ascii=False,
        )
        self.status_publisher.publish(message)

    def on_command(self, message):
        command = message.data.strip().lower()
        if command == "q":
            self.publish_status("stopping", force=True, reason="WebUI 请求结束标定")
            self.get_logger().info("quit requested from web command")
            self.quit_requested = True

    def on_info(self, message):
        self.camera_info_msg = message
        self.camera_matrix = np.array(message.k, dtype=np.float64).reshape(3, 3)
        self.dist_coeffs = np.array(message.d, dtype=np.float64).reshape(-1, 1)

    def on_joint_state(self, message):
        self.latest_joint_state = message
        vector = self._joint_vector()
        if vector is None:
            return
        now = time.monotonic()
        if self.last_joint_vector is None:
            self.joint_stable_since = now
        elif np.max(np.abs(vector - self.last_joint_vector)) > math.radians(
            self.auto_sample_joint_stable_deg
        ):
            self.joint_stable_since = now
        elif self.joint_stable_since is None:
            self.joint_stable_since = now
        self.last_joint_vector = vector

    def _joint_vector(self):
        if self.latest_joint_state is None:
            return None
        names = ["arm_joint1", "arm_joint2", "arm_joint3", "arm_joint4", "arm_joint5"]
        values = dict(zip(self.latest_joint_state.name, self.latest_joint_state.position))
        if not all(name in values for name in names):
            return None
        return np.asarray([float(values[name]) for name in names], dtype=float)

    @staticmethod
    def _rotation_delta_deg(first_rvec, second_rvec):
        first, _ = cv2.Rodrigues(np.asarray(first_rvec, dtype=float))
        second, _ = cv2.Rodrigues(np.asarray(second_rvec, dtype=float))
        relative = first.T @ second
        cosine = np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0)
        return math.degrees(math.acos(float(cosine)))

    def _update_pose_stability(self, detection):
        if not detection.get("ok"):
            self.last_detection_for_stability = None
            self.pose_stable_since = None
            return
        now = time.monotonic()
        previous = self.last_detection_for_stability
        stable = previous is not None
        if stable:
            translation_delta = float(
                np.linalg.norm(
                    np.asarray(detection["tvec"]) - np.asarray(previous["tvec"])
                )
            )
            rotation_delta = self._rotation_delta_deg(
                previous["rvec"], detection["rvec"]
            )
            stable = (
                translation_delta <= self.auto_sample_pose_stable_translation_m
                and rotation_delta <= self.auto_sample_pose_stable_rotation_deg
            )
        if not stable:
            self.pose_stable_since = now
        elif self.pose_stable_since is None:
            self.pose_stable_since = now
        self.last_detection_for_stability = {
            "tvec": list(detection["tvec"]),
            "rvec": list(detection["rvec"]),
        }

    def on_image(self, message):
        frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        overlay, detection = self.detect_and_draw(frame.copy())
        self.latest_detection = detection
        self.latest_frame = frame
        self.latest_image_message = message
        self.latest_overlay = overlay

        self._update_pose_stability(detection)
        self.draw_status(overlay, detection)
        overlay_message = self.bridge.cv2_to_compressed_imgmsg(
            overlay, dst_format="jpg"
        )
        overlay_message.header = message.header
        self.overlay_publisher.publish(overlay_message)
        if not self.received_first_frame:
            self.received_first_frame = True
            self.publish_status("ready", force=True, reason="相机画面已接入，等待稳定")
            self.get_logger().info("camera frame received; web calibration overlay is live")
        self.auto_sample(message, detection, overlay)

    def auto_sample(self, image_msg, detection, overlay):
        if not self.auto_sample_enabled:
            self.publish_status("ready", reason="自动采样已关闭")
            return
        now = time.monotonic()
        corners = int(detection.get("charuco_count", 0))
        current_joints = self._joint_vector()
        stable_sec = min(
            now - self.joint_stable_since if self.joint_stable_since else 0.0,
            now - self.pose_stable_since if self.pose_stable_since else 0.0,
        )
        if not detection.get("ok") or corners < self.auto_sample_min_charuco_corners:
            self.publish_status("ready", reason=f"角点不足：{corners}/{self.auto_sample_min_charuco_corners}")
            return
        if current_joints is None or self.joint_stable_since is None or self.pose_stable_since is None:
            self.publish_status("ready", reason="等待机械臂反馈")
            return
        if stable_sec < self.auto_sample_stable_sec:
            self.publish_status("stabilizing", reason=f"稳定中：{stable_sec:.1f}/{self.auto_sample_stable_sec:.1f}s")
            return
        if self.last_sample_monotonic is not None and now - self.last_sample_monotonic < self.auto_sample_min_interval_sec:
            self.publish_status("ready", reason="刚完成采样，等待下一姿态")
            return
        if self.last_sample_joint_vector is not None and np.max(
            np.abs(current_joints - self.last_sample_joint_vector)
        ) < math.radians(self.auto_sample_min_joint_change_deg):
            self.publish_status("ready", reason="姿态未改变，移动机械臂后自动采样")
            return
        self.save_sample(image_msg, detection, overlay)
        self.last_sample_monotonic = now
        self.last_sample_joint_vector = current_joints.copy()
        self.publish_status("sampling", force=True, reason="已自动保存稳定姿态")

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

        # The web console keeps the camera's required 180° display rotation.
        # Do not rotate the full overlay here: marker geometry must stay in
        # the camera pixel coordinates so it remains registered to the board.
        # Marker-id labels are redrawn with the same pre-rotation as the
        # status text below.  This keeps the ids readable after the web view's
        # 180° transform without touching the camera pixels.
        cv2.aruco.drawDetectedMarkers(frame, corners, None)
        for marker_corners, marker_id in zip(corners, ids.reshape(-1)):
            center = np.mean(marker_corners.reshape(4, 2), axis=0)
            self._put_text_for_rotated_web_view(
                frame,
                str(int(marker_id)),
                (int(center[0] - 6), int(center[1] + 5)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                2,
            )
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
        # The web image is rotated 180° for the physically inverted camera.
        # Render text upside-down in the source frame so that the web rotation
        # makes it readable again.  This changes only glyph orientation; the
        # black status panel remains at the same image location.
        self._put_text_for_rotated_web_view(
            frame, status, (16, 34), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2
        )
        self._put_text_for_rotated_web_view(
            frame,
            f"markers={marker_count} charuco_corners={charuco_count} samples={self.sample_count}",
            (16, 62),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
        )
        self._put_text_for_rotated_web_view(
            frame,
            "AUTO SAMPLE: keep board visible and arm still; finish from web console",
            (16, 88),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            1,
        )
        if pose_ok:
            tvec = detection["tvec"]
            self._put_text_for_rotated_web_view(
                frame,
                f"camera->board xyz: {tvec[0]:+.3f}, {tvec[1]:+.3f}, {tvec[2]:+.3f} m",
                (16, frame.shape[0] - 18),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 0),
                2,
            )

    @staticmethod
    def _put_text_for_rotated_web_view(frame, text, origin, font, scale, color, thickness):
        """Draw text that becomes upright after the web UI's 180° rotation.

        ``origin`` follows OpenCV's normal baseline convention in source-image
        coordinates.  Only the text glyphs are flipped; the camera pixels and
        their detection coordinates are left untouched.
        """
        (text_width, text_height), baseline = cv2.getTextSize(
            text, font, scale, thickness
        )
        padding = max(2, thickness + 1)
        patch = np.zeros(
            (text_height + baseline + 2 * padding, text_width + 2 * padding, 3),
            dtype=np.uint8,
        )
        cv2.putText(
            patch,
            text,
            (padding, text_height + padding),
            font,
            scale,
            color,
            thickness,
            cv2.LINE_AA,
        )
        patch = cv2.flip(patch, -1)
        x = int(origin[0] - padding)
        y = int(origin[1] - text_height - padding)
        x0 = max(0, x)
        y0 = max(0, y)
        x1 = min(frame.shape[1], x + patch.shape[1])
        y1 = min(frame.shape[0], y + patch.shape[0])
        if x0 >= x1 or y0 >= y1:
            return
        patch_x0 = x0 - x
        patch_y0 = y0 - y
        patch_crop = patch[patch_y0:patch_y0 + (y1 - y0), patch_x0:patch_x0 + (x1 - x0)]
        mask = np.any(patch_crop != 0, axis=2)
        frame_crop = frame[y0:y1, x0:x1]
        frame_crop[mask] = patch_crop[mask]

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

    def finish_session(self):
        if self.archive_path is None:
            archive = self.output_dir / f"{self.session_dir.name}.zip"
            manifest = {
                "session_dir": str(self.session_dir),
                "samples": self.sample_count,
                "automatic_sampling": self.auto_sample_enabled,
                "min_charuco_corners": self.auto_sample_min_charuco_corners,
                "stable_sec": self.auto_sample_stable_sec,
                "joint_stable_deg": self.auto_sample_joint_stable_deg,
                "pose_stable_translation_m": self.auto_sample_pose_stable_translation_m,
                "pose_stable_rotation_deg": self.auto_sample_pose_stable_rotation_deg,
                "generated_at": datetime.now().isoformat(timespec="seconds"),
            }
            manifest_path = self.session_dir / "session_manifest.json"
            manifest_path.write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n")
            with zipfile.ZipFile(archive, "w", compression=zipfile.ZIP_DEFLATED) as bundle:
                for file_path in sorted(self.session_dir.iterdir()):
                    if file_path.is_file():
                        bundle.write(file_path, arcname=file_path.name)
            self.archive_path = archive
        self.publish_status("stopped", force=True, reason="采样文件已打包，可从 WebUI 下载")
        self.get_logger().info("saved hand-eye session archive: %s", self.archive_path)


def main():
    rclpy.init()
    node = CharucoHandeyeSampler()
    try:
        # Shut down outside callbacks so command-triggered exit cannot block
        # executor cleanup while a callback is still executing.
        while rclpy.ok() and not node.quit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.finish_session()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
