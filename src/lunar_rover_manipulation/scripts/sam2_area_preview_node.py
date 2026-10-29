#!/usr/bin/env python3
"""Low-frequency, prompt-free SAM2 mask preview with RGB-D projected-area gating."""

from concurrent.futures import ThreadPoolExecutor
from functools import partial
from pathlib import Path
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class Sam2AreaPreview(Node):
    def __init__(self):
        super().__init__("sam2_area_preview")
        self.color_topic = str(self.declare_parameter("color_topic", "/camera/color/image_raw").value)
        self.depth_topic = str(self.declare_parameter("depth_topic", "/camera/depth/image_raw").value)
        self.camera_info_topic = str(
            self.declare_parameter("camera_info_topic", "/camera/color/camera_info").value
        )
        self.model_path = str(
            self.declare_parameter(
                "sam_model", "/home/phragmund/LunarRover/sam2_t.pt"
            ).value
        )
        self.inference_period_sec = float(
            self.declare_parameter("inference_period_sec", 2.0).value
        )
        self.inference_size = int(self.declare_parameter("imgsz", 640).value)
        self.points_per_side = int(self.declare_parameter("points_per_side", 16).value)
        self.min_depth_points = int(self.declare_parameter("min_depth_points", 30).value)
        self.sync_tolerance_sec = float(
            self.declare_parameter("sync_tolerance_sec", 0.08).value
        )
        self.depth_min_m = float(self.declare_parameter("depth_min_m", 0.17).value)
        self.depth_max_m = float(self.declare_parameter("depth_max_m", 1.20).value)
        # Circumscribed sphere for a 50 x 50 x 50 mm cube, before any HSV-only
        # margin; this version uses the user's separate +10% projected-area gate.
        self.sphere_radius_m = float(np.sqrt(3.0) * 0.025)
        self.area_limit_ratio = float(self.declare_parameter("area_limit_ratio", 1.10).value)

        if not Path(self.model_path).is_file():
            raise FileNotFoundError(f"SAM2 weights not found: {self.model_path}")
        from ultralytics import SAM

        self.sam = SAM(self.model_path)
        # Ultralytics exposes full-image automatic masks when no prompts are
        # supplied. Its public predict config does not expose points_stride, so
        # set a lower-density point grid on the predictor's mask generator.
        predictor_class = self.sam._smart_load("predictor")
        predictor = predictor_class(
            overrides={**self.sam.overrides, "task": "segment", "mode": "predict",
                       "imgsz": self.inference_size, "device": 0},
            _callbacks=self.sam.callbacks,
        )
        predictor.setup_model(model=self.sam.model, verbose=False)
        predictor.generate = partial(predictor.generate, points_stride=self.points_per_side)
        self.sam.predictor = predictor
        self.bridge = CvBridge()
        self.depths = []
        self.camera_info = None
        self.latest_color = None
        self.worker_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix="sam2-preview")
        self.inference_future = None
        self.inference_frame = None
        self.last_inference_start = 0.0
        self.latest_masks = None
        self.segmented_view = None
        self.last_inference_status = "SAM2 starting"
        self.window = "纯SAM2分割版本"

        self.create_subscription(Image, self.color_topic, self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.on_camera_info, qos_profile_sensor_data
        )
        self.create_timer(0.04, self.render)
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        self.get_logger().info(
            f"Prompt-free SAM2 preview: model={self.model_path}, "
            f"period={self.inference_period_sec:.1f}s, points/side={self.points_per_side}, "
            f"projected mask area limit={self.area_limit_ratio:.0%}."
        )

    @staticmethod
    def stamp_seconds(header):
        return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9

    def on_color(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, "bgr8")
            self.latest_color = (message, image)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"color conversion failed: {exc}")

    def on_depth(self, message):
        try:
            depth = np.asarray(self.bridge.imgmsg_to_cv2(message, "passthrough"))
            if depth.ndim != 2:
                return
            depth_m = depth.astype(np.float32)
            if depth.dtype == np.uint16:
                depth_m *= 0.001
            elif "32f" not in str(message.encoding).lower():
                return
            self.depths.append((message, depth_m))
            self.depths = self.depths[-8:]
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"depth conversion failed: {exc}")

    def on_camera_info(self, message):
        self.camera_info = message

    def matched_depth(self, color_message):
        if not self.depths:
            return None
        stamp = self.stamp_seconds(color_message.header)
        depth_message, depth = min(
            self.depths,
            key=lambda item: abs(self.stamp_seconds(item[0].header) - stamp),
        )
        if abs(self.stamp_seconds(depth_message.header) - stamp) > self.sync_tolerance_sec:
            return None
        return depth

    def infer_masks(self, image):
        """Run SAM2 automatic mask generation without H/S or object prompts."""
        results = self.sam.predict(
            image,
            imgsz=self.inference_size,
            verbose=False,
            device=0,
        )
        masks = []
        if results and results[0].masks is not None:
            for mask in results[0].masks.data.detach().cpu().numpy():
                if mask.shape != image.shape[:2]:
                    mask = cv2.resize(
                        mask.astype(np.uint8),
                        (image.shape[1], image.shape[0]),
                        interpolation=cv2.INTER_NEAREST,
                    )
                masks.append(mask > 0)
        return masks

    def start_inference_if_due(self, image, depth_m):
        now = time.monotonic()
        if self.inference_future is not None:
            if not self.inference_future.done():
                return
            try:
                self.latest_masks = self.inference_future.result()
                self.segmented_view = self.inference_frame
                self.last_inference_status = f"SAM2 masks: {len(self.latest_masks)}"
                self.last_inference_start = time.monotonic()
                self.get_logger().info(self.last_inference_status)
            except Exception as exc:  # noqa: BLE001
                self.latest_masks = None
                self.segmented_view = None
                self.last_inference_status = f"SAM2 error: {exc}"
                self.get_logger().error(self.last_inference_status)
            self.inference_future = None
            self.inference_frame = None
        if (
            now - self.last_inference_start < self.inference_period_sec
            or depth_m is None
            or depth_m.shape != image.shape[:2]
            or self.camera_info is None
        ):
            return
        self.last_inference_start = now
        self.inference_frame = (image.copy(), depth_m.copy())
        self.inference_future = self.worker_pool.submit(self.infer_masks, image.copy())
        self.last_inference_status = "SAM2 segmenting..."

    def gate_masks_by_projected_area(self, masks, depth_m):
        if self.camera_info is None:
            return [], "waiting for matching camera info / registered depth"
        if not masks:
            return [], "SAM2 returned no masks"
        if depth_m.shape != masks[0].shape:
            return [], "depth and SAM2 mask resolutions differ"
        fx, fy = float(self.camera_info.k[0]), float(self.camera_info.k[4])
        if min(fx, fy) <= 1e-6:
            return [], "invalid camera intrinsics"
        accepted = []
        rejected = 0
        invalid_depth = 0
        for mask in masks:
            ys, xs = np.nonzero(mask)
            area_px = len(xs)
            if area_px == 0:
                continue
            d = depth_m[ys, xs]
            valid = np.isfinite(d) & (d >= self.depth_min_m) & (d <= self.depth_max_m)
            if int(np.count_nonzero(valid)) < self.min_depth_points:
                invalid_depth += 1
                continue
            mean_depth = float(np.mean(d[valid]))
            # Pixel footprint of a sphere at this depth: pi * rx * ry. The
            # visible surface depth is near the sphere front, so estimate its
            # center one radius behind it for perspective projection.
            center_depth = mean_depth + self.sphere_radius_m
            denominator = np.sqrt(max(center_depth * center_depth - self.sphere_radius_m**2, 1e-9))
            rx = fx * self.sphere_radius_m / denominator
            ry = fy * self.sphere_radius_m / denominator
            expected_area_px = float(np.pi * rx * ry)
            if area_px > expected_area_px * self.area_limit_ratio:
                rejected += 1
                continue
            accepted.append((mask, area_px, mean_depth, expected_area_px))
        return accepted, f"pass={len(accepted)}  too-large={rejected}  no-depth={invalid_depth}"

    def render(self):
        if self.latest_color is None:
            return
        message, image = self.latest_color
        depth_m = self.matched_depth(message)
        self.start_inference_if_due(image, depth_m)
        view_image, view_depth = image, depth_m
        if self.segmented_view is not None:
            view_image, view_depth = self.segmented_view
        frame = cv2.rotate(view_image.copy(), cv2.ROTATE_180)
        height, width = frame.shape[:2]
        accepted, gate_status = [], "waiting for synchronized depth"
        if view_depth is not None and self.latest_masks is not None:
            accepted, gate_status = self.gate_masks_by_projected_area(self.latest_masks, view_depth)
            for mask, area_px, depth, expected_px in accepted:
                ys, xs = np.nonzero(mask)
                if len(xs) == 0:
                    continue
                x1, x2 = width - 1 - int(xs.max()), width - 1 - int(xs.min())
                y1, y2 = height - 1 - int(ys.max()), height - 1 - int(ys.min())
                cv2.rectangle(frame, (x1, y1), (x2, y2), (40, 220, 40), 2)
                label = f"PASS {area_px}px <= {expected_px * self.area_limit_ratio:.0f}px @ {depth:.2f}m"
                cv2.putText(frame, label, (x1, max(20, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.42, (40, 220, 40), 1, cv2.LINE_AA)
        cv2.putText(frame, "Prompt-free SAM2 | 50mm cube circumsphere", (12, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, self.last_inference_status, (12, 48),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(frame, gate_status, (12, 70),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.imshow(self.window, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            rclpy.shutdown()

    def destroy_node(self):
        self.worker_pool.shutdown(wait=False, cancel_futures=True)
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = Sam2AreaPreview()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
