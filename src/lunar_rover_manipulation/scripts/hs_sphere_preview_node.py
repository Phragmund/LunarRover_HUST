#!/usr/bin/env python3
"""Standalone RGB-D preview of H/S regions gated by a 50 mm cube's circumsphere."""

from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class HsSpherePreview(Node):
    def __init__(self):
        super().__init__("hs_sphere_preview")
        self.color_topic = self.declare_parameter(
            "color_topic", "/camera/color/image_raw"
        ).value
        self.depth_topic = self.declare_parameter(
            "depth_topic", "/camera/depth/image_raw"
        ).value
        self.camera_info_topic = self.declare_parameter(
            "camera_info_topic", "/camera/color/camera_info"
        ).value
        self.min_area = int(self.declare_parameter("min_area_pixels", 300).value)
        self.min_saturation = int(self.declare_parameter("min_saturation", 45).value)
        self.min_value = int(self.declare_parameter("min_value", 25).value)
        self.hue_tolerance = float(self.declare_parameter("hue_tolerance", 10.0).value)
        self.saturation_tolerance = float(
            self.declare_parameter("saturation_tolerance", 48.0).value
        )
        self.min_boundary_hue_delta = float(
            self.declare_parameter("min_boundary_hue_delta", 6.0).value
        )
        self.min_boundary_saturation_delta = float(
            self.declare_parameter("min_boundary_saturation_delta", 20.0).value
        )
        self.sync_tolerance_sec = float(
            self.declare_parameter("sync_tolerance_sec", 0.08).value
        )
        self.min_depth_points = int(
            self.declare_parameter("min_depth_points", 30).value
        )
        # A 50 mm cube's circumscribed sphere has radius sqrt(3) * 25 mm;
        # add the requested 10% margin to avoid rejecting borderline pixels.
        self.sphere_radius_m = float(
            self.declare_parameter("sphere_radius_m", np.sqrt(3.0) * 0.025 * 1.10).value
        )
        self.depth_min_m = float(self.declare_parameter("depth_min_m", 0.17).value)
        self.depth_max_m = float(self.declare_parameter("depth_max_m", 1.20).value)
        self.ignore_rois = list(self.declare_parameter(
            "ignore_rois", [0, 0, 40, 120, 90, 0, 90, 90, 185, 0, 120, 85, 610, 0, 30, 85]
        ).value)
        self.bridge = CvBridge()
        self.depths = deque(maxlen=8)
        self.camera_info = None
        self.latest_color = None
        self.window = "HS色块提取版本"

        self.create_subscription(Image, self.color_topic, self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.on_camera_info, qos_profile_sensor_data
        )
        self.create_timer(0.05, self.render)
        cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
        self.get_logger().info(
            f"Previewing {self.color_topic} + {self.depth_topic}; "
            f"circumsphere radius={self.sphere_radius_m * 1000:.1f} mm. "
            "Press q or Esc in the image window to close."
        )

    @staticmethod
    def stamp_seconds(header):
        return float(header.stamp.sec) + float(header.stamp.nanosec) * 1e-9

    @staticmethod
    def circular_hue_center(hues):
        angles = np.asarray(hues, dtype=np.float64) * (2.0 * np.pi / 180.0)
        angle = np.arctan2(np.mean(np.sin(angles)), np.mean(np.cos(angles)))
        return float((angle % (2.0 * np.pi)) * (180.0 / (2.0 * np.pi)))

    def on_color(self, message):
        try:
            self.latest_color = (message, self.bridge.imgmsg_to_cv2(message, "bgr8"))
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
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"depth conversion failed: {exc}")

    def on_camera_info(self, message):
        self.camera_info = message

    def matched_depth(self, color_message):
        if not self.depths:
            return None
        color_stamp = self.stamp_seconds(color_message.header)
        depth_message, depth = min(
            self.depths,
            key=lambda item: abs(self.stamp_seconds(item[0].header) - color_stamp),
        )
        if abs(self.stamp_seconds(depth_message.header) - color_stamp) > self.sync_tolerance_sec:
            return None
        return depth

    def proposals(self, image, depth_m):
        if self.camera_info is None or depth_m.shape != image.shape[:2]:
            return [], "waiting for matching camera info / registered depth"
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        hue = hsv[:, :, 0].astype(np.float32)
        saturation = hsv[:, :, 1].astype(np.float32)
        value = hsv[:, :, 2]
        eligible = (saturation >= self.min_saturation) & (value >= self.min_value)
        for index in range(0, len(self.ignore_rois) - 3, 4):
            x, y, width, height = [int(v) for v in self.ignore_rois[index:index + 4]]
            x0, y0 = max(0, x), max(0, y)
            x1, y1 = min(eligible.shape[1], x + width), min(eligible.shape[0], y + height)
            if x1 > x0 and y1 > y0:
                eligible[y0:y1, x0:x1] = False
        if np.count_nonzero(eligible) < self.min_area:
            return [], "no saturated pixels"

        histogram, _, _ = np.histogram2d(
            hue[eligible], saturation[eligible], bins=(36, 16),
            range=((0.0, 180.0), (0.0, 256.0)),
        )
        wrapped = np.concatenate((histogram[-2:], histogram, histogram[:2]), axis=0)
        smooth = cv2.GaussianBlur(wrapped.astype(np.float32), (5, 3), 0)[2:-2]
        peaks = []
        for index in np.argsort(smooth.ravel())[::-1]:
            h_bin, s_bin = np.unravel_index(index, smooth.shape)
            if smooth[h_bin, s_bin] < self.min_area * 0.25:
                break
            center_h, center_s = (h_bin + 0.5) * 5.0, (s_bin + 0.5) * 16.0
            if any(
                min(abs(center_h - old_h), 180.0 - abs(center_h - old_h)) <= 12.0
                and abs(center_s - old_s) <= 40.0
                for old_h, old_s in peaks
            ):
                continue
            peaks.append((center_h, center_s))
            if len(peaks) >= 16:
                break

        found = []
        fx, fy = float(self.camera_info.k[0]), float(self.camera_info.k[4])
        cx, cy = float(self.camera_info.k[2]), float(self.camera_info.k[5])
        if min(fx, fy) <= 1e-6:
            return [], "invalid camera intrinsics"
        open_kernel, close_kernel = np.ones((3, 3), np.uint8), np.ones((5, 5), np.uint8)
        for center_h, center_s in peaks:
            hue_delta = np.abs(hue - center_h)
            hue_delta = np.minimum(hue_delta, 180.0 - hue_delta)
            mask = (
                eligible & (hue_delta <= self.hue_tolerance)
                & (np.abs(saturation - center_s) <= self.saturation_tolerance)
            ).astype(np.uint8) * 255
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
            count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
            for label in range(1, count):
                area = int(stats[label, cv2.CC_STAT_AREA])
                if area < self.min_area:
                    continue
                component = labels == label
                ys, xs = np.nonzero(component)
                d = depth_m[ys, xs]
                valid = np.isfinite(d) & (d >= self.depth_min_m) & (d <= self.depth_max_m)
                if np.count_nonzero(valid) < self.min_depth_points:
                    found.append((component, "REJECT: insufficient depth", False))
                    continue
                xs_v, ys_v, z = xs[valid].astype(np.float64), ys[valid].astype(np.float64), d[valid]
                points = np.column_stack(((xs_v - cx) * z / fx, (ys_v - cy) * z / fy, z))
                front = np.median(points, axis=0)
                # RGB-D sees the near surface. Approximate the sphere center one
                # radius behind it along optical Z, then project its silhouette.
                sphere_center = front.copy()
                sphere_center[2] += self.sphere_radius_m
                if sphere_center[2] <= self.sphere_radius_m:
                    found.append((component, "REJECT: too near", False))
                    continue
                u0 = fx * sphere_center[0] / sphere_center[2] + cx
                v0 = fy * sphere_center[1] / sphere_center[2] + cy
                f = 0.5 * (fx + fy)
                radius_px = f * self.sphere_radius_m / np.sqrt(
                    sphere_center[2] ** 2 - self.sphere_radius_m ** 2
                )
                # Check every segmented region pixel, not just its bbox. A 1 px
                # raster allowance accounts only for pixel-center discretization.
                outside = np.hypot(xs - u0, ys - v0) > radius_px + 1.0
                if np.any(outside):
                    found.append((component, "REJECT: outside sphere", False))
                    continue

                # Keep the prior chromatic-boundary condition so uniform scene
                # areas do not become candidates merely by fitting the circle.
                inner = cv2.dilate(component.astype(np.uint8), np.ones((3, 3), np.uint8)) > 0
                outer = cv2.dilate(component.astype(np.uint8), np.ones((13, 13), np.uint8)) > 0
                ring = outer & ~inner & eligible
                active = component
                if np.count_nonzero(ring) < max(20, area // 20):
                    found.append((component, "REJECT: no boundary", False))
                    continue
                region_h = self.circular_hue_center(hue[active])
                ring_h = self.circular_hue_center(hue[ring])
                hue_gap = min(abs(region_h - ring_h), 180.0 - abs(region_h - ring_h))
                saturation_gap = abs(float(np.median(saturation[active])) - float(np.median(saturation[ring])))
                if hue_gap < self.min_boundary_hue_delta and saturation_gap < self.min_boundary_saturation_delta:
                    found.append((component, "REJECT: weak color edge", False))
                    continue
                found.append((component, f"PASS {radius_px:.0f}px / {np.median(z):.2f}m", True))
        return found, ""

    def render(self):
        if self.latest_color is None:
            return
        message, image = self.latest_color
        self.latest_color = None
        # Keep the camera view inverted, but draw all UI annotations afterward
        # so boxes and text remain upright for the operator.
        frame = cv2.rotate(image.copy(), cv2.ROTATE_180)
        depth_m = self.matched_depth(message)
        if depth_m is None:
            cv2.putText(frame, "Waiting for synchronized depth", (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 180, 255), 2)
        else:
            regions, status = self.proposals(image, depth_m)
            height, width = frame.shape[:2]
            for component, label, passed in regions:
                ys, xs = np.nonzero(component)
                x1, x2, y1, y2 = int(xs.min()), int(xs.max()), int(ys.min()), int(ys.max())
                x1, x2 = width - 1 - x2, width - 1 - x1
                y1, y2 = height - 1 - y2, height - 1 - y1
                color = (40, 220, 40) if passed else (30, 30, 235)
                cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
                cv2.putText(frame, label, (x1, max(20, y1 - 6)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1, cv2.LINE_AA)
            cv2.putText(frame, f"H/S components: {len(regions)}  green=passes circumsphere",
                        (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
            if status:
                cv2.putText(frame, status, (12, 50), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 180, 255), 1)
        cv2.imshow(self.window, frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord("q")):
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = HsSpherePreview()
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
