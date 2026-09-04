#!/usr/bin/env python3
"""Continuously publish the best target detection and usable localization mask."""

import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose


class TargetDetector(Node):
    def __init__(self):
        super().__init__("target_detector")
        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/color/image_raw"
        ).value
        self.image_is_compressed = bool(
            self.declare_parameter("image_is_compressed", False).value
        )
        self.backend_name = str(
            self.declare_parameter("backend", "auto").value
        ).strip().lower()
        self.model_path = str(self.declare_parameter("model_path", "").value).strip()
        self.device = str(self.declare_parameter("device", "0").value)
        self.confidence_threshold = float(
            self.declare_parameter("confidence_threshold", 0.45).value
        )
        self.allowed_classes = set(
            str(value)
            for value in self.declare_parameter(
                "allowed_classes",
                ["wood_block", "artificial_object", "rock_sample"],
            ).value
        )
        self.max_inference_hz = float(
            self.declare_parameter("max_inference_hz", 8.0).value
        )
        self.min_area_pixels = int(
            self.declare_parameter("min_area_pixels", 120).value
        )
        self.min_saturation = int(
            self.declare_parameter("min_saturation", 105).value
        )
        self.min_value = int(self.declare_parameter("min_value", 55).value)
        self.max_area_fraction = float(
            self.declare_parameter("max_area_fraction", 0.35).value
        )
        self.ignore_rois = self._parse_ignore_rois(
            self.declare_parameter("ignore_rois", [0, 0, 0, 0]).value
        )

        sensor_qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.bridge = CvBridge()
        self.mask_publisher = self.create_publisher(
            Image, "/perception/target_mask", sensor_qos
        )
        self.debug_publisher = self.create_publisher(
            Image, "/perception/debug/target_overlay", sensor_qos
        )
        self.debug_compressed_publisher = self.create_publisher(
            CompressedImage, "/perception/debug/target_overlay/compressed", sensor_qos
        )
        self.detection_publisher = self.create_publisher(
            Detection2D, "/perception/target_detection", 10
        )
        message_type = CompressedImage if self.image_is_compressed else Image
        self.create_subscription(
            message_type, self.image_topic, self.on_image, sensor_qos
        )
        self.last_inference = 0.0
        self.sequence = 0
        self.model = None
        self.backend = self._load_backend()
        self.get_logger().info(
            "target detector ready: backend=%s, image=%s (%s)"
            % (
                self.backend,
                self.image_topic,
                "compressed" if self.image_is_compressed else "raw",
            )
        )

    def _load_backend(self):
        if self.backend_name not in ("auto", "hsv", "ultralytics"):
            raise ValueError("backend must be auto, hsv, or ultralytics")
        if self.backend_name == "hsv":
            return "hsv"
        if not self.model_path:
            if self.backend_name == "ultralytics":
                raise ValueError("model_path is required for the ultralytics backend")
            self.get_logger().warning(
                "no custom detector weights configured; using high-saturation HSV fallback"
            )
            return "hsv"
        try:
            from ultralytics import YOLO

            self.model = YOLO(self.model_path)
            return "ultralytics"
        except Exception as exc:  # noqa: BLE001
            if self.backend_name == "ultralytics":
                raise RuntimeError(f"failed to load Ultralytics model: {exc}") from exc
            self.get_logger().warning(
                f"Ultralytics backend unavailable ({exc}); using HSV fallback"
            )
            return "hsv"

    def _hsv_detection(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([0, self.min_saturation, self.min_value], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        self._apply_ignore_rois(mask)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        if count <= 1:
            return None
        image_area = image.shape[0] * image.shape[1]
        candidates = []
        for component in range(1, count):
            area = int(stats[component, cv2.CC_STAT_AREA])
            if area < self.min_area_pixels:
                continue
            if area > self.max_area_fraction * image_area:
                continue
            candidates.append((area, component))
        if not candidates:
            return None
        area, component = max(candidates)
        component_mask = (labels == component).astype(np.uint8) * 255
        x = int(stats[component, cv2.CC_STAT_LEFT])
        y = int(stats[component, cv2.CC_STAT_TOP])
        width = int(stats[component, cv2.CC_STAT_WIDTH])
        height = int(stats[component, cv2.CC_STAT_HEIGHT])
        saturation = float(np.median(hsv[:, :, 1][labels == component])) / 255.0
        area_score = min(1.0, area / max(float(self.min_area_pixels * 4), 1.0))
        score = 0.55 * saturation + 0.45 * area_score
        return component_mask, (x, y, width, height), "high_saturation_object", score

    def _ultralytics_detection(self, image):
        results = self.model.predict(
            source=image,
            conf=self.confidence_threshold,
            device=self.device,
            verbose=False,
        )
        if not results:
            return None
        result = results[0]
        if result.boxes is None:
            return None
        names = result.names
        best = None
        for index, box in enumerate(result.boxes):
            class_index = int(box.cls.item())
            class_name = str(names[class_index])
            if self.allowed_classes and class_name not in self.allowed_classes:
                continue
            score = float(box.conf.item())
            if best is None or score > best[0]:
                best = (score, index, class_name)
        if best is None:
            return None
        score, index, class_name = best
        if result.masks is not None:
            raw_mask = result.masks.data[index].detach().cpu().numpy()
            mask = cv2.resize(
                raw_mask,
                (image.shape[1], image.shape[0]),
                interpolation=cv2.INTER_NEAREST,
            )
            mask = (mask >= 0.5).astype(np.uint8) * 255
        else:
            # A self-trained detection model has no instance mask.  Publish a
            # box mask so RGB-D localization remains available; segmentation
            # models transparently replace it with their precise mask.
            xyxy = result.boxes.xyxy[index].detach().cpu().numpy()
            x0, y0, x1, y1 = np.rint(xyxy).astype(int)
            mask = np.zeros(image.shape[:2], dtype=np.uint8)
            cv2.rectangle(mask, (x0, y0), (x1, y1), 255, thickness=-1)
        self._apply_ignore_rois(mask)
        bbox = self._bbox_from_mask(mask)
        if bbox is None:
            return None
        return mask, bbox, class_name, score

    @staticmethod
    def _parse_ignore_rois(raw_value):
        values = [float(value) for value in raw_value]
        if len(values) % 4 != 0:
            raise ValueError("ignore_rois must contain x,y,width,height groups")
        rois = []
        for index in range(0, len(values), 4):
            x, y, width, height = values[index : index + 4]
            rois.append(
                (
                    int(round(x)),
                    int(round(y)),
                    int(round(width)),
                    int(round(height)),
                )
            )
        return rois

    def _apply_ignore_rois(self, mask):
        image_height, image_width = mask.shape[:2]
        for x, y, roi_width, roi_height in getattr(self, "ignore_rois", []):
            x0 = min(max(x, 0), image_width)
            y0 = min(max(y, 0), image_height)
            x1 = min(max(x + roi_width, 0), image_width)
            y1 = min(max(y + roi_height, 0), image_height)
            if x1 > x0 and y1 > y0:
                mask[y0:y1, x0:x1] = 0

    @staticmethod
    def _bbox_from_mask(mask):
        points = cv2.findNonZero(mask)
        if points is None:
            return None
        x, y, width, height = cv2.boundingRect(points)
        return int(x), int(y), int(width), int(height)

    def on_image(self, message):
        now = time.monotonic()
        if self.max_inference_hz > 0.0:
            interval = 1.0 / self.max_inference_hz
            if now - self.last_inference < interval:
                return
        self.last_inference = now
        try:
            image = self._message_to_bgr(message)
            result = (
                self._ultralytics_detection(image)
                if self.backend == "ultralytics"
                else self._hsv_detection(image)
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"target inference failed: {exc}")
            return
        if result is None:
            self._publish_debug_overlay(image, message.header)
            return
        mask, (x, y, width, height), class_name, score = result
        if np.count_nonzero(mask) < self.min_area_pixels:
            self._publish_debug_overlay(image, message.header)
            return

        self.sequence += 1
        detection = Detection2D()
        detection.header = message.header
        detection.id = f"target-{message.header.stamp.sec}-{message.header.stamp.nanosec}-{self.sequence}"
        detection.bbox.center.position.x = float(x + width * 0.5)
        detection.bbox.center.position.y = float(y + height * 0.5)
        detection.bbox.size_x = float(width)
        detection.bbox.size_y = float(height)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = class_name
        hypothesis.hypothesis.score = float(score)
        hypothesis.pose.pose.orientation.w = 1.0
        detection.results.append(hypothesis)
        self.detection_publisher.publish(detection)

        mask_message = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_message.header = message.header
        self.mask_publisher.publish(mask_message)

        overlay = self._make_debug_overlay(
            image, mask, (x, y, width, height), class_name, score
        )
        self._publish_debug_overlay(overlay, message.header)

    def _make_debug_overlay(self, image, mask, bbox, class_name, score):
        x, y, width, height = bbox
        overlay = image.copy()
        overlay[mask > 0] = (
            0.55 * overlay[mask > 0] + 0.45 * np.array([0, 255, 0])
        ).astype(np.uint8)
        cv2.rectangle(overlay, (x, y), (x + width, y + height), (0, 255, 0), 2)
        cv2.putText(
            overlay,
            f"{class_name} {score:.2f}",
            (max(0, x), max(18, y - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )
        return overlay

    def _publish_debug_overlay(self, image, header):
        debug_message = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
        debug_message.header = header
        self.debug_publisher.publish(debug_message)
        ok, encoded = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        if not ok:
            return
        compressed = CompressedImage()
        compressed.header = header
        compressed.format = "jpeg"
        compressed.data = encoded.tobytes()
        self.debug_compressed_publisher.publish(compressed)

    def _message_to_bgr(self, message):
        if not self.image_is_compressed:
            return self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if image is None:
            raise ValueError(
                f"failed to decode compressed image frame, format={message.format!r}"
            )
        return image


def main():
    rclpy.init()
    node = TargetDetector()
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
