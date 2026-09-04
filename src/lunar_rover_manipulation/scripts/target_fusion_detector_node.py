#!/usr/bin/env python3
"""HSV proposal, self-trained YOLO confirmation, and optional SAM masking."""

import time
from concurrent.futures import ThreadPoolExecutor

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2D, ObjectHypothesisWithPose


class HsvProposal:
    def __init__(self, mask, bbox, score, area):
        self.mask = mask
        self.bbox = bbox
        self.score = score
        self.area = area
        self.class_id = "hsv_candidate"
        self.stage = "hsv"


class HsvLockProfile:
    def __init__(self, hue, saturation, value):
        self.hue = float(hue)
        self.saturation = float(saturation)
        self.value = float(value)


def parse_rois(raw_value):
    values = [float(value) for value in raw_value]
    if len(values) % 4 != 0:
        raise ValueError("ignore_rois must contain x,y,width,height groups")
    rois = []
    for index in range(0, len(values), 4):
        x, y, width, height = values[index : index + 4]
        rois.append(
            (int(round(x)), int(round(y)), int(round(width)), int(round(height)))
        )
    return rois


def apply_ignore_rois(mask, rois):
    image_height, image_width = mask.shape[:2]
    for x, y, roi_width, roi_height in rois:
        x0 = min(max(x, 0), image_width)
        y0 = min(max(y, 0), image_height)
        x1 = min(max(x + roi_width, 0), image_width)
        y1 = min(max(y + roi_height, 0), image_height)
        if x1 > x0 and y1 > y0:
            mask[y0:y1, x0:x1] = 0


def bbox_from_mask(mask):
    points = cv2.findNonZero(mask)
    if points is None:
        return None
    x, y, width, height = cv2.boundingRect(points)
    return int(x), int(y), int(width), int(height)


def expanded_bbox(bbox, image_shape, scale):
    x, y, width, height = bbox
    image_height, image_width = image_shape[:2]
    pad_x = int(round(width * scale))
    pad_y = int(round(height * scale))
    x0 = max(0, x - pad_x)
    y0 = max(0, y - pad_y)
    x1 = min(image_width, x + width + pad_x)
    y1 = min(image_height, y + height + pad_y)
    return x0, y0, x1 - x0, y1 - y0


def clip_mask_to_bbox(mask, bbox):
    """Keep a SAM candidate inside the HSV proposal's expanded prompt box."""
    x, y, width, height = bbox
    clipped = np.zeros_like(mask)
    clipped[y : y + height, x : x + width] = mask[y : y + height, x : x + width]
    return clipped


class TargetFusionDetector(Node):
    def __init__(self):
        super().__init__("target_fusion_detector")
        self.image_topic = self.declare_parameter(
            "image_topic", "/camera/color/image_raw/compressed"
        ).value
        self.image_is_compressed = bool(
            self.declare_parameter("image_is_compressed", True).value
        )
        self.camera_info_topic = str(
            self.declare_parameter("camera_info_topic", "/camera/color/camera_info").value
        )
        self.work_center_frame = str(
            self.declare_parameter("work_center_frame", "handeye_tool_frame").value
        )
        self.draw_work_center = bool(
            self.declare_parameter("draw_work_center", True).value
        )
        self.tf_target_pixel_topic = str(
            self.declare_parameter(
                "tf_target_pixel_topic", "/grasp/tf_target_pixel"
            ).value
        )
        self.use_manual_tool_center_pixel = bool(
            self.declare_parameter("use_manual_tool_center_pixel", False).value
        )
        self.manual_tool_center_pixel = list(
            self.declare_parameter("manual_tool_center_pixel", [-1, -1]).value
        )
        self.tf_timeout_sec = float(
            self.declare_parameter("tf_timeout_sec", 0.03).value
        )
        self.max_inference_hz = float(
            self.declare_parameter("max_inference_hz", 4.0).value
        )
        self.min_saturation = int(
            self.declare_parameter("min_saturation", 105).value
        )
        self.min_value = int(self.declare_parameter("min_value", 55).value)
        self.min_area_pixels = int(
            self.declare_parameter("min_area_pixels", 120).value
        )
        self.max_area_fraction = float(
            self.declare_parameter("max_area_fraction", 0.35).value
        )
        self.max_candidates = int(self.declare_parameter("max_candidates", 6).value)
        self.crop_expand_ratio = float(
            self.declare_parameter("crop_expand_ratio", 0.10).value
        )
        self.ignore_rois = parse_rois(
            self.declare_parameter("ignore_rois", [0, 0, 0, 0]).value
        )

        self.open_vocab_enabled = bool(
            self.declare_parameter("open_vocab_enabled", False).value
        )
        self.open_vocab_model = str(
            self.declare_parameter("open_vocab_model", "yolov8s-world.pt").value
        )
        self.open_vocab_prompts = [
            str(value)
            for value in self.declare_parameter(
                "open_vocab_prompts",
                ["colored cube", "small block", "wooden block", "artificial object"],
            ).value
        ]
        self.open_vocab_confidence = float(
            self.declare_parameter("open_vocab_confidence", 0.20).value
        )
        self.open_vocab_min_iou = float(
            self.declare_parameter("open_vocab_min_iou", 0.10).value
        )
        self.open_vocab_imgsz = int(
            self.declare_parameter("open_vocab_imgsz", 320).value
        )
        self.open_vocab_max_candidates = int(
            self.declare_parameter("open_vocab_max_candidates", 1).value
        )
        # The production wide-search path is deliberately two-stage: HSV
        # supplies fast, colour/shape-localised proposals, then the locally
        # trained detector confirms that the same image region is a permitted
        # target class.  Keep the old open-vocabulary model available only for
        # experiments; it is never the default confirmation model.
        self.yolo_confirm_enabled = bool(
            self.declare_parameter("yolo_confirm_enabled", True).value
        )
        self.yolo_confirm_model = str(
            self.declare_parameter("yolo_confirm_model", "weights/best.pt").value
        )
        self.yolo_confirm_device = str(
            self.declare_parameter("yolo_confirm_device", "0").value
        )
        self.yolo_confirm_confidence = float(
            self.declare_parameter("yolo_confirm_confidence", 0.45).value
        )
        self.yolo_confirm_min_iou = float(
            self.declare_parameter("yolo_confirm_min_iou", 0.10).value
        )
        self.yolo_confirm_imgsz = int(
            self.declare_parameter("yolo_confirm_imgsz", 640).value
        )
        self.yolo_confirm_classes = {
            str(value)
            for value in self.declare_parameter(
                "yolo_confirm_classes", ["wood_block", "artificial_object"]
            ).value
        }

        self.sam_enabled = bool(self.declare_parameter("sam_enabled", False).value)
        self.sam_model = str(self.declare_parameter("sam_model", "sam2_t.pt").value)
        self.sam_async_enabled = bool(
            self.declare_parameter("sam_async_enabled", True).value
        )
        self.grasp_mode = str(
            self.declare_parameter("initial_grasp_mode", "wide_search").value
        )
        self.lock_reacquire_max_center_distance_frac = float(
            self.declare_parameter("lock_reacquire_max_center_distance_frac", 1.5).value
        )
        self.lock_reacquire_min_iou = float(
            self.declare_parameter("lock_reacquire_min_iou", 0.02).value
        )
        self.lock_hsv_hue_tolerance = float(
            self.declare_parameter("lock_hsv_hue_tolerance", 10.0).value
        )
        self.lock_hsv_saturation_tolerance = float(
            self.declare_parameter("lock_hsv_saturation_tolerance", 55.0).value
        )
        self.lock_hsv_value_tolerance = float(
            self.declare_parameter("lock_hsv_value_tolerance", 85.0).value
        )
        self.precise_sam_period_sec = float(
            self.declare_parameter("precise_sam_period_sec", 1.0).value
        )
        self.sam_min_seed_coverage = float(
            self.declare_parameter("sam_min_seed_coverage", 0.45).value
        )
        self.sam_min_seed_iou = float(
            self.declare_parameter("sam_min_seed_iou", 0.10).value
        )
        self.sam_max_area_ratio_to_seed = float(
            self.declare_parameter("sam_max_area_ratio_to_seed", 8.0).value
        )
        self.sam_max_area_fraction = float(
            self.declare_parameter("sam_max_area_fraction", 0.20).value
        )
        self.sam_apply_min_iou = float(
            self.declare_parameter("sam_apply_min_iou", 0.10).value
        )
        self.sam_apply_max_center_distance_frac = float(
            self.declare_parameter("sam_apply_max_center_distance_frac", 0.75).value
        )

        sensor_qos = QoSProfile(depth=3, reliability=ReliabilityPolicy.BEST_EFFORT)
        input_qos = sensor_qos
        self.bridge = CvBridge()
        self.camera_info = None
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.mask_publisher = self.create_publisher(
            Image, "/perception/target_mask", sensor_qos
        )
        self.debug_publisher = self.create_publisher(
            Image, "/perception/debug/fusion_overlay", sensor_qos
        )
        self.debug_compressed_publisher = self.create_publisher(
            CompressedImage, "/perception/debug/fusion_overlay/compressed", sensor_qos
        )
        self.detection_publisher = self.create_publisher(
            Detection2D, "/perception/target_detection", 10
        )
        message_type = CompressedImage if self.image_is_compressed else Image
        self.create_subscription(
            message_type, self.image_topic, self.on_image, input_qos
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.on_camera_info, sensor_qos
        )
        mode_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, "/perception/grasp_mode", self.on_grasp_mode, mode_qos
        )
        self.create_subscription(
            PointStamped, self.tf_target_pixel_topic, self.on_tf_target_pixel, 10
        )

        self.sequence = 0
        self.tf_target_pixel = None
        self.last_inference = 0.0
        self.debug_frame_count = 0
        self.last_precise_sam = -1e9
        self.sam_executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix="sam")
        self.sam_future = None
        self.last_sam_result = None
        self.last_selected_bbox = None
        self.last_selected_class_id = ""
        self.lock_hsv_profile = None
        self.yolo_confirm = self._load_yolo_confirm()
        self.open_vocab = self._load_open_vocab()
        self.sam = self._load_sam()
        self.get_logger().info(
            "fusion detector ready: image=%s, best_pt=%s, open_vocab=%s, sam=%s, mode=%s"
            % (
                self.image_topic,
                bool(self.yolo_confirm),
                bool(self.open_vocab),
                bool(self.sam),
                self.grasp_mode,
            )
        )

    def on_camera_info(self, message):
        self.camera_info = message

    def on_tf_target_pixel(self, message):
        values = (float(message.point.x), float(message.point.y), float(message.point.z))
        if not all(np.isfinite(value) for value in values) or values[2] <= 0.0:
            self.tf_target_pixel = None
            return
        self.tf_target_pixel = values

    def on_grasp_mode(self, message):
        mode = (message.data or "").strip() or "idle"
        if mode == self.grasp_mode:
            return
        self.grasp_mode = mode
        if mode in ("idle", "wide_search"):
            self.last_selected_bbox = None
            self.last_selected_class_id = ""
            self.lock_hsv_profile = None
        self.get_logger().info("grasp perception mode -> %s" % mode)

    def _load_open_vocab(self):
        if not self.open_vocab_enabled:
            return None
        try:
            from ultralytics import YOLOWorld

            model = YOLOWorld(self.open_vocab_model)
            model.set_classes(self.open_vocab_prompts)
            return model
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                f"open-vocabulary model unavailable, using HSV only: {exc}"
            )
            return None

    def _load_yolo_confirm(self):
        if not self.yolo_confirm_enabled:
            return None
        if not self.yolo_confirm_model:
            self.get_logger().warning(
                "self-trained YOLO confirmation disabled: yolo_confirm_model is empty"
            )
            return None
        try:
            from ultralytics import YOLO

            return YOLO(self.yolo_confirm_model)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(
                "self-trained YOLO confirmation unavailable; wide search will "
                f"remain HSV-only until restored: {exc}"
            )
            return None

    def _load_sam(self):
        if not self.sam_enabled:
            return None
        try:
            from ultralytics import SAM

            return SAM(self.sam_model)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"SAM unavailable, using HSV mask: {exc}")
            return None

    def _hsv_proposals(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(
            hsv,
            np.array([0, self.min_saturation, self.min_value], dtype=np.uint8),
            np.array([179, 255, 255], dtype=np.uint8),
        )
        apply_ignore_rois(mask, self.ignore_rois)
        kernel = np.ones((5, 5), dtype=np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        image_area = image.shape[0] * image.shape[1]
        proposals = []
        for component in range(1, count):
            area = int(stats[component, cv2.CC_STAT_AREA])
            if area < self.min_area_pixels:
                continue
            if area > self.max_area_fraction * image_area:
                continue
            component_mask = (labels == component).astype(np.uint8) * 255
            x = int(stats[component, cv2.CC_STAT_LEFT])
            y = int(stats[component, cv2.CC_STAT_TOP])
            width = int(stats[component, cv2.CC_STAT_WIDTH])
            height = int(stats[component, cv2.CC_STAT_HEIGHT])
            saturation = float(np.median(hsv[:, :, 1][labels == component])) / 255.0
            area_score = min(1.0, area / max(float(self.min_area_pixels * 4), 1.0))
            score = 0.55 * saturation + 0.45 * area_score
            proposals.append(
                HsvProposal(component_mask, (x, y, width, height), score, area)
            )
        proposals.sort(key=lambda proposal: proposal.score, reverse=True)
        return proposals[: self.max_candidates]

    @staticmethod
    def _circular_hue_distance(a, b):
        raw = abs(float(a) - float(b))
        return min(raw, 180.0 - raw)

    def _candidate_hsv_profile(self, image, proposal):
        if proposal is None:
            return None
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        active = proposal.mask > 0
        if np.count_nonzero(active) < self.min_area_pixels:
            return None
        return HsvLockProfile(
            np.median(hsv[:, :, 0][active]),
            np.median(hsv[:, :, 1][active]),
            np.median(hsv[:, :, 2][active]),
        )

    def _remember_locked_hsv(self, image, proposal):
        profile = self._candidate_hsv_profile(image, proposal)
        if profile is None:
            return
        self.lock_hsv_profile = profile
        try:
            self.get_logger().info(
                "locked target HSV profile: h=%.1f s=%.1f v=%.1f"
                % (profile.hue, profile.saturation, profile.value)
            )
        except Exception:  # noqa: BLE001
            pass

    def _matches_locked_hsv(self, image, proposal):
        if self.lock_hsv_profile is None:
            return True
        profile = self._candidate_hsv_profile(image, proposal)
        if profile is None:
            return False
        return (
            self._circular_hue_distance(profile.hue, self.lock_hsv_profile.hue)
            <= self.lock_hsv_hue_tolerance
            and abs(profile.saturation - self.lock_hsv_profile.saturation)
            <= self.lock_hsv_saturation_tolerance
            and abs(profile.value - self.lock_hsv_profile.value)
            <= self.lock_hsv_value_tolerance
        )

    def _filter_locked_hsv(self, image, proposals):
        if self.lock_hsv_profile is None:
            return proposals
        return [
            proposal
            for proposal in proposals
            if self._matches_locked_hsv(image, proposal)
        ]

    def _confirm_with_open_vocab(self, image, proposals):
        if self.open_vocab is None or not proposals:
            return proposals
        confirmed = []
        for proposal in proposals[: max(1, self.open_vocab_max_candidates)]:
            x, y, width, height = expanded_bbox(
                proposal.bbox, image.shape, self.crop_expand_ratio
            )
            crop = image[y : y + height, x : x + width]
            if crop.size == 0:
                continue
            try:
                results = self.open_vocab.predict(
                    crop,
                    conf=self.open_vocab_confidence,
                    imgsz=self.open_vocab_imgsz,
                    device="cpu",
                    verbose=False,
                )
            except Exception as exc:  # noqa: BLE001
                self.get_logger().warning(f"open-vocabulary inference failed: {exc}")
                return proposals
            result = results[0] if results else None
            if result is None or result.boxes is None:
                continue
            best = None
            for box in result.boxes:
                cls = int(box.cls.item())
                label = (
                    self.open_vocab_prompts[cls]
                    if cls < len(self.open_vocab_prompts)
                    else str(cls)
                )
                conf = float(box.conf.item())
                bx1, by1, bx2, by2 = [float(value) for value in box.xyxy[0].tolist()]
                global_box = (
                    int(round(x + bx1)),
                    int(round(y + by1)),
                    int(round(bx2 - bx1)),
                    int(round(by2 - by1)),
                )
                overlap = self._iou(proposal.bbox, global_box)
                if overlap < self.open_vocab_min_iou:
                    continue
                if best is None or conf > best[0]:
                    best = (conf, label)
            if best is not None:
                proposal.score = 0.5 * proposal.score + 0.5 * best[0]
                proposal.class_id = best[1]
                proposal.stage = "open_vocab"
                confirmed.append(proposal)
        confirmed.sort(key=lambda proposal: proposal.score, reverse=True)
        return confirmed if confirmed else proposals

    def _confirm_with_yolo(self, image, proposals):
        """Confirm HSV proposals only when ``best.pt`` overlaps the proposal.

        The detector runs once on the whole RGB frame.  This preserves YOLO's
        normal receptive field while ensuring that its label belongs to the
        HSV candidate currently considered by the grasp state machine.
        """
        if self.yolo_confirm is None or not proposals:
            return []
        try:
            results = self.yolo_confirm.predict(
                source=image,
                conf=self.yolo_confirm_confidence,
                imgsz=self.yolo_confirm_imgsz,
                device=self.yolo_confirm_device,
                verbose=False,
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"self-trained YOLO inference failed: {exc}")
            return []
        result = results[0] if results else None
        if result is None or result.boxes is None:
            return []

        names = result.names
        yolo_boxes = []
        for box in result.boxes:
            class_index = int(box.cls.item())
            label = str(names[class_index])
            if self.yolo_confirm_classes and label not in self.yolo_confirm_classes:
                continue
            x1, y1, x2, y2 = [float(value) for value in box.xyxy[0].tolist()]
            yolo_boxes.append(
                (
                    float(box.conf.item()),
                    label,
                    (
                        int(round(x1)),
                        int(round(y1)),
                        max(1, int(round(x2 - x1))),
                        max(1, int(round(y2 - y1))),
                    ),
                )
            )

        confirmed = []
        for proposal in proposals:
            best = None
            for confidence, label, yolo_bbox in yolo_boxes:
                overlap = self._iou(proposal.bbox, yolo_bbox)
                if overlap < self.yolo_confirm_min_iou:
                    continue
                score = confidence * (1.0 + overlap)
                if best is None or score > best[0]:
                    best = (score, confidence, label)
            if best is None:
                continue
            _, confidence, label = best
            proposal.score = 0.5 * proposal.score + 0.5 * confidence
            proposal.class_id = label
            proposal.stage = "best_pt_confirmed"
            confirmed.append(proposal)
        confirmed.sort(key=lambda proposal: proposal.score, reverse=True)
        return confirmed

    def _run_sam_refinement(self, image, source_bbox, source_mask):
        """Run SAM away from the RGB callback so HSV output stays real-time."""
        if self.sam is None:
            return None
        x, y, width, height = expanded_bbox(
            source_bbox, image.shape, self.crop_expand_ratio
        )
        box = [x, y, x + width, y + height]
        try:
            results = self.sam.predict(image, bboxes=[box], verbose=False, device="cpu")
        except Exception as exc:  # noqa: BLE001
            return {"error": str(exc), "source_bbox": source_bbox}
        result = results[0] if results else None
        if result is None or result.masks is None or result.masks.data is None:
            return None
        masks = result.masks.data
        if len(masks) == 0:
            return None

        prompt_bbox = (x, y, width, height)
        selected = self._select_sam_mask(masks, image.shape, prompt_bbox, source_mask)
        if selected is None:
            return None
        mask, bbox = selected
        return {"mask": mask, "bbox": bbox, "source_bbox": source_bbox}

    def _select_sam_mask(self, raw_masks, image_shape, prompt_bbox, source_mask):
        """Select only a compact SAM component supported by the HSV seed.

        SAM can legitimately return a background region for a box prompt. Such
        a region must never replace the locked HSV target, even if its bounding
        box happens to be spatially close to the proposal.
        """
        seed = source_mask > 0
        seed_area = int(np.count_nonzero(seed))
        if seed_area < self.min_area_pixels:
            return None

        image_area = image_shape[0] * image_shape[1]
        max_area = min(
            int(self.sam_max_area_fraction * image_area),
            int(self.sam_max_area_ratio_to_seed * seed_area),
        )
        best = None
        best_score = -1.0
        for raw_mask in raw_masks:
            # SAM may expose boolean masks. OpenCV resize accepts numeric
            # arrays only, so normalize the representation before scaling.
            mask = raw_mask.detach().cpu().numpy().astype(np.float32)
            mask = cv2.resize(
                mask, (image_shape[1], image_shape[0]), interpolation=cv2.INTER_NEAREST
            )
            mask = (mask >= 0.5).astype(np.uint8) * 255
            mask = clip_mask_to_bbox(mask, prompt_bbox)
            apply_ignore_rois(mask, self.ignore_rois)
            component_count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
            for component in range(1, component_count):
                area = int(stats[component, cv2.CC_STAT_AREA])
                if area < self.min_area_pixels or area > max_area:
                    continue
                candidate = (labels == component)
                intersection = int(np.count_nonzero(candidate & seed))
                coverage = intersection / float(seed_area)
                union = area + seed_area - intersection
                iou = intersection / float(union) if union else 0.0
                if coverage < self.sam_min_seed_coverage or iou < self.sam_min_seed_iou:
                    continue
                score = iou + 0.25 * coverage - 0.01 * (area / float(seed_area))
                if score <= best_score:
                    continue
                candidate_mask = candidate.astype(np.uint8) * 255
                bbox = bbox_from_mask(candidate_mask)
                if bbox is None:
                    continue
                best = (candidate_mask, bbox)
                best_score = score
        return best

    def _take_completed_sam_result(self):
        if self.sam_future is None or not self.sam_future.done():
            return
        future = self.sam_future
        self.sam_future = None
        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"SAM worker failed: {exc}")
            return
        if result is not None and "error" in result:
            self.get_logger().warning(f"SAM inference failed: {result['error']}")
            return
        if result is not None:
            self.last_sam_result = result

    def _apply_latest_sam_result(self, proposal):
        result = self.last_sam_result
        if result is None:
            return proposal
        source_bbox = result["source_bbox"]
        overlap = self._iou(source_bbox, proposal.bbox)
        distance_frac = float(
            np.linalg.norm(self._bbox_center(proposal.bbox) - self._bbox_center(source_bbox))
        ) / self._bbox_diagonal(source_bbox)
        if (
            overlap < self.sam_apply_min_iou
            and distance_frac > self.sam_apply_max_center_distance_frac
        ):
            return proposal
        proposal.bbox = result["bbox"]
        proposal.mask = result["mask"].copy()
        if "+sam" not in proposal.class_id:
            proposal.class_id = f"{proposal.class_id}+sam"
        proposal.stage = "sam"
        return proposal

    def _schedule_sam_refinement(self, image, proposal, now):
        self._take_completed_sam_result()
        if self.sam is None or proposal is None:
            return
        if self.sam_future is not None or now - self.last_precise_sam < self.precise_sam_period_sec:
            return
        self.last_precise_sam = now
        self.sam_future = self.sam_executor.submit(
            self._run_sam_refinement,
            image.copy(),
            tuple(proposal.bbox),
            proposal.mask.copy(),
        )

    @staticmethod
    def _bbox_center(bbox):
        x, y, width, height = bbox
        return np.array([x + width * 0.5, y + height * 0.5], dtype=np.float64)

    @staticmethod
    def _bbox_diagonal(bbox):
        return max(1.0, float(np.hypot(bbox[2], bbox[3])))

    def _select_locked_candidate(self, proposals):
        if not proposals:
            return None
        if self.last_selected_bbox is None:
            return proposals[0]

        reference_center = self._bbox_center(self.last_selected_bbox)
        reference_diagonal = self._bbox_diagonal(self.last_selected_bbox)
        best = None
        best_score = -1e9
        for proposal in proposals:
            center = self._bbox_center(proposal.bbox)
            distance_frac = float(np.linalg.norm(center - reference_center)) / reference_diagonal
            overlap = self._iou(self.last_selected_bbox, proposal.bbox)
            gated = (
                overlap >= self.lock_reacquire_min_iou
                or distance_frac <= self.lock_reacquire_max_center_distance_frac
            )
            score = proposal.score + 1.6 * overlap - 0.35 * distance_frac
            if not gated:
                score -= 2.0
            if score > best_score:
                best_score = score
                best = proposal
        return best

    def _apply_mode_pipeline(self, image, proposals, now):
        mode = self.grasp_mode
        if mode in ("idle", ""):
            return None
        if mode == "wide_search":
            # No YOLO match intentionally leaves the proposal labelled
            # ``hsv_candidate``.  The state machine will not lock it, but the
            # operator may explicitly use skip_yolo_confirm for commissioning.
            confirmed = self._confirm_with_yolo(image, proposals)
            if not confirmed and self.yolo_confirm is None:
                confirmed = self._confirm_with_open_vocab(image, proposals)
            selected = confirmed[0] if confirmed else (proposals[0] if proposals else None)
            if selected is not None and selected.stage in (
                "best_pt_confirmed",
                "open_vocab",
            ):
                self._remember_locked_hsv(image, selected)
            return selected, confirmed

        proposals = self._filter_locked_hsv(image, proposals)
        selected = self._select_locked_candidate(proposals)
        confirmed = proposals
        if selected is None:
            return None, confirmed

        if self.last_selected_class_id:
            selected.class_id = self.last_selected_class_id
        elif selected.class_id.startswith("hsv_candidate"):
            selected.class_id = "locked_target"

        if mode == "locked_hsv":
            selected.stage = "locked_hsv"
            return selected, confirmed

        if mode == "precise_hsv_sam":
            if self.sam_async_enabled:
                self._schedule_sam_refinement(image, selected, now)
                selected = self._apply_latest_sam_result(selected)
            elif self.sam is not None and now - self.last_precise_sam >= self.precise_sam_period_sec:
                result = self._run_sam_refinement(
                    image, tuple(selected.bbox), selected.mask
                )
                self.last_precise_sam = now
                if result is not None and "mask" in result:
                    self.last_sam_result = result
                    selected = self._apply_latest_sam_result(selected)
            selected.stage = "precise_hsv_sam"
            return selected, confirmed

        return selected, confirmed

    @staticmethod
    def _iou(a, b):
        ax, ay, aw, ah = a
        bx, by, bw, bh = b
        ax2, ay2 = ax + aw, ay + ah
        bx2, by2 = bx + bw, by + bh
        ix1, iy1 = max(ax, bx), max(ay, by)
        ix2, iy2 = min(ax2, bx2), min(ay2, by2)
        if ix2 <= ix1 or iy2 <= iy1:
            return 0.0
        intersection = float((ix2 - ix1) * (iy2 - iy1))
        union = float(aw * ah + bw * bh) - intersection
        return intersection / union if union > 0.0 else 0.0

    def on_image(self, message):
        now = time.monotonic()
        if self.max_inference_hz > 0.0:
            interval = 1.0 / self.max_inference_hz
            if now - self.last_inference < interval:
                return
        self.last_inference = now
        self.debug_frame_count += 1
        try:
            image = self._message_to_bgr(message)
            proposals = self._hsv_proposals(image)
            if self.debug_frame_count <= 5:
                self.get_logger().info(
                    f"fusion frame {self.debug_frame_count}: hsv proposals={len(proposals)}"
                )
            result = self._apply_mode_pipeline(image, proposals, now)
            if result is None:
                selected, confirmed = None, proposals
            else:
                selected, confirmed = result
            if self.debug_frame_count <= 5:
                self.get_logger().info(
                    "fusion frame %d: mode=%s selected_stage=%s"
                    % (
                        self.debug_frame_count,
                        self.grasp_mode,
                        confirmed[0].stage if confirmed else "none",
                    )
                )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f"fusion inference failed: {exc}")
            return
        if selected is None:
            if self.grasp_mode == "wide_search":
                self.last_selected_bbox = None
                self.last_selected_class_id = ""
            self._publish_debug_overlay(image, message.header, proposals, None)
            return
        self.last_selected_bbox = selected.bbox
        self.last_selected_class_id = selected.class_id
        self._publish_detection(selected, message.header)
        self._publish_mask(selected.mask, message.header)
        overlay = self._make_debug_overlay(image, proposals, selected)
        self._publish_debug_overlay(overlay, message.header, proposals, selected)

    def _publish_detection(self, proposal, header):
        x, y, width, height = proposal.bbox
        self.sequence += 1
        detection = Detection2D()
        detection.header = header
        detection.id = f"fusion-target-{header.stamp.sec}-{header.stamp.nanosec}-{self.sequence}"
        detection.bbox.center.position.x = float(x + width * 0.5)
        detection.bbox.center.position.y = float(y + height * 0.5)
        detection.bbox.size_x = float(width)
        detection.bbox.size_y = float(height)
        hypothesis = ObjectHypothesisWithPose()
        hypothesis.hypothesis.class_id = proposal.class_id
        hypothesis.hypothesis.score = float(proposal.score)
        hypothesis.pose.pose.orientation.w = 1.0
        detection.results.append(hypothesis)
        self.detection_publisher.publish(detection)

    def _publish_mask(self, mask, header):
        mask_message = self.bridge.cv2_to_imgmsg(mask, encoding="mono8")
        mask_message.header = header
        self.mask_publisher.publish(mask_message)

    def _make_debug_overlay(self, image, proposals, selected):
        overlay = image.copy()
        for proposal in proposals:
            x, y, width, height = proposal.bbox
            cv2.rectangle(overlay, (x, y), (x + width, y + height), (255, 180, 0), 3)
        work_center = self._project_work_center(image)
        if work_center is not None:
            u, v, depth = work_center
            cv2.drawMarker(
                overlay,
                (u, v),
                (255, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=28,
                thickness=3,
                line_type=cv2.LINE_AA,
            )
            cv2.circle(overlay, (u, v), 16, (255, 0, 255), 2, cv2.LINE_AA)
            cv2.putText(
                overlay,
                (
                    f"tool/work center z={depth:.2f}m"
                    if np.isfinite(depth)
                    else "tool/work center (manual pixel)"
                ),
                (max(0, u - 90), min(image.shape[0] - 8, v + 34)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 255),
                2,
                cv2.LINE_AA,
            )
        self._draw_tf_target_projection(overlay)
        if selected is not None:
            x, y, width, height = selected.bbox
            overlay[selected.mask > 0] = (
                0.55 * overlay[selected.mask > 0] + 0.45 * np.array([0, 255, 0])
            ).astype(np.uint8)
            cv2.rectangle(overlay, (x, y), (x + width, y + height), (0, 255, 0), 4)
            if work_center is not None:
                target_center = (int(round(x + width * 0.5)), int(round(y + height * 0.5)))
                cv2.arrowedLine(
                    overlay,
                    target_center,
                    (work_center[0], work_center[1]),
                    (255, 0, 255),
                    2,
                    cv2.LINE_AA,
                    tipLength=0.18,
                )
            text_color = (
                (0, 0, 255)
                if selected.stage == "open_vocab"
                else (0, 220, 255)
                if selected.stage == "locked_hsv"
                else (0, 255, 0)
            )
            cv2.putText(
                overlay,
                f"{selected.class_id} {selected.score:.2f} [{self.grasp_mode}/{selected.stage}]",
                (max(0, x), max(18, y - 6)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                text_color,
                1,
                cv2.LINE_AA,
            )
        return overlay

    def _draw_tf_target_projection(self, overlay):
        """Draw the state machine's base->camera TF target projection."""
        if self.grasp_mode not in ("locked_hsv", "precise_hsv_sam"):
            return
        if self.tf_target_pixel is None:
            return
        u, v, depth = self.tf_target_pixel
        height, width = overlay.shape[:2]
        u, v = int(round(u)), int(round(v))
        if not (0 <= u < width and 0 <= v < height):
            return
        # OpenCV BGR yellow, deliberately distinct from magenta tool center.
        cv2.circle(overlay, (u, v), 9, (0, 255, 255), -1, cv2.LINE_AA)
        cv2.circle(overlay, (u, v), 14, (0, 170, 170), 2, cv2.LINE_AA)
        cv2.putText(
            overlay,
            f"TF target z={depth:.2f}m",
            (max(0, u + 16), max(18, v - 12)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )

    def _project_work_center(self, image):
        if not self.draw_work_center or self.camera_info is None:
            return None
        height, width = image.shape[:2]
        if self.use_manual_tool_center_pixel and len(self.manual_tool_center_pixel) >= 2:
            u = int(round(float(self.manual_tool_center_pixel[0])))
            v = int(round(float(self.manual_tool_center_pixel[1])))
            if 0 <= u < width and 0 <= v < height:
                return u, v, float("nan")
        camera_frame = self.camera_info.header.frame_id or "camera_color_optical_frame"
        try:
            transform = self.tf_buffer.lookup_transform(
                camera_frame,
                self.work_center_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_sec),
            )
        except TransformException:
            return None
        point = transform.transform.translation
        z = float(point.z)
        if z <= 0.03:
            return None
        fx = float(self.camera_info.k[0])
        fy = float(self.camera_info.k[4])
        cx = float(self.camera_info.k[2])
        cy = float(self.camera_info.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return None
        u = int(round(fx * float(point.x) / z + cx))
        v = int(round(fy * float(point.y) / z + cy))
        if u < 0 or u >= width or v < 0 or v >= height:
            return None
        return u, v, z

    def _publish_debug_overlay(self, image, header, proposals=None, selected=None):
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
    node = TargetFusionDetector()
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
