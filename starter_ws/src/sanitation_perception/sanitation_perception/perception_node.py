from __future__ import annotations

import hashlib
import json
from pathlib import Path
import time
from collections import deque, Counter


def main() -> None:
    from ament_index_python.packages import get_package_share_directory
    import cv2
    from cv_bridge import CvBridge
    from geometry_msgs.msg import Point32
    import numpy as np
    import onnxruntime as ort
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sanitation_perception.projection import ProjectionError, project_pixel_to_map, robust_depth
    from sanitation_perception.registry import GarbageRegistry
    from sanitation_perception.tracking import TargetTracker
    from sanitation_perception.competition_policy import thresholds, validate_context, in_ground_roi, publishable, stamp
    from sanitation_perception_interfaces.msg import GarbageTarget, GarbageTargetArray
    from sensor_msgs.msg import CameraInfo, Image
    from std_msgs.msg import String
    from tf2_ros import Buffer, TransformException, TransformListener
    from vision_msgs.msg import (
        Detection2D,
        Detection2DArray,
        Detection3D,
        Detection3DArray,
        ObjectHypothesisWithPose,
    )

    from sanitation_perception.preprocessing import (
        CLASS_ORDER,
        MODEL_HEIGHT,
        MODEL_WIDTH,
        preprocess_rgb,
        resize_labels,
    )

    class_order = CLASS_ORDER
    model_height, model_width = MODEL_HEIGHT, MODEL_WIDTH

    class PerceptionNode(Node):
        def __init__(self):
            super().__init__("garbage_perception")
            self.declare_parameter("backend", "onnxruntime")
            self.declare_parameter("model_path", "")
            self.declare_parameter("model_id", "stage5a_synthetic_color_model_v1")
            self.declare_parameter("model_scope", "synthetic_color_domain_only")
            self.declare_parameter("learned_weights", False)
            self.declare_parameter("publish_tentative_targets", False)
            self.declare_parameter("rgbd_max_skew_s", 0.03)
            self.declare_parameter("ground_roi_enabled", True)
            self.declare_parameter("ground_roi_bounds", [0.4, 3.0, -0.8, 0.8, -0.08, 0.30])
            self.declare_parameter("class_thresholds_json", json.dumps({c: 0.8 for c in class_order[1:]}))
            self.declare_parameter("minimum_component_pixels", 24)
            self.thresholds = thresholds(str(self.get_parameter("class_thresholds_json").value), class_order[1:])
            self.roi_bounds = list(self.get_parameter("ground_roi_bounds").value)
            in_ground_roi([0., 0., 0.], self.roi_bounds)  # validate bounds before subscribing
            self.minimum_pixels = int(self.get_parameter("minimum_component_pixels").value)
            if self.minimum_pixels < 1:
                raise ValueError("minimum_component_pixels must be positive")
            self.rejection_counts = Counter()
            self.depth_buffer = deque(maxlen=40)
            self.pending_image = None
            self.last_source_stamp = -1.0
            self.track_state_counts = {}
            backend = str(self.get_parameter("backend").value)
            model_path = Path(str(self.get_parameter("model_path").value))
            self.model_id = str(self.get_parameter("model_id").value)
            self.model_scope = str(self.get_parameter("model_scope").value)
            self.learned_weights = bool(self.get_parameter("learned_weights").value)
            if backend != "onnxruntime":
                raise RuntimeError(f"live Stage5A simulation requires onnxruntime, got {backend}")
            if not model_path.is_file():
                raise RuntimeError(f"ONNX model not found: {model_path}")
            self.model_sha256 = hashlib.sha256(model_path.read_bytes()).hexdigest()
            share = Path(get_package_share_directory("sanitation_perception"))
            self.registry = GarbageRegistry.load(share / "config" / "garbage_registry.yaml")
            self.entries_by_class = {entry.class_id: entry for entry in self.registry.entries.values()}
            session_options = ort.SessionOptions()
            session_options.intra_op_num_threads = 1
            session_options.inter_op_num_threads = 1
            session_options.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL
            self.session = ort.InferenceSession(
                str(model_path), sess_options=session_options, providers=["CPUExecutionProvider"]
            )
            self.bridge = CvBridge()
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.tracker = TargetTracker(confirmation_observations=3)
            self.camera_info = None
            self.depth = None
            self.frame_count = 0
            self.last_latency_ms = None
            self.last_detection_count = 0
            self.last_raw_detection_count = 0
            self.last_map_target_count = 0
            self.last_projection_error = None
            self.last_inference_monotonic = 0.0
            self.segmentation_publisher = self.create_publisher(Image, "/perception/garbage/segmentation", 10)
            self.detection2d_publisher = self.create_publisher(Detection2DArray, "/perception/garbage/detections_2d", 10)
            self.raw_detection2d_publisher = self.create_publisher(Detection2DArray, "/perception/garbage/raw_detections_2d", 10)
            self.detection3d_publisher = self.create_publisher(Detection3DArray, "/perception/garbage/detections_3d", 10)
            self.target_publisher = self.create_publisher(GarbageTargetArray, "/perception/garbage/targets", 10)
            self.diagnostics_publisher = self.create_publisher(String, "/perception/garbage/diagnostics", 10)
            self.create_subscription(CameraInfo, "/camera/color/camera_info", self.on_camera_info, qos_profile_sensor_data)
            self.create_subscription(Image, "/camera/depth/image_rect_raw", self.on_depth, qos_profile_sensor_data)
            self.create_subscription(Image, "/camera/color/image_raw", self.on_image, qos_profile_sensor_data)
            self.create_timer(0.5, self.publish_diagnostics)

        def on_camera_info(self, message):
            self.camera_info = message
            if self.pending_image is not None:
                self.on_image(self.pending_image)

        def on_depth(self, message):
            self.depth = message
            self.depth_buffer.append(message)
            if self.pending_image is not None:
                self.on_image(self.pending_image)

        @staticmethod
        def transform_matrix(transform):
            translation = transform.transform.translation
            quaternion = transform.transform.rotation
            x, y, z, w = quaternion.x, quaternion.y, quaternion.z, quaternion.w
            rotation = np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
            ], dtype=np.float64)
            matrix = np.eye(4, dtype=np.float64)
            matrix[:3, :3] = rotation
            matrix[:3, 3] = (translation.x, translation.y, translation.z)
            return matrix

        def extract_detections(self, labels, logits, image_message):
            if self.camera_info is None or self.depth is None:
                raise ProjectionError("camera_info or depth unavailable")
            if self.camera_info.header.frame_id != image_message.header.frame_id:
                raise ProjectionError("camera_info frame does not match RGB frame")
            depth = self.bridge.imgmsg_to_cv2(self.depth, desired_encoding="passthrough")
            if self.depth.encoding == "16UC1":
                depth = depth.astype(np.float32) * 0.001
            if depth.shape[:2] != labels.shape:
                raise ProjectionError("registered depth shape mismatch")
            try:
                image_time = rclpy.time.Time.from_msg(image_message.header.stamp)
                transform = self.tf_buffer.lookup_transform(
                    "map", image_message.header.frame_id, image_time
                )
                base_transform = self.tf_buffer.lookup_transform(
                    "base_footprint", image_message.header.frame_id, image_time
                )
            except TransformException as exc:
                raise ProjectionError(f"map transform unavailable: {exc}") from exc
            transform_map_camera = self.transform_matrix(transform)
            transform_base_camera = self.transform_matrix(base_transform)
            camera = {
                "fx": self.camera_info.k[0], "fy": self.camera_info.k[4],
                "cx": self.camera_info.k[2], "cy": self.camera_info.k[5],
                "pixel_sigma": 0.5, "depth_sigma_m": 0.02,
            }
            shifted = logits - logits.max(axis=0, keepdims=True)
            probability = np.exp(shifted) / np.exp(shifted).sum(axis=0, keepdims=True)
            detections = []
            for class_index, class_id in enumerate(class_order[1:], 1):
                mask = (labels == class_index).astype(np.uint8)
                component_count, components, stats, centroids = cv2.connectedComponentsWithStats(mask, 8)
                if component_count <= 1:
                    continue
                for component in range(1, component_count):
                    area = int(stats[component, cv2.CC_STAT_AREA])
                    if area < self.minimum_pixels:
                        self.rejection_counts["component_pixels"] += 1
                        continue
                    x = int(stats[component, cv2.CC_STAT_LEFT])
                    y = int(stats[component, cv2.CC_STAT_TOP])
                    width = int(stats[component, cv2.CC_STAT_WIDTH])
                    height = int(stats[component, cv2.CC_STAT_HEIGHT])
                    component_mask = components == component
                    model_mask = cv2.resize(component_mask.astype(np.uint8), (model_width, model_height), interpolation=cv2.INTER_NEAREST).astype(bool)
                    confidence = float(probability[class_index][model_mask].mean()) if model_mask.any() else 0.0
                    if not np.isfinite(confidence) or confidence < self.thresholds[class_id]:
                        self.rejection_counts["confidence:" + class_id] += 1
                        continue
                    try:
                        depth_m = robust_depth(depth[component_mask].reshape(-1))
                    except ProjectionError:
                        self.rejection_counts["depth"] += 1
                        continue
                    u, v = (float(value) for value in centroids[component])
                    xyz_base, _ = project_pixel_to_map(u, v, depth_m, camera, transform_base_camera)
                    if self.get_parameter("ground_roi_enabled").value and not in_ground_roi(xyz_base, self.roi_bounds):
                        self.rejection_counts["ground_roi"] += 1
                        continue
                    xyz, covariance = project_pixel_to_map(u, v, depth_m, camera, transform_map_camera)
                    entry = self.entries_by_class[class_id]
                    detections.append({
                    "class_id": class_id,
                    "target_type": entry.target_type,
                    "cleaning_policy": entry.policy,
                    "x_m": float(xyz[0]), "y_m": float(xyz[1]), "z_m": float(xyz[2]),
                    "confidence": confidence,
                    "covariance": covariance,
                    "covariance_trace": float(np.trace(covariance)),
                    "bbox": (x, y, width, height),
                    "size_m": entry.size_m,
                    "source_backend": "onnxruntime",
                    })
            return detections

        def on_image(self, message):
            if stamp(message) <= self.last_source_stamp:
                return
            self.pending_image = message
            now = time.monotonic()
            if now - self.last_inference_monotonic < 0.5:
                return
            if not self.depth_buffer or self.camera_info is None:
                return
            try:
                self.depth = min(self.depth_buffer, key=lambda d: abs(stamp(d)-stamp(message)))
                validate_context(message, self.depth, self.camera_info, float(self.get_parameter("rgbd_max_skew_s").value))
            except ValueError as exc:
                self.rejection_counts["rgbd_context"] += 1
                self.last_projection_error = str(exc)
                empty = GarbageTargetArray(); empty.header = message.header
                empty.header.frame_id = "map"; empty.registry_sha256 = self.registry.sha256
                self.target_publisher.publish(empty)
                return
            self.pending_image = None
            self.last_source_stamp = stamp(message)
            self.last_inference_monotonic = now
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="rgb8")
            tensor = preprocess_rgb(image)
            start = time.perf_counter()
            logits = self.session.run(["logits"], {"images": tensor})[0]
            self.last_latency_ms = (time.perf_counter() - start) * 1000.0
            labels = np.argmax(logits[0], axis=0).astype(np.uint8)
            full_size = resize_labels(labels, message.width, message.height)
            segmentation = self.bridge.cv2_to_imgmsg(full_size, encoding="mono8")
            segmentation.header = message.header
            self.segmentation_publisher.publish(segmentation)
            # Image-only proposals before confidence, depth, ROI and tracking gates.
            raw = Detection2DArray(); raw.header = message.header
            shifted = logits[0] - logits[0].max(axis=0, keepdims=True)
            probability = np.exp(shifted) / np.exp(shifted).sum(axis=0, keepdims=True)
            for class_index, class_id in enumerate(class_order[1:], 1):
                count, components, stats, _ = cv2.connectedComponentsWithStats((full_size == class_index).astype(np.uint8), 8)
                for component in range(1, count):
                    if int(stats[component, cv2.CC_STAT_AREA]) < self.minimum_pixels:
                        continue
                    mask = cv2.resize((components == component).astype(np.uint8), (model_width, model_height), interpolation=cv2.INTER_NEAREST).astype(bool)
                    confidence = float(probability[class_index][mask].mean()) if mask.any() else 0.0
                    x, y, width, height = (int(stats[component, k]) for k in (cv2.CC_STAT_LEFT, cv2.CC_STAT_TOP, cv2.CC_STAT_WIDTH, cv2.CC_STAT_HEIGHT))
                    detection = Detection2D(); detection.header = message.header; detection.id = f"{class_id}:{component}"
                    detection.bbox.center.position.x = x + width / 2.; detection.bbox.center.position.y = y + height / 2.
                    detection.bbox.size_x = float(width); detection.bbox.size_y = float(height)
                    hypothesis = ObjectHypothesisWithPose(); hypothesis.hypothesis.class_id = class_id; hypothesis.hypothesis.score = confidence
                    detection.results.append(hypothesis); raw.detections.append(detection)
            self.last_raw_detection_count = len(raw.detections)
            self.raw_detection2d_publisher.publish(raw)
            try:
                detections = self.extract_detections(full_size, logits[0], message)
                self.last_projection_error = None
            except ProjectionError as exc:
                detections = []
                self.last_projection_error = str(exc)
            tracks = self.tracker.update(detections)
            self.track_state_counts = dict(Counter(track.state for track in tracks))
            active_tracks = [track for track in tracks if publishable(track.state, bool(self.get_parameter("publish_tentative_targets").value))]
            # A missing current transform/input must not republish old tracks.
            if self.last_projection_error is not None:
                active_tracks = []
            detections_2d = Detection2DArray(); detections_2d.header = message.header
            detections_3d = Detection3DArray(); detections_3d.header.stamp = message.header.stamp; detections_3d.header.frame_id = "map"
            for index, detection in enumerate(detections):
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = detection["class_id"]
                hypothesis.hypothesis.score = detection["confidence"]
                x, y, width, height = detection["bbox"]
                detection_2d = Detection2D(); detection_2d.header = message.header
                detection_2d.id = f"{detection['class_id']}:{index}"
                detection_2d.bbox.center.position.x = x + width / 2.0
                detection_2d.bbox.center.position.y = y + height / 2.0
                detection_2d.bbox.size_x = float(width); detection_2d.bbox.size_y = float(height)
                detection_2d.results.append(hypothesis)
                detections_2d.detections.append(detection_2d)
                detection_3d = Detection3D(); detection_3d.header = detections_3d.header
                detection_3d.id = detection_2d.id
                detection_3d.bbox.center.position.x = detection["x_m"]
                detection_3d.bbox.center.position.y = detection["y_m"]
                detection_3d.bbox.center.position.z = detection["z_m"]
                detection_3d.bbox.center.orientation.w = 1.0
                detection_3d.bbox.size.x, detection_3d.bbox.size.y, detection_3d.bbox.size.z = detection["size_m"]
                hypothesis_3d = ObjectHypothesisWithPose()
                hypothesis_3d.hypothesis.class_id = detection["class_id"]
                hypothesis_3d.hypothesis.score = detection["confidence"]
                hypothesis_3d.pose.pose = detection_3d.bbox.center
                detection_3d.results.append(hypothesis_3d)
                detections_3d.detections.append(detection_3d)
            self.detection2d_publisher.publish(detections_2d)
            self.detection3d_publisher.publish(detections_3d)
            targets = GarbageTargetArray(); targets.header.stamp = message.header.stamp; targets.header.frame_id = "map"; targets.registry_sha256 = self.registry.sha256
            for track in active_tracks:
                entry = self.entries_by_class[track.class_id]
                target = GarbageTarget(); target.header = targets.header
                target.uuid = track.uuid; target.class_id = track.class_id; target.target_type = track.target_type
                target.confidence = track.confidence
                target.map_pose.pose.position.x = track.x_m; target.map_pose.pose.position.y = track.y_m
                target.map_pose.pose.orientation.w = 1.0
                target.map_pose.covariance[0] = track.covariance_trace / 3.0
                target.map_pose.covariance[7] = track.covariance_trace / 3.0
                target.map_pose.covariance[14] = track.covariance_trace / 3.0
                target.size.x, target.size.y, target.size.z = entry.size_m
                for px, py in ((-0.5, -0.5), (0.5, -0.5), (0.5, 0.5), (-0.5, 0.5)):
                    target.polygon.points.append(Point32(x=track.x_m + px * entry.size_m[0], y=track.y_m + py * entry.size_m[1], z=0.0))
                target.first_seen = self.get_clock().now().to_msg(); target.last_seen = message.header.stamp
                target.observation_count = track.observation_count; target.track_state = track.state
                target.cleaning_policy = track.cleaning_policy; target.source_backend = "onnxruntime"
                target.source_stamp = message.header.stamp; target.visibility = 1.0; target.occlusion_ratio = 0.0
                targets.targets.append(target)
            self.target_publisher.publish(targets)
            self.last_detection_count = len(detections)
            self.last_map_target_count = len(targets.targets)
            self.frame_count += 1

        def publish_diagnostics(self):
            payload = {
                "requested_backend": "onnxruntime",
                "active_backend": "onnxruntime",
                "model_sha256": self.model_sha256,
                "model_id": self.model_id,
                "model_scope": self.model_scope,
                "learned_weights": self.learned_weights,
                "class_order": list(class_order),
                "frame_count": self.frame_count,
                "last_latency_ms": self.last_latency_ms,
                "camera_info_received": self.camera_info is not None,
                "depth_received": self.depth is not None,
                "last_detection_count": self.last_detection_count,
                "last_raw_detection_count": self.last_raw_detection_count,
                "last_map_target_count": self.last_map_target_count,
                "map_targets_fail_closed": self.last_projection_error is not None,
                "map_targets_fail_closed_reason": self.last_projection_error,
                "ground_truth_input_used": False,
                "ground_truth_control_violation_count": 0,
                "synthetic_only_model": self.model_scope != "real_domain_validated",
                "competition_perception_pass": False,
                "candidate_thresholds": self.thresholds,
                "publish_tentative_targets": bool(self.get_parameter("publish_tentative_targets").value),
                "rejection_counts": dict(self.rejection_counts),
                "track_state_counts": self.track_state_counts,
                "last_source_stamp_s": self.last_source_stamp,
            }
            self.diagnostics_publisher.publish(String(data=json.dumps(payload, sort_keys=True)))

    rclpy.init()
    node = PerceptionNode()
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
