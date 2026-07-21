from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import subprocess
import time
from collections import Counter

import cv2
import numpy as np


MODELS = {
    "same_color_non_garbage": ("stage5br6_same_color", 0.04),
    "bottle_or_can_shaped_obstacle": ("stage5br6_bottle_shape", 0.09),
    "wet_ground_non_puddle": ("stage5br6_wet_patch", 0.003),
    "shadow": ("stage5br6_shadow_patch", 0.002),
    "leaf_background_non_target": ("stage5br6_leaf_background", 0.009),
    "vehicle_self_structure": ("stage5br6_vehicle_bracket", 0.05),
}
BOUNDARY_CATEGORY = "crop_boundary_artifact"
CAMERA_XYZ = (0.67, 0.34, 0.48)
CAMERA_PITCH = math.radians(50.0)


def set_poses(world: str, poses: list[dict]) -> None:
    request = " ".join(
        "pose { " + f'name: "{item["name"]}" position {{ x: {item["xyz"][0]:.6f} y: {item["xyz"][1]:.6f} z: {item["xyz"][2]:.6f} }} '
        + f'orientation {{ z: {math.sin(item["yaw"] / 2):.8f} w: {math.cos(item["yaw"] / 2):.8f} }} }}'
        for item in poses
    )
    result = subprocess.run(["gz", "service", "-s", f"/world/{world}/set_pose_vector", "--reqtype", "gz.msgs.Pose_V", "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req", request], capture_output=True, text=True)
    if result.returncode or "data: true" not in result.stdout:
        raise RuntimeError(f"set_pose_vector failed: {result.stdout} {result.stderr}")


def sha(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world-id", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    from cv_bridge import CvBridge
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image

    output = Path(args.output)
    for name in ("crops", "rgb", "semantic"):
        (output / name).mkdir(parents=True, exist_ok=True)
    topics = (
        "/verification_camera/color/image_raw",
        "/verification_camera/depth/image_rect_raw",
        "/ground_truth/verification_semantic/image",
        "/ground_truth/verification_instance/image",
    )

    class Collector(Node):
        def __init__(self):
            super().__init__("stage5br6_v4_negative_collector")
            self.bridge = CvBridge()
            self.buffers = {topic: {} for topic in topics}
            self.camera = None
            for topic in topics:
                self.create_subscription(Image, topic, lambda msg, key=topic: self.receive(key, msg), qos_profile_sensor_data)
            self.create_subscription(CameraInfo, "/verification_camera/color/camera_info", lambda msg: setattr(self, "camera", msg), qos_profile_sensor_data)

        def receive(self, topic, message):
            stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            self.buffers[topic][stamp] = message
            for bucket in self.buffers.values():
                while len(bucket) > 40:
                    bucket.pop(next(iter(bucket)))

        def next_exact(self, after_stamp: int, discard: int = 15, timeout: float = 12.0):
            deadline, eligible = time.monotonic() + timeout, []
            while time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.05)
                common = sorted(stamp for stamp in set.intersection(*(set(bucket) for bucket in self.buffers.values())) if stamp > after_stamp)
                eligible.extend(stamp for stamp in common if stamp not in eligible)
                if self.camera is not None and len(eligible) > discard:
                    stamp = eligible[-1]
                    return stamp, [self.buffers[topic][stamp] for topic in topics]
            raise RuntimeError("timed out waiting for exact synchronized V4 frame")

    rclpy.init()
    node = Collector()
    bridge, records, last_stamp = node.bridge, [], 0
    hidden = [{"name": name, "xyz": [-250.0 - index, 220.0, -5.0], "yaw": 0.0} for index, (_, (name, _)) in enumerate(MODELS.items())]
    set_poses(args.world_id, hidden)
    try:
        for category_index, (category, (model_name, z)) in enumerate(MODELS.items()):
            for sample_index in range(10):
                dx = 0.29 + 0.045 * (sample_index % 5)
                lateral = -0.10 if sample_index < 5 else 0.10
                poses = [item for item in hidden if item["name"] != model_name]
                poses.append({"name": model_name, "xyz": [-8.0 + CAMERA_XYZ[0] + dx, CAMERA_XYZ[1] + lateral, z], "yaw": sample_index * math.pi / 10.0})
                set_poses(args.world_id, poses)
                stamp, messages = node.next_exact(last_stamp)
                last_stamp = stamp
                rgb = cv2.cvtColor(bridge.imgmsg_to_cv2(messages[0], "rgb8"), cv2.COLOR_RGB2BGR)
                semantic_rgb = bridge.imgmsg_to_cv2(messages[2], "rgb8")
                semantic = semantic_rgb[:, :, 0]
                fx, fy, cx, cy = node.camera.k[0], node.camera.k[4], node.camera.k[2], node.camera.k[5]
                delta_z = z - CAMERA_XYZ[2]
                depth = math.cos(CAMERA_PITCH) * dx - math.sin(CAMERA_PITCH) * delta_z
                camera_y = lateral
                camera_z = math.sin(CAMERA_PITCH) * dx + math.cos(CAMERA_PITCH) * delta_z
                u, v = cx - fx * camera_y / depth, cy - fy * camera_z / depth
                side = 144
                x0 = max(0, min(rgb.shape[1] - side, int(round(u - side / 2))))
                y0 = max(0, min(rgb.shape[0] - side, int(round(v - side / 2))))
                x1, y1 = x0 + side, y0 + side
                crop, semantic_crop = rgb[y0:y1, x0:x1], semantic[y0:y1, x0:x1]
                target_pixels = int(np.isin(semantic_crop, (1, 2, 3, 4, 5)).sum())
                if target_pixels:
                    raise RuntimeError(f"target pixels in negative crop {category}/{sample_index}")
                stem = f"{category}_{sample_index:02d}"
                crop_path, rgb_path, semantic_path = output / "crops" / f"{stem}.png", output / "rgb" / f"{stem}.png", output / "semantic" / f"{stem}.npy"
                cv2.imwrite(str(crop_path), crop, [cv2.IMWRITE_PNG_COMPRESSION, 9])
                cv2.imwrite(str(rgb_path), rgb, [cv2.IMWRITE_PNG_COMPRESSION, 9])
                np.save(semantic_path, semantic, allow_pickle=False)
                records.append({
                    "sample_index": len(records), "negative_category": category, "model_name": model_name,
                    "timestamp_ns": stamp, "exact_four_sensor_timestamp": True, "camera_contract": "V4",
                    "model_xyz_m": poses[-1]["xyz"], "model_yaw_rad": poses[-1]["yaw"],
                    "projected_center_px": [u, v], "crop_bbox_xyxy": [x0, y0, x1, y1],
                    "semantic_target_pixel_count": target_pixels,
                    "crop_path": crop_path.relative_to(output).as_posix(), "crop_sha256": sha(crop_path.read_bytes()),
                    "rgb_path": rgb_path.relative_to(output).as_posix(), "rgb_sha256": sha(rgb_path.read_bytes()),
                    "semantic_path": semantic_path.relative_to(output).as_posix(),
                })
        for sample_index in range(10):
            set_poses(args.world_id, hidden)
            stamp, messages = node.next_exact(last_stamp)
            last_stamp = stamp
            rgb = cv2.cvtColor(bridge.imgmsg_to_cv2(messages[0], "rgb8"), cv2.COLOR_RGB2BGR)
            semantic = bridge.imgmsg_to_cv2(messages[2], "rgb8")[:, :, 0]
            edge, y0 = ("left" if sample_index % 2 == 0 else "right"), 250 + (sample_index % 5) * 10
            padding = 8 + (sample_index % 5) * 4
            source_width = 128 - padding
            x0 = 0 if edge == "left" else rgb.shape[1] - source_width
            source = rgb[y0:y0 + 96, x0:x0 + source_width]
            semantic_source = semantic[y0:y0 + 96, x0:x0 + source_width]
            target_pixels = int(np.isin(semantic_source, (1, 2, 3, 4, 5)).sum())
            if target_pixels:
                raise RuntimeError("target pixels in boundary negative")
            crop = np.zeros((96, 128, 3), dtype=np.uint8)
            if edge == "left": crop[:, padding:] = source
            else: crop[:, :source_width] = source
            stem = f"{BOUNDARY_CATEGORY}_{sample_index:02d}"
            crop_path, rgb_path, semantic_path = output / "crops" / f"{stem}.png", output / "rgb" / f"{stem}.png", output / "semantic" / f"{stem}.npy"
            cv2.imwrite(str(crop_path), crop, [cv2.IMWRITE_PNG_COMPRESSION, 9]); cv2.imwrite(str(rgb_path), rgb, [cv2.IMWRITE_PNG_COMPRESSION, 9]); np.save(semantic_path, semantic, allow_pickle=False)
            records.append({
                "sample_index": len(records), "negative_category": BOUNDARY_CATEGORY, "model_name": None,
                "timestamp_ns": stamp, "exact_four_sensor_timestamp": True, "camera_contract": "V4",
                "boundary_padding": edge, "padding_width_px": padding,
                "crop_bbox_xyxy": [x0, y0, x0 + source_width, y0 + 96],
                "semantic_target_pixel_count": target_pixels,
                "crop_path": crop_path.relative_to(output).as_posix(), "crop_sha256": sha(crop_path.read_bytes()),
                "rgb_path": rgb_path.relative_to(output).as_posix(), "rgb_sha256": sha(rgb_path.read_bytes()),
                "semantic_path": semantic_path.relative_to(output).as_posix(),
            })
    finally:
        node.destroy_node(); rclpy.shutdown()
    report = {
        "schema_version": 1, "stage": "Stage5BR6-A", "world_id": args.world_id,
        "camera_contract": "V4", "camera_xyz_m": list(CAMERA_XYZ), "camera_pitch_deg": -50.0,
        "capture_count": len(records), "all_exact_four_sensor_timestamps": all(row["exact_four_sensor_timestamp"] for row in records),
        "semantic_target_pixel_count": sum(row["semantic_target_pixel_count"] for row in records),
        "negative_category_counts": dict(Counter(row["negative_category"] for row in records)),
        "project_authored_label_zero_negative_models": True, "production_world_modified": False,
        "records": records,
    }
    (output / "capture_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in report.items() if key != "records"}, indent=2))


if __name__ == "__main__":
    main()
