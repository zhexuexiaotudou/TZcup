from __future__ import annotations

from sanitation_perception.camera_frustum_model import CameraFrustumModel
from sanitation_perception.observation_model import MapPoseMeasurement, TargetObservation


def record_sweep(dynamic_map, stamp_ns: int, *, yaw_rad: float = 0.0, image_frame_id: str | None = None):
    frame_id = image_frame_id or f"rgb-{stamp_ns}"
    model = CameraFrustumModel(
        horizontal_fov_rad=1.4,
        minimum_range_m=0.1,
        maximum_range_m=8.0,
    )
    sweep = model.make_sweep(
        sweep_id=f"sweep-{stamp_ns}",
        mission_id=dynamic_map.mission_id,
        stamp_ns=stamp_ns,
        camera_frame_id="camera_color_optical_frame",
        image_frame_id=frame_id,
        camera_x_m=0.0,
        camera_y_m=0.0,
        camera_yaw_rad=yaw_rad,
    )
    dynamic_map.observed_regions.record(sweep)
    return frame_id


def observation(
    dynamic_map,
    stamp_ns: int,
    *,
    x_m: float = 2.0,
    y_m: float = 0.0,
    confidence: float = 0.9,
    covariance: float = 0.01,
    image_frame_id: str | None = None,
    source_backend: str = "onnxruntime",
    in_current_fov: bool = True,
):
    return TargetObservation(
        observation_id=f"obs-{stamp_ns}-{x_m}-{y_m}",
        mission_id=dynamic_map.mission_id,
        stamp_ns=stamp_ns,
        camera_frame_id="camera_color_optical_frame",
        image_frame_id=image_frame_id or f"rgb-{stamp_ns}",
        source_model="fcos-r50-online-x1",
        source_backend=source_backend,
        target_type="DISCRETE",
        class_probabilities={"plastic_bottle": 0.92, "background": 0.08},
        confidence=confidence,
        map_pose=MapPoseMeasurement(
            x_m=x_m,
            y_m=y_m,
            covariance_xx=covariance * 0.5,
            covariance_yy=covariance * 0.5,
        ),
        bbox_xyxy=(100.0, 100.0, 130.0, 160.0),
        estimated_size_m=(0.07, 0.07, 0.22),
        view_direction_rad=0.0,
        in_current_fov=in_current_fov,
    )
