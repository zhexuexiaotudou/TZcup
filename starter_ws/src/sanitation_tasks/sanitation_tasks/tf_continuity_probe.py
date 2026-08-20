"""Fail-closed TF continuity evidence for mapping and reload phases."""

from __future__ import annotations

import json
from pathlib import Path
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

from .tf_continuity_core import transform_jump


def _yaw(quaternion) -> float:
    import math

    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


class TfContinuityProbe(Node):
    """Continuously sample one TF edge without subscribing to ground truth."""

    def __init__(self) -> None:
        super().__init__("sanitation_tf_continuity_probe")
        self.declare_parameter("output_path", "/tmp/tf_continuity.json")
        self.declare_parameter("parent_frame", "map")
        self.declare_parameter("child_frame", "odom")
        self.declare_parameter("sample_period_sec", 0.10)
        self.declare_parameter("maximum_gap_sec", 0.50)
        self.declare_parameter("warmup_sec", 0.0)
        self.declare_parameter("jump_translation_threshold_m", 1.0)
        self.declare_parameter("jump_yaw_threshold_rad", 0.35)
        self.output_path = Path(str(self.get_parameter("output_path").value))
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.started_wall = time.monotonic()
        self.last_success_wall = None
        self.last_stamp = None
        self.last_transform = None
        self.sample_count = 0
        self.lookup_failure_count = 0
        self.break_count = 0
        self.stamp_regression_count = 0
        self.jump_count = 0
        self.jump_events = []
        self.max_success_gap_sec = 0.0
        self.max_translation_jump_m = 0.0
        self.max_yaw_jump_rad = 0.0
        self.outage_active = False
        self.last_error = None
        self.measurement_started_wall = None
        self.create_timer(
            float(self.get_parameter("sample_period_sec").value), self._tick
        )

    def _tick(self) -> None:
        now = time.monotonic()
        warmup_sec = float(self.get_parameter("warmup_sec").value)
        if (
            self.measurement_started_wall is None
            and now - self.started_wall >= warmup_sec
        ):
            self.measurement_started_wall = now
            self.last_success_wall = None
            self.last_stamp = None
            self.last_transform = None
            self.sample_count = 0
            self.lookup_failure_count = 0
            self.break_count = 0
            self.stamp_regression_count = 0
            self.jump_count = 0
            self.jump_events = []
            self.max_success_gap_sec = 0.0
            self.max_translation_jump_m = 0.0
            self.max_yaw_jump_rad = 0.0
            self.outage_active = False
            self.last_error = None
        try:
            transform = self.tf_buffer.lookup_transform(
                str(self.get_parameter("parent_frame").value),
                str(self.get_parameter("child_frame").value),
                rclpy.time.Time(),
            )
        except TransformException as error:
            if self.measurement_started_wall is None:
                self._write_report(now)
                return
            self.lookup_failure_count += 1
            self.last_error = str(error)
            maximum_gap = float(self.get_parameter("maximum_gap_sec").value)
            if (
                self.last_success_wall is not None
                and now - self.last_success_wall > maximum_gap
                and not self.outage_active
            ):
                self.break_count += 1
                self.outage_active = True
            self._write_report(now)
            return

        stamp = (
            float(transform.header.stamp.sec)
            + float(transform.header.stamp.nanosec) * 1e-9
        )
        translation = transform.transform.translation
        current = (
            float(translation.x),
            float(translation.y),
            _yaw(transform.transform.rotation),
        )
        if self.last_success_wall is not None:
            self.max_success_gap_sec = max(
                self.max_success_gap_sec, now - self.last_success_wall
            )
        if self.last_stamp is not None and stamp + 1e-9 < self.last_stamp:
            self.stamp_regression_count += 1
            self.break_count += 1
        if self.last_transform is not None:
            jump = transform_jump(
                self.last_transform,
                current,
                translation_threshold_m=float(
                    self.get_parameter("jump_translation_threshold_m").value
                ),
                yaw_threshold_rad=float(
                    self.get_parameter("jump_yaw_threshold_rad").value
                ),
            )
            self.max_translation_jump_m = max(
                self.max_translation_jump_m, float(jump["translation_m"])
            )
            self.max_yaw_jump_rad = max(
                self.max_yaw_jump_rad, float(jump["yaw_rad"])
            )
            self.jump_count += int(jump["exceeds_diagnostic_threshold"])
            if jump["exceeds_diagnostic_threshold"] and len(self.jump_events) < 20:
                self.jump_events.append(
                    {
                        "measurement_elapsed_sec": (
                            now - self.measurement_started_wall
                        ),
                        "translation_m": float(jump["translation_m"]),
                        "yaw_rad": float(jump["yaw_rad"]),
                        "previous_xyyaw": list(self.last_transform),
                        "current_xyyaw": list(current),
                    }
                )
        self.sample_count += 1
        self.last_success_wall = now
        self.last_stamp = stamp
        self.last_transform = current
        self.outage_active = False
        self.last_error = None
        self._write_report(now)

    def _write_report(self, now: float) -> None:
        maximum_gap = float(self.get_parameter("maximum_gap_sec").value)
        success_age = (
            now - self.last_success_wall
            if self.last_success_wall is not None
            else None
        )
        report = {
            "schema_version": 1,
            "stage": "PRODUCT-MAPPING-TF-CONTINUITY",
            "parent_frame": str(self.get_parameter("parent_frame").value),
            "child_frame": str(self.get_parameter("child_frame").value),
            "elapsed_wall_sec": now - self.started_wall,
            "warmup_sec": float(self.get_parameter("warmup_sec").value),
            "warmup_complete": self.measurement_started_wall is not None,
            "measurement_elapsed_sec": (
                now - self.measurement_started_wall
                if self.measurement_started_wall is not None
                else 0.0
            ),
            "sample_count": self.sample_count,
            "lookup_failure_count": self.lookup_failure_count,
            "coordinate_frame_break_count": self.break_count,
            "stamp_regression_count": self.stamp_regression_count,
            "maximum_success_gap_sec": self.max_success_gap_sec,
            "last_success_age_sec": success_age,
            "maximum_allowed_gap_sec": maximum_gap,
            "continuous": bool(
                self.measurement_started_wall is not None
                and self.sample_count >= 10
                and self.break_count == 0
                and self.jump_count == 0
                and success_age is not None
                and success_age <= maximum_gap
            ),
            "diagnostic_transform_jump_count": self.jump_count,
            "diagnostic_transform_jump_events": self.jump_events,
            "maximum_translation_jump_m": self.max_translation_jump_m,
            "maximum_yaw_jump_rad": self.max_yaw_jump_rad,
            "last_transform_xyyaw": list(self.last_transform)
            if self.last_transform else None,
            "last_error": self.last_error,
            "ground_truth_used": False,
        }
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output_path.with_suffix(self.output_path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(self.output_path)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TfContinuityProbe()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except _rclpy.RCLError:
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        # Launch shutdown may invalidate the shared context before spin exits.
        # Keep cleanup idempotent so a normal stop cannot become a failure.
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
