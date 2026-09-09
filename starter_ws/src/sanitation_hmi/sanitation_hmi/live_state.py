"""Thread-safe mission telemetry model used by the final-product dashboard."""

from __future__ import annotations

from collections import deque
from copy import deepcopy
import json
import math
from threading import Lock
import time


TERMINAL_STATES = {"COMPLETED", "FAILED", "CANCELED"}
ACTIVE_COMPONENT_STATES = {
    "EXECUTING_SWATH", "EXECUTING_TURN", "EXECUTING_ROTATE",
    "EXECUTING_SHIFT", "EXECUTING_BACKUP", "EXECUTING_BYPASS",
    "REPAIR_SWATH", "REPAIR_TRANSIT",
}
FINAL_DEMO_TOPIC = "/final_demo/state"
FINAL_DEMO_STAGES = {
    "MAPPING",
    "MAP_SAVED",
    "HARD_RESTART",
    "RELOAD_LOCALIZE",
    "COVERAGE",
    "PRODUCT_TERMINAL",
}
FINAL_DEMO_PERCEPTION_PROVIDERS = {"s100p", "pc", "unavailable"}
FINAL_DEMO_STALE_SECONDS = 5.0
LIVE_INPUT_STALE_SECONDS = 5.0
OCCUPANCY_GRID_MAX_AXIS = 192


def _bounded_points(points, maximum: int = 320) -> list[list[float]]:
    normalized = [[float(point[0]), float(point[1])] for point in points]
    if len(normalized) <= maximum:
        return normalized
    stride = max(1, math.ceil(len(normalized) / maximum))
    sampled = normalized[::stride]
    if sampled[-1] != normalized[-1]:
        sampled.append(normalized[-1])
    return sampled


def _compact_occupancy_grid(
    *,
    width: int,
    height: int,
    resolution: float,
    origin_x: float,
    origin_y: float,
    data,
) -> dict | None:
    """Keep a bounded, conservative `/map` image for the browser.

    SLAM maps can contain millions of cells.  The HMI needs a display layer,
    not a second full-resolution map cache, so it samples at most nine points
    per output cell and retains the highest known sampled occupancy.
    """
    source_width = int(width)
    source_height = int(height)
    if (
        source_width <= 0
        or source_height <= 0
        or float(resolution) <= 0.0
        or len(data) != source_width * source_height
    ):
        return None
    stride = max(1, math.ceil(max(source_width, source_height) / OCCUPANCY_GRID_MAX_AXIS))
    compact_width = math.ceil(source_width / stride)
    compact_height = math.ceil(source_height / stride)
    compact_data: list[int] = []
    for compact_y in range(compact_height):
        y_start = compact_y * stride
        y_stop = min(source_height, y_start + stride)
        for compact_x in range(compact_width):
            x_start = compact_x * stride
            x_stop = min(source_width, x_start + stride)
            sample_rows = {y_start, y_start + (y_stop - y_start) // 2, y_stop - 1}
            sample_columns = {x_start, x_start + (x_stop - x_start) // 2, x_stop - 1}
            highest_known = -1
            for source_y in sample_rows:
                row_start = source_y * source_width
                for source_x in sample_columns:
                    value = int(data[row_start + source_x])
                    if value >= 0:
                        highest_known = max(highest_known, min(100, value))
            compact_data.append(highest_known)
    return {
        "width": compact_width,
        "height": compact_height,
        "resolution": float(resolution) * stride,
        "origin": [float(origin_x), float(origin_y)],
        "data": compact_data,
        "downsample_stride": stride,
        "source_dimensions": [source_width, source_height],
    }


class LiveMissionState:
    """Keep a compact, browser-safe snapshot of the current demo mission."""

    def __init__(
        self,
        *,
        expected_components: int = 17,
        mission_id: str = "final_product_visualization",
        geometry: dict | None = None,
        clock=time.monotonic,
    ) -> None:
        self._lock = Lock()
        self._clock = clock
        self._started_monotonic = clock()
        self._last_update_monotonic = self._started_monotonic
        self._state = "BOOTING"
        self._details: dict = {}
        self._expected_components = max(1, int(expected_components))
        self._mission_id = str(mission_id)
        self._geometry = deepcopy(geometry or {})
        self._seen_components: list[str] = []
        self._current_component: str | None = None
        self._estimated_pose: list[float] | None = None
        # `/odom` is useful before SLAM emits a map pose.  It remains a
        # clearly-labelled preview in odom coordinates, not a localization or
        # evaluator-truth substitute.
        self._odometry_preview_pose: list[float] | None = None
        self._evaluation_pose: list[float] | None = None
        # Never turn a missing command/actuator topic into a plausible zero or
        # false operator state.  These values become observable only after a
        # fresh ROS message has actually arrived.
        self._commanded_linear_speed: float | None = None
        self._commanded_angular_speed: float | None = None
        self._command_source = "unavailable"
        self._measured_linear_speed: float | None = None
        self._measured_angular_speed: float | None = None
        self._formal_base_command_seen = False
        self._brush_enabled: bool | None = None
        self._emergency_stop: bool | None = None
        self._planned_path: list[list[float]] = []
        self._occupancy_grid: dict | None = None
        self._map_revision = 0
        self._map_known_cell_count: int | None = None
        self._odometry_preview_trajectory: deque[list[float]] = deque(maxlen=1200)
        self._trajectory: deque[list[float]] = deque(maxlen=1200)
        self._cleaned_trajectory: deque[list[float]] = deque(maxlen=1200)
        self._events: deque[dict] = deque(maxlen=16)
        self._topics_seen: set[str] = set()
        # Keep observation sources independent.  In particular, an
        # evaluation sample may describe the brush state at the point where a
        # coverage metric was computed; it is not a replacement for the live
        # actuator-topic state displayed to an operator.
        self._evaluation_sample_brush_enabled: bool | None = None
        self._front_camera_png: bytes | None = None
        self._live_inputs: dict[str, dict] = {
            "front_camera": {
                "topic": "/sensors/front_rgbd/depth/image_rect_raw/image",
                "received_monotonic": None,
                "error": None,
                "width": None,
                "height": None,
            },
            "perception_targets": {
                "topic": "/perception/garbage/targets",
                "received_monotonic": None,
                "error": None,
                "count": None,
            },
            "perception_diagnostics": {
                "topic": "/perception/open_vocab/diagnostics",
                "received_monotonic": None,
                "error": None,
                "statuses": [],
            },
            "commanded_speed": {
                "topic": "/cmd_vel",
                "received_monotonic": None,
                "error": None,
                "value": None,
            },
            "measured_speed": {
                "topic": "/odom",
                "received_monotonic": None,
                "error": None,
                "value": None,
            },
            "brush": {
                "topic": "/brush_enabled",
                "received_monotonic": None,
                "error": None,
                "value": None,
            },
            "emergency_stop": {
                "topic": "/emergency_stop",
                "received_monotonic": None,
                "error": None,
                "value": None,
            },
            "safety_status": {
                "topic": "/safety/status_json",
                "received_monotonic": None,
                "error": None,
                "value": None,
            },
            "drivetrain_status": {
                "topic": "/model/tzcup_formal_sanitation_vehicle/a300_drivetrain/status",
                "received_monotonic": None,
                "error": None,
                "value": None,
            },
            "cleaning_motor_status": {
                "topic": "/model/tzcup_formal_sanitation_vehicle/cleaning_motors/telemetry_snapshot",
                "received_monotonic": None,
                "error": None,
                "value": None,
            },
            "mapping_lifecycle": {
                "topic": "/formal_mapping/lifecycle_status",
                "received_monotonic": None,
                "error": None,
                "value": None,
                # Mapping evaluates on simulated time.  Preserve the last
                # status and its wall-clock receipt age instead of declaring a
                # low-RTF run failed after an arbitrary dashboard timeout.
                "freshness_mode": "observed",
            },
            "mapping_map_ready": {
                "topic": "/formal_mapping/map_ready",
                "received_monotonic": None,
                "error": None,
                "value": None,
                # This is a transient-local, retained lifecycle fact, not a
                # periodic heartbeat.
                "freshness_mode": "retained",
            },
            "mapping_explorer": {
                "topic": "/formal_mapping/explorer_status",
                "received_monotonic": None,
                "error": None,
                "value": None,
                "freshness_mode": "observed",
            },
            "saved_map_coverage": {
                "topic": "/formal_saved_map_coverage/state",
                "received_monotonic": None,
                "error": None,
                "value": None,
            },
            "map": {
                "topic": "/map",
                "received_monotonic": None,
                "error": None,
                "revision": 0,
                "known_cell_count": None,
                "known_cell_delta": None,
            },
            "map_pose": {
                "topic": "/tf map->base_footprint",
                "received_monotonic": None,
                "error": None,
            },
        }
        self._final_demo: dict = {
            "status": "unavailable",
            "reason": f"未收到 {FINAL_DEMO_TOPIC}",
            "field_dimensions_m": None,
            "vehicle": None,
            "stage": None,
            "map_sha256": None,
            "perception_provider": None,
            "formal_product_acceptance": None,
            "received_monotonic": None,
        }
        self._append_event("BOOTING", "等待 ROS 2 / Gazebo / Nav2 就绪")

    def _touch(self) -> None:
        self._last_update_monotonic = self._clock()

    def _append_event(self, state: str, label: str) -> None:
        now = self._clock()
        self._events.append(
            {
                "state": str(state),
                "label": str(label),
                "elapsed_sec": round(now - self._started_monotonic, 1),
            }
        )

    def mark_topic(self, topic: str) -> None:
        with self._lock:
            self._topics_seen.add(str(topic))
            self._touch()

    def update_state(self, state: str, details: dict | None = None) -> None:
        normalized = str(state or "UNKNOWN")
        with self._lock:
            if normalized != self._state:
                if normalized in TERMINAL_STATES and self._current_component:
                    if self._current_component not in self._seen_components:
                        self._seen_components.append(self._current_component)
                self._append_event(normalized, self._event_label(normalized, details))
            self._state = normalized
            self._details = deepcopy(details or {})
            self._topics_seen.add("/coverage/state")
            self._touch()

    def update_component(self, payload: dict) -> None:
        with self._lock:
            expected = payload.get("expected_components")
            if expected is not None and int(expected) > 0:
                self._expected_components = int(expected)
            kind = payload.get("kind")
            index = payload.get("index")
            state = str(payload.get("state") or self._state)
            if state in ACTIVE_COMPONENT_STATES and kind is not None and index is not None:
                key = str(payload.get("component_id") or f"{kind}:{int(index)}")
                if key != self._current_component:
                    if (
                        self._current_component
                        and self._current_component not in self._seen_components
                    ):
                        self._seen_components.append(self._current_component)
                    self._current_component = key
                    self._append_event(
                        state,
                        f"{'清扫带' if kind == 'swath' else '转弯'} {int(index) + 1}",
                    )
            self._details = deepcopy(payload)
            self._state = state
            self._topics_seen.add("/coverage/component_state")
            self._touch()

    def update_estimated_pose(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        source_topic: str = "/localization/fused_pose",
    ) -> None:
        with self._lock:
            self._estimated_pose = [float(x), float(y), float(yaw)]
            self._live_inputs["map_pose"]["topic"] = str(source_topic)
            self._update_live_input("map_pose")

    def update_formal_odometry_preview(self, x: float, y: float, yaw: float) -> None:
        """Record the live formal `/odom` preview without claiming map alignment."""
        point = [float(x), float(y)]
        with self._lock:
            self._odometry_preview_pose = [point[0], point[1], float(yaw)]
            if (
                not self._odometry_preview_trajectory
                or math.dist(self._odometry_preview_trajectory[-1], point) >= 0.025
            ):
                self._odometry_preview_trajectory.append(point)
            self._topics_seen.add("/odom")
            self._touch()

    def update_evaluation_sample(
        self,
        x: float,
        y: float,
        yaw: float,
        *,
        brush_enabled: bool,
        coverage_state: str | None = None,
    ) -> None:
        point = [float(x), float(y)]
        with self._lock:
            self._evaluation_pose = [point[0], point[1], float(yaw)]
            if not self._trajectory or math.dist(self._trajectory[-1], point) >= 0.025:
                self._trajectory.append(point)
            if brush_enabled and (
                not self._cleaned_trajectory
                or math.dist(self._cleaned_trajectory[-1], point) >= 0.025
            ):
                self._cleaned_trajectory.append(point)
            self._evaluation_sample_brush_enabled = bool(brush_enabled)
            if coverage_state and self._state not in TERMINAL_STATES:
                self._state = str(coverage_state)
            self._topics_seen.add("/coverage/evaluation_sample")
            self._touch()

    def update_velocity(self, linear: float, angular: float) -> None:
        with self._lock:
            # Keep the formal base-controller command stable once it has been
            # observed.  `/cmd_vel` is retained for older demos/fallback, but
            # must not race the final product command stream in the display.
            if not self._formal_base_command_seen:
                self._commanded_linear_speed = float(linear)
                self._commanded_angular_speed = float(angular)
                self._command_source = "/cmd_vel"
                self._update_live_input(
                    "commanded_speed", value=self._commanded_linear_speed
                )

    def update_formal_base_command_velocity(self, linear: float, angular: float) -> None:
        """Record the safety-manager command; it is not measured velocity."""
        with self._lock:
            self._commanded_linear_speed = float(linear)
            self._commanded_angular_speed = float(angular)
            self._command_source = "/base_controller/cmd_vel"
            self._formal_base_command_seen = True
            self._live_inputs["commanded_speed"]["topic"] = "/base_controller/cmd_vel"
            self._update_live_input(
                "commanded_speed", value=self._commanded_linear_speed
            )

    def update_measured_velocity(self, linear: float, angular: float) -> None:
        """Record the odometry twist separately from any controller command."""
        with self._lock:
            self._measured_linear_speed = float(linear)
            self._measured_angular_speed = float(angular)
            self._update_live_input(
                "measured_speed", value=self._measured_linear_speed
            )

    def update_brush(self, enabled: bool) -> None:
        with self._lock:
            self._brush_enabled = bool(enabled)
            self._update_live_input("brush", value=self._brush_enabled)

    def update_front_camera(self, png: bytes, *, width: int, height: int) -> None:
        """Store a real camera frame for the dedicated image endpoint."""
        if not png or width <= 0 or height <= 0:
            raise ValueError(
                "front camera image must be non-empty with positive dimensions"
            )
        with self._lock:
            self._front_camera_png = bytes(png)
            self._update_live_input(
                "front_camera", width=int(width), height=int(height)
            )

    def update_live_input_error(self, name: str, error: str) -> None:
        with self._lock:
            if name not in self._live_inputs:
                raise ValueError(f"unknown live input: {name}")
            entry = self._live_inputs[name]
            entry["error"] = str(error)
            entry["received_monotonic"] = self._clock()
            self._topics_seen.add(str(entry["topic"]))
            self._touch()

    def set_live_input_topic(self, name: str, topic: str) -> None:
        if not isinstance(topic, str) or not topic.startswith("/"):
            raise ValueError("live input topic must be an absolute ROS topic")
        with self._lock:
            if name not in self._live_inputs:
                raise ValueError(f"unknown live input: {name}")
            self._live_inputs[name]["topic"] = topic

    def update_perception_targets(self, count: int) -> None:
        if count < 0:
            raise ValueError("perception target count must not be negative")
        with self._lock:
            self._update_live_input("perception_targets", count=int(count))

    def update_perception_diagnostics(self, statuses: list[dict]) -> None:
        normalized = []
        for status in statuses:
            name = status.get("name")
            message = status.get("message")
            level = status.get("level")
            if not isinstance(name, str) or not isinstance(message, str):
                continue
            if isinstance(level, bool) or not isinstance(level, int):
                continue
            normalized.append({"name": name, "message": message, "level": level})
        with self._lock:
            self._update_live_input("perception_diagnostics", statuses=normalized)

    def update_mapping_lifecycle(self, value: str) -> None:
        with self._lock:
            if self._state == "BOOTING" and str(value):
                self._state = "MAPPING"
                self._details = {"source": "/formal_mapping/lifecycle_status"}
                self._append_event("MAPPING", "首次建图运行中")
            self._update_live_input("mapping_lifecycle", value=str(value))

    def update_mapping_map_ready(self, value: bool) -> None:
        with self._lock:
            self._update_live_input("mapping_map_ready", value=bool(value))

    def update_mapping_explorer(self, value: str) -> None:
        with self._lock:
            self._update_live_input("mapping_explorer", value=str(value))

    def update_saved_map_coverage(self, value: str) -> None:
        with self._lock:
            self._update_live_input("saved_map_coverage", value=str(value))

    def update_safety_status(self, value: str) -> None:
        with self._lock:
            self._update_live_input("safety_status", value=str(value))

    def update_drivetrain_status(self, value: str) -> None:
        with self._lock:
            self._update_live_input("drivetrain_status", value=str(value))

    def update_cleaning_motor_status(self, value: str) -> None:
        with self._lock:
            self._update_live_input("cleaning_motor_status", value=str(value))

    def front_camera_png(self) -> bytes | None:
        """Return a frame only while its actual ROS source remains fresh."""
        with self._lock:
            if self._front_camera_png is None:
                return None
            camera = self._live_inputs["front_camera"]
            received = camera["received_monotonic"]
            if camera["error"] is not None:
                return None
            if received is None or self._clock() - received > LIVE_INPUT_STALE_SECONDS:
                return None
            return self._front_camera_png

    def _update_live_input(self, name: str, **values) -> None:
        entry = self._live_inputs[name]
        entry.update(values)
        entry["error"] = None
        entry["received_monotonic"] = self._clock()
        self._topics_seen.add(str(entry["topic"]))
        self._touch()

    def _live_inputs_snapshot(self, now: float) -> dict:
        values = deepcopy(self._live_inputs)
        for entry in values.values():
            received = entry.pop("received_monotonic", None)
            age = None if received is None else round(now - received, 2)
            entry["age_sec"] = age
            if entry.get("error"):
                entry["status"] = "error"
            elif received is None:
                entry["status"] = "unavailable"
            elif entry.get("freshness_mode") == "retained":
                entry["status"] = "retained"
            elif entry.get("freshness_mode") == "observed":
                entry["status"] = "observed"
            elif age is not None and age > LIVE_INPUT_STALE_SECONDS:
                entry["status"] = "stale"
            else:
                entry["status"] = "live"
        return values

    def update_emergency_stop(self, enabled: bool) -> None:
        with self._lock:
            self._emergency_stop = bool(enabled)
            self._update_live_input("emergency_stop", value=self._emergency_stop)

    def update_planned_path(self, points) -> None:
        with self._lock:
            self._planned_path = _bounded_points(points)
            self._topics_seen.add("/coverage/current_path")
            self._touch()

    def update_occupancy_grid(
        self,
        *,
        width: int,
        height: int,
        resolution: float,
        origin_x: float,
        origin_y: float,
        data,
    ) -> None:
        """Store only a bounded visualization projection of the live `/map`."""
        compact = _compact_occupancy_grid(
            width=width,
            height=height,
            resolution=resolution,
            origin_x=origin_x,
            origin_y=origin_y,
            data=data,
        )
        if compact is None:
            return
        with self._lock:
            known_cells = sum(value >= 0 for value in compact["data"])
            previous_known_cells = self._map_known_cell_count
            self._map_revision += 1
            self._map_known_cell_count = known_cells
            compact["revision"] = self._map_revision
            compact["known_cell_count"] = known_cells
            compact["known_cell_delta"] = (
                None if previous_known_cells is None
                else known_cells - previous_known_cells
            )
            self._occupancy_grid = compact
            self._update_live_input(
                "map",
                revision=self._map_revision,
                known_cell_count=known_cells,
                known_cell_delta=compact["known_cell_delta"],
            )

    def update_final_demo_state(self, raw_payload: str) -> None:
        """Accept only a live, self-describing final-product status record."""
        with self._lock:
            self._topics_seen.add(FINAL_DEMO_TOPIC)
            now = self._clock()
            try:
                payload = json.loads(raw_payload)
                normalized = self._normalize_final_demo_payload(payload)
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                self._final_demo = {
                    "status": "error",
                    "reason": f"无效 {FINAL_DEMO_TOPIC}：{exc}",
                    "field_dimensions_m": None,
                    "vehicle": None,
                    "stage": None,
                    "map_sha256": None,
                    "perception_provider": None,
                    "formal_product_acceptance": None,
                    "received_monotonic": now,
                }
            else:
                self._final_demo = {
                    "status": "live",
                    "reason": None,
                    **normalized,
                    "received_monotonic": now,
                }
            self._touch()

    @staticmethod
    def _normalize_final_demo_payload(payload: object) -> dict:
        if not isinstance(payload, dict):
            raise ValueError("payload must be a JSON object")
        dimensions = payload.get("field_dimensions_m")
        if (
            not isinstance(dimensions, list)
            or len(dimensions) != 2
            or any(isinstance(value, bool) or not isinstance(value, (int, float)) for value in dimensions)
            or not math.isclose(float(dimensions[0]), 200.0)
            or not math.isclose(float(dimensions[1]), 100.0)
        ):
            raise ValueError("field_dimensions_m must be [200, 100]")
        vehicle = payload.get("vehicle")
        if vehicle != "A300":
            raise ValueError("vehicle must be A300")
        stage = payload.get("stage")
        if stage not in FINAL_DEMO_STAGES:
            raise ValueError("stage is not a final-product lifecycle stage")
        map_sha256 = payload.get("map_sha256")
        if map_sha256 is not None and (
            not isinstance(map_sha256, str)
            or len(map_sha256) != 64
            or any(character not in "0123456789abcdef" for character in map_sha256.lower())
        ):
            raise ValueError("map_sha256 must be a SHA-256 string or null")
        provider = payload.get("perception_provider")
        if provider not in FINAL_DEMO_PERCEPTION_PROVIDERS:
            raise ValueError("perception_provider must be s100p, pc, or unavailable")
        acceptance = payload.get("formal_product_acceptance")
        if not isinstance(acceptance, bool):
            raise ValueError("formal_product_acceptance must be boolean")
        return {
            "field_dimensions_m": [float(dimensions[0]), float(dimensions[1])],
            "vehicle": vehicle,
            "stage": stage,
            "map_sha256": map_sha256,
            "perception_provider": provider,
            "formal_product_acceptance": acceptance,
        }

    def _final_demo_snapshot(self, now: float) -> dict:
        final_demo = deepcopy(self._final_demo)
        received = final_demo.pop("received_monotonic", None)
        age = None if received is None else round(now - received, 2)
        if final_demo["status"] == "live" and age is not None and age > FINAL_DEMO_STALE_SECONDS:
            final_demo["status"] = "stale"
            final_demo["reason"] = f"{FINAL_DEMO_TOPIC} 超过 {FINAL_DEMO_STALE_SECONDS:g} 秒未更新"
        final_demo["source_topic"] = FINAL_DEMO_TOPIC
        final_demo["age_sec"] = age
        return final_demo

    def snapshot(self) -> dict:
        with self._lock:
            now = self._clock()
            completed = len(self._seen_components)
            if self._state in TERMINAL_STATES and self._current_component:
                completed = min(self._expected_components, completed)
            active_number = min(
                self._expected_components,
                completed + (1 if self._current_component else 0),
            )
            return {
                "schema_version": 1,
                "mission_id": self._mission_id,
                "status": self._state,
                "terminal": self._state in TERMINAL_STATES,
                "elapsed_sec": round(now - self._started_monotonic, 1),
                "last_update_age_sec": round(now - self._last_update_monotonic, 2),
                "progress": {
                    "completed_components": completed,
                    "active_component_number": active_number,
                    "expected_components": self._expected_components,
                    "ratio": round(
                        min(1.0, completed / self._expected_components), 4
                    ),
                    "current_component": self._current_component,
                },
                "vehicle": {
                    "estimated_pose_map": deepcopy(self._estimated_pose),
                    "odometry_preview_pose_odom": deepcopy(self._odometry_preview_pose),
                    "evaluation_only_pose_map": deepcopy(self._evaluation_pose),
                    "commanded_linear_speed_m_s": (
                        round(self._commanded_linear_speed, 4)
                        if self._commanded_linear_speed is not None else None
                    ),
                    "commanded_angular_speed_rad_s": (
                        round(self._commanded_angular_speed, 4)
                        if self._commanded_angular_speed is not None else None
                    ),
                    "command_source": self._command_source,
                    "measured_linear_speed_m_s": (
                        round(self._measured_linear_speed, 4)
                        if self._measured_linear_speed is not None else None
                    ),
                    "measured_angular_speed_rad_s": (
                        round(self._measured_angular_speed, 4)
                        if self._measured_angular_speed is not None else None
                    ),
                },
                "cleaning": {
                    "brush_enabled": self._brush_enabled,
                    "evaluation_sample_brush_enabled": self._evaluation_sample_brush_enabled,
                    "emergency_stop": self._emergency_stop,
                },
                "visualization": {
                    "planned_path": deepcopy(self._planned_path),
                    "occupancy_grid": deepcopy(self._occupancy_grid),
                    "odometry_preview_trajectory_odom": list(
                        self._odometry_preview_trajectory
                    ),
                    "evaluation_only_trajectory": list(self._trajectory),
                    "evaluation_only_cleaned_trajectory": list(
                        self._cleaned_trajectory
                    ),
                    "geometry": deepcopy(self._geometry),
                },
                "events": list(self._events),
                "topics_seen": sorted(self._topics_seen),
                "details": deepcopy(self._details),
                "final_demo": self._final_demo_snapshot(now),
                "live_inputs": self._live_inputs_snapshot(now),
                "claim_boundary": {
                    "source_level": "LIVE_FINAL_PRODUCT_VISUALIZATION_PREVIEW",
                    "ground_truth_usage": "evaluation_and_visualization_only",
                    "odometry_preview_usage": "live_odom_frame_preview_not_map_localization_or_truth",
                    "learned_perception_pass": False,
                    "real_domain_pass": False,
                    "j6_runtime_pass": False,
                    "competition_matrix_pass": False,
                },
            }

    @staticmethod
    def _event_label(state: str, details: dict | None) -> str:
        labels = {
            "PLANNING": "生成覆盖路径",
            "TRANSIT_PREFLIGHT": "检查起点可达性",
            "TRANSIT": "驶向清扫起点",
            "ALIGNING": "对齐首条清扫带",
            "EXECUTING_SWATH": "执行清扫带",
            "EXECUTING_TURN": "执行转弯",
            "RECOVERY": "导航恢复",
            "COMPLETED": "任务完成",
            "FAILED": "任务失败",
            "CANCELED": "任务取消",
        }
        label = labels.get(state, state)
        if details and details.get("error"):
            label = f"{label}：{details['error']}"
        return label
