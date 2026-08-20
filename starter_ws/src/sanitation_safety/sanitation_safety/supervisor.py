"""Product-wide watchdog with separate motion and cleaning permissions."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
import time


@dataclass(frozen=True)
class SourceRule:
    name: str
    plane: str
    timeout_sec: float

    def __post_init__(self) -> None:
        if self.plane not in {"motion", "cleaning"}:
            raise ValueError(f"invalid health plane: {self.plane}")
        if self.timeout_sec <= 0.0:
            raise ValueError("health timeout must be positive")


DEFAULT_SOURCE_RULES = (
    SourceRule("scan", "motion", 1.0),
    SourceRule("localization", "motion", 1.0),
    SourceRule("coverage", "motion", 2.5),
    SourceRule("rgb", "cleaning", 1.0),
    SourceRule("depth", "cleaning", 1.0),
    SourceRule("camera_info", "cleaning", 1.0),
    SourceRule("perception", "cleaning", 1.0),
    SourceRule("spot_clean", "cleaning", 1.0),
    SourceRule("reobserve", "cleaning", 1.0),
)


@dataclass
class SourceObservation:
    received_monotonic: float
    healthy: bool = True
    reason: str = "ok"


@dataclass
class ProductSupervisorState:
    rules: tuple[SourceRule, ...] = DEFAULT_SOURCE_RULES
    observations: dict[str, SourceObservation] = field(default_factory=dict)
    report_sequence: int = 0

    def __post_init__(self) -> None:
        names = [rule.name for rule in self.rules]
        if len(names) != len(set(names)):
            raise ValueError("health source names must be unique")

    def observe(
        self,
        source: str,
        now: float,
        *,
        healthy: bool = True,
        reason: str = "ok",
    ) -> None:
        if source not in {rule.name for rule in self.rules}:
            raise KeyError(f"unknown product health source: {source}")
        self.observations[source] = SourceObservation(
            received_monotonic=float(now),
            healthy=bool(healthy),
            reason=str(reason),
        )

    def snapshot(self, now: float) -> dict:
        now = float(now)
        faults = {"motion": [], "cleaning": []}
        sources = {}
        for rule in self.rules:
            observation = self.observations.get(rule.name)
            if observation is None:
                age_sec = None
                reason = "missing"
            else:
                age_sec = max(0.0, now - observation.received_monotonic)
                if age_sec > rule.timeout_sec:
                    reason = "stale"
                elif not observation.healthy:
                    reason = observation.reason or "unhealthy"
                else:
                    reason = "ok"
            healthy = reason == "ok"
            if not healthy:
                faults[rule.plane].append(f"{rule.name}:{reason}")
            sources[rule.name] = {
                "plane": rule.plane,
                "healthy": healthy,
                "reason": reason,
                "age_sec": age_sec,
                "timeout_sec": rule.timeout_sec,
            }

        self.report_sequence += 1
        motion_healthy = not faults["motion"]
        cleaning_healthy = motion_healthy and not faults["cleaning"]
        return {
            "schema_version": 1,
            "sequence": self.report_sequence,
            "state": (
                "ACTIVE"
                if cleaning_healthy
                else ("DEGRADED" if motion_healthy else "ERROR")
            ),
            "motion_healthy": motion_healthy,
            "cleaning_healthy": cleaning_healthy,
            "motion_faults": faults["motion"],
            "cleaning_faults": faults["cleaning"],
            "sources": sources,
        }


def perception_health(message: str) -> tuple[bool, str]:
    try:
        payload = json.loads(message)
    except (TypeError, ValueError):
        return False, "invalid_json"
    state = str(payload.get("state", "UNKNOWN"))
    allowed = payload.get("perception_spot_clean_allowed") is True
    return state == "ACTIVE" and allowed, f"state_{state.lower()}"


def coverage_health(message: str) -> tuple[bool, str]:
    state = str(message).strip().upper()
    if not state:
        return False, "empty_state"
    if state == "FAILED":
        return False, "state_failed"
    return True, f"state_{state.lower()}"


def localization_health(covariance: list[float] | tuple[float, ...]) -> tuple[bool, str]:
    if len(covariance) < 8:
        return False, "covariance_missing"
    trace = float(covariance[0]) + float(covariance[7])
    if not math.isfinite(trace):
        return False, "covariance_nonfinite"
    if trace > 0.25:
        return False, "covariance_excessive"
    return True, "ok"


def has_localization_transform(transforms) -> bool:
    """Return true only for the configured global localization heartbeat."""
    for transform in transforms:
        parent = str(transform.header.frame_id).lstrip("/")
        child = str(transform.child_frame_id).lstrip("/")
        if parent == "map" and child == "odom":
            return True
    return False


def main(args=None) -> None:
    import rclpy
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import CameraInfo, Image, LaserScan
    from std_msgs.msg import String
    from tf2_msgs.msg import TFMessage

    class ProductSupervisor(Node):
        def __init__(self) -> None:
            super().__init__("product_supervisor")
            self.state = ProductSupervisorState()
            self._localization_message = None
            self.health_publisher = self.create_publisher(
                String, "/product/health", 10
            )
            self.safety_publisher = self.create_publisher(
                String, "/safety/supervisor_health", 10
            )
            self.create_subscription(LaserScan, "/scan", self._scan, 10)
            localization_qos = QoSProfile(depth=1)
            localization_qos.reliability = ReliabilityPolicy.RELIABLE
            localization_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
            self.create_subscription(
                PoseWithCovarianceStamped,
                "/localization/fused_pose",
                self._localization,
                localization_qos,
            )
            self.create_subscription(TFMessage, "/tf", self._tf, 50)
            self.create_subscription(String, "/coverage/state", self._coverage, 20)
            self.create_subscription(Image, "/camera/color/image_raw", self._rgb, 10)
            self.create_subscription(
                Image, "/camera/depth/image_rect_raw", self._depth, 10
            )
            self.create_subscription(
                CameraInfo, "/camera/color/camera_info", self._camera_info, 10
            )
            self.create_subscription(
                String, "/perception/product/health", self._perception, 20
            )
            self.create_subscription(
                String, "/spot_clean/state", self._spot_clean, 20
            )
            self.create_subscription(
                String, "/reobserve/state", self._reobserve, 20
            )
            self.create_timer(0.1, self._publish)

        @staticmethod
        def _now() -> float:
            return time.monotonic()

        def _observe(self, source: str, healthy: bool = True, reason: str = "ok") -> None:
            self.state.observe(
                source, self._now(), healthy=healthy, reason=reason
            )

        def _scan(self, _message) -> None:
            self._observe("scan")

        def _localization(self, message) -> None:
            self._localization_message = message
            healthy, reason = localization_health(message.pose.covariance)
            self._observe("localization", healthy, reason)

        def _tf(self, message) -> None:
            # The global backend may publish pose only on measurement events,
            # while map->odom is its continuous operational heartbeat. Refresh
            # only when both canonical pose and transform authority are present;
            # a stopped localization process therefore becomes stale.
            if (
                self._localization_message is not None
                and has_localization_transform(message.transforms)
            ):
                healthy, reason = localization_health(
                    self._localization_message.pose.covariance
                )
                self._observe("localization", healthy, reason)

        def _coverage(self, message) -> None:
            healthy, reason = coverage_health(message.data)
            self._observe("coverage", healthy, reason)

        def _rgb(self, _message) -> None:
            self._observe("rgb")

        def _depth(self, _message) -> None:
            self._observe("depth")

        def _camera_info(self, _message) -> None:
            self._observe("camera_info")

        def _perception(self, message) -> None:
            healthy, reason = perception_health(message.data)
            self._observe("perception", healthy, reason)

        def _spot_clean(self, _message) -> None:
            self._observe("spot_clean")

        def _reobserve(self, _message) -> None:
            self._observe("reobserve")

        def _publish(self) -> None:
            encoded = json.dumps(self.state.snapshot(self._now()), sort_keys=True)
            message = String(data=encoded)
            self.health_publisher.publish(message)
            self.safety_publisher.publish(message)

    rclpy.init(args=args)
    node = ProductSupervisor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RuntimeError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except RuntimeError:
            if rclpy.ok():
                raise
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
