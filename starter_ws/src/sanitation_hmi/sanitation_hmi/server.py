"""Local HTTP server for the map-first human visualization console."""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import mimetypes
from pathlib import Path
import secrets
import sys
from urllib.parse import urlparse

from .gateway import CommandGateway
from .reference import load_real_replay, load_reference
from .state import VisualizationState


def build_handler(
    gateway: CommandGateway,
    web_root: Path,
    state: VisualizationState | None = None,
):
    state = state or VisualizationState()

    class Handler(BaseHTTPRequestHandler):
        server_version = "TZcupHMI/2"

        def send_bytes(
            self,
            status: int,
            body: bytes,
            content_type: str,
            *,
            disposition: str | None = None,
        ) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            if disposition:
                self.send_header("Content-Disposition", disposition)
            self.end_headers()
            self.wfile.write(body)

        def send_json(
            self,
            status: int,
            payload: dict | list,
            *,
            disposition: str | None = None,
        ) -> None:
            body = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode()
            self.send_bytes(
                status,
                body,
                "application/json; charset=utf-8",
                disposition=disposition,
            )

        def do_GET(self):
            path = urlparse(self.path).path
            if path == "/healthz":
                snapshot = state.snapshot()
                self.send_json(
                    200,
                    {
                        "status": "ok",
                        "system_status": snapshot["system_status"],
                        "mode": snapshot["mode"],
                    },
                )
                return
            if path == "/api/v1/state":
                self.send_json(200, state.snapshot())
                return
            if path == "/api/v1/replay":
                replay = state.snapshot(include_replay_samples=True).get("replay")
                if replay is None:
                    self.send_json(404, {"status": "unavailable", "reason": "no_real_replay_loaded"})
                else:
                    self.send_json(200, replay)
                return
            if path == "/api/v1/export":
                payload = state.snapshot()
                payload["export_notice"] = (
                    "动态字段来自当前 ROS 数据；参考真值仅用于显示和评测；"
                    "缺失字段不能解释为正常。"
                )
                self.send_json(
                    200,
                    payload,
                    disposition='attachment; filename="tzcup-mission-summary.json"',
                )
                return
            if path.startswith("/api/v1/images/"):
                name = path.rsplit("/", 1)[-1]
                image = state.get_image(name)
                if image is None:
                    self.send_json(404, {"status": "unavailable", "source": name})
                else:
                    self.send_bytes(200, image, "image/png")
                return
            if path in {"/", "/index.html"}:
                self._serve_static("index.html")
                return
            if path.startswith("/static/"):
                self._serve_static(path.removeprefix("/static/"))
                return
            self.send_json(404, {"status": "not_found"})

        def _serve_static(self, relative: str) -> None:
            requested = Path(relative)
            if requested.is_absolute() or ".." in requested.parts:
                self.send_json(403, {"status": "forbidden"})
                return
            target = web_root / requested
            if not target.is_file():
                self.send_json(404, {"status": "not_found"})
                return
            content_type = mimetypes.guess_type(target.name)[0] or "application/octet-stream"
            if content_type.startswith("text/") or content_type in {"application/javascript", "application/json"}:
                content_type += "; charset=utf-8"
            self.send_bytes(200, target.read_bytes(), content_type)

        def do_POST(self):
            path = urlparse(self.path).path
            if path != "/api/v1/commands":
                self.send_json(404, {"status": "not_found"})
                return
            try:
                size = int(self.headers.get("Content-Length", "0"))
                if size <= 0 or size > 16_384:
                    raise ValueError("invalid body size")
                payload = json.loads(self.rfile.read(size))
            except (ValueError, json.JSONDecodeError):
                self.send_json(400, {"status": "REJECTED", "reason": "invalid_json"})
                return
            auth = self.headers.get("Authorization", "")
            token = auth[7:] if auth.startswith("Bearer ") else ""
            status, response = gateway.submit(
                token, self.headers.get("Idempotency-Key", ""), payload
            )
            state.add_event(
                "operator",
                "操作请求已受理" if response["status"] == "ACCEPTED" else "操作请求被拒绝",
                str(response.get("reason") or response.get("dsl", {}).get("intent", "unknown")),
                severity="info" if response["status"] == "ACCEPTED" else "warning",
                source="hmi_gateway",
            )
            self.send_json(status, response)

        def log_message(self, _format, *_args):
            return

    return Handler


def _reference_paths(args) -> tuple[Path, Path, Path] | None:
    if args.registry_path and args.scene_path and args.mission_path:
        return Path(args.registry_path), Path(args.scene_path), Path(args.mission_path)
    # Product/live mode must never auto-load an oracle.  Reference overlays are
    # available only when all three offline paths are supplied explicitly.
    return None


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--operator-token")
    parser.add_argument("--ros", action="store_true", help="attach to live ROS 2 topics")
    parser.add_argument("--camera-topic", default="/camera/color/image_raw")
    parser.add_argument("--registry-path")
    parser.add_argument("--scene-path")
    parser.add_argument("--mission-path")
    parser.add_argument("--replay-csv")
    parser.add_argument("--replay-report")
    argv = sys.argv[1:]
    if "--ros-args" in argv:
        try:
            from rclpy.utilities import remove_ros_args

            argv = remove_ros_args(sys.argv)[1:]
        except (ImportError, ModuleNotFoundError):
            argv = argv[: argv.index("--ros-args")]
    args = parser.parse_args(argv)

    state = VisualizationState()
    paths = _reference_paths(args)
    if paths:
        state.reference = load_reference(*paths)
        state.truth = list(state.reference.get("truth_targets", []))
    if args.replay_csv:
        try:
            state.set_replay(load_real_replay(args.replay_csv, args.replay_report))
        except (OSError, ValueError, json.JSONDecodeError) as exc:
            state.add_event("replay", "历史回放不可用", str(exc), severity="warning", source="startup")

    adapter = None
    if args.ros:
        from .ros_adapter import RosAdapter

        adapter = RosAdapter(state, camera_topic=args.camera_topic)
        adapter.start()

    token = args.operator_token or secrets.token_urlsafe(32)
    gateway = CommandGateway(
        {token: "operator"}, dispatcher=adapter.dispatch if adapter else None
    )
    try:
        from ament_index_python.packages import get_package_share_directory

        web_root = Path(get_package_share_directory("sanitation_hmi")) / "web"
    except (ImportError, ModuleNotFoundError, LookupError):
        web_root = Path(__file__).resolve().parents[1] / "web"
    print(
        json.dumps(
            {
                "url": f"http://{args.host}:{args.port}",
                "token": token,
                "ros": bool(adapter),
                "replay": state.replay is not None,
            },
            ensure_ascii=False,
        ),
        flush=True,
    )
    server = ThreadingHTTPServer(
        (args.host, args.port), build_handler(gateway, web_root, state)
    )
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        if adapter:
            adapter.stop()


if __name__ == "__main__":
    main()
