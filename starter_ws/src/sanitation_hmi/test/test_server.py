import sys

from sanitation_hmi import server


def test_main_accepts_ros_launch_arguments(monkeypatch):
    captured = {}

    class Adapter:
        def __init__(self, state, **_kwargs):
            self.state = state

        def start(self):
            captured["adapter_started"] = True

        def stop(self):
            pass

        def dispatch(self, _dsl):
            return {"accepted": False, "dispatched": False, "reason": "test"}

    class HttpServer:
        def __init__(self, address, _handler):
            captured["address"] = address

        def serve_forever(self):
            raise KeyboardInterrupt

        def server_close(self):
            captured["closed"] = True

    monkeypatch.setattr(server, "_reference_paths", lambda _args: None)
    monkeypatch.setattr(server, "ThreadingHTTPServer", HttpServer)
    monkeypatch.setattr("sanitation_hmi.ros_adapter.RosAdapter", Adapter)
    monkeypatch.setattr(
        sys,
        "argv",
        [
            "sanitation_hmi_server",
            "--ros",
            "--port",
            "8765",
            "--operator-token",
            "test",
            "--ros-args",
            "-r",
            "__node:=sanitation_human_visualization",
        ],
    )
    server.main()
    assert captured == {
        "adapter_started": True,
        "address": ("127.0.0.1", 8765),
        "closed": True,
    }
