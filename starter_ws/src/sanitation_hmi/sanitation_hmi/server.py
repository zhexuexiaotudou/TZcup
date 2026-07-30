"""Small standard-library HTTP server for the local APP interface."""

from __future__ import annotations

import argparse
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets

from .gateway import CommandGateway


def build_handler(gateway: CommandGateway, web_root: Path):
    class Handler(BaseHTTPRequestHandler):
        server_version = "TZcupHMI/1"

        def send_json(self, status: int, payload: dict) -> None:
            body = json.dumps(payload, ensure_ascii=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", "application/json; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/healthz":
                self.send_json(200, {"status": "ok"})
                return
            if self.path in {"/", "/index.html"}:
                body = (web_root / "index.html").read_bytes()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
                return
            self.send_json(404, {"status": "not_found"})

        def do_POST(self):
            if self.path != "/api/v1/commands":
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
            self.send_json(status, response)

        def log_message(self, _format, *_args):
            return

    return Handler


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--operator-token")
    args = parser.parse_args()
    token = args.operator_token or secrets.token_urlsafe(32)
    gateway = CommandGateway({token: "operator"})
    web_root = Path(__file__).resolve().parents[1] / "web"
    print(json.dumps({"url": f"http://{args.host}:{args.port}", "token": token}))
    ThreadingHTTPServer(
        (args.host, args.port), build_handler(gateway, web_root)
    ).serve_forever()


if __name__ == "__main__":
    main()
