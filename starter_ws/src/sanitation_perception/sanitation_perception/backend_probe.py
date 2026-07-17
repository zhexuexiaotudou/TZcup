from __future__ import annotations

import argparse
import json

from .backends import BackendUnavailable, select_backend


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--backend", required=True)
    parser.add_argument("--model")
    parser.add_argument("--expect-failure", action="store_true")
    args = parser.parse_args()
    try:
        selection = select_backend(args.backend, model_path=args.model)
    except BackendUnavailable as exc:
        print(json.dumps({"backend": args.backend, "available": False, "reason": str(exc)}))
        return 0 if args.expect_failure else 2
    print(json.dumps({"backend": selection.active, "available": True, "detail": selection.detail}))
    return 2 if args.expect_failure else 0


if __name__ == "__main__":
    raise SystemExit(main())
