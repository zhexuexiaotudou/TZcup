#!/usr/bin/env python3
"""Transcribe an AUTO-10 speech manifest with faster-whisper."""

import argparse
import json
from pathlib import Path
import time

from faster_whisper import WhisperModel


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--data-root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--model", default="small")
    args = parser.parse_args()
    manifest = json.loads(Path(args.manifest).read_text(encoding="utf-8"))
    model = WhisperModel(args.model, device="cuda", compute_type="float16")
    rows = []
    for index, case in enumerate(manifest["cases"], start=1):
        started = time.perf_counter()
        segments, _ = model.transcribe(
            str(Path(args.data_root) / case["audio"]),
            language=case["language"],
            beam_size=3,
            vad_filter=True,
            condition_on_previous_text=False,
        )
        transcript = "".join(segment.text for segment in segments).strip()
        rows.append(
            {
                "case_id": case["case_id"],
                "transcript": transcript,
                "asr_latency_ms": (time.perf_counter() - started) * 1000,
            }
        )
        if index % 25 == 0:
            print(f"transcribed {index}/{len(manifest['cases'])}", flush=True)
    Path(args.output).write_text(
        json.dumps({"schema_version": 1, "cases": rows}, ensure_ascii=False, indent=2)
        + "\n",
        encoding="utf-8",
    )


if __name__ == "__main__":
    main()
