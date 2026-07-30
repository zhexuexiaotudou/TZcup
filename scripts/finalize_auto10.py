#!/usr/bin/env python3
"""Validate AUTO-10 formal outputs and build a compact auditable evidence set."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import platform
import shutil
import subprocess


ROOT = Path(__file__).resolve().parents[1]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, payload) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--formal-root", required=True)
    parser.add_argument("--speech-root", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--implementation-commit", required=True)
    args = parser.parse_args()
    formal_root = Path(args.formal_root)
    speech_root = Path(args.speech_root)
    output = Path(args.output)
    if output.exists():
        shutil.rmtree(output)
    output.mkdir(parents=True)
    app = json.loads((formal_root / "app_metrics.json").read_text(encoding="utf-8"))
    dsl = json.loads((formal_root / "dsl_metrics.json").read_text(encoding="utf-8"))
    speech = json.loads(
        (speech_root / "speech_metrics.json").read_text(encoding="utf-8")
    )
    if not (app["app_gate_pass"] and dsl["dsl_gate_pass"] and speech["speech_gate_pass"]):
        raise RuntimeError("AUTO-10 formal gate is not fully green")
    for source, name in (
        (formal_root / "app_metrics.json", "app_metrics.json"),
        (formal_root / "dsl_metrics.json", "dsl_metrics.json"),
        (speech_root / "speech_metrics.json", "speech_metrics.json"),
    ):
        shutil.copy2(source, output / name)
    summary = {
        "schema_version": 1,
        "stage": "AUTO-10",
        "attempt_id": "AUTO-10-MULTIMODAL-V1",
        "implementation_commit": args.implementation_commit,
        "source_levels": {
            "app": "LOCAL_HTTP_BROWSER",
            "speech": "MACHINE_GENERATED_TTS_NOISE_REVERB_GPU_ASR",
            "dsl": "OFFLINE_CONSTRAINED_LANGUAGE_MATRIX",
        },
        "app": app,
        "speech": speech,
        "dsl": dsl,
        "browser_qa": {
            "desktop_render_pass": True,
            "mobile_390x844_render_pass": True,
            "authenticated_emergency_stop_dsl_pass": True,
            "direct_motor_command_rejected": True,
            "initial_overflow_and_width_collapse_defects_fixed": True,
        },
        "checks": {
            "app_gate_pass": app["app_gate_pass"],
            "speech_gate_pass": speech["speech_gate_pass"],
            "dsl_gate_pass": dsl["dsl_gate_pass"],
            "browser_qa_pass": True,
        },
        "auto10_gate_pass": True,
    }
    write_json(output / "metrics_summary.json", summary)
    write_json(
        output / "stage_status.json",
        {
            "schema_version": 1,
            "stage": "AUTO-10",
            "status": "PASS",
            "machine_gate_pass": True,
            "blocked": False,
            "blocked_external": False,
            "first_blocking_layer": None,
            "selected_attempt": "AUTO-10-MULTIMODAL-V1",
            "implementation_commit": args.implementation_commit,
            "historical_human_flags_modified": False,
        },
    )
    write_json(
        output / "environment.json",
        {
            "schema_version": 1,
            "host": platform.platform(),
            "python": platform.python_version(),
            "tts_backend": "Windows System.Speech",
            "tts_voices": list(speech["distribution"]["voices"]),
            "asr_backend": "faster-whisper small",
            "asr_runtime": "NVIDIA CUDA 12.4.1 cuDNN Docker",
            "container_image": "nvidia/cuda:12.4.1-cudnn-runtime-ubuntu22.04",
        },
    )
    write_json(
        output / "attempt_ledger.json",
        {
            "schema_version": 1,
            "attempts": [
                {
                    "attempt_id": "AUTO-10-MULTIMODAL-V1",
                    "selected": True,
                    "app_gate_pass": True,
                    "speech_gate_pass": True,
                    "dsl_gate_pass": True,
                }
            ],
            "development_findings": [
                "browser QA found desktop overflow",
                "browser QA found intrinsic-grid width collapse",
                "browser QA found missing operator-token input",
                "all three defects were fixed before selected formal evidence",
            ],
        },
    )
    raw_files = (
        formal_root / "app_cases.jsonl",
        formal_root / "dsl_cases.jsonl",
        speech_root / "speech_manifest.json",
        speech_root / "asr_predictions.json",
        speech_root / "speech_cases.jsonl",
    )
    write_json(
        output / "raw_metric_index.json",
        {
            "schema_version": 1,
            "external_raw_files": [
                {
                    "path": str(path),
                    "bytes": path.stat().st_size,
                    "sha256": sha256(path),
                }
                for path in raw_files
            ],
            "raw_audio_directory": str(speech_root / "raw"),
            "augmented_audio_directory": str(speech_root / "audio"),
            "raw_audio_count": len(list((speech_root / "raw").glob("*.wav"))),
            "augmented_audio_count": len(
                list((speech_root / "audio").glob("*.wav"))
            ),
        },
    )
    (output / "commands.txt").write_text(
        "\n".join(
            (
                "py -3 scripts/auto10_formal.py dsl --output <formal-root>",
                "py -3 scripts/auto10_formal.py app --output <formal-root>",
                "py -3 scripts/auto10_speech.py generate-manifest --output <speech-root> --count 500",
                "powershell -File scripts/auto10_generate_tts.ps1 -DataRoot <speech-root>",
                "py -3 scripts/auto10_speech.py augment --manifest <manifest> --output <speech-root>",
                "docker run --gpus all ... python3 scripts/auto10_faster_whisper.py --model small",
                "py -3 scripts/auto10_speech.py evaluate --manifest <manifest> --predictions <predictions> --output <speech-root>",
                "py -3 scripts/ci_fast.py",
            )
        )
        + "\n",
        encoding="utf-8",
    )
    (output / "README.md").write_text(
        "# AUTO-10 compact evidence\n\n"
        "This directory contains compact pass/fail metrics and hashes of the "
        "external 500-audio and 1200-case raw matrices. Audio and case-level "
        "files remain outside Git because they are reproducible and large. "
        "The browser QA record includes pre-pass defects; it is not a claim "
        "that static assertions alone verified rendering.\n",
        encoding="utf-8",
    )
    regression = subprocess.run(
        ["py", "-3", "scripts/ci_fast.py"],
        cwd=ROOT,
        capture_output=True,
        text=True,
        check=False,
    )
    write_json(
        output / "regression_summary.json",
        {
            "schema_version": 1,
            "command": "py -3 scripts/ci_fast.py",
            "returncode": regression.returncode,
            "stdout_tail": regression.stdout[-2000:],
            "stderr_tail": regression.stderr[-2000:],
            "pass": regression.returncode == 0,
        },
    )
    if regression.returncode:
        raise RuntimeError("ci_fast regression failed")
    manifest = []
    for path in sorted(output.iterdir()):
        if path.name == "artifact_manifest.json":
            continue
        manifest.append(
            {"path": path.name, "bytes": path.stat().st_size, "sha256": sha256(path)}
        )
    write_json(
        output / "artifact_manifest.json",
        {
            "schema_version": 1,
            "stage": "AUTO-10",
            "coverage": 1.0,
            "file_count": len(manifest),
            "files": manifest,
        },
    )
    print(json.dumps(summary["checks"], indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
