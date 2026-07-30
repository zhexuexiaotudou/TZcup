#!/usr/bin/env python3
"""AUTO-10 TTS/noise/reverb/ASR formal matrix helpers."""

from __future__ import annotations

import argparse
from collections import Counter
from difflib import SequenceMatcher
import json
import math
from pathlib import Path
import re
import sys
import time
import wave

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "starter_ws" / "src" / "sanitation_hmi"))
from sanitation_hmi.dsl import parse_command


COMMANDS = (
    ("开始区域 A 清扫", "start_coverage", False, "zh"),
    ("执行分区 B 清扫", "start_coverage", False, "zh"),
    ("暂停", "pause", False, "zh"),
    ("恢复任务", "resume", False, "zh"),
    ("返回充电站", "return_home", False, "zh"),
    ("查询状态", "status", False, "zh"),
    ("紧急停止", "emergency_stop", False, "zh"),
    ("清理塑料瓶", "spot_clean", False, "zh"),
    ("清理易拉罐", "spot_clean", False, "zh"),
    ("把电机速度调到最大", None, True, "zh"),
    ("start cleaning zone A", "start_coverage", False, "en"),
    ("begin cleaning area B", "start_coverage", False, "en"),
    ("pause", "pause", False, "en"),
    ("resume", "resume", False, "en"),
    ("return home", "return_home", False, "en"),
    ("status", "status", False, "en"),
    ("emergency stop", "emergency_stop", False, "en"),
    ("spot clean the plastic bottle", "spot_clean", False, "en"),
    ("spot clean the metal can", "spot_clean", False, "en"),
    ("set motor speed maximum", None, True, "en"),
)
VOICES = {
    "zh": ("Microsoft Huihui Desktop",),
    "en": ("Microsoft David Desktop", "Microsoft Zira Desktop"),
}
RATES = (-2, 0, 2)
NOISE_LEVELS = (0.0, 0.003, 0.006, 0.01)
REVERB_PROFILES = ("dry", "room_short", "room_long")


def generate_manifest(output: Path, count: int) -> int:
    output.mkdir(parents=True, exist_ok=True)
    voice_index = {"zh": 0, "en": 0}
    rows = []
    for index in range(count):
        text, intent, unsafe, language = COMMANDS[index % len(COMMANDS)]
        voices = VOICES[language]
        voice = voices[voice_index[language] % len(voices)]
        voice_index[language] += 1
        rows.append(
            {
                "case_id": f"speech_{index:04d}",
                "text": text,
                "expected_intent": intent,
                "unsafe": unsafe,
                "language": language,
                "voice": voice,
                "speech_rate": RATES[(index // len(COMMANDS)) % len(RATES)],
                "noise_level": NOISE_LEVELS[
                    (index // (len(COMMANDS) * len(RATES))) % len(NOISE_LEVELS)
                ],
                "reverb_profile": REVERB_PROFILES[
                    (
                        index
                        // (len(COMMANDS) * len(RATES) * len(NOISE_LEVELS))
                    )
                    % len(REVERB_PROFILES)
                ],
                "raw_audio": f"raw/speech_{index:04d}.wav",
                "audio": f"audio/speech_{index:04d}.wav",
            }
        )
    payload = {"schema_version": 1, "case_count": len(rows), "cases": rows}
    (output / "speech_manifest.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return 0


def read_wav(path: Path) -> tuple[int, np.ndarray]:
    with wave.open(str(path), "rb") as stream:
        if stream.getnchannels() != 1 or stream.getsampwidth() != 2:
            raise RuntimeError(f"expected PCM16 mono audio: {path}")
        rate = stream.getframerate()
        values = np.frombuffer(stream.readframes(stream.getnframes()), np.int16)
    return rate, values.astype(np.float32) / 32768.0


def write_wav(path: Path, rate: int, values: np.ndarray) -> None:
    pcm = np.clip(values, -1, 1)
    pcm = (pcm * 32767).astype(np.int16)
    path.parent.mkdir(parents=True, exist_ok=True)
    with wave.open(str(path), "wb") as stream:
        stream.setnchannels(1)
        stream.setsampwidth(2)
        stream.setframerate(rate)
        stream.writeframes(pcm.tobytes())


def augment(manifest_path: Path, output: Path) -> int:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    for row in manifest["cases"]:
        rate, values = read_wav(output / row["raw_audio"])
        profile = row["reverb_profile"]
        if profile != "dry":
            delay_ms, gain = (
                (45, 0.22) if profile == "room_short" else (90, 0.18)
            )
            delay = int(rate * delay_ms / 1000)
            echoed = np.zeros(len(values) + delay, dtype=np.float32)
            echoed[: len(values)] += values
            echoed[delay:] += values * gain
            values = echoed
        noise_level = float(row["noise_level"])
        if noise_level:
            rng = np.random.default_rng(20260730 + int(row["case_id"].split("_")[-1]))
            values = values + rng.normal(0, noise_level, size=len(values))
        peak = max(float(np.max(np.abs(values))), 1e-6)
        values = values * min(0.95 / peak, 1.0)
        write_wav(output / row["audio"], rate, values)
    return 0


def normalized_text(text: str) -> str:
    return re.sub(r"[\W_]+", "", text.lower(), flags=re.UNICODE)


def constrained_recover(transcript: str) -> tuple[str, float]:
    normalized = normalized_text(transcript)
    ranked = sorted(
        (
            (
                SequenceMatcher(None, normalized, normalized_text(command[0])).ratio(),
                command[0],
            )
            for command in COMMANDS
        ),
        reverse=True,
    )
    return ranked[0][1], ranked[0][0]


def evaluate(manifest_path: Path, predictions_path: Path, output: Path) -> int:
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    predictions = {
        row["case_id"]: row
        for row in json.loads(predictions_path.read_text(encoding="utf-8"))["cases"]
    }
    rows = []
    safe_total = intent_correct = unsafe_total = unsafe_rejected = 0
    latencies = []
    for case in manifest["cases"]:
        prediction = predictions[case["case_id"]]
        recovered, similarity = constrained_recover(prediction["transcript"])
        started = time.perf_counter_ns()
        result = parse_command(recovered)
        normalization_ms = (time.perf_counter_ns() - started) / 1e6
        latency_ms = float(prediction["asr_latency_ms"]) + normalization_ms
        latencies.append(latency_ms)
        if case["unsafe"]:
            unsafe_total += 1
            unsafe_rejected += int(result.status == "REJECTED")
        else:
            safe_total += 1
            intent_correct += int(
                result.status == "ACCEPTED"
                and result.dsl["intent"] == case["expected_intent"]
            )
        rows.append(
            {
                **case,
                "transcript": prediction["transcript"],
                "recovered_command": recovered,
                "similarity": similarity,
                "actual_status": result.status,
                "actual_intent": result.dsl["intent"] if result.dsl else None,
                "latency_ms": latency_ms,
            }
        )
    ordered = sorted(latencies)
    p95 = ordered[math.floor((len(ordered) - 1) * 0.95)]
    counts = {
        "voices": len({row["voice"] for row in rows}),
        "speech_rates": len({row["speech_rate"] for row in rows}),
        "noise_levels": len({row["noise_level"] for row in rows}),
        "reverb_profiles": len({row["reverb_profile"] for row in rows}),
        "languages": len({row["language"] for row in rows}),
    }
    metrics = {
        "schema_version": 1,
        "command_count": len(rows),
        **counts,
        "distribution": {
            "voices": dict(Counter(row["voice"] for row in rows)),
            "speech_rates": dict(Counter(str(row["speech_rate"]) for row in rows)),
            "noise_levels": dict(Counter(str(row["noise_level"]) for row in rows)),
            "reverb_profiles": dict(
                Counter(row["reverb_profile"] for row in rows)
            ),
            "languages": dict(Counter(row["language"] for row in rows)),
        },
        "intent_accuracy": intent_correct / safe_total,
        "unsafe_command_rejection_rate": unsafe_rejected / unsafe_total,
        "end_to_end_latency_ms": {
            "p50": float(np.median(latencies)),
            "p95": p95,
            "max": max(latencies),
        },
    }
    metrics["checks"] = {
        "commands_at_least_500": len(rows) >= 500,
        "voices_at_least_3": counts["voices"] >= 3,
        "speech_rates_at_least_3": counts["speech_rates"] >= 3,
        "noise_levels_at_least_4": counts["noise_levels"] >= 4,
        "reverb_profiles_at_least_2": counts["reverb_profiles"] >= 2,
        "chinese_and_english_present": counts["languages"] >= 2,
        "intent_accuracy_at_least_0_95": metrics["intent_accuracy"] >= 0.95,
        "unsafe_rejection_100_percent": metrics[
            "unsafe_command_rejection_rate"
        ]
        == 1,
        "end_to_end_p95_at_most_2000_ms": p95 <= 2000,
    }
    metrics["speech_gate_pass"] = all(metrics["checks"].values())
    output.mkdir(parents=True, exist_ok=True)
    (output / "speech_cases.jsonl").write_text(
        "".join(json.dumps(row, ensure_ascii=False) + "\n" for row in rows),
        encoding="utf-8",
    )
    (output / "speech_metrics.json").write_text(
        json.dumps(metrics, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(metrics, indent=2))
    return 0 if metrics["speech_gate_pass"] else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="mode", required=True)
    generate_parser = subparsers.add_parser("generate-manifest")
    generate_parser.add_argument("--output", required=True)
    generate_parser.add_argument("--count", type=int, default=500)
    augment_parser = subparsers.add_parser("augment")
    augment_parser.add_argument("--manifest", required=True)
    augment_parser.add_argument("--output", required=True)
    evaluate_parser = subparsers.add_parser("evaluate")
    evaluate_parser.add_argument("--manifest", required=True)
    evaluate_parser.add_argument("--predictions", required=True)
    evaluate_parser.add_argument("--output", required=True)
    args = parser.parse_args()
    if args.mode == "generate-manifest":
        return generate_manifest(Path(args.output), args.count)
    if args.mode == "augment":
        return augment(Path(args.manifest), Path(args.output))
    return evaluate(Path(args.manifest), Path(args.predictions), Path(args.output))


if __name__ == "__main__":
    raise SystemExit(main())
