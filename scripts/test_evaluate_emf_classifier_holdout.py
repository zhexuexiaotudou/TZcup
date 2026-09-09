#!/usr/bin/env python3

from __future__ import annotations

import contextlib
import importlib.util
import io
import json
import tempfile
import unittest
from pathlib import Path

SCRIPT = Path(__file__).with_name("evaluate_emf_classifier_holdout.py")
SPEC = importlib.util.spec_from_file_location("evaluate_emf_classifier_holdout", SCRIPT)
assert SPEC and SPEC.loader
MOD = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MOD)


class HoldoutEvaluatorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.root = Path(self.temporary.name)
        self.manifest_path = self.root / "CLASSIFIER_HOLDOUT_GT_MANIFEST.json"
        self.records = self._records()
        payload = {
            "schema_version": "emfj6v3.classifier_holdout_gt.v1",
            "protocol_id": "EMFJ6V3",
            "stage": "CLASSIFIER_HOLDOUT_OFFLINE_GT_DEVELOPMENT",
            "source_split": "G10_HOLDOUT",
            "g10_domain_manifest_sha256": MOD.DOMAIN_MANIFEST_SHA256,
            "holdout_world_ids": sorted(MOD.HOLDOUT_WORLDS),
            "counts": {
                "background_or_unknown": 1,
                "metal_can": 60,
                "paper_litter": 60,
                "plastic_bottle": 60,
            },
            "negative_only_frame_count": 1,
            "records": self.records,
            "identity_lock_sha256": MOD.canonical_sha256(self.records),
            "offline_gt_development_only": True,
            "production_runtime_gt_forbidden": True,
            "training_performed": False,
            "threshold_selected": False,
            "threshold_frozen": False,
            "formal_product_evidence": False,
            "pass": True,
        }
        payload["canonical_manifest_sha256"] = MOD.canonical_sha256(payload)
        self._write(self.manifest_path, payload)
        self.manifest_sha = MOD.sha256(self.manifest_path)
        self.manifest_source = {
            "file_sha256": self.manifest_sha,
            "canonical_manifest_sha256": payload["canonical_manifest_sha256"],
            "identity_lock_sha256": payload["identity_lock_sha256"],
            "g10_domain_manifest_sha256": MOD.DOMAIN_MANIFEST_SHA256,
            "holdout_world_ids": sorted(MOD.HOLDOUT_WORLDS),
            "counts": payload["counts"],
        }

    def tearDown(self) -> None:
        self.temporary.cleanup()

    @staticmethod
    def _write(path: Path, payload: dict) -> None:
        path.write_text(
            json.dumps(payload, ensure_ascii=False, sort_keys=True, indent=2) + "\n",
            encoding="utf-8",
        )

    @staticmethod
    def _records() -> list[dict]:
        records = []
        world_by_class = {
            "plastic_bottle": "g10v15_val_w01_07_service_road",
            "metal_can": "g10v15_val_w02_08_mixed_curb_vegetation",
            "paper_litter": "g10v15_val_w03_09_light_paver_pedestrian",
        }
        for class_name in MOD.TARGET_CLASSES:
            for index in range(60):
                record_id = f"emf-holdout-{class_name}-{index:03d}"
                records.append(
                    {
                        "record_id": record_id,
                        "class_name": class_name,
                        "crop_sha256": MOD.canonical_sha256([record_id, "crop"]),
                        "source_identity_sha256": MOD.canonical_sha256([record_id, "source"]),
                        "source_identity": {
                            "source_split": "G10_HOLDOUT",
                            "world_id": world_by_class[class_name],
                            "negative_only": False,
                        },
                        "offline_gt_development_only": True,
                        "production_runtime_eligible": False,
                    }
                )
        record_id = "emf-holdout-background-000"
        records.append(
            {
                "record_id": record_id,
                "class_name": "background_or_unknown",
                "crop_sha256": MOD.canonical_sha256([record_id, "crop"]),
                "source_identity_sha256": MOD.canonical_sha256([record_id, "source"]),
                "source_identity": {
                    "source_split": "G10_HOLDOUT",
                    "world_id": "g10v15_val_w01_07_service_road",
                    "negative_only": True,
                },
                "offline_gt_development_only": True,
                "production_runtime_eligible": False,
            }
        )
        return sorted(records, key=lambda row: row["record_id"])

    def _probabilities(self, model_id: str, class_name: str, flood: bool) -> dict[str, float]:
        order = MOD.MODEL_CONTRACTS[model_id]["class_order"]
        values = {name: 0.0 for name in order}
        if class_name == "background_or_unknown":
            if flood:
                values["plastic"] = 0.80
                values[order[0]] += 0.20
            else:
                values["metal"] = 0.01
                values["paper"] = 0.01
                values["plastic"] = 0.01
                values[order[0]] += 0.97
        else:
            inverse = {product: source for source, product in MOD.EXPECTED_CLASS_MAPPING.items()}
            values[inverse[class_name]] = 0.80
            for target in ("metal", "paper", "plastic"):
                if target != inverse[class_name]:
                    values[target] = 0.02
            values[order[0]] += 0.16
        return {name: values[name] for name in order}

    def _inference(self, model_id: str, *, flood: bool = False) -> tuple[Path, str]:
        contract = MOD.MODEL_CONTRACTS[model_id]
        rows = [
            {
                "record_id": record["record_id"],
                "crop_sha256": record["crop_sha256"],
                "source_identity_sha256": record["source_identity_sha256"],
                "probabilities": self._probabilities(model_id, record["class_name"], flood),
            }
            for record in self.records
        ]
        payload = {
            "schema_version": "emfj6v3.classifier_holdout_raw_inference.v1",
            "protocol_id": "EMFJ6V3",
            "stage": "CLASSIFIER_HOLDOUT_RAW_INFERENCE",
            "source_split": "G10_HOLDOUT",
            "model_id": model_id,
            "model_sha256": contract["model_sha256"],
            "artifact": {"sha256": contract["artifact_sha256"]},
            "class_order": contract["class_order"],
            "class_mapping": MOD.EXPECTED_CLASS_MAPPING,
            "source_manifest": self.manifest_source,
            "rows": rows,
            "training_performed": False,
            "raw_probabilities_only": True,
            "threshold_applied": False,
            "offline_gt_development_only": True,
            "production_runtime_eligible": False,
        }
        path = self.root / f"{model_id}.json"
        self._write(path, payload)
        return path, MOD.sha256(path)

    def test_fixed_grid_selects_one_deterministic_development_threshold(self) -> None:
        c1 = self._inference(MOD.MODEL_ORDER[0])
        c3 = self._inference(MOD.MODEL_ORDER[1])
        report = MOD.evaluate_files(self.manifest_path, self.manifest_sha, [c3, c1])

        self.assertEqual(len(report["candidates"]), 50)
        self.assertEqual(
            report["threshold_selection"]["candidate_id"],
            "c1_wastewise_yolov8n_cls:target_min=0.40:unknown_max=0.50",
        )
        self.assertTrue(report["threshold_selection"]["selected"])
        self.assertTrue(report["threshold_selection"]["frozen"])
        self.assertEqual(
            report["threshold_selection"]["scope"],
            "classifier_development_threshold_only",
        )
        self.assertFalse(report["threshold_selection"]["functional_candidate"])
        self.assertFalse(report["threshold_selection"]["product_candidate"])
        self.assertFalse(report["training"])
        self.assertFalse(report["track_gate_evaluated"])
        self.assertFalse(report["clean_now_gate_evaluated"])
        self.assertFalse(report["runtime_stability_gate_evaluated"])
        self.assertTrue(
            report["selection_rule"]["preregistered_before_new_holdout_raw_read"]
        )
        self.assertEqual(
            report["selection_rule"]["background_rule_source"]["check_id"], "E-07"
        )
        self.assertEqual(
            report["selection_rule"]["background_rule_source"]["threshold"], 0.995
        )
        selected = [row for row in report["candidates"] if row["selected"]]
        self.assertEqual(len(selected), 1)
        self.assertEqual(set(selected[0]["metrics"]["per_target"]), set(MOD.TARGET_CLASSES))
        self.assertEqual(
            selected[0]["metrics"]["independent_negative_only_background"]["support"], 1
        )

    def test_no_eligible_point_does_not_select_or_freeze(self) -> None:
        c1 = self._inference(MOD.MODEL_ORDER[0], flood=True)
        c3 = self._inference(MOD.MODEL_ORDER[1], flood=True)
        report = MOD.evaluate_files(self.manifest_path, self.manifest_sha, [c1, c3])
        self.assertFalse(report["threshold_selection"]["selected"])
        self.assertFalse(report["threshold_selection"]["frozen"])
        self.assertIsNone(report["threshold_selection"]["candidate_id"])
        self.assertTrue(all(not row["eligibility"]["eligible"] for row in report["candidates"]))

    def test_macro_f1_0_98_is_not_an_a5_eligibility_requirement(self) -> None:
        inputs = []
        for model_id in MOD.MODEL_ORDER:
            path, _ = self._inference(model_id)
            payload = json.loads(path.read_text(encoding="utf-8"))
            plastic_rows = [
                row for row in payload["rows"] if "plastic_bottle" in row["record_id"]
            ]
            for row in plastic_rows[1:]:
                order = payload["class_order"]
                probabilities = {name: 0.0 for name in order}
                probabilities["plastic"] = 0.01
                probabilities[order[0]] += 0.99
                row["probabilities"] = probabilities
            self._write(path, payload)
            inputs.append((path, MOD.sha256(path)))
        report = MOD.evaluate_files(self.manifest_path, self.manifest_sha, inputs)
        chosen = next(row for row in report["candidates"] if row["selected"])
        self.assertLess(chosen["metrics"]["macro_f1"], 0.98)
        self.assertTrue(chosen["eligibility"]["each_target_true_positive_gt_zero"])
        self.assertTrue(chosen["eligibility"]["background_specificity_gte_0_995"])

    def test_raw_evidence_sha_lock_is_mandatory(self) -> None:
        c1_path, _ = self._inference(MOD.MODEL_ORDER[0])
        c3 = self._inference(MOD.MODEL_ORDER[1])
        with self.assertRaisesRegex(MOD.EvaluationError, "raw evidence SHA-256"):
            MOD.evaluate_files(self.manifest_path, self.manifest_sha, [(c1_path, "0" * 64), c3])

    def test_rejects_manifest_identity_or_counts_drift(self) -> None:
        c1 = self._inference(MOD.MODEL_ORDER[0])
        c3_path, _ = self._inference(MOD.MODEL_ORDER[1])
        payload = json.loads(c3_path.read_text(encoding="utf-8"))
        payload["source_manifest"]["counts"]["background_or_unknown"] = 2
        self._write(c3_path, payload)
        with self.assertRaisesRegex(MOD.EvaluationError, "identity/counts"):
            MOD.evaluate_files(
                self.manifest_path,
                self.manifest_sha,
                [c1, (c3_path, MOD.sha256(c3_path))],
            )

    def test_rejects_forbidden_marker_before_evaluation(self) -> None:
        c1_path, _ = self._inference(MOD.MODEL_ORDER[0])
        payload = json.loads(c1_path.read_text(encoding="utf-8"))
        payload["note"] = "DEV_VAL must never be used"
        self._write(c1_path, payload)
        with self.assertRaisesRegex(MOD.EvaluationError, "forbidden data marker"):
            MOD.load_inference(c1_path, MOD.sha256(c1_path), MOD.load_manifest(self.manifest_path, self.manifest_sha))

    def test_cli_exposes_no_grid_override(self) -> None:
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit):
            MOD.main(["--target-minimum", "0.99"])


if __name__ == "__main__":
    unittest.main()
