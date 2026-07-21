import json
import random
from datetime import datetime, timedelta, timezone

import pytest

from sanitation_learning.human_review_handoff import CLASSES, _response_template, sha256_bytes, validate_completed_response


def completed_response(package):
    sample_ids = [f"opaque_{index}" for index in range(package["sample_count"])]
    response = _response_template(package["package_id"], sample_ids)
    start = datetime(2026, 7, 21, tzinfo=timezone.utc)
    response.update({
        "reviewer_pseudonym": "reviewer-blue",
        "package_sha256": package["sha256"],
        "started_at": start.isoformat(),
        "completed_at": (start + timedelta(minutes=20)).isoformat(),
        "independence_attestation": True,
        "truth_mapping_not_accessed": True,
    })
    rng = random.Random(7)
    for row in response["responses"]:
        row.update({
            "target_present": True,
            "class": rng.choice(CLASSES),
            "suitable_for_recognition": True,
            "self_occluded": False,
            "confidence_1_to_5": 4,
        })
    return response


def test_completed_response_contract_passes():
    ids = [f"opaque_{index}" for index in range(3)]
    package = {"package_id": "pkg-a", "sha256": "a" * 64, "sample_count": 3,
               "sample_ids_sha256": sha256_bytes(("\n".join(ids) + "\n").encode())}
    validate_completed_response(completed_response(package), package)


@pytest.mark.parametrize("field,value", [
    ("independence_attestation", False),
    ("truth_mapping_not_accessed", None),
    ("package_sha256", "wrong"),
])
def test_completed_response_fails_closed(field, value):
    ids = [f"opaque_{index}" for index in range(2)]
    package = {"package_id": "pkg-a", "sha256": "a" * 64, "sample_count": 2,
               "sample_ids_sha256": sha256_bytes(("\n".join(ids) + "\n").encode())}
    response = completed_response(package)
    response[field] = value
    with pytest.raises(ValueError):
        validate_completed_response(response, package)


def test_duplicate_or_illegal_sample_response_fails_closed():
    ids = [f"opaque_{index}" for index in range(2)]
    package = {"package_id": "pkg-a", "sha256": "a" * 64, "sample_count": 2,
               "sample_ids_sha256": sha256_bytes(("\n".join(ids) + "\n").encode())}
    response = completed_response(package)
    response["responses"][1]["sample_id"] = response["responses"][0]["sample_id"]
    with pytest.raises(ValueError):
        validate_completed_response(response, package)
    response = completed_response(package)
    response["responses"][0]["class"] = "unknown"
    with pytest.raises(ValueError):
        validate_completed_response(response, package)
