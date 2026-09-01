from __future__ import annotations

import importlib.util
from pathlib import Path
import sys

import numpy as np
import pytest


ROOT = Path(__file__).resolve().parents[1]
SCRIPT = ROOT / "scripts" / "scan_formal_vehicle_inertia_and_swept_volume.py"
SPEC = importlib.util.spec_from_file_location("formal_vehicle_inertia_sweep", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


def _obb(center: tuple[float, float, float]) -> object:
    return MODULE.CollisionObb(
        link="link",
        name="collision",
        center=np.asarray(center, dtype=float),
        axes=np.eye(3),
        half_size=np.asarray((0.5, 0.5, 0.5)),
        source="box",
    )


def test_obb_gate_is_fail_closed_for_material_overlap_and_clear_for_gap() -> None:
    assert MODULE._obb_material_overlap(_obb((0, 0, 0)), _obb((0.75, 0, 0)), 0.002) == 0.25
    assert MODULE._obb_material_overlap(_obb((0, 0, 0)), _obb((1.01, 0, 0)), 0.002) is None


def test_bounded_collision_evidence_matches_stable_descending_top_n() -> None:
    heap = []
    penetrations = (0.5, 0.2, 0.5, 0.7)
    for sequence, penetration in enumerate(penetrations):
        MODULE._push_bounded_worst_candidate(
            heap,
            {
                "sample": f"sample_{sequence}",
                "conservative_obb_penetration_m": penetration,
            },
            sequence,
            limit=2,
        )

    retained = MODULE._sorted_bounded_worst_candidates(heap)
    assert [item["sample"] for item in retained] == ["sample_3", "sample_0"]


def test_mesh_vertex_inside_box_is_an_exact_penetration_witness() -> None:
    mesh = MODULE.CollisionShape(
        link="arm",
        name="mesh",
        origin=np.eye(4),
        kind="mesh",
        parameters=(),
        vertices=np.asarray(((0.0, 0.0, 0.0), (0.6, 0.6, 0.6), (-0.6, -0.6, -0.6))),
        source="mesh.stl",
    )
    box = MODULE.CollisionShape(
        link="vehicle",
        name="box",
        origin=np.eye(4),
        kind="box",
        parameters=(1.0, 1.0, 1.0),
        vertices=None,
        source="box",
    )
    assert MODULE._mesh_vertices_strictly_inside_box(mesh, np.eye(4), box, np.eye(4), 0.002) == 1


def test_formal_model_uses_all_physical_inertials_and_real_collision_meshes() -> None:
    model = MODULE.Model(MODULE.DEFAULT_URDF)
    assert len(model.inertials) == len(model.links) - 2
    assert any(shape.kind == "mesh" for shape in model.shapes)
    assert all(shape.vertices is not None for shape in model.shapes if shape.kind == "mesh")


def test_required_arm_samples_cover_anchors_limits_and_halton_space() -> None:
    model = MODULE.Model(MODULE.DEFAULT_URDF)
    samples = MODULE.arm_samples(model, 64)
    labels = {label for label, _ in samples}
    assert {"transport", "pregrasp", "pick", "deposit"} <= labels
    assert {f"limit_corner_{index:02d}" for index in range(64)} <= labels
    assert "halton_00064" in labels


def test_final_wastewater_capacity_respects_exact_expanded_urdf_payload_budget() -> None:
    model = MODULE.Model(MODULE.DEFAULT_URDF)
    transforms = model.transforms({})
    empty_mass, _ = MODULE._mass_moment(
        model, transforms, MODULE._load_override("empty")
    )
    combined_mass, _ = MODULE._mass_moment(
        model, transforms, MODULE._load_override("max_combined")
    )
    fixed_payload = empty_mass - MODULE.A300_CURB_MASS_KG
    maximum_payload = combined_mass - MODULE.A300_CURB_MASS_KG

    assert fixed_payload > 77.705  # final detailed vehicle exceeds the pre-URDF allowance
    assert MODULE.WASTEWATER_CAPACITY_KG == 8.30
    assert maximum_payload <= MODULE.A300_PAYLOAD_DESIGN_LIMIT_KG
    assert MODULE.A300_PAYLOAD_DESIGN_LIMIT_KG - maximum_payload > 0.0
    assert MODULE.A300_RATED_PAYLOAD_KG - maximum_payload == pytest.approx(
        10.182416727
    )
