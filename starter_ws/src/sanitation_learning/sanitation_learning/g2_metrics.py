from __future__ import annotations

from collections import defaultdict
import math

import numpy as np


def _percentiles(values: list[float]) -> dict:
    if not values:
        return {"count": 0, "p10": None, "p50": None, "p90": None}
    return {
        "count": len(values),
        "p10": float(np.percentile(values, 10)),
        "p50": float(np.percentile(values, 50)),
        "p90": float(np.percentile(values, 90)),
    }


def measure_instances(
    semantic: np.ndarray,
    instances: np.ndarray,
    depth_m: np.ndarray,
    expected_objects: list[dict],
) -> list[dict]:
    """Measure real instance masks; expected but zero-pixel objects are not_visible."""
    if semantic.shape != instances.shape or semantic.shape != depth_m.shape:
        raise ValueError("semantic, instance, and depth shapes must match")
    expected = {int(item["runtime_instance_id"]): item for item in expected_objects}
    observed_ids = {int(value) for value in np.unique(instances) if int(value) != 0}
    rows = []
    for instance_id in sorted(set(expected) | observed_ids):
        mask = instances == instance_id
        area = int(mask.sum())
        item = expected.get(instance_id, {})
        if area == 0:
            rows.append({
                "instance_id": instance_id,
                "class_id": item.get("class_id"),
                "visibility": "not_visible",
                "mask_area_px": 0,
                "bbox_xywh_px": None,
                "bbox_shortest_side_px": 0,
                "distance_m": None,
                "occlusion_ratio": None,
            })
            continue
        ys, xs = np.nonzero(mask)
        width = int(xs.max() - xs.min() + 1)
        height = int(ys.max() - ys.min() + 1)
        finite_depth = depth_m[mask]
        finite_depth = finite_depth[np.isfinite(finite_depth) & (finite_depth > 0)]
        reference_area = item.get("unoccluded_reference_area_px")
        occlusion = None
        if reference_area is not None and float(reference_area) > 0:
            occlusion = max(0.0, min(1.0, 1.0 - area / float(reference_area)))
        labels = semantic[mask].astype(np.int64)
        semantic_label = int(np.bincount(labels).argmax()) if labels.size else None
        rows.append({
            "instance_id": instance_id,
            "class_id": item.get("class_id"),
            "semantic_label": semantic_label,
            "visibility": "visible",
            "mask_area_px": area,
            "bbox_xywh_px": [int(xs.min()), int(ys.min()), width, height],
            "bbox_shortest_side_px": min(width, height),
            "distance_m": float(np.median(finite_depth)) if finite_depth.size else None,
            "occlusion_ratio": occlusion,
        })
    return rows


def summarize_instance_rows(rows: list[dict]) -> dict:
    grouped: dict[str, dict[str, list[float]]] = defaultdict(
        lambda: {"bbox_shortest_side_px": [], "mask_area_px": [], "distance_m": [], "occlusion_ratio": []}
    )
    visibility = defaultdict(lambda: {"visible": 0, "not_visible": 0})
    for row in rows:
        class_id = row.get("class_id") or "unknown"
        visibility[class_id][row["visibility"]] += 1
        if row["visibility"] != "visible":
            continue
        for key in grouped[class_id]:
            value = row.get(key)
            if value is not None and math.isfinite(float(value)):
                grouped[class_id][key].append(float(value))
    return {
        "metric_semantics": {
            "instance_size": "per runtime instance-id mask, never per-class aggregate",
            "zero_pixel_object": "not_visible",
            "cross_asset_same_world": "asset-isolated evaluation inside a previously seen world",
            "cross_material": "material-isolated evaluation",
            "cross_world": "world-isolated evaluation; null when fewer than two distinct worlds exist",
        },
        "visibility": dict(visibility),
        "by_class": {
            class_id: {key: _percentiles(values) for key, values in metrics.items()}
            for class_id, metrics in grouped.items()
        },
    }


def normalized_domain_metrics(*, same_world, cross_material, cross_world, distinct_world_count: int) -> dict:
    return {
        "cross_asset_same_world": same_world,
        "cross_material": cross_material,
        "cross_world": cross_world if distinct_world_count >= 2 else None,
        "cross_world_eligibility": distinct_world_count >= 2,
        "distinct_world_count": int(distinct_world_count),
    }
