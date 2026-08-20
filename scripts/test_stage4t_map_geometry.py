from pathlib import Path

import numpy as np

from stage4t_map_geometry import metrics, sdf_primitives


def test_sdf_truth_uses_only_collision_shapes_crossing_lidar_plane(tmp_path: Path):
    sdf = tmp_path / "world.sdf"
    sdf.write_text(
        """<sdf version='1.9'><world name='test'>
        <model name='wall'><static>true</static><pose>0 0 1 0 0 0</pose><link name='l'>
          <collision name='c'><geometry><box><size>2 1 2</size></box></geometry></collision>
        </link></model>
        <model name='roof'><static>true</static><pose>0 0 5 0 0 0</pose><link name='l'>
          <collision name='c'><geometry><box><size>2 1 1</size></box></geometry></collision>
        </link></model>
        <model name='post'><static>true</static><pose>3 0 1 0 0 0</pose><link name='l'>
          <collision name='c'><geometry><cylinder><radius>0.2</radius><length>2</length></cylinder></geometry></collision>
          <visual name='duplicate'><geometry><cylinder><radius>0.2</radius><length>2</length></cylinder></geometry></visual>
        </link></model>
        <model name='glass'><static>true</static><pose>5 0 1 0 0 0</pose><link name='l'>
          <visual name='v'><geometry><box><size>2 0.1 2</size></box></geometry></visual>
        </link></model>
        <model name='near_plane'><static>true</static><pose>7 0 0.50 0 0 0</pose><link name='l'>
          <visual name='v'><geometry><box><size>1 1 0.2</size></box></geometry></visual>
        </link></model>
        </world></sdf>""",
        encoding="utf-8",
    )

    primitives = sdf_primitives(sdf, lidar_height=0.64)

    assert [(item["name"], item["type"]) for item in primitives] == [
        ("wall", "box"),
        ("post", "circle"),
        ("glass", "box"),
        ("near_plane", "box"),
    ]


def test_metrics_ignore_truth_surfaces_hidden_behind_unknown_space():
    truth = np.zeros((20, 20), dtype=bool)
    truth[5:15, 5:15] = True
    observed = np.zeros_like(truth)
    observed[14, 5:15] = True
    known_free = np.zeros_like(truth)
    known_free[15:18, 5:15] = True

    report = metrics(observed, truth, known_free, resolution=0.1)

    assert report["observable_truth_boundary_ratio"] < 0.5
    assert report["boundary_rmse_m"] <= 0.1
    assert report["visible_truth_boundary_recall"] == 1.0
    assert report["loop_ghosting_ratio"] == 0.0


def test_metrics_report_unmodelled_occupied_surfaces_as_ghosting():
    truth = np.zeros((20, 20), dtype=bool)
    truth[5:15, 5:15] = True
    observed = np.zeros_like(truth)
    observed[14, 5:15] = True
    observed[1, 1] = True
    known_free = np.zeros_like(truth)
    known_free[15:18, 5:15] = True

    report = metrics(observed, truth, known_free, resolution=0.1)

    assert report["loop_ghosting_occupied_cells"] == 1
    assert report["loop_ghosting_ratio"] > 0.0
