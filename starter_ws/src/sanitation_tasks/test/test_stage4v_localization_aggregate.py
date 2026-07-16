import math

from sanitation_tasks.stage4v_aggregate import aggregate, percentile


def test_percentile_interpolates_ten_seed_p95():
    assert math.isclose(percentile(list(range(1, 11)), 0.95), 9.55)


def test_aggregate_requires_every_seed_and_every_gate(tmp_path):
    for seed in range(10):
        trial = tmp_path / f'seed_{seed}'
        trial.mkdir()
        (trial / 'trial_summary.json').write_text(
            __import__('json').dumps(
                {
                    'localization': {
                        'competition_localization_pass': True,
                        'xy_error_m': {'rmse': 0.02 + seed * 0.001},
                    },
                    'navigation': {'success': True},
                    'tf_ownership': {'single_owner': True},
                    'scan_refiner': {'accepted_count': seed + 1},
                    'ground_truth_used_for_control': False,
                }
            ),
            encoding='utf-8',
        )
    report = aggregate(tmp_path, 'hybrid_rtk_scan_imu_wheel', 10)
    assert report['formal_gate_pass'] is True
    assert report['navigation_success_count'] == 10
    assert report['scan_refinement_engaged_count'] == 10


def test_hybrid_gate_rejects_rtk_only_fallback(tmp_path):
    trial = tmp_path / 'seed_0'
    trial.mkdir()
    (trial / 'trial_summary.json').write_text(
        __import__('json').dumps(
            {
                'localization': {
                    'competition_localization_pass': True,
                    'xy_error_m': {'rmse': 0.03},
                },
                'navigation': {'success': True},
                'tf_ownership': {'single_owner': True},
                'scan_refiner': {'accepted_count': 0},
                'ground_truth_used_for_control': False,
            }
        ),
        encoding='utf-8',
    )
    report = aggregate(tmp_path, 'hybrid_rtk_scan_imu_wheel', 1)
    assert report['formal_gate_pass'] is False
    assert report['scan_refinement_engaged_count'] == 0
