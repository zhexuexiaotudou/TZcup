import json
import math
from pathlib import Path
import statistics


def percentile(values, quantile):
    ordered = sorted(values)
    if not ordered:
        return None
    position = (len(ordered) - 1) * quantile
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - position) + ordered[upper] * (position - lower)


def aggregate(root, lane, required_seeds):
    root = Path(root)
    trials = [
        json.loads(path.read_text(encoding='utf-8'))
        for path in sorted(root.glob('seed_*/trial_summary.json'))
    ]
    rmses = [
        trial['localization']['xy_error_m']['rmse']
        for trial in trials
        if trial.get('localization')
        and trial['localization'].get('xy_error_m', {}).get('rmse') is not None
    ]
    localization_passes = [
        bool(trial.get('localization', {}).get('competition_localization_pass'))
        for trial in trials
    ]
    navigation_passes = [
        bool(trial.get('navigation', {}).get('success')) for trial in trials
    ]
    tf_passes = [
        bool(trial.get('tf_ownership', {}).get('single_owner')) for trial in trials
    ]
    scan_required = lane in {
        'hybrid_rtk_scan_imu_wheel',
        'gnss_denied_scan_fallback',
    }
    scan_engaged = [
        bool(trial.get('scan_refiner', {}).get('accepted_count', 0) > 0)
        for trial in trials
    ]
    complete = len(trials) == required_seeds and len(rmses) == required_seeds
    return {
        'schema_version': 1,
        'stage': 'Stage4V',
        'lane': lane,
        'required_seed_count': required_seeds,
        'completed_seed_count': len(trials),
        'complete': complete,
        'rmse_m': {
            'values': rmses,
            'p50': statistics.median(rmses) if rmses else None,
            'p95': percentile(rmses, 0.95),
            'max': max(rmses) if rmses else None,
        },
        'all_trials_rmse_le_0_05': bool(
            complete and all(value <= 0.05 for value in rmses)
        ),
        'aggregate_p95_le_0_05': bool(
            complete and rmses and percentile(rmses, 0.95) <= 0.05
        ),
        'localization_pass_count': sum(localization_passes),
        'navigation_success_count': sum(navigation_passes),
        'tf_single_owner_count': sum(tf_passes),
        'scan_refinement_required': scan_required,
        'scan_refinement_engaged_count': sum(scan_engaged),
        'scan_refinement_all_trials_engaged': bool(
            complete and (not scan_required or all(scan_engaged))
        ),
        'ground_truth_control_violation_count': sum(
            bool(trial.get('ground_truth_used_for_control')) for trial in trials
        ),
        'formal_gate_pass': bool(
            complete
            and all(localization_passes)
            and all(navigation_passes)
            and all(tf_passes)
            and (not scan_required or all(scan_engaged))
            and all(value <= 0.05 for value in rmses)
            and percentile(rmses, 0.95) <= 0.05
            and not any(
                bool(trial.get('ground_truth_used_for_control')) for trial in trials
            )
        ),
        'trials': trials,
    }
