import argparse
import json
from pathlib import Path


def build_report(root, required_seeds=5):
    trials = []
    for path in sorted(root.glob('seed_*/stage4w_static_summary.json')):
        summary = json.loads(path.read_text(encoding='utf-8'))
        coverage = summary['coverage']
        components = coverage.get('component_results', [])
        trials.append({
            'seed': summary['seed'],
            'static_gate_pass': summary['static_gate_pass'],
            'transit_success': coverage.get('transit_to_start_success'),
            'full_execution_success': coverage.get('full_execution_success'),
            'component_terminal_success_count': sum(
                bool(item.get('success')) for item in components
            ),
            'component_terminal_total': coverage.get('component_count'),
            'empirical_coverage_rate': coverage.get('empirical_metrics', {}).get(
                'coverage_rate'
            ),
            'collision_count': coverage.get('collision_count'),
            'keepout_violation_sample_count': coverage.get(
                'keepout_violation_sample_count'
            ),
            'brush_state_violation_sample_count': coverage.get(
                'brush_state_violation_sample_count'
            ),
            'brush_disabled_on_exit': coverage.get('brush_disabled_on_exit'),
            'localization': coverage.get(
                'localization_regression_during_coverage', {}
            ),
            'rosbag_replay': summary.get('rosbag_replay'),
        })
    report = {
        'schema_version': 1,
        'stage': 'Stage4W',
        'required_seed_count': required_seeds,
        'completed_seed_count': len(trials),
        'trials': trials,
        'static_coverage_success_count': sum(
            bool(trial['static_gate_pass']) for trial in trials
        ),
    }
    report['success'] = bool(
        len(trials) == required_seeds
        and all(trial['static_gate_pass'] for trial in trials)
    )
    return report


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('root', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--required-seeds', type=int, default=5)
    args = parser.parse_args()
    report = build_report(args.root, args.required_seeds)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + '\n', encoding='utf-8'
    )
    return 0 if report['success'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
