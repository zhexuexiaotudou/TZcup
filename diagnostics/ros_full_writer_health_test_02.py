"""Bounded ROS-only test of real writers, full observer, and original finalizer.

This is admission/storage regression evidence, never a mission acceptance run.
Run under an outer timeout in isolated ROS domain 172. No Gazebo is started.
"""
import argparse
import shutil
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import rclpy
import rosbag2_py
import yaml
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.serialization import serialize_message
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


TYPES = {
    '/clock': Clock, '/safety/status_json': String,
    '/base_controller/cmd_vel': TwistStamped,
    '/safety/relay_cycle_diagnostic_json': String,
    '/formal_vehicle/auxiliary/critical_safety_relay_diagnostic_json': String,
    '/odom': Odometry, '/odom/unfiltered': Odometry,
    '/odometry/gps': Odometry, '/ground_truth/odom': Odometry,
    '/gnss/fix': NavSatFix, '/formal_mapping/lifecycle_status': String,
}
REQUIRED = ['/odom', '/odom/unfiltered', '/odometry/gps', '/gnss/fix',
            '/formal_mapping/lifecycle_status']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def owner(pid):
    txt = Path(f'/proc/{pid}/stat').read_text()
    fields = txt[txt.rfind(')') + 2:].split()
    return {'pid': pid, 'pgid': int(fields[2]), 'session': int(fields[3]),
            'starttime_ticks': int(fields[19]),
            'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip()}


def stop(worker):
    role, proc, log, identity = worker
    forced = False
    initial_identity_missing = identity is None
    if identity is None and proc.poll() is None:
        identity = owner(proc.pid)
        assert identity['pgid'] == identity['session'] == proc.pid
    if proc.poll() is None:
        assert owner(proc.pid) == identity, 'process identity changed'
        os.killpg(identity['pgid'], signal.SIGINT)
    try:
        rc = proc.wait(timeout=8)
    except subprocess.TimeoutExpired:
        forced = True
        assert owner(proc.pid) == identity, 'process identity changed'
        os.killpg(identity['pgid'], signal.SIGKILL)
        rc = proc.wait(timeout=3)
    log.close()
    survivors = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            current = owner(int(entry.name))
            if current['pgid'] == proc.pid or current['session'] == proc.pid:
                survivors.append(current)
        except (FileNotFoundError, ProcessLookupError):
            pass
    return {'role': role, 'initial_identity': identity, 'rc': rc,
            'initial_identity_missing': initial_identity_missing,
            'forced_kill': forced, 'survivors': survivors}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--observer', type=Path, required=True)
    args = parser.parse_args()
    assert os.environ.get('ROS_DOMAIN_ID') == '172'
    root = args.output.resolve()
    root.mkdir(exist_ok=False)
    out = root / 'map'
    out.mkdir()
    session = root / 'formal_acceptance_session.json'
    session.write_text(json.dumps({'test_only': True, 'scope': 'recording admission'}))
    source = Path('/mnt/f/Project/TZcup/.workspace/evidence/short08-b430dda-failure-closed-01/original/frontier-short-readiness-diagnostic-08')
    (root / 'episode/public').mkdir(parents=True)
    shutil.copyfile(source / 'episode/public/episode_manifest.json', root / 'episode/public/episode_manifest.json')
    for name in ['geofence_keepout.yaml','geofence_keepout.pgm','neutral_speed.yaml','neutral_speed.pgm','mission_geometry.yaml','materialization_contract.yaml']:
        shutil.copyfile(source / 'map' / name, out / name)
    binding = out / 'runtime_binding.json' 
    binding.write_text(json.dumps({'test_only': True, 'verified_epoch_ns': time.time_ns()}))
    started_epoch = time.time_ns()
    env = dict(os.environ, FORMAL_ACCEPTANCE_SESSION=str(session),
               FORMAL_RECORDING_RUN_TOKEN='TEST_ONLY_' + root.name,
               FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS=str(time.monotonic_ns() + 28_000_000_000))
    scripts = Path(__file__).resolve().parents[1] / 'scripts'
    workers = []
    terminal = []
    report = {'test_only': True, 'gazebo_started': False, 'passed': False,
              'claim_boundary': 'real ROS recording/admission only; not product or mission acceptance',
              'observer_sha256': sha(args.observer),
              'helper_sha256': sha(scripts / 'capture_formal_first_map_early_recording_audit.py'),
              'finalizer_sha256': sha(scripts / 'finalize_formal_first_map_localization_diagnostic.py')}
    sent = {topic: set() for topic in TYPES}
    rclpy.init()
    def interrupted(signum, _frame):
        raise RuntimeError('outer timeout/signal: ' + str(signum))
    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    node = rclpy.create_node('recording_time_domain_regression')
    pubs = {}
    for topic, typ in TYPES.items():
        qos = (QoSProfile(depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL)
               if topic in ['/safety/status_json', '/formal_mapping/lifecycle_status'] else 10)
        pubs[topic] = node.create_publisher(typ, topic, qos)

    def launch(role, command):
        log = (root / (role + '.log')).open('xb')
        proc = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        workers.append((role, proc, log, None))
        workers[-1] = (role, proc, log, owner(proc.pid))
        return proc

    failure = None
    try:
        for role in ['localization', 'early']:
            launch(role, [sys.executable, str(scripts / 'capture_formal_first_map_early_recording_audit.py'),
                          '--output', str(out), '--timeout', '12', '--role', role])
        launch('lifecycle_producer', [sys.executable, '-c', 'from sanitation_formal_campus_integration.map_lifecycle_manager import main; main()', '--ros-args', '-p', 'episode_manifest:='+str(root / 'episode/public/episode_manifest.json'), '-p', 'artifact_directory:='+str(out), '-p', 'support_artifacts_prepared:=true'])
        started = time.monotonic()
        tick = 0
        arm_seen = None
        observer = None
        while time.monotonic() - started < 25:
            tick += 1
            clock = Clock()
            clock.clock.sec = 7 + tick // 1000
            clock.clock.nanosec = (tick % 1000) * 1_000_000
            for topic, typ in TYPES.items():
                if topic == "/formal_mapping/lifecycle_status": continue
                msg = typ()
                if topic == '/clock':
                    msg = clock
                elif hasattr(msg, 'header'):
                    msg.header.stamp = clock.clock
                    msg.header.frame_id = 'test_payload_unchanged'
                    if typ is Odometry:
                        msg.pose.pose.position.x = tick / 1000.0
                    elif typ is NavSatFix:
                        msg.latitude = 31.0 + tick / 1_000_000.0
                else:
                    safe = arm_seen is not None and time.monotonic() - arm_seen > 1
                    payload = {'test_only': True, 'state': 'ENABLED' if safe else 'INHIBITED',
                               'active_reasons': '' if safe else 'TEST_EARLY_FAULT',
                               'status_publish_count': 1000 + tick,
                               'safety_inputs_permit_actuators': safe,
                               'actuators_enabled': safe, 'managed_controllers_active': safe}
                    if topic == '/formal_mapping/lifecycle_status':
                        payload = {'test_only': True, 'state': 'mapping', 'tick': tick}
                    msg.data = json.dumps(payload, sort_keys=True)
                sent[topic].add(bytes(serialize_message(msg)))
                pubs[topic].publish(msg)
            arm_path = out / 'formal_recording_arm.json'
            if arm_path.exists() and arm_seen is None:
                arm_seen = time.monotonic()
                report['immutable_arm_sha256'] = sha(arm_path)
            if arm_seen is not None and observer is None and time.monotonic() - arm_seen > 3:
                arm = json.loads(arm_path.read_text())
                report['immutable_arm_age_at_process_launch_s'] = (
                    time.monotonic_ns() - arm['ready_monotonic_ns']) / 1e9
                assert report['immutable_arm_age_at_process_launch_s'] > 3
                observer = launch('full_observer', [sys.executable, str(args.observer), str(root)])
            if observer is not None and observer.poll() is not None:
                break
            assert all(w[1].poll() is None for w in workers if w[0] != 'full_observer'), 'writer exited early'
            rclpy.spin_once(node, timeout_sec=.015)
            time.sleep(.02)
        assert observer is not None and observer.poll() is not None, 'full observer exceeded bounded fixture'
        result = json.loads((root / 'frontier_short.result.json').read_text())
        report['observer_result'] = result
        assert result['formal_recording_arm']['accepted_at_start'], result['formal_recording_arm']
        continuity = result['formal_recording_arm']['current_ready_continuity']
        assert continuity['accepted_at_start'], continuity
        for role, arm_ready in [('early', arm), ('localization', arm['localization_writer'])]:
            admission = continuity['admission_' + role + '_ready']
            assert 0 <= admission['ready_monotonic_age_ns'] <= 2_000_000_000
            assert 0 <= admission['ready_epoch_age_ns'] <= 2_000_000_000
            assert admission['owner_identity'] == arm_ready['owner_identity']
            assert admission['formal_acceptance_session'] == arm_ready['formal_acceptance_session']
            assert admission['formal_acceptance_session_sha256'] == arm_ready['formal_acceptance_session_sha256']
            assert admission['run_token'] == arm_ready['run_token']
            assert admission['bag_dir'] == arm_ready['bag_dir']
            assert arm_ready['formal_acceptance_session'] == str(session)
            assert arm_ready['formal_acceptance_session_sha256'] == sha(session)
            assert arm_ready['run_token'] == env['FORMAL_RECORDING_RUN_TOKEN']
            assert admission['owner_identity']['owner_token'] == env['FORMAL_RECORDING_RUN_TOKEN'] + ':' + role
            assert admission['ready_monotonic_ns'] > arm_ready['ready_monotonic_ns']
        handoff = continuity['arm_to_observer_handoff']
        assert handoff['cumulative_unsafe_count'] >= arm['pre_arm_unsafe_count'] > 0
        assert handoff['unsafe_count_since_arm'] >= 0
        assert handoff['latest_safety']['state'] == 'ENABLED'
        report['full_process_current_ready_accepted'] = True
        assert not result['probe_passed'] and not result['overall_acceptance_passed'], 'mock topics must not pass mission'
        assert sha(arm_path) == report['immutable_arm_sha256'], 'immutable arm changed'
        # Exercise actual observer arm/current/runtime functions while producer is
        # alive, then after death, in addition to the full Popen observer above.
        import importlib.util
        fixture_path = Path('/mnt/f/Project/TZcup/.workspace/tools/verify_live_frontier_short_07_fixture.py')
        spec = importlib.util.spec_from_file_location('observer_fixture_loader', fixture_path)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        ns = module.extract_namespace(args.observer, args.observer.read_text())
        from sanitation_formal_campus_integration.lifecycle_health_protocol import validate_health
        ns.update(time=time, sys=sys, validate_health=validate_health,
                  EXPECTED_SESSION=str(session), EXPECTED_RUN_TOKEN=env['FORMAL_RECORDING_RUN_TOKEN'])
        checked_arm, err = ns['validate_formal_recording_arm'](root)
        assert err is None, err
        current_ready, err = ns['validate_current_recording_readiness'](root, checked_arm)
        assert err is None, err
        runtime_ready, err = ns['validate_current_recording_readiness'](root, checked_arm, current_ready)
        assert err is None, err
        producer = next(w[1] for w in workers if w[0] == 'lifecycle_producer')
        producer.terminate(); producer.wait(timeout=5)
        _, arm_error = ns['validate_formal_recording_arm'](root)
        _, current_error = ns['validate_current_recording_readiness'](root, checked_arm)
        _, runtime_error = ns['validate_current_recording_readiness'](root, checked_arm, current_ready)
        assert all('producer_dead_or_identity_mismatch' in e for e in [arm_error,current_error,runtime_error]), (arm_error,current_error,runtime_error)
        sys.path.insert(0, str(scripts))
        from capture_formal_first_map_early_recording_audit import Tracker, valid_local_ready
        early_ready = json.loads((out / 'formal_recording_ready.json').read_text())
        tracker = Tracker('early', TYPES, str(session), sha(session), env['FORMAL_RECORDING_RUN_TOKEN'], str(out / 'early_recording_audit'))
        tracker.lifecycle_packet = early_ready['lifecycle_packet']; tracker.lifecycle_state = early_ready['lifecycle_state']; tracker.lifecycle_wire_sha256 = early_ready['lifecycle_wire_sha256']; tracker.lifecycle_wire_text = early_ready['lifecycle_wire_text']
        assert any('producer_dead_or_identity_mismatch' in reason for reason in tracker.blockers(time.monotonic_ns()))
        assert valid_local_ready(out / 'formal_localization_ready.json', tracker) is None
        report['actual_reader_paths'] = {'alive_arm_current_runtime':True,'dead_arm_error':arm_error,'dead_current_error':current_error,'dead_runtime_error':runtime_error,'recorder_blockers_dead':True,'local_ready_dead':True}
    except BaseException as exc:
        failure = repr(exc)
    finally:
        for worker in reversed(workers):
            try:
                terminal.append(stop(worker))
            except BaseException as exc:
                terminal.append({'role': worker[0], 'cleanup_error': repr(exc)})
                failure = failure or repr(exc)
        node.destroy_node()
        rclpy.shutdown()
        report['terminal'] = terminal
        report['ended_epoch_ns'] = time.time_ns()
        report['failure'] = failure
        (root / 'results.json').write_text(json.dumps(report, indent=2) + '\n')
    try:
        assert failure is None, failure
        assert all(not row.get('cleanup_error') and not row['initial_identity_missing']
                   and not row['forced_kill'] and not row['survivors'] for row in terminal)
        assert all(row['rc'] == 0 for row in terminal if row['role'] not in ('full_observer', 'lifecycle_producer'))
        counts = {}
        report['closed_bags'] = {}
        for bag_name in ['early_recording_audit', 'mapping_localization_diagnostic']:
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(out / bag_name), storage_id='mcap'),
                        rosbag2_py.ConverterOptions('', ''))
            bag_counts = {}
            while reader.has_next():
                topic, data, storage_ns = reader.read_next()
                assert started_epoch <= storage_ns <= report['ended_epoch_ns'], (topic, storage_ns)
                assert topic == "/formal_mapping/lifecycle_status" or bytes(data) in sent[topic], 'serialized payload changed: ' + topic
                bag_counts[topic] = bag_counts.get(topic, 0) + 1
            del reader
            assert all(bag_counts.get(t, 0) > 0 for t in REQUIRED)
            metadata = yaml.safe_load((out / bag_name / 'metadata.yaml').read_text())['rosbag2_bagfile_information']
            metadata_counts = {row['topic_metadata']['name']: row['message_count']
                               for row in metadata['topics_with_message_count']}
            assert all(metadata_counts.get(t, 0) == bag_counts.get(t, 0)
                       for t in set(metadata_counts) | set(bag_counts))
            assert metadata['message_count'] == sum(bag_counts.values())
            closed_path = out / ('formal_recording_closed.json' if bag_name == 'early_recording_audit'
                                 else 'formal_localization_closed.json')
            closed = json.loads(closed_path.read_text())
            assert closed['normal_close'] and closed['close_rc'] == 0
            report['closed_bags'][bag_name] = {'closed_receipt_sha256': sha(closed_path),
                                               'metadata_sha256': sha(out / bag_name / 'metadata.yaml'),
                                               'metadata_count_matches_actual': True}
            counts[bag_name] = bag_counts
        report['actual_mcap_counts'] = counts
        report['all_storage_times_in_receive_epoch_window'] = True
        report['all_recorded_payload_bytes_equal_published_bytes'] = True
        report['published_sim_stamp_start_ns'] = 7_001_000_000
        manifest = out / 'topics.txt'
        manifest.write_text('\n'.join(REQUIRED + ['/ground_truth/odom']) + '\n')
        receipt = out / 'localization_diagnostic_receipt.json'
        command = [sys.executable, str(scripts / 'finalize_formal_first_map_localization_diagnostic.py'),
                   '--run-root', str(out), '--bag-dir', str(out / 'mapping_localization_diagnostic'),
                   '--topic-manifest', str(manifest), '--output', str(receipt), '--recorder-stop-rc', '0',
                   '--optional-topic', '/ground_truth/odom', '--runtime-binding', str(binding),
                   '--started-epoch-ns', str(started_epoch)]
        before = len(workers)
        try:
            finalizer = launch('original_finalizer', command)
            finalizer.wait(timeout=10)
        finally:
            if len(workers) > before:
                finalizer_terminal = stop(workers[-1])
                terminal.append(finalizer_terminal)
                report['terminal'] = terminal
        report['original_finalizer'] = {'argv': command, 'rc': finalizer.returncode,
                                        'log': (root / 'original_finalizer.log').read_text(),
                                        'receipt': json.loads(receipt.read_text()) if receipt.exists() else None}
        assert finalizer.returncode == 0, report['original_finalizer']
        assert not finalizer_terminal['forced_kill'] and not finalizer_terminal['survivors']
        assert not finalizer_terminal['initial_identity_missing']
        report['passed'] = True
    except BaseException as exc:
        report['failure'] = repr(exc)
    (root / 'results.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({'output': str(root), 'passed': report['passed'], 'failure': report['failure']}))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
