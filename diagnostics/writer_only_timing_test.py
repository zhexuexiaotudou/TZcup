"""Bounded ROS-only test of one real writer, minimal lifecycle producer, and strict finalizer.

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
    (root / 'harness_start.json').write_text(json.dumps({'started_epoch_ns':started_epoch,'started_monotonic_ns':time.monotonic_ns()}))
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
               if topic in ['/safety/status_json', '/formal_mapping/lifecycle_status', '/odometry/gps'] else 10)
        pubs[topic] = node.create_publisher(typ, topic, qos)

    def launch(role, command):
        log = (root / (role + '.log')).open('xb')
        proc = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        workers.append((role, proc, log, None))
        workers[-1] = (role, proc, log, owner(proc.pid))
        return proc

    cached = Odometry(); cached.header.stamp.sec = 1; cached.header.frame_id = 'test_cached_before_writer'
    cached_bytes = bytes(serialize_message(cached)); sent['/odometry/gps'].add(cached_bytes)
    pubs['/odometry/gps'].publish(cached)
    report['preexisting_transient_local_cache_epoch_ns'] = time.time_ns()
    failure = None
    try:
        for role in ['localization']:
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
            arm_path = out / 'formal_localization_ready.json'
            if arm_path.exists() and arm_seen is None:
                arm_seen = time.monotonic()
                report['immutable_arm_sha256'] = sha(arm_path)
            if arm_seen is not None and time.monotonic() - arm_seen > 1:
                pubs['/odometry/gps'].publish(cached)
                report['cached_object_republished_epoch_ns'] = time.time_ns()
                for _ in range(10):rclpy.spin_once(node, timeout_sec=.02)
                time.sleep(.1)
                break
            assert all(w[1].poll() is None for w in workers), 'writer or producer exited early'
            rclpy.spin_once(node, timeout_sec=.015)
            time.sleep(.02)
        assert (out / 'formal_localization_ready.json').is_file()
        report['minimal_writer_ready'] = True
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
        for bag_name in ['mapping_localization_diagnostic']:
            reader = rosbag2_py.SequentialReader()
            reader.open(rosbag2_py.StorageOptions(uri=str(out / bag_name), storage_id='mcap'),
                        rosbag2_py.ConverterOptions('', ''))
            bag_counts = {}
            while reader.has_next():
                topic, data, storage_ns = reader.read_next()
                assert json.loads((out / 'formal_localization_writer_open.json').read_bytes())['acceptance_window_start_epoch_ns'] <= storage_ns <= report['ended_epoch_ns'], (topic, storage_ns)
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
        report['synthetic_non_lifecycle_published_bytes_verified'] = True
        report['lifecycle_publisher_wire_bytes_independently_compared'] = False
        report['published_sim_stamp_start_ns'] = 7_001_000_000
        manifest = out / 'topics.txt'
        manifest.write_text('\n'.join(REQUIRED + ['/ground_truth/odom']) + '\n')
        receipt = out / 'localization_diagnostic_receipt.json'
        command = [sys.executable, str(scripts / 'finalize_formal_first_map_localization_diagnostic.py'),
                   '--run-root', str(out), '--bag-dir', str(out / 'mapping_localization_diagnostic'),
                   '--topic-manifest', str(manifest), '--output', str(receipt), '--recorder-stop-rc', '0',
                   '--optional-topic', '/ground_truth/odom', '--runtime-binding', str(binding),
                   '--require-writer-timing', '--started-epoch-ns', str(json.loads((out / 'formal_localization_writer_open.json').read_bytes())['acceptance_window_start_epoch_ns'])]
        before = len(workers)
        try:
            finalizer = launch('strict_finalizer', command)
            finalizer.wait(timeout=10)
        finally:
            if len(workers) > before:
                finalizer_terminal = stop(workers[-1])
                terminal.append(finalizer_terminal)
                report['terminal'] = terminal
        report['strict_finalizer'] = {'argv': command, 'rc': finalizer.returncode,
                                        'log': (root / 'strict_finalizer.log').read_text(),
                                        'receipt': json.loads(receipt.read_text()) if receipt.exists() else None}
        assert finalizer.returncode == 0, report['strict_finalizer']
        report['callback_to_mcap_all_records_verified'] = report['strict_finalizer']['receipt']['writer_timing_contract']['verified']
        assert report['callback_to_mcap_all_records_verified'] is True
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
