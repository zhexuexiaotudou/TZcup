"""Bounded, diagnostic-only live check of three frontier goals."""
import json
import hashlib
import math
import os
import pathlib
import sys
import time
import traceback

import rclpy
from sanitation_formal_campus_integration.lifecycle_health_protocol import validate_health
from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Twist, TwistStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool
from std_msgs.msg import String

CANDIDATE = 'b430dda514ccb7c119b03859b2fd60ff3227ef71'
WALL_LIMIT_S = 570.0
CANCEL_AFTER_EXECUTING_S = 125.0
FRESH_WALL_S = 2.0
FRESH_SIM_S = 2.0
INSET_M = 0.5
MIN_PROGRESS_M = 0.05
COMMAND_EPSILON = 0.0
CHAIN_RECEIPT_REORDER_TOLERANCE_S = 0.1
EXPECTED_SESSION = os.environ.get('FORMAL_ACCEPTANCE_SESSION')
EXPECTED_RUN_TOKEN = os.environ.get('FORMAL_RECORDING_RUN_TOKEN')
ARM_FRESH_NS = 2_000_000_000
ARM_REQUIRED_TOPICS = (
    '/clock',
    '/safety/status_json',
    '/odom',
    '/odom/unfiltered',
    '/odometry/gps',
    '/gnss/fix',
    '/formal_mapping/lifecycle_status',
    '/base_controller/cmd_vel',
    '/safety/relay_cycle_diagnostic_json',
    '/formal_vehicle/auxiliary/critical_safety_relay_diagnostic_json',
)
LOCALIZATION_ARM_REQUIRED_TOPICS = (
    '/odom',
    '/odom/unfiltered',
    '/odometry/gps',
    '/gnss/fix',
    '/formal_mapping/lifecycle_status',
)

out = pathlib.Path(sys.argv[1])
assert out.is_dir() and not out.is_symlink()
events = (out / 'frontier_short.events.jsonl').open('x', encoding='utf-8')
start = time.monotonic()
rclpy.init()
node = rclpy.create_node('bounded_frontier_integration_probe')
qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
grid_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)
status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                        durability=DurabilityPolicy.TRANSIENT_LOCAL)

window = None
window_sequence = 0
clock_ns = None
safety = None
last_safety_publish_count = None
gate_command = None
base_command = None
pre_goal_invalid_base_samples = 0
odom = None
action_snapshot = None
action_signature = None
goals = []
current = None
failure = None
successes = 0
consecutive_failures = 0
cancel_future = None
cancel_started = None
completed_ok = False
log_offset = 0
last_log_scan = 0.0
subs = []
evidence_counts = {}
global_motion_active = False
global_active_wall = None
pre_goal_relay_false_samples = 0
producer_diagnostics = {'received': 0, 'matched_critical_cycles': set(),
                        'last_cycle_id': None, 'last_input_sequences': {},
                        'last_received': None}
manager_diagnostics = {'received': 0, 'matched_status_cycles': set(),
                       'matched_base_cycles': set(), 'active_output_cycles': set(),
                       'bootstrap_noncausal_cycles': set(),
                       'last_cycle_id': None, 'last_received': None}
diagnostic_identities = {}
critical_status_by_count = {}
producer_diagnostic_by_cycle = {}
safety_status_by_count = {}
manager_diagnostic_by_status_count = {}
manager_diagnostic_by_cycle = {}
base_command_by_stamp = {}
base_matched_stamps = {}
recording_current_ready = None
recording_current_error = None
recording_admission_handoff = None
recording_admission_ready_summary = None
recording_runtime_last_summary = None
client = node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')


def emit(kind, data):
    monotonic_ns = time.monotonic_ns()
    events.write(json.dumps({'kind': kind,
                             'wall_s': monotonic_ns * 1e-9 - start,
                             'host_monotonic_ns': monotonic_ns,
                             'host_epoch_ns': time.time_ns(),
                             'clock_ns': clock_ns, 'data': data},
                            allow_nan=False, separators=(',', ':')) + '\n')
    events.flush()


def fail(reason):
    global failure
    if failure is None:
        failure = str(reason)
        emit('failure', {'reason': failure})


def stamp_ns(stamp):
    sec, nanosec = int(stamp.sec), int(stamp.nanosec)
    if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        return None
    return sec * 1_000_000_000 + nanosec


def finite(values):
    try:
        return all(math.isfinite(float(value)) for value in values)
    except (TypeError, ValueError):
        return False


def json_float(value):
    value = float(value)
    if math.isfinite(value):
        return value
    return {'nonfinite_float': repr(value)}


def duration_ns(value):
    return int(value.sec) * 1_000_000_000 + int(value.nanosec)


def on_unstamped_twist(topic, msg):
    now = time.monotonic()
    values = (float(msg.linear.x), float(msg.angular.z))
    raw_nonzero = command_nonzero(*values) if finite(values) else None
    evidence_counts[topic] = evidence_counts.get(topic, 0) + 1
    emit('twist_sample', {
        'topic': topic,
        'linear': [json_float(msg.linear.x), json_float(msg.linear.y),
                   json_float(msg.linear.z)],
        'angular': [json_float(msg.angular.x), json_float(msg.angular.y),
                    json_float(msg.angular.z)],
        'nonzero': raw_nonzero, 'source_stamp_available': False})
    if not finite(values):
        if current and not current.get('outcome'):
            fail('invalid_' + topic.strip('/').replace('/', '_') + '_command')
        return
    if current is None or current.get('outcome') or not raw_nonzero:
        return
    if topic == '/cmd_vel_nav' and current['first_nonzero_nav_wall'] is None:
        current['first_nonzero_nav_wall'] = now
        current['first_nonzero_nav'] = {'linear_x': values[0],
                                        'angular_z': values[1], 'received': now}
        current['command_chain_phase'] = 'PROPAGATING'
        if not global_motion_active and current['safety_phase'] == 'ARMING_ZERO_OUTPUT':
            current['safety_phase'] = 'PROPAGATING'
        emit('first_nonzero_nav_command', {
            'number': current['number'], 'command': current['first_nonzero_nav']})
    elif (topic == '/cmd_vel_smoothed' and
          current['first_nonzero_smoothed_wall'] is None):
        current['first_nonzero_smoothed_wall'] = now
        current['first_nonzero_smoothed'] = {
            'linear_x': values[0], 'angular_z': values[1], 'received': now}
        emit('first_nonzero_smoothed_command', {
            'number': current['number'],
            'command': current['first_nonzero_smoothed']})


def on_bool(topic, msg):
    global pre_goal_relay_false_samples
    evidence_counts[topic] = evidence_counts.get(topic, 0) + 1
    value = bool(msg.data)
    emit('bool_sample', {'topic': topic, 'value': value,
                         'source_stamp_available': False,
                         'goal_number': current['number'] if current else None,
                         'global_motion_active': global_motion_active})
    if topic == '/safety/relay_enabled' and not value:
        if current and not current.get('outcome'):
            current['failing_relay_payload'] = {
                'topic': topic, 'value': value, 'received': time.monotonic(),
                'goal_safety_phase': current['safety_phase']}
            fail('relay_disabled_during_goal')
        elif global_motion_active:
            fail('relay_disabled_after_global_motion_active')
        else:
            pre_goal_relay_false_samples += 1
            emit('pre_goal_deenergized_relay_observation', {
                'topic': topic, 'value': value,
                'classification': 'RECORDED_NOT_SAFE_PASS_EVIDENCE'})


def on_string(topic, msg):
    evidence_counts[topic] = evidence_counts.get(topic, 0) + 1
    parsed = None
    parse_error = None
    try:
        parsed = json.loads(msg.data)
    except (TypeError, ValueError, json.JSONDecodeError) as exc:
        parse_error = type(exc).__name__ + ':' + str(exc)
    emit('string_sample', {'topic': topic, 'raw': msg.data, 'parsed': parsed,
                           'parse_error': parse_error,
                           'source_stamp_available': False})


def integer(value, minimum=0):
    return not isinstance(value, bool) and isinstance(value, int) and value >= minimum


def process_identity_tuple(value):
    if not isinstance(value, dict):
        return None
    valid = (isinstance(EXPECTED_SESSION, str) and bool(EXPECTED_SESSION) and
             isinstance(value.get('boot_id'), str) and bool(value['boot_id']) and
             integer(value.get('pid'), 1) and integer(value.get('pgid'), 1) and
             integer(value.get('process_starttime_ticks'), 1) and
             value.get('formal_acceptance_session') == EXPECTED_SESSION)
    if not valid:
        return None
    return (value['boot_id'], value['pid'], value['pgid'],
            value['process_starttime_ticks'],
            value['formal_acceptance_session'])


def lock_process_identity(role, identity):
    existing = diagnostic_identities.get(role)
    if existing is None:
        diagnostic_identities[role] = identity
        return True
    return existing == identity


def valid_diagnostic_transport(value):
    if not isinstance(value, dict):
        return False
    required = {'capacity', 'depth', 'dropped', 'published', 'errors',
                'last_error', 'worker_alive', 'shutdown_requested',
                'shutdown_timed_out'}
    return (required <= set(value) and integer(value.get('capacity'), 1) and
            integer(value.get('depth')) and value['depth'] <= value['capacity'] and
            value.get('dropped') == 0 and integer(value.get('published')) and
            value.get('errors') == 0 and value.get('last_error') is None and
            value.get('worker_alive') is True and
            value.get('shutdown_requested') is False and
            value.get('shutdown_timed_out') is False)


def process_starttime_ticks(pid, proc_root=pathlib.Path('/proc')):
    try:
        raw = (proc_root / str(pid) / 'stat').read_text(encoding='utf-8')
        close = raw.rfind(')')
        fields_after_comm = raw[close + 2:].split()
        value = int(fields_after_comm[19])
    except (OSError, ValueError, IndexError):
        return None
    return value if value > 0 else None


def arm_owner_alive(payload, proc_root=pathlib.Path('/proc'), getpgid=None):
    if getpgid is None:
        getpgid = os.getpgid
    try:
        pid = payload['pid']
        return (process_starttime_ticks(pid, proc_root) ==
                payload['process_starttime_ticks'] and
                int(getpgid(pid)) == payload['pgid'] and
                (proc_root / 'sys/kernel/random/boot_id').read_text(
                    encoding='utf-8').strip() == payload['boot_id'])
    except (KeyError, OSError, TypeError, ValueError):
        return False


def sha256_regular_file(path):
    if path.is_symlink() or not path.is_file():
        return None
    digest = hashlib.sha256()
    try:
        with path.open('rb') as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(chunk)
    except OSError:
        return None
    return digest.hexdigest()


def writer_topics_ready(counts, last_written, required_topics,
                        now_monotonic_ns, not_after_monotonic_ns=None):
    if not isinstance(counts, dict) or not isinstance(last_written, dict):
        return None
    for topic in required_topics:
        count = counts.get(topic)
        written = last_written.get(topic)
        if (not integer(count, 1) or not integer(written, 1) or
                written > now_monotonic_ns or
                (topic != "/formal_mapping/lifecycle_status" and now_monotonic_ns - written > ARM_FRESH_NS) or
                (not_after_monotonic_ns is not None and
                 written > not_after_monotonic_ns)):
            return topic
    return ''


def validate_recording_role_contract(payload, role, expected_session,
                                     session_sha, run_token, expected_bag,
                                     required_topics, now_monotonic_ns,
                                     now_epoch_ns, require_current_freshness=True):
    if not isinstance(payload, dict):
        return None, 'not_an_object'
    identity = payload.get('owner_identity')
    expected_status = ('FORMAL_RECORDING_ARMED' if role == 'early' else
                       'FORMAL_LOCALIZATION_RECORDING_READY')
    expected_report = ('tzcup_formal_recording_arm_v1' if role == 'early' else
                       'tzcup_formal_localization_recording_ready_v1')
    if (payload.get('schema_version') != 1 or
            payload.get('report_id') != expected_report or
            payload.get('status') != expected_status or
            payload.get('ready') is not True or
            payload.get('role') != role or
            payload.get('formal_acceptance_session') != expected_session or
            payload.get('formal_acceptance_session_sha256') != session_sha or
            payload.get('run_token') != run_token or
            not isinstance(identity, dict) or
            identity.get('owner_token') != run_token + ':' + role or
            not isinstance(identity.get('boot_id'), str) or
            not identity['boot_id'] or
            not integer(identity.get('pid'), 1) or
            not integer(identity.get('pgid'), 1) or
            not integer(identity.get('process_starttime_ticks'), 1) or
            not integer(payload.get('clock_ns'), 1) or
            not integer(payload.get('clock_advances'), 2) or
            payload.get('clock_rollback') is not False or
            not integer(payload.get('last_clock_advance_monotonic_ns'), 1) or
            not integer(payload.get('ready_epoch_ns'), 1) or
            not integer(payload.get('ready_monotonic_ns'), 1)):
        return None, 'invalid_' + role + '_writer_contract'
    if role == 'early':
        safety = payload.get('pre_arm_latest_safety')
        if (payload.get('armed') is not True or
                payload.get('pre_arm_nonzero_seen') is not False or
                payload.get('pre_arm_active_seen') is not False or
                not integer(payload.get('pre_arm_unsafe_count')) or
                not integer(payload.get('pre_arm_status_count'), 1) or
                not integer(payload.get('pre_arm_status_messages_received'), 1) or
                not isinstance(safety, dict) or
                safety.get('status_publish_count') !=
                    payload.get('pre_arm_status_count') or
                not isinstance(payload.get('writer_message_counts'), dict) or
                payload['writer_message_counts'].get('/safety/status_json') !=
                    payload.get('pre_arm_status_messages_received')):
            return None, 'invalid_early_writer_handoff_contract'
    try:
        bag = pathlib.Path(payload['bag_dir'])
    except (KeyError, TypeError):
        return None, 'invalid_' + role + '_writer_bag_path'
    if bag != expected_bag:
        return None, 'wrong_' + role + '_writer_bag_path'
    ready_monotonic = payload['ready_monotonic_ns']
    ready_epoch = payload['ready_epoch_ns']
    if (require_current_freshness and
            (now_monotonic_ns < ready_monotonic or
             now_monotonic_ns - ready_monotonic > ARM_FRESH_NS or
             now_epoch_ns < ready_epoch or
             now_epoch_ns - ready_epoch > ARM_FRESH_NS)):
        return None, role + '_writer_ready_stale_or_future'
    last_clock_advance = payload['last_clock_advance_monotonic_ns']
    freshness_reference = (now_monotonic_ns if require_current_freshness else
                           ready_monotonic)
    if (last_clock_advance > ready_monotonic or
            freshness_reference - last_clock_advance > ARM_FRESH_NS):
        return None, role + '_writer_clock_not_freshly_advancing'
    missing = writer_topics_ready(
        payload.get('writer_message_counts'),
        payload.get('last_message_monotonic_ns_by_topic'), required_topics,
        freshness_reference, ready_monotonic)
    if missing is None:
        return None, 'invalid_' + role + '_writer_maps'
    if missing:
        return None, role + '_writer_topic_not_ready:' + missing
    state, error = validate_health(payload.get('lifecycle_packet'), payload.get('lifecycle_state'),
                                   freshness_reference, expected_session, session_sha, run_token,
                                   wire_sha256=payload.get('lifecycle_wire_sha256'),
                                   wire_text=payload.get('lifecycle_wire_text'))
    if error or state != payload.get('lifecycle_state'):
        return None, role + '_writer_lifecycle_' + (error or 'state_mismatch')
    return identity, None


def validate_formal_recording_arm(run_out, now_monotonic_ns=None,
                                  now_epoch_ns=None,
                                  proc_root=pathlib.Path('/proc'), getpgid=None):
    now_monotonic_ns = (time.monotonic_ns() if now_monotonic_ns is None else
                        now_monotonic_ns)
    now_epoch_ns = time.time_ns() if now_epoch_ns is None else now_epoch_ns
    arm_path = run_out / 'map' / 'formal_recording_arm.json'
    try:
        if (arm_path.is_symlink() or not arm_path.is_file() or
                arm_path.parent.is_symlink() or not arm_path.parent.is_dir()):
            return None, 'formal_recording_arm_not_regular'
        payload = json.loads(arm_path.read_text(encoding='utf-8'))
    except (OSError, ValueError, json.JSONDecodeError):
        return None, 'formal_recording_arm_unreadable'
    session_path = (pathlib.Path(EXPECTED_SESSION)
                    if isinstance(EXPECTED_SESSION, str) else None)
    expected_session_path = run_out / 'formal_acceptance_session.json'
    session_sha = (sha256_regular_file(session_path)
                   if session_path is not None else None)
    if (session_path != expected_session_path or not isinstance(session_sha, str) or
            not isinstance(EXPECTED_RUN_TOKEN, str) or not EXPECTED_RUN_TOKEN):
        return None, 'formal_recording_arm_session_or_token_invalid'
    early_identity, error = validate_recording_role_contract(
        payload, 'early', EXPECTED_SESSION, session_sha, EXPECTED_RUN_TOKEN,
        run_out / 'map' / 'early_recording_audit', ARM_REQUIRED_TOPICS,
        now_monotonic_ns, now_epoch_ns, False)
    if error:
        return None, 'formal_recording_arm_' + error
    early_bag = pathlib.Path(payload['bag_dir'])
    if (early_bag.is_symlink() or not early_bag.is_dir() or
            early_bag.resolve() != (run_out / 'map' /
                                    'early_recording_audit').resolve()):
        return None, 'formal_recording_arm_unsafe_early_writer_bag_path'
    localization = payload.get('localization_writer')
    localization_identity, error = validate_recording_role_contract(
        localization, 'localization', EXPECTED_SESSION, session_sha,
        EXPECTED_RUN_TOKEN,
        run_out / 'map' / 'mapping_localization_diagnostic',
        LOCALIZATION_ARM_REQUIRED_TOPICS, now_monotonic_ns, now_epoch_ns,
        False)
    if error:
        return None, 'formal_recording_arm_' + error
    localization_bag = pathlib.Path(localization['bag_dir'])
    if (localization_bag.is_symlink() or not localization_bag.is_dir() or
            localization_bag.resolve() != (run_out / 'map' /
                'mapping_localization_diagnostic').resolve()):
        return None, 'formal_recording_arm_unsafe_localization_writer_bag_path'
    if early_identity['pid'] == localization_identity['pid']:
        return None, 'formal_recording_arm_writer_roles_not_independent'
    if not arm_owner_alive(early_identity, proc_root, getpgid):
        return None, 'formal_recording_arm_owner_not_alive_or_reused'
    if not arm_owner_alive(localization_identity, proc_root, getpgid):
        return None, 'formal_recording_arm_localization_owner_not_alive_or_reused'
    payload['_observer_early_identity'] = early_identity
    payload['_observer_localization_identity'] = localization_identity
    payload['_observer_arm_path'] = str(arm_path)
    payload['_observer_arm_sha256'] = sha256_regular_file(arm_path)
    return payload, None


def read_regular_json(path, capture=None):
    try:
        if (path.is_symlink() or not path.is_file() or
                path.parent.is_symlink() or not path.parent.is_dir()):
            return None, None
        read_start = time.monotonic_ns() if capture is not None else None
        raw = path.read_bytes()
        read_end = time.monotonic_ns() if capture is not None else None
        if capture is not None:
            capture.update(path=str(path), raw_hex=raw.hex(), sha256=hashlib.sha256(raw).hexdigest(),
                           read_start_monotonic_ns=read_start, read_end_monotonic_ns=read_end)
        value = json.loads(raw.decode('utf-8'))
        if not isinstance(value, dict):
            return None, None
        return value, hashlib.sha256(raw).hexdigest()
    except (OSError, UnicodeError, ValueError, json.JSONDecodeError):
        return None, None


def monotonic_writer_continuity(current, baseline, prior, required_topics):
    for field in ('writer_message_counts',
                  'last_message_monotonic_ns_by_topic'):
        current_map = current.get(field)
        baseline_map = baseline.get(field)
        prior_map = None if prior is None else prior.get(field)
        if not isinstance(current_map, dict) or not isinstance(baseline_map, dict):
            return field + '_invalid'
        for topic in required_topics:
            value = current_map.get(topic)
            base_value = baseline_map.get(topic)
            prior_value = None if prior_map is None else prior_map.get(topic)
            if (not integer(value, 1) or not integer(base_value, 1) or
                    value < base_value or
                    (prior_value is not None and
                     (not integer(prior_value, 1) or value < prior_value))):
                return field + '_rollback:' + topic
    for field in ('clock_ns', 'clock_advances',
                  'last_clock_advance_monotonic_ns'):
        value = current.get(field)
        base_value = baseline.get(field)
        prior_value = None if prior is None else prior.get(field)
        if (not integer(value, 1) or not integer(base_value, 1) or
                value < base_value or
                (prior_value is not None and
                 (not integer(prior_value, 1) or value < prior_value))):
            return field + '_rollback'
    return None


def validate_current_recording_role(payload, role, baseline, prior,
                                    expected_session, session_sha, run_token,
                                    expected_bag, required_topics,
                                    now_monotonic_ns, now_epoch_ns,
                                    proc_root=pathlib.Path('/proc'), getpgid=None):
    expected_status = ('FORMAL_RECORDING_EARLY_READY' if role == 'early' else
                       'FORMAL_LOCALIZATION_RECORDING_READY')
    expected_report = ('tzcup_formal_early_recording_ready_v1'
                       if role == 'early' else
                       'tzcup_formal_localization_recording_ready_v1')
    identity = payload.get('owner_identity') if isinstance(payload, dict) else None
    baseline_identity = baseline.get('owner_identity') if isinstance(baseline, dict) else None
    if (not isinstance(payload, dict) or payload.get('schema_version') != 1 or
            payload.get('report_id') != expected_report or
            payload.get('status') != expected_status or
            payload.get('ready') is not True or payload.get('role') != role or
            payload.get('formal_acceptance_session') != expected_session or
            payload.get('formal_acceptance_session_sha256') != session_sha or
            payload.get('run_token') != run_token or
            identity != baseline_identity or
            not isinstance(identity, dict) or
            identity.get('owner_token') != run_token + ':' + role or
            payload.get('bag_dir') != str(expected_bag) or
            payload.get('clock_rollback') is not False or
            not integer(payload.get('ready_epoch_ns'), 1) or
            not integer(payload.get('ready_monotonic_ns'), 1)):
        return None, 'invalid_' + role + '_current_ready_contract'
    ready_monotonic = payload['ready_monotonic_ns']
    ready_epoch = payload['ready_epoch_ns']
    if (now_monotonic_ns < ready_monotonic or
            now_monotonic_ns - ready_monotonic > ARM_FRESH_NS or
            now_epoch_ns < ready_epoch or
            now_epoch_ns - ready_epoch > ARM_FRESH_NS):
        return None, role + '_current_ready_stale_or_future'
    if (ready_monotonic < baseline.get('ready_monotonic_ns', 0) or
            ready_epoch < baseline.get('ready_epoch_ns', 0)):
        return None, role + '_current_ready_time_before_arm'
    last_clock_advance = payload.get('last_clock_advance_monotonic_ns')
    if (not integer(last_clock_advance, 1) or
            last_clock_advance > ready_monotonic or
            now_monotonic_ns - last_clock_advance > ARM_FRESH_NS):
        return None, role + '_current_clock_not_freshly_advancing'
    missing = writer_topics_ready(
        payload.get('writer_message_counts'),
        payload.get('last_message_monotonic_ns_by_topic'), required_topics,
        now_monotonic_ns, ready_monotonic)
    if missing is None:
        return None, 'invalid_' + role + '_current_ready_maps'
    if missing:
        return None, role + '_current_topic_not_ready:' + missing
    lifecycle_previous = (prior if prior is not None else baseline).get('lifecycle_state')
    lifecycle_state, lifecycle_error = validate_health(
        payload.get('lifecycle_packet'), lifecycle_previous, now_monotonic_ns,
        expected_session, session_sha, run_token, proc_root,
        wire_sha256=payload.get('lifecycle_wire_sha256'),
        wire_text=payload.get('lifecycle_wire_text'))
    if lifecycle_error:
        return None, role + '_lifecycle_' + lifecycle_error
    if lifecycle_state != payload.get('lifecycle_state'):
        return None, role + '_lifecycle_reader_state_mismatch'
    continuity = monotonic_writer_continuity(
        payload, baseline, prior, required_topics)
    if continuity:
        return None, role + '_current_' + continuity
    if prior is not None:
        if (payload['ready_monotonic_ns'] < prior['ready_monotonic_ns'] or
                payload['ready_epoch_ns'] < prior['ready_epoch_ns']):
            return None, role + '_current_ready_time_rollback'
    if role == 'early':
        safety = payload.get('pre_arm_latest_safety')
        for field in ('pre_arm_nonzero_seen', 'pre_arm_active_seen'):
            if not isinstance(payload.get(field), bool):
                return None, 'invalid_early_current_handoff_flags'
            if (baseline.get(field) is True and payload[field] is not True) or (
                    prior is not None and prior.get(field) is True and
                    payload[field] is not True):
                return None, 'early_current_' + field + '_cleared'
        for field in ('pre_arm_unsafe_count', 'pre_arm_status_count',
                      'pre_arm_status_messages_received'):
            value = payload.get(field)
            base_value = baseline.get(field)
            prior_value = None if prior is None else prior.get(field)
            if (not integer(value, 1 if field != 'pre_arm_unsafe_count' else 0) or
                    not integer(base_value,
                                1 if field != 'pre_arm_unsafe_count' else 0) or
                    value < base_value or
                    (prior_value is not None and value < prior_value)):
                return None, 'early_current_' + field + '_rollback'
        if (not isinstance(safety, dict) or
                safety.get('status_publish_count') !=
                    payload['pre_arm_status_count'] or
                payload['writer_message_counts'].get('/safety/status_json') !=
                    payload['pre_arm_status_messages_received']):
            return None, 'invalid_early_current_safety_handoff'
    bag = pathlib.Path(payload['bag_dir'])
    if (bag.is_symlink() or not bag.is_dir() or
            bag.resolve() != expected_bag.resolve()):
        return None, 'unsafe_' + role + '_current_bag_path'
    if not arm_owner_alive(identity, proc_root, getpgid):
        return None, role + '_current_owner_exited_or_reused'
    return payload, None


def save_topic_rejection(run_out, role, payload, captured, error, required_topics,
                         now_monotonic_ns, now_epoch_ns, arm_payload):
    """Failure-only evidence from the exact bytes already consumed; never reread."""
    rows = []
    counts = payload.get('writer_message_counts', {})
    times = payload.get('last_message_monotonic_ns_by_topic', {})
    ready = payload.get('ready_monotonic_ns')
    for topic in required_topics:
        count, written = counts.get(topic), times.get(topic)
        branches = {
            'count_absent_or_nonpositive': not integer(count, 1),
            'source_timestamp_absent_or_nonpositive': not integer(written, 1),
            'source_timestamp_after_validation_time': integer(written, 1) and written > now_monotonic_ns,
            'source_timestamp_age_above_limit': integer(written, 1) and now_monotonic_ns - written > ARM_FRESH_NS,
            'source_timestamp_after_ready_snapshot': integer(written, 1) and written > ready,
        }
        rows.append({'topic': topic, 'count': count, 'source_monotonic_ns': written,
                     'ready_monotonic_ns': ready,
                     'age_ns': now_monotonic_ns - written if integer(written, 1) else None,
                     'predicates': branches})
    raw = bytes.fromhex(captured['raw_hex'])
    assert hashlib.sha256(raw).hexdigest() == captured['sha256']
    assert json.loads(raw.decode('utf-8')) == payload
    evidence = {'schema_version': 1, 'role': role, 'error': error,
                'validation_monotonic_ns': now_monotonic_ns,
                'validation_epoch_ns': now_epoch_ns,
                'recorded_monotonic_ns': time.monotonic_ns(),
                'freshness_limit_ns': ARM_FRESH_NS,
                'consumed_snapshot': captured, 'payload': payload,
                'owner_identity': payload.get('owner_identity'),
                'session': payload.get('formal_acceptance_session'),
                'session_sha256': payload.get('formal_acceptance_session_sha256'),
                'run_token': payload.get('run_token'), 'bag_dir': payload.get('bag_dir'),
                'immutable_arm_sha256': arm_payload.get('_observer_arm_sha256'),
                'topics': rows, 'formal_acceptance': False}
    # Exclusive directory reserves one failure receipt. A stale or partial directory
    # is never reused; rename publishes the complete, fsynced receipt atomically.
    destination = run_out / 'writer_readiness_rejection'
    destination.mkdir(exist_ok=False)
    pending = destination / 'receipt.pending.json'
    with pending.open('x', encoding='utf-8') as stream:
        json.dump(evidence, stream, indent=2, sort_keys=True)
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(pending, destination / 'receipt.json')


def validate_current_recording_readiness(run_out, arm_payload, prior=None,
                                         now_monotonic_ns=None,
                                         now_epoch_ns=None,
                                         proc_root=pathlib.Path('/proc'),
                                         getpgid=None):
    now_monotonic_ns = (time.monotonic_ns() if now_monotonic_ns is None else
                        now_monotonic_ns)
    now_epoch_ns = time.time_ns() if now_epoch_ns is None else now_epoch_ns
    map_dir = run_out / 'map'
    for invalid_name in ('formal_recording_invalid.json',
                         'formal_localization_recording_invalid.json'):
        invalid_path = map_dir / invalid_name
        if invalid_path.is_symlink() or invalid_path.exists():
            return None, 'formal_recording_current_invalidated'
    arm_path = map_dir / 'formal_recording_arm.json'
    if sha256_regular_file(arm_path) != arm_payload.get('_observer_arm_sha256'):
        return None, 'formal_recording_immutable_arm_changed'
    session_path = (pathlib.Path(EXPECTED_SESSION)
                    if isinstance(EXPECTED_SESSION, str) else None)
    expected_session_path = run_out / 'formal_acceptance_session.json'
    session_sha = (sha256_regular_file(session_path)
                   if session_path is not None else None)
    if (session_path != expected_session_path or
            session_sha != arm_payload.get('formal_acceptance_session_sha256') or
            not isinstance(EXPECTED_RUN_TOKEN, str) or
            EXPECTED_RUN_TOKEN != arm_payload.get('run_token')):
        return None, 'formal_recording_current_session_or_token_changed'
    early_path = map_dir / 'formal_recording_ready.json'
    local_path = map_dir / 'formal_localization_ready.json'
    early_capture, local_capture = {}, {}
    early, early_sha = read_regular_json(early_path, early_capture)
    local, local_sha = read_regular_json(local_path, local_capture)
    early_consumed, local_consumed = early, local
    if early is None:
        return None, 'formal_recording_current_early_ready_unreadable'
    if local is None:
        return None, 'formal_recording_current_localization_ready_unreadable'
    previous_early = None if prior is None else prior.get('early')
    previous_local = None if prior is None else prior.get('localization')
    early, error = validate_current_recording_role(
        early, 'early', arm_payload, previous_early, EXPECTED_SESSION,
        session_sha, EXPECTED_RUN_TOKEN, map_dir / 'early_recording_audit',
        ARM_REQUIRED_TOPICS, now_monotonic_ns, now_epoch_ns,
        proc_root, getpgid)
    if error:
        if '_current_topic_not_ready:' in error:
            try:
                save_topic_rejection(run_out, 'early', early_consumed, early_capture, error,
                                     ARM_REQUIRED_TOPICS, now_monotonic_ns, now_epoch_ns, arm_payload)
            except Exception as exc:
                print('REJECTION_EVIDENCE_WRITE_FAILED:' + repr(exc), file=sys.stderr)
        return None, 'formal_recording_' + error
    local, error = validate_current_recording_role(
        local, 'localization', arm_payload['localization_writer'],
        previous_local, EXPECTED_SESSION, session_sha, EXPECTED_RUN_TOKEN,
        map_dir / 'mapping_localization_diagnostic',
        LOCALIZATION_ARM_REQUIRED_TOPICS, now_monotonic_ns, now_epoch_ns,
        proc_root, getpgid)
    if error:
        if '_current_topic_not_ready:' in error:
            try:
                save_topic_rejection(run_out, 'localization', local_consumed, local_capture, error,
                                     LOCALIZATION_ARM_REQUIRED_TOPICS, now_monotonic_ns, now_epoch_ns, arm_payload)
            except Exception as exc:
                print('REJECTION_EVIDENCE_WRITE_FAILED:' + repr(exc), file=sys.stderr)
        return None, 'formal_recording_' + error
    if early['owner_identity']['pid'] == local['owner_identity']['pid']:
        return None, 'formal_recording_current_writer_roles_not_independent'
    return {'early': early, 'localization': local,
            'early_path': str(early_path), 'localization_path': str(local_path),
            'early_sha256': early_sha, 'localization_sha256': local_sha,
            'validated_monotonic_ns': now_monotonic_ns,
            'validated_epoch_ns': now_epoch_ns}, None


def current_recording_handoff(current_ready, arm_payload):
    early = current_ready['early']
    return {
        'cumulative_nonzero_seen': early['pre_arm_nonzero_seen'],
        'cumulative_active_seen': early['pre_arm_active_seen'],
        'cumulative_unsafe_count': early['pre_arm_unsafe_count'],
        'unsafe_count_since_arm': (early['pre_arm_unsafe_count'] -
                                   arm_payload['pre_arm_unsafe_count']),
        'latest_safety': early['pre_arm_latest_safety'],
        'status_publish_count': early['pre_arm_status_count'],
        'status_messages_received': early['pre_arm_status_messages_received']}


def current_ready_result_summary(current_ready, now_monotonic_ns=None,
                                 now_epoch_ns=None):
    now_monotonic_ns = (time.monotonic_ns() if now_monotonic_ns is None else
                        now_monotonic_ns)
    now_epoch_ns = time.time_ns() if now_epoch_ns is None else now_epoch_ns
    result = {}
    for role in ('early', 'localization'):
        value = current_ready[role]
        result[role] = {
            'path': current_ready[role + '_path'],
            'sha256': current_ready[role + '_sha256'],
            'formal_acceptance_session':
                value['formal_acceptance_session'],
            'formal_acceptance_session_sha256':
                value['formal_acceptance_session_sha256'],
            'run_token': value['run_token'],
            'bag_dir': value['bag_dir'],
            'owner_identity': value['owner_identity'],
            'writer_message_counts': value['writer_message_counts'],
            'clock_ns': value['clock_ns'],
            'clock_advances': value['clock_advances'],
            'ready_monotonic_ns': value['ready_monotonic_ns'],
            'ready_epoch_ns': value['ready_epoch_ns'],
            'ready_monotonic_age_ns': (now_monotonic_ns -
                                       value['ready_monotonic_ns']),
            'ready_epoch_age_ns': now_epoch_ns - value['ready_epoch_ns']}
    return result


def formal_recording_arm_runtime_failure(payload, run_out,
                                         proc_root=pathlib.Path('/proc'),
                                         getpgid=None):
    invalid_path = run_out / 'map' / 'formal_recording_invalid.json'
    localization_invalid_path = (
        run_out / 'map' / 'formal_localization_recording_invalid.json')
    if (invalid_path.is_symlink() or invalid_path.exists() or
            localization_invalid_path.is_symlink() or
            localization_invalid_path.exists()):
        return 'formal_recording_arm_invalidated'
    if not arm_owner_alive(payload['_observer_early_identity'],
                           proc_root, getpgid):
        return 'formal_recording_arm_owner_exited_or_reused'
    if not arm_owner_alive(payload['_observer_localization_identity'],
                           proc_root, getpgid):
        return 'formal_recording_arm_localization_owner_exited_or_reused'
    return None


def parse_json_payload(topic, msg):
    evidence_counts[topic] = evidence_counts.get(topic, 0) + 1
    emit('string_sample', {'topic': topic, 'raw': msg.data,
                           'source_stamp_available': False})
    try:
        payload = json.loads(msg.data)
    except (TypeError, ValueError, json.JSONDecodeError):
        fail('invalid_json:' + topic)
        return None
    if not isinstance(payload, dict):
        fail('nonobject_json:' + topic)
        return None
    return payload


def match_producer_cycle(cycle_id):
    diagnostic = producer_diagnostic_by_cycle.get(cycle_id)
    critical = critical_status_by_count.get(cycle_id)
    if diagnostic is None or critical is None:
        return
    if diagnostic['relay_conditions']['relay_enabled'] is not critical['relay_enabled']:
        fail('producer_same_cycle_relay_output_mismatch')
        return
    producer_diagnostics['matched_critical_cycles'].add(cycle_id)
    received = max(diagnostic['_observer_received'], critical['_observer_received'])
    for goal in reversed(goals):
        if (goal.get('active_wall') is not None and
                goal['active_wall'] <= received and
                (goal.get('closed_wall') is None or received <= goal['closed_wall'])):
            if cycle_id not in goal['producer_matched_cycle_ids']:
                goal['producer_matched_cycle_ids'].append(cycle_id)
            break


def on_critical_status(msg):
    topic = '/formal_vehicle/auxiliary/critical_safety_status_json'
    payload = parse_json_payload(topic, msg)
    if payload is None:
        return
    count = payload.get('publish_count')
    if (payload.get('schema_version') != 1 or not integer(count, 1) or
            not isinstance(payload.get('relay_enabled'), bool)):
        fail('invalid_critical_safety_status_contract')
        return
    if count in critical_status_by_count:
        fail('duplicate_critical_safety_publish_count')
        return
    payload['_observer_received'] = time.monotonic()
    critical_status_by_count[count] = payload
    match_producer_cycle(count)


def producer_input_ok(name, sample, cycle_monotonic):
    required = {'last_received_raw_value', 'effective_value', 'seen',
                'last_rx_monotonic_sec', 'age_sec', 'effective_timeout_sec',
                'fresh', 'sequence', 'seq'}
    if not isinstance(sample, dict) or not required <= set(sample):
        return False
    sequence = sample['sequence']
    timeout = sample['effective_timeout_sec']
    if (not integer(sequence) or sample['seq'] != sequence or
            not isinstance(sample['seen'], bool) or
            not isinstance(sample['fresh'], bool) or
            not finite((timeout,)) or float(timeout) <= 0.0):
        return False
    previous = producer_diagnostics['last_input_sequences'].get(name)
    if previous is not None and sequence < previous:
        return False
    producer_diagnostics['last_input_sequences'][name] = sequence
    effective = sample['effective_value']
    raw = sample['last_received_raw_value']
    if name == 'battery':
        if (not isinstance(effective, dict) or
                not finite((effective.get('soc'),)) or
                (effective.get('voltage_v') is not None and
                 not finite((effective.get('voltage_v'),)))):
            return False
        if raw is not None and (not isinstance(raw, dict) or
                                not finite((raw.get('soc'),)) or
                                (raw.get('voltage_v') is not None and
                                 not finite((raw.get('voltage_v'),)))):
            return False
    elif (not isinstance(effective, bool) or
          (raw is not None and not isinstance(raw, bool))):
        return False
    if not sample['seen']:
        return (sample['last_received_raw_value'] is None and
                sample['last_rx_monotonic_sec'] is None and
                sample['age_sec'] is None and not sample['fresh'])
    if (sample['last_received_raw_value'] is None or
            not finite((sample['last_rx_monotonic_sec'], sample['age_sec']))):
        return False
    age = float(sample['age_sec'])
    if not math.isclose(
            float(cycle_monotonic) - float(sample['last_rx_monotonic_sec']),
            age, rel_tol=0.0, abs_tol=1e-6):
        return False
    return sample['fresh'] is (0.0 <= age <= float(timeout))


def on_producer_relay_diagnostic(msg):
    topic = '/formal_vehicle/auxiliary/critical_safety_relay_diagnostic_json'
    payload = parse_json_payload(topic, msg)
    if payload is None:
        return
    cycle_id = payload.get('cycle_id')
    cycle_monotonic = payload.get('cycle_monotonic_sec')
    identity = process_identity_tuple(payload.get('producer'))
    if (payload.get('schema_version') != 1 or not integer(cycle_id, 1) or
            not finite((cycle_monotonic, payload.get('cycle_ros_time_ns'))) or
            identity is None or
            not valid_diagnostic_transport(payload.get('diagnostic_transport')) or
            not isinstance(payload.get('inputs'), dict) or
            not isinstance(payload.get('relay_conditions'), dict)):
        fail('invalid_producer_relay_diagnostic_contract')
        return
    inputs = payload['inputs']
    flat = {name: inputs.get(name) for name in (
        'emergency_stop', 'main_power', 'battery', 'main_isolator',
        'main_contactor')}
    charge = inputs.get('charge')
    if isinstance(charge, dict):
        flat['charge_requested'] = charge.get('requested')
        flat['charge_connected'] = charge.get('connected')
    required_conditions = {
        'battery_fresh', 'safety_power', 'operator_command_fresh',
        'main_power_requested', 'main_isolator_feedback_fresh',
        'main_isolator_closed', 'effective_main_power',
        'emergency_stop_inactive', 'charge_interlock_active',
        'main_contactor_feedback_fresh', 'main_contactor_closed',
        'relay_enabled'}
    prior_input_sequences = dict(producer_diagnostics['last_input_sequences'])
    if (set(flat) != {'emergency_stop', 'main_power', 'battery',
                      'main_isolator', 'main_contactor', 'charge_requested',
                      'charge_connected'} or
            not all(producer_input_ok(name, sample, cycle_monotonic)
                    for name, sample in flat.items()) or
            not required_conditions <= set(payload['relay_conditions']) or
            not all(isinstance(payload['relay_conditions'][name], bool)
                    for name in required_conditions)):
        fail('invalid_producer_relay_input_or_condition_contract')
        producer_diagnostics['last_input_sequences'] = prior_input_sequences
        return
    if not lock_process_identity('producer', identity):
        fail('producer_relay_diagnostic_identity_changed')
        return
    previous_cycle = producer_diagnostics['last_cycle_id']
    if previous_cycle is not None and cycle_id != previous_cycle + 1:
        fail('noncontiguous_producer_relay_diagnostic_cycle')
        return
    producer_diagnostics['last_cycle_id'] = cycle_id
    payload['_observer_received'] = time.monotonic()
    producer_diagnostics['received'] += 1
    producer_diagnostics['last_received'] = payload['_observer_received']
    producer_diagnostic_by_cycle[cycle_id] = payload
    match_producer_cycle(cycle_id)


def validate_edge_record(value):
    return (value is None or
            (isinstance(value, dict) and integer(value.get('sequence'), 1) and
             integer(value.get('unsafe_generation')) and
             isinstance(value.get('value'), bool) and
             finite((value.get('arrival_monotonic_sec'),))))


def is_preactive_bootstrap_base(actual, base_samples):
    pre_goal = current is None or current.get('outcome')
    manager_zero = (actual['linear_x'] == 0.0 and actual['angular_z'] == 0.0)
    samples_are_zero = all(
        not sample['nonzero'] and sample['linear_x'] == 0.0 and
        sample['angular_z'] == 0.0 for sample in base_samples)
    return (pre_goal and not global_motion_active and
            actual['effective_permit'] is False and manager_zero and
            samples_are_zero)


def match_manager_cycle(payload):
    cycle_id = payload['cycle_id']
    if payload['event'] == 'periodic_publish':
        count = payload['status_publish_count']
        status = safety_status_by_count.get(count)
    else:
        status = None
    if status is not None:
        decision = payload['same_cycle_decision']
        reasons = ','.join(decision['active_reasons'])
        actual = payload['base_command_publish']
        if (decision['state'] != status['state'] or
                reasons != status['active_reasons'] or
                actual['effective_permit'] is not status['actuators_enabled']):
            fail('manager_same_cycle_status_mismatch')
            return
        manager_diagnostics['matched_status_cycles'].add(cycle_id)
    actual = payload['base_command_publish']
    base_samples = base_command_by_stamp.get(actual['header_stamp_ns'], [])
    if is_preactive_bootstrap_base(actual, base_samples):
        cycle_id = payload['cycle_id']
        if cycle_id not in manager_diagnostics['bootstrap_noncausal_cycles']:
            manager_diagnostics['bootstrap_noncausal_cycles'].add(cycle_id)
            emit('preactive_bootstrap_base_not_causally_matched', {
                'cycle_id': cycle_id,
                'status_publish_count': payload['status_publish_count'],
                'header_stamp_ns': actual['header_stamp_ns'],
                'sample_count': len(base_samples),
                'nonunique': len(base_samples) != 1,
                'effective_permit': False,
                'command': {'linear_x': actual['linear_x'],
                            'angular_z': actual['angular_z']},
                'classification': 'RECORDED_NONCAUSAL_BOOTSTRAP'})
        return
    if len(base_samples) > 1:
        fail('ambiguous_repeated_base_source_stamp')
        return
    if len(base_samples) == 1:
        base = base_samples[0]
        prior_cycle = base_matched_stamps.get(actual['header_stamp_ns'])
        if prior_cycle is not None and prior_cycle != cycle_id:
            fail('base_source_stamp_matches_multiple_manager_cycles')
            return
        if (base['frame'] != actual['frame_id'] or
                not math.isclose(base['linear_x'], actual['linear_x'],
                                 rel_tol=0.0, abs_tol=1e-12) or
                not math.isclose(base['angular_z'], actual['angular_z'],
                                 rel_tol=0.0, abs_tol=1e-12)):
            fail('manager_same_cycle_base_output_mismatch')
            return
        base_matched_stamps[actual['header_stamp_ns']] = cycle_id
        manager_diagnostics['matched_base_cycles'].add(cycle_id)
        if base['nonzero']:
            if actual['effective_permit'] is not True:
                fail('manager_nonzero_base_without_effective_permit')
                return
            manager_diagnostics['active_output_cycles'].add(cycle_id)
            goal_number = base.get('goal_number')
            if integer(goal_number, 1) and goal_number <= len(goals):
                cycles = goals[goal_number - 1]['diagnostic_active_cycle_ids']
                if cycle_id not in cycles:
                    cycles.append(cycle_id)


def on_manager_relay_diagnostic(msg):
    topic = '/safety/relay_cycle_diagnostic_json'
    payload = parse_json_payload(topic, msg)
    if payload is None:
        return
    cycle_id = payload.get('cycle_id')
    event = payload.get('event')
    actual = payload.get('base_command_publish')
    callback_sequence = payload.get('callback_sequence')
    callback_value = payload.get('last_callback_value')
    callback_monotonic = payload.get('last_callback_monotonic_sec')
    identity = process_identity_tuple(payload.get('producer'))
    if (payload.get('schema_version') != 1 or
            event not in ('periodic_publish', 'immediate_stop',
                          'immediate_base_stop') or
            not integer(cycle_id, 1) or
            not finite((payload.get('evaluation_monotonic_sec'),
                        payload.get('evaluation_ros_time_ns'))) or
            identity is None or
            not valid_diagnostic_transport(payload.get('diagnostic_transport')) or
            not integer(payload.get('status_publish_count')) or
            not integer(callback_sequence) or
            not isinstance(callback_value, (bool, type(None))) or
            ((callback_sequence == 0) != (callback_value is None)) or
            ((callback_sequence == 0 and callback_monotonic is not None) or
             (callback_sequence > 0 and not finite((callback_monotonic,)))) or
            not integer(payload.get('last_callback_unsafe_generation')) or
            not isinstance(payload.get('last_callback_was_unsafe_edge'), bool) or
            not validate_edge_record(payload.get('last_false_callback')) or
            not validate_edge_record(payload.get('last_recovery_callback')) or
            not integer(payload.get('consumed_unsafe_generation')) or
            not isinstance(actual, dict) or
            actual.get('frame_id') != 'base_footprint' or
            not integer(actual.get('header_stamp_ns')) or
            not finite((actual.get('linear_x'), actual.get('angular_z'))) or
            not isinstance(actual.get('effective_permit'), bool)):
        fail('invalid_manager_relay_diagnostic_contract')
        return
    last_false = payload.get('last_false_callback')
    last_recovery = payload.get('last_recovery_callback')
    if ((last_false is not None and
         (last_false['value'] is not False or
          last_false['sequence'] > callback_sequence)) or
            (last_recovery is not None and
             (last_recovery['value'] is not True or
              last_recovery['sequence'] > callback_sequence))):
        fail('invalid_manager_relay_edge_history')
        return
    trigger = payload.get('relay_trigger_snapshot')
    if trigger is not None and (
            event != 'immediate_stop' or not validate_edge_record(trigger) or
            trigger['value'] is not False or
            trigger['sequence'] > callback_sequence):
        fail('invalid_manager_relay_trigger_snapshot')
        return
    if event == 'periodic_publish':
        decision = payload.get('same_cycle_decision')
        command = decision.get('base_publish_command') if isinstance(decision, dict) else None
        if (not isinstance(decision, dict) or
                not isinstance(decision.get('state'), str) or
                not isinstance(decision.get('active_reasons'), list) or
                not all(isinstance(reason, str) for reason in decision['active_reasons']) or
                not isinstance(decision.get('base_command_enabled'), bool) or
                not isinstance(command, dict) or
                not finite((command.get('linear_x'), command.get('angular_z'))) or
                not math.isclose(command['linear_x'], actual['linear_x'],
                                 rel_tol=0.0, abs_tol=1e-12) or
                not math.isclose(command['angular_z'], actual['angular_z'],
                                 rel_tol=0.0, abs_tol=1e-12)):
            fail('invalid_manager_same_cycle_decision_output_contract')
            return
        if payload['status_publish_count'] in manager_diagnostic_by_status_count:
            fail('duplicate_manager_status_publish_count_diagnostic')
            return
    elif (actual['linear_x'] != 0.0 or actual['angular_z'] != 0.0 or
          actual['effective_permit'] is not False):
        fail('invalid_manager_immediate_zero_output_contract')
        return
    if not lock_process_identity('manager', identity):
        fail('manager_relay_diagnostic_identity_changed')
        return
    previous_cycle = manager_diagnostics['last_cycle_id']
    if previous_cycle is not None and cycle_id != previous_cycle + 1:
        fail('noncontiguous_manager_relay_diagnostic_cycle')
        return
    manager_diagnostics['last_cycle_id'] = cycle_id
    if event == 'periodic_publish':
        manager_diagnostic_by_status_count[payload['status_publish_count']] = payload
    payload['_observer_received'] = time.monotonic()
    manager_diagnostics['received'] += 1
    manager_diagnostics['last_received'] = payload['_observer_received']
    manager_diagnostic_by_cycle[cycle_id] = payload
    match_manager_cycle(payload)


def on_battery(msg):
    topic = '/formal_vehicle/power/battery_state'
    evidence_counts[topic] = evidence_counts.get(topic, 0) + 1
    emit('battery_state_sample', {
        'topic': topic, 'source_stamp_ns': stamp_ns(msg.header.stamp),
        'frame': msg.header.frame_id,
        'voltage': json_float(msg.voltage),
        'temperature': json_float(msg.temperature),
        'current': json_float(msg.current), 'charge': json_float(msg.charge),
        'capacity': json_float(msg.capacity),
        'design_capacity': json_float(msg.design_capacity),
        'percentage': json_float(msg.percentage),
        'power_supply_status': int(msg.power_supply_status),
        'power_supply_health': int(msg.power_supply_health),
        'power_supply_technology': int(msg.power_supply_technology),
        'present': bool(msg.present),
        'cell_voltage': [json_float(v) for v in msg.cell_voltage],
        'cell_temperature': [json_float(v) for v in msg.cell_temperature],
        'location': msg.location, 'serial_number': msg.serial_number})


def planar_yaw(q):
    if not finite(q):
        return None
    qx, qy, qz, qw = map(float, q)
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if abs(norm - 1.0) > 1e-6 or abs(qx) > 1e-6 or abs(qy) > 1e-6:
        return None
    return math.atan2(2.0 * (qw*qz + qx*qy), 1.0 - 2.0 * (qy*qy + qz*qz))


def command_nonzero(linear, angular):
    return float(linear) != 0.0 or float(angular) != 0.0


def safety_kind(payload):
    common = (payload.get('safety_inputs_permit_actuators') is True and
              payload.get('actuators_enabled') is True and
              payload.get('managed_controllers_active') is True)
    if (common and payload.get('state') == 'ENABLED' and
            payload.get('active_reasons') == ''):
        return 'SAFE'
    if (common and payload.get('state') == 'BASE_COMMAND_STOPPED' and
            payload.get('active_reasons') == 'command_timeout'):
        return 'STARTUP_COMMAND_TIMEOUT'
    return 'UNSAFE'


def safety_transition(phase, payload, status_count_before_base, recent_zero_base,
                      global_active):
    kind = safety_kind(payload)
    count = payload['status_publish_count']
    if global_active:
        if kind != 'SAFE':
            return 'ACTIVE', 'unsafe_status_after_global_motion_active'
        return 'ACTIVE', None
    if phase in ('ARMING_ZERO_OUTPUT', 'PROPAGATING'):
        if kind == 'SAFE':
            return phase, None
        if kind == 'STARTUP_COMMAND_TIMEOUT' and recent_zero_base:
            return phase, None
        return phase, 'unsafe_or_unproven_zero_output_during_arming'
    if phase == 'ACTIVE':
        if kind != 'SAFE':
            return phase, 'unsafe_status_during_active_motion_window'
        return phase, None
    return phase, 'invalid_goal_safety_phase'


def safety_deadline_failure(goal, now):
    nav_wall = goal.get('first_nonzero_nav_wall')
    smoothed_wall = goal.get('first_nonzero_smoothed_wall')
    gate_wall = goal.get('first_nonzero_gate_wall')
    base_wall = goal.get('first_nonzero_base_wall')
    active_wall = goal.get('active_wall')
    chain = (('nav', nav_wall), ('smoothed', smoothed_wall),
             ('gate', gate_wall), ('base', base_wall))
    for index in range(1, len(chain)):
        upstream_name, upstream_wall = chain[index - 1]
        downstream_name, downstream_wall = chain[index]
        if (downstream_wall is not None and upstream_wall is None and
                now - downstream_wall > CHAIN_RECEIPT_REORDER_TOLERANCE_S):
            return ('nonzero_' + downstream_name + '_without_observed_nonzero_' +
                    upstream_name)
        if (upstream_wall is not None and downstream_wall is not None and
                upstream_wall - downstream_wall > CHAIN_RECEIPT_REORDER_TOLERANCE_S):
            return 'nonzero_command_chain_receipt_order_violation'
    if nav_wall is not None and now - nav_wall > FRESH_WALL_S:
        if smoothed_wall is None:
            return 'nonzero_nav_without_nonzero_smoothed_within_2s'
        if gate_wall is None:
            return 'nonzero_nav_without_nonzero_gate_within_2s'
        if base_wall is None:
            return 'nonzero_nav_without_nonzero_base_within_2s'
        if active_wall is None:
            return 'nonzero_nav_without_safe_active_base_within_2s'
        if any(wall - nav_wall > FRESH_WALL_S
               for wall in (smoothed_wall, gate_wall, base_wall, active_wall)):
            return 'safe_active_base_missed_2s_nav_chain_deadline'
    if (base_wall is not None and now - base_wall > FRESH_WALL_S and
            not goal.get('post_active_status_verified')):
        return 'nonzero_base_without_new_safe_status_within_2s'
    return None


def window_for_goal(raw, goal_x, goal_y, now_clock_ns):
    if raw is None or now_clock_ns is None:
        return None
    if raw['frame'] != 'map' or raw['stamp_ns'] in (None, 0):
        return None
    if raw['width'] <= 0 or raw['height'] <= 0 or raw['res'] <= 0:
        return None
    if raw['data_len'] != raw['width'] * raw['height']:
        return None
    age = (now_clock_ns - raw['stamp_ns']) * 1e-9
    yaw = planar_yaw(raw['q'])
    if yaw is None or not 0.0 <= age <= FRESH_SIM_S:
        return None
    if not finite((goal_x, goal_y, *raw['origin'])):
        return None
    dx, dy = float(goal_x) - raw['origin'][0], float(goal_y) - raw['origin'][1]
    local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
    local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
    inside = (INSET_M <= local_x < raw['width'] * raw['res'] - INSET_M and
              INSET_M <= local_y < raw['height'] * raw['res'] - INSET_M)
    return {'sequence': raw['sequence'], 'stamp_ns': raw['stamp_ns'],
            'source_age_sim_s': age, 'local_xy': [local_x, local_y],
            'inside_half_meter_inset': inside, 'window': dict(raw)}


def active_executing_ids():
    if action_snapshot is None:
        return []
    return [row['uuid'] for row in action_snapshot['rows']
            if row['status'] == GoalStatus.STATUS_EXECUTING]


def bind_execution_if_possible():
    if current is None or current.get('outcome') or current.get('goal_id') is None:
        return
    executing = active_executing_ids()
    if len(executing) > 1:
        fail('multiple_executing_navigate_to_pose_goals')
    elif executing == [current['goal_id']] and current['executing_wall'] is None:
        current['executing_wall'] = action_snapshot['received']
        current['executing_status_receipt_wall'] = action_snapshot['received']
        emit('goal_executing_bound', {'number': current['number'],
                                      'goal_id': current['goal_id'],
                                      'status_receipt_wall_s':
                                          action_snapshot['received'] - start})


def on_clock(msg):
    global clock_ns
    value = stamp_ns(msg.clock)
    if value is None:
        fail('invalid_clock_stamp')
    else:
        clock_ns = value
        evidence_counts['/clock'] = evidence_counts.get('/clock', 0) + 1
        emit('clock_sample', {'source_stamp_ns': value})


def on_grid(msg):
    global window, window_sequence
    window_sequence += 1
    origin = msg.info.origin
    window = {'sequence': window_sequence, 'frame': msg.header.frame_id,
              'stamp_ns': stamp_ns(msg.header.stamp),
              'origin': [float(origin.position.x), float(origin.position.y)],
              'q': [float(origin.orientation.x), float(origin.orientation.y),
                    float(origin.orientation.z), float(origin.orientation.w)],
              'width': int(msg.info.width), 'height': int(msg.info.height),
              'res': float(msg.info.resolution), 'data_len': len(msg.data),
              'received': time.monotonic()}
    emit('costmap_window', window)
    if current and not current.get('outcome') and current['post_window'] is None:
        if window['received'] > current['requested_wall']:
            checked = window_for_goal(window, current['request']['x'],
                                      current['request']['y'], clock_ns)
            if checked is None or not checked['inside_half_meter_inset']:
                fail('invalid_or_outside_post_request_costmap_bracket')
            else:
                current['post_window'] = checked
                emit('post_request_costmap_bracket',
                     {'number': current['number'], 'check': checked})


def on_odom(msg):
    global odom
    source_stamp = stamp_ns(msg.header.stamp)
    xy = [float(msg.pose.pose.position.x), float(msg.pose.pose.position.y)]
    pose, twist = msg.pose.pose, msg.twist.twist
    evidence_counts['/odom'] = evidence_counts.get('/odom', 0) + 1
    emit('odom_sample', {
        'topic': '/odom', 'source_stamp_ns': source_stamp,
        'frame': msg.header.frame_id, 'child_frame': msg.child_frame_id,
        'pose': {'position': [json_float(pose.position.x),
                              json_float(pose.position.y),
                              json_float(pose.position.z)],
                 'orientation': [json_float(pose.orientation.x),
                                 json_float(pose.orientation.y),
                                 json_float(pose.orientation.z),
                                 json_float(pose.orientation.w)]},
        'pose_covariance': [json_float(v) for v in msg.pose.covariance],
        'twist': {'linear': [json_float(twist.linear.x), json_float(twist.linear.y),
                             json_float(twist.linear.z)],
                  'angular': [json_float(twist.angular.x), json_float(twist.angular.y),
                              json_float(twist.angular.z)]},
        'twist_covariance': [json_float(v) for v in msg.twist.covariance]})
    if (msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_footprint' or
            source_stamp in (None, 0) or not finite(xy)):
        if current and not current.get('outcome'):
            fail('invalid_odom_contract')
        return
    odom = {'xy': xy, 'stamp_ns': source_stamp, 'received': time.monotonic()}
    if current and not current.get('outcome') and current['odom_start'] is None:
        current['odom_start'] = dict(odom)


def on_safety(msg):
    global safety, last_safety_publish_count
    evidence_counts['/safety/status_json'] = (
        evidence_counts.get('/safety/status_json', 0) + 1)
    emit('safety_status_raw', {'topic': '/safety/status_json', 'raw': msg.data,
                               'source_stamp_available': False})
    try:
        parsed = json.loads(msg.data)
    except (TypeError, ValueError, json.JSONDecodeError):
        fail('invalid_safety_json')
        return
    parsed['received'] = time.monotonic()
    count = parsed.get('status_publish_count')
    if (isinstance(count, bool) or not isinstance(count, int) or count <= 0 or
            (last_safety_publish_count is not None and
             count <= last_safety_publish_count)):
        fail('invalid_or_nonincreasing_safety_status_publish_count')
        return
    last_safety_publish_count = count
    parsed['classification'] = safety_kind(parsed)
    parsed['goal_number'] = current['number'] if current else None
    parsed['goal_safety_phase'] = current.get('safety_phase') if current else None
    emit('safety_status', parsed)
    safety = parsed
    safety_status_by_count[count] = parsed
    pending_manager = manager_diagnostic_by_status_count.get(count)
    if pending_manager is not None:
        match_manager_cycle(pending_manager)
    if current and not current.get('outcome'):
        current['safety_samples'] += 1
        recent_zero = (base_command is not None and
                       parsed['received'] - base_command['received'] <= FRESH_WALL_S and
                       not base_command['nonzero'])
        next_phase, error = safety_transition(
            current['safety_phase'], parsed,
            current['status_count_before_base'], recent_zero,
            global_motion_active)
        if error:
            current['failing_safety_payload'] = dict(parsed)
            fail(error)
            return
        if parsed['classification'] == 'STARTUP_COMMAND_TIMEOUT':
            current['allowed_startup_command_timeout_samples'] += 1
        current['safety_phase'] = next_phase
        if next_phase == 'ACTIVE':
            current['active_safety_samples'] += 1
            if (current['status_count_before_base'] is not None and
                    count > current['status_count_before_base']):
                current['post_active_status_verified'] = True
    elif global_motion_active:
        _, error = safety_transition('ACTIVE', parsed, None, False, True)
        if error:
            fail('goal_gap_' + error)


def on_gate_command(msg):
    global gate_command
    now = time.monotonic()
    values = (float(msg.linear.x), float(msg.angular.z))
    raw_nonzero = command_nonzero(*values) if finite(values) else None
    evidence_counts['/cmd_vel_gate'] = evidence_counts.get('/cmd_vel_gate', 0) + 1
    emit('twist_sample', {
        'topic': '/cmd_vel_gate',
        'linear': [json_float(msg.linear.x), json_float(msg.linear.y),
                   json_float(msg.linear.z)],
        'angular': [json_float(msg.angular.x), json_float(msg.angular.y),
                    json_float(msg.angular.z)],
        'nonzero': raw_nonzero, 'source_stamp_available': False})
    if not finite(values):
        if current and not current.get('outcome'):
            fail('invalid_gate_command')
        return
    nonzero = command_nonzero(*values)
    gate_command = {'linear_x': values[0], 'angular_z': values[1],
                    'nonzero': nonzero, 'received': now}
    if (current and not current.get('outcome') and nonzero and
            current['first_nonzero_gate_wall'] is None):
        current['first_nonzero_gate_wall'] = now
        current['first_nonzero_gate'] = dict(gate_command)
        emit('first_nonzero_gate_command',
             {'number': current['number'], 'command': gate_command})


def on_base_command(msg):
    global base_command, pre_goal_invalid_base_samples
    global global_motion_active, global_active_wall
    now = time.monotonic()
    source_stamp = stamp_ns(msg.header.stamp)
    values = (float(msg.twist.linear.x), float(msg.twist.angular.z))
    raw_nonzero = command_nonzero(*values) if finite(values) else None
    evidence_counts['/base_controller/cmd_vel'] = (
        evidence_counts.get('/base_controller/cmd_vel', 0) + 1)
    emit('twist_stamped_sample', {
        'topic': '/base_controller/cmd_vel', 'source_stamp_ns': source_stamp,
        'frame': msg.header.frame_id,
        'linear': [json_float(msg.twist.linear.x), json_float(msg.twist.linear.y),
                   json_float(msg.twist.linear.z)],
        'angular': [json_float(msg.twist.angular.x), json_float(msg.twist.angular.y),
                    json_float(msg.twist.angular.z)],
        'nonzero': raw_nonzero})
    if (msg.header.frame_id != 'base_footprint' or source_stamp in (None, 0) or
            not finite(values)):
        if current and not current.get('outcome'):
            fail('invalid_base_command_contract')
        else:
            pre_goal_invalid_base_samples += 1
            if pre_goal_invalid_base_samples == 1:
                emit('pre_goal_base_not_fresh_evidence', {
                    'frame': msg.header.frame_id, 'stamp_ns': source_stamp,
                    'linear_x': values[0], 'angular_z': values[1],
                    'reason': 'recorded_but_not_used_as_recent_zero_base'})
        return
    nonzero = command_nonzero(*values)
    base_command = {'linear_x': values[0], 'angular_z': values[1],
                    'nonzero': nonzero, 'received': now,
                    'frame': msg.header.frame_id, 'stamp_ns': source_stamp,
                    'goal_number': (current['number'] if current and
                                    not current.get('outcome') else None)}
    samples = base_command_by_stamp.setdefault(source_stamp, [])
    samples.append(dict(base_command))
    if len(samples) > 1 and source_stamp in base_matched_stamps:
        fail('repeated_base_source_stamp_after_same_cycle_match')
        return
    for pending_manager in manager_diagnostic_by_status_count.values():
        if pending_manager['base_command_publish']['header_stamp_ns'] == source_stamp:
            match_manager_cycle(pending_manager)
    if nonzero and not global_motion_active:
        global_motion_active = True
        global_active_wall = now
        emit('global_motion_active_latched', {
            'binding': 'first_exact_nonzero_safety_manager_base_output',
            'goal_number': current['number'] if current else None,
            'command': dict(base_command)})
    if current and not current.get('outcome'):
        if not nonzero and current['first_nonzero_base_wall'] is None:
            current['arming_zero_base_samples'] += 1
        elif nonzero and current['first_nonzero_base_wall'] is None:
            current['first_nonzero_base_wall'] = now
            current['first_nonzero_base'] = dict(base_command)
            current['status_count_before_base'] = (
                safety.get('status_publish_count') if safety else None)
            current['safety_phase'] = 'ACTIVE'
            current['command_chain_phase'] = 'ACTIVE'
            current['active_wall'] = now
            emit('first_nonzero_base_command', {
                'number': current['number'], 'command': base_command,
                'status_count_before_base': current['status_count_before_base']})
            emit('goal_safety_active', {
                'number': current['number'],
                'binding': 'first_nonzero_safety_manager_base_output',
                'first_nonzero_nav_wall_s':
                    (None if current['first_nonzero_nav_wall'] is None else
                     current['first_nonzero_nav_wall'] - start),
                'first_nonzero_smoothed_wall_s':
                    (None if current['first_nonzero_smoothed_wall'] is None else
                     current['first_nonzero_smoothed_wall'] - start),
                'first_nonzero_gate_wall_s':
                    (None if current['first_nonzero_gate_wall'] is None else
                     current['first_nonzero_gate_wall'] - start),
                'first_nonzero_base_wall_s': now - start})


def on_action_status(msg):
    global action_snapshot, action_signature
    rows = []
    for row in msg.status_list:
        uuid = bytes(row.goal_info.goal_id.uuid).hex()
        if not uuid or uuid == '00' * 16:
            fail('zero_uuid_in_action_status')
            return
        rows.append({'uuid': uuid, 'status': int(row.status),
                     'goal_stamp_ns': stamp_ns(row.goal_info.stamp)})
    rows.sort(key=lambda row: (row['uuid'], row['status']))
    action_snapshot = {'received': time.monotonic(), 'rows': rows}
    evidence_counts['/navigate_to_pose/_action/status'] = (
        evidence_counts.get('/navigate_to_pose/_action/status', 0) + 1)
    emit('action_status_sample', action_snapshot)
    signature = tuple((row['uuid'], row['status']) for row in rows)
    if signature != action_signature:
        action_signature = signature
        emit('action_status', action_snapshot)
    bind_execution_if_possible()


def progress_ok(goal):
    return (goal['first_positive'] is not None and
            goal['decrease_m'] >= MIN_PROGRESS_M and
            goal['odom_displacement_m'] >= MIN_PROGRESS_M and
            goal['safety_samples'] > 0 and goal['post_window'] is not None and
            goal['executing_wall'] is not None and
            goal['safety_phase'] == 'ACTIVE' and
            goal['active_safety_samples'] > 0 and
            goal['post_active_status_verified'] and
            bool(goal['producer_matched_cycle_ids']) and
            bool(goal['diagnostic_active_cycle_ids']))


def finish_explorer_result(data):
    global successes, consecutive_failures
    if current is None or current.get('outcome'):
        fail('unmatched_or_duplicate_explorer_result')
        return
    state = data.get('state')
    if state == 'frontier_goal_reached':
        if current['cancel_intent']:
            fail('natural_success_after_test_cancel_intent')
            return
        current['outcome_kind'] = 'NATURAL_SUCCEEDED'
        successes += 1
        consecutive_failures = 0
        if (data.get('goals_succeeded') != successes or
                data.get('goals_requested') != len(goals)):
            fail('success_counters_inconsistent')
    elif state == 'frontier_goal_failed':
        if not current['cancel_accepted']:
            if current['cancel_intent']:
                current['pending_explorer_result'] = data
                emit('explorer_result_pending_cancel_response', data)
                return
            fail('natural_navigation_failure')
            return
        if (data.get('status') != GoalStatus.STATUS_CANCELED or
                data.get('cancel_reason') is not None):
            fail('test_cancel_not_reported_as_external_canceled')
            return
        current['outcome_kind'] = 'TEST_INDUCED_CANCELED'
        consecutive_failures += 1
        if data.get('failures') != consecutive_failures:
            fail('failure_counter_inconsistent')
    else:
        fail('unknown_explorer_result_state')
        return
    if not progress_ok(current):
        fail('goal_without_measured_progress_motion_safety_or_window_bracket')
        return
    current['initial_zero_observation'] = (
        'OBSERVED' if current['initial_zero_count'] > 0 else 'NOT_OBSERVED')
    current['outcome'] = data
    current['closed_wall'] = time.monotonic()
    emit('goal_closed', {'number': current['number'],
                         'outcome_kind': current['outcome_kind'], 'status': data})


def on_explorer_status(msg):
    global current
    try:
        data = json.loads(msg.data)
    except (TypeError, ValueError, json.JSONDecodeError):
        fail('invalid_explorer_status_json')
        return
    emit('explorer_status', data)
    state = data.get('state')
    if state == 'frontier_goal_requested':
        if current and not current.get('outcome'):
            fail('overlapping_goal_requests')
            return
        if len(goals) >= 3:
            fail('unexpected_fourth_goal_before_probe_stop')
            return
        if (data.get('goals_requested') != len(goals) + 1 or
                data.get('goals_succeeded') != successes):
            fail('request_counters_inconsistent')
            return
        if not finite((data.get('x'), data.get('y'))):
            fail('invalid_frontier_goal_coordinates')
            return
        before = window_for_goal(window, data['x'], data['y'], clock_ns)
        if before is None or not before['inside_half_meter_inset']:
            fail('goal_outside_fresh_pre_request_costmap_bracket')
            return
        current = {'number': len(goals) + 1, 'request': data,
                   'requested_wall': time.monotonic(), 'pre_window': before,
                   'post_window': None,
                   'window_binding': 'observer_bracket_not_internal_selection_snapshot',
                   'goal_id': None, 'executing_wall': None,
                   'executing_status_receipt_wall': None,
                   'first_feedback_remaining': None, 'initial_zero_count': 0,
                   'zero_after_positive_count': 0, 'first_positive': None,
                   'minimum_remaining': None, 'last_remaining': None,
                   'decrease_m': 0.0, 'feedback_count': 0,
                   'last_feedback_wall': None, 'last_feedback_stamp_ns': None,
                   'odom_start': dict(odom) if odom else None,
                   'odom_displacement_m': 0.0, 'safety_samples': 0,
                    'safety_phase': ('ACTIVE' if global_motion_active else
                                     'ARMING_ZERO_OUTPUT'),
                    'command_chain_phase': 'WAITING_NAV',
                    'global_motion_active_at_open': global_motion_active,
                    'allowed_startup_command_timeout_samples': 0,
                    'arming_zero_base_samples': 0,
                    'first_nonzero_nav_wall': None, 'first_nonzero_nav': None,
                    'first_nonzero_smoothed_wall': None,
                    'first_nonzero_smoothed': None,
                    'first_nonzero_gate_wall': None, 'first_nonzero_gate': None,
                   'first_nonzero_base_wall': None, 'first_nonzero_base': None,
                   'status_count_before_base': None, 'active_wall': None,
                   'active_safety_samples': 0,
                    'post_active_status_verified': False,
                    'producer_matched_cycle_ids': [],
                    'diagnostic_active_cycle_ids': [],
                    'failing_safety_payload': None,
                    'failing_relay_payload': None,
                   'cancel_intent': False, 'cancel_requested_wall': None,
                   'cancel_accepted': False, 'cancel_response': None,
                   'pending_explorer_result': None, 'outcome': None,
                   'outcome_kind': None}
        goals.append(current)
        emit('goal_opened', {'number': current['number'], 'request': data,
                             'pre_window': before,
                             'safety_phase': current['safety_phase'],
                             'global_motion_active': global_motion_active})
    elif state in ('frontier_goal_reached', 'frontier_goal_failed'):
        finish_explorer_result(data)
    elif state and (state.startswith('blocked') or 'timeout' in state or
                    state in ('frontier_goal_rejected', 'frontier_goal_response_error',
                              'frontier_cancel_requested')):
        fail('explorer_failure:' + state)


def on_feedback(msg):
    if current is None or current.get('outcome'):
        return
    goal = current
    uuid = bytes(msg.goal_id.uuid).hex()
    if not uuid or uuid == '00' * 16:
        fail('zero_feedback_uuid')
        return
    if goal['goal_id'] is None:
        goal['goal_id'] = uuid
        emit('feedback_uuid_bound', {'number': goal['number'], 'goal_id': uuid})
        bind_execution_if_possible()
    if uuid != goal['goal_id']:
        fail('ambiguous_feedback_uuid')
        return
    feedback = msg.feedback
    source_stamp = stamp_ns(feedback.current_pose.header.stamp)
    pose = feedback.current_pose.pose
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    evidence_counts['/navigate_to_pose/_action/feedback'] = (
        evidence_counts.get('/navigate_to_pose/_action/feedback', 0) + 1)
    emit('action_feedback_sample', {
        'topic': '/navigate_to_pose/_action/feedback', 'goal_id': uuid,
        'source_stamp_ns': source_stamp,
        'frame': feedback.current_pose.header.frame_id,
        'current_pose': {
            'position': [json_float(pose.position.x), json_float(pose.position.y),
                         json_float(pose.position.z)],
            'orientation': [json_float(v) for v in q]},
        'navigation_time_ns': duration_ns(feedback.navigation_time),
        'estimated_time_remaining_ns':
            duration_ns(feedback.estimated_time_remaining),
        'number_of_recoveries': int(feedback.number_of_recoveries),
        'distance_remaining': json_float(feedback.distance_remaining)})
    if (feedback.current_pose.header.frame_id != 'map' or source_stamp in (None, 0) or
            clock_ns is None or
            not 0.0 <= (clock_ns - source_stamp) * 1e-9 <= FRESH_SIM_S or
            not finite((pose.position.x, pose.position.y, pose.position.z, *q)) or
            planar_yaw(q) is None):
        fail('invalid_or_stale_feedback_pose')
        return
    remaining = float(feedback.distance_remaining)
    if not math.isfinite(remaining) or remaining < 0.0:
        fail('invalid_remaining_distance')
        return
    if goal['first_feedback_remaining'] is None:
        goal['first_feedback_remaining'] = remaining
    if goal['first_positive'] is None:
        if remaining == 0.0:
            goal['initial_zero_count'] += 1
            emit('initial_zero_feedback', {'number': goal['number'], 'goal_id': uuid,
                                           'pose_stamp_ns': source_stamp,
                                           'ordinal': goal['initial_zero_count']})
        else:
            goal['first_positive'] = remaining
    elif remaining == 0.0:
        goal['zero_after_positive_count'] += 1
    if goal['first_positive'] is not None:
        goal['minimum_remaining'] = (remaining if goal['minimum_remaining'] is None
                                     else min(goal['minimum_remaining'], remaining))
        goal['decrease_m'] = max(goal['decrease_m'],
                                 goal['first_positive'] - remaining)
    now = time.monotonic()
    if odom and goal['odom_start'] and now - odom['received'] <= FRESH_WALL_S:
        goal['odom_displacement_m'] = max(
            goal['odom_displacement_m'], math.dist(odom['xy'], goal['odom_start']['xy']))
    goal['feedback_count'] += 1
    goal['last_remaining'] = remaining
    goal['last_feedback_wall'] = now
    goal['last_feedback_stamp_ns'] = source_stamp
    if now - goal.get('last_feedback_emit_wall', 0.0) >= 1.0:
        goal['last_feedback_emit_wall'] = now
        emit('feedback', {'number': goal['number'], 'goal_id': uuid,
                          'remaining_m': remaining, 'pose_stamp_ns': source_stamp,
                          'initial_zero_count': goal['initial_zero_count'],
                          'decrease_m': goal['decrease_m'],
                          'odom_displacement_m': goal['odom_displacement_m']})


def scan_mapping_log():
    global log_offset, last_log_scan
    now = time.monotonic()
    if now - last_log_scan < 1.0:
        return
    last_log_scan = now
    path = out / 'map' / 'mapping.launch.log'
    if not path.exists():
        return
    size = path.stat().st_size
    if size < log_offset:
        fail('mapping_log_truncated_during_probe')
        return
    with path.open('r', encoding='utf-8', errors='replace') as handle:
        handle.seek(log_offset)
        chunk = handle.read()
        log_offset = handle.tell()
    if 'outside bounds' in chunk.lower():
        fail('planner_outside_bounds')


def cancel_preconditions(goal):
    now = time.monotonic()
    if goal['goal_id'] is None or goal['executing_wall'] is None:
        return False, 'missing_uuid_or_executing_status'
    if now - goal['executing_wall'] < CANCEL_AFTER_EXECUTING_S:
        return False, 'executing_duration_short'
    if action_snapshot is None:
        return False, 'missing_cached_action_status'
    if active_executing_ids() != [goal['goal_id']]:
        return False, 'not_unique_executing_uuid'
    if goal['last_feedback_wall'] is None or now - goal['last_feedback_wall'] > FRESH_WALL_S:
        return False, 'stale_feedback'
    if safety is None or now - safety['received'] > FRESH_WALL_S:
        return False, 'stale_safety'
    if (safety.get('state') != 'ENABLED' or
            safety.get('actuators_enabled') is not True or
            safety.get('active_reasons') not in ('', [])):
        return False, 'unsafe_at_cancel'
    if odom is None or now - odom['received'] > FRESH_WALL_S:
        return False, 'stale_odom'
    if not progress_ok(goal):
        return False, 'insufficient_progress_motion_safety_or_bracket'
    return True, None


def diagnostic_validation_ok():
    return (producer_diagnostics['received'] > 0 and
            bool(producer_diagnostics['matched_critical_cycles']) and
            manager_diagnostics['received'] > 0 and
            bool(manager_diagnostics['matched_status_cycles']) and
            bool(manager_diagnostics['matched_base_cycles']) and
            bool(manager_diagnostics['active_output_cycles']))


def diagnostic_deadline_failure(now):
    if not global_motion_active:
        return None
    if global_active_wall is None:
        return 'global_motion_active_without_latch_time'
    if now - global_active_wall <= FRESH_WALL_S:
        return None
    for role, state in (('producer', producer_diagnostics),
                        ('manager', manager_diagnostics)):
        received = state['last_received']
        if received is None or now - received > FRESH_WALL_S:
            return role + '_relay_diagnostic_stream_stale_during_active'
    for cycle_id, diagnostic in producer_diagnostic_by_cycle.items():
        if (diagnostic['_observer_received'] >= global_active_wall and
                now - diagnostic['_observer_received'] > FRESH_WALL_S and
                cycle_id not in producer_diagnostics['matched_critical_cycles']):
            return 'producer_diagnostic_unmatched_after_2s'
    for count, critical in critical_status_by_count.items():
        if (critical['_observer_received'] >= global_active_wall and
                now - critical['_observer_received'] > FRESH_WALL_S and
                count not in producer_diagnostics['matched_critical_cycles']):
            return 'critical_status_unmatched_after_2s'
    for count, status in safety_status_by_count.items():
        if status['received'] < global_active_wall or now - status['received'] <= FRESH_WALL_S:
            continue
        diagnostic = manager_diagnostic_by_status_count.get(count)
        if (diagnostic is None or
                diagnostic['cycle_id'] not in manager_diagnostics['matched_status_cycles']):
            return 'manager_status_unmatched_after_2s'
    for diagnostic in manager_diagnostic_by_cycle.values():
        if (diagnostic['_observer_received'] < global_active_wall or
                now - diagnostic['_observer_received'] <= FRESH_WALL_S):
            continue
        cycle_id = diagnostic['cycle_id']
        if (diagnostic['event'] == 'periodic_publish' and
                cycle_id not in manager_diagnostics['matched_status_cycles']):
            return 'manager_diagnostic_status_unmatched_after_2s'
        if cycle_id not in manager_diagnostics['matched_base_cycles']:
            return 'manager_diagnostic_base_unmatched_after_2s'
    return None


recording_arm, recording_arm_error = validate_formal_recording_arm(out)
if recording_arm_error is not None:
    fail(recording_arm_error)
else:
    recording_current_ready, recording_current_error = (
        validate_current_recording_readiness(out, recording_arm))
    if recording_current_error is not None:
        fail(recording_current_error)
    else:
        recording_admission_handoff = current_recording_handoff(
            recording_current_ready, recording_arm)
        recording_admission_ready_summary = current_ready_result_summary(
            recording_current_ready)
        recording_runtime_last_summary = recording_admission_ready_summary
        if (recording_admission_handoff['cumulative_nonzero_seen'] or
                recording_admission_handoff['cumulative_active_seen']):
            global_motion_active = True
            global_active_wall = start
            emit('recording_handoff_global_motion_latched',
                 recording_admission_handoff)
            if safety_kind(recording_admission_handoff['latest_safety']) != 'SAFE':
                fail('recording_handoff_unsafe_after_global_motion_active')
    emit('formal_recording_arm_accepted', {
        'path': recording_arm['_observer_arm_path'],
        'immutable_arm_sha256': recording_arm['_observer_arm_sha256'],
        'owner_identity': recording_arm['_observer_early_identity'],
        'localization_owner_identity':
            recording_arm['_observer_localization_identity'],
        'formal_acceptance_session': recording_arm['formal_acceptance_session'],
        'formal_acceptance_session_sha256':
            recording_arm['formal_acceptance_session_sha256'],
        'run_token': recording_arm['run_token'],
        'bag_dir': recording_arm['bag_dir'],
        'localization_bag_dir': recording_arm['localization_writer']['bag_dir'],
        'clock_ns': recording_arm['clock_ns'],
        'clock_advances': recording_arm['clock_advances'],
        'pre_arm_handoff': {
            'pre_arm_nonzero_seen': recording_arm['pre_arm_nonzero_seen'],
            'pre_arm_latest_safety': recording_arm['pre_arm_latest_safety'],
            'pre_arm_status_count': recording_arm['pre_arm_status_count'],
            'pre_arm_status_messages_received':
                recording_arm['pre_arm_status_messages_received'],
            'pre_arm_unsafe_count': recording_arm['pre_arm_unsafe_count']},
        'current_ready_accepted': recording_current_error is None,
        'current_ready_error': recording_current_error,
        'arm_to_observer_handoff': recording_admission_handoff,
        'required_topics': list(ARM_REQUIRED_TOPICS),
        'localization_required_topics': list(LOCALIZATION_ARM_REQUIRED_TOPICS),
        'claim_boundary':
            'immutable_historical_arm_plus_current_writer_continuity_not_final_bag_pass'})
try:
    external_observation_deadline_ns = int(
        os.environ['FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS'])
except (KeyError, TypeError, ValueError):
    external_observation_deadline_ns = None
    fail('missing_or_invalid_formal_observation_deadline')
original_observer_deadline_ns = int((start + WALL_LIMIT_S) * 1_000_000_000)
if (external_observation_deadline_ns is not None and
        external_observation_deadline_ns <= time.monotonic_ns()):
    fail('formal_observation_deadline_already_expired')
observation_deadline_ns = (
    original_observer_deadline_ns if external_observation_deadline_ns is None else
    min(original_observer_deadline_ns, external_observation_deadline_ns))


subs.append(node.create_subscription(Clock, '/clock', on_clock, qos_profile_sensor_data))
subs.append(node.create_subscription(OccupancyGrid, '/global_costmap/costmap', on_grid, grid_qos))
subs.append(node.create_subscription(Odometry, '/odom', on_odom, qos))
subs.append(node.create_subscription(String, '/safety/status_json', on_safety, qos))
for topic in ('/cmd_vel_nav', '/cmd_vel_smoothed'):
    subs.append(node.create_subscription(
        Twist, topic, lambda msg, topic=topic: on_unstamped_twist(topic, msg), qos))
subs.append(node.create_subscription(Twist, '/cmd_vel_gate', on_gate_command, qos))
subs.append(node.create_subscription(TwistStamped, '/base_controller/cmd_vel',
                                     on_base_command, qos))
subs.append(node.create_subscription(String, '/formal_mapping/explorer_status',
                                     on_explorer_status, qos))
subs.append(node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                     on_action_status, status_qos))
subs.append(node.create_subscription(NavigateToPose.Impl.FeedbackMessage,
                                     '/navigate_to_pose/_action/feedback', on_feedback, qos))
for topic in (
        '/safety/relay_enabled',
        '/emergency_stop',
        '/formal_vehicle/simulation/command/emergency_stop',
        '/formal_vehicle/simulation/command/main_power',
        '/formal_vehicle/power/main_power_requested',
        '/formal_vehicle/power/main_isolator_closed',
        '/formal_vehicle/power/main_contactor_closed',
        '/formal_vehicle/power/charge_connected'):
    subs.append(node.create_subscription(
        Bool, topic, lambda msg, topic=topic: on_bool(topic, msg), qos))
subs.append(node.create_subscription(
    String, '/safety/relay_cycle_diagnostic_json',
    on_manager_relay_diagnostic, qos))
subs.append(node.create_subscription(
    String, '/formal_vehicle/auxiliary/critical_safety_relay_diagnostic_json',
    on_producer_relay_diagnostic, qos))
subs.append(node.create_subscription(
    String, '/formal_vehicle/auxiliary/critical_safety_status_json',
    on_critical_status, qos))
for topic in ('/formal_vehicle/auxiliary/status_json',):
    subs.append(node.create_subscription(
        String, topic, lambda msg, topic=topic: on_string(topic, msg), qos))
subs.append(node.create_subscription(
    BatteryState, '/formal_vehicle/power/battery_state', on_battery, qos))
emit('protocol', {'candidate': CANDIDATE, 'wall_limit_s': WALL_LIMIT_S,
                  'minimum_goals': 3,
                  'cancel_after_unique_executing_wall_s': CANCEL_AFTER_EXECUTING_S,
                  'costmap_bracket_inset_m': INSET_M,
                  'minimum_progress_and_motion_m': MIN_PROGRESS_M,
                  'command_nonzero_epsilon': COMMAND_EPSILON,
                  'base_motion_latch_semantics':
                      'any_finite_exact_nonzero_x_or_z_even_1e-8',
                  'startup_safety_exception':
                      'BASE_COMMAND_STOPPED_command_timeout_only_with_recent_real_zero_base',
                  'nav_to_smoothed_gate_safe_active_base_deadline_wall_s':
                      FRESH_WALL_S,
                  'command_chain_receipt_reorder_tolerance_wall_s':
                      CHAIN_RECEIPT_REORDER_TOLERANCE_S,
                  'global_active_binding':
                      'first_nonzero_safety_manager_base_output_irreversible',
                  'observer_only_cannot_close_same_cycle_producer_diagnostic': True,
                  'required_diagnostic_topics': [
                      '/formal_vehicle/auxiliary/critical_safety_relay_diagnostic_json',
                      '/safety/relay_cycle_diagnostic_json'],
                  'diagnostic_transport_drop_or_error_permitted': False,
                  'test_cancel_is_arrival_success': False,
                  'action_status_binding':
                      'cached_transition_plus_fresh_same_uuid_feedback',
                  'product_timeouts_unchanged': [120, 900, 5],
                  'formal_first_map_passed': False})

try:
    while failure is None and time.monotonic_ns() < observation_deadline_ns:
        next_recording_ready, recording_runtime_error = (
            validate_current_recording_readiness(
                out, recording_arm, recording_current_ready))
        if recording_runtime_error:
            fail(recording_runtime_error)
            break
        previous_handoff = current_recording_handoff(
            recording_current_ready, recording_arm)
        recording_current_ready = next_recording_ready
        current_handoff = current_recording_handoff(
            recording_current_ready, recording_arm)
        recording_runtime_last_summary = current_ready_result_summary(
            recording_current_ready)
        if current_handoff != previous_handoff:
            emit('recording_handoff_continuity_update', current_handoff)
        if ((current_handoff['cumulative_nonzero_seen'] or
             current_handoff['cumulative_active_seen']) and
                not global_motion_active):
            global_motion_active = True
            global_active_wall = time.monotonic()
            emit('recording_handoff_global_motion_latched', current_handoff)
            if safety_kind(current_handoff['latest_safety']) != 'SAFE':
                fail('recording_handoff_unsafe_after_global_motion_active')
                break
        rclpy.spin_once(node, timeout_sec=0.1)
        scan_mapping_log()
        now = time.monotonic()
        if current and not current.get('outcome'):
            deadline_failure = safety_deadline_failure(current, now)
            if deadline_failure:
                fail(deadline_failure)
                break
        diagnostic_failure = diagnostic_deadline_failure(now)
        if diagnostic_failure:
            fail(diagnostic_failure)
            break
        if cancel_future is not None:
            if cancel_future.done():
                response = cancel_future.result()
                matching = [bytes(item.goal_id.uuid).hex()
                            for item in response.goals_canceling]
                response_data = {'return_code': int(response.return_code),
                                 'goal_ids': matching}
                emit('cancel_response', response_data)
                if (int(response.return_code) != CancelGoal.Response.ERROR_NONE or
                        matching != [current['goal_id']]):
                    fail('exact_goal_cancel_not_accepted')
                else:
                    current['cancel_accepted'] = True
                    current['cancel_response'] = response_data
                    pending = current.get('pending_explorer_result')
                    if pending is not None:
                        current['pending_explorer_result'] = None
                        finish_explorer_result(pending)
                cancel_future = None
            elif now - cancel_started > 5.0:
                fail('test_cancel_response_timeout')
        if (current and current['cancel_intent'] and not current.get('outcome') and
                now - current['cancel_requested_wall'] > 10.0):
            fail('test_cancel_result_timeout')
        if len(goals) == 3 and all(goal.get('outcome') for goal in goals):
            if diagnostic_validation_ok():
                completed_ok = True
                break
            if now - goals[-1]['closed_wall'] > FRESH_WALL_S:
                fail('required_same_cycle_diagnostics_not_validated')
                break
        if current and not current.get('outcome') and not current['cancel_intent']:
            if (current['executing_wall'] is not None and
                    now - current['executing_wall'] >= CANCEL_AFTER_EXECUTING_S):
                ready, reason = cancel_preconditions(current)
                if not ready:
                    fail('cancel_precondition_failed:' + reason)
                    break
                if not client.service_is_ready():
                    fail('cancel_service_not_ready')
                    break
                request = CancelGoal.Request()
                request.goal_info.goal_id.uuid = list(bytes.fromhex(current['goal_id']))
                request.goal_info.stamp.sec = 0
                request.goal_info.stamp.nanosec = 0
                if not any(request.goal_info.goal_id.uuid):
                    fail('refusing_zero_uuid_cancel')
                    break
                current['cancel_intent'] = True
                current['cancel_requested_wall'] = now
                current['executing_duration_before_cancel_s'] = (
                    now - current['executing_wall'])
                cancel_started = now
                emit('test_cancel_intent', {
                    'number': current['number'], 'goal_id': current['goal_id'],
                    'executing_duration_s': current['executing_duration_before_cancel_s'],
                    'unique_executing_ids': active_executing_ids(),
                    'reason': 'bounded_diagnostic_progress_observed_not_arrival'})
                cancel_future = client.call_async(request)
    if failure is None and not completed_ok:
        fail('bounded_budget_exhausted_before_three_verified_outcomes')
except BaseException as exc:  # A callback or future exception must never yield PASS.
    fail('unexpected_exception:' + type(exc).__name__ + ':' + str(exc))
    emit('exception_traceback', {'traceback': traceback.format_exc()})
finally:
    passed = bool(completed_ok and failure is None and len(goals) == 3 and
                  all(goal.get('outcome') for goal in goals) and
                  diagnostic_validation_ok())
    result = {'schema_version': 2, 'probe_passed': passed,
              'diagnostic_only': True, 'overall_acceptance_passed': False,
              'latest_safety': safety, 'latest_gate_command': gate_command,
              'latest_base_command': base_command, 'failure': failure,
              'pre_goal_invalid_base_samples': pre_goal_invalid_base_samples,
              'pre_goal_relay_false_samples': pre_goal_relay_false_samples,
              'global_motion_active': global_motion_active,
              'global_active_wall_s': (None if global_active_wall is None else
                                       global_active_wall - start),
              'formal_recording_arm': {
                  'accepted_at_start': recording_arm_error is None,
                  'error': recording_arm_error,
                  'path': str(out / 'map' / 'formal_recording_arm.json'),
                  'immutable_arm_sha256': (
                      None if recording_arm is None else
                      recording_arm['_observer_arm_sha256']),
                  'owner_identity': (None if recording_arm is None else
                                     recording_arm['_observer_early_identity']),
                  'localization_owner_identity': (
                      None if recording_arm is None else
                      recording_arm['_observer_localization_identity']),
                  'pre_arm_handoff': (None if recording_arm is None else {
                      'pre_arm_nonzero_seen':
                          recording_arm['pre_arm_nonzero_seen'],
                      'pre_arm_latest_safety':
                          recording_arm['pre_arm_latest_safety'],
                      'pre_arm_status_count':
                          recording_arm['pre_arm_status_count'],
                      'pre_arm_status_messages_received':
                          recording_arm['pre_arm_status_messages_received'],
                      'pre_arm_unsafe_count':
                          recording_arm['pre_arm_unsafe_count']}),
                  'current_ready_continuity': {
                      'accepted_at_start': (
                          recording_arm_error is None and
                          recording_current_error is None and
                          recording_current_ready is not None),
                      'error': recording_current_error,
                      'admission_early_ready': (
                          None if recording_admission_ready_summary is None else
                          recording_admission_ready_summary['early']),
                      'admission_localization_ready': (
                          None if recording_admission_ready_summary is None else
                          recording_admission_ready_summary['localization']),
                      'arm_to_observer_handoff': recording_admission_handoff,
                      'runtime_last_validated':
                          recording_runtime_last_summary,
                      'claim_boundary':
                          'writer_continuity_only_not_final_bag_pass'},
                  'claim_boundary':
                      'immutable_historical_arm_plus_current_writer_continuity_not_final_bag_pass'},
              'observation_deadline': {
                  'external_monotonic_ns': external_observation_deadline_ns,
                  'observer_start_plus_570_monotonic_ns':
                      original_observer_deadline_ns,
                  'effective_monotonic_ns': observation_deadline_ns},
              'evidence_topic_sample_counts': evidence_counts,
              'same_cycle_diagnostic_validation': {
                  'passed': diagnostic_validation_ok(),
                  'expected_formal_acceptance_session': EXPECTED_SESSION,
                  'locked_process_identities': {
                      role: list(identity)
                      for role, identity in diagnostic_identities.items()},
                  'producer_last_receipt_age_wall_s': (
                      None if producer_diagnostics['last_received'] is None else
                      time.monotonic() - producer_diagnostics['last_received']),
                  'producer_received': producer_diagnostics['received'],
                  'producer_matched_critical_cycles': sorted(
                      producer_diagnostics['matched_critical_cycles']),
                  'manager_received': manager_diagnostics['received'],
                  'manager_last_receipt_age_wall_s': (
                      None if manager_diagnostics['last_received'] is None else
                      time.monotonic() - manager_diagnostics['last_received']),
                  'manager_matched_status_cycles': sorted(
                      manager_diagnostics['matched_status_cycles']),
                  'manager_matched_base_cycles': sorted(
                      manager_diagnostics['matched_base_cycles']),
                  'manager_bootstrap_noncausal_cycles': sorted(
                      manager_diagnostics['bootstrap_noncausal_cycles']),
                  'manager_active_output_cycles': sorted(
                      manager_diagnostics['active_output_cycles'])},
              'observer_only_same_cycle_producer_closure': False,
              'elapsed_wall_s': time.monotonic() - start,
              'goals': goals, 'arrival_successes': successes,
              'test_induced_cancellations': sum(
                  goal.get('outcome_kind') == 'TEST_INDUCED_CANCELED'
                  for goal in goals),
              'initial_zero_feedback_observed_by_goal': [
                  goal.get('initial_zero_count', 0) for goal in goals],
              'initial_zero_behavior_observed_for_all_goals': all(
                  goal.get('initial_zero_count', 0) > 0 for goal in goals),
              'formal_first_map_passed': False,
              'requires_sealed_mcap_and_cleanup_review': True}
    with (out / 'frontier_short.result.json').open('x', encoding='utf-8') as handle:
        json.dump(result, handle, indent=2, allow_nan=False)
    emit('result', result)
    events.close()
    node.destroy_node()
    rclpy.shutdown()

sys.exit(0 if completed_ok and failure is None else 1)
