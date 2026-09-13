"""Producer health evidence; health does not renew business evaluations."""
import hashlib
import json
import os
from pathlib import Path
import time

HEALTH_MAX_AGE_NS = 2_000_000_000
# New lifecycle business-cache lease; independent of sensor/map input freshness.
BUSINESS_LEASE_NS = 15_000_000_000
VALID_NONTERMINAL = frozenset({
    'waiting_for_fixed_start_and_slam_map', 'waiting_for_fresh_mapping_inputs',
    'waiting_for_gnss_mapping_reference', 'waiting_for_time_aligned_gnss_odometry',
    'waiting_for_new_time_aligned_gnss_odometry', 'exploring_until_observed_fraction_gate',
    'quality_passed_waiting_for_slam_save_service', 'saving_quality_gated_map'})


def digest(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False).encode()).hexdigest()

def positive(value):
    return type(value) is int and value > 0

def process_identity(pid, proc_root=Path('/proc')):
    text = (proc_root / str(pid) / 'stat').read_text()
    fields = text[text.rfind(')') + 2:].split()
    if fields[0] in ('Z', 'X'):
        raise ValueError('producer_dead')
    return {'boot_id': (proc_root / 'sys/kernel/random/boot_id').read_text().strip(),
            'pid': pid, 'process_starttime_ticks': int(fields[19])}

def producer_alive(identity, session, session_sha, token, proc_root=Path('/proc')):
    try:
        if process_identity(identity['pid'], proc_root) != identity:
            return False
        argv = [x.decode() for x in (proc_root / str(identity['pid']) / 'cmdline').read_bytes().split(b'\0') if x]
        python_entry = bool(argv) and Path(argv[0]).name.startswith('python3')
        console_entry = (len(argv) > 1 and Path(argv[1]).name == 'formal-map-lifecycle-manager' and
                         Path(argv[1]).parent.name == 'sanitation_formal_campus_integration')
        module_entry = len(argv) > 2 and argv[1:3] == ['-c', 'from sanitation_formal_campus_integration.map_lifecycle_manager import main; main()']
        if not python_entry or not (console_entry or module_entry):
            return False
        env = dict(item.split(b'=', 1) for item in
                   (proc_root / str(identity['pid']) / 'environ').read_bytes().split(b'\0') if b'=' in item)
        return (env.get(b'FORMAL_ACCEPTANCE_SESSION') == session.encode() and
                env.get(b'FORMAL_RECORDING_RUN_TOKEN') == token.encode() and
                hashlib.sha256(Path(session).read_bytes()).hexdigest() == session_sha)
    except (OSError, ValueError, KeyError, TypeError):
        return False

def validate_health(packet, previous, now, session, session_sha, token,
                    proc_root=Path('/proc'), wire_sha256=None, wire_text=None):
    """Return immutable continuity state; duplicate reads retain original age."""
    try:
        if (not isinstance(wire_text, str) or json.loads(wire_text) != packet or
                hashlib.sha256(wire_text.encode()).hexdigest() != wire_sha256):
            return None, 'wire_payload_hash_mismatch'
        health = packet['lifecycle_health']
        business = {k: v for k, v in packet.items() if k != 'lifecycle_health'}
        if (health['version'] != 1 or health['session'] != session or
                health['session_sha256'] != session_sha or health['run_token'] != token):
            return None, 'session_mismatch'
        identity = health['producer_identity']
        if not producer_alive(identity, session, session_sha, token, proc_root):
            return None, 'producer_dead_or_identity_mismatch'
        hs, ht, es, et = (health[k] for k in ('health_seq', 'health_monotonic_ns', 'eval_seq', 'eval_monotonic_ns'))
        if not all(positive(v) for v in (hs, ht, es, et)):
            return None, 'invalid_sequence_or_time'
        if ht > now or et > ht:
            return None, 'future_time'
        if now - ht > HEALTH_MAX_AGE_NS:
            return None, 'health_expired'
        bh = digest(business)
        if bh != health['business_sha256']:
            return None, 'business_hash_mismatch'
        kind = health['validity_kind']
        if kind == 'evaluation_lease':
            expiry = health['business_valid_until_monotonic_ns']
            if (not positive(expiry) or expiry < et or expiry - et > 15_000_000_000 or now > expiry):
                return None, 'business_expired_or_invalid'
            if business.get('ready') is not False or business.get('status') not in VALID_NONTERMINAL:
                return None, 'nonterminal_ready_mismatch'
        elif kind == 'sealed_artifact':
            if business.get('status') != 'ready_for_localization_cleaning' or business.get('ready') is not True:
                return None, 'terminal_ready_mismatch'
            from .map_lifecycle_core import load_campus_map_contract, validate_saved_map_artifact, MapLifecycleError
            root = Path(session).parent / 'map'
            episode = Path(session).parent / 'episode/public/episode_manifest.json'
            binding = health['sealed_artifact']
            if (binding['artifact_root'] != str(root) or binding['episode_manifest'] != str(episode) or
                    hashlib.sha256(episode.read_bytes()).hexdigest() != binding['episode_sha256'] or
                    hashlib.sha256((root / 'map_lifecycle_manifest.json').read_bytes()).hexdigest() != binding['manifest_sha256']):
                return None, 'sealed_artifact_binding_mismatch'
            try:
                manifest = validate_saved_map_artifact(root, load_campus_map_contract(episode))
            except MapLifecycleError:
                return None, 'sealed_artifact_validation_failed'
            if manifest.get('map_id') != business.get('map_id'):
                return None, 'sealed_artifact_map_mismatch' 
        else:
            return None, 'invalid_or_failed_business_state'
        packet_sha = wire_sha256
        evaluation_sha = digest({k: v for k, v in health.items() if k not in ('health_seq', 'health_monotonic_ns')})
        state = {'producer_identity': identity, 'health_seq': hs, 'health_monotonic_ns': ht,
                 'eval_seq': es, 'eval_monotonic_ns': et, 'business_sha256': bh,
                 'packet_sha256': packet_sha, 'evaluation_contract_sha256': evaluation_sha}
        if previous is not None:
            if identity != previous['producer_identity']:
                return None, 'producer_restart'
            if hs < previous['health_seq'] or es < previous['eval_seq'] or ht < previous['health_monotonic_ns'] or et < previous['eval_monotonic_ns']:
                return None, 'sequence_or_time_rollback'
            if hs == previous['health_seq'] and packet_sha != previous['packet_sha256']:
                return None, 'same_health_sequence_changed_bytes'
            if es == previous['eval_seq'] and (bh != previous['business_sha256'] or et != previous['eval_monotonic_ns'] or evaluation_sha != previous['evaluation_contract_sha256']):
                return None, 'same_evaluation_sequence_changed_business'
            if hs > previous['health_seq'] and ht <= previous['health_monotonic_ns']:
                return None, 'health_time_not_advanced'
            if es > previous['eval_seq'] and et <= previous['eval_monotonic_ns']:
                return None, 'evaluation_time_not_advanced'
        return state, None
    except (KeyError, TypeError, ValueError, OverflowError, OSError):
        return None, 'invalid_health_contract'


class HealthProducer:
    """Cache evaluations separately from fresh process heartbeat packets."""
    def __init__(self, session, token):
        self.session, self.token = session, token
        self.session_sha = hashlib.sha256(Path(session).read_bytes()).hexdigest()
        self.identity = process_identity(os.getpid())
        self.health_seq = self.eval_seq = 0
        self.business = None
        self.evaluation = None

    def evaluate(self, business, sealed_artifact=None):
        now = time.monotonic_ns()
        self.eval_seq += 1
        self.business = json.loads(json.dumps(business))
        kind = ('sealed_artifact' if business.get('status') == 'ready_for_localization_cleaning' and business.get('ready') is True
                else 'evaluation_lease' if business.get('status') in VALID_NONTERMINAL and business.get('ready') is False
                else 'failed')
        self.evaluation = {'eval_seq': self.eval_seq, 'eval_monotonic_ns': now,
                           'business_sha256': digest(self.business), 'validity_kind': kind,
                           'business_valid_until_monotonic_ns': now + BUSINESS_LEASE_NS}
        if sealed_artifact is not None:
            self.evaluation['sealed_artifact'] = dict(sealed_artifact)

    def heartbeat(self):
        if self.business is None:
            return None
        self.health_seq += 1
        envelope = dict(self.evaluation, version=1, session=self.session,
                        session_sha256=self.session_sha, run_token=self.token,
                        producer_identity=self.identity, health_seq=self.health_seq,
                        health_monotonic_ns=time.monotonic_ns())
        return dict(self.business, lifecycle_health=envelope)
