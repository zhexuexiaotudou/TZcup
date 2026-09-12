#!/usr/bin/env python3
"""Dual-role actual rosbag2 writer readiness for the first-map run.

``early`` records clock, localization streams, and the short05 causal topics.
``localization`` records the unchanged primary localization-bag contract and
observes (but does not add) /clock.  The early role issues the aggregate arm
only while both writers are current.  Arm freshness is admission-only.
"""
from __future__ import annotations

import argparse, hashlib, json, os, pathlib, signal, sys, time
from dataclasses import dataclass, field
from typing import Any

CLOCK = "/clock"
REQUIRED = {
    "/odom": "nav_msgs/msg/Odometry", "/odom/unfiltered": "nav_msgs/msg/Odometry",
    "/odometry/gps": "nav_msgs/msg/Odometry", "/gnss/fix": "sensor_msgs/msg/NavSatFix",
    "/formal_mapping/lifecycle_status": "std_msgs/msg/String",
}
ASSOCIATION = {
    "/base_controller/cmd_vel": "geometry_msgs/msg/TwistStamped",
    "/safety/status_json": "std_msgs/msg/String",
    "/safety/relay_cycle_diagnostic_json": "std_msgs/msg/String",
    "/formal_vehicle/auxiliary/critical_safety_relay_diagnostic_json": "std_msgs/msg/String",
}
EARLY_TYPES = {CLOCK: "rosgraph_msgs/msg/Clock", **REQUIRED, **ASSOCIATION}
LOCAL_TYPES = {**REQUIRED, "/ground_truth/odom": "nav_msgs/msg/Odometry"}
FRESH_NS = 2_000_000_000
REFRESH_NS = 200_000_000


def boot_id() -> str:
    return pathlib.Path("/proc/sys/kernel/random/boot_id").read_text().strip()


def start_ticks(pid: int) -> int:
    text = pathlib.Path(f"/proc/{pid}/stat").read_text()
    return int(text[text.rfind(")") + 2:].split()[19])


def identity(role: str, token: str) -> dict[str, Any]:
    pid = os.getpid()
    return {"boot_id": boot_id(), "pid": pid, "pgid": os.getpgid(pid),
            "process_starttime_ticks": start_ticks(pid), "owner_token": f"{token}:{role}"}


def sha(path: pathlib.Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def write_exclusive(path: pathlib.Path, value: dict[str, Any]) -> None:
    if path.exists() or path.is_symlink(): raise FileExistsError(path)
    temp = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    data = (json.dumps(value, sort_keys=True, indent=2) + "\n").encode()
    fd = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(fd, "wb") as out: out.write(data); out.flush(); os.fsync(out.fileno())
        os.link(temp, path)
    finally:
        try: temp.unlink()
        except FileNotFoundError: pass


def write_current(path: pathlib.Path, value: dict[str, Any]) -> None:
    temp = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    data = (json.dumps(value, sort_keys=True, indent=2) + "\n").encode()
    fd = os.open(temp, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    with os.fdopen(fd, "wb") as out: out.write(data); out.flush(); os.fsync(out.fileno())
    os.replace(temp, path)


def clock_value(message: Any) -> int:
    return int(message.clock.sec) * 1_000_000_000 + int(message.clock.nanosec)


def stamped_or_now(message: Any, now: int) -> int:
    stamp = getattr(getattr(message, "header", None), "stamp", None)
    value = 0 if stamp is None else int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    return value if value > 0 else now


@dataclass
class Tracker:
    role: str; topics: dict[str, str]; session: str; session_sha: str; token: str; bag_dir: str
    counts: dict[str, int] = field(init=False)
    last: dict[str, int] = field(default_factory=dict)
    clock_ns: int = 0; clock_advances: int = 0; previous_clock_ns: int = 0; clock_rollback: bool = False; last_clock_advance_monotonic_ns: int = 0
    pre_arm_nonzero_seen: bool = False; pre_arm_unsafe_count: int = 0; pre_arm_status_count: int | None = None; pre_arm_status_messages_received: int = 0; pre_arm_active_seen: bool = False; pre_arm_latest_safety: dict[str, Any] | None = None
    def __post_init__(self) -> None: self.counts = {topic: 0 for topic in self.topics}
    def observe_clock(self, value: int, mono: int) -> None:
        if value <= 0: return
        if self.previous_clock_ns:
            if value < self.previous_clock_ns: self.clock_rollback = True
            elif value > self.previous_clock_ns:
                self.clock_advances += 1; self.last_clock_advance_monotonic_ns = mono
        self.previous_clock_ns = value; self.clock_ns = value
    def written(self, topic: str, message: Any, mono: int) -> None:
        self.counts[topic] += 1; self.last[topic] = mono
        if topic == CLOCK: self.observe_clock(clock_value(message), mono)
        elif topic == "/base_controller/cmd_vel":
            t = message.twist
            if any(abs(float(v)) > 0.0 for v in (t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z)): self.pre_arm_nonzero_seen = True
        elif topic == "/safety/status_json":
            self.pre_arm_status_messages_received += 1
            try: data = json.loads(message.data)
            except (TypeError, json.JSONDecodeError): data = {"raw": getattr(message, "data", None)}
            self.pre_arm_latest_safety = data if isinstance(data, dict) else {"raw": data}
            published_count = self.pre_arm_latest_safety.get("status_publish_count")
            self.pre_arm_status_count = (
                published_count
                if isinstance(published_count, int) and not isinstance(published_count, bool)
                and published_count >= 0 else None
            )
            state = self.pre_arm_latest_safety.get("state", self.pre_arm_latest_safety.get("global_state"))
            safe = (
                state == "ENABLED"
                and self.pre_arm_latest_safety.get("safety_inputs_permit_actuators") is True
                and self.pre_arm_latest_safety.get("actuators_enabled") is True
                and self.pre_arm_latest_safety.get("managed_controllers_active") is True
                and self.pre_arm_latest_safety.get("active_reasons") == ""
            )
            if not safe: self.pre_arm_unsafe_count += 1
            if state == "ACTIVE": self.pre_arm_active_seen = True
    def blockers(self, mono: int) -> list[str]:
        result = []
        if self.clock_rollback: result.append("clock rollback observed")
        if self.clock_ns <= 0 or self.clock_advances < 2: result.append("clock did not strictly advance at least twice")
        if self.last_clock_advance_monotonic_ns <= 0 or mono - self.last_clock_advance_monotonic_ns > FRESH_NS:
            result.append("clock evidence is stalled")
        for topic in REQUIRED:
            if self.counts.get(topic, 0) <= 0: result.append(f"writer has no message for {topic}"); continue
            if topic not in self.last or mono - self.last[topic] > FRESH_NS: result.append(f"writer evidence for {topic} is stale")
        if self.role == "early":
            for topic in ASSOCIATION:
                if self.counts.get(topic, 0) <= 0: result.append(f"writer has no message for {topic}")
                elif mono - self.last[topic] > FRESH_NS: result.append(f"writer evidence for {topic} is stale")
            if self.pre_arm_nonzero_seen: result.append("pre-arm nonzero base command observed")
            if self.pre_arm_active_seen: result.append("pre-arm ACTIVE safety evidence observed")
            if self.pre_arm_status_count is None: result.append("pre-arm safety status lacks status_publish_count")
        return result
    def ready(self, mono: int, status: str) -> dict[str, Any]:
        return {"schema_version":1,"report_id":f"tzcup_formal_{self.role}_recording_ready_v1","status":status,"ready":not self.blockers(mono),"role":self.role,"formal_acceptance_session":self.session,"formal_acceptance_session_sha256":self.session_sha,"run_token":self.token,"owner_identity":identity(self.role,self.token),"bag_dir":self.bag_dir,"writer_message_counts":self.counts,"clock_ns":self.clock_ns,"clock_advances":self.clock_advances,"clock_rollback":self.clock_rollback,"last_clock_advance_monotonic_ns":self.last_clock_advance_monotonic_ns,"last_message_monotonic_ns_by_topic":self.last,"pre_arm_nonzero_seen":self.pre_arm_nonzero_seen,"pre_arm_unsafe_count":self.pre_arm_unsafe_count,"pre_arm_status_count":self.pre_arm_status_count,"pre_arm_status_messages_received":self.pre_arm_status_messages_received,"pre_arm_active_seen":self.pre_arm_active_seen,"pre_arm_latest_safety":self.pre_arm_latest_safety,"ready_epoch_ns":time.time_ns(),"ready_monotonic_ns":mono}


def owner_live(value: dict[str, Any]) -> bool:
    try:
        row=value["owner_identity"]; pid=int(row["pid"])
        return boot_id()==row["boot_id"] and os.getpgid(pid)==int(row["pgid"]) and start_ticks(pid)==int(row["process_starttime_ticks"])
    except (KeyError,OSError,ValueError,TypeError): return False


def valid_local_ready(path: pathlib.Path, tracker: Tracker) -> tuple[dict[str, Any], int] | None:
    try: value=json.loads(path.read_text())
    except (OSError,json.JSONDecodeError): return None
    final_now = time.monotonic_ns()
    if not isinstance(value,dict) or value.get("status")!="FORMAL_LOCALIZATION_RECORDING_READY" or value.get("ready") is not True: return None
    if value.get("formal_acceptance_session") != tracker.session or value.get("formal_acceptance_session_sha256")!=tracker.session_sha or value.get("run_token")!=tracker.token or value.get("owner_identity", {}).get("owner_token") != f"{tracker.token}:localization" or value.get("bag_dir") != str(path.parent / "mapping_localization_diagnostic") or not owner_live(value): return None
    if not isinstance(value.get("ready_monotonic_ns"),int) or not 0 <= final_now-value["ready_monotonic_ns"] <= FRESH_NS: return None
    counts=value.get("writer_message_counts"); last=value.get("last_message_monotonic_ns_by_topic")
    if not isinstance(counts,dict) or not isinstance(last,dict) or value.get("clock_ns",0)<=0 or value.get("clock_advances",0)<2 or value.get("clock_rollback") is not False or not isinstance(value.get("last_clock_advance_monotonic_ns"), int) or not 0 <= final_now-value["last_clock_advance_monotonic_ns"] <= FRESH_NS: return None
    for topic in REQUIRED:
        if not isinstance(counts.get(topic),int) or counts[topic]<=0 or not isinstance(last.get(topic),int) or not 0 <= final_now-last[topic] <= FRESH_NS: return None
    return value, final_now


def closed_summary(bag: pathlib.Path, counts: dict[str,int]) -> dict[str,Any]:
    import yaml
    meta=bag/"metadata.yaml"
    if not meta.is_file() or meta.is_symlink(): raise RuntimeError("closed bag has no metadata")
    info=yaml.safe_load(meta.read_text()).get("rosbag2_bagfile_information",{})
    actual={r.get("topic_metadata",{}).get("name"):r.get("message_count") for r in info.get("topics_with_message_count",[])}
    if actual!=counts: raise RuntimeError("closed bag metadata counts differ from writer counts")
    files=info.get("relative_file_paths")
    if not isinstance(files,list) or not files: raise RuntimeError("closed bag has no data files")
    for name in files:
        path=bag/str(name)
        if not path.is_file() or path.is_symlink() or path.stat().st_size<16: raise RuntimeError("closed bag data file invalid")
        with path.open("rb") as stream:
            if stream.read(8)!=b"\x89MCAP0\r\n": raise RuntimeError("closed bag lacks MCAP header")
            stream.seek(-8,os.SEEK_END)
            if stream.read(8)!=b"\x89MCAP0\r\n": raise RuntimeError("closed bag lacks MCAP footer")
    return {"metadata_message_counts":actual,"metadata_message_count":info.get("message_count"),"data_files":files}


def main(argv: list[str] | None=None) -> int:
    parser=argparse.ArgumentParser(); parser.add_argument("--output",required=True,type=pathlib.Path); parser.add_argument("--role",required=True,choices=("early","localization")); parser.add_argument("--timeout",type=int,default=120); args=parser.parse_args(argv)
    if args.timeout<=0 or not args.output.is_dir() or args.output.is_symlink(): raise ValueError("output must be real and timeout positive")
    session=os.environ.get("FORMAL_ACCEPTANCE_SESSION",""); token=os.environ.get("FORMAL_RECORDING_RUN_TOKEN",""); deadline_text=os.environ.get("FORMAL_OBSERVATION_DEADLINE_MONOTONIC_NS",""); session_path=pathlib.Path(session)
    if not session_path.is_file() or session_path.is_symlink() or not token or not deadline_text.isdigit(): raise ValueError("session, run token, shared deadline required")
    shared_deadline=int(deadline_text)
    admission_deadline=min(shared_deadline,time.monotonic_ns()+args.timeout*1_000_000_000)
    if shared_deadline<=time.monotonic_ns(): raise ValueError("deadline expired")
    prefix="formal_recording" if args.role=="early" else "formal_localization"; bag=args.output/("early_recording_audit" if args.role=="early" else "mapping_localization_diagnostic")
    paths={name:args.output/f"{prefix}_{suffix}.json" for name,suffix in {"open":"writer_open","ready":"ready","initial":"initial_ready","closed":"closed"}.items()}; paths["invalid"]=args.output/("formal_recording_invalid.json" if args.role=="early" else "formal_localization_recording_invalid.json"); paths["arm"]=args.output/"formal_recording_arm.json"
    for path in (*paths.values(),bag):
        if path.exists() or path.is_symlink() or not path.parent.is_dir() or path.parent.is_symlink(): raise FileExistsError(path)
    import rclpy, rosbag2_py
    from geometry_msgs.msg import TwistStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.serialization import serialize_message
    from rosgraph_msgs.msg import Clock
    from sensor_msgs.msg import NavSatFix
    from std_msgs.msg import String
    classes={CLOCK:Clock,"/base_controller/cmd_vel":TwistStamped,"/safety/status_json":String,"/safety/relay_cycle_diagnostic_json":String,"/formal_vehicle/auxiliary/critical_safety_relay_diagnostic_json":String,"/odom":Odometry,"/odom/unfiltered":Odometry,"/odometry/gps":Odometry,"/ground_truth/odom":Odometry,"/gnss/fix":NavSatFix,"/formal_mapping/lifecycle_status":String}
    topics=EARLY_TYPES if args.role=="early" else LOCAL_TYPES; tracker=Tracker(args.role,topics,session,sha(session_path),token,str(bag))
    rclpy.init(); node=Node(f"formal_first_map_{args.role}_recording"); writer=rosbag2_py.SequentialWriter(); writer.open(rosbag2_py.StorageOptions(uri=str(bag),storage_id="mcap"),rosbag2_py.ConverterOptions("cdr","cdr"))
    for index,(topic,type_name) in enumerate(topics.items()): writer.create_topic(rosbag2_py.TopicMetadata(id=index,name=topic,type=type_name,serialization_format="cdr",offered_qos_profiles=[]))
    write_exclusive(paths["open"],{"schema_version":1,"report_id":f"tzcup_formal_{args.role}_writer_open_v1","status":"FORMAL_RECORDING_WRITER_OPEN","opened":True,"role":args.role,"formal_acceptance_session":session,"formal_acceptance_session_sha256":tracker.session_sha,"run_token":token,"owner_identity":identity(args.role,token),"bag_dir":str(bag),"topics":topics,"opened_epoch_ns":time.time_ns(),"opened_monotonic_ns":time.monotonic_ns()})
    stopped=False; error: str|None=None; armed=False; last_refresh=0
    def invalidate(reason: str) -> None:
        write_exclusive(paths["invalid"],{"schema_version":1,"report_id":("tzcup_formal_recording_invalid_v1" if args.role=="early" else "tzcup_formal_localization_recording_invalid_v1"),"status":("FORMAL_RECORDING_INVALID" if args.role=="early" else "FORMAL_LOCALIZATION_RECORDING_INVALID"),"invalidated":True,"armed":False,"role":args.role,"writer_error":reason,"formal_acceptance_session":session,"formal_acceptance_session_sha256":tracker.session_sha,"run_token":token,"owner_identity":identity(args.role,token),"arm_receipt_path":str(paths["arm"]),"arm_receipt_sha256":sha(paths["arm"]) if paths["arm"].is_file() else None,"invalidated_epoch_ns":time.time_ns(),"invalidated_monotonic_ns":time.monotonic_ns()})
    def stop(_signum: int,_frame: Any) -> None:
        nonlocal stopped; stopped=True
    signal.signal(signal.SIGINT,stop); signal.signal(signal.SIGTERM,stop)
    def write_message(topic: str,message: Any) -> None:
        nonlocal error,stopped
        if error is not None:return
        try:
            now=time.monotonic_ns(); stamp=clock_value(message) if topic==CLOCK else stamped_or_now(message,node.get_clock().now().nanoseconds); writer.write(topic,serialize_message(message),stamp); tracker.written(topic,message,now)
        except Exception as exc: error=str(exc); invalidate(error); stopped=True
    def observe_clock(message: Any) -> None:
        nonlocal error,stopped
        try: tracker.observe_clock(clock_value(message),time.monotonic_ns())
        except Exception as exc: error=str(exc); invalidate(error); stopped=True
    subscriptions=[node.create_subscription(classes[t],t,lambda message,t=t:write_message(t,message),qos_profile_sensor_data) for t in topics]
    if args.role=="localization": subscriptions.append(node.create_subscription(Clock,CLOCK,observe_clock,qos_profile_sensor_data))
    _=subscriptions; rc=3
    try:
        while not stopped:
            rclpy.spin_once(node,timeout_sec=0.1); now=time.monotonic_ns()
            if tracker.clock_rollback and error is None: error="clock rollback observed"; invalidate(error); stopped=True; continue
            ready=not tracker.blockers(now)
            if ready and now-last_refresh>=REFRESH_NS:
                payload=tracker.ready(now,"FORMAL_RECORDING_EARLY_READY" if args.role=="early" else "FORMAL_LOCALIZATION_RECORDING_READY"); write_current(paths["ready"],payload); last_refresh=now
                if not paths["initial"].exists():write_exclusive(paths["initial"],payload)
            if args.role=="early" and (args.output / "formal_localization_recording_invalid.json").exists() and error is None:
                error="localization writer invalidated"; invalidate(error); stopped=True; continue
            if args.role=="early" and ready and not armed:
                local_result=valid_local_ready(args.output/"formal_localization_ready.json",tracker)
                if local_result is not None:
                    local, final_now=local_result
                    if not tracker.blockers(final_now):
                        arm=tracker.ready(final_now,"FORMAL_RECORDING_ARMED"); arm.update({"report_id":"tzcup_formal_recording_arm_v1","armed":True,"armed_epoch_ns":time.time_ns(),"armed_monotonic_ns":final_now,"localization_writer":local}); write_exclusive(paths["arm"],arm); armed=True
            if now>=admission_deadline and ((args.role=="early" and not armed) or (args.role=="localization" and not ready)):
                rc=2; break
            if now>=shared_deadline:
                if (args.role=="early" and not armed) or (args.role=="localization" and not ready): rc=2
                else: stopped=True
                break
        if error is not None: rc=1
        elif stopped: rc=0 if (armed if args.role=="early" else paths["ready"].is_file()) else 3
    finally:
        try:
            node.destroy_node()
            writer.close()
        except Exception as exc:
            if error is None:
                error=f"writer close failure: {exc}"
                try:
                    invalidate(error)
                except Exception:
                    pass
            rc=1
        finally:
            rclpy.shutdown()
        try: summary=closed_summary(bag,tracker.counts)
        except Exception as exc: summary={"validation_error":str(exc)}; rc=1
        write_exclusive(paths["closed"],{"schema_version":1,"report_id":f"tzcup_formal_{args.role}_recording_closed_v1","status":"FORMAL_RECORDING_CLOSED","closed":True,"role":args.role,"close_rc":rc,"normal_close":rc==0 and error is None,"actual_writer_message_counts":tracker.counts,"bag_validation":summary,"writer_error":error,"closed_epoch_ns":time.time_ns(),"closed_monotonic_ns":time.monotonic_ns()})
    return rc


if __name__=="__main__":
    try: raise SystemExit(main())
    except Exception as exc: print(f"first-map recording writer failed: {exc}",file=sys.stderr); raise SystemExit(1)
