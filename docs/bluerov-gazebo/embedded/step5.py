import argparse
import hashlib
import json
import os
import shutil
import signal
import subprocess
import threading
import time
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path

from pymavlink import mavutil

from adapters.ports import reserve_instance
from adapters.trace import Trace
from architectures.profiles import PROFILES
from esc.config import CONNECTOR, PLUGIN, apply_esc
from faults.specs import FAULTS, targets
from launch import BLUEOS_IMAGE, arm, docker_gateway, observe_topic, read_parameter
from validate_config import PACKAGE, REPO


def stop_owned(process, fast=False):
    if process is None or process.poll() is not None: return
    if fast:
        process.kill()
    else:
        process.send_signal(signal.SIGINT)

    try:
        process.wait(timeout=1 if fast else 4)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait()

class Processes:
    def __init__(self, session, trace, container_name):
        self.session = session
        self.trace = trace
        self.container_name = container_name
        self.active = {}
        self.all_processes = []
        self.logs = []
        self.threads = []
        self.counts = {}

    def start(self, role, command, cwd=REPO, env=None, parse_output=False):
        self.counts[role] = self.counts.get(role, 0) + 1
        output = (self.session / f"{role}-{self.counts[role]}.log").open("w")
        self.logs.append(output)
        process = subprocess.Popen(command, cwd=cwd, env=env,
                                   stdout=subprocess.PIPE if parse_output else output,
                                   stderr=subprocess.STDOUT, text=parse_output,
                                   start_new_session=True)
        self.active[role] = process

        self.all_processes.append(process)

        self.trace.emit("process_started", role=role, pid=process.pid, command=command)

        if parse_output:
            def read_output():
                for line in process.stdout:
                    output.write(line)

                    if role == "router" and ("got message" in line or "accepted message" in line):
                        self.trace.emit("router_message", detail=line.strip())
                    elif role in ("serial_link", "navigator_io", "claw"):
                        try:
                            record = json.loads(line)
                        except json.JSONDecodeError:
                            continue

                        self.trace.emit("component_event", role=role, detail=record)

            thread = threading.Thread(target=read_output, daemon=True)
            thread.start()
            self.threads.append(thread)

        return process

    def stop(self, role, fast=False):
        process = self.active.get(role)

        if process is None:
            return
        if role == "router":
            subprocess.run(["docker", "rm", "-f", self.container_name],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=False)
            
        if process.poll() is None:
            stop_owned(process, fast)

        self.trace.emit("process_stopped", role=role, pid=process.pid, returncode=process.poll())
        
        del self.active[role]

    def status(self):
        return {role: {"pid": process.pid, "alive": process.poll() is None}
                for role, process in self.active.items()}

    def cleanup(self):
        for role in list(self.active)[::-1]: self.stop(role)

        for process in self.all_processes:
            if process.poll() is None:
                stop_owned(process)

        for thread in self.threads: thread.join(timeout=2)
        for output in self.logs: output.close()


def wait_ready(path, process, timeout=8):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        if path.exists(): return json.loads(path.read_text())
        if process.poll() is not None: raise RuntimeError(f"{process.args[0]} exited before ready")
        
        time.sleep(0.05)

    raise TimeoutError(f"component did not create {path}")


def observe_mavlink(link, sysid, channel, pwm, trace, command_id, seconds, fault_hooks=None):
    deadline = time.monotonic() + seconds
    first_send = True
    samples = []

    while time.monotonic() < deadline:
        if fault_hooks is not None: fault_hooks()

        values = [1500] * 8
        values[channel - 1] = pwm

        link.mav.rc_channels_override_send(sysid, 1, *values)

        if first_send:
            trace.emit("topside_command_sent", command_id=command_id, channel=channel, pwm_us=pwm)
            first_send = False

        until = time.monotonic() + 0.1

        while time.monotonic() < until:
            message = link.recv_match(blocking=True, timeout=0.02)

            if message is None:continue
            if message.get_type() == "HEARTBEAT":
                trace.emit("topside_heartbeat", source_sysid=message.get_srcSystem(),
                           armed=bool(message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
           
            if message.get_type() != "SERVO_OUTPUT_RAW": continue

            outputs = [getattr(message, f"servo{i}_raw") for i in range(1, 9)]
            
            samples.append(outputs)

            trace.emit("autopilot_servo_output", command_id=command_id, pwm_us=outputs,
                       mavlink_seq=message.get_seq())
            
    return samples

def trace_fault_window(session, summary):
    rows = [json.loads(line) for line in (session / "trace.jsonl").read_text().splitlines()]
    onset = summary["fault"].get("actual_onset_monotonic_ns")
    restored = summary["fault"].get("recovery_command_monotonic_ns")

    if onset is None: return

    end = restored or max(row["host_monotonic_ns"] for row in rows)
    window = [row for row in rows if onset <= row["host_monotonic_ns"] <= end]

    def metrics_for(selected):
        metrics = {}

        for event in ("gazebo_motor_1", "esc_raw_pwm_1", "esc_source_valid_1", "esc_output_valid_1"):
            values = [row["value"] for row in selected if row["event"] == event and "value" in row]
            metrics[event] = {"samples": len(values), "distinct_values": sorted(set(values))}
       
        metrics["servo_samples"] = sum(row["event"] == "autopilot_servo_output" for row in selected)
        metrics["topside_heartbeats"] = sum(row["event"] == "topside_heartbeat" for row in selected)
       
        return metrics

    summary["fault"]["observed_during_fault"] = metrics_for(window)

    if restored is not None:
        summary["fault"]["actual_command_window_s"] = (restored - onset) / 1e9
        summary["fault"]["observed_after_recovery_command"] = metrics_for(
            [row for row in rows if row["host_monotonic_ns"] > restored])


def verify_fault_effect(session, summary):
    rows = [json.loads(line) for line in (session / "trace.jsonl").read_text().splitlines()]
    forward = next(row["host_monotonic_ns"] for row in rows
                   if row["event"] == "topside_command_sent" and row["command_id"] == "forward")
    onset = summary["fault"].get("actual_onset_monotonic_ns")
    initial = [row for row in rows if forward <= row["host_monotonic_ns"] < (onset or float("inf"))]
    initial_motor = any(row["event"] == "gazebo_motor_1" and row.get("value") != 128 for row in initial)
    
    summary["initial_motor_response_observed"] = initial_motor
    
    if not initial_motor:
        summary["status"] = "failed"
        summary["error"] = "no non-neutral motor response before fault injection"
    
    fault = summary["fault"]["name"]
    
    if fault == "none":
        return
    
    onset = summary["fault"]["actual_onset_monotonic_ns"]
    recovery = summary["fault"]["recovery_command_monotonic_ns"]
    during = [row for row in rows if onset <= row["host_monotonic_ns"] <= recovery]
    applied = next(row for row in during if row["event"] == "fault_applied")
    target_roles = set(summary["fault"]["targets"]) - {"esc_motor_1"}
    stopped = {row["role"] for row in during if row["event"] == "process_stopped"}
    expected_stops = target_roles - ({"serial_link"} if fault == "pi_failure" else set())

    if fault == "usb_loss":
        expected_stops = set()

    effects = {"expected_processes_stopped": expected_stops <= stopped,
               "unrelated_processes_untouched": not (stopped - target_roles),
               "unrelated_active_processes_alive": all(
                   value["alive"] for role, value in applied["after"].items()
                   if role not in target_roles)}
    
    component_events = [row["detail"]["event"] for row in rows
                        if row["event"] == "component_event" and row.get("role") == "serial_link"]
    
    if fault in ("usb_loss", "pi_failure"): effects["serial_link_toggled"] = "link_down" in component_events and "link_up" in component_events
    if fault == "esc_stuck_neutral":
        observations = summary["fault"]["observed_during_fault"]
        effects["esc_neutral_with_valid_source"] = (
            128 in observations["gazebo_motor_1"]["distinct_values"] and
            any(pwm != 1500 for pwm in observations["esc_raw_pwm_1"]["distinct_values"]) and
            True in observations["esc_output_valid_1"]["distinct_values"])
    
    if fault in ("navigator_io_failure", "pixhawk_restart", "ardusub_failure", "shared_5v_loss"):
        effects["esc_output_became_invalid"] = (
            False in summary["fault"]["observed_during_fault"]["esc_output_valid_1"]["distinct_values"])
    
    summary["fault"]["effect_checks"] = effects
    
    if not all(effects.values()):
        summary["status"] = "failed"
        summary["error"] = f"fault effect check failed: {effects}"

def launch(run_dir, fault="none", fault_onset_s=0.8, fault_duration_s=1.5, seed=0,
           command_pwm=1600, esc_mode="pass_through"):
    
    run_dir = run_dir.resolve()
    manifest = json.loads((run_dir / "manifest.json").read_text())
    resolved = json.loads((run_dir / "resolved.json").read_text())

    if manifest["status"] != "prepared_not_executed":
        raise ValueError("run directory is not a prepared step-2 world")
    
    profile = resolved["architecture"]
    fault_targets = targets(profile, fault)

    if fault_onset_s < 0 or fault_duration_s <= 0: raise ValueError("fault onset and duration must be positive")
    if not 1100 <= command_pwm <= 1900 or command_pwm == 1500: raise ValueError("command PWM must be in 1100..1900 and non-neutral")
    if esc_mode == "legacy": raise ValueError("step 5 requires a virtual ESC mode")
    
    frame = resolved["plant"]["frame"]
    motor_count = len(resolved["plant"]["motor_channels"])
    session_id = uuid.uuid4().hex[:12]
    session = run_dir / "sessions" / session_id

    session.mkdir(parents=True, exist_ok=False)

    lock, ports = reserve_instance()
    trace = Trace(session / "trace.jsonl", manifest["run_id"], session_id)
    container_name = f"mate-embedded-{session_id}"
    managed = Processes(session, trace, container_name)
    observers = []
    observer_threads = []
    link = None

    summary = {"session_id": session_id, "profile": profile, "architecture": PROFILES[profile],
               "ports": ports, "status": "running", "fidelity": "F1 topology; F2 BlueOS router binary only",
               "functional_architecture_comparison_ready": True,
               "quantitative_architecture_comparison_ready": False,
               "fault": {"name": fault, "targets": fault_targets, "requested_onset_s": fault_onset_s,
                         "requested_duration_s": fault_duration_s, "seed": seed,
                         "seed_usage": "recorded only; step-5 faults are deterministic"},
               "esc": {"mode": esc_mode, "stuck_neutral_motor": 0,
                       "plugin_sha256": hashlib.sha256(PLUGIN.read_bytes()).hexdigest(),
                       "connector_sha256": hashlib.sha256(CONNECTOR.read_bytes()).hexdigest()}}
   
    try:
        generated = session / "generated"

        shutil.copytree(run_dir / "generated", generated)

        model_relative = Path(manifest["model"])

        if model_relative.is_absolute() or ".." in model_relative.parts: raise ValueError("manifest model path must stay inside the session")
        
        model_path = session / model_relative
        tree = ET.parse(model_path)
        port_node = tree.getroot().find("model/plugin[@name='ArduPilotPlugin']/fdm_port_in")
        
        if port_node is None: raise ValueError("generated model lacks ArduPilot connector port")
        
        port_node.text = str(ports["gazebo_json_udp"] if profile == "pi_pixhawk" else
                             ports["navigator_connector_udp"])
        
        model_name = frame.replace("-", "_") + "_test"
       
        apply_esc(tree, model_name, motor_count, esc_mode)
        
        tree.write(model_path, encoding="utf-8", xml_declaration=True)

        partition = f"mate_{session_id}"
        env = dict(os.environ, GZ_IP="127.0.0.1", GZ_PARTITION=partition,
                   GZ_SIM_RESOURCE_PATH=str(generated / "models"),
                   GZ_SIM_SYSTEM_PLUGIN_PATH=str(PACKAGE / ".local" / "rov-sim"))
        summary["gazebo_partition"] = partition

        (session / "session.json").write_text(json.dumps(summary, indent=2) + "\n")

        world = generated / "worlds" / f"{frame}.sdf"

        managed.start("gazebo", ["gz", "sim", "-s", "-r", "-v", "2", str(world)], env=env)
        time.sleep(2.5)

        gateway = docker_gateway()
        defaults_name = "sub-6dof.parm" if frame == "bluerov2-heavy" else "sub.parm"
        defaults = ",".join((str(REPO / "Tools/autotest/default_params" / defaults_name),
                             str(generated / "params" / f"{frame}.parm")))

        def start_serial():
            path = session / "serial_link_ready.json"

            path.unlink(missing_ok=True)

            command = [str(REPO / ".venv/bin/python"),
                       str(PACKAGE / "embedded/architectures/serial_link.py"),
                       "--port", str(ports["serial_bridge_tcp"]), "--ready-path", str(path)]
            
            process = managed.start("serial_link", command, parse_output=True)
            summary["serial_link"] = wait_ready(path, process)

        def start_io():
            path = session / "navigator_io_ready.json"

            path.unlink(missing_ok=True)

            command = [str(REPO / ".venv/bin/python"),
                       str(PACKAGE / "embedded/architectures/navigator_io.py"),
                       "--sitl-port", str(ports["gazebo_json_udp"]),
                       "--gazebo-port", str(ports["navigator_connector_udp"]),
                       "--ready-path", str(path)]
            
            process = managed.start("navigator_io", command, parse_output=True)
            summary["navigator_io"] = wait_ready(path, process)

        def start_sitl():
            args = [str(REPO / "build/sitl/bin/ardusub"), "--model", "JSON", "--defaults", defaults,
                    f"-I{ports['sitl_instance']}", "--sysid", str(ports["sysid"]),
                    "--home", "-35.1192416,150.7296753,0.0,0"]
            
            if profile == "pi_pixhawk": args.append(f"--serial0=uart:{summary['serial_link']['slave_path']}:115200")
            
            managed.start("sitl", args, cwd=session)

        def start_claw():
            managed.start("claw", [str(REPO / ".venv/bin/python"),
                          str(PACKAGE / "embedded/architectures/claw_service.py"),
                          "--port", str(ports["claw_udp"])], parse_output=True)

        def start_router():
            endpoint = ports["serial_bridge_tcp"] if profile == "pi_pixhawk" else ports["mavlink_tcp"]
            command = ["docker", "run", "--rm", "--platform", "linux/amd64",
                       "--name", container_name, "--entrypoint", "mavlink-routerd", BLUEOS_IMAGE,
                       "-p", f"{gateway}:{endpoint}",
                       "-e", f"{gateway}:{ports['topside_udp']}",
                       "-e", f"{gateway}:{ports['claw_udp']}", "-s", "254", "-t", "0", "-v"]
            
            managed.start("router", command, parse_output=True)

        if profile == "pi_pixhawk":
            start_serial()
        else:
            start_io()

        start_sitl()
        start_claw()
        start_router()

        summary["router"] = "BlueOS 1.4.6 mavlink-routerd in Docker"

        trace.emit("architecture_started", profile=profile, processes=managed.status())

        for channel in range(1, motor_count + 1):
            prefix = f"/{model_name}/motor_{channel}"

            for suffix, event in (("", f"gazebo_motor_{channel}"),
                                  ("/pwm_us", f"esc_raw_pwm_{channel}"),
                                  ("/pwm_us/source_valid", f"esc_source_valid_{channel}"),
                                  ("/output_valid", f"esc_output_valid_{channel}")):
                observe_topic(prefix + suffix, ["--json-output"], env, trace,
                              event, observers, observer_threads, session)
        observe_topic(f"/{model_name}/motor_1/force", ["--json-output"], env, trace,
                      "gazebo_force_1", observers, observer_threads, session)
        observe_topic(f"/world/{model_name}_world/pose/info", ["--json-output"], env, trace,
                      "gazebo_pose", observers, observer_threads, session)

        link = mavutil.mavlink_connection(f"udpin:127.0.0.1:{ports['topside_udp']}", source_system=255)
        heartbeat = link.wait_heartbeat(timeout=35)
        
        if heartbeat is None or heartbeat.get_srcSystem() != ports["sysid"]: raise TimeoutError("no expected ArduSub heartbeat through architecture path")
        
        trace.emit("topside_heartbeat", source_sysid=heartbeat.get_srcSystem(), armed=False)
        
        actual_frame = read_parameter(link, "FRAME_CONFIG", ports["sysid"])
        
        if actual_frame != resolved["plant"]["frame_config"]: raise ValueError("FRAME_CONFIG does not match selected frame")
        
        forward_channel = read_parameter(link, "RCMAP_FORWARD", ports["sysid"])
        
        trace.emit("topside_parameter_reply", name="FRAME_CONFIG", value=actual_frame)
        link.mav.manual_control_send(ports["sysid"], 0, 0, 500, 0, 1 << 3)
        trace.emit("topside_claw_button", pressed=True)

        time.sleep(0.2)

        link.mav.manual_control_send(ports["sysid"], 0, 0, 500, 0, 0)
        trace.emit("topside_claw_button", pressed=False)

        time.sleep(0.2)

        link.mav.command_long_send(ports["sysid"], 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                   0, mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 100000, 0, 0, 0, 0, 0)
       
        arm(link, ports["sysid"], trace)

        observe_mavlink(link, ports["sysid"], forward_channel, 1500, trace, "neutral", 0.5)

        scenario_start = time.monotonic()
        fault_started = False
        fault_restored = False

        def set_esc_stuck(motor):
            topic = f"/{model_name}/esc/stuck_neutral_motor"

            subprocess.run(["gz", "topic", "-t", topic, "-m", "gz.msgs.Int32",
                            "-p", f"data: {motor}", "-d", "0.5"], env=env, check=True, timeout=5,
                           stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
           
            trace.emit("esc_fault_command", topic=topic, motor=motor)

        def apply_fault():
            nonlocal fault_started, fault_restored

            elapsed = time.monotonic() - scenario_start

            if fault == "none": return

            if not fault_started and elapsed >= fault_onset_s:
                fault_started = True
                summary["fault"]["actual_onset_monotonic_ns"] = time.monotonic_ns()
                trace.emit("fault_onset", name=fault, targets=fault_targets, seed=seed,
                           requested_onset_s=fault_onset_s, requested_duration_s=fault_duration_s,
                           before=managed.status())
                
                if fault == "shared_5v_loss":
                    trace.emit("power_rail_state", rail="regulated_5v", powered=False,
                               model="process availability only")
                    
                if fault == "esc_stuck_neutral": set_esc_stuck(1)
                elif fault == "usb_loss": os.kill(managed.active["serial_link"].pid, signal.SIGUSR1)
                elif fault == "pi_failure":
                    managed.stop("router", fast=True)
                    managed.stop("claw", fast=True)
                    os.kill(managed.active["serial_link"].pid, signal.SIGUSR1)
                else:
                    for role in fault_targets:
                        managed.stop(role, fast=True)

                trace.emit("fault_applied", name=fault, after=managed.status())
           
            if fault_started and not fault_restored and elapsed >= fault_onset_s + fault_duration_s:
                fault_restored = True
                if fault == "esc_stuck_neutral": set_esc_stuck(0)
                elif fault == "usb_loss": os.kill(managed.active["serial_link"].pid, signal.SIGUSR2)
                elif fault == "pi_failure":
                    os.kill(managed.active["serial_link"].pid, signal.SIGUSR2)
                    start_claw()
                    start_router()
                elif fault == "shared_5v_loss":
                    if profile == "pi_pixhawk":
                        start_serial()
                    else:
                        start_io()

                    start_sitl()
                    start_claw()
                    start_router()
                else:
                    for role in fault_targets:
                        {"router": start_router, "sitl": start_sitl,
                         "navigator_io": start_io}[role]()
                summary["fault"]["recovery_command_monotonic_ns"] = time.monotonic_ns()
                if fault == "shared_5v_loss":
                    trace.emit("power_rail_state", rail="regulated_5v", powered=True,
                               model="process availability only")
                trace.emit("fault_recovery_command", name=fault, after=managed.status())

        duration = max(4.5, fault_onset_s + fault_duration_s + 5.0) if fault != "none" else 3.0
        samples = observe_mavlink(link, ports["sysid"], forward_channel, command_pwm,
                                  trace, "forward", duration, apply_fault)
        
        if fault != "none" and (not fault_started or not fault_restored): raise RuntimeError("fault did not reach both onset and recovery")
       
        observe_mavlink(link, ports["sysid"], forward_channel, 1500, trace, "release", 0.5)
        
        link.mav.command_long_send(ports["sysid"], 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                   0, 0, 0, 0, 0, 0, 0, 0)
        
        trace.emit("topside_disarm_request")

        time.sleep(0.6)

        summary["servo_samples"] = len(samples)
        summary["status"] = "passed"

    except Exception as exc:
        summary.update(status="failed", error=str(exc))
        raise
    finally:
        if link is not None: link.close()
        for process in reversed(observers): stop_owned(process)
        for thread in observer_threads: thread.join(timeout=2)

        managed.cleanup()

        if summary["status"] == "passed":
            try:
                trace_fault_window(session, summary)
                verify_fault_effect(session, summary)
            except (KeyError, ValueError, StopIteration) as exc:
                summary.update(status="failed", error=f"fault analysis failed: {exc}")
        
        trace.emit("session_finished", status=summary["status"])
        trace.close()
        lock.close()

        (session / "session.json").write_text(json.dumps(summary, indent=2) + "\n")
    
    if summary["status"] != "passed": raise RuntimeError(summary["error"])
    return session

def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--fault", default="none", choices=("none", *sorted(set().union(*(
        set(profile) for profile in FAULTS.values())))))
    parser.add_argument("--fault-onset-s", type=float, default=0.8)
    parser.add_argument("--fault-duration-s", type=float, default=1.5)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--command-pwm", type=int, default=1600)
    parser.add_argument("--esc-mode", choices=("pass_through", "prior_year"), default="pass_through")
   
    args = parser.parse_args()

    try:
        print(launch(args.run_dir, args.fault, args.fault_onset_s,
                     args.fault_duration_s, args.seed, args.command_pwm, args.esc_mode))
    except (OSError, RuntimeError, ValueError, TimeoutError, subprocess.CalledProcessError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    main()