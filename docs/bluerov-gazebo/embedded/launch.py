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
from esc.config import CONNECTOR, PLUGIN, apply_esc
from validate_config import PACKAGE, REPO

BLUEOS_IMAGE = "bluerobotics/blueos-core:1.4.6"


def stop(process):
    if process is None or process.poll() is not None:
        return
    
    os.killpg(process.pid, signal.SIGINT)

    try:
        process.wait(timeout=4)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()


def docker_gateway():
    result = subprocess.check_output(
        ["docker", "run", "--rm", "--platform", "linux/amd64", "--entrypoint", "getent",
         BLUEOS_IMAGE, "ahostsv4", "host.docker.internal"], text=True)
    
    return result.split()[0]

def read_parameter(link, name, sysid, timeout=8):
    link.mav.param_request_read_send(sysid, 1, name.encode(), -1)

    end = time.monotonic() + timeout

    while time.monotonic() < end:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.3)
        
        if message is not None and str(message.param_id).rstrip("\x00") == name:
            return int(round(message.param_value))
   
    raise TimeoutError(f"no {name} parameter response")

def arm(link, sysid, trace):
    end = time.monotonic() + 15

    last_send = 0

    while time.monotonic() < end:
        if time.monotonic() - last_send >= 1:
            link.mav.command_long_send(sysid, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                       0, 1, 0, 0, 0, 0, 0, 0)
            
            trace.emit("topside_arm_request", mavlink_message="COMMAND_LONG", target_sysid=sysid)
            
            last_send = time.monotonic()

        message = link.recv_match(blocking=True, timeout=0.25)

        if message is None:
            continue

        if message.get_type() == "COMMAND_ACK" and message.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            trace.emit("topside_arm_ack", result=message.result, mavlink_seq=message.get_seq())
       
        if message.get_type() == "HEARTBEAT" and message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            trace.emit("topside_armed_heartbeat", mavlink_seq=message.get_seq())
            return
        
    raise TimeoutError("ArduSub is not armed")

def hold_input(link, sysid, channel, pwm, seconds, trace, command_id):
    end = time.monotonic() + seconds
    values = []
    sent = 0
    
    while time.monotonic() < end:
        channels = [1500] * 8
        channels[channel - 1] = pwm

        link.mav.rc_channels_override_send(sysid, 1, *channels)

        if sent == 0:
            trace.emit("topside_command_sent", command_id=command_id, channel=channel,
                       pwm_us=pwm, mavlink_message="RC_CHANNELS_OVERRIDE", source_sysid=255,
                       target_sysid=sysid)
            
        sent += 1
        until = time.monotonic() + 0.1

        while time.monotonic() < until:
            message = link.recv_match(type="SERVO_OUTPUT_RAW", blocking=True, timeout=0.02)
           
            if message is None:
                continue
           
            outputs = [getattr(message, f"servo{i}_raw") for i in range(1, 9)]
            
            values.append(outputs)
            
            trace.emit("autopilot_servo_output", command_id=command_id, pwm_us=outputs,
                       mavlink_seq=message.get_seq())
    
    return values

def observe_topic(topic, args, env, trace, event, processes, threads, session):
    output = (session / f"{event}.log").open("w")
    process = subprocess.Popen(["gz", "topic", *args, "-e", "-t", topic], cwd=REPO,
                               env=env, stdout=subprocess.PIPE, stderr=output,
                               text=True, start_new_session=True)
    
    processes.append(process)

    def read():
        for line in process.stdout:
            output.write(line)

            if event == "gazebo_pose":
                try:
                    record = json.loads(line)
                except json.JSONDecodeError:
                    continue

                stamp = record.get("header", {}).get("stamp", {})
                sim_time = int(stamp.get("sec", 0)) + int(stamp.get("nsec", 0)) / 1e9
                model_name = topic.split("/")[2].removesuffix("_world")
                pose = next((item for item in record.get("pose", []) if item.get("name") == model_name), None)
                
                if pose is not None:
                    position = pose.get("position", {})
                    orientation = pose.get("orientation", {})

                    trace.emit(event, sim_time_s=sim_time,
                               position_m={axis: float(position.get(axis, 0)) for axis in ("x", "y", "z")},
                               orientation_xyzw={axis: float(orientation.get(axis, 1 if axis == "w" else 0))
                                                 for axis in ("x", "y", "z", "w")})
            else:
                try:
                    record = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if event.startswith(("esc_output_valid_", "esc_source_valid_")):
                    trace.emit(event, value=bool(record.get("data", False)))
                else:
                    trace.emit(event, value=float(record.get("data", 0)))
        output.close()

    thread = threading.Thread(target=read, daemon=True)

    thread.start()
    threads.append(thread)

def launch(run_dir, esc_mode="legacy", command_pwm=1600, timeout_s=0.5, delay_s=0.0,
           stuck_neutral_motor=0, deadband_us=None, drop_input_after_s=-1.0):
    
    run_dir = run_dir.resolve()
    manifest = json.loads((run_dir / "manifest.json").read_text())
    resolved = json.loads((run_dir / "resolved.json").read_text())

    if manifest["status"] != "prepared_not_executed":
        raise ValueError("run directory is not a prepared world")
    
    profile = resolved["architecture"]
    frame = resolved["plant"]["frame"]
    session_id = uuid.uuid4().hex[:12]
    session = run_dir / "sessions" / session_id
    session.mkdir(parents=True, exist_ok=False)
    lock, ports = reserve_instance()
    processes = []
    threads = []
    logs = []
    link = None
    trace = Trace(session / "trace.jsonl", manifest["run_id"], session_id)
    container_name = f"mate-embedded-{session_id}"
    summary = {"profile": profile, "session_id": session_id, "ports": ports,
               "esc": {"mode": esc_mode, "command_pwm": command_pwm, "timeout_s": timeout_s,
                       "delay_s": delay_s, "stuck_neutral_motor": stuck_neutral_motor,
                       "deadband_us": deadband_us, "drop_input_after_s": drop_input_after_s},
               "router": "BlueOS 1.4.6 mavlink-routerd in Docker, TCP surrogate",
               "fidelity": "F1 topology; F2 BlueOS router binary only",
               "architecture_comparison_ready": False,
               "requested_real_time_factor": 1.0,
               "achieved_real_time_factor": None,
               "injected_jitter": None,
               "status": "running"}
    
    try:
        generated = session / "generated"

        shutil.copytree(run_dir / "generated", generated)

        model_relative = Path(manifest["model"])

        if model_relative.is_absolute() or ".." in model_relative.parts:
            raise ValueError("manifest model path must stay inside the session")
        
        model_path = session / model_relative
        tree = ET.parse(model_path)
        port_node = tree.getroot().find("model/plugin[@name='ArduPilotPlugin']/fdm_port_in")

        if port_node is None:
            raise ValueError("generated model lacks ArduPilot connector port")
        
        port_node.text = str(ports["gazebo_json_udp"])
        model_name = frame.replace("-", "_") + "_test"
        motor_count = len(resolved["plant"]["motor_channels"])

        apply_esc(tree, model_name, motor_count, esc_mode, timeout_s, delay_s,
                  stuck_neutral_motor, deadband_us, drop_input_after_s)
        
        if esc_mode != "legacy":
            summary["esc"]["plugin_sha256"] = hashlib.sha256(PLUGIN.read_bytes()).hexdigest()
            summary["esc"]["connector_sha256"] = hashlib.sha256(CONNECTOR.read_bytes()).hexdigest()

        tree.write(model_path, encoding="utf-8", xml_declaration=True)

        partition = f"mate_{session_id}"
        summary["gazebo_partition"] = partition

        (session / "session.json").write_text(json.dumps(summary, indent=2) + "\n")
        env = dict(os.environ, GZ_IP="127.0.0.1", GZ_PARTITION=partition,
                   GZ_SIM_RESOURCE_PATH=str(generated / "models"),
                   GZ_SIM_SYSTEM_PLUGIN_PATH=str(PACKAGE / ".local" / "rov-sim"))

        def start(args, name, cwd=REPO, process_env=None, trace_router=False):
            output = (session / name).open("w")

            logs.append(output)

            process = subprocess.Popen(args, cwd=cwd, env=process_env,
                                       stdout=subprocess.PIPE if trace_router else output,
                                       stderr=subprocess.STDOUT, text=trace_router,
                                       start_new_session=True)
            
            processes.append(process)

            trace.emit("process_started", name=name, pid=process.pid)

            if trace_router:
                def read_router():
                    for line in process.stdout:
                        output.write(line)

                        if "got message" in line or "accepted message" in line:
                            trace.emit("router_message", detail=line.strip())

                thread = threading.Thread(target=read_router, daemon=True)

                thread.start()
                threads.append(thread)

            return process

        world = generated / "worlds" / f"{frame}.sdf"

        start(["gz", "sim", "-s", "-r", "-v", "2", str(world)], "gazebo.log", process_env=env)
        
        time.sleep(2.5)
       
        defaults_name = "sub-6dof.parm" if frame == "bluerov2-heavy" else "sub.parm"
        defaults = ",".join((str(REPO / "Tools/autotest/default_params" / defaults_name),
                             str(generated / "params" / f"{frame}.parm")))
        
        start([str(REPO / "build/sitl/bin/ardusub"), "--model", "JSON", "--defaults", defaults,
               f"-I{ports['sitl_instance']}", "--sysid", str(ports["sysid"]),
               "--home", "-35.1192416,150.7296753,0.0,0"], "sitl.log", cwd=session)
        
        gateway = docker_gateway()
       
        start(["docker", "run", "--rm", "--platform", "linux/amd64", "--name", container_name,
               "--entrypoint", "mavlink-routerd", BLUEOS_IMAGE,
               "-p", f"{gateway}:{ports['mavlink_tcp']}",
               "-e", f"{gateway}:{ports['topside_udp']}", "-t", "0", "-v"],
              "router.log", trace_router=True)
        trace.emit("router_started", gateway=gateway, transport="Docker TCP/UDP")

        for channel in range(1, motor_count + 1):
            prefix = f"/{model_name}/motor_{channel}"
            observe_topic(prefix, ["--json-output"], env, trace,
                          f"gazebo_motor_{channel}", processes, threads, session)
            if esc_mode != "legacy":
                observe_topic(prefix + "/pwm_us", ["--json-output"], env, trace,
                              f"esc_raw_pwm_{channel}", processes, threads, session)
                observe_topic(prefix + "/pwm_us/source_valid", ["--json-output"], env, trace,
                              f"esc_source_valid_{channel}", processes, threads, session)
                observe_topic(prefix + "/output_valid", ["--json-output"], env, trace,
                              f"esc_output_valid_{channel}", processes, threads, session)
        observe_topic(f"/{model_name}/motor_1/force", ["--json-output"], env, trace,
                      "gazebo_force_1", processes, threads, session)
        observe_topic(f"/world/{model_name}_world/pose/info", ["--json-output"], env, trace,
                      "gazebo_pose", processes, threads, session)

        link = mavutil.mavlink_connection(f"udpin:127.0.0.1:{ports['topside_udp']}", source_system=255)
        heartbeat = link.wait_heartbeat(timeout=30)

        if heartbeat is None or heartbeat.get_srcSystem() != ports["sysid"]:
            raise TimeoutError("no expected ArduSub heartbeat through router")
      
        trace.emit("topside_heartbeat", source_sysid=heartbeat.get_srcSystem(),
                   source_compid=heartbeat.get_srcComponent(), mavlink_seq=heartbeat.get_seq())
        
        actual_frame = read_parameter(link, "FRAME_CONFIG", ports["sysid"])
        if actual_frame != resolved["plant"]["frame_config"]:
            raise ValueError(f"FRAME_CONFIG {actual_frame} does not match selected frame")
        
        trace.emit("topside_parameter_reply", name="FRAME_CONFIG", value=actual_frame)

        forward_channel = read_parameter(link, "RCMAP_FORWARD", ports["sysid"])

        link.mav.command_long_send(ports["sysid"], 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                                   0, mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 100000, 0, 0, 0, 0, 0)
        
        arm(link, ports["sysid"], trace)

        hold_input(link, ports["sysid"], forward_channel, 1500, 0.5, trace, "neutral")
        
        values = hold_input(link, ports["sysid"], forward_channel, command_pwm, 2.0, trace, "forward")
        
        hold_input(link, ports["sysid"], forward_channel, 1500, 0.5, trace, "release")
       
        if not values or not any(any(pwm != 1500 for pwm in row[:len(resolved["plant"]["motor_channels"])])
                                 for row in values):
            raise RuntimeError("no non-neutral servo output observed")
        
        link.mav.command_long_send(ports["sysid"], 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                   0, 0, 0, 0, 0, 0, 0, 0)
        
        trace.emit("topside_disarm_request")

        time.sleep(0.8)

        summary.update(status="passed", servo_samples=len(values),
                       last_servo_pwm_us=values[-1][:len(resolved["plant"]["motor_channels"])])
    
    except Exception as exc:
        summary.update(status="failed", error=str(exc))
        raise

    finally:
        if link is not None:
            try:
                link.mav.command_long_send(ports["sysid"], 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                           0, 0, 0, 0, 0, 0, 0, 0)
                link.close()
            except OSError:
                pass

        for process in reversed(processes):
            stop(process)

        subprocess.run(["docker", "rm", "-f", container_name], stdout=subprocess.DEVNULL,
                       stderr=subprocess.DEVNULL, check=False)
        
        for thread in threads:
            thread.join(timeout=2)

        for output in logs:
            output.close()

        trace.emit("session_finished", status=summary["status"])
        trace.close()
        lock.close()
        (session / "session.json").write_text(json.dumps(summary, indent=2) + "\n")
    return session

def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--esc-mode", choices=("legacy", "pass_through", "prior_year"), default="legacy")
    parser.add_argument("--command-pwm", type=int, choices=range(1100, 1901), metavar="1100..1900", default=1600)
    parser.add_argument("--esc-timeout-s", type=float, default=0.5)
    parser.add_argument("--esc-delay-s", type=float, default=0.0)
    parser.add_argument("--esc-stuck-neutral-motor", type=int, default=0)
    parser.add_argument("--esc-deadband-us", type=int)
    parser.add_argument("--esc-drop-input-after-s", type=float, default=-1.0)
    
    args = parser.parse_args()

    try:
        print(launch(args.run_dir, args.esc_mode, args.command_pwm, args.esc_timeout_s,
                     args.esc_delay_s, args.esc_stuck_neutral_motor, args.esc_deadband_us,
                     args.esc_drop_input_after_s))
    except (OSError, RuntimeError, ValueError, TimeoutError, subprocess.CalledProcessError) as exc:
        parser.error(str(exc))

if __name__ == "__main__":
    main()