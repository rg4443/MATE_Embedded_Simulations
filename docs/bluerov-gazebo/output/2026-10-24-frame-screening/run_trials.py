import argparse
import csv
import json
import math
import os
import signal
import subprocess
import threading
import time
from pathlib import Path

from pymavlink import mavutil


ROOT = Path(__file__).resolve().parents[4]
PACKAGE = ROOT / "docs/bluerov-gazebo"
OUTPUT = Path(__file__).resolve().parent
RUNTIME = Path("/private/tmp/mate-frame-screening-20261024")

FRAMES = {
    "bluerov1": ("sub.parm", 0, 6),
    "bluerov2": ("sub.parm", 1, 6),
    "bluerov2-heavy": ("sub-6dof.parm", 2, 8),
}

AXES = ("forward", "lateral", "dive", "yaw", "roll", "pitch")

SAMPLE_COLUMNS = (
    "trial_id", "layout", "axis", "phase", "wall_elapsed_s", "sim_time_s",
    "x_m", "y_m", "z_m", "roll_deg", "pitch_deg", "yaw_deg",
)

SUMMARY_COLUMNS = (
    "trial_id", "layout", "thrusters", "frame_config", "axis", "command_pwm",
    "duration_s", "mode", "start_x_m", "start_y_m", "start_z_m",
    "delta_x_m", "delta_y_m", "delta_z_m", "horizontal_m",
    "delta_roll_deg", "delta_pitch_deg", "delta_yaw_deg", "pose_samples",
    "motor_pwm_min", "motor_pwm_max", "motor_saturation_pct",
    "armed", "rcmap_channel", "status", "notes",
)

def quaternion_to_euler(q):
    x, y, z, w = (float(q.get(key, 0 if key != "w" else 1)) for key in ("x", "y", "z", "w"))
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    return tuple(math.degrees(value) for value in (roll, pitch, yaw))

def median(values):
    values = sorted(values)

    if not values: raise ValueError("No samples in median")

    middle = len(values) // 2

    return values[middle] if len(values) % 2 else (values[middle - 1] + values[middle]) / 2

def angle_difference(end, start):
    return math.degrees(math.atan2(math.sin(math.radians(end - start)), math.cos(math.radians(end - start))))

def stop_process(process):
    if process is None or process.poll() is not None: return

    os.killpg(process.pid, signal.SIGINT)

    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        os.killpg(process.pid, signal.SIGTERM)
        process.wait(timeout=5)

def wait_for_master(timeout=25):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        try:
            link = mavutil.mavlink_connection("tcp:127.0.0.1:5770", source_system=255)
            
            if link.wait_heartbeat(timeout=3): return link

            link.close()

        except (ConnectionError, OSError):
            pass

        time.sleep(0.3)

    raise TimeoutError("No ArduSub heartbeat on TCP 5770")

def get_parameter(link, name, timeout=6):
    link.mav.param_request_read_send(2, 1, name.encode(), -1)

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        msg = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        
        if msg is None: continue

        param_id = msg.param_id.decode(errors="replace") if isinstance(msg.param_id, bytes) else msg.param_id
        
        if param_id.rstrip("\x00") == name: return int(round(msg.param_value))
    
    raise TimeoutError(f"No parameter response for {name}")

def arm(link):
    deadline = time.monotonic() + 15

    while time.monotonic() < deadline:
        link.mav.command_long_send(2, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        
        step_end = time.monotonic() + 1

        while time.monotonic() < step_end:
            msg = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
            if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED: return True
   
    return False

def send_rc(link, channel, pwm):
    channels = [1500] * 8
    channels[channel - 1] = pwm
    link.mav.rc_channels_override_send(2, 1, *channels)

def hold_rc(link, channel, pwm, seconds, phase, shared, motor_count, motor_values):
    shared["phase"] = phase
    end = time.monotonic() + seconds

    while time.monotonic() < end:
        send_rc(link, channel, pwm)

        if phase == "command":
            while True:
                message = link.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
                
                if message is None: break

                motor_values.extend(getattr(message, f"servo{index}_raw") for index in range(1, motor_count + 1))
        
        time.sleep(0.1)

def pose_reader(process, trial_id, layout, axis, shared, samples, start):
    last_saved = 0.0

    for line in process.stdout:
        now = time.monotonic()

        if now - last_saved < 0.08: continue

        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue

        pose = next((item for item in record.get("pose", []) if item.get("name") == layout.replace("-", "_") + "_test"), None)
        
        if pose is None: continue

        stamp = record.get("header", {}).get("stamp", {})
        pos = pose.get("position", {})
        roll, pitch, yaw = quaternion_to_euler(pose.get("orientation", {}))
        
        samples.append((
            trial_id, layout, axis, shared["phase"], round(now - start, 3),
            int(stamp.get("sec", 0)) + int(stamp.get("nsec", 0)) / 1e9,
            float(pos.get("x", 0)), float(pos.get("y", 0)), float(pos.get("z", 0)),
            roll, pitch, yaw,
        ))

        last_saved = now

def trial_summary(trial_id, layout, axis, channel, pwm, armed, samples, motor_values):
    frame = FRAMES[layout]
    motor_min = min(motor_values) if motor_values else ""
    motor_max = max(motor_values) if motor_values else ""
    saturation = (
        round(100 * sum(value <= 1100 or value >= 1900 for value in motor_values) / len(motor_values), 2)
        if motor_values else ""
    )
    neutral = [row for row in samples if row[3] == "baseline"]
    driven = [row for row in samples if row[3] == "command"]

    if not neutral or not driven:
        return (trial_id, layout, frame[2], frame[1], axis, pwm, 2.0, "MANUAL",
                *("" for _ in range(10)), len(samples), motor_min, motor_max, saturation,
                armed, channel, "invalid", "Missing baseline or command pose")
    
    neutral = neutral[-min(len(neutral), 4):]
    driven = driven[-min(len(driven), 4):]
    start = [median([row[index] for row in neutral]) for index in range(6, 12)]
    end = [median([row[index] for row in driven]) for index in range(6, 12)]
    delta = [end[i] - start[i] for i in range(3)]
    rotations = [angle_difference(end[i], start[i]) for i in range(3, 6)]
    
    return (
        trial_id, layout, frame[2], frame[1], axis, pwm, 2.0, "MANUAL",
        *[round(value, 5) for value in start[:3]],
        *[round(value, 5) for value in delta], round(math.hypot(delta[0], delta[1]), 5),
        *[round(value, 3) for value in rotations], len(samples), motor_min, motor_max, saturation,
        armed, channel, "measured",
        "One preliminary run; no mass change or motor fault",
    )

def run_trial(layout, axis, trial_number):
    trial_id = f"{layout}-{axis}-{trial_number:02d}"
    runtime = RUNTIME / trial_id
    runtime.mkdir(parents=True, exist_ok=False)
    prefix = layout.replace("-", "_") + "_test_world"
    server = sitl = pose = link = None
    samples = []
    motor_values = []
    shared = {"phase": "startup"}
    thread = None

    try:
        server_log = (runtime / "gazebo.log").open("w")

        server = subprocess.Popen(
            ["bash", str(PACKAGE / "run_gazebo_mac.sh"), layout, "server"],
            cwd=ROOT, stdout=server_log, stderr=subprocess.STDOUT, start_new_session=True,
        )

        time.sleep(2.5)

        sitl_log = (runtime / "sitl.log").open("w")
        defaults = ",".join((
            str(ROOT / "Tools/autotest/default_params" / FRAMES[layout][0]),
            str(PACKAGE / "generated/params" / f"{layout}.parm"),
        ))

        sitl = subprocess.Popen(
            [str(ROOT / "build/sitl/bin/ardusub"), "--model", "JSON", "--defaults", defaults,
             "-I1", "--sysid", "2", "--home", "-35.1192416,150.7296753,0.0,0"],
            cwd=runtime, stdout=sitl_log, stderr=subprocess.STDOUT, start_new_session=True,
        )

        link = wait_for_master()
        expected_frame = FRAMES[layout][1]
        actual_frame = get_parameter(link, "FRAME_CONFIG")

        if actual_frame != expected_frame: raise RuntimeError(f"FRAME_CONFIG {actual_frame}, expected {expected_frame}")
        
        link.mav.command_long_send(
            2, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 100000, 0, 0, 0, 0, 0,
        )

        name = "throttle" if axis == "dive" else axis
        rcmap = get_parameter(link, f"RCMAP_{name.upper()}")
        pwm = 1400 if axis == "dive" else 1600
        env = dict(os.environ, GZ_IP="127.0.0.1", GZ_PARTITION=f"{layout}_sitl_demo")
        pose = subprocess.Popen(
            ["gz", "topic", "--json-output", "-e", "-t", f"/world/{prefix}/pose/info"],
            cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, bufsize=1, start_new_session=True,
        )

        start = time.monotonic()

        thread = threading.Thread(
            target=pose_reader, args=(pose, trial_id, layout, axis, shared, samples, start), daemon=True,
        )

        thread.start()

        armed = arm(link)

        if not armed: raise RuntimeError("ArduSub did not arm")

        hold_rc(link, rcmap, 1500, 1.5, "baseline", shared, FRAMES[layout][2], motor_values)
        hold_rc(link, rcmap, pwm, 2.0, "command", shared, FRAMES[layout][2], motor_values)
        hold_rc(link, rcmap, 1500, 0.5, "release", shared, FRAMES[layout][2], motor_values)
        
        summary = trial_summary(trial_id, layout, axis, rcmap, pwm, armed, samples, motor_values)
        
        return summary, list(samples)
    finally:
        if link is not None:
            try:
                link.mav.command_long_send(2, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
                link.close()
            except (ConnectionError, OSError):
                pass

        for process in (pose, sitl, server): 
            stop_process(process)

        if thread is not None: 
            thread.join(timeout=2)

        for file in (locals().get("server_log"), locals().get("sitl_log")):
            if file is not None:
                file.close()

        time.sleep(1)


def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--layout", choices=FRAMES, action="append")
    parser.add_argument("--axis", choices=AXES, action="append")
    parser.add_argument("--run-number", type=int, default=1, help="Use 2 or higher for a fresh repeat")
    
    args = parser.parse_args()

    if args.run_number < 1: parser.error("--run-number must be positive")

    layouts = args.layout or list(FRAMES)
    axes = args.axis or list(AXES)
    suffix = "" if args.run_number == 1 else f"_run{args.run_number:02d}"
    summary_path = OUTPUT / f"trial_summary{suffix}.csv"
    samples_path = OUTPUT / f"gazebo_pose_samples{suffix}.csv"

    if summary_path.exists() or samples_path.exists(): parser.error("Output already exists; choose a new --run-number")
    
    summaries = []
    samples = []

    for layout in layouts:
        for axis in axes:
            trial_id = f"{layout}-{axis}-{args.run_number:02d}"

            print(f"Starting {trial_id}", flush=True)

            try:
                summary, series = run_trial(layout, axis, args.run_number)
                
                summaries.append(summary)
                samples.extend(series)

                print(f"Finished {trial_id}: {summary[11:18]}", flush=True)
            except Exception as exc:
                print(f"Failed {trial_id}: {exc}", flush=True)
                raise
            with summary_path.open("w", newline="") as file:
                writer = csv.writer(file)

                writer.writerow(SUMMARY_COLUMNS)
                writer.writerows(summaries)
            with samples_path.open("w", newline="") as file:
                writer = csv.writer(file)

                writer.writerow(SAMPLE_COLUMNS)
                writer.writerows(samples)


if __name__ == "__main__":
    main()