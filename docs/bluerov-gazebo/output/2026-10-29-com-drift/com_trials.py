import argparse
import csv
import importlib.util
import os
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

from pymavlink import mavutil


ROOT = Path(__file__).resolve().parents[4]
PACKAGE = ROOT / "docs/bluerov-gazebo"
OUTPUT = Path(__file__).resolve().parent
BASELINE_SCRIPT = PACKAGE / "output/2026-10-24-frame-screening/run_trials.py"
INSTANCE = 2
SYSID = 3
JSON_PORT = 9022
MAVLINK_PORT = 5780
BASE_BODY = (0.6, 0.45, 0.16)

CASES = {
    "compact_light": ((0.45, 0.32, 0.15), 7.5, 0.0),
    "compact_mid": ((0.45, 0.32, 0.15), 11.5, 0.0),
    "roomy_light": ((0.58, 0.44, 0.18), 7.5, 0.0),
    "under18": ((0.46, 0.34, 0.25), 15.0, 0.237),
    "18to25": ((0.60, 0.48, 0.28), 22.0, 0.237),
}

CENTERS = {
    "center": (0.0, 0.0, 0.0),
    "forward5cm": (0.05, 0.0, 0.0),
    "side5cm": (0.0, 0.05, 0.0),
}

CONDITION_COMMANDS = {
    "neutral": ("THROTTLE", 1500),
    "dive": ("THROTTLE", 1400),
    "forward": ("FORWARD", 1600),
    "lateral": ("LATERAL", 1600),
}
CONDITIONS = tuple(CONDITION_COMMANDS)

SUMMARY_COLUMNS = (
    "trial_id", "case_id", "layout", "mass_kg", "com_position", "target_com_x_m", "target_com_y_m",
    "target_com_z_m", "condition", "repeat", "command_pwm", "armed", "frame_config", "pose_samples",
    "baseline_samples", "command_samples", "delta_x_m", "delta_y_m", "delta_z_m", "delta_roll_deg",
    "delta_pitch_deg", "delta_yaw_deg", "motor_pwm_min", "motor_pwm_max", "status", "error",
)

SAMPLE_COLUMNS = ("trial_id", "phase", "wall_elapsed_s", "sim_time_s", "x_m", "y_m", "z_m",
                  "roll_deg", "pitch_deg", "yaw_deg")

sys.path.insert(0, str(PACKAGE))
import generate  # noqa: E402

spec = importlib.util.spec_from_file_location("frame_baseline", BASELINE_SCRIPT)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)


def case_geometry(case_id, layout):
    dimensions, mass_kg, heavy_extra = CASES[case_id]
    body = (dimensions[0], dimensions[1] + (heavy_extra if layout == "bluerov2-heavy" else 0), dimensions[2])
    positions = []

    for index, position in enumerate(generate.FRAMES[layout]["positions"]):
        width = dimensions[1] if layout == "bluerov2-heavy" and index < 4 else body[1]
        scales = (body[0] / BASE_BODY[0], width / BASE_BODY[1], body[2] / BASE_BODY[2])
        positions.append(tuple(position[axis] * scales[axis] for axis in range(3)))
    
    return body, mass_kg, tuple(positions)

def set_total_com(model_path, target):
    tree = ET.parse(model_path)
    model = tree.getroot().find("model")
    body = model.find("link[@name='rov_body']")
    body_mass = float(body.findtext("inertial/mass"))
    motors = [link for link in model.findall("link") if link.get("name", "").startswith("motor_")]
    motor_masses = [float(link.findtext("inertial/mass")) for link in motors]
    motor_positions = [tuple(map(float, link.findtext("pose").split()[:3])) for link in motors]
    total_mass = body_mass + sum(motor_masses)
    body_center = tuple((total_mass * target[axis] - sum(mass * position[axis]
                         for mass, position in zip(motor_masses, motor_positions))) / body_mass
                        for axis in range(3))
    
    ET.SubElement(body.find("inertial"), "pose").text = " ".join(f"{value:.9f}" for value in body_center) + " 0 0 0"
    ET.indent(tree, space="  ")
    
    tree.write(model_path, encoding="utf-8", xml_declaration=True)
    
    actual = tuple((body_mass * body_center[axis] + sum(mass * position[axis]
                    for mass, position in zip(motor_masses, motor_positions))) / total_mass
                   for axis in range(3))
    if max(abs(actual[axis] - target[axis]) for axis in range(3)) > 1e-8:
        raise AssertionError("total center of mass does not match target")
    
    return body_center

def prepare_models():
    connector = PACKAGE / ".local/ardupilot_gazebo/build/libArduPilotPlugin.dylib"
    
    if not connector.is_file(): raise FileNotFoundError(connector)

    rows = []

    for case_id in CASES:
        for layout in generate.FRAMES:
            body, mass_kg, positions = case_geometry(case_id, layout)
            frame_spec = dict(generate.FRAMES[layout], positions=positions)
            
            for center_name, target in CENTERS.items():
                generated = OUTPUT / "models" / case_id / layout / center_name
                model_name = layout.replace("-", "_") + "_test"
                model_path = generated / "models" / model_name / "model.sdf"
                
                if model_path.exists(): raise FileExistsError(f"Refusing to replace existing model: {model_path}")
                
                (generated / "worlds").mkdir(parents=True)
                (generated / "params").mkdir()
                
                generate.write_model(model_path.parent, layout, frame_spec, connector, "dylib",
                                     body_m=body, mass_kg=mass_kg)
                generate.write_world(generated / "worlds" / f"{layout}.sdf", model_name, "dylib",
                                     displaced_volume_m3=mass_kg / 1000,
                                     center_of_buoyancy_z_m=0.02 * body[2] / BASE_BODY[2])
                
                (generated / "params" / f"{layout}.parm").write_text(
                    f"FRAME_CONFIG {frame_spec['frame_config']}\n")
                
                body_center = set_total_com(model_path, target)
                tree = ET.parse(model_path)
                plugin = tree.getroot().find("model/plugin[@name='ArduPilotPlugin']/fdm_port_in")
                plugin.text = str(JSON_PORT)

                tree.write(model_path, encoding="utf-8", xml_declaration=True)
                rows.append((case_id, layout, center_name, *body, mass_kg, mass_kg / 1000,
                             *target, *body_center, len(frame_spec["factors"])))
    
    path = OUTPUT / "model_definitions.csv"

    with path.open("w", newline="") as file:
        writer = csv.writer(file)

        writer.writerow(("case_id", "layout", "com_position", "body_length_m", "body_width_m", "body_height_m",
                         "total_mass_kg", "displaced_volume_m3", "target_com_x_m", "target_com_y_m", "target_com_z_m",
                         "body_inertial_x_m", "body_inertial_y_m", "body_inertial_z_m", "thruster_count"))
        
        writer.writerows(rows)

    print(f"Prepared {len(rows)} models in {OUTPUT / 'models'}", flush=True)

def wait_for_sitl(timeout=25):
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        try:
            link = mavutil.mavlink_connection(f"tcp:127.0.0.1:{MAVLINK_PORT}", source_system=255)
            
            if link.wait_heartbeat(timeout=2): return link

            link.close()
        except (ConnectionError, OSError):
            pass
        time.sleep(0.25)

    raise TimeoutError("No Ardusub heartbeat")

def get_parameter(link, name, timeout=6):
    link.mav.param_request_read_send(SYSID, 1, name.encode(), -1)

    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        message = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
        
        if message is None: continue

        param_id = message.param_id.decode(errors="replace") if isinstance(message.param_id, bytes) else message.param_id
        
        if param_id.rstrip("\x00") == name: return int(round(message.param_value))
    
    raise TimeoutError(f"No parameter response for {name}")

def arm(link):
    deadline = time.monotonic() + 15

    while time.monotonic() < deadline:
        link.mav.command_long_send(SYSID, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
        
        step_end = time.monotonic() + 1

        while time.monotonic() < step_end:
            message = link.recv_match(type="HEARTBEAT", blocking=True, timeout=0.3)
            
            if message and message.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED: return True
    
    return False

def hold_rc(link, channel, pwm, seconds, phase, shared, motor_count, motor_values):
    shared["phase"] = phase

    end = time.monotonic() + seconds

    while time.monotonic() < end:
        channels = [1500] * 8

        channels[channel - 1] = pwm

        link.mav.rc_channels_override_send(SYSID, 1, *channels)

        if phase == "command":
            while True:
                message = link.recv_match(type="SERVO_OUTPUT_RAW", blocking=False)
                
                if message is None: break

                motor_values.extend(getattr(message, f"servo{index}_raw")
                                    for index in range(1, motor_count + 1))
        
        time.sleep(0.1)

def summarize_pose(samples):
    neutral = [row for row in samples if row[3] == "baseline"]
    driven = [row for row in samples if row[3] == "command"]

    if len(neutral) < 4 or len(driven) < 4: raise ValueError(f"Too few pose samples: baseline={len(neutral)}, command={len(driven)}")
    
    start = [base.median([row[index] for row in neutral[-4:]]) for index in range(6, 12)]
    end = [base.median([row[index] for row in driven[-4:]]) for index in range(6, 12)]
    translation = [end[index] - start[index] for index in range(3)]
    rotation = [base.angle_difference(end[index], start[index]) for index in range(3, 6)]
    
    return len(neutral), len(driven), (*translation, *rotation)

def run_trial(case_id, layout, center_name, condition, repeat):
    trial_id = f"{case_id}-{layout}-{center_name}-{condition}-{repeat:02d}"
    generated = OUTPUT / "models" / case_id / layout / center_name
    runtime = OUTPUT / "runtime" / trial_id
    runtime.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, GZ_IP="127.0.0.1", GZ_PARTITION=f"mate_com_{trial_id}",
               GZ_SIM_RESOURCE_PATH=str(generated / "models"),
               GZ_SIM_SYSTEM_PLUGIN_PATH=str(PACKAGE / ".local/rov-sim"))
    
    server = sitl = pose = link = thread = None
    logs = []
    samples = []
    motor_values = []
    shared = {"phase": "startup"}
    start = time.monotonic()
    rcmap_name, pwm = CONDITION_COMMANDS[condition]
    frame_spec = generate.FRAMES[layout]
    row = dict.fromkeys(SUMMARY_COLUMNS, "")

    row.update(trial_id=trial_id, case_id=case_id, layout=layout, mass_kg=CASES[case_id][1],
               com_position=center_name, target_com_x_m=CENTERS[center_name][0],
               target_com_y_m=CENTERS[center_name][1], target_com_z_m=CENTERS[center_name][2],
               condition=condition, repeat=repeat, command_pwm=pwm,
               frame_config=frame_spec["frame_config"], status="started")
    
    try:
        server_log = (runtime / "gazebo.log").open("w")

        logs.append(server_log)

        server = subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "2",
                                   str(generated / "worlds" / f"{layout}.sdf")],
                                  cwd=ROOT, env=env, stdout=server_log, stderr=subprocess.STDOUT,
                                  start_new_session=True)
        
        time.sleep(2.5)

        if server.poll() is not None: raise RuntimeError("Gazebo server exited during startup")
        
        defaults_name = "sub-6dof.parm" if layout == "bluerov2-heavy" else "sub.parm"
        defaults = ",".join((str(ROOT / "Tools/autotest/default_params" / defaults_name),
                             str(generated / "params" / f"{layout}.parm")))
        sitl_log = (runtime / "sitl.log").open("w")

        logs.append(sitl_log)

        sitl = subprocess.Popen([str(ROOT / "build/sitl/bin/ardusub"), "--model", "JSON",
                                 "--defaults", defaults, f"-I{INSTANCE}", "--sysid", str(SYSID),
                                 "--home", "-35.1192416,150.7296753,0.0,0"],
                                cwd=runtime, stdout=sitl_log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        
        link = wait_for_sitl()
        actual_frame = get_parameter(link, "FRAME_CONFIG")

        if actual_frame != frame_spec["frame_config"]: raise RuntimeError(f"FRAME_CONFIG {actual_frame} != {frame_spec['frame_config']}")
        
        link.mav.command_long_send(SYSID, 1, mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                   mavutil.mavlink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW, 100000, 0, 0, 0, 0, 0)
        
        command_channel = get_parameter(link, f"RCMAP_{rcmap_name}")
        model_name = layout.replace("-", "_") + "_test"
        world_name = model_name + "_world"

        pose = subprocess.Popen(["gz", "topic", "--json-output", "-e", "-t",
                                 f"/world/{world_name}/pose/info"], cwd=ROOT, env=env,
                                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
                                text=True, bufsize=1, start_new_session=True)
        
        thread = threading.Thread(target=base.pose_reader,
                                  args=(pose, trial_id, layout, condition, shared, samples, start), daemon=True)
        
        thread.start()

        row["armed"] = arm(link)

        if not row["armed"]: raise RuntimeError("ArduSub did not arm")

        motor_count = len(frame_spec["factors"])

        hold_rc(link, command_channel, 1500, 1.5, "baseline", shared, motor_count, motor_values)
        hold_rc(link, command_channel, pwm, 2.0, "command", shared, motor_count, motor_values)
        hold_rc(link, command_channel, 1500, 0.5, "release", shared, motor_count, motor_values)
        
        baseline_count, command_count, movement = summarize_pose(samples)

        row.update(pose_samples=len(samples), baseline_samples=baseline_count, command_samples=command_count,
                   delta_x_m=round(movement[0], 5), delta_y_m=round(movement[1], 5),
                   delta_z_m=round(movement[2], 5), delta_roll_deg=round(movement[3], 4),
                   delta_pitch_deg=round(movement[4], 4), delta_yaw_deg=round(movement[5], 4),
                   motor_pwm_min=min(motor_values) if motor_values else "",
                   motor_pwm_max=max(motor_values) if motor_values else "", status="measured")
    
    except Exception as error:
        row.update(status="failed", error=str(error), pose_samples=len(samples))
    finally:
        if link is not None:
            try:
                link.mav.command_long_send(SYSID, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                           0, 0, 0, 0, 0, 0, 0, 0)
                link.close()
            except (ConnectionError, OSError):
                pass
        for process in (pose, sitl, server):
            base.stop_process(process)

        if thread is not None: thread.join(timeout=2)

        for log in logs:
            log.close()

        time.sleep(0.5)

    sample_rows = ((trial_id, sample[3], sample[4], sample[5], *sample[6:]) for sample in samples)
    return row, list(sample_rows)

def load_rows(path):
    if not path.exists(): return []

    with path.open(newline="") as file:
        return list(csv.DictReader(file))

def append_rows(path, columns, rows):
    new_file = not path.exists()
    with path.open("a", newline="") as file:
        writer = csv.writer(file)
        if new_file: writer.writerow(columns)

        writer.writerows(rows)

def run_all(repeats, limit, cases, layouts, centers, conditions):
    completed = {row["trial_id"] for row in load_rows(OUTPUT / "trial_summary.csv") if row["status"] == "measured"}
    count = 0

    for case_id in cases:
        for layout in layouts:
            for center_name in centers:
                for repeat in range(1, repeats + 1):
                    ordered_conditions = conditions if repeat % 2 else tuple(reversed(conditions))
                    for condition in ordered_conditions:
                        trial_id = f"{case_id}-{layout}-{center_name}-{condition}-{repeat:02d}"
                        
                        if trial_id in completed: continue

                        print(f"Starting {trial_id}", flush=True)
                        
                        row, samples = run_trial(case_id, layout, center_name, condition, repeat)
                        
                        append_rows(OUTPUT / "trial_summary.csv", SUMMARY_COLUMNS,
                                    (tuple(row[key] for key in SUMMARY_COLUMNS),))
                        
                        append_rows(OUTPUT / "pose_samples.csv", SAMPLE_COLUMNS, samples)
                        
                        print(f"{row['status']} {trial_id}: z={row['delta_z_m']} roll={row['delta_roll_deg']} "
                              f"pitch={row['delta_pitch_deg']} {row['error']}", flush=True)
                        
                        if row["status"] != "measured": raise RuntimeError(f"Trial failed: {trial_id}: {row['error']}")
                        
                        completed.add(trial_id)
                        count += 1

                        if limit and count >= limit: return

def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("action", choices=("prepare", "run"))
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--limit", type=int, default=0, help="stop after this many new trials")
    parser.add_argument("--case", choices=CASES, action="append")
    parser.add_argument("--layout", choices=generate.FRAMES, action="append")
    parser.add_argument("--center", choices=CENTERS, action="append")
    parser.add_argument("--condition", choices=CONDITIONS, action="append")
    
    args = parser.parse_args()
    
    if args.repeats < 2: parser.error("at least two repeats are required")
    
    if args.action == "prepare":
        prepare_models()
    else:
        run_all(args.repeats, args.limit, args.case or CASES,
                args.layout or generate.FRAMES, args.center or CENTERS,
                tuple(dict.fromkeys(args.condition or CONDITIONS)))

if __name__ == "__main__":
    main()
