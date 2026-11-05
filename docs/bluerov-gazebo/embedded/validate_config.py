import json
import math
import sys
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
REPO = PACKAGE.parents[1]
sys.path.insert(0, str(PACKAGE))

import generate  # noqa: E402
import verify  # noqa: E402

class ConfigError(ValueError):
    pass

def keys(value, required, context):
    if not isinstance(value, dict) or set(value) != set(required):
        raise ConfigError(f"{context} keys must be {sorted(required)}")

def number(value, context, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ConfigError(f"{context} must be a finite number")
    
    if positive and value <= 0:
        raise ConfigError(f"{context} must be positive")
    
    return float(value)

def vector(value, context, positive=False):
    if not isinstance(value, list) or len(value) != 3:
        raise ConfigError(f"{context} must contain three numbers")
    
    return tuple(number(item, f"{context}[{i}]", positive) for i, item in enumerate(value))

def wrench_rows(positions, factors):
    rows = []

    for position, factor in zip(positions, factors):
        x, y, z = position
        _, _, _, throttle, forward, lateral = factor
        fx, fy, fz = forward, lateral, -throttle
        norm = math.sqrt(fx * fx + fy * fy + fz * fz)
        fx, fy, fz = fx / norm, fy / norm, fz / norm
        rows.append((y * fz - z * fy, z * fx - x * fz, x * fy - y * fx, fz, fx, fy))
    return rows

def validate(config):
    keys(config, ("schema_version", "plant", "architecture", "scenario"), "configuration")
    if type(config["schema_version"]) is not int or config["schema_version"] != 1: raise ConfigError("schema_version must be 1")
    if config["architecture"] not in ("pi_pixhawk", "pi_navigator"): raise ConfigError("architecture must be pi_pixhawk or pi_navigator")
    if config["scenario"] != "baseline": raise ConfigError("step 2 supports only the baseline scenario")

    plant = config["plant"]
    keys(plant, ("frame", "body_m", "mass_kg", "displaced_volume_m3",
                 "center_of_buoyancy_z_m", "motor_positions_m", "motor_channels"), "plant")
   
    frame = plant["frame"]

    if not isinstance(frame, str) or frame not in generate.FRAMES: raise ConfigError(f"unsupported ArduSub frame: {frame}")
    
    source = (REPO / "libraries/AP_Motors/AP_Motors6DOF.cpp").read_text()
    expected = verify.source_factors(source, verify.SOURCE_CASES[frame])
    factors = generate.FRAMES[frame]["factors"]

    if expected != [tuple(float(value) for value in row) for row in factors]: raise ConfigError(f"{frame} motor factors differ from checked-out ArduSub source")

    body_m = vector(plant["body_m"], "body_m", positive=True)
    mass_kg = number(plant["mass_kg"], "mass_kg", positive=True)
    volume = number(plant["displaced_volume_m3"], "displaced_volume_m3", positive=True)
    cob_z = number(plant["center_of_buoyancy_z_m"], "center_of_buoyancy_z_m")
    
    if mass_kg <= generate.T200_MASS_KG * len(factors): raise ConfigError("body mass would be nonpositive after thruster links")
    if not isinstance(plant["motor_positions_m"], list) or len(plant["motor_positions_m"]) != len(factors): raise ConfigError(f"motor_positions_m must have {len(factors)} entries for {frame}")
    
    positions = tuple(vector(item, f"motor_positions_m[{i}]")
                      for i, item in enumerate(plant["motor_positions_m"]))
    channels = plant["motor_channels"]

    if not isinstance(channels, list) or any(type(item) is not int for item in channels): raise ConfigError("motor_channels must be an integer list")
    if channels != list(range(len(factors))): raise ConfigError("motor_channels must preserve the checked-out ArduSub motor order")
    
    wrench = wrench_rows(positions, factors)
    rank = verify.matrix_rank([tuple(row[col] for row in wrench) for col in range(6)])
    expected_rank = 5 if frame == "bluerov2" else 6

    if rank != expected_rank: raise ConfigError(f"thruster geometry rank {rank} does not match {frame} expected rank {expected_rank}")
    
    for axis in range(6):
        axis_factors = [row[axis] for row in factors]

        if not any(axis_factors): continue

        response = sum(wrench[motor][axis] * axis_factors[motor] for motor in range(len(factors)))

        if (axis == 3 and response >= 0) or (axis != 3 and response <= 0): raise ConfigError(f"thruster geometry reverses or cancels axis {axis}")

    return {
        "schema_version": 1,
        "plant": {
            "frame": frame,
            "frame_config": generate.FRAMES[frame]["frame_config"],
            "body_m": body_m,
            "mass_kg": mass_kg,
            "displaced_volume_m3": volume,
            "center_of_buoyancy_z_m": cob_z,
            "motor_positions_m": positions,
            "motor_channels": channels,
            "wrench_rank": rank,
        },
        "architecture": config["architecture"],
        "scenario": "baseline",
    }


def load_and_validate(path):
    return validate(json.loads(Path(path).read_text()))

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("config", type=Path)
    args = parser.parse_args()

    try:
        resolved = load_and_validate(args.config)
    except (ConfigError, ValueError, KeyError) as exc:
        parser.error(str(exc))

    print(json.dumps(resolved, indent=2))
