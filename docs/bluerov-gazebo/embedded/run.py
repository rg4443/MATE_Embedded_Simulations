import argparse
import json
import subprocess
import sys
import uuid
from datetime import datetime, timezone
from pathlib import Path

from validate_config import ConfigError, PACKAGE, REPO, load_and_validate

sys.path.insert(0, str(PACKAGE))
import generate # noqa: E402
import verify # noqa: E402


def git_state(path):
    try:
        revision = subprocess.check_output(["git", "-C", str(path), "rev-parse", "HEAD"], text=True).strip()
        dirty = bool(subprocess.check_output(["git", "-C", str(path), "status", "--porcelain"], text=True).strip())
    except (subprocess.CalledProcessError, FileNotFoundError):
        return {"revision": None, "dirty": None}
    
    return {"revision": revision, "dirty": dirty}

def prepare(config_path, output_root, connector_lib, plugin_suffix):
    resolved = load_and_validate(config_path)
    connector_lib = connector_lib.resolve()

    if not connector_lib.is_file() or connector_lib.suffix != f".{plugin_suffix}":
        raise ConfigError(f"missing .{plugin_suffix} connector library: {connector_lib}")
    
    plugin_dir = PACKAGE / ".local" / "rov-sim" / "plugins"

    for name in ("EER_Thruster", "EER_Hydrodynamics", "EER_Buoyancy"):
        if not (plugin_dir / f"lib{name}.{plugin_suffix}").is_file():
            raise ConfigError(f"missing Gazebo plugin: {name}")

    output_root.mkdir(parents=True, exist_ok=True)

    stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    run_id = f"{stamp}-{uuid.uuid4().hex[:8]}"
    output = output_root / run_id

    output.mkdir(exist_ok=False)

    generated = output / "generated"

    for folder in ("models", "worlds", "params"):
        (generated / folder).mkdir(parents=True, exist_ok=False)

    plant = resolved["plant"]
    frame = plant["frame"]
    spec = dict(generate.FRAMES[frame], positions=plant["motor_positions_m"])
    
    model_name = generate.write_model(
        generated / "models" / (frame.replace("-", "_") + "_test"),
        frame, spec, connector_lib, plugin_suffix,
        body_m=plant["body_m"], mass_kg=plant["mass_kg"],
        channels=plant["motor_channels"],
    )

    generate.write_world(
        generated / "worlds" / f"{frame}.sdf", model_name, plugin_suffix,
        displaced_volume_m3=plant["displaced_volume_m3"],
        center_of_buoyancy_z_m=plant["center_of_buoyancy_z_m"],
    )
    (generated / "params" / f"{frame}.parm").write_text(f"FRAME_CONFIG {plant['frame_config']}\n")
    
    verify.verify_selected(generated, plant)

    (output / "input.json").write_text(Path(config_path).read_text())
    (output / "resolved.json").write_text(json.dumps(resolved, indent=2) + "\n")
    
    manifest = {
        "run_id": run_id,
        "status": "prepared_not_executed",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "configuration": "resolved.json",
        "world": f"generated/worlds/{frame}.sdf",
        "parameters": f"generated/params/{frame}.parm",
        "model": f"generated/models/{model_name}/model.sdf",
        "connector_library": str(connector_lib),
        "plugin_suffix": plugin_suffix,
        "revisions": {
            "mate_checkout": git_state(REPO),
            "ardupilot_gazebo": git_state(PACKAGE / ".local" / "ardupilot_gazebo"),
            "rov_sim": git_state(PACKAGE / ".local" / "rov-sim"),
        },
        "fidelity": "F1 topology definition only; no architecture processes launched",
    }

    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    
    return output

def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output-root", type=Path, default=Path(__file__).resolve().parent / "runs")
    parser.add_argument("--plugin-suffix", choices=("dylib", "so"),
                        default="dylib" if sys.platform == "darwin" else "so")
    parser.add_argument("--connector-lib", type=Path)
    
    args = parser.parse_args()
    connector_lib = args.connector_lib or PACKAGE / ".local" / "ardupilot_gazebo" / "build" / (
        "libArduPilotPlugin." + args.plugin_suffix)
    
    try:
        output = prepare(args.config, args.output_root, connector_lib, args.plugin_suffix)
    except (ConfigError, ValueError, OSError, AssertionError) as exc:
        parser.error(str(exc))
    
    print(output)