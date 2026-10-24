import argparse
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

FRAMES = {
    "bluerov1": {
        "frame_config": 0,
        "factors": (
            (0, 0, -1, 0, 1, 0),
            (0, 0, 1, 0, 1, 0),
            (-0.5, 0.5, 0, 0.45, 0, 0),
            (0.5, 0.5, 0, 0.45, 0, 0),
            (0, -1, 0, 1, 0, 0),
            (-0.25, 0, 0, 0, 0, 1),
        ),
        "positions": (
            (-0.15, 0.20, 0),
            (-0.15, -0.20, 0),
            (0.20, 0.15, 0),
            (0.20, -0.15, 0),
            (-0.18, 0, 0),
            (0, 0, 0.05),
        ),
    },
    "bluerov2": {
        "frame_config": 1,
        "factors": (
            (0, 0, 1, 0, -1, 1),
            (0, 0, -1, 0, -1, -1),
            (0, 0, -1, 0, 1, 1),
            (0, 0, 1, 0, 1, -1),
            (1, 0, 0, -1, 0, 0),
            (-1, 0, 0, -1, 0, 0),
        ),
        "positions": (
            (0.25, 0.20, 0),
            (0.25, -0.20, 0),
            (-0.25, 0.20, 0),
            (-0.25, -0.20, 0),
            (0, 0.20, 0),
            (0, -0.20, 0),
        ),
    },
    "bluerov2-heavy": {
        "frame_config": 2,
        "factors": (
            (0, 0, 1, 0, -1, 1),
            (0, 0, -1, 0, -1, -1),
            (0, 0, -1, 0, 1, 1),
            (0, 0, 1, 0, 1, -1),
            (1, -1, 0, -1, 0, 0),
            (-1, -1, 0, -1, 0, 0),
            (1, 1, 0, -1, 0, 0),
            (-1, 1, 0, -1, 0, 0),
        ),
        "positions": (
            (0.25, 0.20, 0),
            (0.25, -0.20, 0),
            (-0.25, 0.20, 0),
            (-0.25, -0.20, 0),
            (0.25, 0.20, 0),
            (0.25, -0.20, 0),
            (-0.25, 0.20, 0),
            (-0.25, -0.20, 0),
        ),
    },
}


def text(parent, tag, value, **attributes):
    child = ET.SubElement(parent, tag, attributes)
    child.text = str(value)

    return child

def inertial(link, mass, ixx, iyy, izz):
    element = ET.SubElement(link, "inertial")

    text(element, "mass", mass)

    tensor = ET.SubElement(element, "inertia")

    for tag, value in (("ixx", ixx), ("iyy", iyy), ("izz", izz)):
        text(tensor, tag, value)

def pose_string(values):
    return " ".join(f"{value:.8f}" for value in values)

def joint_orientation(axis):
    """Return roll, pitch, yaw rotating local +Z onto a Gazebo model axis."""
    roll = -math.asin(max(-1.0, min(1.0, axis[1])))
    pitch = math.atan2(axis[0], axis[2])

    return roll, pitch, 0.0

def motor_axis(factors):
    """Joint +Z direction, allowing for ESC thrust decreasing with PWM."""
    _, _, _, throttle, forward, lateral = factors
    axis = (-forward, lateral, -throttle)
    size = math.sqrt(sum(value * value for value in axis))

    return tuple(value / size for value in axis)


def add_thruster(model, namespace, number, position, factors, plugin_suffix):
    x, y, z = position
    model_position = (x, -y, -z)
    axis = motor_axis(factors)
    roll, pitch, yaw = joint_orientation(axis)
    pose = pose_string((*model_position, roll, pitch, yaw))
    name = f"motor_{number}"
    link = ET.SubElement(model, "link", {"name": name})

    text(link, "pose", pose)
    inertial(link, 0.002, 0.00001, 0.00001, 0.00001)

    visual = ET.SubElement(link, "visual", {"name": "housing"})
    geometry = ET.SubElement(visual, "geometry")
    cylinder = ET.SubElement(geometry, "cylinder")

    text(cylinder, "radius", 0.04)
    text(cylinder, "length", 0.07)

    joint = ET.SubElement(model, "joint", {"name": f"{name}_joint", "type": "revolute"})

    text(joint, "parent", "rov_body")
    text(joint, "child", name)
    text(joint, "pose", pose, relative_to="__model__")

    axis_element = ET.SubElement(joint, "axis")

    text(axis_element, "xyz", "0 0 1")

    thruster = ET.SubElement(model, "plugin", {
        "filename": f"plugins/libEER_Thruster.{plugin_suffix}",
        "name": "gz::sim::systems::EER_Thruster",
    })

    for tag, value in (
        ("namespace", namespace),
        ("joint_name", f"{name}_joint"),
        ("use_esc_cmd", "true"),
        ("fluid_density", 1000),
        ("topic", name),
        ("propeller_diameter", 0.076),
    ):
        text(thruster, tag, value)

    return name

def add_connector(model, namespace, count, connector_lib):
    plugin = ET.SubElement(model, "plugin", {
        "filename": str(connector_lib),
        "name": "ArduPilotPlugin",
    })

    for tag, value in (
        ("fdm_addr", "127.0.0.1"),
        ("fdm_port_in", 9012),
        ("connectionTimeoutMaxCount", 5),
        ("lock_step", 0),
        ("no_time_sync", 1),
        ("have_32_channels", 0),
        ("imuName", "imu"),
    ):
        text(plugin, tag, value)

    text(plugin, "modelXYZToAirplaneXForwardZDown", "0 0 0 180 0 0", degrees="true")
    text(plugin, "gazeboXYZToNED", "0 0 0 180 0 0", degrees="true")

    for index in range(count):
        name = f"motor_{index + 1}"
        control = ET.SubElement(plugin, "control", {"channel": str(index)})

        for tag, value in (
            ("jointName", f"{name}_joint"),
            ("type", "COMMAND"),
            ("cmd_topic", f"/{namespace}/{name}"),
            ("multiplier", 255),
            ("offset", 0),
            ("servo_min", 1100),
            ("servo_max", 1900),
        ):
            text(control, tag, value)

def add_hydrodynamics(model, plugin_suffix):
    plugin = ET.SubElement(model, "plugin", {
        "filename": f"plugins/libEER_Hydrodynamics.{plugin_suffix}",
        "name": "gz::sim::systems::EER_Hydrodynamics",
    })

    for tag, value in (
        ("link_name", "rov_body"),
        ("water_density", 1000),
        ("xDotU", 1.32246275),
        ("yDotV", 1.04396026),
        ("zDotW", 6.64814797),
        ("kDotP", 0.03548512),
        ("mDotQ", 0.02818614),
        ("nDotR", 0.01814365),
        ("xUabsU", -40),
        ("xU", -30),
        ("yVabsV", -40),
        ("yV", -25),
        ("zWabsW", -50),
        ("zW", -50),
        ("kPabsP", -20),
        ("kP", -20),
        ("mQabsQ", -125),
        ("mQ", -50),
        ("nRabsR", -20),
        ("nR", -20),
    ):
        text(plugin, tag, value)

def write_model(directory, frame, spec, connector_lib, plugin_suffix):
    directory.mkdir(parents=True, exist_ok=True)

    model_name = frame.replace("-", "_") + "_test"
    root = ET.Element("sdf", {"version": "1.7"})
    model = ET.SubElement(root, "model", {"name": model_name})
    body = ET.SubElement(model, "link", {"name": "rov_body"})

    inertial(body, 12 - 0.002 * len(spec["factors"]), 0.18, 0.25, 0.32)

    for tag in ("visual", "collision"):
        part = ET.SubElement(body, tag, {"name": "body"})
        geometry = ET.SubElement(part, "geometry")
        box = ET.SubElement(geometry, "box")
        text(box, "size", "0.6 0.45 0.16")

    imu = ET.SubElement(body, "sensor", {"name": "imu", "type": "imu"})

    text(imu, "always_on", 1)
    text(imu, "update_rate", 100)

    for number, (position, factors) in enumerate(zip(spec["positions"], spec["factors"]), 1):
        add_thruster(model, model_name, number, position, factors, plugin_suffix)

    add_hydrodynamics(model, plugin_suffix)
    add_connector(model, model_name, len(spec["factors"]), connector_lib)

    ET.indent(root, space="  ")
    ET.ElementTree(root).write(directory / "model.sdf", encoding="utf-8", xml_declaration=True)

    config = ET.Element("model")

    text(config, "name", model_name)
    text(config, "version", "1.0")

    sdf = text(config, "sdf", "model.sdf", version="1.7")
    sdf.tail = "\n"

    ET.ElementTree(config).write(directory / "model.config", encoding="utf-8", xml_declaration=True)
    
    return model_name

def write_world(path, model_name, plugin_suffix):
    root = ET.Element("sdf", {"version": "1.10"})
    world = ET.SubElement(root, "world", {"name": model_name + "_world"})
    physics = ET.SubElement(world, "physics", {"name": "3ms", "type": "ignored"})

    text(physics, "max_step_size", 0.003)
    text(physics, "real_time_factor", 1.0)

    for filename, plugin_name in (
        ("gz-sim-physics-system", "gz::sim::systems::Physics"),
        ("gz-sim-imu-system", "gz::sim::systems::Imu"),
        ("gz-sim-user-commands-system", "gz::sim::systems::UserCommands"),
        ("gz-sim-scene-broadcaster-system", "gz::sim::systems::SceneBroadcaster"),
    ):
        ET.SubElement(world, "plugin", {"filename": filename, "name": plugin_name})

    buoyancy = ET.SubElement(world, "plugin", {
        "filename": f"plugins/libEER_Buoyancy.{plugin_suffix}",
        "name": "gz::sim::systems::EER_Buoyancy",
    })

    text(buoyancy, "uniform_fluid_density", 1000)

    volume = ET.SubElement(buoyancy, "set_volume")

    text(volume, "entity", model_name + "::rov_body")
    text(volume, "volume", 0.012)
    text(volume, "center_of_volume", "0 0 0.02")

    model = ET.SubElement(world, "include")

    text(model, "uri", "model://" + model_name)
    text(model, "pose", "0 0 -1 0 0 0")

    ET.indent(root, space="  ")
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--connector-lib", type=Path, required=True)
    parser.add_argument("--plugin-suffix", choices=("dylib", "so"),
                        default="dylib" if sys.platform == "darwin" else "so",
                        help="Gazebo plugin library extension (default: host platform)")
    
    args = parser.parse_args()
    output = args.output.resolve()
    connector_lib = args.connector_lib.resolve()

    if not connector_lib.is_file():
        parser.error(f"Connector library does not exist: {connector_lib}")

    if connector_lib.suffix != f".{args.plugin_suffix}":
        parser.error(f"Connector library must end in .{args.plugin_suffix}: {connector_lib}")

    if output.exists():
        parser.error(f"Output already exists: {output}")

    (output / "models").mkdir(parents=True)
    (output / "worlds").mkdir()
    (output / "params").mkdir()

    for frame, spec in FRAMES.items():
        model_name = write_model(output / "models" / (frame.replace("-", "_") + "_test"), frame, spec,
                                 connector_lib, args.plugin_suffix)
        
        write_world(output / "worlds" / f"{frame}.sdf", model_name, args.plugin_suffix)
        (output / "params" / f"{frame}.parm").write_text(f"FRAME_CONFIG {spec['frame_config']}\n")

        print(f"Generated {frame}: {len(spec['factors'])} motors, FRAME_CONFIG={spec['frame_config']}")

if __name__ == "__main__":
    main()