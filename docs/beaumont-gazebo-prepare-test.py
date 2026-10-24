#!/usr/bin/env python3
# AP_FLAKE8_CLEAN

"""Create an isolated, experimental Beaumont model for the ArduSub link test."""

import argparse
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path


THRUSTERS = (
    "for_star_top",
    "for_port_top",
    "for_star_bot",
    "for_port_bot",
    "aft_star_top",
    "aft_port_top",
    "aft_star_bot",
    "aft_port_bot",
)


def add_text(parent, tag, value):
    ET.SubElement(parent, tag).text = str(value)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rov-sim", type=Path, required=True)
    parser.add_argument("--connector-lib", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    source = args.rov_sim.resolve()
    connector = args.connector_lib.resolve()
    output = args.output.resolve()
    if not connector.is_file():
        parser.error(f"Connector library does not exist: {connector}")

    output.mkdir(parents=True, exist_ok=False)
    model_dir = output / "models" / "Beaumont"
    shutil.copytree(source / "models" / "Beaumont", model_dir)
    (output / "worlds").mkdir()
    (output / "sitl").mkdir()

    world_path = output / "worlds" / "BeaumontOceanWorld.sdf"
    shutil.copy2(source / "worlds" / "BeaumontOceanWorld.sdf", world_path)
    world = ET.parse(world_path)
    world.find(".//include/uri").text = model_dir.as_uri()
    world.write(world_path, encoding="utf-8", xml_declaration=True)

    model_path = model_dir / "model.sdf"
    model = ET.parse(model_path)
    model_elem = model.getroot().find("model")
    imu = model_elem.find(".//link[@name='rov_body']/sensor[@name='imu']")
    imu.remove(imu.find("topic"))

    plugin = ET.SubElement(model_elem, "plugin", {"name": "ArduPilotPlugin", "filename": str(connector)})
    settings = (
        ("fdm_addr", "127.0.0.1"),
        ("fdm_port_in", "9012"),
        ("connectionTimeoutMaxCount", "5"),
        ("lock_step", "0"),
        ("no_time_sync", "1"),
        ("have_32_channels", "0"),
        ("modelXYZToAirplaneXForwardZDown", "0 0 0 180 0 -90"),
        ("gazeboXYZToNED", "0 0 0 180 0 90"),
        ("imuName", "imu"),
    )
    for tag, value in settings:
        element = ET.SubElement(plugin, tag)
        element.text = value
        if tag in ("modelXYZToAirplaneXForwardZDown", "gazeboXYZToNED"):
            element.set("degrees", "true")

    for channel, name in enumerate(THRUSTERS):
        control = ET.SubElement(plugin, "control", {"channel": str(channel)})
        for tag, value in (
            ("jointName", name + "_propeller_joint"),
            ("type", "COMMAND"),
            ("cmd_topic", "/beaumont/" + name),
            ("multiplier", "255"),
            ("offset", "0"),
            ("servo_min", "1100"),
            ("servo_max", "1900"),
        ):
            add_text(control, tag, value)

    model.write(model_path, encoding="utf-8", xml_declaration=True)
    print(f"Wrote {world_path} and {model_path}")


if __name__ == "__main__":
    main()
