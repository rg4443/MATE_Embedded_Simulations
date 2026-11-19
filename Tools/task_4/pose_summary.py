import argparse
import json
import math
from pathlib import Path


def samples_from(path, model):
    samples = []

    for line in path.read_text().splitlines():
        try:
            record = json.loads(line)
        except json.JSONDecodeError:
            continue

        pose = next((item for item in record.get("pose", []) if item.get("name") == model), None)
        if pose is None: continue

        position = pose.get("position", {})
        orientation = pose.get("orientation", {})
        stamp = record.get("header", {}).get("stamp", {})

        x = float(position.get("x", 0))
        y = float(position.get("y", 0))
        z = float(position.get("z", 0))
        qx = float(orientation.get("x", 0))
        qy = float(orientation.get("y", 0))
        qz = float(orientation.get("z", 0))
        qw = float(orientation.get("w", 1))
        time_s = float(stamp.get("sec", 0)) + float(stamp.get("nsec", 0)) / 1e9
        yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))

        samples.append((time_s, x, y, z, yaw))

    return samples


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("path", type=Path)
    parser.add_argument("--model", default="bluerov1_test")
    args = parser.parse_args()

    samples = samples_from(args.path, args.model)
    if len(samples) < 2: parser.error("Need at least two gazebo pose samples for the selected model")

    first = samples[0]
    last = samples[-1]
    dx = last[1] - first[1]
    dy = last[2] - first[2]
    dz = last[3] - first[3]
    distance_xy = math.hypot(dx, dy)
    forward = dx * math.cos(first[4]) + dy * math.sin(first[4])
    yaw_delta = math.degrees(math.atan2(math.sin(last[4] - first[4]), math.cos(last[4] - first[4])))
    forward_excursion = max(abs((x - first[1]) * math.cos(first[4]) +
                                (y - first[2]) * math.sin(first[4])) for _, x, y, _, _ in samples)
    yaw_excursion = max(abs(math.degrees(math.atan2(math.sin(yaw - first[4]),
                                                   math.cos(yaw - first[4])))) for _, _, _, _, yaw in samples)

    print(f"Samples: {len(samples)}")
    print(f"Simulation time: {last[0] - first[0]:.2f} s")
    print(f"World displacement: x={dx:.3f} m y={dy:.3f} m z={dz:.3f} m")
    print(f"Horizontal distance: {distance_xy:.3f} m")
    print(f"Forward displacement: {forward:.3f} m")
    print(f"Largest forward excursion: {forward_excursion:.3f} m")
    print(f"Yaw change: {yaw_delta:.1f} deg")
    print(f"Largest yaw excursion: {yaw_excursion:.1f} deg")


if __name__ == "__main__":
    main()
