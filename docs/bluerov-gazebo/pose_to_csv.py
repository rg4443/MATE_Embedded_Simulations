import argparse
import csv
import json
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="Gazebo pose JSON Lines recording")
    parser.add_argument("--model", required=True, help="Gazebo model name, such as bluerov2_test")
    parser.add_argument("--output", type=Path, help="CSV path (default: input path with .csv suffix)")
    args = parser.parse_args()

    if not args.input.is_file():
        parser.error(f"Recording not found: {args.input}")

    output = args.output or args.input.with_suffix(".csv")
    rows = []

    with args.input.open(encoding="utf-8") as source:
        for line_number, line in enumerate(source, start=1):
            if not line.strip():
                continue

            try:
                message = json.loads(line)
            except json.JSONDecodeError as exc:
                parser.error(f"Invalid Gazebo JSON on line {line_number}: {exc}")

            model_pose = next(
                (pose for pose in message.get("pose", []) if pose.get("name") == args.model),
                None,
            )

            if model_pose is None:
                continue

            stamp = message.get("header", {}).get("stamp", {})
            position = model_pose.get("position", {})
            orientation = model_pose.get("orientation", {})

            rows.append((
                int(stamp.get("sec", 0)) + int(stamp.get("nsec", 0)) / 1_000_000_000,
                position.get("x", 0), position.get("y", 0), position.get("z", 0),
                orientation.get("x", 0), orientation.get("y", 0),
                orientation.get("z", 0), orientation.get("w", 1),
            ))

    if not rows:
        parser.error(f"No poses for {args.model!r} found in {args.input}; check the model name and recording")

    with output.open("w", newline="", encoding="utf-8") as target:
        writer = csv.writer(target)
        writer.writerow(("sim_time_s", "x_m", "y_m", "z_m", "qx", "qy", "qz", "qw"))
        writer.writerows(rows)

    print(f"Wrote {len(rows)} poses for {args.model} to {output}")

if __name__ == "__main__":
    main()