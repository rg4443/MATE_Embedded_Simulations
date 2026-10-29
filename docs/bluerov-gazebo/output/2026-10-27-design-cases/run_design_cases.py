import argparse
import csv
import importlib.util
import os
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
OUTPUT = Path(__file__).resolve().parent
BASELINE = ROOT / "docs/bluerov-gazebo/output/2026-10-24-frame-screening/run_trials.py"
RUNTIME = Path("/private/tmp/mate-design-cases-20261027")
SUMMARY_PATH = OUTPUT / "trial_summary.csv"
SAMPLES_PATH = OUTPUT / "gazebo_pose_samples.csv"
CASE_COLUMNS = ("case_id", "body_length_cm", "body_width_cm", "body_height_cm", "outer_length_cm",
                "outer_width_cm", "outer_height_cm", "outer_sum_cm", "mass_kg", "displaced_volume_m3")

spec = importlib.util.spec_from_file_location("baseline_frame_trials", BASELINE)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)


def load_rows(path):
    if not path.exists():
        return []
    with path.open(newline="") as file:
        return list(csv.DictReader(file))

def save_rows(path, columns, rows):
    pending = path.with_suffix(".tmp")

    with pending.open("w", newline="") as file:
        writer = csv.DictWriter(file, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)

    os.replace(pending, path)

def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--case", choices=("compact_light", "compact_mid", "roomy_light"), action="append")
    parser.add_argument("--layout", choices=base.FRAMES, action="append")
    parser.add_argument("--axis", choices=base.AXES, action="append")

    args = parser.parse_args()
    cases = args.case or ("compact_light", "compact_mid", "roomy_light")
    layouts = args.layout or base.FRAMES
    axes = args.axis or base.AXES

    definitions = {(row["case_id"], row["layout"]): row for row in load_rows(OUTPUT / "case_definitions.csv")}
    summary_columns = (*CASE_COLUMNS, *base.SUMMARY_COLUMNS)
    samples_columns = ("case_id", *base.SAMPLE_COLUMNS)
    summaries = load_rows(SUMMARY_PATH)
    samples = load_rows(SAMPLES_PATH)
    completed = {(row["case_id"], row["layout"], row["axis"]) for row in summaries if row["status"] == "measured"}

    for case_id in cases:
        base.PACKAGE = OUTPUT / "cases" / case_id
        base.RUNTIME = RUNTIME / case_id

        for layout in layouts:
            definition = definitions[(case_id, layout)]
            
            for axis in axes:
                if (case_id, layout, axis) in completed:
                    continue
                print(f"Starting {case_id}/{layout}/{axis}", flush=True)

                summary, series = base.run_trial(layout, axis, 1)
                record = {key: definition[key] for key in CASE_COLUMNS}

                record.update(dict(zip(base.SUMMARY_COLUMNS, summary)))

                record["notes"] = "One trial; mass and size case; neutral buoyancy assumption; placeholder drag/thrust"
                
                if record["status"] != "measured" or not record["armed"] or not series:
                    raise RuntimeError(f"Invalid trial: {case_id}/{layout}/{axis}")
                
                summaries.append(record)
                samples.extend({"case_id": case_id, **dict(zip(base.SAMPLE_COLUMNS, row))} for row in series)
                
                save_rows(SUMMARY_PATH, summary_columns, summaries)
                save_rows(SAMPLES_PATH, samples_columns, samples)

                completed.add((case_id, layout, axis))

                print(f"Finished {case_id}/{layout}/{axis}: {record['pose_samples']} pose samples", flush=True)
    
    print(f"Measured {len(completed)} unique case/layout/axis trials", flush=True)

if __name__ == "__main__":
    main()