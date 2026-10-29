import csv
import tarfile
import tempfile
from collections import defaultdict
from pathlib import Path

from pymavlink import mavutil


OUTPUT = Path(__file__).resolve().parent
ARCHIVE = OUTPUT / "trial_logs.tar.gz"
ARCHIVE_ROOT = "mate-design-cases-20261027"

def command_outputs(path, channel, command_pwm, motor_count):
    log = mavutil.mavlink_connection(str(path))
    latest_input = None
    outputs = []

    while message := log.recv_match(type=["RCIN", "RCOU"], blocking=False):
        if message.get_type() == "RCIN":
            latest_input = getattr(message, f"C{channel}")

        elif latest_input == command_pwm:
            outputs.append([getattr(message, f"C{motor}") for motor in range(1, motor_count + 1)])
    
    return outputs

def main():
    with (OUTPUT / "trial_summary.csv").open(newline="") as file:
        trials = list(csv.DictReader(file))

    if len(trials) != 54:
        raise ValueError(f"Expected 54 trials, found {len(trials)}")

    grouped = defaultdict(lambda: defaultdict(list))
    detail = []

    with tarfile.open(ARCHIVE, "r:gz") as archive, tempfile.TemporaryDirectory() as directory:
        path = Path(directory) / "trial.BIN"

        for trial in trials:
            archive_path = (f"{ARCHIVE_ROOT}/{trial['case_id']}/{trial['trial_id']}"
                            "/logs/00000001.BIN")
            
            member = archive.extractfile(archive_path)

            if member is None:
                raise FileNotFoundError(archive_path)
            
            path.write_bytes(member.read())

            count = int(trial["thrusters"])
            outputs = command_outputs(path, int(trial["rcmap_channel"]),
                                      int(trial["command_pwm"]), count)
            
            if len(outputs) < 8:
                raise ValueError(f"Too few command-phase RCOU samples: {trial['trial_id']}: {len(outputs)}")
            
            for motor in range(1, count + 1):
                values = [sample[motor - 1] for sample in outputs]
                grouped[(trial["case_id"], trial["layout"])][motor].extend(values)
                detail.append((trial["case_id"], trial["layout"], trial["axis"], motor,
                               len(values), min(values), max(values)))

    with (OUTPUT / "motor_pwm_by_trial.csv").open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(("case_id", "layout", "axis", "motor", "samples", "min_pwm", "max_pwm"))
        writer.writerows(detail)

    with (OUTPUT / "motor_pwm_envelope.csv").open("w", newline="") as file:
        writer = csv.writer(file)
        writer.writerow(("case_id", "layout", "motor", "samples", "min_pwm", "max_pwm",
                         "max_abs_deviation_from_1500_pwm"))
        for (case, layout), motors in grouped.items():
            for motor, values in sorted(motors.items()):
                writer.writerow((case, layout, motor, len(values), min(values), max(values),
                                 max(abs(value - 1500) for value in values)))
                
    print(f"Extracted {len(detail)} trial/motor rows and {sum(map(len, grouped.values()))} envelopes")

if __name__ == "__main__":
    main()