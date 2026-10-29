import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


DIRECTORY = Path(__file__).resolve().parent
ROWS = list(csv.DictReader((DIRECTORY / "trial_summary.csv").open(encoding="utf-8")))
LAYOUTS = ("bluerov1", "bluerov2", "bluerov2-heavy")
LABELS = ("BlueROV1 (6)", "BlueROV2 (6)", "BlueROV2 Heavy (8)")
COLORS = ("#2d6a94", "#009e73", "#d55e00")

def response(layout, axis):
    row = next(item for item in ROWS if item["layout"] == layout and item["axis"] == axis)
    field = {
        "forward": "delta_x_m", "lateral": "delta_y_m", "dive": "delta_z_m",
        "yaw": "delta_yaw_deg", "roll": "delta_roll_deg", "pitch": "delta_pitch_deg",
    }[axis]

    return abs(float(row[field]))

figure, panels = plt.subplots(1, 2, figsize=(11, 4.5), layout="constrained")

for panel, axes, unit in (
    (panels[0], ("forward", "lateral", "dive"), "Distance (m)"),
    (panels[1], ("yaw", "roll", "pitch"), "Angle (degrees)"),
):
    x = np.arange(len(axes))

    for index, (layout, label, color) in enumerate(zip(LAYOUTS, LABELS, COLORS)):
        values = [response(layout, axis) for axis in axes]
        panel.bar(x + (index - 1) * 0.25, values, width=0.23, label=label, color=color)
    
    panel.set_xticks(x, [axis.capitalize() for axis in axes])
    panel.set_ylabel(unit)
    panel.grid(axis="y", alpha=0.25)
    panel.set_axisbelow(True)

panels[0].set_title("Translation along commanded axis.")
panels[1].set_title("Rotation about commanded axis.")
panels[1].legend(loc="upper right", fontsize=8)
figure.suptitle("2 sec manual-mode commands | one run per layout and axis.")
figure.savefig(DIRECTORY / "initial_response.png", dpi=180)