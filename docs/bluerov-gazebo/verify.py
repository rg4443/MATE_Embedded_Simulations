import argparse
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path

import generate


SOURCE_CASES = {
    "bluerov1": "SUB_FRAME_BLUEROV1",
    "bluerov2": "SUB_FRAME_VECTORED",
    "bluerov2-heavy": "SUB_FRAME_VECTORED_6DOF",
}

def source_factors(source, case):
    section = source.split(f"case {case}:", 1)[1].split("break;", 1)[0]
    rows = []

    for match in re.finditer(r"add_motor_raw_6dof\(AP_MOTORS_MOT_(\d+),([^;]+)\);", section):
        values = [entry.strip().rstrip("f") for entry in match.group(2).split(",")]
        rows.append((int(match.group(1)), tuple(float(entry) for entry in values[:6])))

    return [factors for _, factors in sorted(rows)]

def matrix_rank(rows, threshold=1e-8):
    matrix = [list(row) for row in rows]
    rank = 0

    for column in range(len(matrix[0])):
        pivot = next((row for row in range(rank, len(matrix)) if abs(matrix[row][column]) > threshold), None)
        
        if pivot is None:
            continue

        matrix[rank], matrix[pivot] = matrix[pivot], matrix[rank]

        value = matrix[rank][column]

        matrix[rank] = [entry / value for entry in matrix[rank]]

        for row in range(len(matrix)):
            if row != rank:
                factor = matrix[row][column]
                matrix[row] = [a - factor * b for a, b in zip(matrix[row], matrix[rank])]

        rank += 1

        if rank == len(matrix):
            break

    return rank

def verify_frame(root, frame, spec):
    model_name = frame.replace("-", "_") + "_test"
    model_path = root / "models" / model_name / "model.sdf"
    model = ET.parse(model_path).getroot().find("model")
    controls = model.findall("./plugin[@name='ArduPilotPlugin']/control")
    thrusters = model.findall("./plugin[@name='gz::sim::systems::EER_Thruster']")

    assert len(controls) == len(thrusters) == len(spec["factors"])
    assert model.find("./plugin[@name='gz::sim::systems::EER_Hydrodynamics']") is not None

    wrench = []

    for index, (position, factors) in enumerate(zip(spec["positions"], spec["factors"]), 1):
        assert controls[index - 1].get("channel") == str(index - 1)
        assert controls[index - 1].findtext("cmd_topic") == f"/{model_name}/motor_{index}"
        assert thrusters[index - 1].findtext("topic") == f"motor_{index}"
        
        link = model.find(f"./link[@name='motor_{index}']")
        visual = link.find("visual[@name='housing']/geometry/cylinder")
        assert abs(float(link.findtext("inertial/mass")) - generate.T200_MASS_KG) < 1e-8
        assert abs(float(visual.findtext("radius")) - generate.T200_RADIUS_M) < 1e-8
        assert abs(float(visual.findtext("length")) - generate.T200_LENGTH_M) < 1e-8
        propeller_diameter = float(thrusters[index - 1].findtext("propeller_diameter"))
        assert abs(propeller_diameter - generate.T200_PROPELLER_DIAMETER_M) < 1e-8
        x, y, z, roll, pitch, yaw = map(float, link.findtext("pose").split())
        actual_axis = (
            math.cos(yaw) * math.sin(pitch) * math.cos(roll) + math.sin(yaw) * math.sin(roll),
            math.sin(yaw) * math.sin(pitch) * math.cos(roll) - math.cos(yaw) * math.sin(roll),
            math.cos(pitch) * math.cos(roll),
        )
        expected_axis = generate.motor_axis(factors)

        assert max(abs(a - b) for a, b in zip(actual_axis, expected_axis)) < 1e-6
        assert max(abs(a - b) for a, b in zip((x, y, z), (position[0], -position[1], -position[2]))) < 1e-6

        fx, fy, fz = factors[4], factors[5], -factors[3]
        magnitude = math.sqrt(fx * fx + fy * fy + fz * fz)
        fx, fy, fz = fx / magnitude, fy / magnitude, fz / magnitude
        px, py, pz = position

        wrench.append((py * fz - pz * fy, pz * fx - px * fz, px * fy - py * fx, fz, fx, fy))

    rank = matrix_rank([tuple(motor[column] for motor in wrench) for column in range(6)])
    expected_rank = 5 if frame == "bluerov2" else 6

    assert rank == expected_rank, (frame, rank, expected_rank)
    for axis in range(6):
        factor_column = [row[axis] for row in spec["factors"]]
        response = sum(wrench[motor][axis] * factor_column[motor] for motor in range(len(wrench)))

        if any(factor_column):
            if axis == 3:
                assert response < 0, (frame, axis, response)
            else:
                assert response > 0, (frame, axis, response)

    return rank


def verify_selected(root, plant):
    """Check one generated model and world against its resolved configuration."""
    frame = plant["frame"]
    spec = dict(generate.FRAMES[frame], positions=plant["motor_positions_m"])
    rank = verify_frame(root, frame, spec)

    if rank != plant["wrench_rank"]:
        raise ValueError(f"generated wrench rank {rank} differs from configuration")

    model_name = frame.replace("-", "_") + "_test"
    model = ET.parse(root / "models" / model_name / "model.sdf").getroot().find("model")
    body = model.find("link[@name='rov_body']")
    actual_body_m = tuple(map(float, body.findtext("visual/geometry/box/size").split()))
    actual_mass = sum(float(link.findtext("inertial/mass")) for link in model.findall("link"))
    
    assert max(abs(a - b) for a, b in zip(actual_body_m, plant["body_m"])) < 1e-8
    assert abs(actual_mass - plant["mass_kg"]) < 1e-8

    world = ET.parse(root / "worlds" / f"{frame}.sdf").getroot().find("world")
    buoyancy = world.find("plugin[@name='gz::sim::systems::EER_Buoyancy']/set_volume")
    
    assert abs(float(buoyancy.findtext("volume")) - plant["displaced_volume_m3"]) < 1e-8
    
    cob = tuple(map(float, buoyancy.findtext("center_of_volume").split()))

    assert abs(cob[2] - plant["center_of_buoyancy_z_m"]) < 1e-8
    assert cob[0] == cob[1] == 0

    params = (root / "params" / f"{frame}.parm").read_text()

    assert params == f"FRAME_CONFIG {plant['frame_config']}\n"

    return rank

def main():
    parser = argparse.ArgumentParser(description=__doc__)
    
    parser.add_argument("--generated", type=Path, required=True)
    parser.add_argument("--ardupilot", type=Path, required=True)

    args = parser.parse_args()
    source = (args.ardupilot / "libraries/AP_Motors/AP_Motors6DOF.cpp").read_text()

    for frame, spec in generate.FRAMES.items():
        actual = source_factors(source, SOURCE_CASES[frame])

        assert actual == [tuple(float(value) for value in row) for row in spec["factors"]], frame

        rank = verify_frame(args.generated, frame, spec)

        print(f"{frame}: {len(actual)} motor rows match ArduSub; Gazebo axes valid; wrench rank {rank}")

if __name__ == "__main__":
    main()