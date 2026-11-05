import copy
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

EMBEDDED = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EMBEDDED))

from validate_config import ConfigError, generate, validate  # noqa: E402

from launch import launch as launch_probe  # noqa: E402
from step5 import launch as launch_architecture  # noqa: E402


class ConfigTests(unittest.TestCase):
    def setUp(self):
        path = EMBEDDED / "configs" / "compact_light_bluerov2.json"
        self.config = json.loads(path.read_text())

    def test_supported_config_has_expected_motor_count(self):
        resolved = validate(self.config)

        self.assertEqual(len(resolved["plant"]["motor_positions_m"]), 6)
        self.assertEqual(resolved["plant"]["wrench_rank"], 5)

    def test_heave_mass_boundary(self):
        limit = generate.HEAVE_ADDED_MASS_KG

        for mass in (5.0, limit):
            with self.subTest(mass=mass):
                self.config["plant"]["mass_kg"] = mass
                with self.assertRaisesRegex(ConfigError, "heave added mass"):
                    validate(self.config)

        mass = math.nextafter(limit, math.inf)
        self.config["plant"]["mass_kg"] = mass
        self.assertEqual(validate(self.config)["plant"]["mass_kg"], mass)

    def test_direct_generator_rejects_light_model_before_writing(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "model"
            with self.assertRaisesRegex(ValueError, "heave added mass"):
                generate.write_model(output, "bluerov2", generate.FRAMES["bluerov2"],
                                     Path(tmp) / "connector.dylib", "dylib", mass_kg=5.0)
            
            self.assertFalse(output.exists())

    def test_launchers_reject_previously_prepared_light_run(self):
        resolved = validate(self.config)
        resolved["plant"]["mass_kg"] = 5.0

        with tempfile.TemporaryDirectory() as tmp:
            run_dir = Path(tmp)

            (run_dir / "manifest.json").write_text(json.dumps({"status": "prepared_not_executed"}))
            (run_dir / "resolved.json").write_text(json.dumps(resolved))
            
            for launch in (launch_probe, launch_architecture):
                with self.subTest(launcher=launch.__module__):
                    with self.assertRaisesRegex(ConfigError, "heave added mass"):
                        launch(run_dir)
                        
            self.assertFalse((run_dir / "sessions").exists())

    def test_swapped_motor_channel_is_rejected(self):
        invalid = copy.deepcopy(self.config)

        invalid["plant"]["motor_channels"][0:2] = [1, 0]

        with self.assertRaisesRegex(ConfigError, "motor_channels"):
            validate(invalid)

    def test_missing_thruster_is_rejected(self):
        invalid = copy.deepcopy(self.config)

        invalid["plant"]["motor_positions_m"].pop()

        with self.assertRaisesRegex(ConfigError, "motor_positions_m"):
            validate(invalid)

    def test_reversed_thruster_geometry_is_rejected(self):
        invalid = copy.deepcopy(self.config)

        for position in invalid["plant"]["motor_positions_m"]:
            position[0] = -position[0]
            position[1] = -position[1]

        with self.assertRaisesRegex(ConfigError, "axis"):
            validate(invalid)

if __name__ == "__main__":
    unittest.main()