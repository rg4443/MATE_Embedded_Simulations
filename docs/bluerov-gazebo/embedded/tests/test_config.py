import copy
import json
import sys
import unittest
from pathlib import Path

EMBEDDED = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EMBEDDED))

from validate_config import ConfigError, validate  # noqa: E402


class ConfigTests(unittest.TestCase):
    def setUp(self):
        path = EMBEDDED / "configs" / "compact_light_bluerov2.json"
        self.config = json.loads(path.read_text())

    def test_supported_config_has_expected_motor_count(self):
        resolved = validate(self.config)

        self.assertEqual(len(resolved["plant"]["motor_positions_m"]), 6)
        self.assertEqual(resolved["plant"]["wrench_rank"], 5)

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