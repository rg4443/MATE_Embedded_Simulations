import copy
import json
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

EMBEDDED = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EMBEDDED))

from validate_config import ConfigError, generate, validate  # noqa: E402



class ConfigTests(unittest.TestCase):
    def setUp(self):
        path = EMBEDDED / "configs" / "compact_light_bluerov2.json"
        self.config = json.loads(path.read_text())

    def test_supported_config_has_expected_motor_count(self):
        resolved = validate(self.config)

        self.assertEqual(len(resolved["plant"]["motor_positions_m"]), 6)
        self.assertEqual(resolved["plant"]["wrench_rank"], 5)

    def test_light_model_uses_native_added_mass(self):
        self.config["plant"]["mass_kg"] = 5.0
        resolved = validate(self.config)

        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "model"

            generate.write_model(output, "bluerov2", generate.FRAMES["bluerov2"],
                                 Path(tmp) / "connector.dylib", "dylib", mass_kg=resolved["plant"]["mass_kg"])

            root = ET.parse(output / "model.sdf").getroot()
            model = root.find("model")
            plugin = model.find("plugin[@name='gz::sim::systems::EER_Hydrodynamics']")

            self.assertEqual(root.get("version"), "1.11")
            self.assertEqual(model.findtext("link/inertial/fluid_added_mass/zz"), "6.64814797")
            self.assertIsNone(plugin.find("zDotW"))
            self.assertEqual(plugin.findtext("disable_added_mass"), "true")
            self.assertEqual(plugin.findtext("disable_coriolis"), "true")

    def test_legacy_added_mass_is_migrated_once(self):
        root = ET.fromstring(
            '<sdf version="1.7"><model><link name="rov_body"><inertial/></link>'
            '<plugin name="gz::sim::systems::EER_Hydrodynamics"><link_name>rov_body</link_name>'
            '<zDotW>7.5</zDotW></plugin></model></sdf>')

        generate.use_native_added_mass(root)
        generate.use_native_added_mass(root)

        self.assertEqual(root.findtext("model/link/inertial/fluid_added_mass/zz"), "7.5")
        self.assertEqual(len(root.findall("model/link/inertial/fluid_added_mass")), 1)
        self.assertIsNone(root.find("model/plugin/zDotW"))

    def test_direct_generator_rejects_nonpositive_body_mass(self):
        with tempfile.TemporaryDirectory() as tmp:
            output = Path(tmp) / "model"

            with self.assertRaisesRegex(ValueError, "combined T200 thruster mass"):
                generate.write_model(output, "bluerov2", generate.FRAMES["bluerov2"],
                                     Path(tmp) / "connector.dylib", "dylib", mass_kg=2.0)

            self.assertFalse(output.exists())

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