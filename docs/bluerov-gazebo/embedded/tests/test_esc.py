import sys
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path

EMBEDDED = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EMBEDDED))

from esc.config import apply_esc, pwm_to_motor  # noqa: E402

class ESCTests(unittest.TestCase):
    def model(self):
        root = ET.Element("sdf")
        model = ET.SubElement(root, "model")
        connector = ET.SubElement(model, "plugin", {"name": "ArduPilotPlugin"})
        
        for index in range(6):
            control = ET.SubElement(connector, "control", {"channel": str(index)})
            for tag, value in (("cmd_topic", f"/bluerov2_test/motor_{index + 1}"),
                               ("multiplier", "255"), ("offset", "0"),
                               ("servo_min", "1100"), ("servo_max", "1900")):
                
                ET.SubElement(control, tag).text = value

        return ET.ElementTree(root)

    def test_pwm_mapping_and_direction(self):
        self.assertEqual([pwm_to_motor(pwm) for pwm in (1100, 1340, 1400, 1500, 1600, 1900)],
                         [0, 77, 96, 128, 159, 255])
                         
        self.assertLess(pwm_to_motor(1400), pwm_to_motor(1500))
        
        self.assertGreater(pwm_to_motor(1600), pwm_to_motor(1500))

    def test_legacy_model_is_unchanged(self):
        tree = self.model()
        before = ET.tostring(tree.getroot())

        apply_esc(tree, "bluerov2_test", 6, "legacy")

        self.assertEqual(ET.tostring(tree.getroot()), before)

    def test_pass_through_wires_all_six_channels(self):
        tree = self.model()

        apply_esc(tree, "bluerov2_test", 6, "pass_through", plugin=Path(__file__),
                  connector_plugin=Path(__file__))
        
        connector = tree.getroot().find("model/plugin[@name='ArduPilotPlugin']")
        
        for index, control in enumerate(connector.findall("control"), 1):
            self.assertEqual(control.findtext("cmd_topic"), f"/bluerov2_test/motor_{index}/pwm_us")
            self.assertEqual(control.findtext("multiplier"), "800")
            self.assertEqual(control.findtext("offset"), "1.375")

        esc = tree.getroot().find("model/plugin[@name='mate::VirtualESC']")
        
        self.assertEqual(esc.findtext("min_pwm"), "1100")
        self.assertEqual(esc.findtext("max_pwm"), "1900")
        self.assertEqual(esc.findtext("deadband_us"), "0")

    def test_prior_year_limit_and_fault(self):
        tree = self.model()

        apply_esc(tree, "bluerov2_test", 6, "prior_year", stuck_neutral_motor=2, plugin=Path(__file__),
                  connector_plugin=Path(__file__))
        
        esc = tree.getroot().find("model/plugin[@name='mate::VirtualESC']")
        
        self.assertEqual(esc.findtext("min_pwm"), "1160")
        self.assertEqual(esc.findtext("max_pwm"), "1840")
        self.assertEqual(esc.findtext("deadband_us"), "25")
        self.assertEqual(esc.findtext("stuck_neutral_motor"), "2")

    def test_wrong_channel_is_rejected(self):
        tree = self.model()

        tree.getroot().find("model/plugin/control").set("channel", "1")

        with self.assertRaisesRegex(ValueError, "contiguous"):
            apply_esc(tree, "bluerov2_test", 6, "pass_through", plugin=Path(__file__),
                      connector_plugin=Path(__file__))


if __name__ == "__main__":
    unittest.main()