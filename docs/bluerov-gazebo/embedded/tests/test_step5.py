import json
import sys
import tempfile
import unittest
from pathlib import Path

EMBEDDED = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EMBEDDED))

from architectures.profiles import PROFILES  # noqa: E402
from faults.specs import targets  # noqa: E402
from step5 import verify_fault_effect  # noqa: E402

class Step5Contracts(unittest.TestCase):
    def test_pixhawk_pi_failure_keeps_separate_autopilot(self):
        self.assertNotIn("sitl", targets("pi_pixhawk", "pi_failure"))
        self.assertIn("sitl", PROFILES["pi_pixhawk"]["shared_5v_domain"])

    def test_navigator_io_fault_targets_only_io(self):
        self.assertEqual(targets("pi_navigator", "navigator_io_failure"), ("navigator_io",))
        self.assertIn("sitl", PROFILES["pi_navigator"]["pi_domain"])

    def test_shared_rail_owns_pi_and_autopilot_in_both(self):
        for profile in PROFILES:
            self.assertEqual(targets(profile, "shared_5v_loss"),
                             PROFILES[profile]["shared_5v_domain"])

    def test_profile_inapplicable_fault_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "not supported"):
            targets("pi_navigator", "usb_loss")

    def test_esc_fault_cannot_pass_without_a_neutral_motor_response(self):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            records = [{"event": "topside_command_sent", "command_id": "forward", "host_monotonic_ns": 10},
                       {"event": "gazebo_motor_1", "value": 96, "host_monotonic_ns": 20},
                       {"event": "fault_applied", "host_monotonic_ns": 120,
                        "after": {"sitl": {"alive": True}}}]
            
            (directory / "trace.jsonl").write_text("\n".join(json.dumps(row) for row in records) + "\n")
            
            summary = {"status": "passed", "fault": {"name": "esc_stuck_neutral",
                       "targets": ("esc_motor_1",), "actual_onset_monotonic_ns": 100,
                       "recovery_command_monotonic_ns": 200,
                       "observed_during_fault": {
                           "gazebo_motor_1": {"distinct_values": [96]},
                           "esc_raw_pwm_1": {"distinct_values": [1400]},
                           "esc_output_valid_1": {"distinct_values": [True]}}}}
            
            verify_fault_effect(directory, summary)

            self.assertEqual(summary["status"], "failed")
            self.assertFalse(summary["fault"]["effect_checks"]["esc_neutral_with_valid_source"])

if __name__ == "__main__":
    unittest.main()