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

    def check_trace(self, fault, target_roles, records):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            (directory / "trace.jsonl").write_text("\n".join(json.dumps(row) for row in records) + "\n")
            summary = {"status": "passed", "ports": {"sysid": 2},
                       "fault": {"name": fault, "targets": target_roles,
                                 "actual_onset_monotonic_ns": 1_000_000_000,
                                 "recovery_command_monotonic_ns": 3_000_000_000}}
            verify_fault_effect(directory, summary)
            return summary

    def event(self, ms, name, **fields):
        return dict(event=name, host_monotonic_ns=ms * 1_000_000, **fields)

    def telemetry(self, ms, sysid=2):
        return [self.event(ms, name, source_sysid=sysid, source_compid=1)
                for name in ("topside_heartbeat", "autopilot_servo_output")]

    def motor(self, ms, pwm, command, valid):
        return self.event(ms, "esc_sample_1", input_pwm_us=pwm, motor_command=command, output_valid=valid)

    def baseline(self):
        return [*self.telemetry(50), self.event(100, "topside_command_sent", command_id="forward"),
                self.motor(200, 1400, 96, True)]

    def test_router_failure_requires_vehicle_telemetry_after_restart(self):
        router = {"router": {"alive": True}}
        records = [*self.baseline(), self.event(1100, "process_stopped", role="router"),
                   self.event(1200, "fault_applied", after={}),
                   self.event(3100, "process_started", role="router"),
                   self.event(3200, "fault_recovery_command", after=router)]
        
        for recovery in ([], self.telemetry(3300, sysid=3), self.telemetry(3100)):
            with self.subTest(recovery=recovery):
                summary = self.check_trace("router_failure", ("router",), records + recovery)
                self.assertEqual(summary["status"], "failed")
                self.assertFalse(summary["fault"]["effect_checks"]["recovery_telemetry"])

        summary = self.check_trace("router_failure", ("router",), records + self.telemetry(3300))
        self.assertEqual(summary["status"], "passed")

        summary = self.check_trace("router_failure", ("router",),
                                   records + self.telemetry(2000) + self.telemetry(3300))
        self.assertEqual(summary["status"], "failed")
        self.assertFalse(summary["fault"]["effect_checks"]["telemetry_interrupted"])

    def test_esc_fault_requires_sustained_neutral_output_and_recovery(self):
        records = [*self.baseline(), self.event(1200, "fault_applied", after={}),
                   self.event(3200, "fault_recovery_command", after={}), *self.telemetry(3300)]
        
        samples = [self.motor(ms, 1400, 128, True) for ms in range(1400, 3000, 50)]
        recovered = self.motor(3400, 1400, 96, True)

        for name, incomplete in (
            ("single sample", samples[:1]),
            ("missing start", samples[3:]),
            ("missing middle", samples[:8] + samples[12:]),
            ("missing end", samples[:-3]),
        ):
            with self.subTest(name=name):
                summary = self.check_trace("esc_stuck_neutral", ("esc_motor_1",),
                                           records + incomplete + [recovered])
                
                self.assertEqual(summary["status"], "failed")
                self.assertFalse(summary["fault"]["effect_checks"]["esc_fault_sample_coverage"])

        for name, sample in (
            ("thrust resumed", self.motor(2000, 1400, 96, True)),
            ("input went neutral", self.motor(2000, 1500, 128, True)),
            ("signal became invalid", self.motor(2000, 1400, 128, False)),
        ):
            with self.subTest(name=name):
                changed = [sample if row["host_monotonic_ns"] == sample["host_monotonic_ns"] else row
                           for row in samples]
                
                summary = self.check_trace("esc_stuck_neutral", ("esc_motor_1",),
                                           records + changed + [recovered])
                
                self.assertEqual(summary["status"], "failed")
                self.assertTrue(summary["fault"]["effect_checks"]["esc_fault_sample_coverage"])
                self.assertFalse(summary["fault"]["effect_checks"]["esc_neutral_with_valid_source"])

        summary = self.check_trace("esc_stuck_neutral", ("esc_motor_1",), records + samples)
        self.assertEqual(summary["status"], "failed")
        self.assertFalse(summary["fault"]["effect_checks"]["esc_motor_response_restored"])

        settling = self.motor(1300, 1400, 96, True)
        summary = self.check_trace("esc_stuck_neutral", ("esc_motor_1",),
                                   records + [settling, *samples, recovered])
        self.assertEqual(summary["status"], "passed")

if __name__ == "__main__":
    unittest.main()