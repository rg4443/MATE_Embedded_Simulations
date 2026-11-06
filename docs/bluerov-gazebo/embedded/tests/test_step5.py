import json
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

EMBEDDED = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EMBEDDED))

from architectures.profiles import PROFILES  # noqa: E402
from faults.specs import targets  # noqa: E402
from step5 import verify_fault_effect  # noqa: E402

from adapters.trace import Trace
from launch import arm, has_motor_response, read_parameter, wait_vehicle_heartbeat

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

    def check_trace(self, fault, target_roles, records, motor_count=1):
        with tempfile.TemporaryDirectory() as temp:
            directory = Path(temp)
            (directory / "trace.jsonl").write_text("\n".join(json.dumps(row) for row in records) + "\n")
            summary = {"status": "passed", "ports": {"sysid": 2}, "motor_count": motor_count,
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

    def test_motor_loss_requires_sustained_neutral_output_on_every_channel(self):
        role = "navigator_io"
        records = [*self.baseline(), self.event(1100, "process_stopped", role=role),
                   self.event(1200, "fault_applied", after={}),
                   self.event(3100, "process_started", role=role),
                   self.event(3200, "fault_recovery_command", after={role: {"alive": True}}),
                   *self.telemetry(3300), self.motor(3400, 1400, 96, True),
                   self.event(3400, "esc_sample_2", output_valid=True)]
        samples = [self.event(ms, f"esc_sample_{channel}", motor_command=128, output_valid=False)
                   for channel in (1, 2) for ms in range(1900, 3000, 50)]

        summary = self.check_trace("navigator_io_failure", (role,), records + samples, motor_count=2)

        self.assertEqual(summary["status"], "passed")

        missing_channel = [row for row in samples if row["event"] == "esc_sample_1"]
        summary = self.check_trace("navigator_io_failure", (role,), records + missing_channel, motor_count=2)

        self.assertFalse(summary["fault"]["effect_checks"]["esc_fault_sample_coverage"])

        resumed = self.event(2500, "esc_sample_2", motor_command=96, output_valid=True)
        summary = self.check_trace("navigator_io_failure", (role,), records + samples + [resumed], motor_count=2)

        self.assertEqual(summary["status"], "failed")
        self.assertFalse(summary["fault"]["effect_checks"]["esc_output_became_invalid"])

    def test_trace_stays_open_across_events(self):
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "trace.jsonl"
            trace = Trace(path, "run", "session")

            trace.emit("first")
            trace.emit("second")
            trace.close()

            self.assertEqual([json.loads(line)["event"] for line in path.read_text().splitlines()],
                             ["first", "second"])

    def test_motor_response_requires_valid_non_neutral_pwm(self):
        self.assertFalse(has_motor_response([[0, 1500, 2000, 1700]], motor_count=3))
        self.assertTrue(has_motor_response([[0, 1100, 1500]], motor_count=3))

    def test_vehicle_checks_ignore_other_systems_and_components(self):
        link = Mock()
        trace = Mock()

        def message(sysid, compid, value):
            return SimpleNamespace(param_id="FRAME_CONFIG", param_value=value, base_mode=128,
                                   get_srcSystem=lambda: sysid, get_srcComponent=lambda: compid,
                                   get_type=lambda: "HEARTBEAT", get_seq=lambda: sysid)

        expected = message(2, 1, 1)
        link.recv_match.side_effect = [message(99, 1, 2), message(2, 3, 2), expected]

        self.assertIs(wait_vehicle_heartbeat(link, 2, timeout=1), expected)

        link.recv_match.side_effect = [message(99, 1, 2), message(2, 3, 2), message(2, 1, 1)]

        self.assertEqual(read_parameter(link, "FRAME_CONFIG", 2), 1)

        link.recv_match.side_effect = [message(99, 1, 2), message(2, 3, 2), message(2, 1, 1)]

        arm(link, 2, trace)

        trace.emit.assert_called_with("topside_armed_heartbeat", mavlink_seq=2)


if __name__ == "__main__":
    unittest.main()