import sys
import unittest
from pathlib import Path

EMBEDDED = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EMBEDDED))

from adapters.ports import ports_for_instance  # noqa: E402

class PortTests(unittest.TestCase):
    def test_instances_have_distinct_endpoints_and_system_ids(self):
        first = ports_for_instance(2)
        second = ports_for_instance(3)

        for name in ("sysid", "gazebo_json_udp", "mavlink_tcp", "topside_udp"): self.assertNotEqual(first[name], second[name])
        
        self.assertEqual(first["gazebo_json_udp"], 9022)
        self.assertEqual(second["mavlink_tcp"], 5790)

    def test_preexisting_instance_one_is_not_allocated(self):
        with self.assertRaises(ValueError):
            ports_for_instance(1)

if __name__ == "__main__":
    unittest.main()