# AP_FLAKE8_CLEAN
"""Documented SITL port derivation and collision checks."""

import fcntl
import socket
from pathlib import Path


def ports_for_instance(instance):
    if not 2 <= instance <= 9:
        raise ValueError("embedded sessions use SITL instances 2 through 9")
    return {
        "sitl_instance": instance,
        "sysid": instance + 1,
        "gazebo_json_udp": 9002 + 10 * instance,
        "mavlink_tcp": 5760 + 10 * instance,
        "sitl_serial1_tcp": 5762 + 10 * instance,
        "sitl_serial2_tcp": 5763 + 10 * instance,
        "topside_udp": 14600 + instance,
        "claw_udp": 14700 + instance,
        "serial_bridge_tcp": 16000 + instance,
        "navigator_connector_udp": 10002 + 10 * instance,
    }


def port_is_free(port, kind):
    family = socket.SOCK_STREAM if kind == "tcp" else socket.SOCK_DGRAM
    with socket.socket(socket.AF_INET, family) as sock:
        try:
            sock.bind(("127.0.0.1", port))
        except OSError:
            return False
    return True


def reserve_instance():
    """Hold a per-instance advisory lock until the caller closes the file."""
    for instance in range(2, 10):
        lock_path = Path("/private/tmp") / f"mate-embedded-instance-{instance}.lock"
        lock = lock_path.open("w")
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            lock.close()
            continue
        ports = ports_for_instance(instance)
        tcp = ("mavlink_tcp", "sitl_serial1_tcp", "sitl_serial2_tcp", "serial_bridge_tcp")
        udp = ("gazebo_json_udp", "topside_udp", "claw_udp", "navigator_connector_udp")
        if all(port_is_free(ports[key], "tcp") for key in tcp) and all(
                port_is_free(ports[key], "udp") for key in udp):
            return lock, ports
        lock.close()
    raise RuntimeError("no free embedded SITL instance from 2 through 9")
