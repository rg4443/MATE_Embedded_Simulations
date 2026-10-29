import argparse
import json
import signal
import time

from pymavlink import mavutil

def emit(event, **fields):
    print(json.dumps({"event": event, "host_monotonic_ns": time.monotonic_ns(), **fields}), flush=True)

def serve(port):
    link = mavutil.mavlink_connection(f"udpin:127.0.0.1:{port}", source_system=254,
                                      source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)
    state = {"running": True, "relay_high": True, "y_was_pressed": False}

    emit("relay_state", relay_high=True, reason="safe_initialization")

    def stop(_signum, _frame): 
        state["running"] = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    last_message = time.monotonic()
    last_heartbeat = 0
    timed_out = False

    try:
        while state["running"]:
            message = link.recv_match(blocking=True, timeout=0.25)

            if message is None:
                if not timed_out and time.monotonic() - last_message > 10:
                    emit("telemetry_timeout")
                    timed_out = True

                continue

            last_message = time.monotonic()
            timed_out = False

            if time.monotonic() - last_heartbeat >= 1:
                link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                
                last_heartbeat = time.monotonic()

                emit("router_registration")

            if message.get_type() != "MANUAL_CONTROL": continue

            pressed = bool(message.buttons & (1 << 3))

            if pressed and not state["y_was_pressed"]:
                state["relay_high"] = not state["relay_high"]
                emit("relay_state", relay_high=state["relay_high"], reason="y_rising_edge")
            
            state["y_was_pressed"] = pressed
    
    finally:
        state["relay_high"] = True

        emit("relay_state", relay_high=True, reason="safe_cleanup")

        link.close()

def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--port", type=int, required=True)

    args = parser.parse_args()

    serve(args.port)

if __name__ == "__main__":
    main()