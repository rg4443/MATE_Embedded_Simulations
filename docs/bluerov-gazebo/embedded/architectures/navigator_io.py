import argparse
import json
import select
import signal
import socket
from pathlib import Path

def serve(sitl_port, gazebo_port, ready_path):
    from_sitl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    from_sitl.bind(("127.0.0.1", sitl_port))
    from_gazebo = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    from_gazebo.bind(("127.0.0.1", 0))
    ready_path.write_text(json.dumps({"sitl_port": sitl_port, "gazebo_port": gazebo_port,
                                      "relay_port": from_gazebo.getsockname()[1]}) + "\n")
    
    print(json.dumps({"event": "ready", "sitl_port": sitl_port, "gazebo_port": gazebo_port}), flush=True)
    
    sitl_address = None
    servo_packets = 0
    sensor_packets = 0
    next_report = 1000
    state = {"running": True}

    def stop(_signum, _frame):
        state["running"] = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    try:
        while state["running"]:
            readable, _, _ = select.select([from_sitl, from_gazebo], [], [], 0.5)
            
            if from_sitl in readable:
                data, sitl_address = from_sitl.recvfrom(65535)
                from_gazebo.sendto(data, ("127.0.0.1", gazebo_port))
                servo_packets += 1

            if from_gazebo in readable:
                data, _ = from_gazebo.recvfrom(65535)

                if sitl_address is not None:
                    from_sitl.sendto(data, sitl_address)
                    sensor_packets += 1

            if servo_packets >= next_report:
                print(json.dumps({"event": "packet_counts", "servo": servo_packets,
                                  "sensor": sensor_packets}), flush=True)
                
                next_report += 1000
    finally:
        from_sitl.close()
        from_gazebo.close()

def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--sitl-port", type=int, required=True)
    parser.add_argument("--gazebo-port", type=int, required=True)
    parser.add_argument("--ready-path", type=Path, required=True)

    args = parser.parse_args()

    serve(args.sitl_port, args.gazebo_port, args.ready_path)

if __name__ == "__main__":
    main()