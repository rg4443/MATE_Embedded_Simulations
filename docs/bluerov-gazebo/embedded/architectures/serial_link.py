import argparse
import json
import os
import select
import signal
import socket
import termios
import tty
from pathlib import Path


def set_raw(fd):
    tty.setraw(fd)

    attributes = termios.tcgetattr(fd)
    attributes[2] |= termios.CLOCAL

    termios.tcsetattr(fd, termios.TCSANOW, attributes)

def serve(port, ready_path):
    master, slave = os.openpty()

    set_raw(master)
    set_raw(slave)

    slave_path = os.ttyname(slave)
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(("0.0.0.0", port))
    server.listen(1)
    server.setblocking(False)
    os.set_blocking(master, False)

    state = {"up": True, "client": None, "running": True}

    def stop(_signum, _frame):
        state["running"] = False

    def down(_signum, _frame):
        state["up"] = False

        if state["client"] is not None:
            state["client"].close()
            state["client"] = None

        print(json.dumps({"event": "link_down"}), flush=True)

    def up(_signum, _frame):
        state["up"] = True
        print(json.dumps({"event": "link_up"}), flush=True)

    signal.signal(signal.SIGUSR1, down)
    signal.signal(signal.SIGUSR2, up)
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    ready_path.write_text(json.dumps({"slave_path": slave_path, "tcp_port": port}) + "\n")
    
    print(json.dumps({"event": "ready", "slave_path": slave_path, "tcp_port": port}), flush=True)
    
    try:
        while state["running"]:
            readable = [master, server]

            if state["client"] is not None:
                readable.append(state["client"])
                
            try:
                ready, _, _ = select.select(readable, [], [], 0.2)
            except (OSError, ValueError):
                continue

            if server in ready:
                client, _ = server.accept()

                client.setblocking(False)

                if state["up"] and state["client"] is None:
                    state["client"] = client
                    print(json.dumps({"event": "client_connected"}), flush=True)
                else:
                    client.close()

            if master in ready:
                try:
                    data = os.read(master, 4096)
                except OSError:
                    data = b""

                if data and state["up"] and state["client"] is not None:
                    try:
                        state["client"].sendall(data)
                    except OSError:
                        state["client"].close()
                        state["client"] = None

            client = state["client"]

            if client is not None and client in ready:
                try:
                    data = client.recv(4096)
                except OSError:
                    data = b""
                    
                if data and state["up"]:
                    try:
                        os.write(master, data)
                    except OSError:
                        pass
                else:
                    client.close()
                    state["client"] = None
                    print(json.dumps({"event": "client_disconnected"}), flush=True)
    
    finally:
        if state["client"] is not None:
            state["client"].close()

        server.close()
        os.close(slave)
        os.close(master)

def main():
    parser = argparse.ArgumentParser(description=__doc__)

    parser.add_argument("--port", type=int, required=True)
    parser.add_argument("--ready-path", type=Path, required=True)

    args = parser.parse_args()

    serve(args.port, args.ready_path)

if __name__ == "__main__":
    main()