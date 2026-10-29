# AP_FLAKE8_CLEAN
"""Write trace events against one host monotonic clock."""

import json
import threading
import time
from datetime import datetime, timezone


class Trace:
    def __init__(self, path, run_id, session_id):
        self.file = path.open("w")
        self.lock = threading.Lock()
        self.run_id = run_id
        self.session_id = session_id

    def emit(self, event, **fields):
        record = {
            "run_id": self.run_id,
            "session_id": self.session_id,
            "event": event,
            "host_monotonic_ns": time.monotonic_ns(),
            "wall_utc": datetime.now(timezone.utc).isoformat(),
            **fields,
        }
        with self.lock:
            self.file.write(json.dumps(record, separators=(",", ":")) + "\n")
            self.file.flush()

    def close(self):
        with self.lock:
            self.file.close()
