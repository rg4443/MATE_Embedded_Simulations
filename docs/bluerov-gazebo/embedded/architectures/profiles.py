PROFILES = {
    "pi_pixhawk": {
        "control_host": "separate Pixhawk logical host",
        "mavlink_link": "PTY serial bytes via controllable TCP bridge (F1 USB surrogate)",
        "sensor_io": "Gazebo JSON direct to ArduSub SITL",
        "pi_domain": ("router", "claw", "serial_link"),
        "autopilot_domain": ("sitl",),
        "shared_5v_domain": ("router", "claw", "serial_link", "sitl"),
    },
    "pi_navigator": {
        "control_host": "Pi logical host; ArduSub and router share failure domain",
        "mavlink_link": "local TCP between BlueOS router and ArduSub SITL (F1 host surrogate)",
        "sensor_io": "bidirectional UDP relay for Navigator sensor/PWM boundary (F1)",
        "pi_domain": ("router", "claw", "sitl", "navigator_io"),
        "autopilot_domain": ("sitl",),
        "shared_5v_domain": ("router", "claw", "sitl", "navigator_io"),
    },
}
