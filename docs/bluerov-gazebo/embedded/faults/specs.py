FAULTS = {
    "pi_pixhawk": {
        "pi_failure": ("router", "claw", "serial_link"),
        "usb_loss": ("serial_link",),
        "pixhawk_restart": ("sitl",),
        "shared_5v_loss": ("router", "claw", "serial_link", "sitl"),
        "esc_stuck_neutral": ("esc_motor_1",),
    },

    "pi_navigator": {
        "router_failure": ("router",),
        "ardusub_failure": ("sitl",),
        "navigator_io_failure": ("navigator_io",),
        "shared_5v_loss": ("router", "claw", "sitl", "navigator_io"),
        "esc_stuck_neutral": ("esc_motor_1",),
    },
}


def targets(profile, fault):
    if fault == "none": return ()

    try:
        return FAULTS[profile][fault]
    
    except KeyError as exc:
        raise ValueError(f"fault {fault!r} is not supported by {profile!r}") from exc