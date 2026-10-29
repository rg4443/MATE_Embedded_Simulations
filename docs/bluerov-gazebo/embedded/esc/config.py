import xml.etree.ElementTree as ET
import math
from pathlib import Path

PLUGIN = Path(__file__).parent / "build" / "libMATE_VirtualESC.dylib"
CONNECTOR = Path(__file__).parent / "build" / "libMATE_ArduPilotPlugin.dylib"

def pwm_to_motor(pwm_us):
    return math.floor(255 * (pwm_us - 1100) / 800 + 0.5)

def apply_esc(tree, model_name, motor_count, mode, timeout_s=0.5, delay_s=0.0,
              stuck_neutral_motor=0, deadband_us=None, drop_input_after_s=-1.0,
              plugin=PLUGIN, connector_plugin=CONNECTOR):
    
    if mode == "legacy": return
    if mode not in ("pass_through", "prior_year"): raise ValueError(f"unknown ESC mode: {mode}")
    if not plugin.is_file(): raise FileNotFoundError(f"build virtual ESC first: {plugin}")
    if not connector_plugin.is_file(): raise FileNotFoundError(f"build adapted connector first: {connector_plugin}")
    if (timeout_s <= 0 or delay_s < 0 or drop_input_after_s < 0 and drop_input_after_s != -1 or
            not 0 <= stuck_neutral_motor <= motor_count):
        
        raise ValueError("invalid ESC timing or fault configuration")
    
    if deadband_us is None: deadband_us = 0 if mode == "pass_through" else 25
    if not 0 <= deadband_us <= 100: raise ValueError("deadband_us must be 0..100")
    
    model = tree.getroot().find("model")
    connector = model.find("plugin[@name='ArduPilotPlugin']")
    connector.set("filename", str(connector_plugin.resolve()))
    controls = connector.findall("control")

    if len(controls) != motor_count: raise ValueError("connector control count does not match motor count")
    
    for index, control in enumerate(controls, 1):
        if int(control.attrib["channel"]) != index - 1: raise ValueError("connector channels are not contiguous")
        if control.findtext("servo_min") != "1100" or control.findtext("servo_max") != "1900": raise ValueError("connector PWM range is not 1100..1900")
        if control.findtext("cmd_topic") != f"/{model_name}/motor_{index}": raise ValueError("connector motor topic mismatch")
        
        control.find("cmd_topic").text += "/pwm_us"
        control.find("multiplier").text = "800"
        control.find("offset").text = "1.375"
        
    esc = ET.SubElement(model, "plugin", {"filename": str(plugin.resolve()), "name": "mate::VirtualESC"})
    settings = {
        "namespace": model_name,
        "motors": motor_count,
        "min_pwm": 1100 if mode == "pass_through" else 1160,
        "max_pwm": 1900 if mode == "pass_through" else 1840,
        "deadband_us": deadband_us,
        "timeout_s": timeout_s,
        "delay_s": delay_s,
        "drop_input_after_s": drop_input_after_s,
        "stuck_neutral_motor": stuck_neutral_motor,
    }

    for key, value in settings.items():
        ET.SubElement(esc, key).text = str(value)