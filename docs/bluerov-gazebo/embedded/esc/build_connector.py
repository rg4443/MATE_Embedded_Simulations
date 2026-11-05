from pathlib import Path

SOURCE = Path(__file__).resolve().parents[2] / ".local" / "ardupilot_gazebo" / "src" / "ArduPilotPlugin.cc"
OUTPUT = Path(__file__).parent / "build" / "ArduPilotPlugin.cc"

def replace_once(source, before, after):
    if source.count(before) != 1:
        raise ValueError(f"connector source changed; expected one occurrence of {before[:60]!r}")
    
    return source.replace(before, after, 1)

def prepare():
    source = SOURCE.read_text()
    source = replace_once(
        source,
        '    if (this->dataPtr->controls[i].type == "COMMAND")\n'
        '    {\n'
        '      msgs::Int32 cmd;\n'
        '      cmd.set_data(static_cast<int32_t>(std::lround(this->dataPtr->controls[i].cmd)));\n'
        '      this->dataPtr->controls[i].pub.Publish(cmd);\n'
        '      continue;\n'
        '    }',
        '    if (this->dataPtr->controls[i].type == "COMMAND")\n'
        '    {\n'
        '      continue;  // Published once per received servo packet.\n'
        '    }',
    )
    source = replace_once(
        source,
        '                const double pwm = _pwm[this->dataPtr->controls[i].channel];',
        '                const double pwm = _pwm[this->dataPtr->controls[i].channel];\n'
        '                if (this->dataPtr->controls[i].type == "COMMAND")\n'
        '                {\n'
        '                  msgs::Int32 cmd;\n'
        '                  cmd.set_data(static_cast<int32_t>(pwm));\n'
        '                  this->dataPtr->controls[i].pub.Publish(cmd);\n'
        '                  continue;\n'
        '                }',
    )
    source = replace_once(
        source,
        '    // this->dataPtr->controls[i].pid.Reset();',
        '    if (this->dataPtr->controls[i].type == "COMMAND")\n'
        '    {\n'
        '      msgs::Int32 cmd;\n'
        '      cmd.set_data(0);\n'
        '      this->dataPtr->controls[i].pub.Publish(cmd);\n'
        '    }\n'
        '    // this->dataPtr->controls[i].pid.Reset();',
    )

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(source)

    return OUTPUT

if __name__ == "__main__":
    print(prepare())