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

    source = replace_once(
        source,
        '  TServoPacket &_pkt\n',
        '  TServoPacket &_pkt,\n'
        '  std::array<uint16_t, 32> &discardedPwm\n',
    )
    source = replace_once(
        source,
        '        counter++;\n        _pkt = last_pkt;',
        '        constexpr uint16_t expectedMagic = sizeof(_pkt.pwm) == 64 ? 29569 : 18458;\n'
        '        if (recvSize == sizeof(TServoPacket) && _pkt.magic == expectedMagic)\n'
        '        {\n'
        '            for (size_t i = 0; i < sizeof(_pkt.pwm) / sizeof(_pkt.pwm[0]); ++i)\n'
        '            {\n'
        '                if (_pkt.pwm[i] < 1100 || _pkt.pwm[i] > 1900)\n'
        '                {\n'
        '                    discardedPwm[i] = _pkt.pwm[i];\n'
        '                }\n'
        '            }\n'
        '        }\n'
        '        counter++;\n'
        '        _pkt = last_pkt;',
    )
    source = replace_once(
        source,
        '    std::array<uint16_t, 32> pkt_pwm;',
        '    std::array<uint16_t, 32> pkt_pwm{};\n'
        '    std::array<uint16_t, 32> discardedPwm;\n'
        '    discardedPwm.fill(1500);',
    )
    for bits in (32, 16):
        source = replace_once(source, f'      servo_packet_{bits} pkt;\n', f'      servo_packet_{bits} pkt{{}};\n')

    packet_read = '          pkt);\n      pkt_magic = pkt.magic;'

    if source.count(packet_read) != 2: raise ValueError("expected two connector servo packet readers")

    source = source.replace(
        packet_read,
        '          pkt, discardedPwm);\n'
        '      if (recvSize != -1 && recvSize != sizeof(pkt))\n'
        '      {\n'
        '          return false;\n'
        '      }\n'
        '      pkt_magic = pkt.magic;',
    )
    source = replace_once(
        source,
        '    this->UpdateMotorCommands(pkt_pwm);',
        '    for (auto &control : this->dataPtr->controls)\n'
        '    {\n'
        '        if (control.type != "COMMAND" || control.channel >= discardedPwm.size())\n'
        '        {\n'
        '            continue;\n'
        '        }\n'
        '        const auto pwm = discardedPwm[control.channel];\n'
        '        if (pwm != 1500)\n'
        '        {\n'
        '            msgs::Int32 loss;\n'
        '            loss.set_data(pwm);\n'
        '            control.pub.Publish(loss);\n'
        '        }\n'
        '    }\n'
        '\n'
        '    this->UpdateMotorCommands(pkt_pwm);',
    )

    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(source)

    return OUTPUT

if __name__ == "__main__":
    print(prepare())