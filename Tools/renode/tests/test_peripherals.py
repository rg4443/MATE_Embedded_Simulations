# AP_FLAKE8_CLEAN

"""Exercise reset and flash behavior in Renode without vehicle firmware."""

import os
import re
import subprocess
import sys

from pathlib import Path

import pytest

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))

import gen_board  # noqa: E402

from launch import clean_monitor_text  # noqa: E402
from process_utils import terminate_process_group  # noqa: E402


@pytest.fixture
def renode(tmp_path):
    executable = Path(os.environ.get('RENODE', ROOT / 'build/renode/renode'))
    if not executable.is_file():
        pytest.skip('build Renode or set RENODE')

    def run(script):
        path = tmp_path / 'test.resc'
        path.write_text(script)
        process = subprocess.Popen(
            [str(executable.resolve()), '--disable-xwt', '--console',
             '-e', 'include @%s' % path, '-e', 'quit'],
            cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
            env=dict(os.environ, TMPDIR=str(tmp_path),
                     XDG_CONFIG_HOME=str(tmp_path / 'xdg')),
            start_new_session=True)
        try:
            stdout, _ = process.communicate(timeout=60)
        except subprocess.TimeoutExpired:
            terminate_process_group(process, graceful_timeout=0.2)
            stdout, _ = process.communicate()
            pytest.fail(clean_monitor_text(stdout))
        output = clean_monitor_text(stdout)
        assert process.returncode == 0, output
        assert 'There was an error' not in output, output
        return output

    return run


def test_generated_gpio_defaults_survive_reset(tmp_path, renode):
    generated = gen_board.generate(ROOT, 'CubeOrangePlus', tmp_path / 'generated')
    script = generated['resc'].read_text()
    reset = re.search(r'macro reset\n""".*?"""\nrunMacro \$reset', script, re.S)
    assert reset is not None
    platform = tmp_path / 'gpio.repl'
    platform.write_text('''
flash: Memory.MappedMemory @ sysbus 0x08000000
    size: 0x200000
nvic: IRQControllers.NVIC @ sysbus 0xE000E000
    IRQ -> cpu@0
cpu: CPU.CortexM @ sysbus
    cpuType: "cortex-m7"
    nvic: nvic
gpioPortA: GPIOPort.STM32_GPIOPort @ sysbus <0x58020000, +0x400>
gpioPortB: GPIOPort.STM32_GPIOPort @ sysbus <0x58020400, +0x400>
gpioPortC: GPIOPort.STM32_GPIOPort @ sysbus <0x58020800, +0x400>
gpioPortE: GPIOPort.STM32_GPIOPort @ sysbus <0x58021000, +0x400>
''')
    output = renode('''
mach create
machine LoadPlatformDescription @%s
$vector_base=0x08020000
%s
sysbus ReadDoubleWord 0x58020410
machine Reset
sysbus ReadDoubleWord 0x58020410
cpu VectorTableOffset
''' % (platform, reset.group()))
    assert output.count('0x000000A0') == 2, output
    assert '0x08020000' in output, output


@pytest.mark.parametrize(('platform_name', 'address', 'page_size'), [
    ('stm32f103_base.repl', 0x08000800, 1024),
    ('stm32f105_base.repl', 0x08020800, 2048),
])
def test_f1_flash_page_geometry(renode, platform_name, address, page_size):
    includes = (HERE / 'scripts/ardupilot_f103.resc').read_text().split('mach create')[0]
    output = renode('''
$repo=@%s
include $repo/Tools/renode/peripherals/common/AP_SigrokInterface.cs
%s
mach create
machine LoadPlatformDescription @%s
sysbus WriteDoubleWord 0x%X 0x12345678
sysbus WriteDoubleWord 0x%X 0x12345678
sysbus WriteDoubleWord 0x%X 0x12345678
sysbus WriteDoubleWord 0x40022004 0x45670123
sysbus WriteDoubleWord 0x40022004 0xCDEF89AB
sysbus WriteDoubleWord 0x40022014 0x%X
sysbus WriteDoubleWord 0x40022010 0x42
sysbus ReadDoubleWord 0x%X
sysbus ReadDoubleWord 0x%X
sysbus ReadDoubleWord 0x%X
''' % (ROOT, includes, HERE / 'platforms' / platform_name,
       address, address + page_size - 4, address + page_size,
       address, address, address + page_size - 4, address + page_size))
    assert output.count('0xFFFFFFFF') == 2, output
    assert '0x12345678' in output, output


@pytest.mark.parametrize('blocked_write', [False, True])
def test_persistent_flash_replaces_existing_image(tmp_path, renode, blocked_write):
    image = tmp_path / 'flash.img'
    image.write_bytes(b'old image')
    temporary = image.with_name(image.name + '.tmp')
    if blocked_write:
        temporary.mkdir()
    platform = tmp_path / 'flash.repl'
    platform.write_text('''
flash: Memory.MappedMemory @ sysbus 0x08000000
    size: 0x1000
persistence: Miscellaneous.AP_PersistentMemory @ none
    fileName: "%s"
    address: 0x08000000
    size: 0x1000
''' % image)
    output = renode('''
include @%s
mach create
machine LoadPlatformDescription @%s
sysbus WriteDoubleWord 0x08000000 0x12345678
start
pause
''' % (HERE / 'peripherals/common/AP_PersistentMemory.cs', platform))
    if blocked_write:
        assert 'Failed to persist' in output
        assert image.read_bytes() == b'old image'
    else:
        assert image.read_bytes() == b'\x78\x56\x34\x12' + bytes(4092)
        assert not temporary.exists()

def test_pixhawk6c_compass_and_depth_platform(tmp_path):
    generated = gen_board.generate(
        ROOT, 'Pixhawk6C', tmp_path / 'generated',
        attachments=[{'device': 'keller-depth', 'port': 'I2C1'}])
    
    platform = generated['repl'].read_text()

    assert 'Sensors.AP_IST8310 @ i2c4 0x0C\n    Rotation: 0' in platform
    assert 'Sensors.AP_RM3100 @ i2c4 0x20\n    Rotation: 8' in platform
    assert 'Sensors.AP_Baro_KellerLD_Depth @ i2c1 0x40' in platform


def test_pixhawk6c_ignores_legacy_onboard_compass_attachment(tmp_path):
    generated = gen_board.generate(
        ROOT, 'Pixhawk6C', tmp_path / 'generated',
        attachments=[
            {'device': 'rm3100-compass', 'port': 'I2C0', 'configuration': {'orientation': 0}},
            {'device': 'rm3100-compass', 'port': 'I2C1'},
        ])
    
    platform = generated['repl'].read_text()

    assert platform.count('@ i2c4 0x20') == 1
    assert 'Sensors.AP_RM3100 @ i2c4 0x20\n    Rotation: 8' in platform
    assert 'configDevice1: Sensors.AP_RM3100 @ i2c1 0x20' in platform
    assert [attachment['id'] for attachment in generated['attachments']] == ['configDevice1']

    ports = gen_board.configuration_ports(ROOT, 'Pixhawk6C', tmp_path / 'ports')
    i2c0 = next(port for port in ports if port['id'] == 'I2C0')
    
    assert i2c0['onboard_compasses'] == {0x0C: 'Sensors.AP_IST8310', 0x20: 'Sensors.AP_RM3100'}
    assert gen_board.is_onboard_compass_attachment(i2c0, gen_board.ATTACHABLE_DEVICES['rm3100-compass'])

def test_pixhawk6c_rejects_conflicting_onboard_compass_attachment(tmp_path):
    with pytest.raises(ValueError, match='0x0C is reserved for onboard'):
        gen_board.generate(
            ROOT, 'Pixhawk6C', tmp_path / 'generated',
            attachments=[{'device': 'ak09916-compass', 'port': 'I2C0'}])


def test_iomcu_timeout_and_keller_depth(tmp_path, renode):
    probe = tmp_path / 'Probe.cs'
    probe.write_text('''
using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Miscellaneous;
using Antmicro.Renode.Peripherals.Sensors;
using Antmicro.Renode.Peripherals.UART;

namespace Antmicro.Renode.Peripherals.Miscellaneous
{
    public class Probe : IDoubleWordPeripheral, IKnownSize, IUART
    {
        public Probe(IMachine machine)
        {
            io = new AP_IOMCU(machine, this, 0);
            truth = AP_PhysicsState.ForMachine(machine).Current;
            depth = new AP_Baro_KellerLD_Depth(machine);
        }

        public void Configure()
        {
            WriteRegisters(50, 12, 22027);
            var failsafe = new ushort[16];
            failsafe[0] = 1500;
            failsafe[1] = 1550;
            WriteRegisters(55, 0, failsafe);
            WriteRegisters(54, 0, 1700, 1800);
        }

        public void Resume() => WriteRegisters(54, 0, 1600);
        public void SafetyOn() => WriteRegisters(50, 14, 22027);

        public void CheckOutputs(ushort first, ushort second)
        {
            var outputs = new AP_PhysicsActuator[AP_PhysicsActuator.Count];
            io.Sample(outputs);
            if(outputs[0].Value != first || outputs[1].Value != second)
            {
                throw new Exception($"Unexpected PWM: {outputs[0].Value}, {outputs[1].Value}");
            }
        }

        public void CheckPressure()
        {
            var surface = PressureRaw(0);
            var atFiveMeters = PressureRaw(5);
            // Keller's 2 bar calibration uses 32768 counts across its span.
            if(Math.Abs((atFiveMeters - surface) - 8034) > 2 || Math.Abs(PressureRaw(-1) - surface) > 1)
            {
                throw new Exception($"Unexpected pressure samples: {surface}, {atFiveMeters}");
            }
            for(var i = 0; i < 128; i++)
            {
                if(PressureRaw(0) != surface)
                {
                    return;
                }
            }
            throw new Exception("Stationary depth sensor samples never change");
        }

        private int PressureRaw(double down)
        {
            truth.PositionNedM[2] = down;
            depth.Write(new byte[] { 0xAC });
            var measurement = depth.Read(5);
            return (measurement[1] << 8) | measurement[2];
        }

        private void WriteRegisters(byte page, byte offset, params ushort[] values)
        {
            var packet = new byte[4 + 2 * values.Length];
            packet[0] = (byte)(0x40 | values.Length);
            packet[2] = page;
            packet[3] = offset;
            for(var i = 0; i < values.Length; i++)
            {
                packet[4 + 2 * i] = (byte)values[i];
                packet[5 + 2 * i] = (byte)(values[i] >> 8);
            }
            byte crc = 0;
            foreach(var value in packet)
            {
                crc ^= value;
                for(var bit = 0; bit < 8; bit++)
                {
                    crc = (byte)((crc & 0x80) != 0 ? (crc << 1) ^ 0x07 : crc << 1);
                }
            }
            packet[1] = crc;
            foreach(var value in packet)
            {
                CharReceived(value);
            }
        }

        public void Reset() => io.Reset();
        public void WriteChar(byte value) { }
        public uint ReadDoubleWord(long offset) => 0;
        public void WriteDoubleWord(long offset, uint value) { }
        public long Size => 4;
        public uint BaudRate => 1500000;
        public Parity ParityBit => Parity.None;
        public Bits StopBits => Bits.One;
        public event Action<byte> CharReceived;

        private readonly AP_IOMCU io;
        private readonly AP_PhysicsTruth truth;
        private readonly AP_Baro_KellerLD_Depth depth;
    }
}
''')
    platform = tmp_path / 'probe.repl'
    platform.write_text('probe: Miscellaneous.Probe @ sysbus 0x1000\n')
    output = renode(f'''
$repo=@{ROOT}
include $repo/Tools/renode/peripherals/common/AP_Physics.cs
include $repo/Tools/renode/peripherals/common/AP_IOMCU.cs
include $repo/Tools/renode/peripherals/common/AP_I2CRegisterDevice.cs
include $repo/Tools/renode/peripherals/sensors/AP_AdditionalBarometers.cs
include @{probe}
mach create
machine LoadPlatformDescription @{platform}
sysbus.probe Configure
emulation RunFor "0.199"
sysbus.probe CheckOutputs 1700 1800
emulation RunFor "0.002"
sysbus.probe CheckOutputs 1500 1550
sysbus.probe Resume
sysbus.probe CheckOutputs 1600 1550
emulation RunFor "0.201"
sysbus.probe SafetyOn
sysbus.probe CheckOutputs 0 0
sysbus.probe CheckPressure
echo "Peripheral checks passed"
''')
    assert 'Peripheral checks passed' in output
