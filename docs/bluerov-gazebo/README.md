# MATE Task #2: BlueROV frame checks with ArduSub SITL and Gazebo

## Start here: task package

This directory is the source package for the team's **preliminary** frame
comparison. A fresh checkout does not contain compiled Gazebo plugins or the
`generated/` worlds; those are specific to each computer and are intentionally
excluded from Git. The [beginner presentation](output/ArduSub_Gazebo_Beginner_Guide.pptx)
explains the parts of the simulation. The [Task #2 brief](https://docs.google.com/document/d/17-y8o9hq2qN4Wap0Io4mJ_gr75AT7Tn-V-Mq-fetGS4/edit)
provides the broader onboarding exercise. The [Beaumont integration record](../beaumont-ardusub-gazebo-verification.md)
records the earlier experiment; its `/Users/rafaelgarcia` paths are historical
and are not setup commands for other members.

Task #2 is completed **individually**. Follow the installation and run path
for the computer you have: the Mac and Windows sections are alternatives, not
two setups that every member must complete. Record your own setup and clearly
identify the source of any data you analyze.

The integrated route below was checked on **macOS with Gazebo Harmonic**.
An **experimental Ubuntu/WSL2 route** follows it. Its Linux generation checks
can run without Windows, but the connector build, WSL graphics, and Windows
QGroundControl link have not been verified on a Windows machine. Record which
route was used for every trial.

### Checked Mac route

1. Install [Gazebo Harmonic for macOS](https://gazebosim.org/docs/harmonic/install_osx/),
   CMake, Git, Qt5, and ArduPilot's SITL requirements. The task brief covers
   the introductory installation.
   Build this checkout's ArduSub binary with `./waf configure --board sitl`
   followed by `./waf sub`. For the optional graphical SITL launcher, install
   MAVProxy and wxPython into this checkout's `.venv`.
2. From the ArduPilot root, run `bash docs/bluerov-gazebo/prepare_mac.sh`.
   It clones pinned `rov-sim` and `ardupilot_gazebo` revisions into the ignored
   `.local/` directory, applies the included macOS and integer-ESC patches,
   builds the needed plugins, generates all three worlds, and checks their
   motor mappings. It does not alter either upstream repository. Existing
   `generated/` files are never overwritten; move that folder aside before
   rerunning preparation if it points to an old connector build.
3. In Terminal A, run `bash docs/bluerov-gazebo/run_gazebo_mac.sh bluerov2 server`.
   In Terminal B, run `bash docs/bluerov-gazebo/run_gazebo_mac.sh bluerov2 gui`.
   Both terminals run on the same Mac. The GUI is optional for data collection.
4. In Terminal C, run `bash docs/bluerov-gazebo/launch_sitl_gui.sh bluerov2`.
   Stop other SITL instances first, then click **Launch SITL**. QGroundControl
   can connect to vehicle ID 2 on UDP port 14660 and use its virtual joystick.
   For the direct terminal route, see the command record below.
5. Repeat with `bluerov1` and `bluerov2-heavy`, changing the layout name in
   both Gazebo commands and the SITL launcher. Stop one layout before starting
   another because they share the SITL/Gazebo ports.

The task output is a small, reproducible data set for the team's next frame
decision: for each layout, save the SITL `.BIN` log, Gazebo pose recording,
the parameters and command sequence, a plot with units, and a note of the
model assumptions. These models have no measured mass, drag, buoyancy, or
thruster geometry from the eventual ROV. A comparison can support a
**provisional** choice or identify what needs measurement; it cannot establish
real-vehicle performance. See [Trial records](#trial-records) below.

### Experimental Windows route through Ubuntu on WSL2

Run ArduSub SITL and Gazebo in the **same Ubuntu 24.04 WSL2 distribution**.
Run QGroundControl as a normal Windows application. This keeps the
ArduSub-to-Gazebo JSON link inside Ubuntu. The Windows-to-WSL MAVLink link
still needs a live check.

1. Follow the [ArduPilot WSL setup](https://ardupilot.org/dev/docs/sitl-on-windows-wsl.html)
   and [Gazebo Harmonic Ubuntu installation](https://gazebosim.org/docs/harmonic/install_ubuntu/).
   Inside Ubuntu, install `git`, `cmake`, `build-essential`, `pkg-config`,
   `qtbase5-dev`, and `rapidjson-dev`; then build this task checkout with
   `./waf configure --board sitl` and `./waf sub`. QGroundControl stays on
   Windows. The WSL route needs this prepared task checkout, not a fresh
   upstream ArduPilot clone.
2. From the ArduPilot root **inside WSL**, run
   `bash docs/bluerov-gazebo/prepare_wsl.sh`. It builds Linux `.so` plugins,
   generates the three models, and runs static motor-map checks. Move an old
   `generated/` folder aside before changing between Mac and Linux builds.
3. In separate Ubuntu terminals, run
   `bash docs/bluerov-gazebo/run_gazebo_wsl.sh bluerov2 server` and, if WSLg
   graphics work, `bash docs/bluerov-gazebo/run_gazebo_wsl.sh bluerov2 gui`.
   The server can run without the GUI.
4. Set `QGC_HOST` to the address of the Windows host as seen from WSL. Under
   standard WSL2 NAT, inspect `ip -4 route show default` and use its gateway
   address. With mirrored networking, `127.0.0.1` may work. Verify either
   address rather than assuming it works. Start SITL with
   `QGC_HOST=<address> bash docs/bluerov-gazebo/launch_sitl_wsl.sh bluerov2`.
5. In Windows QGroundControl, create a UDP link listening on port `14660`.
   Confirm a heartbeat from vehicle **2** and changing position or altitude.
   Select Manual, arm, move one virtual joystick axis briefly, and disarm.
   In WSL, confirm that the matching Gazebo motor topic changes from neutral
   and the model moves in the commanded direction. If only the heartbeat
   arrives, check the joystick/MAVLink command path; if no heartbeat arrives,
   check `QGC_HOST`, WSL networking, and the Windows firewall.

This is a **proposed, unverified route**, not a claim of successful Windows
integration. If the integrated WSL route does not work, complete the task
individually by specifying a controlled frame-comparison trial and analyzing
any Mac-generated `.BIN` logs and Gazebo pose recordings provided for the task.
If no run data is available, report the proposed measurements and the limits
of the conclusion. Identify who ran any shared trial and which setup produced
its data; do not present an unrun trial as a measured result.

This folder contains three **schematic** Gazebo ROVs for checking the motor
mapping used by ArduSub. They are separate from Beaumont and Waterwitch.
They use a simple box body, generic thruster positions, approximate neutral
buoyancy, and the same placeholder drag coefficients for all three layouts.
They are useful for learning, checking axes, and testing the SITL-to-Gazebo
connection. They are **not** dimensionally or hydrodynamically validated
models of commercial BlueROVs and must not be used to rank real hardware
performance or to control a physical ROV.

| Model/world | ArduSub `FRAME_CONFIG` | Thrusters | Axes represented |
| --- | ---: | ---: | --- |
| `bluerov1.sdf` | 0, BlueROV1 | 6 | roll, pitch, yaw, depth, forward, lateral |
| `bluerov2.sdf` | 1, Vectored | 6 | roll, yaw, depth, forward, lateral |
| `bluerov2-heavy.sdf` | 2, Vectored 6DOF | 8 | roll, pitch, yaw, depth, forward, lateral |

ArduSub's frame names and motor factors come from
[`AP_Motors6DOF.cpp`](../../libraries/AP_Motors/AP_Motors6DOF.cpp) and its
[frame guide](https://software.ardupilot.org/sub/docs/sub-frames.html).
The six-motor BlueROV2 layout has no direct pitch control; the Heavy layout
adds two more vertical thrusters for pitch and stronger vertical control.

## What was created

- [`generate.py`](generate.py) writes three model SDFs, three worlds, and
  one `FRAME_CONFIG` parameter file per world into `generated/`.
- [`verify.py`](verify.py) checks all motor-factor rows against the checked-out
  ArduSub source, confirms every Gazebo thruster axis and topic mapping, and
  checks that the idealized force-and-torque matrix has the expected rank.
- `generated/` is per-computer output from `prepare_mac.sh` or
  `prepare_wsl.sh`. It is ignored by Git because its SDFs contain an absolute
  path to the locally built ArduPilot Gazebo connector. Regenerate after
  moving or rebuilding the library.

No ArduSub motor-control code or original `rov-sim` model was changed.
The required connector changes for macOS and `rov-sim`'s integer ESC topics
are recorded in the [Beaumont connector notes](../beaumont-ardusub-gazebo-verification.md)
and [test patch](../beaumont-ardupilot-gazebo-test.patch). The connector patch
is experimental and changes the `COMMAND` topic type to `gz.msgs.Int32`.

## How ArduSub and Gazebo are connected

The connection is **inside each generated Gazebo model**, not in the Gazebo
window or QGroundControl settings. Open the
[BlueROV2 model SDF](generated/models/bluerov2_test/model.sdf) and look for
its `<plugin name="ArduPilotPlugin">` block. Gazebo loads the locally built
`libArduPilotPlugin.dylib` on Mac or `libArduPilotPlugin.so` in Linux/WSL
when it loads the model. That block
sets `<fdm_port_in>9012</fdm_port_in>` and maps ArduSub motor outputs to the
Gazebo topics `/bluerov2_test/motor_1` through `/motor_6`. The nearby
`EER_Thruster` plugin blocks subscribe to those topics and apply thrust to
the model. BlueROV1 and Heavy have the same connection with six and eight
motors respectively.

For example, the BlueROV2 SDF contains this motor-1 connection (other motors
have corresponding `<control>` blocks):

```xml
<plugin name="ArduPilotPlugin" filename="/path/on/this/mac/libArduPilotPlugin.dylib">
  <fdm_port_in>9012</fdm_port_in>
  <control channel="0">
    <type>COMMAND</type>
    <cmd_topic>/bluerov2_test/motor_1</cmd_topic>
  </control>
</plugin>
```

Here `channel="0"` is ArduSub motor output **1**: the connector numbers
channels from zero, while ArduSub numbers motors from one.

ArduSub must start with **both** `--model JSON` and `-I1`. `--model JSON` tells
SITL to use an external physics simulation. Instance 1 sends its simulator
packets to UDP port `9002 + 10 × 1 = 9012`, which matches the SDF plugin.
The connector receives motor outputs from ArduSub, publishes ESC commands
inside Gazebo, then sends Gazebo's position, orientation, velocity, and IMU
readings back to ArduSub as JSON. This happens automatically once the two
processes start; there is no extra `connect` command.

```text
QGroundControl ──UDP 14660──> MAVProxy ──TCP 5770──> ArduSub SITL
                              │ motor outputs
                              ▼ UDP 9012
                         ArduPilotPlugin in Gazebo
                              │ integer ESC topics
                              ▼
                         Gazebo thrusters and physics
                              │ position + IMU readings
                              └──────JSON over UDP──────> ArduSub SITL
                                                       └──MAVLink position/depth──> QGroundControl
```

The MAVProxy TCP port `5770` carries **pilot commands and telemetry**. The
Gazebo connector's UDP port `9012` carries **motor and simulated sensor data**.
`GZ_PARTITION` only keeps this Gazebo server and its GUI together and separate
from other Gazebo sessions; it is not the ArduSub connection port. Your usual
SITL instance 0 and QGroundControl session can remain separate while testing
instance 1.

Before running the world, check that the connector library named in the SDF
exists. `prepare_mac.sh` normally creates it at:

```sh
ls docs/bluerov-gazebo/.local/ardupilot_gazebo/build/libArduPilotPlugin.dylib
```

If it has been removed, move `generated/` aside and rerun `prepare_mac.sh`.

## Checks performed on 2026-10-24

ArduPilot revision: `215fb9a118a80a8ef63a7b51272888f5b619df4a`.
The three generated worlds were initially run separately with Gazebo Harmonic,
ArduSub `--model JSON -I1`, and MAVProxy system ID 255. Gazebo returned
position, orientation, velocity, and IMU data to SITL in each run. That first
test exposed a reversed vertical direction in the generated models. The
vertical thruster axes and their schematic placements have since been fixed.

| Frame | Checks after the vertical-axis fix |
| --- | --- |
| BlueROV1 | Static motor-factor, force-direction, and rank checks passed; dynamic vertical retest pending |
| BlueROV2 | Static checks passed; isolated down/neutral/up MAVLink test confirmed Gazebo movement direction |
| BlueROV2 Heavy | Static motor-factor, force-direction, and rank checks passed; dynamic vertical retest pending |

Before the vertical-axis fix, a BlueROV2 forward RC override of 1600
produced ESC values `96, 96, 159, 159, 128, 128` for motors 1–6 and moved
the body forward. The horizontal motor axes were not changed. The corrected
vertical behavior is documented below with a fresh dynamic test.

Static check command, run from the ArduPilot root:

```sh
python3 docs/bluerov-gazebo/verify.py \
  --generated docs/bluerov-gazebo/generated \
  --ardupilot "$PWD"
```

It reported six matching motor rows and rank 6 for BlueROV1, six rows and
rank 5 for BlueROV2, and eight rows and rank 6 for BlueROV2 Heavy. Its
force-direction check now requires positive throttle to produce **upward**
force. Both Python files also passed `flake8` in the local `.venv`.

## Run one layout on a prepared Mac

Use **one layout at a time**: all three worlds use Gazebo connector UDP port
9012, matched to SITL instance 1. The commands below select BlueROV2. To try
another layout, change `bluerov2` to `bluerov1` or `bluerov2-heavy` in the
world and parameter filenames, and use `sub-6dof.parm` as the base defaults
file for Heavy. Stop each test before starting the next.

Run these from the ArduPilot repository root. The wrapper scripts set the
Gazebo partition and plugin paths for each layout.

**Terminal 1: Gazebo server**

```sh
bash docs/bluerov-gazebo/run_gazebo_mac.sh bluerov2 server
```

**Terminal 2: Gazebo window (optional on macOS)**

```sh
bash docs/bluerov-gazebo/run_gazebo_mac.sh bluerov2 gui
```

**Terminal 3: separate ArduSub SITL**

```sh
ardupilot_root="$PWD"
mkdir -p /private/tmp/bluerov2-sitl
cd /private/tmp/bluerov2-sitl
"$ardupilot_root/build/sitl/bin/ardusub" \
  --model JSON \
  --defaults "$ardupilot_root/Tools/autotest/default_params/sub.parm,$ardupilot_root/docs/bluerov-gazebo/generated/params/bluerov2.parm" \
  -I1 \
  --sysid 2 \
  --home -35.1192416,150.7296753,0.0,0
```

**Terminal 4: test controls**

```sh
ardupilot_root="$PWD"
cd /private/tmp/bluerov2-sitl
"$ardupilot_root/.venv/bin/mavproxy.py" \
  --master tcp:127.0.0.1:5770 \
  --baudrate 115200 \
  --source-system 255 \
  --out 127.0.0.1:14660
```

At the `MANUAL>` prompt, wait for `ArduPilot Ready`, then try one short
command at a time:

```text
param show FRAME_CONFIG
arm throttle
rc 5 1600
rc 5 1500
rc 3 1400
rc 3 1500
disarm
```

In these SITL defaults, RC channel 5 means forward and channel 3 means
vertical movement. **1400 on channel 3 means dive; 1600 means rise.** The
Gazebo `z` coordinate gets smaller as the vehicle dives. `--home` starts
the SITL vehicle at the existing `Jervis_Bay` location in
`Tools/autotest/locations.txt`. Stop MAVProxy, SITL, and the Gazebo server
with `Ctrl+C` when done.
The `--sysid 2` option gives this vehicle a different identity from your
existing SITL instance 0. The `--out` option sends its MAVLink traffic to a
separate QGroundControl UDP listening port. Keep MAVProxy open while using
QGroundControl.

### Use the familiar SITL launcher instead of Terminals 3 and 4

First start the Gazebo server (Terminal 1) and, if desired, its window
(Terminal 2). **Stop the BlueROV ArduSub instance 1 and its MAVProxy before
starting this alternative**, since both use TCP 5770 and Gazebo UDP 9012.
Also stop any other ArduPilot SITL instance. On this Mac, the stock
`sim_vehicle.py` launcher lacks `psutil` and its exit cleanup can stop other
SITL processes; the convenience script refuses to start if another vehicle
is running. The direct four-terminal method above avoids this limitation.
Then run this from any folder:

```sh
bash docs/bluerov-gazebo/launch_sitl_gui.sh bluerov2
```

The script opens the existing ArduPilot launcher with the following settings
(shown in full if you prefer to type them yourself from the ArduPilot root):

```sh
export MAVPROXY_CMD="$PWD/.venv/bin/mavproxy.py"
export DISPLAY=:0
.venv/bin/python Tools/autotest/sim_vehicle.py \
  --gui -v ArduSub -f vectored --model JSON \
  -I1 --sysid 2 -L Jervis_Bay -N \
  --add-param-file "$PWD/Tools/autotest/default_params/sub.parm" \
  --add-param-file "$PWD/docs/bluerov-gazebo/generated/params/bluerov2.parm" \
  --no-extra-ports --out 127.0.0.1:14660 \
  --mavproxy-args=--source-system=255 \
  --use-dir /private/tmp/bluerov2-gui-sitl
```

The ArduPilot SITL Launcher window should show **ArduSub** and **Jervis_Bay**.
Leave **Enable MAVProxy** checked, then click **Launch SITL**. The frame,
external JSON physics model, vehicle ID, and QGroundControl port are set by
the command above; the launcher window does not expose all of them. This
launcher starts ArduSub and MAVProxy, while Gazebo still starts in Terminal 1.
Use `.venv/bin/python`: the system `python3` on this Mac lacks `wxPython`,
which the launcher needs. `-N` reuses the existing SITL binary.

For BlueROV1, keep `-f vectored` and change the added frame parameter file
to `bluerov1.parm`. For BlueROV2 Heavy, change `-f vectored` to
`-f vectored_6dof`, use `sub-6dof.parm` as the base file, and use
`bluerov2-heavy.parm` as the frame file. Start the corresponding Gazebo world.

### Drive the Gazebo vehicle from QGroundControl

1. Start Gazebo, ArduSub, and MAVProxy with the four terminals above, or use
   the launcher alternative for ArduSub and MAVProxy. You can omit Terminal 2
   if you do not need to see Gazebo's 3D window.
2. In QGroundControl, open **Application Settings → Comm Links**, add a **UDP**
   link whose **local/listening port is 14660**, and connect it. Use this port
   even if QGroundControl already auto-connected to your original SITL on
   port 14550. You should see a second vehicle, system ID **2**.
3. Select vehicle 2 in QGroundControl. Check that it reports the frame you
   launched, for example BlueROV2, and that its position and depth update.
4. In **Application Settings → Fly View** (or **General** in some QGC
   versions), enable **Virtual Joystick**. Return to Fly view, select **Manual**,
   and arm vehicle 2. Move the forward stick gently, then release it. The
   Gazebo vehicle should move and QGroundControl's position should update.
   Test depth the same way while watching the depth value. Disarm when done.

The control path is QGroundControl virtual joystick → MAVLink
`MANUAL_CONTROL` → MAVProxy → ArduSub motor mixer → Gazebo connector → Gazebo
thrusters. Position and simulated sensor readings return through the connector
and ArduSub, then QGroundControl receives them as MAVLink telemetry.
QGroundControl does **not** connect directly to Gazebo.

QGroundControl's 2D map plots the latitude and longitude reported by ArduSub;
it does not render Gazebo's 3D ocean or model. With the Jervis Bay start
location above, the map marker should appear at Jervis Bay, Australia. The
generated Gazebo world is still a generic ocean scene, not a 3D recreation
of Jervis Bay. Depth is a telemetry value; the 2D map cannot show the
underwater scene.

**Control-link check performed on 2026-10-24:** With BlueROV2 Gazebo and
ArduSub instance 1 (`--sysid 2`) running, a test MAVLink client listened on
UDP 14660 and sent `MANUAL_CONTROL` forward input to vehicle 2. It received
vehicle-2 heartbeat and `GLOBAL_POSITION_INT` telemetry. ArduSub reported
forward channel 5 at 1600 for a 500-unit input; for an 800-unit input, its
motor outputs were `1340, 1340, 1660, 1660, 1500, 1500`, and reported
latitude changed from `-35.3632525` to `-35.3632263` degrees. This verifies
the same MAVLink command route used by the virtual joystick. The actual
QGroundControl interface was not operated in this check.

**Vertical-direction check after the fix:** A separate BlueROV2 Gazebo world
and ArduSub instance 2 (`--sysid 3`) started at Jervis Bay. A test MAVLink
client sent downward `MANUAL_CONTROL` `z=250`; ArduSub reported RC channel 3
at 1400 and vertical motors 5–6 at PWM 1600. Reported altitude moved from
about `-0.01 m` to `-0.52 m`. Gazebo's model `z` moved from approximately
`-0.89 m` to `-1.45 m` after a further downward pulse. An upward `z=750`
input moved reported altitude from about `-0.58 m` to `+0.02 m`. This tested
the same down/up command signs that the QGroundControl joystick sends.

**Confirm that the connection is working:** After MAVProxy connects, the
SITL terminal should print `JSON received` and list `imu`, `position`,
`quaternion`, and `velocity`. At neutral, the motor topic below should show
`data: 128`. While `rc 5 1600` is active, BlueROV2 motor 1 showed `data: 96`
in our test:

```sh
GZ_IP=127.0.0.1 GZ_PARTITION=bluerov2_sitl_demo \
  gz topic -e -n 1 -t /bluerov2_test/motor_1
```

If there is no `JSON received`, check that you launched the **generated**
world, that its connector `.dylib` exists, and that SITL used both
`--model JSON` and `-I1`. If the motor topic stays at `128`, check that SITL
says `ARMED` and that MAVProxy connected as `source_system=255`. The original
Beaumont world does not contain this connector.

## Regenerate after rebuilding the connector

```sh
python3 docs/bluerov-gazebo/generate.py \
  --connector-lib "$PWD/docs/bluerov-gazebo/.local/ardupilot_gazebo/build/libArduPilotPlugin.dylib" \
  --output /private/tmp/bluerov_models_new
```

The generator refuses to overwrite an existing output directory. The
`GZ_SIM_RESOURCE_PATH` and world paths in the run commands must point to the
new output if you use a different location. `rov-sim`'s thruster plugin
requires `GZ_SIM_SYSTEM_PLUGIN_PATH` to be **one directory** here; the
connector library is named by absolute path in each generated SDF.

## Trial records

Copy [`trial_record_template.csv`](trial_record_template.csv) into a separate
results folder and add one row per trial. Do not put large `.BIN` logs or raw
Gazebo recordings into this source package. For each layout, use the same
starting pose, mode, command size, duration, and nominal physical assumptions.
Save the exact `.BIN` file from `/private/tmp/<layout>-gui-sitl/logs/` when
using the launcher, or from the direct SITL working directory when using the
four-terminal route. Also record the Gazebo pose topic during each trial:

```sh
mkdir -p trials/bluerov2
GZ_IP=127.0.0.1 GZ_PARTITION=bluerov2_sitl_demo \
  gz topic --json-output -e -t /world/bluerov2_test_world/pose/info \
  > trials/bluerov2/pose.jsonl
```

Stop the recording with `Ctrl+C`. For BlueROV1 or Heavy, change both the
partition and topic names to the chosen layout (`bluerov1_test_world` or
`bluerov2_heavy_test_world`). Record the model assumptions even if no values
were changed. Plot at least depth, horizontal travel, and unintended motion
with units and trial counts. The CSV is a metadata template, not example
results; leave measured-result cells empty until a trial has actually run.

## Limits before using these models for design decisions

- The box geometry, 12 kg mass, 12 L displacement, thruster positions, and
  shared drag coefficients are approximate teaching values. Fit them to CAD,
  measured mass/inertia, buoyancy, and tow/drag measurements before comparing
  performance of real frames.
- The `rov-sim` T200 lookup curve defaults to 12 V here. Confirm voltage,
  propeller direction, ESC deadband, and motor response against the intended
  hardware.
- There is an IMU, but no separately modeled underwater pressure sensor,
  camera stream, tether, current, or architecture fault injection. Validate
  these before drawing conclusions about Depth Hold or embedded reliability.
- BlueROV1's source mixer has some intentional roll/lateral coupling. A
  rank-6 allocation means six axes are controllable, not perfectly decoupled.
