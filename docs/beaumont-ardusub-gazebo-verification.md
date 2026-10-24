# Beaumont + ArduSub SITL + Gazebo: verification notes

Date: 2026-10-24. This is an experimental integration record, not a validated
ROV control configuration. All motor commands below were sent to a separate
software simulation. No real vehicle was connected.

## Result

The Gazebo connector exchanged data in both directions with a separate ArduSub
SITL instance:

- SITL printed `JSON received` with `imu: gyro`, `imu: accel_body`, `position`,
  `quaternion`, and `velocity` after the Gazebo IMU topic was corrected.
- MAVProxy reported `Frame: VECTORED_6DOF`, `ArduPilot Ready`, and
  `FRAME_CONFIG 2`.
- With the test SITL armed and `rc 3 1600`, Beaumont's aft four ESC command
  topics changed from neutral `128` to `96`. Gazebo's
  `/beaumont/aft_star_top/force` topic reported `3.5047657109368804` N.
- `rc 3 1500` and `disarm` returned the test to neutral. The test processes
  were stopped afterward.

This proves sensor and motor-message exchange. It does **not** validate the
vehicle's motion, Depth Hold, or the proposed motor allocation. QGroundControl
was not used for this isolated test; MAVProxy supplied the RC input. The user's
existing QGroundControl and SITL session was left alone.

## Why the current motor allocation is wrong

ArduSub's `FRAME_CONFIG=2` selects the hard-coded `SUB_FRAME_VECTORED_6DOF`
case in `libraries/AP_Motors/AP_Motors6DOF.cpp`. Motors 1–4 have zero throttle
(vertical) factor. Motors 5–8 have zero forward and lateral factors. The
current test bridge assigned motor outputs 1–8 to Beaumont thrusters in SDF
order, without calculating an allocation matrix.

Beaumont's custom thruster plugin applies force along each propeller joint's
local `+Z` axis, transformed into the world frame. Rotating those axes by the
joint poses in `rov-sim/models/Beaumont/model.sdf` gives the following unit
directions in the Beaumont body frame (rounded):

| ArduSub output | Beaumont thruster | Body-frame force axis (x, y, z) | Stock mixer role |
| --- | --- | --- | --- |
| 1 | `for_star_top` | (+0.500, -0.500, +0.707) | horizontal |
| 2 | `for_port_top` | (-0.500, -0.500, +0.707) | horizontal |
| 3 | `for_star_bot` | (+0.500, -0.500, -0.707) | horizontal |
| 4 | `for_port_bot` | (-0.500, -0.500, -0.707) | horizontal |
| 5 | `aft_star_top` | (+0.500, +0.500, +0.707) | vertical |
| 6 | `aft_port_top` | (-0.500, +0.500, +0.707) | vertical |
| 7 | `aft_star_bot` | (+0.500, +0.500, -0.707) | vertical |
| 8 | `aft_port_bot` | (-0.500, +0.500, -0.707) | vertical |

Every Beaumont thruster has both horizontal and vertical components. A simple
permutation or polarity change of these eight outputs cannot make them match
the stock mixer. The isolated depth-input test is a concrete example: the
stock mixer commanded motors 5–8 together. For the four aft thrusters listed
above, equal positive thrust has zero net x and z component and a positive y
component. At 3.5048 N per thruster, the idealized body-frame resultant is
approximately `(0, +7.01, 0)` N, before buoyancy, drag, and moments are
considered. A depth command can therefore drive sideways motion in this test
mapping.

The direction calculation used the SDF joint poses and the `+Z` joint axis:

```sh
python3 - <<'PY'
import math
import xml.etree.ElementTree as ET
p = '/Users/rafaelgarcia/rov-sim/models/Beaumont/model.sdf'
for j in ET.parse(p).getroot().findall('.//joint'):
    if 'propeller_joint' not in j.get('name', ''):
        continue
    x, y, z, roll, pitch, yaw = map(float, j.find('pose').text.split())
    vx = math.cos(yaw)*math.sin(pitch)*math.cos(roll) + math.sin(yaw)*math.sin(roll)
    vy = math.sin(yaw)*math.sin(pitch)*math.cos(roll) - math.cos(yaw)*math.sin(roll)
    vz = math.cos(pitch)*math.cos(roll)
    print(j.get('name'), (round(vx, 3), round(vy, 3), round(vz, 3)))
PY
```

## Source and changes used for the isolated test

The source revisions at the time of testing were:

- ArduPilot: `215fb9a118a80a8ef63a7b51272888f5b619df4a`
- `rov-sim`: `3597a5c2c616a1c6ca59070d4ff3e61a67252fb2` **plus existing
  uncommitted local changes**. The test copied the current working-tree files.
- ArduPilot Gazebo connector: `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
  plus the test-only patch in
  [`beaumont-ardupilot-gazebo-test.patch`](beaumont-ardupilot-gazebo-test.patch).

No ArduPilot control code or original `rov-sim` source file was edited for this
test. The connector was cloned into `/private/tmp/ardupilot_gazebo_codex`, and
the world/model copy was put in `/private/tmp/beaumont_ap_test`.

The connector patch makes these test-only changes:

1. Explicitly finds Qt5 and skips the connector's separate GStreamer camera
   plugin target for this local macOS build.
2. Changes the connector's `COMMAND` control publisher from
   `gz.msgs.Double` to `gz.msgs.Int32`, rounding to the nearest integer,
   because Beaumont's ESC input subscriber accepts `gz.msgs.Int32` values
   from 0 to 255.

The copied Beaumont files were changed as follows:

1. The world include URI was changed to the absolute copied model path.
2. A Gazebo `ArduPilotPlugin` block was appended to the copied model, with
   UDP port `9012`, eight `COMMAND` controls, ESC scaling from PWM
   1100–1900 to integer 0–255, and the SDF-order topic mapping in the table.
3. The copied model's explicit IMU `<topic>imu</topic>` was removed. The
   connector looks up the scoped sensor topic
   `/world/BeaumontOceanWorld/model/Beaumont/link/rov_body/sensor/imu/imu`.
   Before that correction, motor packets reached Gazebo but SITL received no
   JSON sensor state.

Python's XML writer reformatted the copied SDF and omitted XML comments; those
formatting differences were not intentional model changes. The model-to-body
and Gazebo-to-NED rotations used in the connector block were provisional and
have not been validated against expected vehicle heading and depth axes.

## Commands used to repeat the test on this Mac

The temporary paths may be removed by the operating system. To recreate the
connector in a fresh directory, run the following commands. The current
`/private/tmp/ardupilot_gazebo_codex` checkout already has this patch; do not
apply it a second time there.

```sh
git clone https://github.com/ArduPilot/ardupilot_gazebo.git /private/tmp/ardupilot_gazebo_codex
git -C /private/tmp/ardupilot_gazebo_codex checkout 082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
git -C /private/tmp/ardupilot_gazebo_codex apply --unidiff-zero /Users/rafaelgarcia/ardupilot/docs/beaumont-ardupilot-gazebo-test.patch
GZ_VERSION=harmonic cmake -S /private/tmp/ardupilot_gazebo_codex -B /private/tmp/ardupilot_gazebo_codex/build -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build /private/tmp/ardupilot_gazebo_codex/build --target ArduPilotPlugin -j4
```

Create a fresh copy of the current `rov-sim` working tree with the persistent
[`beaumont-gazebo-prepare-test.py`](beaumont-gazebo-prepare-test.py) helper.
It consolidates the initial world/model copy, connector block, and later IMU
topic correction. It refuses to overwrite an existing output directory; the
command below is for first-time creation, or after moving the current test
directory aside.

```sh
/Users/rafaelgarcia/ardupilot/.venv/bin/python \
  /Users/rafaelgarcia/ardupilot/docs/beaumont-gazebo-prepare-test.py \
  --rov-sim /Users/rafaelgarcia/rov-sim \
  --connector-lib /private/tmp/ardupilot_gazebo_codex/build/libArduPilotPlugin.dylib \
  --output /private/tmp/beaumont_ap_test
```

The generated SDF is an experimental bridge, not a ready-to-merge model. The
helper was tested on a separate copy: it produced eight controls, removed the
explicit IMU topic, and pointed the world at the copied model.

In terminal 1, start Gazebo's server only. Its partition keeps it separate
from the user's normal Gazebo session:

```sh
GZ_IP=127.0.0.1 \
GZ_PARTITION=codex-beaumont-verify \
GZ_LOG_PATH=/private/tmp/beaumont_ap_test/gzlogs \
GZ_SIM_SYSTEM_PLUGIN_PATH=/Users/rafaelgarcia/rov-sim \
GZ_SIM_RESOURCE_PATH=/private/tmp/beaumont_ap_test/models:/private/tmp/beaumont_ap_test/worlds:/Users/rafaelgarcia/rov-sim \
gz sim -s -r -v 2 /private/tmp/beaumont_ap_test/worlds/BeaumontOceanWorld.sdf
```

In terminal 2, start separate SITL instance 1. This uses JSON external
physics, `FRAME_CONFIG=2` from the 6DOF defaults file, and simulator UDP
port `9002 + 10*1 = 9012`:

```sh
cd /private/tmp/beaumont_ap_test/sitl
/Users/rafaelgarcia/ardupilot/build/sitl/bin/ardusub \
  --model JSON \
  --defaults /Users/rafaelgarcia/ardupilot/Tools/autotest/default_params/sub-6dof.parm \
  -I1
```

In terminal 3, connect MAVProxy to that SITL instance. Leave its normal
`source_system=255`; this checkout's `MAV_GCS_SYSID` is 255 and ArduSub only
accepts RC overrides from that ID. A first attempt with
`--source-system 254` was rejected: telemetry worked but RC channel 3 and all
ESC outputs remained neutral.

```sh
cd /private/tmp/beaumont_ap_test/sitl
/Users/rafaelgarcia/ardupilot/.venv/bin/mavproxy.py --master tcp:127.0.0.1:5770 --baudrate 115200
```

At the MAVProxy prompt, the brief motor check was:

```text
param show FRAME_CONFIG
arm throttle
rc 3 1600
rc status
rc 3 1500
disarm
```

While `rc 3 1600` was held, read the Gazebo topics from another terminal
using the same partition:

```sh
GZ_IP=127.0.0.1 GZ_PARTITION=codex-beaumont-verify gz topic -e -n 1 -t /beaumont/aft_star_top
GZ_IP=127.0.0.1 GZ_PARTITION=codex-beaumont-verify gz topic -e -n 1 -t /beaumont/aft_star_top/force
GZ_IP=127.0.0.1 GZ_PARTITION=codex-beaumont-verify gz topic -i -t /world/BeaumontOceanWorld/model/Beaumont/link/rov_body/sensor/imu/imu
```

The original test data and logs are in `/private/tmp/beaumont_ap_test/sitl`
and `/private/tmp/beaumont_ap_test/verify-sitl.log` and `verify-gz.log`.
MAVProxy wrote `mav.tlog` there. The isolated Gazebo run was server-only;
no Gazebo GUI or QGroundControl session was attached to it.

## What remains before using this for design decisions

1. Specify and review a Beaumont control-allocation matrix from each
   thruster's force axis, location relative to the center of mass, propeller
   polarity, and thrust curve. Implement it in the appropriate control layer
   and verify one thruster and one motion axis at a time.
2. Verify the coordinate transforms, buoyancy, hydrodynamics, depth/pressure
   behavior, and mode response against known outcomes. The Beaumont model
   defines an IMU and cameras; it does not currently define a pressure sensor.
3. Only then attach QGroundControl to the Gazebo-backed SITL MAVLink stream
   and test its virtual joystick. QGroundControl-to-this-instance control was
   not tested here.

The current patch is a connection experiment, not a safe motor mixer for a
physical ROV or a basis for comparing thruster configurations.
