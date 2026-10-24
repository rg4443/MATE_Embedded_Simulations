#!/usr/bin/env bash
# Prepare the locally built dependencies and generated BlueROV worlds on macOS.

set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
package_dir="$repo_root/docs/bluerov-gazebo"
local_dir="$package_dir/.local"
rov_sim_dir="$local_dir/rov-sim"
connector_dir="$local_dir/ardupilot_gazebo"
generated_dir="$package_dir/generated"

if [[ $(uname -s) != Darwin ]]; then
    echo "This preparation script has only been checked for macOS." >&2
    exit 1
fi
for command in git cmake gz python3; do
    if ! command -v "$command" >/dev/null 2>&1; then
        echo "Missing required command: $command" >&2
        exit 1
    fi
done
if [[ ! -x "$repo_root/build/sitl/bin/ardusub" ]]; then
    echo "Build ArduSub SITL first: ./waf configure --board sitl && ./waf sub" >&2
    exit 1
fi
if [[ -e "$generated_dir" ]]; then
    echo "Generated worlds already exist at $generated_dir; move that folder aside before preparation." >&2
    echo "Existing worlds were not changed." >&2
    exit 1
fi

mkdir -p "$local_dir"
if [[ ! -d "$rov_sim_dir/.git" ]]; then
    git clone https://github.com/EasternEdgeRobotics/rov-sim.git "$rov_sim_dir"
    git -C "$rov_sim_dir" checkout 3597a5c2c616a1c6ca59070d4ff3e61a67252fb2
    git -C "$rov_sim_dir" apply --unidiff-zero "$package_dir/rov-sim-macos.patch"
fi
if [[ $(git -C "$rov_sim_dir" rev-parse HEAD) != 3597a5c2c616a1c6ca59070d4ff3e61a67252fb2 ]] ||
    ! git -C "$rov_sim_dir" apply --unidiff-zero --reverse --check "$package_dir/rov-sim-macos.patch"; then
    echo "rov-sim checkout does not match the pinned, patched task version: $rov_sim_dir" >&2
    exit 1
fi
if [[ ! -d "$connector_dir/.git" ]]; then
    git clone https://github.com/ArduPilot/ardupilot_gazebo.git "$connector_dir"
    git -C "$connector_dir" checkout 082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5
    git -C "$connector_dir" apply --unidiff-zero "$repo_root/docs/beaumont-ardupilot-gazebo-test.patch"
fi
if [[ $(git -C "$connector_dir" rev-parse HEAD) != 082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5 ]] ||
    ! git -C "$connector_dir" apply --unidiff-zero --reverse --check "$repo_root/docs/beaumont-ardupilot-gazebo-test.patch"; then
    echo "Connector checkout does not match the pinned, patched task version: $connector_dir" >&2
    exit 1
fi

cmake -S "$rov_sim_dir" -B "$rov_sim_dir/build-macos" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$rov_sim_dir/build-macos" --target EER_Buoyancy EER_Hydrodynamics EER_Thruster -j4
GZ_VERSION=harmonic cmake -S "$connector_dir" -B "$connector_dir/build" -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build "$connector_dir/build" --target ArduPilotPlugin -j4

for plugin in EER_Buoyancy EER_Hydrodynamics EER_Thruster; do
    if [[ ! -f "$rov_sim_dir/plugins/lib${plugin}.dylib" ]]; then
        echo "Missing built rov-sim plugin: lib${plugin}.dylib" >&2
        exit 1
    fi
done
connector_lib="$connector_dir/build/libArduPilotPlugin.dylib"
if [[ ! -f "$connector_lib" ]]; then
    echo "Missing built connector: $connector_lib" >&2
    exit 1
fi
python3 "$package_dir/generate.py" --connector-lib "$connector_lib" --output "$generated_dir"
python3 "$package_dir/verify.py" --generated "$generated_dir" --ardupilot "$repo_root"
echo "Prepared BlueROV task package in $package_dir"
