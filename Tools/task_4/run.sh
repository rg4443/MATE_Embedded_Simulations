#!/usr/bin/env bash
set -euo pipefail

task_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd "$task_dir/../.." && pwd)
package_dir="$repo_root/docs/bluerov-gazebo"
state_dir="$task_dir/.local"
layout=${2:-bluerov2}

if [[ $(uname -s) != Linux ]]; then
    echo "Run this inside ubuntu. See Tools/task_4/SETUP.md." >&2
    exit 1
fi

case "$layout" in
    bluerov1|bluerov2|bluerov2-heavy) ;;
    *) echo "Usage: $0 {blueos|gazebo|gui|sitl} [bluerov1|bluerov2|bluerov2-heavy]" >&2; exit 2 ;;
esac

case "${1:-}" in
    blueos)
        command -v docker >/dev/null || { echo "Docker is required." >&2; exit 1; }
        docker info >/dev/null

        mkdir -p "$state_dir/blueos/config" "$state_dir/blueos/blueos" \
            "$state_dir/blueos/logs" "$state_dir/blueos/userdata"

        exec docker run --rm --name mate-task-4-blueos \
            --privileged --network host --pid host \
            -e BLUEOS_DISABLE_SERVICES=cable_guy,wifi,commander \
            -e BLUEOS_DISABLE_MEMORY_LIMIT=true \
            -e BLUEOS_DISABLE_STARTUP_UPDATE=true \
            -v "$state_dir/blueos/config:/root/.config" \
            -v "$state_dir/blueos/blueos:/etc/blueos" \
            -v "$state_dir/blueos/logs:/var/logs/blueos" \
            -v "$state_dir/blueos/userdata:/usr/blueos/userdata" \
            bluerobotics/blueos-core:master
        ;;
    gazebo|gui)
        command -v gz >/dev/null || { echo "Gazebo is required." >&2; exit 1; }
        [[ -f "$package_dir/generated/worlds/$layout.sdf" ]] || {
            echo "Prepare the linux gazebo worlds first; see Tools/task_4/SETUP.md." >&2
            exit 1
        }

        export GZ_IP=127.0.0.1
        export GZ_PARTITION=mate_task_4
        export GZ_SIM_RESOURCE_PATH="$package_dir/generated/models"

        if [[ $1 == gazebo ]]; then
            export GZ_SIM_SYSTEM_PLUGIN_PATH="$package_dir/.local/rov-sim"
            exec gz sim -s -r -v 2 "$package_dir/generated/worlds/$layout.sdf"
        fi

        exec gz sim -g
        ;;
    sitl)
        [[ -x "$repo_root/build/sitl/bin/ardusub" ]] || {
            echo "Build ArduSub SITL first: ./waf configure --board sitl && ./waf sub" >&2
            exit 1
        }
        [[ -f "$package_dir/generated/params/$layout.parm" ]] || {
            echo "Prepare the linux gazebo worlds first; see Tools/task_4/SETUP.md." >&2
            exit 1
        }

        if [[ $layout == bluerov2-heavy ]]; then
            base_params=sub-6dof.parm
        else
            base_params=sub.parm
        fi

        mkdir -p "$state_dir/sitl/$layout"
        cd "$state_dir/sitl/$layout"

        defaults="$repo_root/Tools/autotest/default_params/$base_params,$package_dir/generated/params/$layout.parm"

        exec "$repo_root/build/sitl/bin/ardusub" --model JSON --defaults "$defaults" \
            -I1 --sysid 2 --home -35.1192416,150.7296753,0.0,0
        ;;
    *)
        echo "Usage: $0 {blueos|gazebo|gui|sitl} [bluerov1|bluerov2|bluerov2-heavy]" >&2
        exit 2
        ;;
esac
