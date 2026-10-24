set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
layout=${1:-bluerov2}

case "$layout" in
    bluerov1)
        frame=vectored
        base_params=sub.parm
        ;;
    bluerov2)
        frame=vectored
        base_params=sub.parm
        ;;
    bluerov2-heavy)
        frame=vectored_6dof
        base_params=sub-6dof.parm
        ;;
    *)
        echo "Usage: $0 [bluerov1|bluerov2|bluerov2-heavy]" >&2
        exit 2
        ;;
esac

if [[ ! -x "$repo_root/.venv/bin/python" ]]; then
    echo "Missing $repo_root/.venv/bin/python (with wxPython installed)" >&2
    exit 1
fi

set +e
running_vehicles=$(pgrep -x 'ardusub|arducopter|arduplane|ardurover|antennatracker|blimp' 2>&1)
pgrep_status=$?
set -e

if [[ $pgrep_status -eq 0 ]]; then
    echo "Stop all other ArduPilot SITL instances before using this launcher." >&2
    echo "The standard launcher can stop them when it exits on this Mac." >&2
    exit 1
fi
if [[ $pgrep_status -ne 1 ]]; then
    echo "Could not check for other SITL processes: $running_vehicles" >&2
    exit 1
fi

cd "$repo_root"
if [[ ! -f "$repo_root/docs/bluerov-gazebo/generated/params/$layout.parm" ]]; then
    echo "Generated frame parameters are missing. Run docs/bluerov-gazebo/prepare_mac.sh first." >&2
    exit 1
fi

export MAVPROXY_CMD="$repo_root/.venv/bin/mavproxy.py"
export PATH="$repo_root/.venv/bin:$PATH"
export DISPLAY="${DISPLAY:-:0}"
exec "$repo_root/.venv/bin/python" "$repo_root/Tools/autotest/sim_vehicle.py" \
    --gui -v ArduSub -f "$frame" --model JSON \
    -I1 --sysid 2 -L Jervis_Bay -N \
    --add-param-file "$repo_root/Tools/autotest/default_params/$base_params" \
    --add-param-file "$repo_root/docs/bluerov-gazebo/generated/params/$layout.parm" \
    --no-extra-ports --out 127.0.0.1:14660 \
    --mavproxy-args=--source-system=255 \
    --use-dir "/private/tmp/$layout-gui-sitl"