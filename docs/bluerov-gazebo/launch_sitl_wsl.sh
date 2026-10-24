set -euo pipefail

repo_root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
layout=${1:-bluerov2}

case "$layout" in
    bluerov1|bluerov2)
        frame=vectored
        base_params=sub.parm
        ;;
    bluerov2-heavy)
        frame=vectored_6dof
        base_params=sub-6dof.parm
        ;;
    *) echo "Usage: QGC_HOST=<Windows-host-IP> $0 [bluerov1|bluerov2|bluerov2-heavy]" >&2; exit 2 ;;
esac

if [[ $(uname -s) != Linux ]]; then
    echo "Run this launcher inside Ubuntu on WSL2 or another Linux host." >&2
    exit 1
fi

if [[ -z ${QGC_HOST:-} ]]; then
    echo "Set QGC_HOST to the Windows host address that QGroundControl can receive from WSL." >&2
    echo "For standard WSL2 NAT, inspect: ip -4 route show default" >&2
    echo "For mirrored networking, 127.0.0.1 may work; verify the QGroundControl link." >&2
    exit 2
fi

if [[ ! -x "$repo_root/build/sitl/bin/ardusub" ]]; then
    echo "Build ArduSub SITL first: ./waf configure --board sitl && ./waf sub" >&2
    exit 1
fi

if [[ ! -f "$repo_root/docs/bluerov-gazebo/generated/params/$layout.parm" ]]; then
    echo "Generated frame parameters are missing. Run docs/bluerov-gazebo/prepare_wsl.sh first." >&2
    exit 1
fi

if ! command -v mavproxy.py >/dev/null 2>&1; then
    echo "MAVProxy is missing from PATH. Install the ArduPilot Ubuntu prerequisites." >&2
    exit 1
fi

cd "$repo_root"
export MAVPROXY_CMD=$(command -v mavproxy.py)
exec python3 Tools/autotest/sim_vehicle.py \
    -v ArduSub -f "$frame" --model JSON \
    -I1 --sysid 2 -L Jervis_Bay -N \
    --add-param-file "$repo_root/Tools/autotest/default_params/$base_params" \
    --add-param-file "$repo_root/docs/bluerov-gazebo/generated/params/$layout.parm" \
    --no-extra-ports --out "$QGC_HOST:14660" \
    --mavproxy-args=--source-system=255 \
    --use-dir "$repo_root/docs/bluerov-gazebo/.local/sitl/$layout"
