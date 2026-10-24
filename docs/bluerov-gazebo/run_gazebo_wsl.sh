set -euo pipefail

package_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
layout=${1:-bluerov2}
part=${2:-server}
rov_sim_dir="$package_dir/.local/rov-sim"

case "$layout" in
    bluerov1|bluerov2|bluerov2-heavy) ;;
    *) echo "Usage: $0 [bluerov1|bluerov2|bluerov2-heavy] [server|gui]" >&2; exit 2 ;;
esac

case "$part" in
    server|gui) ;;
    *) echo "Usage: $0 [bluerov1|bluerov2|bluerov2-heavy] [server|gui]" >&2; exit 2 ;;
esac

if [[ ! -f "$package_dir/generated/worlds/$layout.sdf" ]]; then
    echo "Generated world missing. Run docs/bluerov-gazebo/prepare_wsl.sh first." >&2
    exit 1
fi

if [[ ! -f "$rov_sim_dir/plugins/libEER_Thruster.so" ]]; then
    echo "Linux rov-sim plugins missing under $rov_sim_dir" >&2
    exit 1
fi

export GZ_IP=127.0.0.1
export GZ_PARTITION="${layout}_sitl_demo"
export GZ_SIM_RESOURCE_PATH="$package_dir/generated/models"

if [[ "$part" == server ]]; then
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$rov_sim_dir"
    exec gz sim -s -r -v 2 "$package_dir/generated/worlds/$layout.sdf"
fi

exec gz sim -g
