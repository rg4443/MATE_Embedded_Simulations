set -euo pipefail
layout=${1:?layout required}
export GZ_IP=127.0.0.1
export GZ_PARTITION="${layout}_sitl_demo"
export GZ_SIM_RESOURCE_PATH='/Users/rafaelgarcia/ardupilot/docs/bluerov-gazebo/output/2026-10-27-design-cases/cases/compact_light/generated/models'
export GZ_SIM_SYSTEM_PLUGIN_PATH='/Users/rafaelgarcia/ardupilot/docs/bluerov-gazebo/.local/rov-sim'
exec gz sim -s -r -v 2 "/Users/rafaelgarcia/ardupilot/docs/bluerov-gazebo/output/2026-10-27-design-cases/cases/compact_light/generated/worlds/$layout.sdf"
