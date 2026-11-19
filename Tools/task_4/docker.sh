#!/usr/bin/env bash
set -euo pipefail

task_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
repo_root=$(cd "$task_dir/../.." && pwd)
image=mate-task-4-ubuntu
network=mate-task-4-network
volume=mate-task-4-work
sim=mate-task-4-sim
blueos=mate-task-4-blueos
state_dir="$task_dir/.local/docker-blueos"

container_ip() {
    docker inspect "$1" --format "{{(index .NetworkSettings.Networks \"$network\").IPAddress}}"
}

start_sim() {
    if docker container inspect "$sim" >/dev/null 2>&1; then
        docker start "$sim" >/dev/null 2>&1 || true
        return
    fi

    docker run -d --name "$sim" --network "$network" \
        -p 127.0.0.1:16080:6080 \
        -v "$volume:/work" \
        "$image" sleep infinity >/dev/null
}

case "${1:-}" in
    prepare)
        docker build -f "$task_dir/Ubuntu.Dockerfile" -t "$image" "$task_dir"
        docker network inspect "$network" >/dev/null 2>&1 || docker network create "$network" >/dev/null
        docker volume inspect "$volume" >/dev/null 2>&1 || docker volume create "$volume" >/dev/null
        start_sim

        COPYFILE_DISABLE=1 tar --no-xattrs -C "$repo_root" \
            --exclude='./build' \
            --exclude='./Tools/task_4/.local' \
            --exclude='./docs/bluerov-gazebo/.local' \
            --exclude='./docs/bluerov-gazebo/generated' \
            --exclude='./docs/bluerov-gazebo/output' \
            --exclude='./docs/bluerov-gazebo/embedded' \
            -cf - . | docker exec -i "$sim" tar -xf - -C /work

        docker exec "$sim" sh -lc 'git config --global --add safe.directory /work && cd /work && ./waf configure --board sitl && ./waf sub -j4 && bash docs/bluerov-gazebo/prepare_wsl.sh && cmake -S Tools/task_4 -B Tools/task_4/.local/build && cmake --build Tools/task_4/.local/build -j4'
        ;;
    start)
        if docker container inspect "$blueos" >/dev/null 2>&1; then
            echo "Blueos is already started; stop this stack before starting it again." >&2
            exit 1
        fi

        start_sim
        docker cp "$task_dir/run.sh" "$sim:/work/Tools/task_4/run.sh"
        docker cp "$task_dir/pose_summary.py" "$sim:/work/Tools/task_4/pose_summary.py"

        docker exec "$sim" test -x /work/Tools/task_4/.local/build/task_4_topside
        docker exec "$sim" test -f /work/docs/bluerov-gazebo/generated/worlds/bluerov1.sdf

        docker exec -d "$sim" sh -lc 'Xvfb :99 -screen 0 1280x800x24 -nolisten tcp > /work/Tools/task_4/.local/xvfb.log 2>&1'
        for ((i = 0; i < 50; i++)); do
            if docker exec "$sim" test -S /tmp/.X11-unix/X99; then break; fi
            sleep 0.1
        done
        docker exec "$sim" test -S /tmp/.X11-unix/X99

        docker exec -d -e DISPLAY=:99 "$sim" sh -lc 'openbox > /work/Tools/task_4/.local/openbox.log 2>&1'
        docker exec -d "$sim" sh -lc 'x11vnc -display :99 -nopw -forever -shared -rfbport 5900 -listen 127.0.0.1 > /work/Tools/task_4/.local/vnc.log 2>&1'
        docker exec -d "$sim" sh -lc 'websockify --web=/usr/share/novnc 0.0.0.0:6080 127.0.0.1:5900 > /work/Tools/task_4/.local/websockify.log 2>&1'
        docker exec -d -e DISPLAY=:99 "$sim" sh -lc 'cd /work && bash Tools/task_4/run.sh gazebo bluerov1 > Tools/task_4/.local/gazebo.log 2>&1'
        docker exec -d -e DISPLAY=:99 "$sim" sh -lc 'cd /work && bash Tools/task_4/run.sh gui bluerov1 > Tools/task_4/.local/gui.log 2>&1'
        docker exec -d "$sim" sh -lc 'cd /work && bash Tools/task_4/run.sh sitl bluerov1 > Tools/task_4/.local/sitl.log 2>&1'

        mkdir -p "$state_dir/config" "$state_dir/blueos" "$state_dir/logs" "$state_dir/userdata"
        docker run -d --name "$blueos" --network "$network" \
            -p 127.0.0.1:80:80 \
            -e BLUEOS_DISABLE_SERVICES=cable_guy,wifi,commander \
            -e BLUEOS_DISABLE_MEMORY_LIMIT=true \
            -e BLUEOS_DISABLE_STARTUP_UPDATE=true \
            -v "$state_dir/config:/root/.config" \
            -v "$state_dir/blueos:/etc/blueos" \
            -v "$state_dir/logs:/var/logs/blueos" \
            -v "$state_dir/userdata:/usr/blueos/userdata" \
            bluerobotics/blueos-core:master >/dev/null

        echo "Blueos dashboard: http://127.0.0.1/"
        echo "Gazebo gui: http://127.0.0.1:16080/vnc.html"
        echo "SITL address for the blueos master endpoint: $(container_ip "$sim"):5770"
        echo "Blueos endpoint address for the client: $(container_ip "$blueos"):14680"
        ;;
    shell)
        docker exec -it -w /work "$sim" bash
        ;;
    ip)
        echo "SITL: $(container_ip "$sim"):5770"
        echo "Blueos: $(container_ip "$blueos"):14680"
        ;;
    stop)
        docker rm -f "$blueos" "$sim" >/dev/null
        ;;
    *)
        echo "Usage: $0 {prepare|start|shell|ip|stop}" >&2
        exit 2
        ;;
esac
