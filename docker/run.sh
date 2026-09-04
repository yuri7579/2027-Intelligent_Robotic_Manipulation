#!/usr/bin/env bash
# Run a command (default: bash) inside the ROS 2 Jazzy container with the repo
# mounted at /workspace and the host X display + GPU passed through.
#
#   ./docker/run.sh                                  # interactive shell
#   ./docker/run.sh python3 omy_teleop/omyTeleop.py  # run the assignment
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(dirname "$HERE")"

# let the container talk to the host X server (RViz / the slider window)
if command -v xhost >/dev/null 2>&1 && [ -n "$DISPLAY" ]; then
    xhost +local:docker >/dev/null
fi

docker run --rm -it \
    --name irm_jazzy \
    --ipc=host \
    --network=host \
    -e DISPLAY="$DISPLAY" \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v "$REPO:/workspace:rw" \
    irm:jazzy "$@"
