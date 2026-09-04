#!/usr/bin/env bash
# Build the ROS 2 Jazzy image used to run this repo on a non-24.04 machine.
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
docker build \
    --build-arg UID="$(id -u)" \
    --build-arg GID="$(id -g)" \
    -t irm:jazzy \
    "$HERE"
echo
echo "built 'irm:jazzy'.  run it with:  $HERE/run.sh"
