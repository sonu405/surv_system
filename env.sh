#!/usr/bin/env bash
# Usage: source env.sh [IMPORTANT -- simple ./env.sh wouldn't work]
_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/lyrical/setup.bash

if [ ! -f "$_DIR/.venv/bin/activate" ]; then
    echo "env.sh: no .venv found. Run ./setup.sh first." >&2
    return 1
fi
source "$_DIR/.venv/bin/activate"
hash -r

# Building with the system colcon bakes the wrong Python into the nodes
if [ "$(command -v colcon)" != "$_DIR/.venv/bin/colcon" ]; then
    echo "env.sh: WARNING: colcon is not the venv's. Run: pip install --ignore-installed colcon-common-extensions" >&2
fi

# Only exists after a colcon build
[ -f "$_DIR/install/setup.bash" ] && source "$_DIR/install/setup.bash"