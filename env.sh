#!/usr/bin/env bash
# Usage: source env.sh [IMPORTANT -- simple ./env.sh wouldn't work]
_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source /opt/ros/lyrical/setup.bash
source "$_DIR/.venv/bin/activate"
# Only exists after a colcon build
[ -f "$_DIR/install/setup.bash" ] && source "$_DIR/install/setup.bash"
