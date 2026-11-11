#!/usr/bin/env bash
# First-time setup: creates venv, installs deps, and builds the workspace.

set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
source /opt/ros/lyrical/setup.bash

# --system-site-packages keeps the system ROS packages (rclpy, cv_bridge) visible
python3 -m venv --system-site-packages .venv
source .venv/bin/activate

pip install -r requirements.txt --extra-index-url https://download.pytorch.org/whl/cpu

# --ignore-installed forces a copy into the venv even if the system already has colcon.
# Without it, pip skips the install and the build below uses the system colcon.
pip install --ignore-installed colcon-common-extensions
hash -r

python3 src/surveillance_sim/scripts/generate_world.py
colcon build --symlink-install
source install/setup.bash

# Guard: the node launcher must run under the venv's Python, or it can't import ultralytics
if ! head -1 install/perception/lib/perception/yolo_surveillance | grep -q "\.venv"; then
    echo "ERROR: yolo_surveillance does not point at .venv; the build used the wrong colcon." >&2
    exit 1
fi

echo ""
echo "Setup complete. In future sessions run:  source env.sh"