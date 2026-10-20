#!/usr/bin/env bash

# TODO: Ensure this works correctly before pushing.

set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
source /opt/ros/lyrical/setup.bash
python3 -m venv --system-site-packages .venv
source .venv/bin/activate
pip install -r requirements.txt --extra-index-url https://download.pytorch.org/whl/cpu
