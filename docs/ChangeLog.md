# Changelog

All notable changes to this project will be documented in this file.

## [Unreleased] – 2026-11-10

### Fixed
- **`perception/setup.py`**: Added missing `console_scripts` entry point
  `yolo_surveillance = perception.yolo_node:main`. Without this the launch file
  and `ros2 run perception yolo_surveillance` would fail at runtime.
- **`surveillance_sim/setup.py`**: Added `data_files` entries for `launch/*.py`
  and `worlds/*.world` so that `ros2 launch surveillance_sim sim_launch.py` can
  locate these files after `colcon build`.
- **`setup.sh`**: Removed stale `TODO` comment, added `colcon build --symlink-install`
  and `source install/setup.bash` so the workspace is fully usable after first-time
  setup. Added a completion message directing users to `source env.sh` for
  subsequent sessions.
