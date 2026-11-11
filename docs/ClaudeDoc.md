# ClaudeDoc — surv_system Project Reference

> **Purpose**: Persistent context document for LLM agents working on this codebase.
> Update this file after every change session.

---

## Project Overview

A **ROS 2 (Lyrical / Gazebo-classic)** surveillance system that:
1. Simulates a walled arena with buildings, pillars, and 3 surveillance cameras.
2. Spawns a scripted walking-person actor.
3. Runs YOLOv8n person detection on camera feeds and classifies activity
   (Staying / Walking / Running) from pixel velocity.
4. Publishes a navigation goal when a person stays stationary for ≥ 5 seconds.

## Repository Layout

```
surv_system/
├── setup.sh                  # First-time bootstrap (venv + deps + build)
├── env.sh                    # Source for returning sessions
├── requirements.txt          # Pinned Python deps (PyTorch CPU, Ultralytics, Polars…)
├── ChangeLog.md              # Change history
├── ClaudeDoc.md              # ← This file
│
└── src/
    ├── perception/                     # ROS 2 ament_python package
    │   ├── package.xml
    │   ├── setup.py                    # entry_point: yolo_surveillance → perception.yolo_node:main
    │   └── perception/
    │       └── yolo_node.py            # YoloNode — detection + activity + goal publisher
    │
    └── surveillance_sim/               # ROS 2 ament_python package
        ├── package.xml
        ├── setup.py                    # installs launch/ and worlds/ via data_files
        ├── launch/
        │   └── sim_launch.py           # Starts Gazebo world + yolo_surveillance node
        ├── scripts/
        │   └── generate_world.py       # Procedural SDF world generator
        └── worlds/
            └── surveillance_arena.world
```

## ROS 2 Topics & Interfaces

| Topic | Type | Direction | Description |
|---|---|---|---|
| `/surveillance_camera/image_raw` | `sensor_msgs/Image` | Sub (yolo_node) | Camera feed from Gazebo |
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | Sub (yolo_node) | Model positions (callback is a no-op placeholder) |
| `/person_activity` | `std_msgs/String` | Pub (yolo_node) | "Staying", "Walking", or "Running" |
| `/goal_pose` | `geometry_msgs/PoseStamped` | Pub (yolo_node) | Navigation goal when person stays ≥ 5 s |

### Gazebo Camera Topics (under `/surveillance/` namespace)

Each camera publishes on `/surveillance/<cam_name>/image_raw`:
- `cam_pillar_north`
- `cam_shed_sw`
- `cam_building_se`

## Known Issues / TODOs

1. **Topic mismatch** — `yolo_node.py` subscribes to `/surveillance_camera/image_raw`
   but the Gazebo cameras publish under `/surveillance/<cam_name>/image_raw`.
   Needs remapping or multi-camera subscription.
2. **Hardcoded goal pose** — `send_goal_to_person()` always sends `(-14.0, 0.5)`
   regardless of detected position. `model_states_callback` is a no-op.
3. **Single-person / single-camera** — The node processes only the first detected
   person (`break` on line 144) and subscribes to one camera topic despite 3 cameras.

## Build & Run

```bash
# First time
./setup.sh

# Returning
source env.sh
colcon build --symlink-install
source install/setup.bash

# Launch
ros2 launch surveillance_sim sim_launch.py
```

## Last Updated

2026-11-10 — Fixed entry points, data_files installation, and setup.sh.
