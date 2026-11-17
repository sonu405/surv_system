import os
import sys
import argparse
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory

from path_planning.planner_node import GridMap, AStarPlanner, WaypointPruner
from path_planning.smoother_node import BSplineTrajectoryGenerator
from nav_msgs.msg import OccupancyGrid
import yaml


def load_yaml_map(yaml_path: str):
    with open(yaml_path, 'r') as f:
        meta = yaml.safe_load(f)
    img_rel = meta.get('image', '')
    img_path = os.path.join(os.path.dirname(yaml_path), img_rel)
    with open(img_path, 'rb') as f:
        f.readline() # P5
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.strip().split())
        f.readline()
        raw = np.frombuffer(f.read(), dtype=np.uint8).reshape((h, w))

    occ = OccupancyGrid()
    occ.info.resolution = float(meta['resolution'])
    occ.info.width = w
    occ.info.height = h
    occ.info.origin.position.x = float(meta['origin'][0])
    occ.info.origin.position.y = float(meta['origin'][1])

    flipped = np.flipud(raw)
    data = np.full((h, w), -1, dtype=np.int8)
    data[flipped == 254] = 0
    data[flipped == 0] = 100
    occ.data = [int(v) for v in data.flatten()]
    return occ, meta


def main():
    parser = argparse.ArgumentParser(description='Figure 28 Path & Smoother Visualizer (MIL Framework)')
    parser.add_argument('--start', nargs=2, type=float, default=[-8.0, -3.0], help='Start (x, y) in meters')
    parser.add_argument('--goal', nargs=2, type=float, default=[-5.0, -1.0], help='Goal (x, y) in meters')
    parser.add_argument('--output', type=str, default='path_visualization.png', help='Output image filename')
    parser.add_argument('--show', action='store_true', help='Show interactive matplotlib window')
    args = parser.parse_args()

    # Find map
    map_yaml = ''
    try:
        pkg_nav = get_package_share_directory('robot_nav')
        cand = os.path.join(pkg_nav, 'maps', 'ongoing_arena_map.yaml')
        if os.path.exists(cand):
            map_yaml = cand
    except Exception:
        pass

    if not map_yaml:
        # Search relative paths
        for root in ['.', '..', 'src', '/root/ros_ws/surv_system']:
            cand = os.path.join(root, 'src/robot_nav/maps/ongoing_arena_map.yaml')
            if os.path.exists(cand):
                map_yaml = os.path.abspath(cand)
                break

    if not map_yaml or not os.path.exists(map_yaml):
        print(f'Error: could not find ongoing_arena_map.yaml')
        sys.exit(1)

    print(f'Loading map from: {map_yaml}')
    occ, meta = load_yaml_map(map_yaml)
    grid_map = GridMap(occ, robot_radius=0.05)

    start_cell = grid_map.world_to_cell(args.start[0], args.start[1])
    goal_cell = grid_map.world_to_cell(args.goal[0], args.goal[1])

    if not grid_map.is_free(*start_cell):
        start_cell = grid_map.find_nearest_free(*start_cell)
    if not grid_map.is_free(*goal_cell):
        goal_cell = grid_map.find_nearest_free(*goal_cell)

    print(f'Planning from cell {start_cell} ({args.start}) to {goal_cell} ({args.goal})...')
    planner = AStarPlanner(grid_map)
    raw_cells = planner.plan(start_cell, goal_cell)
    if not raw_cells:
        print('A* failed to find a path!')
        sys.exit(1)

    pruner = WaypointPruner(grid_map, safety_margin=0)
    pruned_cells = pruner.prune(raw_cells)

    raw_world = np.array([grid_map.cell_to_world(ci, cj) for ci, cj in raw_cells])
    pruned_world = np.array([grid_map.cell_to_world(ci, cj) for ci, cj in pruned_cells])

    # B-Spline Smoothing
    smoother = BSplineTrajectoryGenerator(degree=3, num_samples=200)
    smooth_traj = smoother.generate(pruned_world)

    print(f'A* Raw: {len(raw_cells)} cells | LOS Pruned: {len(pruned_cells)} wps | Smoothed: {len(smooth_traj)} pts')

    # Create Figure 28 style plot
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8), gridspec_kw={'width_ratios': [1.2, 1]})
    fig.suptitle('Autonomous Surveillance System: A* Path Planning & B-Spline Smoothing\n(MIL Framework Figure 28 Implementation)', fontsize=14, fontweight='bold')

    # Panel 1: Full Arena Map with Paths
    ox = float(meta['origin'][0])
    oy = float(meta['origin'][1])
    res = float(meta['resolution'])
    extent = [ox, ox + grid_map.W * res, oy, oy + grid_map.H * res]

    # Display map (0 = free/white, 100 = occupied/black)
    ax1.imshow(grid_map.grid.T, cmap='Greys', origin='lower', extent=extent, alpha=0.6)

    # Raw A* Path (MIL Listing 3: solid grey)
    ax1.plot(raw_world[:, 0], raw_world[:, 1], color='#7f7f7f', linestyle='-', linewidth=1.5, label=f'Raw A* Path ({len(raw_cells)} cells)')

    # Pruned Waypoints (MIL Listing 5: orange dashed line + diamonds)
    ax1.plot(pruned_world[:, 0], pruned_world[:, 1], color='#ff7f0e', linestyle='--', linewidth=2.0,
             marker='D', markersize=6, markerfacecolor='#ff7f0e', markeredgecolor='black',
             label=f'LOS Pruned Waypoints ({len(pruned_cells)} wps)')

    # Final Smooth B-Spline Trajectory (MIL Listing 7: solid blue)
    ax1.plot(smooth_traj[:, 0], smooth_traj[:, 1], color='#1f77b4', linestyle='-', linewidth=2.8,
             label=f'Smooth B-Spline ({len(smooth_traj)} samples)')

    # Start & Goal markers
    ax1.plot(args.start[0], args.start[1], 's', color='#2ca02c', markersize=10, markeredgecolor='black', label=f'Start ({args.start[0]}, {args.start[1]})')
    ax1.plot(args.goal[0], args.goal[1], '*', color='#d62728', markersize=14, markeredgecolor='black', label=f'Goal ({args.goal[0]}, {args.goal[1]})')

    ax1.set_title('Arena Overview (Occupancy Grid & Trajectory)', fontsize=12)
    ax1.set_xlabel('X [meters]')
    ax1.set_ylabel('Y [meters]')
    ax1.legend(loc='upper right', framealpha=0.9)
    ax1.grid(True, linestyle=':', alpha=0.5)

    # Panel 2: Zoomed-in Comparison & Heading Profile
    # Zoom in around the trajectory bounding box with a margin
    min_x, max_x = np.min(smooth_traj[:, 0]) - 1.0, np.max(smooth_traj[:, 0]) + 1.0
    min_y, max_y = np.min(smooth_traj[:, 1]) - 1.0, np.max(smooth_traj[:, 1]) + 1.0

    ax2.imshow(grid_map.grid.T, cmap='Greys', origin='lower', extent=extent, alpha=0.5)
    ax2.plot(raw_world[:, 0], raw_world[:, 1], color='#7f7f7f', linestyle='-', linewidth=1.5, label='Raw A* (staircase cells)')
    ax2.plot(pruned_world[:, 0], pruned_world[:, 1], color='#ff7f0e', linestyle='--', linewidth=2.0, marker='D', markersize=7, label='Pruned LOS segments')
    ax2.plot(smooth_traj[:, 0], smooth_traj[:, 1], color='#1f77b4', linestyle='-', linewidth=3.0, label='Cubic B-Spline ($C^2$ continuous)')
    ax2.plot(args.start[0], args.start[1], 's', color='#2ca02c', markersize=10, markeredgecolor='black')
    ax2.plot(args.goal[0], args.goal[1], '*', color='#d62728', markersize=14, markeredgecolor='black')

    # Draw heading quiver arrows along the B-spline
    stride = max(1, len(smooth_traj) // 15)
    ax2.quiver(smooth_traj[::stride, 0], smooth_traj[::stride, 1],
               np.cos(smooth_traj[::stride, 2]), np.sin(smooth_traj[::stride, 2]),
               color='navy', scale=25, width=0.005, alpha=0.7, label='Heading vectors $(\\theta)$')

    ax2.set_xlim(min_x, max_x)
    ax2.set_ylim(min_y, max_y)
    ax2.set_title('Detailed Zoom: Curvature & Heading Vectors', fontsize=12)
    ax2.set_xlabel('X [meters]')
    ax2.set_ylabel('Y [meters]')
    ax2.legend(loc='lower left', framealpha=0.9)
    ax2.grid(True, linestyle=':', alpha=0.6)

    plt.tight_layout()
    plt.savefig(args.output, dpi=200)
    print(f'Visualization successfully saved to: {os.path.abspath(args.output)}')

    if args.show:
        plt.show()


if __name__ == '__main__':
    main()
