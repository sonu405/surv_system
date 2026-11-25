import os
import heapq
import yaml
import numpy as np
from scipy.ndimage import binary_dilation, generate_binary_structure
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf2_ros


class GridMap:
    MOVES_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    MOVES_8 = MOVES_4 + [(-1, -1), (-1, 1), (1, -1), (1, 1)]

    def __init__(self, occ_msg: OccupancyGrid, robot_radius: float = 0.22):
        self.resolution = occ_msg.info.resolution
        self.origin_x = occ_msg.info.origin.position.x
        self.origin_y = occ_msg.info.origin.position.y
        self.W = occ_msg.info.width
        self.H = occ_msg.info.height
        self.cell_size = self.resolution

        raw_data = np.array(occ_msg.data, dtype=np.int8).reshape((self.H, self.W))
        # Transpose so index is [x_cell, y_cell]
        raw_grid = raw_data.T

        # Only actual obstacles (occupancy > 50) are blocked.
        # Unknown cells (-1) are treated as free for planning.
        # Standard ROS occupancy grid: 0 = free, 100 = occupied, -1 = unknown.
        # Unknown cells (-1) outside the arena perimeter walls are marked untraversable,
        # ensuring all planned trajectories stay strictly within the arena boundaries.
        binary_occupied = (raw_grid != 0)

        r_cells = int(np.ceil(robot_radius / self.cell_size))
        struct = generate_binary_structure(2, 2)
        inflated = binary_dilation(binary_occupied, structure=struct, iterations=r_cells)
        self.grid = inflated.astype(np.uint8)

    def is_free(self, i: int, j: int) -> bool:
        return (0 <= i < self.W and 0 <= j < self.H and self.grid[i, j] == 0)

    def find_nearest_free(self, i: int, j: int, max_radius: int = 25):
        if self.is_free(i, j):
            return i, j
        for r in range(1, max_radius + 1):
            for di in range(-r, r + 1):
                for dj in range(-r, r + 1):
                    if max(abs(di), abs(dj)) == r:
                        ni, nj = i + di, j + dj
                        if self.is_free(ni, nj):
                            return ni, nj
        return None

    def neighbors(self, i: int, j: int, connectivity: int = 8):
        moves = self.MOVES_8 if connectivity == 8 else self.MOVES_4
        for di, dj in moves:
            ni, nj = i + di, j + dj
            if self.is_free(ni, nj):
                cost = self.cell_size * (np.sqrt(2) if di != 0 and dj != 0 else 1.0)
                yield (ni, nj), cost

    def world_to_cell(self, x: float, y: float):
        i = int(np.floor((x - self.origin_x) / self.resolution))
        j = int(np.floor((y - self.origin_y) / self.resolution))
        return i, j

    def cell_to_world(self, i: int, j: int):
        x = self.origin_x + (i + 0.5) * self.resolution
        y = self.origin_y + (j + 0.5) * self.resolution
        return x, y


class AStarPlanner:
    def __init__(self, grid_map: GridMap):
        self.map = grid_map

    def _octile(self, a: tuple, b: tuple) -> float:
        dx = abs(a[0] - b[0])
        dy = abs(a[1] - b[1])
        return self.map.cell_size * (max(dx, dy) + (np.sqrt(2) - 1.0) * min(dx, dy))

    def _reconstruct(self, parent, start, goal):
        path, node = [], goal
        while node != start:
            path.append(node)
            node = tuple(parent[node])
        path.append(start)
        path.reverse()
        return path

    def plan(self, start: tuple, goal: tuple):
        W, H = self.map.W, self.map.H
        INF = float('inf')
        g_cost = np.full((W, H), INF, dtype=np.float64)
        parent = np.full((W, H, 2), -1, dtype=np.int32)
        closed = np.zeros((W, H), dtype=bool)

        g_cost[start] = 0.0
        open_heap = [(self._octile(start, goal), 0.0, start)]

        while open_heap:
            f_cur, g_cur, node = heapq.heappop(open_heap)
            if closed[node]:
                continue
            closed[node] = True

            if node == goal:
                return self._reconstruct(parent, start, goal)

            for (ni, nj), cost in self.map.neighbors(*node):
                if closed[ni, nj]:
                    continue
                g_new = g_cur + cost
                if g_new < g_cost[ni, nj]:
                    g_cost[ni, nj] = g_new
                    parent[ni, nj] = node
                    f_new = g_new + self._octile((ni, nj), goal)
                    heapq.heappush(open_heap, (f_new, g_new, (ni, nj)))

        return None


class WaypointPruner:
    def __init__(self, grid_map: GridMap, safety_margin: int = 1):
        self.map = grid_map
        self.margin = safety_margin

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        x, y = x0, y0
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1

        if dx >= dy:
            err = dx // 2
            while x != x1:
                cells.append((x, y))
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy // 2
            while y != y1:
                cells.append((x, y))
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy
        cells.append((x1, y1))
        return cells

    def _los(self, a: tuple, b: tuple) -> bool:
        for ci, cj in self._bresenham(a[0], a[1], b[0], b[1]):
            for di in range(-self.margin, self.margin + 1):
                for dj in range(-self.margin, self.margin + 1):
                    if not self.map.is_free(ci + di, cj + dj):
                        return False
        return True

    def prune(self, path: list) -> list:
        if len(path) <= 2:
            return path
        pruned = [path[0]]
        i = 0
        while i < len(path) - 1:
            j = len(path) - 1
            while j > i + 1:
                if self._los(path[i], path[j]):
                    break
                j -= 1
            pruned.append(path[j])
            i = j
        return pruned


class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')

        self.declare_parameter('robot_radius', 0.05)
        self.declare_parameter('safety_margin', 0)
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('default_start_x', 0.0)
        self.declare_parameter('default_start_y', 0.0)
        self.declare_parameter('map_yaml', '')

        self.robot_radius = self.get_parameter('robot_radius').value
        self.safety_margin = self.get_parameter('safety_margin').value
        self.default_start_x = self.get_parameter('default_start_x').value
        self.default_start_y = self.get_parameter('default_start_y').value
        goal_topic = self.get_parameter('goal_topic').value
        map_yaml = self.get_parameter('map_yaml').value

        self.grid_map = None
        self.current_pose = None
        self.fallback_occ = None
        self.has_external_map = False

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_pub = self.create_publisher(OccupancyGrid, '/map', map_qos)
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, map_qos)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, 10)
        self.initial_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initial_pose_callback, 10)

        self.goal_sub = self.create_subscription(
            PoseStamped, goal_topic, self.goal_callback, 10)
        # Also subscribe to /yolo_pose in case groupmate sends there
        if goal_topic != '/yolo_pose':
            self.create_subscription(PoseStamped, '/yolo_pose', self.goal_callback, 10)

        self.raw_path_pub = self.create_publisher(Path, '/raw_path', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Try to load pre-built map as fallback if available
        if not map_yaml:
            try:
                pkg_nav = get_package_share_directory('robot_nav')
                default_yaml = os.path.join(pkg_nav, 'maps', 'ongoing_arena_map.yaml')
                if os.path.exists(default_yaml):
                    map_yaml = default_yaml
            except Exception:
                pass

        if map_yaml and os.path.exists(map_yaml):
            self.load_map_from_yaml(map_yaml)

        self.map_timer = self.create_timer(2.0, self._publish_fallback_map)

        self.get_logger().info('PlannerNode (A* + LOS Pruning) initialized')

    def _publish_fallback_map(self):
        if not self.has_external_map and self.fallback_occ is not None:
            self.fallback_occ.header.stamp = self.get_clock().now().to_msg()
            self.map_pub.publish(self.fallback_occ)

    def load_map_from_yaml(self, yaml_path: str):
        try:
            with open(yaml_path, 'r') as f:
                meta = yaml.safe_load(f)
            img_rel = meta.get('image', '')
            img_path = os.path.join(os.path.dirname(yaml_path), img_rel)
            if not os.path.exists(img_path):
                return
            with open(img_path, 'rb') as f:
                f.readline() # P5
                line = f.readline()
                while line.startswith(b'#'):
                    line = f.readline()
                w, h = map(int, line.strip().split())
                f.readline() # maxval
                raw = np.frombuffer(f.read(), dtype=np.uint8).reshape((h, w))

            occ = OccupancyGrid()
            occ.header.frame_id = 'map'
            occ.header.stamp = self.get_clock().now().to_msg()
            occ.info.resolution = float(meta['resolution'])
            occ.info.width = w
            occ.info.height = h
            occ.info.origin.position.x = float(meta['origin'][0])
            occ.info.origin.position.y = float(meta['origin'][1])
            occ.info.origin.orientation.w = 1.0

            # In ROS OccupancyGrid, row 0 is bottom (y=origin_y), but in image row 0 is top
            flipped = np.flipud(raw)
            # ROS mapping: free (254) -> 0, occ (0) -> 100, unknown (205) -> -1
            data = np.full((h, w), -1, dtype=np.int8)
            data[flipped == 254] = 0
            data[flipped == 0] = 100
            occ.data = [int(v) for v in data.flatten()]

            self.fallback_occ = occ
            self.grid_map = GridMap(occ, robot_radius=self.robot_radius)
            self.get_logger().info(f'Loaded default map from {yaml_path}: {w}x{h} cells')
            self.map_pub.publish(occ)
        except Exception as e:
            self.get_logger().warning(f'Failed to load default map from YAML: {e}')

    def map_callback(self, msg: OccupancyGrid):
        # Ignore fallback map published by this node
        if self.fallback_occ is not None and len(msg.data) == len(self.fallback_occ.data):
            if abs(msg.info.origin.position.x - self.fallback_occ.info.origin.position.x) < 1e-4:
                return
        self.has_external_map = True
        self.grid_map = GridMap(msg, robot_radius=self.robot_radius)
        self.get_logger().info(f'Received live map: {self.grid_map.W}x{self.grid_map.H} cells, resolution: {self.grid_map.resolution}m')

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def initial_pose_callback(self, msg: PoseWithCovarianceStamped):
        self.current_pose = (msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.get_logger().info(f'Start pose updated from RViz 2D Pose Estimate: [{self.current_pose[0]:.2f}, {self.current_pose[1]:.2f}]')

    def get_robot_pose(self):
        if self.current_pose is not None:
            return self.current_pose
        try:
            t = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return (t.transform.translation.x, t.transform.translation.y)
        except Exception:
            pass
        self.get_logger().info(f'No AMCL/TF pose; using default start: [{self.default_start_x:.2f}, {self.default_start_y:.2f}]')
        return (self.default_start_x, self.default_start_y)

    def goal_callback(self, msg: PoseStamped):
        if self.grid_map is None:
            self.get_logger().warning('No map received yet. Cannot plan.')
            return

        start_xy = self.get_robot_pose()
        if start_xy is None:
            self.get_logger().warning('No robot pose available from AMCL or TF. Cannot plan.')
            return

        goal_xy = (msg.pose.position.x, msg.pose.position.y)
        self.get_logger().info(f'Planning: [{start_xy[0]:.2f}, {start_xy[1]:.2f}] -> [{goal_xy[0]:.2f}, {goal_xy[1]:.2f}]')

        start_cell = self.grid_map.world_to_cell(*start_xy)
        goal_cell = self.grid_map.world_to_cell(*goal_xy)

        if not self.grid_map.is_free(*start_cell):
            free_start = self.grid_map.find_nearest_free(*start_cell)
            if free_start is None:
                self.get_logger().error(f'Start cell {start_cell} is occupied and no free neighbor found.')
                return
            start_cell = free_start

        if not self.grid_map.is_free(*goal_cell):
            free_goal = self.grid_map.find_nearest_free(*goal_cell)
            if free_goal is None:
                self.get_logger().error(f'Goal cell {goal_cell} is occupied and no free neighbor found.')
                return
            goal_cell = free_goal

        planner = AStarPlanner(self.grid_map)
        raw_path = planner.plan(start_cell, goal_cell)

        if not raw_path:
            self.get_logger().error('A* failed to find a path.')
            return

        # Publish raw grid path for visualization/debugging
        raw_msg = Path()
        raw_msg.header.frame_id = 'map'
        raw_msg.header.stamp = self.get_clock().now().to_msg()
        for ci, cj in raw_path:
            wx, wy = self.grid_map.cell_to_world(ci, cj)
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            raw_msg.poses.append(ps)
        self.raw_path_pub.publish(raw_msg)

        pruner = WaypointPruner(self.grid_map, safety_margin=self.safety_margin)
        pruned_path = pruner.prune(raw_path)

        self.get_logger().info(f'Path planned: {len(raw_path)} cells -> {len(pruned_path)} pruned waypoints')

        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()

        for ci, cj in pruned_path:
            wx, wy = self.grid_map.cell_to_world(ci, cj)
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)
        self.get_logger().info(f'Published /planned_path with {len(path_msg.poses)} waypoints')


def main(args=None):
    rclpy.init(args=args)
    node = PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
