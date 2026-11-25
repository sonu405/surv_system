import numpy as np
from scipy.interpolate import splprep, splev

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class BSplineTrajectoryGenerator:
    def __init__(self, degree: int = 3, num_samples: int = 200, resample_step: float = 0.25):
        self.degree = degree
        self.num_samples = num_samples
        self.resample_step = resample_step

    def generate(self, waypoints: np.ndarray) -> np.ndarray:
        if len(waypoints) < 2:
            return np.hstack([waypoints, np.zeros((len(waypoints), 1))])

        wps = np.array(waypoints, dtype=float)

        # Compute cumulative distance along piecewise linear path
        dists = np.sqrt(np.sum(np.diff(wps, axis=0)**2, axis=1))
        cum_dist = np.insert(np.cumsum(dists), 0, 0.0)
        total_dist = cum_dist[-1]

        if total_dist < 1e-3:
            pts = np.repeat(wps[:1], self.num_samples, axis=0)
            return np.hstack([pts, np.zeros((self.num_samples, 1))])

        # Resample uniformly along arc-length to eliminate polynomial overshoot (Runge phenomenon)
        step = self.resample_step if self.resample_step > 0 else 0.25
        num_pts = max(int(np.ceil(total_dist / step)), self.degree + 2)
        query_dists = np.linspace(0.0, total_dist, num_pts)
        resampled_x = np.interp(query_dists, cum_dist, wps[:, 0])
        resampled_y = np.interp(query_dists, cum_dist, wps[:, 1])
        resampled = np.column_stack([resampled_x, resampled_y])

        k = min(self.degree, len(resampled) - 1)
        # Small smoothing factor s ensures curve follows path tightly without sharp corner whipping
        tck, u = splprep([resampled[:, 0], resampled[:, 1]], k=k, s=0.01)

        u_fine = np.linspace(0.0, 1.0, self.num_samples)
        x_fine, y_fine = splev(u_fine, tck)
        pts = np.column_stack([x_fine, y_fine])

        dx, dy = splev(u_fine, tck, der=1)
        theta = np.arctan2(dy, dx)

        return np.column_stack([pts, theta])


class SmootherNode(Node):
    def __init__(self):
        super().__init__('smoother_node')

        self.declare_parameter('degree', 3)
        self.declare_parameter('num_samples', 200)

        degree = self.get_parameter('degree').value
        num_samples = self.get_parameter('num_samples').value

        self.generator = BSplineTrajectoryGenerator(degree=degree, num_samples=num_samples)

        self.sub_path = self.create_subscription(
            Path, '/planned_path', self.path_callback, 10)
        self.pub_smoothed_path = self.create_publisher(
            Path, '/smoothed_path', 10)

        self.get_logger().info(f'SmootherNode (Cubic B-Spline, {num_samples} samples) initialized')

    def path_callback(self, msg: Path):
        if not msg.poses:
            self.get_logger().warning('Received empty planned path.')
            return

        points = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        self.get_logger().info(f'Smoothing path with {len(points)} waypoints...')

        try:
            traj = self.generator.generate(points)
        except Exception as e:
            self.get_logger().error(f'B-Spline smoothing failed: {e}')
            return

        smoothed_msg = Path()
        smoothed_msg.header = msg.header
        smoothed_msg.header.stamp = self.get_clock().now().to_msg()

        for x, y, yaw in traj:
            ps = PoseStamped()
            ps.header.frame_id = msg.header.frame_id
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.position.z = 0.0
            # Yaw to quaternion (rotation around z-axis)
            ps.pose.orientation.z = float(np.sin(yaw / 2.0))
            ps.pose.orientation.w = float(np.cos(yaw / 2.0))
            smoothed_msg.poses.append(ps)

        self.pub_smoothed_path.publish(smoothed_msg)
        self.get_logger().info(f'Published /smoothed_path with {len(smoothed_msg.poses)} points')


def main(args=None):
    rclpy.init(args=args)
    node = SmootherNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
