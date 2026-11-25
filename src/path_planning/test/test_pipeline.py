import time
import unittest
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

from path_planning.planner_node import PlannerNode
from path_planning.smoother_node import SmootherNode


class TestPlanningPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_full_pipeline(self):
        test_node = Node('test_pipeline_tester')
        planner = PlannerNode()
        smoother = SmootherNode()

        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        map_pub = test_node.create_publisher(OccupancyGrid, '/map', map_qos)
        amcl_pub = test_node.create_publisher(PoseWithCovarianceStamped, '/amcl_pose', 10)
        goal_pub = test_node.create_publisher(PoseStamped, '/goal_pose', 10)

        received_planned = []
        received_smoothed = []

        test_node.create_subscription(
            Path, '/planned_path', lambda m: received_planned.append(m), 10)
        test_node.create_subscription(
            Path, '/smoothed_path', lambda m: received_smoothed.append(m), 10)

        # 1. Publish Map
        occ = OccupancyGrid()
        occ.header.frame_id = 'map'
        occ.info.resolution = 0.2
        occ.info.width = 50
        occ.info.height = 50
        occ.info.origin.position.x = 0.0
        occ.info.origin.position.y = 0.0
        grid = np.zeros((50, 50), dtype=np.int8)
        grid[20:30, 20:30] = 100
        occ.data = [int(v) for v in grid.flatten()]
        map_pub.publish(occ)

        # Spin to process map
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(test_node)
        executor.add_node(planner)
        executor.add_node(smoother)

        for _ in range(5):
            executor.spin_once(timeout_sec=0.1)

        # 2. Publish Robot Pose
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.position.x = 2.0
        pose_msg.pose.pose.position.y = 2.0
        pose_msg.pose.pose.orientation.w = 1.0
        amcl_pub.publish(pose_msg)

        for _ in range(5):
            executor.spin_once(timeout_sec=0.1)

        # 3. Publish Goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = 8.0
        goal_msg.pose.position.y = 8.0
        goal_msg.pose.orientation.w = 1.0
        goal_pub.publish(goal_msg)

        # Wait for path and smoothed path
        start_time = time.time()
        while time.time() - start_time < 3.0:
            executor.spin_once(timeout_sec=0.1)
            if received_planned and received_smoothed:
                break

        self.assertTrue(len(received_planned) > 0, 'Did not receive /planned_path')
        self.assertTrue(len(received_smoothed) > 0, 'Did not receive /smoothed_path')
        self.assertEqual(len(received_smoothed[0].poses), 200, 'Smoothed path does not have 200 samples')

        test_node.destroy_node()
        planner.destroy_node()
        smoother.destroy_node()


if __name__ == '__main__':
    unittest.main()
