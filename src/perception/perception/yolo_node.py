import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
import math

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class YoloNode(Node):

    def __init__(self):
        super().__init__('yolo_node')

        self.bridge = CvBridge()
        self.declare_parameter('model_path', 'models/yolov8n.pt')
        self.model = YOLO(self.get_parameter('model_path').value)

        # All times below are in SIM seconds, taken from msg.header.stamp.
        # Rate limiter: process ~6 frames per sim-second
        self.last_process_time = None
        self.process_interval = 0.15

        # Pixel velocity tracking
        self.prev_center = None
        self.prev_time = None

        # Staying detection state
        self.stay_start_time = None
        self.goal_sent = False
        self.last_goal_time = None
        self.goal_cooldown = 15.0  # sim seconds between goals

        # Fixed map-frame goal for the detected person
        self.target_x = -14.0
        self.target_y = 0.50

        # ---------- Subscribers ----------
        self.create_subscription(
            Image,
            '/surveillance_camera/image_raw',  # bridged from Gazebo by ros_gz_bridge
            self.image_callback,
            1)

        # ---------- Publishers ----------
        self.goal_pub = self.create_publisher(
            PoseStamped, '/goal_pose', 10)
        self.activity_pub = self.create_publisher(
            String, '/person_activity', 10)

        cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)
        self.get_logger().info("YOLO + Staying Detection READY")

    def classify_activity(self, velocity):
        """
        Classify a person's activity from their pixel velocity (px per sim-second).
        Thresholds are tuned for 640x640 frames processed at ~6 FPS.
        """
        if velocity < 20:
            return "Staying"
        elif velocity < 150:
            return "Walking"
        else:
            return "Running"

    def reset_tracking(self):
        """Forget all motion state (used when sim time jumps backwards)."""
        self.prev_center = None
        self.prev_time = None
        self.stay_start_time = None
        self.goal_sent = False
        self.last_goal_time = None

    def image_callback(self, msg):
        # Sim time of this frame, in seconds
        now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Sim was reset (time went backwards): drop stale state
        if self.last_process_time is not None and now < self.last_process_time:
            self.reset_tracking()
            self.last_process_time = None

        # Skip frames to maintain ~6 FPS processing rate
        if (self.last_process_time is not None
                and now - self.last_process_time < self.process_interval):
            return
        self.last_process_time = now

        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Run YOLOv8n - detect objects with confidence >= 0.3
        results = self.model(frame, conf=0.3, imgsz=640,
                             verbose=False)

        for r in results:
            if r.boxes is None:
                continue
            for box in r.boxes:
                if int(box.cls[0]) != 0:  # class 0 = person
                    continue

                # Bounding box centre
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)

                # Compute pixel velocity (px per sim-second)
                velocity = 0.0
                if self.prev_center:
                    dx = cx - self.prev_center[0]
                    dy = cy - self.prev_center[1]
                    dt = now - self.prev_time
                    if dt > 0:
                        velocity = math.sqrt(dx * dx + dy * dy) / dt

                self.prev_center = (cx, cy)
                self.prev_time = now

                activity = self.classify_activity(velocity)

                # Publish activity label
                self.activity_pub.publish(String(data=activity))

                # ---------- Staying logic ----------
                if activity == "Staying":
                    if self.stay_start_time is None:
                        self.stay_start_time = now
                    elif (now - self.stay_start_time >= 5.0
                          and not self.goal_sent):
                        self.send_goal_to_person(now)  # TRIGGER!
                else:
                    # Person moved - reset timer
                    self.stay_start_time = None
                    self.goal_sent = False

                # ---------- Draw on frame ----------
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(frame, activity,
                            (x1, y1 - 25),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.6, (0, 255, 255), 2)
                cv2.putText(frame, f"v={velocity:.1f} px/s",
                            (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (255, 255, 0), 2)
                break  # process only the first detected person

        cv2.imshow("YOLO Detection", frame)
        cv2.waitKey(1)

    def send_goal_to_person(self, now):
        # Enforce cooldown between consecutive goals (sim time)
        if (self.last_goal_time is not None
                and now - self.last_goal_time < self.goal_cooldown):
            return

        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        # Fixed coordinates of the suspicious person's position
        goal.pose.position.x = self.target_x  # -14.0
        goal.pose.position.y = self.target_y  # 0.5
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)

        self.last_goal_time = now
        self.goal_sent = True

        self.get_logger().info(
            f"GOAL sent: ({self.target_x}, {self.target_y})")


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()