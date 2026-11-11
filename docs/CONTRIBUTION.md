

For our project, the main entry point is through the package `surveillance_sim`.

It holds the `launch.py` that sets up the space for other packages containing the essential nodes.
Currently, it launches 3 separates processes.
1. gz sim (via ros_gz_sim's gz_sim.launch.py)
    loads surveillance_arena.world: walls, cameras, actor
    cameras publish on Gazebo Transport (not ROS yet)

2. parameter_bridge (package ros_gz_bridge)
    Gazebo Transport  ──►  ROS 2 topics
    /cam_pillar_north/image ► /surveillance_camera/image_raw (remapped)

Basically,
- Gazebo publishes on a Gazebo Transport topic named `/cam_pillar_north/image`. ROS 2 can’t see it.
- The bridge subscribes to that Gazebo topic and republishes each frame as a ROS 2 sensor_msgs/Image.

Gazebo Transport (gz-transport)	ROS 2
Discovery and delivery	its own implementation (ZeroMQ-based)	DDS, through an rmw layer
Message format	Protobuf (gz.msgs.Image)	ROS IDL (sensor_msgs/msg/Image)
Client library	gz-transport	rclpy / rclcpp
Tools	gz topic	ros2 topic

So, basically, the Gazebo Bridge listens to the gazebo messages (ZeroMQ based) and put them on the DDS 
so that the our packages (such as perception) can subcribe to those messages.

Gazebo (camera sensor)
   │ publishes
   ▼
Gazebo Transport (ZeroMQ)      ← Gazebo's own pub/sub, stays inside Gazebo's world
   │ subscribed to by
   ▼
ros_gz_bridge (parameter_bridge)    ← the only piece that speaks both
   │ publishes
   ▼
DDS (ROS 2)
   │ subscribed to by
   ▼
perception / yolo_node

3. yolo_surveillance (package perception)
    subscribes /surveillance_camera/image_raw
    publishes  /person_activity, /goal_pose



# Working of `ros2 launch`
## Phase 1: building the description (no processes yet)
   
`ros2 launch surveillance_sim sim_launch.py` calls the `generate_launch_description()` once.
1. We generate `CAMERA` here.
2. run the whole simulation on gz.
3. The function returns LaunchDescription([gazebo, bridge, yolo_node]).

## Phase 2: executing the actions
t=0   gazebo    → includes gz_sim.launch.py, which starts the gz sim processes
                  (server + GUI) with args "-r -v 4 <world>"
t=0   bridge    → starts parameter_bridge as a process
t=0   yolo_node → starts yolo_surveillance as a process



# Topics and Their Pub/Sub across the System
Gazebo (gz-transport, not visible to ROS)
  pubs: /cam_pillar_north/image, /cam_shed_sw/image, /cam_building_se/image

camera_bridge (ros_gz_bridge)          ← this is the real ROS publisher, not Gazebo
  pubs: /surveillance_camera/image_raw   (from Gazebo /cam_pillar_north/image)
        /cam_shed_sw/image               (no subscriber yet)
        /cam_building_se/image           (no subscriber yet)

YOLO NODE (/yolo_node)
  subs: /surveillance_camera/image_raw
  pubs: /person_activity, /goal_pose

Nobody yet subscribes to: /person_activity, /goal_pose

