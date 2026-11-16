

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


Note however that `sim_launch.py` is only the launch file for simulation and YOLO only. For mapping or navigation, 
we use a different `launch.py`.


# Working of `ros2 launch`
## Phase 1: building the description (no processes yet)

Ros2 running convention
```
ros2 run <package> <executable> --ros-args -p name:=value ...
```

`ros2 launch surveillance_sim sim_launch.py` calls the `generate_launch_description()` once.
1. We generate `CAMERA` here.
2. run the whole simulation on gz.
3. The function returns LaunchDescription([gazebo, bridge, yolo_node]).

## Phase 2: executing the actions
t=0   gazebo    → includes gz_sim.launch.py, which starts the gz sim processes
                  (server + GUI) with args "-r -v 4 <world>"
t=0   bridge    → starts parameter_bridge as a process
t=0   yolo_node → starts yolo_surveillance as a process


## Note about launch.py
In our launch.py, we also start (not created) a node `robot_state_publisher`.
Its pubs/subs
Direction	Topic	Type
subscribes	/joint_states	sensor_msgs/JointState
publishes	/tf and /tf_static	tf2_msgs/TFMessage
publishes	/robot_description	the robot model, used by the spawn step

We needed this for our turtlebot. 

# SLAM
Note that we'd be replacing Cartographer with `slam_toolbox` from NAV2 as it is more maintained and 
fits better with our project.

For keyboard, we use `teleop_twist_keyboard`. It publishes plain `Twist` by
default, but a stamped parameter switches it to `TwistStamped`, which is what the
TB4’s /cmd_vel bridge expects. It also has speed and turn parameters.


Note that the `teleop_twist_keyboard` is just a simple cmd line utility. We use it to 
publish `TwistStamped` with setting some forward and turn speed.

We run it as 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=cmd_vel -p stamped:=true -p speed:=0.2 -p turn:=0.5
```

Teleop reads keypresses from the terminal and publishes them on a topic called `cmd_vel`. 
`cmd_vel` is already expected by bridge_ros_gz as setup by the turtle bot 4 launch file. 

Flow:
```
teleop_twist_keyboard
   │ publishes /cmd_vel (TwistStamped, ROS 2)
   ▼
bridge_ros_gz   (started by spawn_tb4.launch.py, config in tb4_bridge.yaml)
   │ converts to gz.msgs.Twist, direction ROS → Gazebo
   ▼
Gazebo topic /cmd_vel
   │ subscribed to by
   ▼
diff-drive plugin inside the robot (gz-sim-diff-drive-system, topic /cmd_vel)
   ▼
wheels turn, and odometry comes back out on /odom
```


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


### Turtle Bot
What the robot publishes (Gazebo to ROS)

ROS topic	Type	What it is
/clock	rosgraph_msgs/Clock	Simulation time
/odom	nav_msgs/Odometry	Wheel odometry from the diff-drive plugin
/tf	tf2_msgs/TFMessage	Transforms from Gazebo, from the diff-drive plugin per the bridge file’s comments
/joint_states	sensor_msgs/JointState	Wheel joint positions
/scan	sensor_msgs/LaserScan	2D lidar
/imu	sensor_msgs/Imu	IMU
/rgbd_camera/image	sensor_msgs/Image	OAK-D colour image
/rgbd_camera/depth_image	sensor_msgs/Image	OAK-D depth image
/rgbd_camera/camera_info	sensor_msgs/CameraInfo	Camera intrinsics

What the robot subscribes to (ROS to Gazebo)
ROS topic	Type	What it does
/cmd_vel	geometry_msgs/msg/TwistStamped



# Navigation Package Guide
The config folder contains the configuration for the `slam_toolbox` package.
The launch folder contains two ways to launch the slam_toolbox. `mapping_launch` is for the one time 
launch of slam_toolbox to make a SLAM based map of the system. The other one is the usual launch which uses the 
already built map.