ROS2 -- lyrical 
Gazebo -- Jetty (10.5.0)

# ROS 2 GUIDE
We have three important components in ROS
- Workspace: Overall ROS2 environment.
- Package: Individual ros project like `surveillance_sim`
- Overlay: workspace's installed packages layered on top of the ROS 2 installation

Note that ROS 2 doesn't know automatically about the workspace so we much source the environement.

Command	                                Purpose
source  /opt/ros/<distro>/setup.bash	Load ROS 2 environment
source install/setup.bash	            Load your workspace packages
colcon build	                        Build your workspace
ros2 pkg list	                        List available ROS packages
ros2 pkg executables	                List executables provided by packages

For building a specific packages
`colcon build --packages-select surveillance_sim surveillance_sim`

Creating a package
```py
# Example:
ros2 pkg create --build-type ament_python pkg_name --dependencies rclpy sensor_msgs cv_bridge
```

Note that ,we need to source our workspace again after building.
`source install/setup.bash`

# Interaction Between ROS2 and Gazebo
Modern Gazebo and ROS 2 communicate through `ros_gz` integration packages.


# Package Commands
1. All packages disconverable by ros:- 
`ros2 pkg list`

2. Where ROS intalled a package
```
ros2 pkg prefix surveillance_sim
```
ROS doesn't necessarily execute files directly from src/. It discovers installed
package resources through the workspace environment.



# How `setup.py` in a package work
When we run `ros2 launch robot_nav mapping_launch.py`, ROS2 doesn't look in the src/ folder. It instead looks
into `install/robot_nav/share/robot_nav/`. 

`data_files` list inside `setup.py` tells the build which files to copy into the `install/` folder.
Without it, our launch files and configs stay in `src/` and nothing can find them at runtime.

Each entry of `data_files` is the pair `(destination folder, [source files])`.
The destination is relative to the `install` prefix and the sources are relative to `setup.py`.

A newly created package by default contains
```
('share/ament_index/resource_index/packages', ['resource/' + package_name]),
('share/' + package_name, ['package.xml']),
```

1. The first copies a tiny marker file (resource/robot_nav) into the ament
index. This registers the package, and it’s how `ros2 pkg list`, `ros2 launch` and
`get_package_share_directory` know `robot_nav` exists.

2. The second copies `package.xml` into the package’s share folder, so tools can
read its metadata and dependencies.


# Launch Files
ROS2 launch files are recipe builders. When we run `ros2 launch ...`, ROS
executes `generate_launch_description()`, collects the blueprint
(LaunchDescription) we return, and then hands it off to an internal execution
engine to manage processes, events, and parameters.

Launch files have 4 core patterns.

1. Paths: `FindPackageShare` + `PathJoinSubstitution` 

```py
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Resolves to: <workspace>/install/my_robot_nav/share/my_robot_nav/config/nav2_params.yaml
params_file = PathJoinSubstitution([
    FindPackageShare('my_robot_nav'),
    'config',
    'nav2_params.yaml'
])
```

2. CLI Arguments
Instead of hardcoding argument, we expose them on the CLI so that they can overridden from the 
terminal.

This always work in pair.
1. `DeclareLaunchArgument`: Defines the flag, its default value, and a description.
2. `LaunchConfiguration`: A variable handle that fetches whatever value was passed in.

```py
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# 1. Declare the argument (put this inside generate_launch_description)
declare_map_cmd = DeclareLaunchArgument(
    'map',
    default_value=PathJoinSubstitution([FindPackageShare('my_robot_nav'), 'maps', 'warehouse.yaml']),
    description='Full path to map yaml file to load'
)

# 2. Create a reference to read it later
map_yaml = LaunchConfiguration('map')
```

> Note that we must register the declare_map_cmd inside our `LaunchDescription([...])` list so 
> ROS registers the CLI flag.

3. Composing Launch Files (`IncludeLaunchDescription`)
Instead of putting an entire robot stack into a single giant file, we build 
modular launch files and include them inside a top-level "bringup" launch file. 

```py
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

nav2_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        PathJoinSubstitution([FindPackageShare('nav2_bringup'), 'launch', 'localization_launch.py'])
    ]),
    launch_arguments={
        'map': map_yaml,
        'use_sim_time': 'True',
        'params_file': params_file
    }.items()
)
```

4. Conditional Logic & Event Handlers (`IfCondition` & `RegisterEventHandler`)

Because LaunchConfiguration values aren't known when Python initially parses the file, standard Python if use_rviz: statements will not work. For this, we use ROS launch conditions and event handlers.

Conditional Execution: Start RViz only if use_rviz:=true.
```py
from launch.conditions import IfCondition

rviz_node = Node(
    package='rviz2',
    executable='rviz2',
    condition=IfCondition(LaunchConfiguration('use_rviz'))
)
```

# Cartographer
It is Google's SLAM system. It does real time SLAM (that is build a map while tracing where the robot is) in 2D and 
3D, with lidar and optional IMU and odometry. 

The ROS2 integration comes in few packages
1. `cartographer`: the core C++ SLAM library.
2. `cartographer_ros`: ros2 wrapper 
3. `cartographer_ros_msgs` and `cartographer_rviz`: messages and RViz display support.

Note, that the maintainance of Cartographer is on hold. `slam_toolbox` by NAV2 is more actively maintained 
so we'll be using that instead of Cartogrpaher for our project. 


# slam_toolbox
Note that the slam_toolbox, the map server, AMCL and Nav2 are existing nodes. We only write the configurations
that start them with the right settings. 


# TF
TF is ROS2's system for tracking ***coordinate frames*** and how they relate to each other. 

Every part of the robot has its own frame (its own origin and axes), and TF
stores the transforms between them as a tree.

map ──► odom ──► base_link ──► laser
                     └──────► camera_link

Transforms are published on the /tf and /tf_static topics. We can view them as 
```py
ros2 run tf2_tools view_frames     # writes frames.pdf showing your current tree
ros2 run tf2_ros tf2_echo odom base_link
```

# Odometry
Odometry is the robot’s estimate of how far it has moved since it started, computed from its own motion sensors, with no reference to the map. Our turtle bot publies it on /odom.