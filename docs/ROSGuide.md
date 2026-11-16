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



