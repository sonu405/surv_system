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

