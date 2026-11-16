import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_robot_nav = get_package_share_directory('robot_nav')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    default_params = os.path.join(pkg_robot_nav, 'config', 'slam_toolbox_mapping.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use the Gazebo clock (/clock) instead of wall time')
    declare_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=default_params,
        description='slam_toolbox parameter file')

    # Reuse slam_toolbox's own launch file, but with our parameters.
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'slam_params_file': slam_params_file,
            'use_sim_time': use_sim_time,
        }.items()
    )

    return LaunchDescription([declare_use_sim_time, declare_params_file, slam])