import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_planning = get_package_share_directory('path_planning')
    default_rviz_config = os.path.join(pkg_planning, 'rviz', 'planning.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='false',
        description='Launch RViz2 with pre-configured planning view if true'
    )

    planner_node = Node(
        package='path_planning',
        executable='planner_node',
        name='planner_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_radius': 0.05,
            'safety_margin': 0,
            'goal_topic': '/goal_pose',
        }]
    )

    smoother_node = Node(
        package='path_planning',
        executable='smoother_node',
        name='smoother_node',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'degree': 3,
            'num_samples': 200,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config],
        condition=IfCondition(use_rviz),
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        declare_sim_time_arg,
        declare_use_rviz_arg,
        planner_node,
        smoother_node,
        rviz_node,
    ])
