from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
    
def generate_launch_description():
    pkg_share = FindPackageShare('robot_nav') 
    default_nav2_params = PathJoinSubstitution([pkg_share, 'config', 'nav2_params.yaml'])
    default_map = PathJoinSubstitution([pkg_share, 'maps', 'ongaing_arena_map.yaml'])

    # Declare CLI arguments
    declare_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # Capture configurations
    use_sim_time = LaunchConfiguration('use_sim_time')


    # This is the node from nav2 which we must run. It uploads the saved map on /map which acts as input 
    # to the ACML along with some initial state. 
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            default_nav2_params,
            {'yaml_filename': default_map, 'use_sim_time': use_sim_time}
        ],
   )

    # Estimates the robot's pose on the map using laser scans and odometry
    # It uses the /map from map_server along with some initial pose. This initial
    # pose comes from the nav2_params.yaml.
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            default_nav2_params,
            {'use_sim_time': use_sim_time}
        ],
        )

    # Transitions map_server and amcl from unconfigured -> active
    lifecycle_manager_node = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl'],
            }],
        )

    return LaunchDescription([
        declare_sim_time_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node
    ]) 