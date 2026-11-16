import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node

CAMERAS = ['cam_pillar_north',
            # 'cam_shed_sw', 'cam_building_se'
            ]

# TurtleBot 4 spawn pose in the Gazebo world (open floor, clear of the shop and sheds).
ROBOT_POSE = {'x': '-8.0', 'y': '-3.0', 'z': '0.01', 'yaw': '0.0'}

# Seconds to wait before spawning, so Gazebo (and the actor mesh download) is ready.
SPAWN_DELAY = 10.0


def generate_launch_description():
    pkg_sim = get_package_share_directory('surveillance_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_tb4_sim = get_package_share_directory('nav2_minimal_tb4_sim')
    pkg_tb4_desc = get_package_share_directory('nav2_minimal_tb4_description')

    world_file = os.path.join(pkg_sim, 'worlds', 'surveillance_arena.world')
    robot_xacro = os.path.join(pkg_tb4_desc, 'urdf', 'standard', 'turtlebot4.urdf.xacro')

    # -r: start the simulation running (not paused), -v 4: verbose logging
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        # adding -s in gz_args would start the Gazebo server only.
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    # Gazebo Transport -> ROS 2 bridge for the three surveillance cameras.
    # Format: <topic>@<ROS type>[<gz type>  ("[" = gz -> ROS only)
    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='camera_bridge',
        arguments=[
            f'/{cam}/image@sensor_msgs/msg/Image[gz.msgs.Image'
            for cam in CAMERAS
        ],
        # yolo_node still listens on /surveillance_camera/image_raw, so
        # feed it the north pillar camera for now.
        remappings=[
            ('/cam_pillar_north/image', '/surveillance_camera/image_raw'),
        ],
        output='screen'
    )

    # Publishes the robot_description topic that the spawn step reads,
    # plus the TF between the robot's links.
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'robot_description': Command(['xacro', ' ', robot_xacro]),
        }],
    )

    # Spawns the TurtleBot 4 and starts its bridges (/clock, /odom, /scan, /imu,
    # /tf, /joint_states, /cmd_vel, OAK-D camera). Provided by nav2_minimal_tb4_sim.
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_tb4_sim, 'launch', 'spawn_tb4.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'true',
            'x_pose': ROBOT_POSE['x'],
            'y_pose': ROBOT_POSE['y'],
            'z_pose': ROBOT_POSE['z'],
            'yaw': ROBOT_POSE['yaw'],
        }.items()
    )

    yolo_node = Node(
        package='perception',
        executable='yolo_surveillance',
        name='yolo_node',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        gazebo,
        camera_bridge,
        robot_state_publisher,
        TimerAction(period=SPAWN_DELAY, actions=[spawn_robot]),
        yolo_node,
    ])