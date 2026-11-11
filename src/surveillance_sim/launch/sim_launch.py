import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

CAMERAS = ['cam_pillar_north',
            # 'cam_shed_sw', 'cam_building_se' Commentedd right now to save GPU from exploding
            ]


def generate_launch_description():
    pkg_sim = get_package_share_directory('surveillance_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_sim, 'worlds', 'surveillance_arena.world')

    # -r: start the simulation running (not paused), -v 4: verbose logging
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    # Gazebo Transport -> ROS 2 bridge for all three camera image topics.
    # Format: <topic>@<ROS type>[<gz type>  ("[" = gz -> ROS only)
    bridge = Node(
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

    yolo_node = Node(
        package='perception',
        executable='yolo_surveillance',
        name='yolo_node',
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        bridge,
        yolo_node
    ])