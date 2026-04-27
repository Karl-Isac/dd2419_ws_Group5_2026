from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([

        # Static TF: base_link -> lidar_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            output='screen',
            arguments=['0.0', '0.09', '0.1', '0', '0', '0', 'base_link', 'lidar_link'],
        ),

        # Run your lidar via pixi
        ExecuteProcess(
            cmd=['pixi', 'run', 'lidar'],
            output='screen'
        ),

    ])
