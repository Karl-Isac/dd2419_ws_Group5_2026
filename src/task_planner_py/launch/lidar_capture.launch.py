from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('realsense2_description'),
            'urdf',
            'test_d435_camera.urdf.xacro'
        ]),
        ' use_nominal_extrinsics:=false'
    ])

    return LaunchDescription([
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz',
        #     output='screen',
        # ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_description},
                {'publish_frequency': 30.0},
                {'use_sim_time': False},
            ],
        ),

        # Temporary fake transform: map -> odom
        # Only use this if you intentionally want a fixed odom in map.
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_tf',
            output='screen',
            arguments=['0.5', '0.49', '0.0', '0', '0', '0', 'map', 'odom'],
        ),

        # Lidar mount: base_link -> lidar_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            output='screen',
            arguments=['0.0', '0.09', '0.1', '0', '0', '0', 'base_link', 'lidar_link'],
        ),

        # Node(
        #     package='mapping',
        #     executable='space',
        #     name='make_space',
        #     output='screen',
        # ),
        #
        # Node(
        #     package='mapping',
        #     executable='stuff',
        #     name='place_stuff',
        #     output='screen',
        # ),
    ])
