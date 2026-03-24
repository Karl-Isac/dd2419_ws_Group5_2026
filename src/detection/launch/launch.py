from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('detection')
    rviz_config = os.path.join(package_dir, 'rviz', 'view.rviz')

    # static tf brodcaster：map -> odom
    
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_realsense',
        arguments=[
            '0.49', '0.50', '0',        # x, y, z (in meters)
            '0', '0', '0',        # yaw, pitch, roll (in rads)
            'map',                # parent frame
            'odom'  # child frame
        ],
        output='screen'
    )

    # static tf brodcaster：map -> base_link

    # static_tf_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_map_to_realsense',
    #     arguments=[
    #         '0.49', '0.50', '0',        # x, y, z (in meters)
    #         '0', '0', '0',        # yaw, pitch, roll (in rads)
    #         'map',                # parent frame
    #         'base_link'  # child frame
    #     ],
    #     output='screen'
    # )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    # realsense = ExecuteProcess(
    #     cmd=['pixi', 'run', 'realsense'],
    #     output='screen'
    # )

    # odometry = ExecuteProcess(
    #     cmd=['pixi', 'run', 'ros2', 'run', 'odometry', 'odometry'],
    #     output='screen'
    # )

    # phidgets = ExecuteProcess(
    #     cmd=['pixi', 'run', 'phidgets'],
    #     output='screen'
    # )

    # detection = ExecuteProcess(
    #     cmd=['pixi', 'run', 'ros2', 'run', 'detection', 'detection'],
    #     output='screen'
    # )

    return LaunchDescription([static_tf_node, rviz_node])