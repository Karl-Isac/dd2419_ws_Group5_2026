from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('detection')
    rviz_config = os.path.join(package_dir, 'rviz', 'view.rviz')

    # 静态变换发布器：map -> realsense_camera_link
    # 请将下面的平移和旋转参数替换为你的实际数值
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_to_realsense',
        arguments=[
            '1', '1', '0',        # x, y, z (平移，单位：米)
            '1', '0', '0',        # yaw, pitch, roll (旋转，单位：弧度)
            'map',                # 父坐标系
            'realsense_camera_link'  # 子坐标系
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([static_tf_node, rviz_node])