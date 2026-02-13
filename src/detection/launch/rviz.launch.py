from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    package_dir = get_package_share_directory('detection')

    rviz_config = os.path.join(
        package_dir,
        'rviz',
        'view.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output= 'screen',
    )

    return LaunchDescription([rviz_node])