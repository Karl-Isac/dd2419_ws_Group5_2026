from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_dir = get_package_share_directory('detection')
    rviz_config = os.path.join(package_dir, 'rviz', 'view.rviz')


    realsense = ExecuteProcess(
        cmd=['pixi', 'run', 'realsense'],
        output='screen'
    )

    odometry = ExecuteProcess(
        cmd=['pixi', 'run', 'odometry'],
        output='screen'
    )

    phidgets = ExecuteProcess(
        cmd=['pixi', 'run', 'phidgets'],
        output='screen'
    )

    return LaunchDescription([realsense, odometry, phidgets])