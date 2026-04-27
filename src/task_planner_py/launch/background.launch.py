from launch import LaunchDescription 
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess

def generate_launch_description():


    robp_launch_dir = get_package_share_directory('robp_launch')

    arm_launch_file = os.path.join(robp_launch_dir, 'launch', 'arm_launch.yaml')
    arm_camera_launch_file = os.path.join(robp_launch_dir, 'launch', 'arm_camera_launch.yaml')

    return LaunchDescription([
#         ExecuteProcess(
#             cmd=['pixi', 'run', 'phidgets'],
#             output='screen'
#         ),
        ExecuteProcess(
            cmd=['pixi', 'run', 'realsense'],
            output='screen'
        ),

        # ExecuteProcess(
        #     cmd=['pixi', 'run', 'lidar'],
        #     output='screen'
        # ),

#         Node(
#             package='learning_tf2_py',
#             executable='arm_safe_republisher',
#             name='arm_safe_republisher',
#             output='screen',
#          ),
        
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(arm_launch_file)
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(arm_camera_launch_file)
        ),
    ])
