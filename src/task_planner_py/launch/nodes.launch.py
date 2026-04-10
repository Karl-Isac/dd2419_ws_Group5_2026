from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    return LaunchDescription([

        Node(
            package='learning_tf2_py',
            executable='arm_safe_republisher',
            name='arm_safe_republisher',
            output='screen'
        ),
        Node(
            package='learning_tf2_py',
            executable='arm_control',
            name='arm_control',
            output='screen'
        ),
        Node(
            package='task_planner_py',
            executable='task_planner',
            name='task_planner',
            output='screen'
        ),
        Node(
            package='odometry',
            executable='odometry',
            name='odometry',
            output='screen'
        ),
        Node(
            package='mapping',
            executable='fix_pose',
            name='fix_pose',
            output='screen'
        ),
        Node(
            package='grumpy_navigation_py',
            executable='fake_obstacles',
            name='fake_obstacles',
            output='screen'
        ),
    ])
