
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    exploration_mapper = Node(
        package='learning_tf2_py',
        executable='exploration_mapper',
        name='exploration_mapper',
        output='screen'
    )

    detection = Node(
        package='detection',
        executable='alter',
        name='detection',
        output='screen'
    )


    return LaunchDescription([
        exploration_mapper,
        detection,
    ])
