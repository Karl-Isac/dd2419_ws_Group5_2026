
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # exploration_mapper = Node(
    #     package='learning_tf2_py',
    #     executable='exploration_mapper',
    #     name='exploration_mapper',
    #     output='screen'
    # )

    exploration_mapper_new = Node(
        package='learning_tf2_py',
        executable='exploration_mapper_new',
        name='exploration_mapper',
        output='screen'
    )

    # detection = Node(
    #     package='detection',
    #     executable='alter',
    #     name='detection',
    #     output='screen'
    # )

    detection_apr20 = Node(
        package='detection',
        executable='alter_apr20',
        name='detection_apr20',
        output='screen'
    )


    return LaunchDescription([
        # exploration_mapper,
        exploration_mapper_new,
        # detection,
        detection_apr20,
    ])
