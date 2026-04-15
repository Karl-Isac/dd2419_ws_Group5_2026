from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    arm_safe_republisher = Node(
        package='learning_tf2_py',
        executable='arm_safe_republisher',
        name='arm_safe_republisher',
        output='screen'
    )

    arm_control = Node(
        package='learning_tf2_py',
        executable='arm_control',
        name='arm_control',
        output='screen'
    )

    task_planner = Node(
        package='task_planner_py',
        executable='task_planner',
        name='task_planner',
        output='screen'
    )

    odometry = Node(
        package='odometry',
        executable='odometry',
        name='odometry',
        output='screen'
    )


    planner = Node(
        package='grumpy_navigation_py',
        executable='planner',
        name='planner',
        output='screen'
    )

    controller = Node(
        package='grumpy_navigation_py',
        executable='controller',
        name='controller',
        output='screen'
    )

    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0.49', '0.5', '0', '0', '0', '0', 'map', 'odom']
    )

    return LaunchDescription([
        arm_safe_republisher,
        arm_control,
        task_planner,
        odometry,
        planner,
        controller,
        map_to_odom,
    ])
