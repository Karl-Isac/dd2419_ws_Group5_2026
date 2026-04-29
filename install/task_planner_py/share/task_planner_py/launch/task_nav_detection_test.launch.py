from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

    controller = Node(
        package='grumpy_navigation_py',
        executable='controller',
        name='controller',
        output='screen'
    )

    planner = Node(
        package='grumpy_navigation_py',
        executable='planner',
        name='planner',
        output='screen'
    )

    tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
    )

    odometry = Node(
        package='odometry',
        executable='odometry',
        name='odometry',
        output='screen'
    )

    # phidgets = ExecuteProcess(
    #     cmd=['pixi', 'run', 'phidgets'],
    #     output='screen'
    # )


    realsense = ExecuteProcess(
        cmd=['pixi', 'run', 'realsense'],
        output = 'screen'
    )

    # detection = Node(
    #     package='detection',
    #     executable='detection',
    #     name='detection',
    #     output='screen'
    # )




    return LaunchDescription([
        controller,
        planner,
        tf_odom,
        odometry,
        # phidgets,
        # detection,
        realsense
    ])
