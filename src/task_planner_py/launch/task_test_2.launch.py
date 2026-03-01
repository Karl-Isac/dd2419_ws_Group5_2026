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

    # planner = Node(
    #     package='grumpy_navigation_py',
    #     executable='planner',
    #     name='planner',
    #     output='screen'
    # )

    planner = Node(
        package='grumpy_navigation_py',
        executable='planner_itemarray',
        name='planner_itemarray',
        output='screen'
    )

    # planner = Node(
    #     package='grumpy_navigation_py',
    #     executable='planner_posearray',
    #     name='planner_posearray',
    #     output='screen'
    # )

    tf_object = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_object_0',
        arguments=['1.0', '1.0', '0', '0', '0', '0', 'map', 'object_0']
    )

    tf_box = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_box_0',
        arguments=['0.0', '-1.0', '0', '0', '0', '0', 'map', 'box_0']
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

    phidgets = ExecuteProcess(
        cmd=['pixi', 'run', 'phidgets'],
        output='screen'
    )

    fake_arm = Node(
        package='task_planner_py',
        executable='fake_arm_node',
        name='fake_arm_node',
        output='screen'
    )

    return LaunchDescription([
        controller,
        planner,
        tf_object,
        tf_box,
        tf_odom,
        odometry,
        phidgets,
        fake_arm
    ])
