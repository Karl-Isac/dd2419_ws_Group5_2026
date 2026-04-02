from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():

#     controller = Node(
#         package='grumpy_navigation_py',
#         executable='controller',
#         name='controller',
#         output='screen'
#     )

#     planner = Node(
#         package='grumpy_navigation_py',
#         executable='planner',
#         name='planner',
#         output='screen'
#     )

    fake_obstacles = Node(
        package='grumpy_navigation_py',
        executable='fake_obstacles',
        name='fake_obstacles',
        output='screen'
    )

    tf_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0.49', '0.5', '0', '0', '0', '0', 'map', 'odom']
    )

    odometry = Node(
        package='odometry',
        executable='odometry',
        name='odometry',
        output='screen'
    )

#     phidgets = ExecuteProcess(
#         cmd=['pixi', 'run', 'phidgets'],
#         # cmd=['phidgets'],
#         output='screen'
#     )

#     detection = Node(
#         package='detection',
#         executable='detection',
#         name='detection',
#         output='screen'
#     )

    realsense = ExecuteProcess(
        cmd=['pixi', 'run', 'realsense'],
        # cmd=['realsense'],
        output = 'screen'
    )

#     arm_init = ExecuteProcess(
#         cmd=['pixi', 'run', 'arm_init'],
#         output='screen'
#     )
# 
#     arm_control = ExecuteProcess(
#         cmd=['pixi', 'run', 'ros2', 'run', 'learning_tf2_py', 'arm_control'],
#         output='screen'
#     )

#     task_planner = Node(
#         package='task_planner_py',
#         executable='task_planner',
#         name='task_planner',
#         output='screen'
#     )

    





    return LaunchDescription([
        # controller,
        # planner,
        fake_obstacles,
        tf_odom,
        odometry,
        #phidgets,
        # detection,
        realsense,
        #arm_init,
        #arm_control,
        #task_planner,

    ])
