from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    # -----------------------
    # External launch files
    # -----------------------
    robp_launch_dir = get_package_share_directory('robp_launch')

    arm_launch_file = os.path.join(robp_launch_dir, 'launch', 'arm_launch.yaml')
    arm_camera_launch_file = os.path.join(robp_launch_dir, 'launch', 'arm_camera_launch.yaml')

    # -----------------------
    # Hardware processes
    # -----------------------
    realsense = ExecuteProcess(
        cmd=['pixi', 'run', 'realsense'],
        output='screen'
    )

    lidar = ExecuteProcess(
        cmd=['pixi', 'run', 'lidar'],
        output='screen'
    )

    # -----------------------
    # Static TFs
    # -----------------------
    lidar_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf',
        output='screen',
        arguments=['0.0', '0.09', '0.1', '0', '0', '0', 'base_link', 'lidar_link'],
    )

    # -----------------------
    # Nodes
    # -----------------------
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
        executable='odometry_fixed',
        name='odometry_fixed',
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

    move_backwards_timer = Node(
        package='grumpy_navigation_py',
        executable='move_backwards_timer',
        name='move_backwards_timer',
        output='screen'
    )

    obstacle_detection = Node(
        package='grumpy_navigation_py',
        executable='obstacle_detection',
        name='obstacle_detection',
        output='screen'
    )

    approach_goal = Node(
        package='grumpy_navigation_py',
        executable='approach_goal',
        name='approach_goal',
        output='screen'
    )

    # -----------------------
    # Launch description
    # -----------------------
    return LaunchDescription([
        # Hardware
        realsense,
        lidar,

        # Static TF
        lidar_static_tf,

        # Arm + camera includes
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(arm_launch_file)
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(arm_camera_launch_file)
        ),

        # Core system
        arm_safe_republisher,
        arm_control,
        task_planner,
        odometry,
        planner,
        controller,
        move_backwards_timer,
        obstacle_detection,
        approach_goal,
    ])
