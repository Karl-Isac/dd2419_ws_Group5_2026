from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['pixi', 'run', 'phidgets'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['pixi', 'run', 'realsense'],
            output='screen'
        ),

        Node(
            package='learning_tf2_py',
            executable='arm_safe_republisher',
            name='arm_safe_republisher',
            output='screen',
        ),
        
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(arm_launch_file)
        ),

        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(arm_camera_launch_file)
        ),
    ])
