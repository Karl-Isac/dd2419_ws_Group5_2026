from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():

    phidgets = ExecuteProcess(
        cmd=["pixi", "run", "phidgets"],
        output="screen",
        # IMPORTANT if your pixi.toml is not in the workspace root:
        # cwd="/absolute/path/to/your/pixi/project",
    )

    odom = Node(
        package="odometry",
        executable="odometry",
        name="odometry",
        output="screen",
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf",
        output="screen",
        arguments=[
            "0", "0", "0",
            "0", "0", "0",
            "--frame-id", "map",
            "--child-frame-id", "odom",
        ],
    )

    return LaunchDescription([
        phidgets,
        odom,
        static_tf,
    ])
