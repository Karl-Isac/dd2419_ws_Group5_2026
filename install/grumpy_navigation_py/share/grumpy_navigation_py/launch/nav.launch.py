
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    common_params = {
        "world_frame": "map",      # or "odom" if you're not localizing
        "base_frame": "base_link",
    }

    planner_params = {
        **common_params,
        "num_points": 40,
    }

    controller_params = {
        **common_params,
        "rate_hz": 20.0,

        # Stop distances (tune these for your arm)
        "object_stop_distance": 0.35,
        "box_stop_distance": 0.25,

        # Following behavior
        "lookahead": 0.40,
        "angle_tolerance": 0.25,

        # Duty cycle command limits
        "duty_forward": 0.20,
        "duty_turn": 0.15,
        "max_duty": 0.25,
    }

    planner = Node(
        package="grumpy_navigation_py",
        executable="planner",
        name="planner",
        output="screen",
        parameters=[planner_params],
        remappings=[
            # (" /nav/goal", "/nav/goal"),   # default already
            # ("/nav/path", "/nav/path"),
        ],
    )

    controller = Node(
        package="grumpy_navigation_py",
        executable="controller",
        name="controller",
        output="screen",
        parameters=[controller_params],
        remappings=[
            # default: /nav/path, /nav/phase, /nav/reached, /phidgets/motor/duty_cycles
        ],
    )

    return LaunchDescription([planner, controller])
