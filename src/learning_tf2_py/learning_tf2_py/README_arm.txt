Interfaces:

    The node listens on the /arm/cmd topic for the following String messages:
        pick
        place
        (Might be implemented in the future:
        pick red
        pick green
        pick blue
        pick wood)

    It can publish the following String messages to /arm/report_back:
        pick_success
        pick_fail
        place_success
        place_fail


How to run:
pixi run arm_init
pixi run ros2 run learning_tf2_py arm_control
... and start publishing your commands