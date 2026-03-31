Interfaces:
    The node listens on the /exploration/request_unexplored_point topic for any String message,
    returns a Point message on topic /exploration/return_unexplored_point.
    Normally the point is in the (x,y,0) format, if no unexplored points are left it's (0,0,42) instead.

    It also publishes the exploration map to /exploration/map to visualize

How to run:
    pixi run ros2 run learning_tf2_py arm_control
    make sure to re-run each time you restart the robot to wipe the map from memory
