6 terminals are needed. Enable pixi shell in all of them. 

When you run the last command (detection) the robot will start moving directly.


Run commands in this order:


pixi run phidgets

pixi run arm_init

pixi run ros2 run learning_tf2_py arm_control

ros2 launch task_planner_py final_test.launch.py

ros2 run task_planner_py task_planner_node_posearray_3

ros2 run detection detection

