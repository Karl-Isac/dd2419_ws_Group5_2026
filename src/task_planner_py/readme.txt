pixi shell

pixi run colcon build --packages-select grumpy_navigation_py task_planner_py detection --symlink-install

pixi run colcon build --packages-select grumpy_navigation_py task_planner_py detection 

ros2 launch task_planner_py task_test_2.launch.py

ros2 run task_planner_py task_planner_node_posearray

pixi run realsense

pixi run ros2 run detection detection
