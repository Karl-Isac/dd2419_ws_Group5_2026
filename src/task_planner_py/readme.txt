5 terminals are needed. Enable pixi shell in all of them. 

When you run the last command the robot will start moving.


First run this: 
colcon build --packages-select grumpy_navigation_py detection learning_tf2_py task_planner_py odometry grumpy_interfaces --symlink-install

Then run this in every terminal:
source install/setup.bash


For rviz vizualisation follow these steps:
- Be on your computer
- Be in the folder ~/dd2419_ws_Group5_2026/
- Run pixi shell
- Then run this command:
ros2 run rviz2 rviz2 -d ~/dd2419_ws_Group5_2026/my_conf_2.rviz

(You can also create your own rviz config if you want, or just add all the relevant topics in rviz)


Then run commands in this order order is important:

ros2 launch task_planner_py background.launch.py
ros2 launch task_planner_py nodes.launch.py
ros2 run grumpy_navigation_py fake_obstacles
pixi run phidgets
ros2 launch task_planner_py exploration_detection.launch.py



HELPERS:

If you want to instantly start 6 terminals in tmux you can run the following script:
start_tmuxs.sh


