The structure of the detection package:
- config: stores the initial map and final output map (in name of detection_output.csv)
- detection: the main function of detection, map file reading and writing and communication between this node and task planner
- launch: the launch file of rviz and static tf between map to base_link for testing
- rviz: the configuration file of rviz specialized for this part in testing, including visualization of some tf, one or two pointclouds (points and ds_points, the latter is for box detection)

Interfaces:
    Publisher: 
        Type: PoseArray
        Name: /detected_objects
        Info: Publish Pose of objects, will be called at init and a new object is detected

    Publisher: 
        Type: PoseArray
        Name: /detected_boxes
        Info: Publish Pose of boxes, will be called at init and a new box is detected

    Subscriber (will be wrote later):
        Type: (Not Decided)
        Name: (Not Decided)
        Info: Tells the whole task is over and this node can write the final map file

How to run the code:
-------- MAKE SURE TO PUBLISH TF FROM map TO base_link IN ADVANCE, IF NOT, USE THE LINE IN BRANKET WHICH INCLUDES A STATIC TF FOR TESTING AND Rviz VISUALIZATION --------------------
pixi run realsense
(pixi run ros2 launch detection rviz.launch.py)
pixi run ros2 run detection detection
