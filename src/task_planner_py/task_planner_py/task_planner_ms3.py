#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
import tf2_ros

from std_msgs.msg import Bool, String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, Point

from grumpy_interfaces.msg import GoalWithType, PathWithType, PathWithStatus

import sys


class TrackedObject:
    def __init__(self, obj_id: int, x: float, y: float):
        self.id = obj_id
        self.x = x
        self.y = y
        self.status = "detected"   # detected / picked / placed


class TrackedBox:
    def __init__(self, box_id: int, x: float, y: float):
        self.id = box_id
        self.x = x
        self.y = y


class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__("task_planner_node")

        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("rate_hz", 5.0)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value

        # Local IDs generated from latest detections
        self.next_object_id = 0
        self.next_box_id = 0

        # Tracked detections
        self.known_objects = []   # list[TrackedObject]
        self.known_boxes = []     # list[TrackedBox]

        # Active task
        self.current_object = None
        self.current_box = None
        
        self.current_exploration_point = None

        self.ox = None
        self.oy = None
        self.bx = None
        self.by = None

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            
        # ===== state flags =====
        self.generate_exploration_pose_success = False
        self.generate_exploration_path_success = False
        self.execute_exploration_path_success = False

        self.generate_path_object_success = None

        self.execute_path_object_success = None

        self.generate_path_box_success = None
        self.execute_path_box_success = None
        
        self.generate_exploration_path_failed = False

        self.approach_success = False

        self.move_backwards_success = False
        self.state_after_move_backward = None

        self.last_planner_status = None

        # Planner state
        # self.state = "SELECT_OBJECT"
        self.state = "GENERATE_EXPLORATION_POSE"
        #self.nav_reached = False
        self.pick_done = False
        self.place_done = False
        self._published_this_state = False

        self.path_to_goal = None

        # Publishers
        #self.goal_pub = self.create_publisher(PoseStamped, "/nav/goal", 10)
        self.goal_pub = self.create_publisher(GoalWithType, "/nav/goal", 10)
        self.path_to_controller_pub = self.create_publisher(PathWithType, "/nav/path_to_controller", 10)
        self.path_to_controller_pub_viz = self.create_publisher(Path, "/nav/path_to_controller_viz", 10)
        # self.approach_goal_pub = self.create_publisher(PoseStamped, "/nav/approach_start", 10)
        self.approach_goal_pub = self.create_publisher(GoalWithType, "/nav/approach_start", 10)
        self.path_blocked_pub = self.create_publisher(Bool, "/nav/path_blocked", 10)

        self.arm_pub = self.create_publisher(String, "/arm/cmd", 10)

        self.move_backwards_pub = self.create_publisher(Bool, "/nav/move_backwards_start", 10)

        # Exploration
        self.exploration_pub = self.create_publisher(String, "/exploration/request_unexplored_point", 10) # content can be anything
        self.create_subscription(Point, "/exploration/return_unexplored_point", self.on_exploration_point, 10) # z value irrelevant, 
        # TODO: if z is not 0 -> stop exploring

        # Subscribers
        self.create_subscription(Bool, "/nav/reached", self.on_reached, 10)
        # self.create_subscription(Path, "/nav/path_from_planner", self.on_path_from_planner, 10)
        self.create_subscription(PathWithStatus, "/nav/path_from_planner", self.on_path_from_planner, 10)
        self.create_subscription(String, "/arm/report_back", self.on_report_back, 10)
        self.create_subscription(PoseArray, "/detected_objects", self.on_objects, 10)
        self.create_subscription(PoseArray, "/detected_boxes", self.on_boxes, 10)
        self.create_subscription(PoseStamped, "/nav/approach_finished", self.on_approach_finished, 10)
        self.create_subscription(Bool, "/nav/move_backwards_finished", self.on_move_backwards_finished, 10)

        # Update ICP state
        self.ICP_pub = self.create_publisher(String, "/localization/start_update_ICP", 10)  # contant can be anything
        self.create_subscription(String, "/localization/finished_update_ICP", self.on_finished_update_ICP, 10)  # contant can be anything
        # self.icp_timer = self.create_timer(10, self.start_update_ICP)
        self.state_after_update_icp = None
        self.update_ICP_done = False
        self.update_ICP_start_time = None
        self.update_ICP_timeout = 0.1  # seconds

        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.step_timer = self.create_timer(dt, self.step)



        self.get_logger().info(
            "TaskPlannerNode up. "
            "Pub: /nav/goal, /arm/cmd  "
            "Sub: /nav/reached, /arm/report_back, /detected_objects, /detected_boxes"
        )

    def cancel_controller(self):
        self.path_blocked_pub.publish(Bool(data=True))

    def on_move_backwards_finished(self, msg):
        if msg.data:
            self.move_backwards_success = True
    
    def on_approach_finished(self, msg):
        self.approach_success = True


    # def reset_flags(self):
    #     self.generate_exploration_pose_success = False
    #     self.generate_exploration_path_success = False
    #     self.execute_exploration_path_success = False
    #     self.generate_path_object_success = False
    #     self.execute_path_object_success = False
    #     self.generate_path_box_success = False
    #     self.execute_path_box_success = False

    def on_finished_update_ICP(self, msg):
        self.update_ICP_done = True

    # def start_update_ICP(self):
    #     self.state_after_update_icp = self.state
    #     self.enter_state("UPDATE_ICP")
    #     self.update_ICP_start_time = self.get_clock().now()
    #     self.update_ICP_done = False

    def start_update_icp(self, next_state: str):
        self.state_after_update_icp = next_state
        self.update_ICP_start_time = self.get_clock().now()
        self.update_ICP_done = False
        self.enter_state("UPDATE_ICP")

    def on_exploration_point(self, msg: Point):
        self.current_exploration_point = msg
        self.generate_exploration_pose_success = True


    # def on_path_from_planner(self, msg: Path):
    #     self.path_to_goal = msg
    #     if self.state == "GENERATE_PATH_TO_OBJECT":
    #         self.generate_path_object_success = True
    #     elif self.state == "GENERATE_PATH_TO_BOX":
    #         self.generate_path_box_success = True
    #     elif self.state == "GENERATE_EXPLORATION_PATH":
    #         self.generate_exploration_path_success = True

    def on_path_from_planner(self, msg: PathWithStatus):

        # success = len(msg.poses) > 0
        success = msg.status == "success" and len(msg.path.poses) > 0
        self.last_planner_status = msg.status

        if self.state == "GENERATE_PATH_TO_OBJECT":
            self.generate_path_object_success = success
            

        elif self.state == "GENERATE_PATH_TO_BOX":
            self.generate_path_box_success = success

        elif self.state == "GENERATE_EXPLORATION_PATH":
            self.generate_exploration_path_success = success
            self.generate_exploration_path_failed = not success

        if success:
            self.path_to_goal = msg.path
        else:
            self.path_to_goal = None
            self.get_logger().warn(f"Empty path received in state {self.state}, status={msg.status}")

    
    def distance_sq(self, x1: float, y1: float, x2: float, y2: float) -> float:
        dx = x1 - x2
        dy = y1 - y2
        return dx * dx + dy * dy

    def on_objects(self, msg: PoseArray):
        # Replace with latest tracked objects from detection
        #self.known_objects = []

        for p in msg.poses:
            x = float(p.position.x)
            y = float(p.position.y)
            

            tracked_obj = TrackedObject(self.next_object_id, x, y)
            self.next_object_id += 1
            self.known_objects.append(tracked_obj)

        # self.get_logger().info(f"Updated objects: {len(self.known_objects)}")

    def on_boxes(self, msg: PoseArray):
        # Replace with latest tracked boxes from detection
        #self.known_boxes = []

        for p in msg.poses:
            x = float(p.position.x)
            y = float(p.position.y)

            tracked_box = TrackedBox(self.next_box_id, x, y)
            self.next_box_id += 1
            self.known_boxes.append(tracked_box)

        # self.get_logger().info(f"Updated boxes: {len(self.known_boxes)}")

    def on_report_back(self, msg: String):
        self.get_logger().info(f"on_report_back: {msg.data}")

        if msg.data == "pick_success":
            self.pick_done = True

        elif msg.data == "pick_fail":
            self.pick_done = False
            # self.current_object.status == "detected" # TODO: check if this is nessessary
            
            if self.current_object is not None:
                self.current_object.status = "failed"

                self.get_logger().warn(f"Arm failed to pickup obejct id={self.current_object.id} -> Marking as failed")

            self.current_object = None
            self.enter_state("SELECT_OBJECT")

        elif msg.data == "place_success":
            self.place_done = True

        elif msg.data == "place_fail":
            self.place_done = False
            self.enter_state("GENERATE_PATH_TO_OBJECT")

    def on_reached(self, msg: Bool):
        nav_reached = bool(msg.data)
        # print(f"nav_reaced: {nav_reached}")
        self.get_logger().info(f"on_reached. nav_reached = {nav_reached}")
        if self.state == "EXECUTE_PATH_TO_OBJECT":
            self.execute_path_object_success = nav_reached
        elif self.state == "EXECUTE_PATH_TO_BOX":
            self.execute_path_box_success = nav_reached
        elif self.state == "EXECUTE_EXPLORATION_PATH":
            self.execute_exploration_path_success = nav_reached




    def lookup_xy(self, target_frame: str):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                target_frame,
                rclpy.time.Time()
            )
        except Exception:
            return None

        t = tf.transform.translation
        return (float(t.x), float(t.y))

    # def publish_goal_xy(self, x: float, y: float, goal_type: str):
    #     p = PoseStamped()
    #     p.header.frame_id = self.world_frame
    #     p.header.stamp = self.get_clock().now().to_msg()
    #     p.pose.position.x = float(x)
    #     p.pose.position.y = float(y)
    #     p.pose.position.z = 0.0
    #     p.pose.orientation.w = 1.0
    #
    #     goal = Goal()
    #     goal.pose = p
    #
    #     if goal_type == "object":
    #         goal.type = 0
    #     elif goal_type == "box":
    #         goal.type = 1
    #     else:
    #         self.get_logger().error(f"Invalid goal_type: {goal_type}")
    #         return
    #
    #     self.goal_pub.publish(goal)
    #     self.get_logger().info(f"Published {goal_type} goal ({x:.2f}, {y:.2f})")

#     def publish_pose_to_path_planner(self, x: float, y: float):
#         p = PoseStamped()
#         p.header.frame_id = self.world_frame
#         p.header.stamp = self.get_clock().now().to_msg()
#         p.pose.position.x = x
#         p.pose.position.y = y
#         p.pose.position.z = 0.0
#         p.pose.orientation.w = 1.0
# 
#         self.goal_pub.publish(p)
#         self.get_logger().info(f"Published PoseStamped to path planner: ({x:.2f}, {y:.2f})")

    def publish_goal_to_path_planner(self, x: float, y: float, goal_type: str):
        p = PoseStamped()
        p.header.frame_id = self.world_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = 0.0
        p.pose.orientation.w = 1.0

        goal = GoalWithType()
        goal.goal = p

        if goal_type == "object":
            goal.type = GoalWithType.OBJECT
        elif goal_type == "box":
            goal.type = GoalWithType.BOX
        elif goal_type == "exploration_point":
            goal.type = GoalWithType.EXPLORATION_POINT
        else:
            self.get_logger().error(f"Invalid goal_type: {goal_type}")
            return

        self.goal_pub.publish(goal)
        self.get_logger().info(
            f"Published {goal_type} goal to path planner: ({x:.2f}, {y:.2f})"
        )






    def publish_path_to_controller(self, path: Path, goal_type: str):
        path_with_type = PathWithType()
        path_with_type.path = path

        if goal_type == "object":
            path_with_type.type = 0 
        elif goal_type == "box":
            path_with_type.type = 1 
        elif goal_type == "exploration_point":
            path_with_type.type = 2

        self.path_to_controller_pub.publish(path_with_type)
        self.path_to_controller_pub_viz.publish(path)

        # sys.exit()

        self.get_logger().info(f"Published {goal_type}") 

    # def enter_state(self, new_state: str):
    #     self.state = new_state
    #     self._published_this_state = False
    #     self.get_logger().info(f"State -> {new_state}")

    def enter_state(self, new_state: str):
        self.state = new_state
        self._published_this_state = False

        if new_state == "EXECUTE_EXPLORATION_PATH":
            self.execute_exploration_path_success = None
        elif new_state == "EXECUTE_PATH_TO_OBJECT":
            self.execute_path_object_success = None
        elif new_state == "EXECUTE_PATH_TO_BOX":
            self.execute_path_box_success = None

        self.get_logger().info(f"State -> {new_state}")

    def step(self):
        robot = self.lookup_xy(self.base_frame)
        if robot is None:
            return

        rx, ry = robot

        # Preempt exploration as soon as we know at least one object and one box
        exploration_states = (
            "GENERATE_EXPLORATION_POSE",
            "GENERATE_EXPLORATION_PATH",
            "EXECUTE_EXPLORATION_PATH",
        )

        # if (
        #     self.state in exploration_states
        #     and len(self.known_objects) > 0
        #     and len(self.known_boxes) > 0
        # ):
        #     self.enter_state("SELECT_OBJECT")
        #     return

        available_objects = [obj for obj in self.known_objects if obj.status == "detected"]

        if (
            self.state in exploration_states
            and len(available_objects) > 0
            and len(self.known_boxes) > 0
        ):
            # self.enter_state("SELECT_OBJECT")
            # return
            self.get_logger().warn("Preempting exploration because object and box are known")
            self.cancel_controller()
            self.execute_exploration_path_success = None
            self.enter_state("SELECT_OBJECT")
            return

        # ICP:
        if self.state == "UPDATE_ICP":

            # self.get_logger().info("UPDATE_ICP")
            if not self._published_this_state:
                self.ICP_pub.publish(String(data="update"))
                self._published_this_state = True
            if self.update_ICP_done:
                self.enter_state(self.state_after_update_icp)
                return

            # timeout case
            now = self.get_clock().now()
            elapsed = (now - self.update_ICP_start_time).nanoseconds * 1e-9

            if elapsed > self.update_ICP_timeout:
                self.get_logger().warn("ICP update timeout")
                self.enter_state(self.state_after_update_icp)
                return

            return

        elif self.state == "GENERATE_EXPLORATION_POSE":
            if not self._published_this_state:
                self.get_logger().info("inside GENERATE_EXPLORATION_POSE")
                self.exploration_pub.publish(String(data="Generate path"))
                self._published_this_state = True
                self.generate_exploration_pose_success = False

                print("generate exploration state")

            if self.generate_exploration_pose_success:
                self.enter_state("GENERATE_EXPLORATION_PATH")

        # elif self.state == "GENERATE_EXPLORATION_PATH":
        #     if not self._published_this_state:
        #         # self.get_logger().info("GENERATE_EXPLORATION_PATH")
        #         # self.publish_pose_to_path_planner(
        #         #     self.current_exploration_point.x,
        #         #     self.current_exploration_point.y,
        #         # )
        #         self.publish_goal_to_path_planner(
        #             self.current_exploration_point.x,
        #             self.current_exploration_point.y,
        #             goal_type="exploration_point"
        #         )
        #         self._published_this_state = True
        #         self.generate_exploration_path_success = False

        elif self.state == "GENERATE_EXPLORATION_PATH":
            if not self._published_this_state:
                self.publish_goal_to_path_planner(
                    self.current_exploration_point.x,
                    self.current_exploration_point.y,
                    goal_type="exploration_point"
                )

                self._published_this_state = True
                self.generate_exploration_path_success = False
                self.generate_exploration_path_failed = False

            if self.generate_exploration_path_success:
                self.start_update_icp("EXECUTE_EXPLORATION_PATH")
                return

            if self.generate_exploration_path_failed:
                if self.last_planner_status == "no_path":
                    self.get_logger().warn("Exploration point was not reachable, requesting a new one")
                    self.current_exploration_point = None
                    self.enter_state("GENERATE_EXPLORATION_POSE")
                if self.last_planner_status == "start_occupied" or self.last_planner_status == "start_out_of_bounds":
                    self.current_exploration_point = None
                    self.state_after_move_backward = "GENERATE_EXPLORATION_POSE"
                    self.enter_state("MOVE_BACKWARD")
                return


        elif self.state == "EXECUTE_EXPLORATION_PATH":
            if not self._published_this_state:
                # self.get_logger().info("EXECUTE_EXPLORATION_PATH")
                self.publish_path_to_controller(self.path_to_goal, goal_type="exploration_point")  # or exploration type if you add one
                self._published_this_state = True
                self.execute_exploration_path_success = None

            if self.execute_exploration_path_success is True:
                if len(self.known_objects) > 0 and len(self.known_boxes) > 0:
                    self.enter_state("SELECT_OBJECT")
                else:
                    self.enter_state("GENERATE_EXPLORATION_POSE")
                return

            elif self.execute_exploration_path_success is False:
                self.get_logger().warn("Exploration path blocked/cancelled, replanning same exploration")
                self.enter_state("GENERATE_EXPLORATION_PATH")
                return


        elif self.state == "SELECT_OBJECT":

#             if not self._published_this_state:
#                 self._not_published_this_state = True
#                 self.get_logger().info("SELECT_OBJECT")

            if len(self.known_objects) == 0 or len(self.known_boxes) == 0:
                self.enter_state("GENERATE_EXPLORATION_POSE")
                return

            available_objects = [obj for obj in self.known_objects if obj.status == "detected"]
            if len(self.known_boxes) == 0:
                self.enter_state("GENERATE_EXPLORATION_POSE")
                return

            if len(available_objects) == 0:
                self.enter_state("GENERATE_EXPLORATION_POSE")
                return

            self.current_object = min(
                available_objects,
                key=lambda obj: self.distance_sq(rx, ry, obj.x, obj.y)
            )

            self.current_box = min(
                self.known_boxes,
                key=lambda box: self.distance_sq(rx, ry, box.x, box.y)
            )

            self.ox = float(self.current_object.x)
            self.oy = float(self.current_object.y)
            self.bx = float(self.current_box.x)
            self.by = float(self.current_box.y)

            self.get_logger().info(
                f"Selected object id={self.current_object.id} at ({self.ox:.2f}, {self.oy:.2f}) "
                f"and box id={self.current_box.id} at ({self.bx:.2f}, {self.by:.2f})"
            )

            self.enter_state("GENERATE_PATH_TO_OBJECT")
            return

        if self.current_object is None and self.state not in ("SELECT_OBJECT", "DONE", "DROP_OBJECT", "MOVE_BACKWARD"):
            return

        
        if self.state == "GENERATE_PATH_TO_OBJECT": 
            if not self._published_this_state:
                # self.get_logger().info("GENERATE_PATH_TO_OBJECT")
                #self.publish_goal_xy(self.ox, self.oy, goal_type="object") # TODO: probably dont need goal type here
                # self.publish_pose_to_path_planner(self.ox, self.oy)
                self.get_logger().info(f"(ox,oy) = ({self.ox}, {self.oy})")
                self.publish_goal_to_path_planner(self.ox, self.oy, goal_type="object")
                self._published_this_state = True
                self.generate_path_object_success = None
                self.get_logger().info("GENERATE_PATH_TO_OBJECT: published object goal")

            if self.generate_path_object_success is True:
                # self.enter_state("EXECUTE_PATH_TO_OBJECT")
                self.start_update_icp("EXECUTE_PATH_TO_OBJECT")

            if self.generate_path_object_success is False:
                if self.last_planner_status in ("start_occupied", "start_out_of_bounds"):
                    self.state_after_move_backward = "GENERATE_PATH_TO_OBJECT"
                    self.enter_state("MOVE_BACKWARD")
                else:
                    self.get_logger().warn(f"GENERATE_PATH_TO_OBJECT: path failed status = {self.last_planner_status}")
                    self.enter_state("SELECT_OBJECT")

            



        elif self.state == "EXECUTE_PATH_TO_OBJECT": 

            if not self._published_this_state:
                # self.get_logger().info("EXECUTE_PATH_TO_OBJECT")
                self.publish_path_to_controller(path=self.path_to_goal, goal_type="object")
                self._published_this_state = True
                self.execute_path_object_success = None
                self.get_logger().info("EXECUTE_PATH_TO_OBJECT: published object path")

            if self.execute_path_object_success is True:
                self.enter_state("APPROACH_OBJECT")
                # self.enter_state("PICK_OBJECT")

            # if self.execute_path_object_success is False:
            #     self.current_object.status = "failed"
            #     self.enter_state("SELECT_OBJECT")

            # only scenario this becomes false is if the path is in collision course
            if self.execute_path_object_success is False:
                self.get_logger().warn("Object path blocked/cancelled, replanning same object")
                self.enter_state("GENERATE_PATH_TO_OBJECT")

        elif self.state == "APPROACH_OBJECT":
            if not self._published_this_state: 
                self._published_this_state = True
                approach_pose = PoseStamped()
                approach_pose.header.frame_id = self.world_frame
                approach_pose.pose.position.x = self.ox
                approach_pose.pose.position.y = self.oy
                approach_pose.pose.position.z = 0.0
                approach_pose.pose.orientation.w = 1.0

                approach_goal = GoalWithType()
                approach_goal.goal = approach_pose
                approach_goal.type = GoalWithType.OBJECT

                self.approach_goal_pub.publish(approach_goal)


            if self.approach_success:
                self.approach_success = False
                self.enter_state("PICK_OBJECT")


        elif self.state == "GENERATE_PATH_TO_BOX": 
            if not self._published_this_state:
                # self.get_logger().info("GENERATE_PATH_TO_BOX")
                # self.publish_goal_xy(self.bx, self.by) 
                #self.publish_pose_to_path_planner(self.bx, self.by)
                self.publish_goal_to_path_planner(self.bx, self.by, goal_type="box")
                self._published_this_state = True
                self.generate_path_box_success = None
                self.get_logger().info("GENERATE_PATH_TO_BOX: published object goal")

            if self.generate_path_box_success is True:
                # self.enter_state("EXECUTE_PATH_TO_BOX")
                self.start_update_icp("EXECUTE_PATH_TO_BOX")


            if self.generate_path_box_success is False:
                if self.last_planner_status in ("start_occupied", "start_out_of_bounds"):
                    self.state_after_move_backward = "GENERATE_PATH_TO_BOX"
                    self.enter_state("MOVE_BACKWARD")
                else:
                    self.get_logger().info(f"GENERATE_PATH_TO_BOX: path plannur status message: {self.last_planner_status}")
                    self.enter_state("SELECT_OBJECT")


        elif self.state == "EXECUTE_PATH_TO_BOX": 
            if not self._published_this_state:
                # self.get_logger().info("EXECUTE_PATH_TO_BOX")
                self.publish_path_to_controller(path=self.path_to_goal, goal_type="box")
                self._published_this_state = True
                self.execute_path_box_success = None
                self.get_logger().info("EXECUTE_PATH_TO_BOX: published object path")

            if self.execute_path_box_success is True:
                # self.enter_state("DROP_OBJECT")
                self.enter_state("APPROACH_BOX")

            if self.execute_path_box_success is False:
                self.get_logger().warn("Box path blocked/cancelled, replanning same box")
                self.enter_state("GENERATE_PATH_TO_BOX")


        elif self.state == "APPROACH_BOX":
            if not self._published_this_state: 
                self._published_this_state = True
                approach_pose = PoseStamped()
                approach_pose.header.frame_id = self.world_frame
                approach_pose.pose.position.x = self.bx
                approach_pose.pose.position.y = self.by
                approach_pose.pose.position.z = 0.0
                approach_pose.pose.orientation.w = 1.0

                approach_goal = GoalWithType()
                approach_goal.goal = approach_pose
                approach_goal.type = GoalWithType.BOX



                # self.approach_goal_pub.publish(approach_pose)
                self.approach_goal_pub.publish(approach_goal)

            if self.approach_success:
                self.approach_success = False
                self.enter_state("DROP_OBJECT")




        # if self.state == "NAV_TO_OBJECT":
        #     if not self._published_this_state:
        #         self.publish_goal_xy(self.ox, self.oy, goal_type="object")
        #         self._published_this_state = True
        #         self.nav_reached = False
        #         self.get_logger().info("NAV_TO_OBJECT: published object goal")
        #
        #     if self.nav_reached:
        #         self.enter_state("PICK_OBJECT")



        elif self.state == "PICK_OBJECT":
            if not self._published_this_state:
                self.pick_done = False
                self.arm_pub.publish(String(data="pick"))
                self._published_this_state = True
                self.get_logger().info("PICK_OBJECT: published pick")

            if self.pick_done:
                self.current_object.status = "picked"
                self.get_logger().info(
                    f"Marked picked object id={self.current_object.id} "
                    f"at ({self.ox:.2f}, {self.oy:.2f})"
                )
                # self.enter_state("GENERATE_PATH_TO_BOX")
                self.state_after_move_backward = "GENERATE_PATH_TO_BOX" 
                self.enter_state("MOVE_BACKWARD")

        # elif self.state == "NAV_TO_BOX":
        #     if not self._published_this_state:
        #         self.publish_goal_xy(self.bx, self.by, goal_type="box")
        #         self._published_this_state = True
        #         self.nav_reached = False
        #         self.get_logger().info("NAV_TO_BOX: published box goal")
        #
        #     if self.nav_reached:
        #         self.enter_state("DROP_OBJECT")

        elif self.state == "DROP_OBJECT":
            if not self._published_this_state:
                # self.get_logger().info("DROP_OBJECT")
                self.place_done = False
                self.arm_pub.publish(String(data="place"))
                self._published_this_state = True
                self.get_logger().info("DROP_OBJECT: published place")

            if self.place_done:
                self.place_done = False

                if self.current_object is not None:
                    self.current_object.status = "placed"

                # clear active task
                self.current_object = None
                self.current_box = None
                self.ox = None
                self.oy = None
                self.bx = None
                self.by = None

                # self.enter_state("DONE")
                self.state_after_move_backward = "DONE"
                self.enter_state("MOVE_BACKWARD")

        elif self.state == "MOVE_BACKWARD":
            if not self._published_this_state:
                self.get_logger().info("in MOVE_BACKWARD state")
                self._published_this_state = True
                self.move_backwards_success = False
                b = Bool()
                b.data = True
                self.move_backwards_pub.publish(b)

            if self.move_backwards_success:
                self.move_backwards_success = False
                self.get_logger().info(f"robot pos = ({rx}, {ry})")
                self.enter_state(self.state_after_move_backward)
                     
            




        elif self.state == "DONE":
            self.enter_state("SELECT_OBJECT")


def main():
    rclpy.init()
    node = TaskPlannerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
