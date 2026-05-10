#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
import tf2_ros

from std_msgs.msg import Bool, String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray, Point

from grumpy_interfaces.msg import GoalWithType, PathWithType, PathWithStatus


class TrackedObject:
    def __init__(self, obj_id: int, x: float, y: float):
        self.id = obj_id
        self.x = x
        self.y = y
        self.status = "detected"


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

        self.start_position = None
        self.sx = None
        self.sy = None
        self.start_yaw = None

        self.next_object_id = 0
        self.next_box_id = 0

        self.known_objects = []
        self.known_boxes = []

        self.current_object = None
        self.current_box = None
        self.current_exploration_point = None

        self.ox = None
        self.oy = None
        self.bx = None
        self.by = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.generate_exploration_pose_success = False
        self.generate_exploration_path_success = False
        self.execute_exploration_path_success = None
        self.generate_exploration_path_failed = False

        self.generate_path_object_success = None
        self.execute_path_object_success = None

        self.generate_path_box_success = None
        self.execute_path_box_success = None

        self.generate_path_start_success = None
        self.execute_path_start_success = None

        self.approach_success = False
        self.move_backwards_success = False
        self.rotate_success = False

        self.state_after_move_backward = None
        self.last_planner_status = None

        self.state = "GENERATE_EXPLORATION_POSE"
        self.pick_done = False
        self.place_done = False
        self._published_this_state = False

        self.path_to_goal = None

        self.started = False

        self.goal_pub = self.create_publisher(GoalWithType, "/nav/goal", 10)
        self.path_to_controller_pub = self.create_publisher(PathWithType, "/nav/path_to_controller", 10)
        self.path_to_controller_pub_viz = self.create_publisher(Path, "/nav/path_to_controller_viz", 10)
        self.approach_goal_pub = self.create_publisher(GoalWithType, "/nav/approach_start", 10)
        self.path_blocked_pub = self.create_publisher(Bool, "/nav/path_blocked", 10)
        self.object_success_pub = self.create_publisher(Point, "/Success", 10)
        self.object_failure_pub = self.create_publisher(Point, "/Failure", 10)

        self.arm_pub = self.create_publisher(String, "/arm/cmd", 10)
        self.move_backwards_pub = self.create_publisher(Bool, "/nav/move_backwards_start", 10)

        # New rotate-to-start-yaw interface
        self.rotate_start_pub = self.create_publisher(PoseStamped, "/nav/rotate_to_pose", 10)

        self.exploration_pub = self.create_publisher(String, "/exploration/request_unexplored_point", 10)

        self.create_subscription(Point, "/exploration/return_unexplored_point", self.on_exploration_point, 10)
        self.create_subscription(Bool, "/nav/reached", self.on_reached, 10)
        self.create_subscription(PathWithStatus, "/nav/path_from_planner", self.on_path_from_planner, 10)
        self.create_subscription(String, "/arm/report_back", self.on_report_back, 10)
        self.create_subscription(PoseArray, "/detected_objects", self.on_objects, 10)
        self.create_subscription(PoseArray, "/detected_boxes", self.on_boxes, 10)
        self.create_subscription(PoseStamped, "/nav/approach_finished", self.on_approach_finished, 10)
        self.create_subscription(Bool, "/nav/move_backwards_finished", self.on_move_backwards_finished, 10)
        self.create_subscription(Bool, "/nav/rotate_finished", self.on_rotate_finished, 10)

        self.ICP_pub = self.create_publisher(String, "/localization/start_update_ICP", 10)
        self.create_subscription(String, "/localization/finished_update_ICP", self.on_finished_update_ICP, 10)

        self.create_subscription(Bool, "/task_planner/start", self.on_start, 10)

        self.state_after_update_icp = None
        self.update_ICP_done = False
        self.update_ICP_start_time = None
        self.update_ICP_timeout = 2.0
        self.update_ICP_command = None

        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.step_timer = self.create_timer(dt, self.step)

        self.get_logger().info("TaskPlannerNode up.")

    def on_start(self, msg: Bool):
        if msg.data:
            self.started = True
            self.get_logger().info("Task planner started")

    def cancel_controller(self):
        self.path_blocked_pub.publish(Bool(data=True))

    def on_move_backwards_finished(self, msg):
        if msg.data:
            self.move_backwards_success = True

    def on_rotate_finished(self, msg):
        if msg.data:
            self.rotate_success = True

    def on_approach_finished(self, msg):
        self.approach_success = True

    def on_finished_update_ICP(self, msg):
        self.update_ICP_done = True

    def start_update_icp(self, command: str, next_state: str):
        self.update_ICP_command = command
        self.state_after_update_icp = next_state
        self.update_ICP_start_time = self.get_clock().now()
        self.update_ICP_done = False
        self.enter_state("UPDATE_ICP")

    def on_exploration_point(self, msg: Point):
        self.current_exploration_point = msg
        self.generate_exploration_pose_success = True

    def on_path_from_planner(self, msg: PathWithStatus):
        success = msg.status == "success" and len(msg.path.poses) > 0
        self.last_planner_status = msg.status

        if self.state == "GENERATE_PATH_TO_OBJECT":
            self.generate_path_object_success = success
        elif self.state == "GENERATE_PATH_TO_BOX":
            self.generate_path_box_success = success
        elif self.state == "GENERATE_EXPLORATION_PATH":
            self.generate_exploration_path_success = success
            self.generate_exploration_path_failed = not success
        elif self.state == "GENERATE_PATH_TO_START":
            self.generate_path_start_success = success

        if success:
            self.path_to_goal = msg.path
        else:
            self.path_to_goal = None
            self.get_logger().warn(
                f"Empty path received in state {self.state}, status={msg.status}"
            )

    def distance_sq(self, x1, y1, x2, y2):
        dx = x1 - x2
        dy = y1 - y2
        return dx * dx + dy * dy

    def on_objects(self, msg: PoseArray):
        for p in msg.poses:
            x = float(p.position.x)
            y = float(p.position.y)
            tracked_obj = TrackedObject(self.next_object_id, x, y)
            self.next_object_id += 1
            self.known_objects.append(tracked_obj)

    def on_boxes(self, msg: PoseArray):
        for p in msg.poses:
            x = float(p.position.x)
            y = float(p.position.y)
            tracked_box = TrackedBox(self.next_box_id, x, y)
            self.next_box_id += 1
            self.known_boxes.append(tracked_box)

    def on_report_back(self, msg: String):
        self.get_logger().info(f"on_report_back: {msg.data}")

        if self.current_object is None:
            self.get_logger().warn("Arm reported back but current_object is None")
            return

        p = Point()
        p.x = float(self.current_object.x)
        p.y = float(self.current_object.y)
        p.z = 0.0

        if msg.data == "pick_success":
            self.object_success_pub.publish(p)
            self.pick_done = True
            self.current_object.status = "picked"

        elif msg.data == "pick_fail":
            self.object_failure_pub.publish(p)
            self.pick_done = False
            self.current_object.status = "failed"
            self.current_object = None
            self.enter_state("SELECT_OBJECT")

        elif msg.data == "place_success":
            self.place_done = True

        elif msg.data == "place_fail":
            self.place_done = False
            self.enter_state("GENERATE_PATH_TO_OBJECT")

    def on_reached(self, msg: Bool):
        nav_reached = bool(msg.data)
        self.get_logger().info(f"on_reached: {nav_reached}")

        if self.state == "EXECUTE_PATH_TO_OBJECT":
            self.execute_path_object_success = nav_reached
        elif self.state == "EXECUTE_PATH_TO_BOX":
            self.execute_path_box_success = nav_reached
        elif self.state == "EXECUTE_EXPLORATION_PATH":
            self.execute_exploration_path_success = nav_reached
        elif self.state == "EXECUTE_PATH_TO_START":
            self.execute_path_start_success = nav_reached

    def yaw_from_quat(self, q):
        return math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

    def lookup_pose_2d(self, target_frame: str):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.world_frame,
                target_frame,
                rclpy.time.Time()
            )
        except Exception:
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = self.yaw_from_quat(q)

        return float(t.x), float(t.y), yaw

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
        elif goal_type == "start_position":
            goal.type = GoalWithType.EXPLORATION_POINT
        else:
            self.get_logger().error(f"Invalid goal_type: {goal_type}")
            return

        self.goal_pub.publish(goal)
        self.get_logger().info(
            f"Published {goal_type} goal: ({x:.2f}, {y:.2f})"
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
        elif goal_type == "start_position":
            path_with_type.type = 2
        else:
            self.get_logger().error(f"Invalid path goal_type: {goal_type}")
            return

        self.path_to_controller_pub.publish(path_with_type)
        self.path_to_controller_pub_viz.publish(path)

        self.get_logger().info(f"Published path to controller: {goal_type}")

    def enter_state(self, new_state: str):
        self.state = new_state
        self._published_this_state = False

        if new_state == "EXECUTE_EXPLORATION_PATH":
            self.execute_exploration_path_success = None
        elif new_state == "EXECUTE_PATH_TO_OBJECT":
            self.execute_path_object_success = None
        elif new_state == "EXECUTE_PATH_TO_BOX":
            self.execute_path_box_success = None
        elif new_state == "EXECUTE_PATH_TO_START":
            self.execute_path_start_success = None
        elif new_state == "APPROACH_OBJECT":
            self.approach_success = False
        elif new_state == "APPROACH_BOX":
            self.approach_success = False
        elif new_state == "ROTATE_TO_START_YAW":
            self.rotate_success = False

        self.get_logger().info(f"State -> {new_state}")

    def carrying_object(self):
        return self.current_object is not None and self.current_object.status == "picked"

    def step(self):
        if not self.started:
            return

        robot = self.lookup_pose_2d(self.base_frame)
        if robot is None:
            return

        rx, ry, ryaw = robot

        if self.start_position is None:
            self.start_position = (rx, ry, ryaw)
            self.sx = rx
            self.sy = ry
            self.start_yaw = ryaw

            self.get_logger().info(
                f"Saved start pose: ({self.sx:.2f}, {self.sy:.2f}, yaw={self.start_yaw:.2f})"
            )

            self.start_update_icp("start", "GENERATE_EXPLORATION_POSE")
            return

        if self.state == "UPDATE_ICP":
            if not self._published_this_state:
                self.get_logger().info(
                    f"UPDATE_ICP: publishing '{self.update_ICP_command}'"
                )
                self.ICP_pub.publish(String(data=self.update_ICP_command))
                self._published_this_state = True

            if self.update_ICP_done:
                self.update_ICP_done = False
                self.enter_state(self.state_after_update_icp)
                return

            now = self.get_clock().now()
            elapsed = (now - self.update_ICP_start_time).nanoseconds * 1e-9

            if elapsed > self.update_ICP_timeout:
                self.get_logger().warn(
                    f"ICP timeout for command '{self.update_ICP_command}'"
                )
                self.enter_state(self.state_after_update_icp)
                return

            return

        exploration_states = (
            "GENERATE_EXPLORATION_POSE",
            "GENERATE_EXPLORATION_PATH",
            "EXECUTE_EXPLORATION_PATH",
        )

        available_objects = [
            obj for obj in self.known_objects
            if obj.status == "detected"
        ]

        if (
            self.state in exploration_states
            and len(available_objects) > 0
            and len(self.known_boxes) > 0
        ):
            self.get_logger().warn(
                "Preempting exploration because object and box are known"
            )
            self.cancel_controller()
            self.execute_exploration_path_success = None
            self.start_update_icp("correct", "SELECT_OBJECT")
            return

        if self.state == "GENERATE_EXPLORATION_POSE":
            if not self._published_this_state:
                self.exploration_pub.publish(String(data="Generate path"))
                self._published_this_state = True
                self.generate_exploration_pose_success = False

            if self.generate_exploration_pose_success:
                self.enter_state("GENERATE_EXPLORATION_PATH")
                return

        elif self.state == "GENERATE_EXPLORATION_PATH":
            if not self._published_this_state:
                if self.current_exploration_point is None:
                    self.enter_state("GENERATE_EXPLORATION_POSE")
                    return

                self.publish_goal_to_path_planner(
                    self.current_exploration_point.x,
                    self.current_exploration_point.y,
                    goal_type="exploration_point"
                )

                self._published_this_state = True
                self.generate_exploration_path_success = False
                self.generate_exploration_path_failed = False

            if self.generate_exploration_path_success:
                self.enter_state("EXECUTE_EXPLORATION_PATH")
                return

            if self.generate_exploration_path_failed:
                if self.last_planner_status == "no_path":
                    self.current_exploration_point = None

                    if self.carrying_object():
                        self.enter_state("SELECT_BOX")
                    else:
                        self.enter_state("GENERATE_EXPLORATION_POSE")
                    return

                if self.last_planner_status in ("start_occupied", "start_out_of_bounds"):
                    self.current_exploration_point = None
                    self.state_after_move_backward = "GENERATE_EXPLORATION_POSE"
                    self.enter_state("MOVE_BACKWARD")
                    return

        elif self.state == "EXECUTE_EXPLORATION_PATH":
            if not self._published_this_state:
                self.publish_path_to_controller(
                    self.path_to_goal,
                    goal_type="exploration_point"
                )
                self._published_this_state = True
                self.execute_exploration_path_success = None

            if self.execute_exploration_path_success is True:
                if len(self.known_objects) > 0 and len(self.known_boxes) > 0:
                    self.start_update_icp("correct", "SELECT_OBJECT")
                else:
                    if self.carrying_object():
                        self.start_update_icp("correct", "SELECT_BOX")
                    else:
                        self.start_update_icp("correct", "GENERATE_EXPLORATION_POSE")
                return

            if self.execute_exploration_path_success is False:
                self.enter_state("GENERATE_EXPLORATION_PATH")
                return

        elif self.state == "SELECT_OBJECT":
            available_objects = [
                obj for obj in self.known_objects
                if obj.status == "detected"
            ]

            if len(available_objects) == 0:
                if self.carrying_object():
                    self.enter_state("SELECT_BOX")
                else:
                    self.enter_state("GENERATE_EXPLORATION_POSE")
                return

            self.current_object = min(
                available_objects,
                key=lambda obj: self.distance_sq(rx, ry, obj.x, obj.y)
            )

            self.ox = float(self.current_object.x)
            self.oy = float(self.current_object.y)

            self.get_logger().info(
                f"Selected object id={self.current_object.id} "
                f"at ({self.ox:.2f}, {self.oy:.2f})"
            )

            self.enter_state("GENERATE_PATH_TO_OBJECT")
            return

        elif self.state == "SELECT_BOX":
            if self.current_object is None:
                self.enter_state("SELECT_OBJECT")
                return

            if len(self.known_boxes) == 0:
                self.get_logger().warn("Carrying object but no box known yet")
                return

            self.current_box = min(
                self.known_boxes,
                key=lambda box: self.distance_sq(rx, ry, box.x, box.y)
            )

            self.bx = float(self.current_box.x)
            self.by = float(self.current_box.y)

            self.get_logger().info(
                f"Selected box id={self.current_box.id} "
                f"at ({self.bx:.2f}, {self.by:.2f})"
            )

            self.enter_state("GENERATE_PATH_TO_BOX")
            return

        if self.current_object is None and self.state not in (
            "SELECT_OBJECT",
            "SELECT_BOX",
            "DONE",
            "DROP_OBJECT",
            "MOVE_BACKWARD",
            "GENERATE_PATH_TO_START",
            "EXECUTE_PATH_TO_START",
            "ROTATE_TO_START_YAW",
        ):
            return

        if self.state == "GENERATE_PATH_TO_OBJECT":
            if not self._published_this_state:
                self.publish_goal_to_path_planner(
                    self.ox,
                    self.oy,
                    goal_type="object"
                )
                self._published_this_state = True
                self.generate_path_object_success = None

            if self.generate_path_object_success is True:
                self.start_update_icp("pre", "EXECUTE_PATH_TO_OBJECT")
                return

            if self.generate_path_object_success is False:
                if self.last_planner_status in ("start_occupied", "start_out_of_bounds"):
                    self.state_after_move_backward = "GENERATE_PATH_TO_OBJECT"
                    self.enter_state("MOVE_BACKWARD")
                else:
                    self.enter_state("SELECT_OBJECT")
                return

        elif self.state == "EXECUTE_PATH_TO_OBJECT":
            if not self._published_this_state:
                self.publish_path_to_controller(
                    path=self.path_to_goal,
                    goal_type="object"
                )
                self._published_this_state = True
                self.execute_path_object_success = None

            if self.execute_path_object_success is True:
                self.start_update_icp("correct", "APPROACH_OBJECT")
                return

            if self.execute_path_object_success is False:
                self.enter_state("GENERATE_PATH_TO_OBJECT")
                return

        elif self.state == "APPROACH_OBJECT":
            if not self._published_this_state:
                approach_pose = PoseStamped()
                approach_pose.header.frame_id = self.world_frame
                approach_pose.header.stamp = self.get_clock().now().to_msg()
                approach_pose.pose.position.x = self.ox
                approach_pose.pose.position.y = self.oy
                approach_pose.pose.position.z = 0.0
                approach_pose.pose.orientation.w = 1.0

                approach_goal = GoalWithType()
                approach_goal.goal = approach_pose
                approach_goal.type = GoalWithType.OBJECT

                self.approach_goal_pub.publish(approach_goal)
                self._published_this_state = True

            if self.approach_success:
                self.approach_success = False
                self.enter_state("PICK_OBJECT")
                return

        elif self.state == "PICK_OBJECT":
            if not self._published_this_state:
                self.pick_done = False
                self.arm_pub.publish(String(data="pick"))
                self._published_this_state = True

            if self.pick_done:
                self.state_after_move_backward = "GENERATE_PATH_TO_START"
                self.enter_state("MOVE_BACKWARD")
                return

        elif self.state == "GENERATE_PATH_TO_START":
            if self.start_position is None:
                self.enter_state("SELECT_BOX")
                return

            if not self._published_this_state:
                self.publish_goal_to_path_planner(
                    self.sx,
                    self.sy,
                    goal_type="start_position"
                )
                self._published_this_state = True
                self.generate_path_start_success = None

            if self.generate_path_start_success is True:
                self.start_update_icp("pre", "EXECUTE_PATH_TO_START")
                return

            if self.generate_path_start_success is False:
                if self.last_planner_status in ("start_occupied", "start_out_of_bounds"):
                    self.state_after_move_backward = "GENERATE_PATH_TO_START"
                    self.enter_state("MOVE_BACKWARD")
                    return

                self.enter_state("SELECT_BOX")
                return

        elif self.state == "EXECUTE_PATH_TO_START":
            if not self._published_this_state:
                self.publish_path_to_controller(
                    path=self.path_to_goal,
                    goal_type="start_position"
                )
                self._published_this_state = True
                self.execute_path_start_success = None

            if self.execute_path_start_success is True:
                self.start_update_icp("correct", "ROTATE_TO_START_YAW")
                return

            if self.execute_path_start_success is False:
                self.enter_state("GENERATE_PATH_TO_START")
                return

        elif self.state == "ROTATE_TO_START_YAW":
            if self.start_yaw is None:
                self.get_logger().warn("No start yaw saved, continuing to SELECT_BOX")
                self.enter_state("SELECT_BOX")
                return

            if not self._published_this_state:
                self.rotate_success = False

                pose = PoseStamped()
                pose.header.frame_id = self.world_frame
                pose.header.stamp = self.get_clock().now().to_msg()
                pose.pose.position.x = self.sx
                pose.pose.position.y = self.sy
                pose.pose.position.z = 0.0
                pose.pose.orientation.z = math.sin(self.start_yaw / 2.0)
                pose.pose.orientation.w = math.cos(self.start_yaw / 2.0)

                self.rotate_start_pub.publish(pose)
                self._published_this_state = True

                self.get_logger().info(
                    f"ROTATE_TO_START_YAW: target yaw={self.start_yaw:.2f}"
                )

            if self.rotate_success:
                self.rotate_success = False
                self.enter_state("SELECT_BOX")
                return

        elif self.state == "GENERATE_PATH_TO_BOX":
            if not self._published_this_state:
                self.publish_goal_to_path_planner(
                    self.bx,
                    self.by,
                    goal_type="box"
                )
                self._published_this_state = True
                self.generate_path_box_success = None

            if self.generate_path_box_success is True:
                self.start_update_icp("pre", "EXECUTE_PATH_TO_BOX")
                return

            if self.generate_path_box_success is False:
                if self.last_planner_status in ("start_occupied", "start_out_of_bounds"):
                    self.state_after_move_backward = "GENERATE_PATH_TO_BOX"
                    self.enter_state("MOVE_BACKWARD")
                else:
                    if self.carrying_object():
                        self.enter_state("SELECT_BOX")
                    else:
                        self.enter_state("SELECT_OBJECT")
                return

        elif self.state == "EXECUTE_PATH_TO_BOX":
            if not self._published_this_state:
                self.publish_path_to_controller(
                    path=self.path_to_goal,
                    goal_type="box"
                )
                self._published_this_state = True
                self.execute_path_box_success = None

            if self.execute_path_box_success is True:
                self.start_update_icp("correct", "APPROACH_BOX")
                return

            if self.execute_path_box_success is False:
                self.enter_state("GENERATE_PATH_TO_BOX")
                return

        elif self.state == "APPROACH_BOX":
            if not self._published_this_state:
                approach_pose = PoseStamped()
                approach_pose.header.frame_id = self.world_frame
                approach_pose.header.stamp = self.get_clock().now().to_msg()
                approach_pose.pose.position.x = self.bx
                approach_pose.pose.position.y = self.by
                approach_pose.pose.position.z = 0.0
                approach_pose.pose.orientation.w = 1.0

                approach_goal = GoalWithType()
                approach_goal.goal = approach_pose
                approach_goal.type = GoalWithType.BOX

                self.approach_goal_pub.publish(approach_goal)
                self._published_this_state = True

            if self.approach_success:
                self.approach_success = False
                self.enter_state("DROP_OBJECT")
                return

        elif self.state == "DROP_OBJECT":
            if not self._published_this_state:
                self.place_done = False
                self.arm_pub.publish(String(data="place"))
                self._published_this_state = True

            if self.place_done:
                self.place_done = False

                if self.current_object is not None:
                    self.current_object.status = "placed"

                self.current_object = None
                self.current_box = None
                self.ox = None
                self.oy = None
                self.bx = None
                self.by = None

                self.state_after_move_backward = "DONE"
                self.enter_state("MOVE_BACKWARD")
                return

        elif self.state == "MOVE_BACKWARD":
            if not self._published_this_state:
                self.move_backwards_success = False
                self.move_backwards_pub.publish(Bool(data=True))
                self._published_this_state = True

            if self.move_backwards_success:
                self.move_backwards_success = False
                self.enter_state(self.state_after_move_backward)
                return

        elif self.state == "DONE":
            self.enter_state("SELECT_OBJECT")
            return


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
