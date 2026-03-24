#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
import tf2_ros

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, PoseArray, Path, Point

from grumpy_interfaces.msg import Goal, PathWithType


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

        self.ox = None
        self.oy = None
        self.bx = None
        self.by = None

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Planner state
        self.state = "SELECT_OBJECT"
        self.nav_reached = False
        self.pick_done = False
        self.place_done = False
        self._published_this_state = False

        # Publishers
        self.goal_pub = self.create_publisher(Goal, "/nav/goal", 10)
        self.path_pub = self.create_publisher(PathWithType, "/nav/path", 10)
        self.arm_pub = self.create_publisher(String, "/arm/cmd", 10)

        # create pub and sub
        # topics: 
        self.exploration_pub = self.create_publisher(String, "/exploration/request_unexplored_point", 10) # content can be anything

        # TODO: create callback for this
        self.create_subscription(Point, "/exploration/return_unexplored_point", self.on_exploration_point, 10) # z value irrelevant

        # Subscribers
        self.create_subscription(Bool, "/nav/reached", self.on_reached, 10)
        self.create_subscription(String, "/arm/report_back", self.on_report_back, 10)
        self.create_subscription(PoseArray, "/detected_objects", self.on_objects, 10)
        self.create_subscription(PoseArray, "/detected_boxes", self.on_boxes, 10)


        #TODO: create ICP state and callback for these:
        self.ICP_pub = self.create_publisher(String, "/localization/start_update_ICP", 10)  # contant can be anything
        self.create_subscription(String, "/localization/finished_update_ICP", self.on_finshed_update_ICP, 10)  # contant can be anything


        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(dt, self.step)

        self.get_logger().info(
            "TaskPlannerNode up. "
            "Pub: /nav/goal, /arm/cmd  "
            "Sub: /nav/reached, /arm/report_back, /detected_objects, /detected_boxes"
        )

    def distance_sq(self, x1: float, y1: float, x2: float, y2: float) -> float:
        dx = x1 - x2
        dy = y1 - y2
        return dx * dx + dy * dy

    def on_objects(self, msg: PoseArray):
        # Replace with latest tracked objects from detection
        self.known_objects = []

        for p in msg.poses:
            x = float(p.position.x)
            y = float(p.position.y)

            tracked_obj = TrackedObject(self.next_object_id, x, y)
            self.next_object_id += 1
            self.known_objects.append(tracked_obj)

        self.get_logger().info(f"Updated objects: {len(self.known_objects)}")

    def on_boxes(self, msg: PoseArray):
        # Replace with latest tracked boxes from detection
        self.known_boxes = []

        for p in msg.poses:
            x = float(p.position.x)
            y = float(p.position.y)

            tracked_box = TrackedBox(self.next_box_id, x, y)
            self.next_box_id += 1
            self.known_boxes.append(tracked_box)

        self.get_logger().info(f"Updated boxes: {len(self.known_boxes)}")

    def on_report_back(self, msg: String):
        self.get_logger().info(f"on_report_back: {msg.data}")

        if msg.data == "pick_success":
            self.pick_done = True

        elif msg.data == "pick_fail":
            self.pick_done = False
            self.current_object = None
            self.enter_state("SELECT_OBJECT")

        elif msg.data == "place_success":
            self.place_done = True

        elif msg.data == "place_fail":
            self.place_done = False
            self.enter_state("NAV_TO_BOX")

    def on_reached(self, msg: Bool):
        self.nav_reached = bool(msg.data)

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

    def publish_goal_xy(self, x: float, y: float, goal_type: str):
        p = PoseStamped()
        p.header.frame_id = self.world_frame
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.position.x = float(x)
        p.pose.position.y = float(y)
        p.pose.position.z = 0.0
        p.pose.orientation.w = 1.0

        goal = Goal()
        goal.pose = p

        if goal_type == "object":
            goal.type = 0
        elif goal_type == "box":
            goal.type = 1
        else:
            self.get_logger().error(f"Invalid goal_type: {goal_type}")
            return

        self.goal_pub.publish(goal)
        self.get_logger().info(f"Published {goal_type} goal ({x:.2f}, {y:.2f})")

    def publish_path_to_controller(self, path: Path, goal_type: str):
        path_to_goal = PathWithType()
        if goal_type == "object":
            path_to_goal.type = 0 
        elif goal_type == "box":
            path_to_goal.type = 1 

        self.path_pub.publish(path)
        self.get_logger().info(f"Published {goal_type} goal ({x:.2f}, {y:.2f})")

    def enter_state(self, new_state: str):
        self.state = new_state
        self._published_this_state = False
        self.get_logger().info(f"State -> {new_state}")

    def step(self):
        robot = self.lookup_xy(self.base_frame)
        if robot is None:
            return

        rx, ry = robot

        if self.state == "SELECT_OBJECT":
            if len(self.known_objects) == 0 or len(self.known_boxes) == 0:
                return

            # choose closest object to robot
            self.current_object = min(
                self.known_objects,
                key=lambda obj: self.distance_sq(rx, ry, obj.x, obj.y)
            )

            # choose closest box to robot
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

            self.enter_state("NAV_TO_OBJECT")
            return

        if self.current_object is None and self.state not in ("SELECT_OBJECT", "DONE", "DROP_OBJECT"):
            return


        # TODO: create these new states
        # GENERATE_PATH_TO_OBJECT
        # EXECUTE_PATH_TO_OBJECT
        # GENERATE_PATH_TO_BOX
        # EXECUTE_PATH_TO_BOX

        # TODO: next create for box as well

        # TODO: 
        # states: 
        # genereate exploration pose 
        # execute exploration pose 
        # 
        
        if self.state == "GENERATE_PATH_TO_OBJECT": 
            if not self._published_this_state:
                #self.publish_goal_xy(self.ox, self.oy, goal_type="object") # TODO: probably dont need goal type here
                self.publish_goal_xy() # TODO: probably dont need goal type here
                self._published_this_state = True
                self.generate_path_object_success = False
                self.get_logger().info("GENERATE_PATH_TO_OBJECT: published object goal")

            if self.generate_path_object_success:
                self.enter_state("EXECUTE_PATH_TO_OBJECT")

        if self.state == "EXECUTE_PATH_TO_OBJECT": 
            if not self._published_this_state:
                self.publish_path_to_controller(path=self.path_to_goal, goal_type="object")
                self._published_this_state = True
                self.execute_path_object_success = False
                self.get_logger().info("GENERATE_PATH_TO_OBJECT: published object goal")

            if self.execute_path_object_success:
                self.enter_state("PICK_OBJECT")


        if self.state == "GENERATE_PATH_TO_BOX": 
            if not self._published_this_state:
                self.publish_goal_xy(self.bx, self.by) 
                self._published_this_state = True
                self.generate_path_box_success = False
                self.get_logger().info("GENERATE_PATH_TO_BOX: published object goal")

            if self.generate_path_box_success:
                self.enter_state("EXECUTE_PATH_TO_BOX")


        if self.state == "EXECUTE_PATH_TO_BOX": 
            if not self._published_this_state:
                self.publish_path_to_controller(path=self.path_to_goal, goal_type="box")
                self._published_this_state = True
                self.execute_path_box_success = False
                self.get_logger().info("GENERATE_PATH_TO_BOX: published object goal")

            if self.execute_path_box_success:
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
                self.enter_state("GENERATE_PATH_TO_BOX")

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

                self.enter_state("DONE")

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
