#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped, PoseArray


class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__("task_planner_node")

        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("object_frame", "object_0")
        self.declare_parameter("box_frame", "box_0")
        self.declare_parameter("rate_hz", 5.0)

        # distance threshold for:
        # 1) deciding whether an object is already picked
        # 2) deciding whether a newly detected object is already known
        self.declare_parameter("picked_dist", 0.20)
        self.declare_parameter("known_dist", 0.15)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.object_frame = self.get_parameter("object_frame").value
        self.box_frame = self.get_parameter("box_frame").value
        self.picked_dist = float(self.get_parameter("picked_dist").value)
        self.known_dist = float(self.get_parameter("known_dist").value)

        # Current detections
        self.objects = []       # latest PoseArray objects
        self.boxes = []         # latest PoseArray boxes

        # Persistent memory
        self.known_objects = []   # remembered unpicked objects
        self.latest_box = None

        # Picked object positions
        self.picked_ids = []   # list of (x, y)

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

        # pubs
        self.goal_pub = self.create_publisher(PoseStamped, "/nav/goal", 10)
        self.phase_pub = self.create_publisher(String, "/nav/phase", 10)
        self.arm_pub = self.create_publisher(String, "/arm/cmd", 10)

        # subs
        self.create_subscription(Bool, "/nav/reached", self.on_reached, 10)
        self.create_subscription(String, "/arm/report_back", self.on_report_back, 10)
        self.create_subscription(PoseArray, "/detected_objects", self.on_objects, 10)
        self.create_subscription(PoseArray, "/detected_boxes", self.on_boxes, 10)

        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(dt, self.step)

        self.get_logger().info(
            "TaskPlannerNode up. "
            "Pub: /nav/goal, /nav/phase, /arm/cmd  "
            "Sub: /nav/reached, /arm/report_back, /detected_objects, /detected_boxes"
        )

    def _is_picked_xy(self, x: float, y: float) -> bool:
        thr2 = self.picked_dist * self.picked_dist
        for (px, py) in self.picked_ids:
            dx = x - px
            dy = y - py
            if dx * dx + dy * dy <= thr2:
                return True
        return False

    def _is_known_xy(self, x: float, y: float) -> bool:
        thr2 = self.known_dist * self.known_dist
        for p in self.known_objects:
            dx = float(p.position.x) - x
            dy = float(p.position.y) - y
            if dx * dx + dy * dy <= thr2:
                return True
        return False

    def on_objects(self, msg: PoseArray):
        # latest detections only
        self.objects = list(msg.poses)

        for p in msg.poses:
            x = float(p.position.x)
            y = float(p.position.y)

            # do not remember picked objects
            if self._is_picked_xy(x, y):
                continue

            # do not duplicate remembered objects
            if self._is_known_xy(x, y):
                continue

            self.known_objects.append(p)
            self.get_logger().info(f"Remembered new object at ({x:.2f}, {y:.2f})")

    def on_boxes(self, msg: PoseArray):
        self.boxes = list(msg.poses)
        self.latest_box = self.boxes[0] if len(self.boxes) > 0 else None

    def on_report_back(self, msg: String):
        self.get_logger().info(f"on_report_back: {msg.data}")

        if msg.data == "pick_success":
            self.pick_done = True

        elif msg.data == "pick_fail":
            self.pick_done = False
            # easiest behavior: abandon current object and try next one
            self.current_object = None
            self.enter_state("SELECT_OBJECT")

        elif msg.data == "place_success":
            self.place_done = True

        elif msg.data == "place_fail":
            self.place_done = False
            # easiest behavior: retry by going back to box state
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
        return (t.x, t.y)

    def publish_goal_xy(self, x: float, y: float):
        g = PoseStamped()
        g.header.frame_id = self.world_frame
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = float(x)
        g.pose.position.y = float(y)
        g.pose.position.z = 0.0
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)

        self.get_logger().info(f"Published goal ({x:.2f}, {y:.2f})")

    def enter_state(self, new_state: str):
        self.state = new_state
        self._published_this_state = False
        self.get_logger().info(f"State -> {new_state}")

    def step(self):
        robot = self.lookup_xy(self.base_frame)
        if robot is None:
            return

        if self.state == "SELECT_OBJECT":
            if len(self.known_objects) == 0 or len(self.boxes) == 0:
                return

            # FIFO queue: pick oldest remembered object
            self.current_object = self.known_objects.pop(0)
            self.current_box = self.boxes[0]

            self.ox = float(self.current_object.position.x)
            self.oy = float(self.current_object.position.y)
            self.bx = float(self.current_box.position.x)
            self.by = float(self.current_box.position.y)

            self.get_logger().info(
                f"Selected object ({self.ox:.2f}, {self.oy:.2f}) "
                f"-> box ({self.bx:.2f}, {self.by:.2f})"
            )

            self.enter_state("NAV_TO_OBJECT")
            return

        if self.current_object is None and self.state not in ("SELECT_OBJECT", "DONE", "DROP_OBJECT"):
            return

        if self.state == "NAV_TO_OBJECT":
            if not self._published_this_state:
                self.phase_pub.publish(String(data="object"))
                self.publish_goal_xy(self.ox, self.oy)
                self._published_this_state = True
                self.nav_reached = False
                self.get_logger().info("NAV_TO_OBJECT: published object goal")

            if self.nav_reached:
                self.enter_state("PICK_OBJECT")

        elif self.state == "PICK_OBJECT":
            if not self._published_this_state:
                self.pick_done = False
                self.arm_pub.publish(String(data="pick"))
                self._published_this_state = True
                self.get_logger().info("PICK_OBJECT: published pick")

            if self.pick_done:
                self.picked_ids.append((self.ox, self.oy))
                self.get_logger().info(f"Marked picked object at ({self.ox:.2f}, {self.oy:.2f})")
                self.enter_state("NAV_TO_BOX")

        elif self.state == "NAV_TO_BOX":
            if not self._published_this_state:
                self.phase_pub.publish(String(data="box"))
                self.publish_goal_xy(self.bx, self.by)
                self._published_this_state = True
                self.nav_reached = False
                self.get_logger().info("NAV_TO_BOX: published box goal")

            if self.nav_reached:
                self.enter_state("DROP_OBJECT")

        elif self.state == "DROP_OBJECT":
            if not self._published_this_state:
                self.place_done = False
                self.arm_pub.publish(String(data="place"))
                self._published_this_state = True
                self.get_logger().info("DROP_OBJECT: published place")

                # done with this object
                self.current_object = None

            if self.place_done:
                self.place_done = False
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
