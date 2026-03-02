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

        # NEW: distance threshold (meters) to consider an object "already picked"
        self.declare_parameter("picked_dist", 0.25)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.object_frame = self.get_parameter("object_frame").value
        self.box_frame = self.get_parameter("box_frame").value
        self.picked_dist = float(self.get_parameter("picked_dist").value)

        self.latest_object = None
        self.latest_box = None

        # CHANGED: store picked (x,y) instead of IDs
        self.picked_ids = []  # list[(x,y)]

        self.current_object = None
        self.current_box = None

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Planner state
        self.state = "SELECT_OBJECT"
        self.nav_reached = False
        self.pick_done = False
        self._published_this_state = False

        # pubs
        self.goal_pub = self.create_publisher(PoseStamped, "/nav/goal", 10)
        self.phase_pub = self.create_publisher(String, "/nav/phase", 10)
        self.arm_pub = self.create_publisher(String, "/arm/cmd", 10)

        # subs
        self.create_subscription(Bool, "/nav/reached", self.on_reached, 10)
        self.create_subscription(Bool, "/arm/done_pick", self.on_pick_done, 10)

        # CHANGED: PoseArray instead of ItemArray
        self.create_subscription(PoseArray, "/detected_objects", self.on_objects, 10)
        self.create_subscription(PoseArray, "/detected_boxes", self.on_boxes, 10)

        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(dt, self.step)

        self.get_logger().info("TaskPlannerNode up. Pub: /nav/goal, /nav/phase, /arm/cmd  Sub: /nav/reached, /arm/done_pick")
        self.get_logger().info("TaskPlannerNode up. Pub: /nav/goal, /nav/phase, /arm/cmd  Sub: /nav/reached, /arm/done_pick")

    # NEW: small helper for "similar coordinates"
    def _is_picked_xy(self, x: float, y: float) -> bool:
        thr2 = self.picked_dist * self.picked_dist
        for (px, py) in self.picked_ids:
            dx = x - px
            dy = y - py
            if dx * dx + dy * dy <= thr2:
                return True
        return False

    # CHANGED: PoseArray callback
    def on_objects(self, msg: PoseArray):
        self.latest_object = None
        self.get_logger().info("on_objects 1")
        for p in msg.poses:
            self.get_logger().info("on_objects 2")
            x = float(p.position.x)
            y = float(p.position.y)
            if not self._is_picked_xy(x, y):
                self.get_logger().info("on_objects 3")
                self.latest_object = p
                break

    # CHANGED: PoseArray callback
    def on_boxes(self, msg: PoseArray):
        self.latest_box = msg.poses[0] if len(msg.poses) > 0 else None

    def on_pick_done(self, msg: Bool):
        self.pick_done = bool(msg.data)

    def on_reached(self, msg: Bool):
        self.nav_reached = bool(msg.data)

    def lookup_xy(self, target_frame: str):
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, target_frame, rclpy.time.Time())
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
        # yaw not used for MS2; identity is fine
        g.pose.orientation.w = 1.0
        self.goal_pub.publish(g)
        self.get_logger().info("publish goal xy")

    def enter_state(self, new_state: str):
        self.state = new_state
        self._published_this_state = False
        self.get_logger().info(f"State -> {new_state}")

    def step(self):
        # We still lookup TF so we can publish goals from TF frames
        robot = self.lookup_xy(self.base_frame)
        if robot is None:
            return

        if self.state == "SELECT_OBJECT":
            obj = self.latest_object
            box = self.latest_box
            if obj is None or box is None:
                return

            self.current_object = obj
            self.current_box = box

            # CHANGED: Pose has position directly (no .pose)
            self.ox = float(self.current_object.position.x)
            self.oy = float(self.current_object.position.y)
            self.bx = float(self.current_box.position.x)
            self.by = float(self.current_box.position.y)

            self.enter_state("NAV_TO_OBJECT")
            return

        # if self.current_object is None or self.current_box is None:
        if self.current_object is None:
            self.get_logger().info("here")
            return

        if self.state == "NAV_TO_OBJECT":
            if not self._published_this_state:
                self.get_logger().info("nav to object state")
                self.phase_pub.publish(String(data="object"))
                # IMPORTANT: goal should be the object center TF (controller handles standoff)
                self.publish_goal_xy(self.ox, self.oy)
                self._published_this_state = True
                self.nav_reached = False
                self.get_logger().info("nav to object state 2")

            if self.nav_reached:
                self.enter_state("PICK_OBJECT")

        elif self.state == "PICK_OBJECT":
            if not self._published_this_state:
                self.pick_done = False
                self.arm_pub.publish(String(data="pick"))
                self._published_this_state = True

            if self.pick_done:
                # CHANGED: mark picked by (x,y) instead of ID
                self.picked_ids.append((self.ox, self.oy))
                self.enter_state("NAV_TO_BOX")

        elif self.state == "NAV_TO_BOX":
            if not self._published_this_state:
                self.phase_pub.publish(String(data="box"))
                # goal should be the box center TF
                self.publish_goal_xy(self.bx, self.by)
                self._published_this_state = True
                self.nav_reached = False

            if self.nav_reached:
                self.enter_state("DROP_OBJECT")

        elif self.state == "DROP_OBJECT":
            if not self._published_this_state:
                self.arm_pub.publish(String(data="drop"))
                self._published_this_state = True

                # Done with this cycle
                self.current_object = None
                # self.current_box = None

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
