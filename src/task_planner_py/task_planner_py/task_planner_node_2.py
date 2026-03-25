#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import tf2_ros

from std_msgs.msg import Bool, String
from geometry_msgs.msg import PoseStamped


class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__("task_planner_node")

        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("object_frame", "object_0")
        self.declare_parameter("box_frame", "box_0")
        self.declare_parameter("rate_hz", 5.0)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.object_frame = self.get_parameter("object_frame").value
        self.box_frame = self.get_parameter("box_frame").value

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

        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(dt, self.step)

        self.get_logger().info("TaskPlannerNode up. Pub: /nav/goal, /nav/phase, /arm/cmd  Sub: /nav/reached, /arm/done_pick")

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

    def enter_state(self, new_state: str):
        self.state = new_state
        self._published_this_state = False
        self.get_logger().info(f"State -> {new_state}")

    def step(self):
        # We still lookup TF so we can publish goals from TF frames
        robot = self.lookup_xy(self.base_frame)
        obj = self.lookup_xy(self.object_frame)
        box = self.lookup_xy(self.box_frame)

        if robot is None or obj is None or box is None:
            return

        ox, oy = obj
        bx, by = box

        if self.state == "SELECT_OBJECT":
            # In MS2 you can hardcode object_0 and box_0
            self.enter_state("NAV_TO_OBJECT")

        elif self.state == "NAV_TO_OBJECT":
            if not self._published_this_state:
                self.phase_pub.publish(String(data="object"))
                # IMPORTANT: goal should be the object center TF (controller handles standoff)
                self.publish_goal_xy(ox, oy)
                self._published_this_state = True
                self.nav_reached = False

            if self.nav_reached:
                self.enter_state("PICK_OBJECT")

        elif self.state == "PICK_OBJECT":
            if not self._published_this_state:
                self.pick_done = False
                self.arm_pub.publish(String(data="pick"))
                self._published_this_state = True

            if self.pick_done:
                self.enter_state("NAV_TO_BOX")

        elif self.state == "NAV_TO_BOX":
            if not self._published_this_state:
                self.phase_pub.publish(String(data="box"))
                # goal should be the box center TF
                self.publish_goal_xy(bx, by)
                self._published_this_state = True
                self.nav_reached = False

            if self.nav_reached:
                self.enter_state("DROP_OBJECT")

        elif self.state == "DROP_OBJECT":
            if not self._published_this_state:
                self.arm_pub.publish(String(data="drop"))
                self._published_this_state = True
            self.enter_state("DONE")

        elif self.state == "DONE":
            pass


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
