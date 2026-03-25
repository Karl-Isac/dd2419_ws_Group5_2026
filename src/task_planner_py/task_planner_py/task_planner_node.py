#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

import tf2_ros

from std_msgs.msg import Bool, String
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped


class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__("task_planner_node")

        self.declare_parameter("world_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("object_frame", "object_0")
        self.declare_parameter("box_frame", "box_0")
        self.declare_parameter("rate_hz", 5.0)
        self.declare_parameter("path_points", 20)

        self.world_frame = self.get_parameter("world_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.object_frame = self.get_parameter("object_frame").value
        self.box_frame = self.get_parameter("box_frame").value
        self.path_points = int(self.get_parameter("path_points").value)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Planner state
        self.state = "SELECT_OBJECT"
        self.nav_reached = False
        self._published_this_state = False

        # pubs
        self.path_pub = self.create_publisher(Path, "/nav/path", 10)
        self.phase_pub = self.create_publisher(String, "/nav/phase", 10)
        self.arm_pub = self.create_publisher(String, "/arm/cmd", 10)  # adapt to your arm interface

        # subs
        self.create_subscription(Bool, "/nav/reached", self.on_reached, 10)

        dt = 1.0 / float(self.get_parameter("rate_hz").value)
        self.timer = self.create_timer(dt, self.step)

        self.pick_done = False
        self.create_subscription(Bool, "/arm/done_pick", self.on_pick_done, 10)

        self.get_logger().info("TaskPlannerNode up.")

    def on_pick_done(self, msg: Bool):
        self.pick_done = True

    def on_reached(self, msg: Bool):
        self.nav_reached = bool(msg.data)

    def lookup_xy(self, target_frame: str):
        try:
            tf = self.tf_buffer.lookup_transform(self.world_frame, target_frame, rclpy.time.Time())
        except Exception:
            return None
        t = tf.transform.translation
        return (t.x, t.y)

    def make_straight_path(self, x0, y0, x1, y1) -> Path:
        path = Path()
        path.header.frame_id = self.world_frame

        n = max(2, self.path_points)
        for i in range(n):
            a = float(i) / float(n - 1)
            ps = PoseStamped()
            ps.header.frame_id = self.world_frame
            ps.pose.position.x = x0 + (x1 - x0) * a
            ps.pose.position.y = y0 + (y1 - y0) * a
            ps.pose.position.z = 0.0
            path.poses.append(ps)
        return path

    def enter_state(self, new_state: str):
        self.state = new_state
        self._published_this_state = False
        self.get_logger().info(f"State -> {new_state}")



    def step(self):
        robot = self.lookup_xy(self.base_frame)
        obj = self.lookup_xy(self.object_frame)
        box = self.lookup_xy(self.box_frame)

        if robot is None or obj is None or box is None:
            # Don’t spam logs; keep it quiet unless needed
            return

        rx, ry = robot
        ox, oy = obj
        bx, by = box

        if self.state == "SELECT_OBJECT":
            # In MS2 you can hardcode object_0 and box_0
            self.enter_state("NAV_TO_OBJECT")

        elif self.state == "NAV_TO_OBJECT":
            if not self._published_this_state:
                self.phase_pub.publish(String(data="object"))
                self.path_pub.publish(self.make_straight_path(rx, ry, ox, oy))
                self._published_this_state = True
                self.nav_reached = False

            if self.nav_reached:
                self.enter_state("PICK_OBJECT")

        elif self.state == "PICK_OBJECT":
            if not self._published_this_state:
                self.pick_done = False
                self.arm_pub.publish(String(data="pick"))
                self._published_this_state = True

            # move on immediately for testing
            # self.enter_state("NAV_TO_BOX")

            if self.pick_done:
                self.enter_state("NAV_TO_BOX")

        elif self.state == "NAV_TO_BOX":
            if not self._published_this_state:
                self.phase_pub.publish(String(data="box"))
                self.path_pub.publish(self.make_straight_path(rx, ry, bx, by))
                self._published_this_state = True
                self.nav_reached = False

            if self.nav_reached:
                self.enter_state("DROP_OBJECT")

        elif self.state == "DROP_OBJECT":
            if not self._published_this_state:
                self.arm_pub.publish(String(data="drop"))  # adapt
                self._published_this_state = True

            self.enter_state("DONE")

        elif self.state == "DONE":
            # Idle; or loop for more objects
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
