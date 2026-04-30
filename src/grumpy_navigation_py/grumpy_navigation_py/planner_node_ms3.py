#!/usr/bin/env python3
import os
import math
import heapq
import yaml

import numpy as np
import matplotlib.pyplot as plt

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker

from grumpy_interfaces.msg import GoalWithType, PathWithStatus

from ament_index_python.packages import get_package_share_directory

from tf2_ros import Buffer, TransformListener

from visualization_msgs.msg import Marker

from rclpy.duration import Duration
from rclpy.time import Time

from rclpy.executors import MultiThreadedExecutor

from shapely.geometry import Polygon
from shapely import contains_xy


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__("astar_planner_node")

        # ---------------------------------------
        # Parameters
        # ---------------------------------------
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("workspace_file", "fake_workspace.yaml")
        self.declare_parameter("grid_resolution", 0.05)

        # fallback start if TF fails
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)

        # inflation
        self.declare_parameter("workspace_inflation_m", 0.18)
        self.declare_parameter("object_inflation_m", 0.20)
        self.declare_parameter("box_inflation_m", 0.40)
        self.declare_parameter("obstacle_inflation_m", 0.32)

        # candidate search
        self.declare_parameter("candidate_search_radius_m", 0.50)
        self.declare_parameter("max_candidate_cells", 20)

        # path blocked checking
        self.declare_parameter("path_check_rate_hz", 5.0)

        self.world_frame = self.get_parameter("world_frame").value
        self.resolution = float(self.get_parameter("grid_resolution").value)
        self.start_x = float(self.get_parameter("start_x").value)
        self.start_y = float(self.get_parameter("start_y").value)

        self.workspace_inflation_m = float(
            self.get_parameter("workspace_inflation_m").value
        )
        self.object_inflation_m = float(
            self.get_parameter("object_inflation_m").value
        )
        self.box_inflation_m = float(
            self.get_parameter("box_inflation_m").value
        )
        self.obstacle_inflation_m = float(
            self.get_parameter("obstacle_inflation_m").value
        )

        self.candidate_search_radius_m = float(
            self.get_parameter("candidate_search_radius_m").value
        )
        self.max_candidate_cells = int(
            self.get_parameter("max_candidate_cells").value
        )

        path_check_rate_hz = float(self.get_parameter("path_check_rate_hz").value)

        package_share = get_package_share_directory("grumpy_navigation_py")
        self.workspace_file = os.path.join(package_share, "config", "fake_workspace.yaml")

        # ---------------------------------------
        # TF
        # ---------------------------------------
        self.tf_buffer = Buffer(
            cache_time=Duration(seconds=30.0)
        )
        # self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---------------------------------------
        # State
        # ---------------------------------------
        self.objects = []
        self.boxes = []
        self.obstacles = []

        self.goal = None                  # (x, y)
        self.goal_type = None             # GoalWithType enum value
        # self.previous_goal = None

        self.current_grid = None
        self.current_path_cells = []
        self.current_path_blocked = False

        self.rerun_on_goal_later = False

        # ---------------------------------------
        # Workspace
        # ---------------------------------------
        self.workspace_poly = None
        self.min_x = 0.0
        self.max_x = 0.0
        self.min_y = 0.0
        self.max_y = 0.0
        self.grid_width = 0
        self.grid_height = 0
        self.load_workspace()
        self.build_workspace_mask()

        # ---------------------------------------
        # ROS
        # ---------------------------------------
        self.create_subscription(PoseArray, "/detected_objects", self.on_objects, 10)
        self.create_subscription(PoseArray, "/detected_boxes", self.on_boxes, 10)

        # self.create_subscription(PoseStamped, "/fake_obstacles", self.on_obstacle, 10)
        self.create_subscription(PoseArray, "/detected_obstacles", self.on_obstacles, 10)

        self.create_subscription(GoalWithType, "/nav/goal", self.on_goal, 10)

        self.path_pub = self.create_publisher(PathWithStatus, "/nav/path_from_planner", 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/nav/grid", 10)
        self.goal_marker_pub = self.create_publisher(Marker, "/nav/goal_marker", 10)

        # New: planner tells task planner/controller that current path is blocked
        self.path_blocked_pub = self.create_publisher(Bool, "/nav/path_blocked", 10)

        #debug
        self.marker_pub = self.create_publisher(Marker, "/debug/robot_start", 10)

        # Continuous path validity checking
        timer_period = 1.0 / max(path_check_rate_hz, 1e-6)
        self.create_timer(timer_period, self.check_current_path_collision)

        self.get_logger().info("Planner ready")

    def publish_robot_start_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"   # IMPORTANT: match your RViz fixed frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "robot_start"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = 0.0

        marker.pose.orientation.w = 1.0

        # Size of the point
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # Color (RGBA)
        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.2
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    # ---------------------------------------
    # Workspace
    # ---------------------------------------


    def build_workspace_mask(self):
        if self.workspace_poly is None:
            self.base_grid = None
            return

        w = int(math.ceil((self.max_x - self.min_x) / self.resolution))
        h = int(math.ceil((self.max_y - self.min_y) / self.resolution))

        self.grid_width = w
        self.grid_height = h

        poly = Polygon(self.workspace_poly)
        safe_poly = poly.buffer(-self.workspace_inflation_m)

        xs = self.min_x + (np.arange(w) + 0.5) * self.resolution
        ys = self.min_y + (np.arange(h) + 0.5) * self.resolution
        xx, yy = np.meshgrid(xs, ys)

        inside = contains_xy(safe_poly, xx, yy)

        self.base_grid = np.full((h, w), 100, dtype=np.int8)
        self.base_grid[inside] = 0   

    def load_workspace(self):

        try:
            with open(self.workspace_file, "r") as f:
                data = yaml.safe_load(f)

            self.workspace_poly = [tuple(p) for p in data["workspace"]["perimeter"]]

            xs = [p[0] for p in self.workspace_poly]
            ys = [p[1] for p in self.workspace_poly]

            self.min_x = min(xs)
            self.max_x = max(xs)
            self.min_y = min(ys)
            self.max_y = max(ys)

        except Exception as e:
            self.get_logger().error(f"Workspace load failed: {e}")
            self.workspace_poly = None

    def inside_poly(self, x, y):
        if self.workspace_poly is None:
            return True

        inside = False
        poly = self.workspace_poly
        j = len(poly) - 1

        for i in range(len(poly)):
            xi, yi = poly[i]
            xj, yj = poly[j]

            if ((yi > y) != (yj > y)) and (
                x < (xj - xi) * (y - yi) / (yj - yi + 1e-12) + xi
            ):
                inside = not inside
            j = i

        return inside

    def point_to_segment_distance(self, px, py, ax, ay, bx, by):
        abx = bx - ax
        aby = by - ay
        apx = px - ax
        apy = py - ay

        ab_len_sq = abx * abx + aby * aby
        if ab_len_sq == 0.0:
            return math.sqrt((px - ax) ** 2 + (py - ay) ** 2)

        t = (apx * abx + apy * aby) / ab_len_sq
        t = max(0.0, min(1.0, t))

        closest_x = ax + t * abx
        closest_y = ay + t * aby

        dx = px - closest_x
        dy = py - closest_y
        return math.sqrt(dx * dx + dy * dy)

    def distance_to_polygon_edges(self, x, y):
        if self.workspace_poly is None or len(self.workspace_poly) < 2:
            return float("inf")

        best = float("inf")
        poly = self.workspace_poly

        for i in range(len(poly)):
            ax, ay = poly[i]
            bx, by = poly[(i + 1) % len(poly)]
            d = self.point_to_segment_distance(x, y, ax, ay, bx, by)
            if d < best:
                best = d

        return best

    # ---------------------------------------
    # Callbacks
    # ---------------------------------------
    # def on_objects(self, msg):
    #     self.objects = [(p.position.x, p.position.y) for p in msg.poses]
    #     self.current_grid = self.rebuild_grid()
    #
    # def on_boxes(self, msg):
    #     self.boxes = [(p.position.x, p.position.y) for p in msg.poses]
    #     self.current_grid = self.rebuild_grid()

    def on_objects(self, msg):
        self.get_logger().info("on_objects")
        for p in msg.poses:
            self.objects.append((p.position.x, p.position.y))

        self.current_grid = self.rebuild_grid()


    def on_boxes(self, msg):
        self.get_logger().info("on_boxes")
        for p in msg.poses:
            self.boxes.append((p.position.x, p.position.y))

        self.current_grid = self.rebuild_grid()

    # def on_obstacle(self, msg):
    #     # Keeping append behavior here since this topic was already single obstacle style
    #     x = msg.pose.position.x
    #     y = msg.pose.position.y
    #     self.obstacles.append((x, y))
    #     self.current_grid = self.rebuild_grid()

    def on_obstacles(self, msg):
        
        # self.get_logger().info("on_obstacles")

        # overwrite, not append (important!)
        self.obstacles = [
            (p.position.x, p.position.y)
            for p in msg.poses
        ]

        self.current_grid = self.rebuild_grid()

    def on_goal(self, msg):

        self.get_logger().info("on_goal")

        self.latest_on_goal_message = msg

        # if self.goal is not None:
        #     self.previous_goal = self.goal

        goal_pose = msg.goal.pose
        self.goal = (goal_pose.position.x, goal_pose.position.y)
        self.goal_type = msg.type

        self.publish_goal_marker(self.goal[0], self.goal[1])

        grid = self.rebuild_grid()
        self.current_grid = grid
        if grid is None:
            # self.publish_empty_path()
            self.publish_empty_path_result("grid_failed")
            return

        msg_time = Time.from_msg(msg.goal.header.stamp)
        robot = self.lookup_robot_xy(msg_time)
        if robot is None:
            self.get_logger().warn("Could not get robot pose, using parameter fallback")
            # robot = (self.start_x, self.start_y)
            return

        start = self.world_to_grid(robot[0], robot[1])
        self.get_logger().info(f"robot start pos = ({robot[0]}, {robot[1]})")
        self.publish_robot_start_marker(robot[0], robot[1])

        raw_goal = self.world_to_grid(self.goal[0], self.goal[1])

        if not self.cell_in_bounds(start[0], start[1], grid):
            self.get_logger().warn("Start cell out of bounds")
            self.current_path_cells = []
            # self.publish_empty_path()
            self.publish_empty_path_result("start_out_of_bounds")
            return

        if grid[start[1]][start[0]] != 0:
            self.get_logger().warn("Start cell is occupied")
            self.current_path_cells = []
            # self.publish_empty_path()
            self.publish_empty_path_result("start_occupied")
            return

        if self.goal_type_is_object_or_box(msg.type):
            result = self.plan_to_free_cell_near_goal(grid, start, raw_goal)
        else:
            result = self.plan_directly_to_goal(grid, start, raw_goal)

        if result is None:
            self.get_logger().warn("Planner found no valid path")
            self.current_path_cells = []
            # self.publish_empty_path()
            self.publish_empty_path_result("no_path")
            self.publish_path_blocked(False)
            return

        best_path_cells, best_cost = result

        self.get_logger().info(f"Planned path with cost {best_cost:.3f}")
        self.current_path_cells = best_path_cells
        self.current_path_blocked = False
        self.publish_path_blocked(False)
        # self.publish_path(best_path_cells)
        self.publish_path_result(best_path_cells, "success")

        if msg.type == GoalWithType.OBJECT:
            self.remove_object_at_goal(self.goal)

    def goal_type_is_object_or_box(self, goal_type):
        if goal_type == GoalWithType.OBJECT:
            return True

        # Safe if BOX is not defined in the message
        box_enum = getattr(GoalWithType, "BOX", None)
        if box_enum is not None and goal_type == box_enum:
            return True

        return False

    def publish_goal_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "goal"
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.goal_marker_pub.publish(marker)

    # ---------------------------------------
    # TF
    # ---------------------------------------
    # def lookup_robot_xy(self):
    #     target_frame = "base_link"
    #     world_frame = self.world_frame
    #
    #     try:
    #         tf = self.tf_buffer.lookup_transform(
    #             world_frame,
    #             target_frame,
    #             rclpy.time.Time(),
    #             timeout=Duration(seconds=5.0)
    #         )
    #     except Exception as e:
    #         self.get_logger().warn(f"TF lookup failed: {e}")
    #         return None
    #
    #     t = tf.transform.translation
    #     return (float(t.x), float(t.y))


    def lookup_robot_xy(self, msg_time):
        target_frame = "base_link"
        world_frame = self.world_frame

        try:
            # future = self.tf_buffer.wait_for_transform_async(
            #     world_frame,
            #     target_frame,
            #     # Time()
            #     self.get_clock().now()
            # )
            #
            # rclpy.spin_until_future_complete(
            #     self,
            #     future,
            #     timeout_sec=1.0
            # )
            #
            # if not future.done():
            #     self.get_logger().warn("TF async wait timed out")
            #     return None
            #
            # future.result()

            # msg_time = Time.from_msg(stamp)    

            tf = self.tf_buffer.lookup_transform(
                world_frame,
                target_frame,
                # Time(),
                msg_time,
                timeout=Duration(seconds=0.1)
            )

        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")

            # self.rerun_on_goal_later = True

            self.rerun_on_goal_timer = self.create_timer(0.1, self.rerun_on_goal_callback)

            return None

        t = tf.transform.translation
        return (float(t.x), float(t.y))

    def rerun_on_goal_callback(self):
        self.get_logger().info("rerun_on_goal_callback")
        self.rerun_on_goal_timer.destroy() 
        self.on_goal(self.latest_on_goal_message)

    # def lookup_robot_xy(self):
    #     target_frame = "base_link"
    #     world_frame = self.world_frame
    #
    #     max_age = 0.15          # seconds; tune this
    #     timeout_sec = 5.0
    #     start = self.get_clock().now()
    #
    #     while (self.get_clock().now() - start).nanoseconds * 1e-9 < timeout_sec:
    #         try:
    #             tf = self.tf_buffer.lookup_transform(
    #                 world_frame,
    #                 target_frame,
    #                 Time(),  # latest available
    #                 timeout=Duration(seconds=0.1)
    #             )
    #
    #             tf_time = Time.from_msg(tf.header.stamp)
    #             age = (self.get_clock().now() - tf_time).nanoseconds * 1e-9
    #
    #             if age <= max_age:
    #                 t = tf.transform.translation
    #                 return (float(t.x), float(t.y))
    #
    #             self.get_logger().warn(f"Waiting for fresh TF, age={age:.3f}s")
    #
    #         except Exception as e:
    #             self.get_logger().warn(f"TF lookup failed: {e}")
    #
    #         rclpy.spin_once(self, timeout_sec=0.05)
    #
    #     self.get_logger().warn("Timed out waiting for fresh TF")
    #     return None

    # def lookup_robot_xy(self):
    #     target_frame = "base_link"
    #     world_frame = self.world_frame
    #
    #     try:
    #         tf = self.tf_buffer.lookup_transform(
    #             world_frame,
    #             target_frame,
    #             Time(),  # latest available transform
    #             timeout=Duration(seconds=1.0)
    #         )
    #     except Exception as e:
    #         self.get_logger().warn(f"TF lookup failed: {e}")
    #         return None
    #
    #     # reject stale transforms
    #     tf_time = Time.from_msg(tf.header.stamp)
    #     age = (self.get_clock().now() - tf_time).nanoseconds * 1e-9
    #
    #     if age > 0.2:
    #         self.get_logger().warn(f"TF too old: {age:.3f}s")
    #         return None
    #
    #     t = tf.transform.translation
    #     return (float(t.x), float(t.y))

    # ---------------------------------------
    # Grid helpers
    # ---------------------------------------

    def remove_object_at_goal(self, goal_xy, tolerance=0.20):
        if not self.objects:
            return

        gx, gy = goal_xy

        best_idx = None
        best_dist_sq = float("inf")

        for i, (x, y) in enumerate(self.objects):
            dist_sq = (x - gx) ** 2 + (y - gy) ** 2
            if dist_sq < best_dist_sq:
                best_dist_sq = dist_sq
                best_idx = i

        if best_idx is not None and best_dist_sq <= tolerance * tolerance:
            removed = self.objects.pop(best_idx)
            self.get_logger().info(f"Removed object from memory at {removed}")
        else:
            self.get_logger().warn("No matching object found near goal to remove")

    def world_to_grid(self, x, y):
        gx = int((x - self.min_x) / self.resolution)
        gy = int((y - self.min_y) / self.resolution)

        # w = int(math.ceil((self.max_x - self.min_x) / self.resolution))
        # h = int(math.ceil((self.max_y - self.min_y) / self.resolution))
        w = self.grid_width
        h = self.grid_height

        gx = max(0, min(gx, w - 1))
        gy = max(0, min(gy, h - 1))

        return gx, gy

    def grid_to_world(self, gx, gy):
        x = self.min_x + (gx + 0.5) * self.resolution
        y = self.min_y + (gy + 0.5) * self.resolution
        return x, y

    def cell_in_bounds(self, gx, gy, grid):
        h = len(grid)
        w = len(grid[0])
        return 0 <= gx < w and 0 <= gy < h

    def visualize_grid(self, grid, filename="grid.png"):
        grid_np = np.array(grid)

        plt.figure()
        plt.imshow(grid_np, origin="lower")
        plt.colorbar(label="Occupancy (0=free, 100=occupied)")
        plt.title("Occupancy Grid")

        save_path = os.path.join(
            get_package_share_directory("grumpy_navigation_py"),
            "config",
            filename
        )

        plt.savefig(save_path)
        plt.close()

        # self.get_logger().info(f"Saved grid visualization to: {save_path}")

    def rebuild_grid(self):
        if self.base_grid is None:
            self.get_logger().warn("No workspace grid loaded")
            return None

        grid = self.base_grid.copy()
        h, w = grid.shape

        def inflate_positions(positions, inflation_radius_m):
            inflation_cells = int(math.ceil(inflation_radius_m / self.resolution))

            for (x, y) in positions:
                gx, gy = self.world_to_grid(x, y)

                x0 = max(0, gx - inflation_cells)
                x1 = min(w, gx + inflation_cells + 1)
                y0 = max(0, gy - inflation_cells)
                y1 = min(h, gy + inflation_cells + 1)

                yy, xx = np.ogrid[y0:y1, x0:x1]
                mask = (xx - gx) ** 2 + (yy - gy) ** 2 <= inflation_cells ** 2

                grid[y0:y1, x0:x1][mask] = 100

        inflate_positions(self.objects, self.object_inflation_m)
        inflate_positions(self.boxes, self.box_inflation_m)
        inflate_positions(self.obstacles, self.obstacle_inflation_m)

        self.publish_grid(grid, w, h)
        return grid

    def publish_grid(self, grid, w, h):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame

        msg.info.resolution = self.resolution
        msg.info.width = w
        msg.info.height = h
        msg.info.origin.position.x = self.min_x
        msg.info.origin.position.y = self.min_y
        msg.info.origin.orientation.w = 1.0

        # msg.data = [cell for row in grid for cell in row]
        msg.data = grid.astype(np.int8).ravel().tolist()
        self.grid_pub.publish(msg)

    # ---------------------------------------
    # Planning
    # ---------------------------------------
    def plan_directly_to_goal(self, grid, start, goal):
        if not self.cell_in_bounds(goal[0], goal[1], grid):
            self.get_logger().warn("Goal cell out of bounds")
            return None

        if grid[goal[1]][goal[0]] != 0:
            self.get_logger().warn("Goal cell is occupied")
            return None

        path_cells, cost = self.run_astar(grid, start, goal)
        if not path_cells:
            return None

        return path_cells, cost

    def plan_to_free_cell_near_goal(self, grid, start, goal):
        candidates = self.find_candidate_goal_cells(grid, goal)

        if not candidates:
            self.get_logger().warn("No candidate free cells found near goal")
            return None

        best_path = None
        best_score = float("inf")
        best_path_cost = float("inf")

        goalx, goaly = goal

        for candidate in candidates:
            path_cells, path_cost = self.run_astar(grid, start, candidate)
            if not path_cells:
                continue

            # Score = path cost + small penalty for being farther from the true goal
            dx = candidate[0] - goalx
            dy = candidate[1] - goaly
            dist_to_goal_cells = math.sqrt(dx * dx + dy * dy)

            score = path_cost + 0.25 * dist_to_goal_cells

            if score < best_score:
                best_score = score
                best_path_cost = path_cost
                best_path = path_cells

        if best_path is None:
            return None

        return best_path, best_path_cost

    def find_candidate_goal_cells(self, grid, goal):
        """
        Return free cells near the raw goal cell that lie on the boundary
        of occupied space.

        A candidate cell must:
        - be within search radius of goal
        - be free
        - have at least one occupied neighbor in its 8-neighborhood
        """
        candidates = []

        gx, gy = goal
        h = len(grid)
        w = len(grid[0])

        search_radius_cells = int(
            math.ceil(self.candidate_search_radius_m / self.resolution)
        )

        for dy in range(-search_radius_cells, search_radius_cells + 1):
            for dx in range(-search_radius_cells, search_radius_cells + 1):
                nx = gx + dx
                ny = gy + dy

                if not (0 <= nx < w and 0 <= ny < h):
                    continue

                # only consider cells inside circular radius
                if dx * dx + dy * dy > search_radius_cells * search_radius_cells:
                    continue

                # candidate must be free
                if grid[ny][nx] != 0:
                    continue

                # has_occupied_neighbor = False
                #
                # for ddy in [-1, 0, 1]:
                #     for ddx in [-1, 0, 1]:
                #         if ddx == 0 and ddy == 0:
                #             continue
                #
                #         cx = nx + ddx
                #         cy = ny + ddy
                #
                #         if not (0 <= cx < w and 0 <= cy < h):
                #             continue
                #
                #         if grid[cy][cx] == 100:
                #             has_occupied_neighbor = True
                #             break
                #
                #     if has_occupied_neighbor:
                #         break
                #
                # if not has_occupied_neighbor:
                #     continue

                # require clearance from occupied cells
                min_clearance_cells = int(math.ceil(0.08 / self.resolution))  # tune 0.08-0.15

                too_close = False
                for ddy in range(-min_clearance_cells, min_clearance_cells + 1):
                    for ddx in range(-min_clearance_cells, min_clearance_cells + 1):
                        cx = nx + ddx
                        cy = ny + ddy

                        if not (0 <= cx < w and 0 <= cy < h):
                            continue

                        if ddx * ddx + ddy * ddy > min_clearance_cells * min_clearance_cells:
                            continue

                        if grid[cy][cx] == 100:
                            too_close = True
                            break

                    if too_close:
                        break

                if too_close:
                    continue

                dist = math.sqrt(dx * dx + dy * dy)
                candidates.append((dist, (nx, ny)))

        candidates.sort(key=lambda x: x[0])
        candidates = [cell for _, cell in candidates[:self.max_candidate_cells]]

        return candidates

    # ---------------------------------------
    # Path validity checking
    # ---------------------------------------
    def find_closest_path_index_to_robot(self, msg_time):
        robot = self.lookup_robot_xy(msg_time)
        if robot is None:
            return 0

        rx, ry = robot
        best_idx = 0
        best_dist = float("inf")

        for i, (gx, gy) in enumerate(self.current_path_cells):
            x, y = self.grid_to_world(gx, gy)
            d = math.hypot(x - rx, y - ry)
            if d < best_dist:
                best_dist = d
                best_idx = i

        return best_idx

    def check_current_path_collision(self):
        if self.current_grid is None:
            return

        if not self.current_path_cells:
            return

        # Optionally refresh the grid here so dynamic updates are always checked
        grid = self.rebuild_grid()
        if grid is None:
            return

        self.current_grid = grid

        blocked = False

        # for gx, gy in self.current_path_cells:
       
        time = Time()
        start_idx = self.find_closest_path_index_to_robot(time)
        future_cells = self.current_path_cells[start_idx:]
        for gx, gy in future_cells:
            if not self.cell_in_bounds(gx, gy, grid):
                blocked = True
                break

            if grid[gy][gx] == 100:
                blocked = True
                break

        if blocked and not self.current_path_blocked:
            self.current_path_blocked = True
            self.get_logger().warn("Current path is now blocked")
            self.publish_path_blocked(True)

        elif not blocked and self.current_path_blocked:
            self.current_path_blocked = False
            self.publish_path_blocked(False)

    def publish_path_blocked(self, blocked):
        msg = Bool()
        msg.data = blocked
        self.path_blocked_pub.publish(msg)

    # ---------------------------------------
    # Path publishing
    # ---------------------------------------
    # def publish_path(self, cells):
    #     path = Path()
    #     path.header.stamp = self.get_clock().now().to_msg()
    #     path.header.frame_id = self.world_frame
    #
    #     for gx, gy in cells:
    #         x, y = self.grid_to_world(gx, gy)
    #
    #         p = PoseStamped()
    #         p.header = path.header
    #         p.pose.position.x = x
    #         p.pose.position.y = y
    #         p.pose.position.z = 0.0
    #         p.pose.orientation.w = 1.0
    #
    #         path.poses.append(p)
    #
    #     self.path_pub.publish(path)
    #
    # def publish_empty_path(self):
    #     path = Path()
    #     path.header.stamp = self.get_clock().now().to_msg()
    #     path.header.frame_id = self.world_frame
    #     self.path_pub.publish(path)

    def make_path_msg(self, cells):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.world_frame

        for gx, gy in cells:
            x, y = self.grid_to_world(gx, gy)

            p = PoseStamped()
            p.header = path.header
            p.pose.position.x = x
            p.pose.position.y = y
            p.pose.position.z = 0.0
            p.pose.orientation.w = 1.0

            path.poses.append(p)

        return path


    def publish_path_result(self, cells, status):
        msg = PathWithStatus()
        msg.path = self.make_path_msg(cells)
        msg.status = status
        self.path_pub.publish(msg)


    def publish_empty_path_result(self, status):
        msg = PathWithStatus()

        msg.path.header.stamp = self.get_clock().now().to_msg()
        msg.path.header.frame_id = self.world_frame
        msg.status = status

        self.path_pub.publish(msg)

    # ---------------------------------------
    # A*
    # ---------------------------------------
    def run_astar(self, grid, start, goal):
        """
        grid[y][x] == 0    -> free
        grid[y][x] != 0    -> occupied

        Returns:
            (path_cells, total_cost)

        If no path:
            ([], inf)
        """
        def h(a, b):
            return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

        def reconstruct(came_from, current):
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return path[::-1]

        open_set = []
        heapq.heappush(open_set, (h(start, goal), start))

        came_from = {}
        g_score = {start: 0.0}
        closed_set = set()

        while open_set:
            _, current = heapq.heappop(open_set)

            if current in closed_set:
                continue
            closed_set.add(current)

            if current == goal:
                return reconstruct(came_from, current), g_score[current]

            cx, cy = current

            neighbors = [
                (cx + 1, cy, 1.0),
                (cx - 1, cy, 1.0),
                (cx, cy + 1, 1.0),
                (cx, cy - 1, 1.0),
                (cx + 1, cy + 1, math.sqrt(2)),
                (cx - 1, cy + 1, math.sqrt(2)),
                (cx + 1, cy - 1, math.sqrt(2)),
                (cx - 1, cy - 1, math.sqrt(2)),
            ]

            for nx, ny, move_cost in neighbors:
                if not self.cell_in_bounds(nx, ny, grid):
                    continue

                if grid[ny][nx] != 0:
                    continue

                # Prevent diagonal corner cutting
                if nx != cx and ny != cy:
                    if grid[cy][nx] != 0 or grid[ny][cx] != 0:
                        continue

                neighbor = (nx, ny)
                tentative_g = g_score[current] + move_cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + h(neighbor, goal)
                    heapq.heappush(open_set, (f_score, neighbor))

        return [], float("inf")


def main():
    rclpy.init()
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

# def main():
#     rclpy.init()
#     node = AStarPlannerNode()
#
#     executor = MultiThreadedExecutor(num_threads=2)
#     executor.add_node(node)
#
#     try:
#         executor.spin()
#     finally:
#         executor.shutdown()
#         node.destroy_node()
#         rclpy.shutdown()


if __name__ == "__main__":
    main()
