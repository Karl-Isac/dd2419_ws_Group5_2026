#!/usr/bin/env python3
import yaml
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path, OccupancyGrid
from grumpy_interfaces.msg import GoalWithType
from visualization_msgs.msg import Marker

import numpy as np

import heapq
import math

from ament_index_python.packages import get_package_share_directory
import os

import matplotlib.pyplot as plt

import tf2_ros
from tf2_ros import Buffer, TransformListener


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__("astar_planner_node")

        # ---- Parameters ----
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("workspace_file", "fake_workspace.yaml")
        self.declare_parameter("grid_resolution", 0.02)

        # temporary fixed start position
        self.declare_parameter("start_x", 0.0)
        self.declare_parameter("start_y", 0.0)

        self.world_frame = self.get_parameter("world_frame").value

        package_share = get_package_share_directory("grumpy_navigation_py")
        self.workspace_file = os.path.join(
            package_share, "config", "fake_workspace.yaml"
        )

        self.resolution = float(self.get_parameter("grid_resolution").value)
        self.start_x = float(self.get_parameter("start_x").value)
        self.start_y = float(self.get_parameter("start_y").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ---- Storage ----
        self.objects = []
        self.boxes = []
        self.obstacles = []
        self.goal = None
        self.previous_goal = None

        # ---- Workspace ----
        self.workspace_poly = None
        self.min_x = 0.0
        self.max_x = 0.0
        self.min_y = 0.0
        self.max_y = 0.0
        self.load_workspace()

        # ---- ROS ----
        self.create_subscription(PoseArray, "/detected_objects", self.on_objects, 10)
        self.create_subscription(PoseArray, "/detected_boxes", self.on_boxes, 10)
        self.create_subscription(PoseStamped, "/fake_obstacles", self.on_obstacle, 10)
        self.create_subscription(GoalWithType, "/nav/goal", self.on_goal, 10)

        self.path_pub = self.create_publisher(Path, "/nav/path_from_planner", 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/nav/grid", 10)
        self.goal_marker_pub = self.create_publisher(Marker, "/nav/goal_marker", 10)

        self.get_logger().info("Planner ready")

    # ---------------------------------------
    # Workspace
    # ---------------------------------------
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
    def on_objects(self, msg):
        for p in msg.poses:
            self.objects.append((p.position.x, p.position.y))
        self.rebuild_grid()

    def on_boxes(self, msg):
        for p in msg.poses:
            self.boxes.append((p.position.x, p.position.y))
        self.rebuild_grid()

    def on_obstacle(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.obstacles.append((x, y))
        self.rebuild_grid()

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

    def on_goal(self, msg):
        if self.goal is not None:
            self.previous_goal = self.goal

        goal_pose = msg.goal.pose
        self.goal = (goal_pose.position.x, goal_pose.position.y)

        # publish marker for visualization
        self.publish_goal_marker(self.goal[0], self.goal[1])

        if msg.type == GoalWithType.OBJECT:
            self.remove_object_at_goal(self.goal)

        grid = self.rebuild_grid()
        if grid is None:
            return

        robot = self.lookup_robot_xy()
        if robot is None:
            self.get_logger().warn("Could not get robot pose")
            return

        start = self.world_to_grid(robot[0], robot[1])
        goal = self.world_to_grid(self.goal[0], self.goal[1])

        if not self.cell_in_bounds(start[0], start[1], grid):
            self.get_logger().warn("Start cell out of bounds")
            return

        if not self.cell_in_bounds(goal[0], goal[1], grid):
            self.get_logger().warn("Goal cell out of bounds")
            return

        if grid[start[1]][start[0]] != 0:
            self.get_logger().warn("Start cell is occupied")
            return

        if grid[goal[1]][goal[0]] != 0:
            self.get_logger().warn("Goal cell is occupied")
            return

        cells = self.run_astar(grid, start, goal)

        if not cells:
            self.get_logger().warn("A* returned no path")
            self.publish_empty_path()
            return

        self.publish_path(cells)

    def remove_object_at_goal(self, goal_xy, tolerance=0.15):
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

    # ---------------------------------------
    # TF lookup
    # ---------------------------------------
    def lookup_robot_xy(self):
        target_frame = "base_link"
        world_frame = "map"
        try:
            tf = self.tf_buffer.lookup_transform(
                world_frame,
                target_frame,
                rclpy.time.Time()
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return None

        t = tf.transform.translation
        return (float(t.x), float(t.y))

    # ---------------------------------------
    # Grid helpers
    # ---------------------------------------
    def visualize_grid(self, grid, filename="grid.png"):
        """
        Visualize occupancy grid and save as image.

        grid: 2D list [y][x]
        """
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

        self.get_logger().info(f"Saved grid visualization to: {save_path}")

    def world_to_grid(self, x, y):
        gx = int((x - self.min_x) / self.resolution)
        gy = int((y - self.min_y) / self.resolution)

        w = int(math.ceil((self.max_x - self.min_x) / self.resolution))
        h = int(math.ceil((self.max_y - self.min_y) / self.resolution))

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

    def clear_region_around_cell(self, grid, center_gx, center_gy, radius_m):
        radius_cells = int(math.ceil(radius_m / self.resolution))
        h = len(grid)
        w = len(grid[0])

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                gx = center_gx + dx
                gy = center_gy + dy

                if not (0 <= gx < w and 0 <= gy < h):
                    continue

                dist = math.sqrt(
                    (dx * self.resolution) ** 2 +
                    (dy * self.resolution) ** 2
                )

                if dist <= radius_m:
                    grid[gy][gx] = 0

    def rebuild_grid(self):
        if self.workspace_poly is None:
            self.get_logger().warn("No workspace polygon loaded")
            return None

        w = int(math.ceil((self.max_x - self.min_x) / self.resolution))
        h = int(math.ceil((self.max_y - self.min_y) / self.resolution))

        self.grid_width = w
        self.grid_height = h

        grid = [[0 for _ in range(w)] for _ in range(h)]

        # --- Workspace inflation ---
        workspace_inflation_m = 0.15

        for gy in range(h):
            for gx in range(w):
                x, y = self.grid_to_world(gx, gy)

                outside_workspace = not self.inside_poly(x, y)
                too_close_to_wall = (
                    self.distance_to_polygon_edges(x, y) < workspace_inflation_m
                )

                if outside_workspace or too_close_to_wall:
                    grid[gy][gx] = 100

        goal_cell = None
        if self.goal is not None:
            goal_cell = self.world_to_grid(self.goal[0], self.goal[1])

        previous_goal_cell = None
        if self.previous_goal is not None:
            previous_goal_cell = self.world_to_grid(
                self.previous_goal[0], self.previous_goal[1]
            )

        object_inflation_m = 0.15
        box_inflation_m = 0.35
        obstacle_inflation_m = 0.35

        def inflate_positions(positions, inflation_radius_m):
            inflation_cells = int(math.ceil(inflation_radius_m / self.resolution))

            for (x, y) in positions:
                gx, gy = self.world_to_grid(x, y)

                if goal_cell is not None and (gx, gy) == goal_cell:
                    continue

                if previous_goal_cell is not None and (gx, gy) == previous_goal_cell:
                    continue

                for dy in range(-inflation_cells, inflation_cells + 1):
                    for dx in range(-inflation_cells, inflation_cells + 1):
                        nx = gx + dx
                        ny = gy + dy

                        if not (0 <= nx < w and 0 <= ny < h):
                            continue

                        if dx * dx + dy * dy > inflation_cells * inflation_cells:
                            continue

                        if goal_cell is not None and (nx, ny) == goal_cell:
                            continue

                        if previous_goal_cell is not None and (nx, ny) == previous_goal_cell:
                            continue

                        grid[ny][nx] = 100

        inflate_positions(self.objects, object_inflation_m)
        inflate_positions(self.boxes, box_inflation_m)
        inflate_positions(self.obstacles, obstacle_inflation_m)

        # Clear a region around the active goal so goals near workspace edges
        # are still reachable even with workspace inflation enabled.
        if goal_cell is not None:
            self.clear_region_around_cell(
                grid,
                goal_cell[0],
                goal_cell[1],
                radius_m=0.15
            )

        # Optional: also clear around previous goal
        if previous_goal_cell is not None:
            self.clear_region_around_cell(
                grid,
                previous_goal_cell[0],
                previous_goal_cell[1],
                radius_m=0.10
            )

        self.publish_grid(grid, w, h)
        self.visualize_grid(grid)

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

        msg.data = [cell for row in grid for cell in row]
        self.grid_pub.publish(msg)

    # ---------------------------------------
    # Path publishing
    # ---------------------------------------
    def publish_path(self, cells):
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

        self.path_pub.publish(path)

    def publish_empty_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.world_frame
        self.path_pub.publish(path)

    def run_astar(self, grid, start, goal):
        """
        grid[y][x] == 0    -> free
        grid[y][x] != 0    -> occupied

        start = (gx, gy)
        goal  = (gx, gy)

        Return:
            [(gx1, gy1), (gx2, gy2), ...]
        """

        def h(a, b):
            return math.sqrt((a[0] - b[0])**2 + (a[1] - b[1])**2)

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

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                return reconstruct(came_from, current)

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

            for nx, ny, cost in neighbors:
                if ny < 0 or ny >= len(grid) or nx < 0 or nx >= len(grid[0]):
                    continue

                if grid[ny][nx] != 0:
                    continue

                neighbor = (nx, ny)
                tentative_g = g_score[current] + cost

                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + h(neighbor, goal)
                    heapq.heappush(open_set, (f, neighbor))

        return []


def main():
    rclpy.init()
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
