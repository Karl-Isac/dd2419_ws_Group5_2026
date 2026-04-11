#!/usr/bin/env python3
import yaml
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped, PoseArray
from nav_msgs.msg import Path, OccupancyGrid

import numpy as np

import heapq
import math

from ament_index_python.packages import get_package_share_directory
import os

import matplotlib.pyplot as plt

from rclpy.time import Time

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

        # self.workspace_file = self.get_parameter("workspace_file").value
#         workspace_file_param = self.get_parameter("workspace_file").value
#         script_dir = os.path.dirname(os.path.abspath(__file__))
#         self.workspace_file = os.path.join(script_dir, workspace_file_param)

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
        self.create_subscription(PoseStamped, "/nav/goal", self.on_goal, 10)

        self.path_pub = self.create_publisher(Path, "/nav/path_from_planner", 10)
        self.grid_pub = self.create_publisher(OccupancyGrid, "/nav/grid", 10)

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

    # ---------------------------------------
    # Callbacks
    # ---------------------------------------
    def on_objects(self, msg):
        for p in msg.poses:
            # REMOVE LATER
            if len(self.objects) >= 2:
                break
            self.objects.append((p.position.x, p.position.y))
        self.rebuild_grid()

    def on_boxes(self, msg):
        for p in msg.poses:
            if len(self.boxes) >=1:
                break
            self.boxes.append((p.position.x, p.position.y))
        self.rebuild_grid()

    def on_obstacle(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.obstacles.append((x, y))
        self.rebuild_grid()

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)

        grid = self.rebuild_grid()

        if grid is None:
            return

        robot = self.lookup_robot_xy()
        if robot is None:
            self.get_logger().warn("Could not get robot pose")
            return

        start = self.world_to_grid(robot[0], robot[1])
        
        # start = self.world_to_grid(self.start_x, self.start_y)





        goal = self.world_to_grid(self.goal[0], self.goal[1])

        print(f"world (x, y) = {self.goal[0]}, {self.goal[1]}")
        print(f"grid (x, y) = {goal[0]}, {goal[1]}")

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

    # ------- Lookup transform ------

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
#     def world_to_grid(self, x, y):
#         gx = int((x - self.min_x) / self.resolution)
#         gy = int((y - self.min_y) / self.resolution)
# 
# #         gx = max(0, min(gx, self.grid_width - 1))
# #         gy = max(0, min(gy, self.grid_height - 1))
# 
#         return gx, gy

    def visualize_grid(self, grid, filename="grid.png"):
        """
        Visualize occupancy grid and save as image.

        grid: 2D list [y][x]
        """

        # Convert to numpy
        grid_np = np.array(grid)

        plt.figure()

        # Show grid (flip so origin matches world frame visually)
        plt.imshow(grid_np, origin="lower")

        plt.colorbar(label="Occupancy (0=free, 100=occupied)")
        plt.title("Occupancy Grid")

        # Save to file (inside your package config or wherever you want)
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

    def rebuild_grid(self):
        if self.workspace_poly is None:
            self.get_logger().warn("No workspace polygon loaded")
            return None

        w = int(math.ceil((self.max_x - self.min_x) / self.resolution))
        h = int(math.ceil((self.max_y - self.min_y) / self.resolution))

        self.grid_width = w
        self.grid_height = h

        grid = [[0 for _ in range(w)] for _ in range(h)]

        # outside workspace = occupied
        for gy in range(h):
            for gx in range(w):
                x, y = self.grid_to_world(gx, gy)
                if not self.inside_poly(x, y):
                    grid[gy][gx] = 100

        goal_cell = None
        if self.goal is not None:
            goal_cell = self.world_to_grid(self.goal[0], self.goal[1])

        
        inflation_radius_m = 0.40
        inflation_cells = int(math.ceil(inflation_radius_m / self.resolution))

        for (x, y) in self.obstacles + self.objects + self.boxes:
            gx, gy = self.world_to_grid(x, y)

            if goal_cell is not None and (gx, gy) == goal_cell:
                print(f"Skipping blocker at goal cell: world=({x}, {y}) grid=({gx}, {gy})")
                continue

            # if 0 <= gx < w and 0 <= gy < h:
            #     grid[gy][gx] = 100
            for dy in range(-inflation_cells, inflation_cells + 1):
                for dx in range(-inflation_cells, inflation_cells + 1):
                    nx = gx + dx
                    ny = gy + dy

                    if not (0 <= nx < w and 0 <= ny < h):
                        continue

                    # circular inflation
                    if dx * dx + dy * dy > inflation_cells * inflation_cells:
                        continue

                    if goal_cell is not None and (nx, ny) == goal_cell:
                        continue

                    grid[ny][nx] = 100


        if goal_cell is not None:
            gx, gy = goal_cell
            if 0 <= gx < w and 0 <= gy < h:
                grid[gy][gx] = 0

            print("goal_cell =", goal_cell, "value =", grid[goal_cell[1]][goal_cell[0]])

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

    # def run_astar(self, grid, start, goal):
    #     """
    #     grid[y][x] == 0    -> free
    #     grid[y][x] != 0    -> occupied
    #
    #     start = (gx, gy)
    #     goal  = (gx, gy)
    #
    #     Return:
    #         [(gx1, gy1), (gx2, gy2), ...]
    #     """
    #
    #     def h(a,b):
    #         return math.sqrt((a[0]-b[0])**2 + (a[1]-b[1])**2)
    #
    #     def reconstruct(came_from, current):
    #         path = [current]
    #         while current in came_from:
    #             current = came_from[current]
    #             path.append(current)
    #         return path[::-1]
    #
    #     open_set = []
    #     heapq.heappush(open_set, (0, start))
    #
    #     came_from = {}
    #
    #     g_score = {}
    #     g_score[start] = 0
    #
    #     f_score = {}
    #     f_score[start] = h(start, goal)
    #
    #     while len(open_set) != 0:
    #         current = open_set[0]
    #         if current == goal:
    #             return reconstruct(came_from, current)
    #
    #         heapq.heappop(open_set)
    #
    #         cx, cy = current
    #
    #         neighbors = [
    #             (cx + 1, cy, 1.0),
    #             (cx - 1, cy, 1.0),
    #             (cx, cy + 1, 1.0),
    #             (cx, cy - 1, 1.0),
    #             (cx + 1, cy + 1, math.sqrt(2)),
    #             (cx - 1, cy + 1, math.sqrt(2)),
    #             (cx + 1, cy - 1, math.sqrt(2)),
    #             (cx - 1, cy - 1, math.sqrt(2)),
    #         ]
    #
    #         for nx, ny, cost in neighbors:
    #             if ny < 0 or ny >= len(grid) or nx < 0 or nx >= len(grid[0]):
    #                 continue
    #
    #             if grid[ny][nx] != 0:
    #                 continue
    #
    #             neighbor = (nx,ny)
    #
    #             tentative_g = g_score[current] + cost
    #             if neighbor not in g_score or tentative_g < g_score[neighbor]:
    #                 came_from[neighbor] = current
    #                 g_score[neighbor] = tentative_g
    #                 f = tentative_g + h(neighbor, goal)
    #                 heapq.heappush(open_set, (f, neighbor))
    #
    #
    #
    #     return []


        

    # def astar(self, grid, start, goal):
    #     """
    #     grid[y][x] = 0 free, !=0 occupied
    #     start = (x, y)
    #     goal  = (x, y)
    #     """
    #
    #     def h(a, b):
    #         dx = a[0] - b[0]
    #         dy = a[1] - b[1]
    #         return math.sqrt(dx * dx + dy * dy)
    #
    #     def reconstruct(came_from, current):
    #         path = [current]
    #         while current in came_from:
    #             current = came_from[current]
    #             path.append(current)
    #         return path[::-1]
    #
    #     open_heap = []
    #     heapq.heappush(open_heap, (0, start))
    #
    #     came_from = {}
    #     g_score = {start: 0}
    #
    #     while open_heap:
    #         _, current = heapq.heappop(open_heap)
    #
    #         if current == goal:
    #             return reconstruct(came_from, current)
    #
    #         cx, cy = current
    #
    #         neighbors = [
    #             (cx + 1, cy, 1.0),
    #             (cx - 1, cy, 1.0),
    #             (cx, cy + 1, 1.0),
    #             (cx, cy - 1, 1.0),
    #             (cx + 1, cy + 1, math.sqrt(2)),
    #             (cx - 1, cy + 1, math.sqrt(2)),
    #             (cx + 1, cy - 1, math.sqrt(2)),
    #             (cx - 1, cy - 1, math.sqrt(2)),
    #         ]
    #
    #         for nx, ny, cost in neighbors:
    #             if ny < 0 or ny >= len(grid) or nx < 0 or nx >= len(grid[0]):
    #                 continue
    #
    #             if grid[ny][nx] != 0:
    #                 continue
    #
    #             neighbor = (nx, ny)
    #             tentative_g = g_score[current] + cost
    #
    #             if neighbor not in g_score or tentative_g < g_score[neighbor]:
    #                 came_from[neighbor] = current
    #                 g_score[neighbor] = tentative_g
    #                 f = tentative_g + h(neighbor, goal)
    #                 heapq.heappush(open_heap, (f, neighbor))
    #
    #     return []





def main():
    rclpy.init()
    node = AStarPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
