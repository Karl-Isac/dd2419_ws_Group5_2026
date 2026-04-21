import rclpy
from rclpy.node import Node
import tf2_ros
from geometry_msgs.msg import Point
from std_msgs.msg import String
from sensor_msgs.msg import Image
import csv
from shapely.geometry import Polygon
from shapely.geometry import Point as shapelyPoint
import numpy as np
import random
import math
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
from cv_bridge import CvBridge

import os
from ament_index_python.packages import get_package_share_directory

from visualization_msgs.msg import Marker



class ExplorationMapper(Node):
    def __init__(self):
        super().__init__('exploration_mapper')

        # Subscribe to requests, publish unexplored points
        self.pub = self.create_publisher(Point, "/exploration/return_unexplored_point", 10) # z value 0, if not that represents no more unexplored points left
        self.create_subscription(
            String, '/exploration/request_unexplored_point', self.get_unexplored_point_callback, 10) # content can be anything
        
        # marker publisher for viz
        self.marker_pub = self.create_publisher(Marker, "/exploration/goal_marker", 10)
        
        # Publish exploration map as image for debugging
        self.publish_exploration_map = True
        self.image_pub = self.create_publisher(Image, '/exploration/map', 10)
        self.bridge = CvBridge()

        # Update the map every 0.5 sec
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # TF
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer, self)
        
        # Setup map
        package_share = get_package_share_directory('learning_tf2_py')
        csv_path = os.path.join(
            package_share,
            'config',
            'workspace_1.csv'
        )
        poly = read_workspace_as_polygon(csv_path)

        # Convert read polygon into a grid, where:
        # 0 = outside workspace
        # 1 = unexplored
        # 2 = explored
        resolution = 0.1
        minx, miny, maxx, maxy = poly.bounds

        self.grid_xvalues = np.arange(minx, maxx, resolution)
        self.grid_yvalues = np.arange(miny, maxy, resolution)

        self.grid = np.zeros((len(self.grid_yvalues), len(self.grid_xvalues)), dtype=np.uint8)

        for i, y in enumerate(self.grid_yvalues):
            for j, x in enumerate(self.grid_xvalues):
                if poly.contains(shapelyPoint(x, y)):
                    self.grid[i, j] = 1

    def publish_exploration_marker(self, x, y):
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "exploration"
        marker.id = 1
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.1

        # yellow = exploration (different from goal green)
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def count_unexplored_neighbors(self, i, j):
        count = 0
        rows, cols = self.grid.shape

        for di in [-1, 0, 1]:
            for dj in [-1, 0, 1]:
                if di == 0 and dj == 0:
                    continue

                ni = i + di
                nj = j + dj

                if 0 <= ni < rows and 0 <= nj < cols:
                    if self.grid[ni, nj] == 1:
                        count += 1

        return count

    def get_unexplored_point_callback(self, _):
        try:
            # Get robot position
            transform = self.buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time()
            )
            rx = transform.transform.translation.x
            ry = transform.transform.translation.y

            best_score = -float("inf")
            best_cell = None

            rows, cols = self.grid.shape

            for i in range(rows):
                for j in range(cols):
                    # Only choose already explored cells as goals
                    if self.grid[i, j] != 2:
                        continue

                    x = self.grid_xvalues[j]
                    y = self.grid_yvalues[i]

                    unknown_neighbors = self.count_unexplored_neighbors(i, j)

                    # Ignore explored cells that are not near unexplored space
                    if unknown_neighbors == 0:
                        continue

                    dist = math.hypot(x - rx, y - ry)

                    # Simple score:
                    # prefer cells near unexplored space, and somewhat far from robot
                    score = 3.0 * unknown_neighbors + 0.5 * dist

                    if score > best_score:
                        best_score = score
                        best_cell = (i, j)

            if best_cell is None:
                raise RuntimeError("No valid exploration goal found")

            i, j = best_cell
            x = self.grid_xvalues[j]
            y = self.grid_yvalues[i]
            z = 0.0

            self.publish_exploration_marker(x, y)

        except Exception:
            # if any error z => 42 to tell global task planner of crash or everything explored
            x = 0.0
            y = 0.0
            z = 42.0

        msg = Point()
        msg.x = x
        msg.y = y
        msg.z = z

        self.pub.publish(msg)

    def timer_callback(self):
        try:
            # Get the current robot position (latest baselink transform from map)
            transform = self.buffer.lookup_transform(
                'map',        # target frame
                'base_link',  # source frame
                rclpy.time.Time()  # latest
            )
            t = transform.transform.translation
            r = transform.transform.rotation

            x = t.x
            y = t.y
            yaw = quat_to_yaw(r)

            # Create a triangle in front of the robot
            FOV = 60                # degrees,  set these based on detection performance
            detection_range = 0.6   # meters - this is the triangle height, not edge length

            FOVradians = FOV/180*math.pi
            triangle_side_length = detection_range/math.cos(FOVradians/2)

            point1 = shapelyPoint(x,y)
            point2 = shapelyPoint(x+triangle_side_length*math.cos(yaw+FOVradians/2), y+triangle_side_length*math.sin(yaw+FOVradians/2))
            point3 = shapelyPoint(x+triangle_side_length*math.cos(yaw-FOVradians/2), y+triangle_side_length*math.sin(yaw-FOVradians/2))
            seen_triangle = Polygon((point1,point2,point3))

            # Iterate through grid cell centerpoints, if any are within the triangle, mark them as explored
            for i, y in enumerate(self.grid_yvalues):
                for j, x in enumerate(self.grid_xvalues):
                    if self.grid[i,j] == 1:     # if the point is unexplored so far
                        point = shapelyPoint(x,y)
                        if seen_triangle.contains(point):
                            self.grid[i,j] = 2      # mark as explored if its within the detection triangle
        except (LookupException, ConnectivityException, ExtrapolationException):
            self.get_logger().warn("Transform not available")
        
        if self.publish_exploration_map:        # Visualize map in Rviz for debugging
            grayscale_grid = self.grid*127
            grayscale_grid = grayscale_grid[::-1]       # flip image for visualization
            msg = self.bridge.cv2_to_imgmsg(grayscale_grid, encoding='mono8')
            self.image_pub.publish(msg)

def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def read_workspace_as_polygon(csv_path):        
        points = []
        with open(csv_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile, skipinitialspace=True)
            for row in reader:
                x = float(row['x'])/100  # convert to meters
                y = float(row['y'])/100
                points.append((x, y))
        poly = Polygon(points)
        return poly

def main():
    rclpy.init()
    node = ExplorationMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()
