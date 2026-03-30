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


class ExplorationMapper(Node):
    def __init__(self):
        super().__init__('exploration_mapper')

        # Subscribe to requests, publish unexplored points
        self.pub = self.create_publisher(Point, "/exploration/return_unexplored_point", 10) # z value 0, if not that represents no more unexplored points left
        self.create_subscription(
            String, '/exploration/request_unexplored_point', self.get_unexplored_point_callback, 10) # content can be anything
        
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
        # Read workspace csv
        csv_path = "workspace.csv"              # agree on a common location, atm its inside dd2419_ws_Group5_2026
        poly = read_workspace_as_polygon(csv_path)

        # Convert read polygon into a grid, where 0 means outside of ws, 1 means unexplored, 2 means explored (nothing is explored at init) 
        resolution = 0.1
        minx, miny, maxx, maxy = poly.bounds

        self.grid_xvalues = np.arange(minx, maxx, resolution)
        self.grid_yvalues = np.arange(miny, maxy, resolution)

        self.grid = np.zeros((len(self.grid_yvalues), len(self.grid_xvalues)), dtype=np.uint8)

        for i, y in enumerate(self.grid_yvalues):
            for j, x in enumerate(self.grid_xvalues):
                if poly.contains(shapelyPoint(x, y)):
                    self.grid[i, j] = 1

    def get_unexplored_point_callback(self,_):
        unexplored_indices = []
        for i, y in enumerate(self.grid_yvalues):
            for j, x in enumerate(self.grid_xvalues):
                if self.grid[i, j] == 1:
                    unexplored_indices.append([i, j])
        i,j = random.choice(unexplored_indices)
        try:
            x = self.grid_xvalues[j]
            y = self.grid_yvalues[i]
            z = 0
        except:     # if any error z => -1 to tell global task planner of crash or everything explored
            x = 0
            y = 0
            z = -1
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

            self.get_logger().info(f"The current robot location is: x={x}, y={y}")

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
                            self.get_logger().info("something just got explored")
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
