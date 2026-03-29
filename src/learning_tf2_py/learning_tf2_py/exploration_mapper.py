import rclpy
from rclpy.node import Node
import tf2_ros
from robp_interfaces.msg import Point, String
import csv
from shapely.geometry import Polygon
from shapely.geometry import Point as shapelyPoint
import numpy as np


class ExplorationMapper(Node):
    def __init__(self):
        super().__init__('exploration_mapper')

        # Subscribe to requests, publish unexplored points
        self.pub = self.create_publisher(Point, "/exploration/return_unexplored_point", 10) # z value irrelevant
        self.create_subscription(
            String, '/exploration/request_unexplored_point', self.get_unexplored_point_callback, 10) # content can be anything
        
        # Update the map every 0.5 sec
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Setup map
        # Read workspace csv
        csv_path = "workspace.csv"              # agree on a common location
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

    def get_unexplored_point_callback(self):
        # TODO
        # iterate through grid points, if its unexplored add its i,j index into a list, pick a random one at the end
        # return corresponding x and y
        pass

    def timer_callback(self):
        # TODO
        pass
        # get current robot location
        # draw a cone based on that
        # iterate through grid points, if a point is inside the cone and was unexplored, mark it as explored

        # for visualization also create an image publisher, which outputs the exploration map as a bitmap, robot marked as a black dot or smth

def read_workspace_as_polygon(csv_path):        
        points = []
        
        with open(csv_path, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
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
