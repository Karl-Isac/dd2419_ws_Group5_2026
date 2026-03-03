#!/usr/bin/env python

from visualization_msgs.msg import Marker
import math

import numpy as np
import csv

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion

from geometry_msgs.msg import TransformStamped, PoseStamped
from robp_interfaces.msg import Encoders
from sensor_msgs.msg import Imu
from nav_msgs.msg import Path

########################################################################################
    # TODO: Change path
######################################################################################## 


WS_PATH = '/Users/ki/Desktop/Skola/Robot/dd2419_ws_Group5_2026/Workspace/workspace_1.csv'

class make_space(Node):
    
    def __init__(self):
        super().__init__('space')
        self._marker_pub = self.create_publisher(Marker, 'workspace_marker', 10)
        print("thing")
        self.publish_workspace()
        
    def publish_workspace(self):

        marker = Marker()
        marker.header.frame_id = "odom"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "workspace"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05  # line width

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Rectangle corners
        width = 4.0
        height = 3.0
        
        points = []
        
        with open(WS_PATH, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                x = float(row['x'])/100  # convert to int
                y = float(row['y'])/100
                points.append((x, y))
        points.append(points[0])

        #points = [
        #    (0.0, 0.0),
        #    (width, 0.0),
        #    (width, height),
        #    (0.0, height),
        #    (0.0, 0.0)
        #]

        for x, y in points:
            p = PoseStamped().pose.position
            p.x = x
            p.y = y
            p.z = 0.0
            marker.points.append(p)

        self._marker_pub.publish(marker)

def main():
    rclpy.init()
    node = make_space()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()