#!/usr/bin/env python
import csv
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker

########################################################################################
    # TODO: Change path
######################################################################################## 

base_dir = Path(__file__).resolve().parent.parent.parent.parent.parent.parent.parent.parent
WS_PATH = base_dir / 'Workspace/workspace_1.csv'
print(WS_PATH)

class make_space(Node):
    
    def __init__(self):
        super().__init__('space')
        
        qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self._marker_pub = self.create_publisher(
            Marker,
            'workspace_marker',
            qos)
        
        print("thing")
        
        self.publish_workspace()
        
    def publish_workspace(self):

        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "workspace"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.scale.x = 0.05  # line width

        marker.color.r = 232/255
        marker.color.g = 61/255
        marker.color.b = 132/255
        marker.color.a = 1.0
        #marker.color.r, marker.color.g, marker.color.b, marker.color.a = 232/255, 61/255, 132/255, 1
        
        points = []
        
        with open(WS_PATH, newline='') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                x = float(row['x'])/100  # convert to meters
                y = float(row['y'])/100
                points.append((x, y))
        points.append(points[0])

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