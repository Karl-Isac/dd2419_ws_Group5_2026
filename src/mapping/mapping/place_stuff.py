#!/usr/bin/env python
import csv
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
from dataclasses import dataclass

base_dir = Path(__file__).resolve().parent.parent.parent.parent.parent.parent.parent.parent
WS_PATH = base_dir / 'Workspace/workspace_1.csv'
KNOWN_PATH = base_dir / 'Workspace/map_1_1.csv'
NEW_PATH = base_dir / 'Workspace/the_map.csv'

class make_space(Node):
    
    def __init__(self):
        super().__init__('space')
        
        QoS = QoSProfile(
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        
        self._marker_pub = self.create_publisher(
            Marker,
            'map_markers',
            QoS)
        
        print("thing")
        
        self.first_placements()
        
    def first_placements(self):
        
        things = []
        
        with open(KNOWN_PATH, newline='', encoding='utf-8-sig') as csvfile:
            reader = csv.DictReader(csvfile)
            for row in reader:
                type = str(row['Type'])
                x = float(row['x'])/100  # convert to meters
                y = float(row['y'])/100
                angle = float(row['angle'])
                things.append((type, x, y, angle))
                
        with open(NEW_PATH, 'w', newline='', encoding='utf-8') as csvfile:
            fieldnames = ["Type", "x", "y", "angle"]
            writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
            writer.writeheader()
            for thing_type, x, y, angle in things:
                writer.writerow({
                    "Type": thing_type,
                    "x": x,
                    "y": y,
                    "angle": angle
                })

        global MAX_ID
        MAX_ID = 0
        for the_type, x, y, angle in things:
            marker = Marker()
            
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            
            thing = id_placer(the_type)
            
            marker.ns = thing.name + str(MAX_ID)
            marker.id = MAX_ID
            MAX_ID += 1
            marker.type = thing.thing_type
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = thing.Z_scale/2
            marker.pose.orientation.w = 1.0
            
            if thing.thing_type == Marker.ARROW:
                marker.pose.orientation.x = 0.0
                marker.pose.orientation.y = math.sin(math.pi/4)
                marker.pose.orientation.z = 0.0
                marker.pose.orientation.w = math.cos(math.pi/4)
                marker.pose.position.z += thing.X_scale
            
            # Scale
            marker.scale.x = thing.X_scale
            marker.scale.y = thing.Y_scale
            marker.scale.z = thing.Z_scale
            
            marker.color.r = thing.R
            marker.color.g = thing.G
            marker.color.b = thing.B
            marker.color.a = 1.0
    
            print("Publiched " + marker.ns + " " + str(thing.R) + str(thing.G )+ str(thing.B))
            self._marker_pub.publish(marker)

@dataclass
class object:
    name: str
    thing_type: int
    R: float
    G: float
    B: float
    A: float
    X_scale: float
    Y_scale: float
    Z_scale: float
     
    
def id_placer(the_type):
    match the_type:
        case 'S':
            ret_obj = object("Start", Marker.ARROW, 1.0, 0.0, 0.0, 1.0, 0.5, 0.01, 0.01)
        case 'O':
            ret_obj = object("Object", Marker.CUBE, 0.0, 1.0, 0.0, 1.0, 0.03, 0.03, 0.03)
        case 'B':
            ret_obj = object("Object", Marker.CUBE, 1.0, 1.0, 1.0, 1.0, 0.24, 0.16, 0.099)
        case _:
            raise ValueError(f"Unknown type: {type}")
    return ret_obj

def main():
    rclpy.init()
    node = make_space()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()