import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'

        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0

        msg.pose.pose.orientation.z = 0.983
        msg.pose.pose.orientation.w = 0.183

        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        # publish once after short delay
        self.timer = self.create_timer(1.0, self.publish_once)

    def publish_once(self):
        self.publisher_.publish(self.msg)
        self.get_logger().info('Initial pose published')
        rclpy.shutdown()

def main():
    rclpy.init()
    node = InitialPosePublisher()
    node.msg = PoseWithCovarianceStamped()
    node.msg.header.frame_id = 'map'
    node.msg.pose.pose.orientation.z = 0.983
    node.msg.pose.pose.orientation.w = 0.183
    node.create_timer(1.0, node.publish_once)
    rclpy.spin(node)

if __name__ == '__main__':
    main()