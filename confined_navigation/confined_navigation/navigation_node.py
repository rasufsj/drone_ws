#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        self.ref_pub = self.create_publisher(
            PoseStamped, '/uav1/control_manager/reference', 10)
        self.timer = self.create_timer(0.1, self.hover_safe)

    def hover_safe(self):
        ref = PoseStamped()
        ref.header.frame_id = "local_origin"
        ref.header.stamp = self.get_clock().now().to_msg()
        ref.pose.position.z = 2.0
        ref.pose.orientation.w = 1.0
        self.ref_pub.publish(ref)

def main():
    rclpy.init()
    node = NavigationNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()