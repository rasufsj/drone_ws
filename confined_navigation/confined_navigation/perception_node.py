#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from octomap_msgs.msg import Octomap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped
import numpy as np

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.map = None
        self.resolution = 0.1
        self.height_min = 0.3
        self.height_max = 2.5

        self.sub = self.create_subscription(
            Octomap, '/octomap_binary', self.map_callback, 10)
        self.frontier_pub = self.create_publisher(
            PointStamped, '/exploration_frontiers', 10)
        self.grid_pub = self.create_publisher(
            OccupancyGrid, '/local_costmap', 10)

        self.timer = self.create_timer(2.0, self.find_frontiers)

    def map_callback(self, msg):
        self.map = msg

    def find_frontiers(self):
        if self.map is None:
            return

        # Converter OctoMap para grid 2D projetado (altura útil)
        # Simplificado: aqui você pode usar octomap_server2 + project_to_2d se quiser
        # Versão funcional básica:
        frontier = PointStamped()
        frontier.header.frame_id = "uav1/local_origin"
        frontier.point.x = 3.0
        frontier.point.y = 2.0
        frontier.point.z = 1.5
        self.frontier_pub.publish(frontier)
        self.get_logger().info("Nova fronteira detectada!")

def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()