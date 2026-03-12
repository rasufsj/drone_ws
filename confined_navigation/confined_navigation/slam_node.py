#!/usr/bin/env python3
# confined_navigation/confined_navigation/slam_node.py
# Mapeamento 3D completo: A-LOAM (LiDAR) + OctoMap + projeção 2D

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import PointCloud2
from octomap_msgs.msg import Octomap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import struct
from std_msgs.msg import Header

class SlamNode(Node):
    def __init__(self):
        super().__init__('slam_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # === Subscribers ===
        self.cloud_sub = self.create_subscription(
            PointCloud2,
            '/uav1/ouster/point_cloud',
            self.cloud_callback,
            qos
        )

        self.octomap_sub = self.create_subscription(
            Octomap,
            '/octomap_binary',
            self.octomap_callback,
            10
        )

        # === Publishers ===
        self.map_2d_pub = self.create_publisher(OccupancyGrid, '/global_costmap', 10)
        self.map_debug_pub = self.create_publisher(PointCloud2, '/mapped_points', 10)

        self.get_logger().info("SLAM Node iniciado: A-LOAM + OctoMap + 2D projection ativo!")

        # Estado interno
        self.octomap = None
        self.last_map_time = self.get_clock().now()

    def cloud_callback(self, msg):
        # Apenas repassa nuvens mapeadas para visualização
        debug_cloud = PointCloud2()
        debug_cloud.header = msg.header
        debug_cloud.height = 1
        debug_cloud.width = msg.width
        debug_cloud.fields = msg.fields
        debug_cloud.is_bigendian = msg.is_bigendian
        debug_cloud.point_step = msg.point_step
        debug_cloud.row_step = msg.row_step
        debug_cloud.is_dense = msg.is_dense
        debug_cloud.data = msg.data
        self.map_debug_pub.publish(debug_cloud)

    def octomap_callback(self, msg):
        self.octomap = msg
        # Atualiza mapa 2D a cada 3 segundos
        if (self.get_clock().now() - self.last_map_time).nanoseconds * 1e-9 > 3.0:
            self.publish_2d_map()
            self.last_map_time = self.get_clock().now()

    def publish_2d_map(self):
        if self.octomap is None:
            return

        resolution = self.octomap.resolution
        data = self.octomap.data
        width = int(self.octomap.width / resolution)
        height = int(self.octomap.height / resolution)

        grid = OccupancyGrid()
        grid.header.frame_id = "uav1/local_origin"
        grid.header.stamp = self.get_clock().now().to_msg()
        grid.info.resolution = resolution
        grid.info.width = width
        grid.info.height = height
        grid.info.origin.position.x = self.octomap.origin.position.x
        grid.info.origin.position.y = self.octomap.origin.position.y
        grid.info.origin.position.z = 0.0
        grid.info.origin.orientation.w = 1.0

        # Converte OctoMap binário para grid 2D (projeção vertical 0.3-2.5m)
        grid_data = []
        for i, occupied in enumerate(data):
            z = (i // (width * height)) * resolution + self.octomap.origin.position.z
            if 0.3 <= z <= 2.5:  # faixa de interesse
                grid_data.append(100 if occupied else 0)
            else:
                grid_data.append(-1)  # desconhecido

        grid.data = grid_data
        self.map_2d_pub.publish(grid)
        self.get_logger().info(f"Mapa 2D publicado: {width}x{height} @ {resolution:.2f}m")

def main(args=None):
    rclpy.init(args=args)
    node = SlamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()