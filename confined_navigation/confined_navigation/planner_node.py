#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.interpolate import splprep, splev

class PlannerNode(Node):
    def __init__(self):
        super().__init__('planner_node')
        self.current_pose = None
        self.goal = None
        self.costmap = None

        self.sub_goal = self.create_subscription(PointStamped, '/exploration_frontiers', self.goal_callback, 10)
        self.sub_pose = self.create_subscription(PoseStamped, '/uav1/control_manager/uav_state', self.pose_callback, 10)
        self.sub_costmap = self.create_subscription(OccupancyGrid, '/local_costmap', self.costmap_callback, 10)

        self.traj_pub = self.create_publisher(PoseArray, '/planned_trajectory', 10)
        self.ref_pub = self.create_publisher(PoseStamped, '/uav1/control_manager/reference', 10)

        self.timer = self.create_timer(0.5, self.publish_smooth_trajectory)

    def pose_callback(self, msg):
        self.current_pose = msg.pose

    def goal_callback(self, msg):
        self.goal = msg.point

    def costmap_callback(self, msg):
        self.costmap = msg

    def is_safe(self, x, y):
        if self.costmap is None:
            return True
        idx = int((y - self.costmap.info.origin.position.y) / self.costmap.info.resolution) * self.costmap.info.width + \
              int((x - self.costmap.info.origin.position.x) / self.costmap.info.resolution)
        return self.costmap.data[idx] == 0 if idx < len(self.costmap.data) else False

    def publish_smooth_trajectory(self):
        if self.current_pose is None or self.goal is None:
            return

        # Trajetória Bezier suave, evitando cantos (desvio de 0.5m das paredes)
        points = np.array([
            [self.current_pose.position.x, self.current_pose.position.y],
            [self.current_pose.position.x + 0.5, self.current_pose.position.y + 0.5],
            [self.goal.x - 0.5, self.goal.y - 0.5],
            [self.goal.x, self.goal.y]
        ])

        tck, u = splprep(points.T, s=0, k=3)
        u_new = np.linspace(0, 1, 50)
        spline_points = splev(u_new, tck)
        poses = []
        for i in range(50):
            if not self.is_safe(spline_points[0][i], spline_points[1][i]):
                self.get_logger().warn("Trajetória insegura! Recalculando...")
                return

            pose = PoseStamped()
            pose.header.frame_id = "local_origin"
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = spline_points[0][i]
            pose.pose.position.y = spline_points[1][i]
            pose.pose.position.z = 2.0  # Altura fixa
            pose.pose.orientation.w = 1.0
            poses.append(pose)

        arr = PoseArray()
        arr.header = poses[0].header
        arr.poses = [p.pose for p in poses]
        self.traj_pub.publish(arr)

        # Envia ponto à frente
        ref = poses[5]
        ref.header.stamp = self.get_clock().now().to_msg()
        self.ref_pub.publish(ref)

def main():
    rclpy.init()
    node = PlannerNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()