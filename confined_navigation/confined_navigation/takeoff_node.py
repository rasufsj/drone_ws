#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time

class TakeoffNode(Node):
    def __init__(self):
        super().__init__('takeoff_node')
        self.arming_client = self.create_client(SetBool, '/uav1/hw_api/arming')
        self.offboard_client = self.create_client(Trigger, '/uav1/hw_api/offboard')
        self.ref_pub = self.create_publisher(PoseStamped, '/uav1/control_manager/reference', 10)

        self.system_ready = False
        self.odom_received = False  # Flag para log único
        self.odom_sub = self.create_subscription(Odometry, '/uav1/estimation_manager/odom_main', self.odom_callback, 10)

        self.timer = self.create_timer(1.0, self.check_and_takeoff)

    def odom_callback(self, msg):
        if not self.odom_received:
            self.get_logger().info("Sistema inicializado: odometria detectada.")
            self.odom_received = True
        self.system_ready = True

    def check_and_takeoff(self):
        if not self.system_ready:
            self.get_logger().warn("Aguardando inicialização do sistema...")
            return

        self.get_logger().info("Sistema pronto. Iniciando decolagem suave...")

        # Arming
        arming_req = SetBool.Request()
        arming_req.data = True
        self.call_service(self.arming_client, arming_req, "Arming")
        time.sleep(2)

        # Offboard
        self.call_service(self.offboard_client, Trigger.Request(), "Offboard")
        time.sleep(2)

        # Hover a 2.0m
        ref = PoseStamped()
        ref.header.frame_id = "local_origin"
        ref.pose.position.z = 2.0
        for _ in range(50):
            ref.header.stamp = self.get_clock().now().to_msg()
            self.ref_pub.publish(ref)
            time.sleep(0.1)

        self.get_logger().info("Decolagem concluída! Iniciando exploração...")
        self.timer.cancel()

    def call_service(self, client, request, name):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"{name} service not available...")
        client.call_async(request)

def main():
    rclpy.init()
    node = TakeoffNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()