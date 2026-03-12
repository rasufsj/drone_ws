#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
from datetime import datetime

class DomainAwareFrameSaver(Node):
    def __init__(self):
        super().__init__('domain_aware_frame_saver')
        
        # Verificar DOMAIN_ID
        domain_id = os.environ.get('ROS_DOMAIN_ID', 'NOT SET')
        print(f"🔍 ROS_DOMAIN_ID: {domain_id}")
        print("💡 Se não for o mesmo da simulação, execute:")
        print("   export ROS_DOMAIN_ID=0  # ou o número correto)")
        print("=" * 50)
        
        self.bridge = CvBridge()
        self.frame_count = {'down': 0, 'front': 0}
        self.save_interval = 0.5
        self.last_save_time = time.time()
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.base_dir = f"frames_domain_{domain_id}_{timestamp}"
        os.makedirs(self.base_dir, exist_ok=True)
        
        print(f"📁 Diretório: {self.base_dir}")
        print("🎯 Aguardando frames... (Pressione Ctrl+C para parar)\n")
        
        # Subscribers
        self.down_sub = self.create_subscription(
            Image, '/uav1/bluefox_down/image_raw', self.down_callback, 10
        )
        self.front_sub = self.create_subscription(
            Image, '/uav1/rgbd_front/color/image_raw', self.front_callback, 10
        )
        
        self.start_time = time.time()
        self.received_any = False
        
        # Timer para verificar se estamos recebendo algo
        self.create_timer(5.0, self.check_connection)
    
    def check_connection(self):
        elapsed = time.time() - self.start_time
        if not self.received_any:
            print(f"⏳ Aguardando... {int(elapsed)}s - Verifique ROS_DOMAIN_ID!")
    
    def down_callback(self, msg):
        if not self.received_any:
            self.received_any = True
            elapsed = time.time() - self.start_time
            print(f"✅ CONECTADO! Primeiro frame DOWN após {elapsed:.1f}s")
        
        if time.time() - self.last_save_time >= self.save_interval:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                filename = f"{self.base_dir}/down_{self.frame_count['down']:06d}.png"
                cv2.imwrite(filename, cv_image)
                self.frame_count['down'] += 1
                self.last_save_time = time.time()
                print(f"📸 DOWN: {filename}")
            except Exception as e:
                print(f"❌ Erro DOWN: {e}")
    
    def front_callback(self, msg):
        if time.time() - self.last_save_time >= self.save_interval:
            try:
                cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
                filename = f"{self.base_dir}/front_{self.frame_count['front']:06d}.png"
                cv2.imwrite(filename, cv_image)
                self.frame_count['front'] += 1
                self.last_save_time = time.time()
                print(f"📸 FRONT: {filename}")
            except Exception as e:
                print(f"❌ Erro FRONT: {e}")

def main():
    rclpy.init()
    node = DomainAwareFrameSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print(f"\n⏹️  Finalizado!")
    finally:
        print(f"\n🎯 Resultados:")
        print(f"   DOWN: {node.frame_count['down']} frames")
        print(f"   FRONT: {node.frame_count['front']} frames")
        print(f"   Pasta: {node.base_dir}")
        
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()