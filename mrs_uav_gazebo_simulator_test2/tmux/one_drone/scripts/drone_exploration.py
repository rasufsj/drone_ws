#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2, LaserScan, Image
from sensor_msgs_py import point_cloud2

# IMPORTS DO SISTEMA MRS
from mrs_msgs.msg import ReferenceStamped
from std_srvs.srv import SetBool, Trigger
import time
import cv2
from cv_bridge import CvBridge

class DroneExploration(Node):
    def __init__(self):
        super().__init__('drone_exploration')
        
        # Variáveis de estado
        self.obstacle_detected = False
        self.min_safe_distance = 1.0
        self.critical_distance = 0.5
        
        # Bridge para processamento de imagem
        self.bridge = CvBridge()
        
        # Dados dos sensores
        self.ouster_data = None
        self.rangefinder_data = None
        
        # Controle de tempo para movimentos
        self.movement_start_time = None
        self.current_step = 0
        
        # QoS profiles
        qos_profile = QoSProfile(depth=10)
        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # === SISTEMA DE CONTROLE MRS ===
        # Publisher para control_manager (ReferenceStamped)
        self.reference_pub = self.create_publisher(
            ReferenceStamped,
            '/uav1/control_manager/reference',
            qos_profile
        )
        
        # Clientes de serviço
        self.arming_client = self.create_client(SetBool, '/uav1/uav_manager/arm')
        self.offboard_client = self.create_client(Trigger, '/uav1/uav_manager/offboard')
        self.takeoff_client = self.create_client(Trigger, '/uav1/uav_manager/takeoff')

        # Subscriber para odometria
        self.odom_sub = self.create_subscription(
            Odometry, '/uav1/odometry/odom_main', self.odom_cb, qos_profile)
        
        # Ouster LiDAR
        self.ouster_sub = self.create_subscription(
            PointCloud2, '/uav1/ouster/points', self.ouster_cb, qos_sensor)
        
        # Rangefinder
        self.rangefinder_sub = self.create_subscription(
            LaserScan, '/uav1/rangefinder/range', self.rangefinder_cb, qos_sensor)
        
        # Variáveis de estado do drone
        self.current_pose = None
        self.pose_received = False
        self.current_position = [0.0, 0.0, 0.0]
        self.current_yaw = 0.0
        
        # Controle de evasão inteligente
        self.obstacle_avoidance_count = 0
        self.last_avoidance_time = 0
        self.avoidance_strategy_index = 0
        
        # Estratégias de desvio disponíveis
        self.avoidance_strategies = [
            "forward_right",    # Frente + Direita
            "forward_left",     # Frente + Esquerda  
            "backward_right",   # Trás + Direita
            "backward_left"     # Trás + Esquerda
        ]
        
        # SEQUÊNCIA DO QUADRADO 8x8 METROS
        self.exploration_sequence = [
            # [x, y, z, yaw, duration, description]
            [1.0, -1.0, 2.0, 0.0, 8.0, "Takeoff e Hover Inicial"],
            
            # Lado 1: Frente (Eixo X)
            [6.0, -1.0, 2.0, 0.0, 15.0, "Lado 1: Frente 5m"],
            
            # Lado 2: Direita (Eixo Y) 
            [6.0, -5.0, 2.0, 0.0, 15.0, "Lado 2: Direita 5m"],
            
            # Lado 3: Trás (Eixo X negativo)
            [1.0, -5.0, 2.0, 0.0, 15.0, "Lado 3: Trás 5m"],
            
            # Lado 4: Esquerda (Eixo Y negativo)
            [1.0, -1.0, 2.0, 0.0, 15.0, "Lado 4: Esquerda 5m"],
            
            [1.0, -1.0, 2.0, 0.0, 5.0, "Hover Final no ponto inicial"],
        ]
        
        # Timer - 5Hz
        self.timer = self.create_timer(0.2, self.control_loop)
        
        self.state = "INITIALIZING"
        self.current_target = [0.0, 0.0, 2.0, 0.0]
        self.service_check_time = self.get_clock().now()
        self.service_check_attempts = 0
        
        self.get_logger().info("🚀 Drone exploration started - QUADRADO 8x8m")
        
    def odom_cb(self, msg):
        """Callback para odometria"""
        self.current_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        
        # Extrair yaw da orientação
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        
        self.current_yaw = math.atan2(2.0 * (w * z + x * y),
                                    1.0 - 2.0 * (y * y + z * z))

    def wait_for_service_with_timeout(self, client, service_name, timeout=10.0):
        """Aguarda serviço com timeout"""
        start_time = time.time()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > timeout:
                self.get_logger().error(f'❌ Timeout waiting for service: {service_name}')
                return False
        return True

    def call_service(self, client, request, name, timeout=5.0):
        """Chama serviços com timeout"""
        if not self.wait_for_service_with_timeout(client, name, timeout):
            return False
            
        future = client.call_async(request)
        start_time = time.time()
        
        while rclpy.ok():
            if future.done():
                try:
                    result = future.result()
                    return True
                except Exception as e:
                    return False
                    
            if time.time() - start_time > timeout:
                return False
                
            time.sleep(0.1)
        
        return False

    def initialize_drone(self):
        """Inicialização simplificada do drone"""
        self.get_logger().info("🛠️  Initializing drone...")
        
        # Tentar modo OFFBOARD
        offboard_req = Trigger.Request()
        if self.call_service(self.offboard_client, offboard_req, "offboard_mode"):
            time.sleep(2)
            
            # Enviar comando inicial
            self.send_reference_command(0.0, 0.0, 2.0, 0.0)
            time.sleep(2)
            
            # Tentar ARM
            arm_req = SetBool.Request()
            arm_req.data = True
            
            if self.call_service(self.arming_client, arm_req, "arming"):
                time.sleep(2)
                return True
        
        return False

    def ouster_cb(self, msg):
        """Callback para dados do Ouster LiDAR"""
        try:
            self.ouster_data = msg
        except:
            pass
    
    def rangefinder_cb(self, msg):
        """Callback para dados do rangefinder"""
        try:
            self.rangefinder_data = msg
        except:
            pass
    
    def sensor_processing(self):
        """Processa dados de sensores para detecção de obstáculos"""
        try:
            obstacle_detected = False
            
            # Processar rangefinder primeiro (sensor frontal)
            if self.rangefinder_data and hasattr(self.rangefinder_data, 'range'):
                range_val = self.rangefinder_data.range
                if 0.1 < range_val < self.critical_distance:
                    self.get_logger().warn(f"🚨 RANGEFINDER: {range_val:.2f}m")
                    obstacle_detected = True
                elif range_val < self.min_safe_distance:
                    self.get_logger().info(f"⚠️  Rangefinder warning: {range_val:.2f}m")
            
            # Processar Ouster LiDAR
            if not obstacle_detected and self.ouster_data:
                obstacle_detected = self.process_ouster_data()
            
            self.obstacle_detected = obstacle_detected
            
        except Exception as e:
            pass

    def process_ouster_data(self):
        """Processa dados do Ouster LiDAR - FILTRAGEM MAIS RESTRITIVA"""
        try:
            points = list(point_cloud2.read_points(
                self.ouster_data, 
                field_names=("x", "y", "z"), 
                skip_nans=True
            ))
            
            if not points:
                return False
            
            critical_points = 0
            total_points_checked = 0
            
            for point in points:
                x, y, z = point
                total_points_checked += 1
                
                # DISTÂNCIA EUCLIDIANA ATÉ O DRONE
                distance_to_drone = math.sqrt(x**2 + y**2 + z**2)
                
                # FILTRAGEM MAIS RESTRITIVA
                if distance_to_drone < 0.8:
                    continue
                    
                if distance_to_drone > 5.0:
                    continue
                
                # Considerar apenas pontos em frente ao drone
                cos_yaw = math.cos(self.current_yaw)
                sin_yaw = math.sin(self.current_yaw)
                
                # Rotacionar ponto para alinhar com orientação do drone
                point_x_rotated = x * cos_yaw - y * sin_yaw
                point_y_rotated = x * sin_yaw + y * cos_yaw
                
                # FILTROS MAIS RESTRITIVOS
                if point_x_rotated < 0.3:
                    continue
                    
                if point_x_rotated > self.min_safe_distance:
                    continue
                    
                if abs(point_y_rotated) > 0.8:
                    continue
                
                if abs(z) > 1.0:
                    continue
                    
                critical_points += 1
            
            # Log detalhado para debug
            if total_points_checked > 0:
                self.get_logger().debug(f"LiDAR: {critical_points}/{total_points_checked} critical points")
            
            if critical_points > 15:
                self.get_logger().warn(f"🚨 OUSTER: {critical_points} critical points (threshold: 15)")
                return True
                
            return False
            
        except Exception as e:
            self.get_logger().error(f"Error processing Ouster data: {e}")
            return False

    def send_reference_command(self, x, y, z, yaw=0.0):
        """Envia comando ReferenceStamped"""
        msg = ReferenceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'uav1/local_origin'
        msg.reference.position.x = float(x)
        msg.reference.position.y = float(y)
        msg.reference.position.z = float(z)
        msg.reference.heading = float(yaw)

        self.reference_pub.publish(msg)
        self.current_target = [x, y, z, yaw]

    def calculate_avoidance_point(self, strategy):
        """Calcula ponto de evasão - MESMA ALTURA"""
        current_x, current_y, current_z = self.current_position
        
        # Distâncias de evasão
        forward_distance = 2.0
        backward_distance = 2.0
        lateral_distance = 2.0

        # Vetores de direção baseados no yaw atual
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        
        if strategy == "forward_right":
            # Frente + Direita (MESMA ALTURA)
            dx = forward_distance * cos_yaw - lateral_distance * sin_yaw
            dy = forward_distance * sin_yaw + lateral_distance * cos_yaw
            avoid_z = current_z
            
        elif strategy == "forward_left":
            # Frente + Esquerda (MESMA ALTURA)
            dx = forward_distance * cos_yaw + lateral_distance * sin_yaw
            dy = forward_distance * sin_yaw - lateral_distance * cos_yaw
            avoid_z = current_z
            
        elif strategy == "backward_right":
            # Trás + Direita (MESMA ALTURA)
            dx = -backward_distance * cos_yaw - lateral_distance * sin_yaw
            dy = -backward_distance * sin_yaw + lateral_distance * cos_yaw
            avoid_z = current_z
            
        elif strategy == "backward_left":
            # Trás + Esquerda (MESMA ALTURA)
            dx = -backward_distance * cos_yaw + lateral_distance * sin_yaw
            dy = -backward_distance * sin_yaw - lateral_distance * cos_yaw
            avoid_z = current_z
            
        else:
            # Fallback: recuar
            dx = -backward_distance * cos_yaw
            dy = -backward_distance * sin_yaw
            avoid_z = current_z
        
        avoid_x = current_x + dx
        avoid_y = current_y + dy
            
        return avoid_x, avoid_y, avoid_z

    def smart_avoidance(self):
        """Estratégia de evasão - MESMA ALTURA"""
        current_time = time.time()
        
        # Se muitas evasões em pouco tempo, tentar recuar MAIS
        if current_time - self.last_avoidance_time < 20.0 and self.obstacle_avoidance_count > 2:
            self.get_logger().warn("🆘 EMERGENCY: Too many avoidances, MAJOR RETREAT")
            
            # Tentar escapar recuando MUITO
            current_x, current_y, current_z = self.current_position
            
            # Recuar 5 metros na direção oposta
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            escape_x = current_x - 5.0 * cos_yaw
            escape_y = current_y - 5.0 * sin_yaw
            
            self.get_logger().info(f"🆘 MAJOR retreat to: [{escape_x:.1f}, {escape_y:.1f}]")
            
            # Recuar por 8 segundos
            start_time = time.time()
            while time.time() - start_time < 8.0 and rclpy.ok():
                self.send_reference_command(escape_x, escape_y, current_z, self.current_yaw)
                time.sleep(0.2)
            
            # Pular para próximo waypoint após escape
            self.current_step += 1
            self.obstacle_avoidance_count = 0
            self.movement_start_time = self.get_clock().now()
            self.obstacle_detected = False
            return
        
        self.obstacle_avoidance_count += 1
        self.last_avoidance_time = current_time
        
        # Selecionar estratégia de evasão
        strategy = self.avoidance_strategies[self.avoidance_strategy_index]
        self.avoidance_strategy_index = (self.avoidance_strategy_index + 1) % len(self.avoidance_strategies)
        
        self.get_logger().warn(f"🔄 Avoidance #{self.obstacle_avoidance_count} - Strategy: {strategy}")
        
        # Calcular ponto de evasão (MESMA ALTURA)
        avoid_x, avoid_y, avoid_z = self.calculate_avoidance_point(strategy)
        
        self.get_logger().info(f"🎯 Avoiding to: [{avoid_x:.1f}, {avoid_y:.1f}, {avoid_z:.1f}] - {strategy}")
        
        # Enviar comando de evasão por 6 segundos
        start_time = time.time()
        avoidance_duration = 6.0
        
        while time.time() - start_time < avoidance_duration and rclpy.ok():
            self.send_reference_command(avoid_x, avoid_y, avoid_z, self.current_yaw)
            time.sleep(0.2)
        
        # Aguardar estabilização
        time.sleep(2.0)
        
        self.obstacle_detected = False
        self.get_logger().info("✅ Avoidance completed")

    def control_loop(self):
        """Loop principal de controle"""
        try:
            if self.state == "INITIALIZING":
                # Tentar inicializar uma vez e seguir mesmo se falhar
                self.initialize_drone()
                self.state = "EXPLORING"
                self.movement_start_time = self.get_clock().now()
                self.current_step = 0
                self.get_logger().info("🎯 Starting 8x8m square exploration...")
                        
            elif self.state == "EXPLORING":
                # Processar sensores
                self.sensor_processing()
                
                # Evitar obstáculos se detectados
                if self.obstacle_detected:
                    self.smart_avoidance()
                    return
                
                # Executar sequência de waypoints
                if self.current_step < len(self.exploration_sequence):
                    current_waypoint = self.exploration_sequence[self.current_step]
                    x, y, z, yaw, duration, description = current_waypoint
                    
                    # Enviar comando continuamente
                    self.send_reference_command(x, y, z, yaw)
                    
                    # Verificar progresso
                    current_time = self.get_clock().now()
                    elapsed_time = (current_time - self.movement_start_time).nanoseconds / 1e9
                    
                    # Calcular distância até o alvo
                    dist_to_target = math.sqrt(
                        (x - self.current_position[0])**2 + 
                        (y - self.current_position[1])**2
                    )
                    
                    # Log de progresso a cada 5 segundos
                    if int(elapsed_time) % 5 == 0:
                        self.get_logger().info(f"📊 {description} - Distância: {dist_to_target:.1f}m - Tempo: {elapsed_time:.0f}s")
                    
                    # Resetar contador de evasões se movendo bem
                    if elapsed_time > 5.0 and self.obstacle_avoidance_count > 0:
                        self.obstacle_avoidance_count = 0
                        self.get_logger().info("🔄 Reset avoidance counter - moving well")
                    
                    # Mudar waypoint após tempo OU se estiver muito próximo do alvo
                    if elapsed_time >= duration or dist_to_target < 0.5:
                        self.get_logger().info(f"✅ {description} COMPLETED")
                        self.current_step += 1
                        
                        if self.current_step < len(self.exploration_sequence):
                            self.movement_start_time = self.get_clock().now()
                            next_wp = self.exploration_sequence[self.current_step]
                            self.get_logger().info(f"🎯 Próximo: {next_wp[5]}")
                        else:
                            self.get_logger().info("🎉 Exploração do quadrado 8x8m COMPLETADA!")
                            self.state = "COMPLETED"
                
                else:
                    self.state = "COMPLETED"
                    
            elif self.state == "COMPLETED":
                # Manter posição final
                last_waypoint = self.exploration_sequence[-1]
                self.send_reference_command(last_waypoint[0], last_waypoint[1], last_waypoint[2], last_waypoint[3])
                
        except Exception as e:
            self.get_logger().error(f"💥 Control loop error: {str(e)}")

def main():
    rclpy.init()
    drone = DroneExploration()
    
    try:
        rclpy.spin(drone)
    except KeyboardInterrupt:
        drone.get_logger().info("🛑 Shutdown requested")
    except Exception as e:
        drone.get_logger().error(f"💥 Unexpected error: {e}")
    finally:
        try:
            drone.destroy_node()
            rclpy.shutdown()
        except:
            pass

if __name__ == '__main__':
    main()