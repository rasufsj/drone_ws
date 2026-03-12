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
        self.min_safe_distance = 1.0  # AUMENTADO: 1m de distância segura
        self.critical_distance = 0.5  # AUMENTADO: 0.5m para crítica
        
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
        self.arming_client = self.create_client(SetBool, '/uav1/hw_api/arming')
        self.offboard_client = self.create_client(Trigger, '/uav1/hw_api/offboard')

        # Subscriber para verificar a posição atual
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/uav1/control_manager/current_reference',
            self.pose_callback,
            10
        )
        
        # Subscribers dos sensores
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
        self.current_position = [0.0, 0.0, 0.0]  # [x, y, z]
        
        # Sequência de exploração MODIFICADA - pontos mais conservadores
        self.exploration_sequence = [
            # [x, y, z, yaw, duration, description]
            [0.0, 0.0, 1.5, 0.0, 15.0, "Hover Start"],
            [4.0, 0.0, 1.5, 0.0, 15.0, "Move Forward 5m"], 
            [0.0, 0.0, 1.5, -1.57, 15.0, "virar 90 graus direita"],
            [4.0, 0.0, 1.5, -1.57, 15.0, "Move Left 5m"],
            [0.0, 0.0, 1.5, -3.14, 15.0, "virar 90 graus direita"],
            [4.0, 0.0, 1.5, -3.14, 15.0, "Move Backward 5m"],
            [0.0, 0.0, 1.5, -4.71, 15.0, "virar 90 graus direita"],
            [4.0, 0.0, 1.5, -4.71, 15.0, "Return to Start"],
            [0.0, 0.0, 1.5, 0.0, 15.0, "virar 90 graus direita"],
            [0.0, 0.0, 1.5, 0.0, 15.0, "Hover Final"],
        ]
        
        # Timer - 5Hz
        self.timer = self.create_timer(0.2, self.control_loop)
        
        self.state = "CHECKING_SERVICES"
        self.current_target = [0.0, 0.0, 1.5, 0.0]
        self.service_check_time = self.get_clock().now()
        self.service_check_attempts = 0
        
        self.get_logger().info("🚀 Drone exploration started - Safe mode enabled")
        
    def pose_callback(self, msg):
        """Callback para receber a posição atual do drone"""
        self.current_pose = msg.pose
        self.pose_received = True
        self.current_position = [
            msg.pose.position.x,
            msg.pose.position.y, 
            msg.pose.position.z
        ]
        self.get_logger().info(f'📡 Posição atual: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}m', throttle_duration_sec=3)

    def wait_for_service_with_timeout(self, client, service_name, timeout=10.0):
        """Aguarda serviço com timeout"""
        start_time = time.time()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > timeout:
                self.get_logger().error(f'❌ Timeout waiting for service: {service_name}')
                return False
            self.get_logger().info(f'⏳ Waiting for service: {service_name}...', throttle_duration_sec=2.0)
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
                    self.get_logger().info(f'✅ Serviço {name} executado com sucesso')
                    return True
                except Exception as e:
                    self.get_logger().error(f'❌ Exception in service {name}: {str(e)}')
                    return False
                    
            if time.time() - start_time > timeout:
                self.get_logger().error(f'❌ Timeout calling service: {name}')
                return False
                
            time.sleep(0.1)
        
        return False

    def arm_drone(self):
        """Tenta armar o drone com fallback"""
        self.get_logger().info("🛠️  Attempting to arm drone...")
        
        # Tentativa 1: Armar via serviço
        req = SetBool.Request()
        req.data = True
        
        if self.call_service(self.arming_client, req, "arming", timeout=5.0):
            time.sleep(1)
            
            # Ativar Offboard
            req_offboard = Trigger.Request()
            if self.call_service(self.offboard_client, req_offboard, "offboard", timeout=5.0):
                self.get_logger().info('✅ Drone armado e em modo offboard via serviços')
                return True
            else:
                self.get_logger().warning('⚠️  Offboard failed, but arming succeeded')
                return True
        else:
            self.get_logger().warning('⚠️  Serviços de arming não disponíveis, assumindo drone já armado')
            return True

    def check_services_available(self):
        """Verifica se os serviços estão disponíveis"""
        arming_available = self.arming_client.wait_for_service(timeout_sec=2.0)
        offboard_available = self.offboard_client.wait_for_service(timeout_sec=2.0)
        
        self.get_logger().info(f'🔍 Serviços - Arming: {arming_available}, Offboard: {offboard_available}')
        return arming_available or offboard_available
    
    def odom_cb(self, msg):
        """Callback para odometria"""
        # Atualizar posição atual da odometria também
        self.current_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
    
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
            # Log do rangefinder para debug
            if hasattr(msg, 'range'):
                if msg.range < self.min_safe_distance:
                    self.get_logger().warn(f"📍 Rangefinder: {msg.range:.2f}m - TOO CLOSE!", throttle_duration_sec=1.0)
        except:
            pass
    
    def sensor_processing(self):
        """Processa dados de sensores para detecção de obstáculos"""
        try:
            obstacle_detected = False
            
            # Processar Ouster LiDAR (principal)
            if self.ouster_data:
                obstacle_detected = self.process_ouster_data()
            
            # Processar rangefinder (backup)
            if self.rangefinder_data and hasattr(self.rangefinder_data, 'range'):
                if self.rangefinder_data.range < self.critical_distance:
                    obstacle_detected = True
                    self.get_logger().warn(f"🚨 RANGEFINDER CRITICAL: {self.rangefinder_data.range:.2f}m")
            
            self.obstacle_detected = obstacle_detected
            
        except Exception as e:
            self.get_logger().error(f"💥 Sensor processing error: {e}")

    def process_ouster_data(self):
        """Processa dados do Ouster LiDAR - MELHORADO"""
        try:
            points = list(point_cloud2.read_points(
                self.ouster_data, 
                field_names=("x", "y", "z"), 
                skip_nans=True
            ))
            
            if not points:
                return False
            
            critical_points = 0
            warning_points = 0
            
            for point in points:
                x, y, z = point
                
                # Converter para coordenadas do drone (o LiDAR pode estar rotacionado)
                # Assumindo que o LiDAR está virado para frente
                front_distance = x  # Distância na frente
                left_distance = y   # Distância à esquerda  
                right_distance = -y # Distância à direita
                
                # Ignorar pontos muito próximos (ruído) ou muito altos/baixos
                if abs(x) < 0.1 or abs(y) < 0.1:  # Ruído
                    continue
                if abs(z) > 2.0:  # Muito alto/baixo
                    continue
                
                # Verificar obstáculos na frente (direção do movimento)
                current_target_x, current_target_y, current_target_z, current_yaw = self.current_target
                current_x, current_y, current_z = self.current_position
                
                # Determinar direção do movimento
                dx = current_target_x - current_x
                dy = current_target_y - current_y
                
                # Normalizar direção
                dist_to_target = math.sqrt(dx**2 + dy**2)
                if dist_to_target > 0.1:  # Só verificar se está se movendo
                    dir_x = dx / dist_to_target
                    dir_y = dy / dist_to_target
                    
                    # Projetar ponto na direção do movimento
                    projection = x * dir_x + y * dir_y
                    
                    if projection > 0:  # Ponto na frente
                        if projection < self.critical_distance:
                            critical_points += 1
                        elif projection < self.min_safe_distance:
                            warning_points += 1
            
            if critical_points > 1:  # Reduzido o threshold
                self.get_logger().warn(f"🚨 OUSTER CRITICAL: {critical_points} points < {self.critical_distance}m")
                return True
            elif warning_points > 3:
                self.get_logger().info(f"⚠️  OUSTER WARNING: {warning_points} points < {self.min_safe_distance}m")
                # Não retorna True aqui, só alerta
                
            return False
            
        except Exception as e:
            self.get_logger().error(f"💥 Ouster processing error: {e}")
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

    def avoid_obstacle(self):
        """Estratégia de evasão de obstáculos - MELHORADA"""
        self.get_logger().warn("🔄 EXECUTING OBSTACLE AVOIDANCE...")
        
        # Usar posição atual como base para evasão
        current_x, current_y, current_z = self.current_position
        
        # Estratégia: subir e voltar para posição segura
        avoid_x = current_x + 0.5  # Voltar 0.5m
        avoid_y = current_y - 0.5
        avoid_z = current_z  # Subir 0.8m
        avoid_yaw = 0.0
        
        self.get_logger().info(f"🔄 Moving to safe position: [{avoid_x:.1f}, {avoid_y:.1f}, {avoid_z:.1f}]")
        
        # Enviar comando de evasão por 4 segundos
        start_time = time.time()
        while time.time() - start_time < 4.0:
            self.send_reference_command(avoid_x, avoid_y, avoid_z, avoid_yaw)
            time.sleep(0.2)
        
        # Aguardar estabilização
        time.sleep(2)
        
        self.obstacle_detected = False
        self.get_logger().info("✅ Avoidance completed - Returning to exploration")
        
        # Reiniciar o passo atual
        self.movement_start_time = self.get_clock().now()

    def control_loop(self):
        """Loop principal de controle"""
        try:
            if self.state == "CHECKING_SERVICES":
                current_time = self.get_clock().now()
                elapsed_time = (current_time - self.service_check_time).nanoseconds / 1e9
                
                if elapsed_time > 5.0:
                    self.service_check_attempts += 1
                    
                    if self.check_services_available() or self.service_check_attempts >= 2:  # Reduzido para 2 tentativas
                        self.state = "ARMING"
                        self.get_logger().info("🔧 Starting arming sequence...")
                    else:
                        self.get_logger().warning(f"🔧 Services not available, attempt {self.service_check_attempts}/2")
                        self.service_check_time = self.get_clock().now()
                        
            elif self.state == "ARMING":
                if self.arm_drone():
                    self.state = "EXPLORING"
                    self.movement_start_time = self.get_clock().now()
                    self.current_step = 0
                    self.get_logger().info("🎯 Starting SAFE exploration sequence...")
                else:
                    self.get_logger().error('❌ Critical failure in arming, cannot proceed')
                    self.state = "FAILED"
                        
            elif self.state == "EXPLORING":
                # Processar sensores para detecção de obstáculos
                self.sensor_processing()
                
                # Verificar e evitar obstáculos (PRIORIDADE MÁXIMA)
                if self.obstacle_detected:
                    self.avoid_obstacle()
                    return
                
                # Executar sequência de waypoints
                if self.current_step < len(self.exploration_sequence):
                    current_waypoint = self.exploration_sequence[self.current_step]
                    x, y, z, yaw, duration, description = current_waypoint
                    
                    # Enviar comando de referência continuamente
                    self.send_reference_command(x, y, z, yaw)
                    
                    # Verificar tempo decorrido
                    current_time = self.get_clock().now()
                    elapsed_time = (current_time - self.movement_start_time).nanoseconds / 1e9
                    
                    # Log a cada 2 segundos
                    if int(elapsed_time) % 2 == 0:
                        self.get_logger().info(f"🔄 {description} - {elapsed_time:.1f}s / {duration}s")
                    
                    # Mudar para próximo waypoint após o tempo
                    if elapsed_time >= duration:
                        self.get_logger().info(f"✓ Completed: {description}")
                        self.current_step += 1
                        
                        if self.current_step < len(self.exploration_sequence):
                            next_waypoint = self.exploration_sequence[self.current_step]
                            self.get_logger().info(f"➡️ Next: {next_waypoint[5]} - Position: [{next_waypoint[0]:.1f}, {next_waypoint[1]:.1f}, {next_waypoint[2]:.1f}]")
                            self.movement_start_time = self.get_clock().now()
                        else:
                            self.get_logger().info("🎉 Exploration sequence completed!")
                            self.state = "COMPLETED"
                
                else:
                    self.state = "COMPLETED"
                    
            elif self.state == "COMPLETED":
                # Manter a última posição
                last_waypoint = self.exploration_sequence[-1]
                self.send_reference_command(last_waypoint[0], last_waypoint[1], last_waypoint[2], last_waypoint[3])
                self.get_logger().info("✅ Mission completed - Holding position", throttle_duration_sec=5.0)
                
            elif self.state == "FAILED":
                self.get_logger().error("💥 Mission failed - Check drone status")
                
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
        drone.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
            [5.0, 0.0, 1.5, 1.57, 5.0, "Move Left 5m"],
            [0.0, 0.0, 1.5, 3.14, 5.0, "virar 90 graus direita"],
            [5.0, 0.0, 1.5, 3.14, 5.0, "Move Backward 5m"],
            [0.0, 0.0, 1.5, 4.71, 5.0, "virar 90 graus direita"],
            [5.0, 0.0, 1.5, 4.71, 5.0, "Return to Start"],
            [0.0, 0.0, 1.5, 0.0, 3.0, "virar 90 graus direita"],
            [0.0, 0.0, 1.5, 0.0, 3.0, "Hover Final"],
        ]
        
        # Timer - 5Hz
        self.timer = self.create_timer(0.2, self.control_loop)
        
        self.state = "CHECKING_SERVICES"
        self.current_target = [0.0, 0.0, 1.5, 0.0]
        self.service_check_time = self.get_clock().now()
        self.service_check_attempts = 0
        
        self.get_logger().info("🚀 Drone exploration started - Safe mode enabled")
        
    def pose_callback(self, msg):
        """Callback para receber a posição atual do drone"""
        self.current_pose = msg.pose
        self.pose_received = True
        self.current_position = [
            msg.pose.position.x,
            msg.pose.position.y, 
            msg.pose.position.z
        ]
        self.get_logger().info(f'📡 Posição atual: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}m', throttle_duration_sec=3)

    def wait_for_service_with_timeout(self, client, service_name, timeout=10.0):
        """Aguarda serviço com timeout"""
        start_time = time.time()
        while not client.wait_for_service(timeout_sec=1.0):
            if time.time() - start_time > timeout:
                self.get_logger().error(f'❌ Timeout waiting for service: {service_name}')
                return False
            self.get_logger().info(f'⏳ Waiting for service: {service_name}...', throttle_duration_sec=2.0)
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
                    self.get_logger().info(f'✅ Serviço {name} executado com sucesso')
                    return True
                except Exception as e:
                    self.get_logger().error(f'❌ Exception in service {name}: {str(e)}')
                    return False
                    
            if time.time() - start_time > timeout:
                self.get_logger().error(f'❌ Timeout calling service: {name}')
                return False
                
            time.sleep(0.1)
        
        return False

    def arm_drone(self):
        """Tenta armar o drone com fallback"""
        self.get_logger().info("🛠️  Attempting to arm drone...")
        
        # Tentativa 1: Armar via serviço
        req = SetBool.Request()
        req.data = True
        
        if self.call_service(self.arming_client, req, "arming", timeout=5.0):
            time.sleep(1)
            
            # Ativar Offboard
            req_offboard = Trigger.Request()
            if self.call_service(self.offboard_client, req_offboard, "offboard", timeout=5.0):
                self.get_logger().info('✅ Drone armado e em modo offboard via serviços')
                return True
            else:
                self.get_logger().warning('⚠️  Offboard failed, but arming succeeded')
                return True
        else:
            self.get_logger().warning('⚠️  Serviços de arming não disponíveis, assumindo drone já armado')
            return True

    def check_services_available(self):
        """Verifica se os serviços estão disponíveis"""
        arming_available = self.arming_client.wait_for_service(timeout_sec=2.0)
        offboard_available = self.offboard_client.wait_for_service(timeout_sec=2.0)
        
        self.get_logger().info(f'🔍 Serviços - Arming: {arming_available}, Offboard: {offboard_available}')
        return arming_available or offboard_available
    
    def odom_cb(self, msg):
        """Callback para odometria"""
        # Atualizar posição atual da odometria também
        self.current_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
    
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
            # Log do rangefinder para debug
            if hasattr(msg, 'range'):
                if msg.range < self.min_safe_distance:
                    self.get_logger().warn(f"📍 Rangefinder: {msg.range:.2f}m - TOO CLOSE!", throttle_duration_sec=1.0)
        except:
            pass
    
    def sensor_processing(self):
        """Processa dados de sensores para detecção de obstáculos"""
        try:
            obstacle_detected = False
            
            # Processar Ouster LiDAR (principal)
            if self.ouster_data:
                obstacle_detected = self.process_ouster_data()
            
            # Processar rangefinder (backup)
            if self.rangefinder_data and hasattr(self.rangefinder_data, 'range'):
                if self.rangefinder_data.range < self.critical_distance:
                    obstacle_detected = True
                    self.get_logger().warn(f"🚨 RANGEFINDER CRITICAL: {self.rangefinder_data.range:.2f}m")
            
            self.obstacle_detected = obstacle_detected
            
        except Exception as e:
            self.get_logger().error(f"💥 Sensor processing error: {e}")

    def process_ouster_data(self):
        """Processa dados do Ouster LiDAR - MELHORADO"""
        try:
            points = list(point_cloud2.read_points(
                self.ouster_data, 
                field_names=("x", "y", "z"), 
                skip_nans=True
            ))
            
            if not points:
                return False
            
            critical_points = 0
            warning_points = 0
            
            for point in points:
                x, y, z = point
                
                # Converter para coordenadas do drone (o LiDAR pode estar rotacionado)
                # Assumindo que o LiDAR está virado para frente
                front_distance = x  # Distância na frente
                left_distance = y   # Distância à esquerda  
                right_distance = -y # Distância à direita
                
                # Ignorar pontos muito próximos (ruído) ou muito altos/baixos
                if abs(x) < 0.1 or abs(y) < 0.1:  # Ruído
                    continue
                if abs(z) > 2.0:  # Muito alto/baixo
                    continue
                
                # Verificar obstáculos na frente (direção do movimento)
                current_target_x, current_target_y, current_target_z, current_yaw = self.current_target
                current_x, current_y, current_z = self.current_position
                
                # Determinar direção do movimento
                dx = current_target_x - current_x
                dy = current_target_y - current_y
                
                # Normalizar direção
                dist_to_target = math.sqrt(dx**2 + dy**2)
                if dist_to_target > 0.1:  # Só verificar se está se movendo
                    dir_x = dx / dist_to_target
                    dir_y = dy / dist_to_target
                    
                    # Projetar ponto na direção do movimento
                    projection = x * dir_x + y * dir_y
                    
                    if projection > 0:  # Ponto na frente
                        if projection < self.critical_distance:
                            critical_points += 1
                        elif projection < self.min_safe_distance:
                            warning_points += 1
            
            if critical_points > 1:  # Reduzido o threshold
                self.get_logger().warn(f"🚨 OUSTER CRITICAL: {critical_points} points < {self.critical_distance}m")
                return True
            elif warning_points > 3:
                self.get_logger().info(f"⚠️  OUSTER WARNING: {warning_points} points < {self.min_safe_distance}m")
                # Não retorna True aqui, só alerta
                
            return False
            
        except Exception as e:
            self.get_logger().error(f"💥 Ouster processing error: {e}")
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

    def avoid_obstacle(self):
        """Estratégia de evasão de obstáculos - MELHORADA"""
        self.get_logger().warn("🔄 EXECUTING OBSTACLE AVOIDANCE...")
        
        # Usar posição atual como base para evasão
        current_x, current_y, current_z = self.current_position
        
        # Estratégia: subir e voltar para posição segura
        avoid_x = current_x + 0.5  # Voltar 0.5m
        avoid_y = current_y - 0.5
        avoid_z = current_z  # Subir 0.8m
        avoid_yaw = 0.0
        
        self.get_logger().info(f"🔄 Moving to safe position: [{avoid_x:.1f}, {avoid_y:.1f}, {avoid_z:.1f}]")
        
        # Enviar comando de evasão por 4 segundos
        start_time = time.time()
        while time.time() - start_time < 4.0:
            self.send_reference_command(avoid_x, avoid_y, avoid_z, avoid_yaw)
            time.sleep(0.2)
        
        # Aguardar estabilização
        time.sleep(2)
        
        self.obstacle_detected = False
        self.get_logger().info("✅ Avoidance completed - Returning to exploration")
        
        # Reiniciar o passo atual
        self.movement_start_time = self.get_clock().now()

    def control_loop(self):
        """Loop principal de controle"""
        try:
            if self.state == "CHECKING_SERVICES":
                current_time = self.get_clock().now()
                elapsed_time = (current_time - self.service_check_time).nanoseconds / 1e9
                
                if elapsed_time > 5.0:
                    self.service_check_attempts += 1
                    
                    if self.check_services_available() or self.service_check_attempts >= 2:  # Reduzido para 2 tentativas
                        self.state = "ARMING"
                        self.get_logger().info("🔧 Starting arming sequence...")
                    else:
                        self.get_logger().warning(f"🔧 Services not available, attempt {self.service_check_attempts}/2")
                        self.service_check_time = self.get_clock().now()
                        
            elif self.state == "ARMING":
                if self.arm_drone():
                    self.state = "EXPLORING"
                    self.movement_start_time = self.get_clock().now()
                    self.current_step = 0
                    self.get_logger().info("🎯 Starting SAFE exploration sequence...")
                else:
                    self.get_logger().error('❌ Critical failure in arming, cannot proceed')
                    self.state = "FAILED"
                        
            elif self.state == "EXPLORING":
                # Processar sensores para detecção de obstáculos
                self.sensor_processing()
                
                # Verificar e evitar obstáculos (PRIORIDADE MÁXIMA)
                if self.obstacle_detected:
                    self.avoid_obstacle()
                    return
                
                # Executar sequência de waypoints
                if self.current_step < len(self.exploration_sequence):
                    current_waypoint = self.exploration_sequence[self.current_step]
                    x, y, z, yaw, duration, description = current_waypoint
                    
                    # Enviar comando de referência continuamente
                    self.send_reference_command(x, y, z, yaw)
                    
                    # Verificar tempo decorrido
                    current_time = self.get_clock().now()
                    elapsed_time = (current_time - self.movement_start_time).nanoseconds / 1e9
                    
                    # Log a cada 2 segundos
                    if int(elapsed_time) % 2 == 0:
                        self.get_logger().info(f"🔄 {description} - {elapsed_time:.1f}s / {duration}s")
                    
                    # Mudar para próximo waypoint após o tempo
                    if elapsed_time >= duration:
                        self.get_logger().info(f"✓ Completed: {description}")
                        self.current_step += 1
                        
                        if self.current_step < len(self.exploration_sequence):
                            next_waypoint = self.exploration_sequence[self.current_step]
                            self.get_logger().info(f"➡️ Next: {next_waypoint[5]} - Position: [{next_waypoint[0]:.1f}, {next_waypoint[1]:.1f}, {next_waypoint[2]:.1f}]")
                            self.movement_start_time = self.get_clock().now()
                        else:
                            self.get_logger().info("🎉 Exploration sequence completed!")
                            self.state = "COMPLETED"
                
                else:
                    self.state = "COMPLETED"
                    
            elif self.state == "COMPLETED":
                # Manter a última posição
                last_waypoint = self.exploration_sequence[-1]
                self.send_reference_command(last_waypoint[0], last_waypoint[1], last_waypoint[2], last_waypoint[3])
                self.get_logger().info("✅ Mission completed - Holding position", throttle_duration_sec=5.0)
                
            elif self.state == "FAILED":
                self.get_logger().error("💥 Mission failed - Check drone status")
                
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
        drone.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()