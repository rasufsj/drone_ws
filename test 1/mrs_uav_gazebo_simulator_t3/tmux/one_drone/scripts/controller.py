#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from mrs_msgs.msg import ReferenceStamped
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import PoseStamped
import time


class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')

        # Publisher para o control_manager
        self.pub = self.create_publisher(
            ReferenceStamped,
            '/uav1/control_manager/reference',
            10
        )

        # Clientes de serviço (arming, offboard, takeoff)
        self.arming_client = self.create_client(SetBool, '/uav1/hw_api/arming')
        self.offboard_client = self.create_client(Trigger, '/uav1/hw_api/offboard')
        self.takeoff_client = self.create_client(Trigger, '/uav1/uav_manager/takeoff')

        # Subscriber para verificar a posição atual do drone
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/uav1/control_manager/current_reference',
            self.pose_callback,
            10
        )

        # Variáveis de estado
        self.current_pose = None
        self.pose_received = False
        self.altitude_threshold = 0.5  # Altitude mínima para considerar que já está voando

        # Trajetória definida diretamente no código
        self.trajectory = [
            {"x": 0.0, "y": 0.0, "z": 2.0, "yaw": 0.0},
            {"x": 2.0, "y": 0.0, "z": 2.5, "yaw": 0.0},
            {"x": 2.0, "y": 2.0, "z": 3.0, "yaw": 1.57},
            {"x": 0.0, "y": 2.0, "z": 2.5, "yaw": 3.14},
            {"x": 0.0, "y": 0.0, "z": 2.0, "yaw": 0.0},
        ]

    def pose_callback(self, msg):
        """Callback para receber a posição atual do drone"""
        self.current_pose = msg.pose
        self.pose_received = True
        self.get_logger().info(f'📡 Posição atual: z={msg.pose.position.z:.2f}m', throttle_duration_sec=5)

    def wait_for_pose(self, timeout=10.0):
        """Espera até receber a primeira mensagem de pose"""
        start_time = time.time()
        while not self.pose_received and (time.time() - start_time) < timeout:
            self.get_logger().info('⏳ Aguardando dados de posição...')
            time.sleep(1)
        
        if not self.pose_received:
            self.get_logger().warning('⚠️  Não recebeu dados de posição, continuando...')
        return self.pose_received

    def needs_takeoff(self):
        """Verifica se o drone precisa decolar"""
        if not self.pose_received or self.current_pose is None:
            self.get_logger().warning('⚠️  Sem dados de posição, assumindo que precisa decolar')
            return True
        
        current_altitude = self.current_pose.position.z
        self.get_logger().info(f'📊 Altitude atual: {current_altitude:.2f}m')
        
        # Se estiver muito perto do chão, precisa decolar
        if current_altitude < self.altitude_threshold:
            self.get_logger().info('🚀 Drone no chão, necessário decolar')
            return True
        else:
            self.get_logger().info(f'✈️  Drone já está voando ({current_altitude:.2f}m), pulando decolagem')
            return False

    def call_service(self, client, request, name):
        while not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info(f'⏳ Esperando serviço {name}...')
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(f'✅ Serviço {name} executado com sucesso')
            return True
        else:
            self.get_logger().error(f'❌ Falha ao chamar serviço {name}')
            return False

    def arm_and_takeoff(self):
        """Processo completo de arming e decolagem"""
        # 1) Armar
        req = SetBool.Request()
        req.data = True
        if not self.call_service(self.arming_client, req, "arming"):
            return False
        time.sleep(1)

        # 2) Ativar Offboard
        req = Trigger.Request()
        if not self.call_service(self.offboard_client, req, "offboard"):
            return False
        time.sleep(1)

        # 3) Decolar
        req = Trigger.Request()
        if not self.call_service(self.takeoff_client, req, "takeoff"):
            return False
        
        self.get_logger().info('🛫 Aguardando estabilização após decolagem...')
        time.sleep(5)  # esperar estabilizar
        return True

    def arm_only(self):
        """Apenas arma o drone sem decolar (para quando já está voando)"""
        # 1) Armar
        req = SetBool.Request()
        req.data = True
        if not self.call_service(self.arming_client, req, "arming"):
            return False
        time.sleep(1)

        # 2) Ativar Offboard
        req = Trigger.Request()
        if not self.call_service(self.offboard_client, req, "offboard"):
            return False
        time.sleep(1)

        self.get_logger().info('✅ Drone armado e em modo offboard (já estava voando)')
        return True

    def follow_trajectory(self):
        """Executa a trajetória definida"""
        self.get_logger().info('🛤️  Iniciando trajetória...')
        
        for i, point in enumerate(self.trajectory):
            msg = ReferenceStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'uav1/local_origin'
            msg.reference.position.x = float(point['x'])
            msg.reference.position.y = float(point['y'])
            msg.reference.position.z = float(point['z'])
            msg.reference.heading = float(point['yaw'])

            self.pub.publish(msg)
            self.get_logger().info(f"📍 Ponto {i+1}/{len(self.trajectory)}: x={point['x']}, y={point['y']}, z={point['z']}, yaw={point['yaw']:.2f}")
            time.sleep(3)  # espera entre pontos

        self.get_logger().info('🎯 Trajetória concluída!')


def main(args=None):
    rclpy.init(args=args)
    node = DroneController()

    try:
        # Aguardar um pouco para receber dados de posição
        node.get_logger().info('🔍 Verificando estado do drone...')
        node.wait_for_pose(timeout=5.0)

        # Verificar se precisa decolar
        if node.needs_takeoff():
            node.get_logger().info('🚀 Iniciando sequência de decolagem...')
            if not node.arm_and_takeoff():
                node.get_logger().error('❌ Falha na decolagem, abortando...')
                return
        else:
            node.get_logger().info('⚡ Drone já está voando, apenas armando...')
            if not node.arm_only():
                node.get_logger().error('❌ Falha no arming, abortando...')
                return

        # Seguir trajetória
        node.follow_trajectory()

    except Exception as e:
        node.get_logger().error(f'💥 Erro durante execução: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()