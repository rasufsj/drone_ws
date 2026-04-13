# 13 — Módulos Principais (linha por linha)

Este documento explica **linha por linha** os quatro módulos Python principais
do pacote `yolo_pad_pose`.

---

## Índice

1. [`yolo_pad_pose_ros2.py`](#1-yolo_pad_pose_ros2py)
2. [`odom_tf_broadcaster.py`](#2-odom_tf_broadcasterpy)
3. [`capture_dataset.py`](#3-capture_datasetpy)
4. [`base_waypoint_publisher.py`](#4-base_waypoint_publisherpy)

---

## 1. `yolo_pad_pose_ros2.py`

Nó ROS 2 principal. Recebe imagens RGB e depth sincronizadas de duas câmeras,
executa inferência YOLO, projeta a detecção para o frame `uav1/base_link`
via TF e publica a posição relativa do pad como `PointStamped`.

---

### 1.1 Imports e dependências

```python
import math          # usado em math.hypot (distância euclidiana) e math.atan2
import time          # usado em time.monotonic() para throttle de warnings

import rclpy                          # biblioteca de cliente ROS 2 Python
from rclpy.node import Node           # classe base para todos os nós ROS 2

from sensor_msgs.msg import Image, CameraInfo
# Image: mensagem ROS para imagens (RGB e depth)
# CameraInfo: parâmetros intrínsecos da câmera (fx, fy, cx, cy, distorção)

from geometry_msgs.msg import PointStamped, Point
# PointStamped: ponto 3D com cabeçalho (stamp + frame_id)
# Point: ponto 3D simples (x, y, z)

from cv_bridge import CvBridge
# Converte mensagens ROS Image para arrays NumPy (OpenCV)

import numpy as np
# Processamento numérico: cálculo de profundidade, recorte de janela, mediana

from message_filters import Subscriber, ApproximateTimeSynchronizer
# Subscriber: wrapper de subscriber com message_filters
# ApproximateTimeSynchronizer: sincroniza mensagens de múltiplos tópicos
#   com timestamps aproximadamente iguais (tolerância configurável)

from ultralytics import YOLO
# Carrega e executa modelos YOLO v8 (detecção de objetos)

import tf2_ros                        # biblioteca TF2 para ROS 2
from tf2_geometry_msgs import do_transform_point
# Aplica um transform (TransformStamped) a um PointStamped
```

---

### 1.2 Função `range_filter_check`

```python
def range_filter_check(right: float, front: float, max_range_m: float) -> bool:
    """Return True if the detection passes the range filter (safe to publish).
    ..."""
```

```python
    if max_range_m <= 0.0:
        return True
    # Se max_range_m for zero ou negativo, o filtro está DESABILITADO.
    # Retorna True imediatamente (qualquer detecção é aceita).
    # Permite desabilitar o filtro via parâmetro ROS: -p max_base_range_m:=0.0
```

```python
    return math.hypot(right, front) <= max_range_m
    # Calcula a distância euclidiana horizontal entre o drone e a detecção:
    #   distance = sqrt(right² + front²)
    # Retorna True se a distância for <= ao limite configurado.
    # Exemplo: right=3.0, front=4.0 → hypot=5.0; max=6.0 → True (aceita)
    # Exemplo: right=3.0, front=4.0 → hypot=5.0; max=4.9 → False (rejeita)
```

---

### 1.3 Função `robust_depth_m`

```python
def robust_depth_m(depth_m: np.ndarray, u: int, v: int, win: int = 7) -> float:
```

```python
    h, w = depth_m.shape[:2]
    # Obtém altura (h) e largura (w) da imagem de profundidade.
    # shape[:2] ignora o canal (para imagens monocromáticas shape é (h, w)).
```

```python
    r = win // 2
    # Raio da janela de busca. Para win=7: r=3.
    # A janela terá tamanho (2r+1) × (2r+1) = 7×7 pixels.
```

```python
    u0, u1 = max(0, u - r), min(w, u + r + 1)
    v0, v1 = max(0, v - r), min(h, v + r + 1)
    # Define os limites da janela centrada em (u, v), garantindo que não
    # ultrapasse as bordas da imagem (clamp entre 0 e w/h).
    # u0..u1: colunas; v0..v1: linhas.
```

```python
    patch = depth_m[v0:v1, u0:u1]
    # Recorta o patch 7×7 centrado no centro da bounding box detectada.
    # Índice NumPy: [linhas, colunas] → [v0:v1, u0:u1].
```

```python
    vals = patch[np.isfinite(patch) & (patch > 0.0)]
    # Filtra pixels válidos: descarta NaN, Inf e valores <= 0.
    # np.isfinite: True para números finitos (não NaN, não Inf)
    # patch > 0.0: descarta profundidades nulas ou negativas (sensor inválido)
```

```python
    if vals.size == 0:
        return float("nan")
    # Se não há nenhum pixel válido na janela, retorna NaN.
    # O chamador verifica np.isfinite(Z) antes de usar o valor.
```

```python
    return float(np.median(vals))
    # Retorna a mediana dos pixels válidos.
    # A mediana é mais robusta que a média: um único pixel espúrio não
    # afeta o resultado, ao contrário da média aritmética.
```

---

### 1.4 Classe `YoloPadPose.__init__`

```python
class YoloPadPose(Node):
    def __init__(self):
        super().__init__("yolo_pad_pose")
        # Inicializa o nó ROS 2 com o nome "yolo_pad_pose".
        # super().__init__ chama Node.__init__, registrando o nó no sistema ROS.
```

#### Bloco de declaração de parâmetros

```python
        self.declare_parameter("model_path", "best.pt")
        # Caminho para o arquivo de pesos do modelo YOLO.
        # Padrão: "best.pt" no diretório de trabalho.
        # Sobrescrever via: -p model_path:=/path/to/best.pt

        self.declare_parameter("conf", 0.25)
        # Limiar de confiança mínima para aceitar uma detecção.
        # Padrão conservador (0.25); usar 0.7 em produção.

        self.declare_parameter("class_pad", 0)
        # DEPRECATED: alias legado de base_class_id. Mantido para
        # compatibilidade com launches antigos que usavam class_pad.

        self.declare_parameter("base_class_id", 0)
        # ID da classe YOLO que representa a base do landing pad.
        # Deve corresponder ao índice 0 do YAML do dataset.

        self.declare_parameter("h_class_id", 1)
        # ID da classe YOLO que representa o marcador H.
        # Deve corresponder ao índice 1 do YAML do dataset.

        self.declare_parameter("target_frame", "uav1/base_link")
        # Frame TF de destino para a transformação da detecção.
        # Todas as posições publicadas estarão neste frame.

        self.declare_parameter("fixed_z", 1.5)
        # Componente Z fixo no PointStamped publicado.
        # Não representa a profundidade real; é um valor simbólico para
        # facilitar o uso por nós de controle que operam em 2D (x, y).

        self.declare_parameter("max_base_range_m", 6.0)
        # Alcance máximo (m) para publicar detecções da base.
        # Detecções a mais de 6 m são suprimidas (com warning throttled).

        self.declare_parameter("max_h_range_m", 6.0)
        # Idem para o marcador H.
```

#### Bloco de tópicos de câmera

```python
        self.declare_parameter("front_rgb",  "/uav1/rgbd_front/color/image_raw")
        # Tópico da imagem RGB da câmera frontal (RealSense D435).

        self.declare_parameter("front_depth","/uav1/rgbd_front/depth/image_raw")
        # Tópico da imagem de profundidade da câmera frontal.
        # Encoding esperado: 32FC1 (metros, float) ou 16UC1 (milímetros, uint16).

        self.declare_parameter("front_info", "/uav1/rgbd_front/color/camera_info")
        # Tópico com os parâmetros intrínsecos (fx, fy, cx, cy) da câmera frontal.

        self.declare_parameter("down_rgb",   "/uav1/rgbd_down/color/image_raw")
        self.declare_parameter("down_depth", "/uav1/rgbd_down/depth/image_raw")
        self.declare_parameter("down_info",  "/uav1/rgbd_down/color/camera_info")
        # Idem para a câmera apontada para baixo (usada na fase de pouso).
```

#### Bloco de leitura de parâmetros e compatibilidade legada

```python
        self.model_path = self.get_parameter("model_path").value
        self.conf = float(self.get_parameter("conf").value)
        self.base_class_id = int(self.get_parameter("base_class_id").value)
        self.h_class_id = int(self.get_parameter("h_class_id").value)
        # Lê os quatro parâmetros principais e converte para os tipos corretos.
        # float() e int() garantem que parâmetros passados como string via
        # linha de comando sejam corretamente tipados.

        _class_pad = int(self.get_parameter("class_pad").value)
        if _class_pad != 0 and self.base_class_id == 0:
            self.base_class_id = _class_pad
            self.get_logger().warning(
                "[DEPRECATED] 'class_pad' parameter is deprecated; "
                f"use 'base_class_id' instead. Using class_pad={_class_pad}."
            )
        # Lógica de compatibilidade reversa:
        # Se o usuário passou class_pad != 0 (valor não-padrão) E
        # base_class_id ainda está no padrão (0, não foi sobrescrito),
        # usa class_pad como base_class_id e emite aviso de depreciação.
        # Se base_class_id foi explicitamente definido, ele tem precedência.
```

#### Bloco de carregamento do modelo YOLO

```python
        self.get_logger().info(f"Loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)
        # Instancia o modelo YOLO a partir do arquivo .pt.
        # YOLO() carrega os pesos em memória (GPU ou CPU, dependendo do ambiente).
        # Esta operação pode demorar alguns segundos na primeira execução.

        self.bridge = CvBridge()
        # Cria um conversor ROS Image ↔ NumPy array.
        # Reutilizado em cada callback para evitar alocação repetida.
```

#### Bloco de TF

```python
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=10.0))
        # Buffer que armazena os últimos 10 segundos de transforms TF.
        # Permite lookups retroativos (útil quando o timestamp da câmera
        # é ligeiramente anterior ao momento do processamento).

        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        # Subscreve /tf e /tf_static e popula o tf_buffer automaticamente.
        # Deve ser criado depois do buffer; precisa do nó como segundo argumento.
```

#### Bloco de publishers

```python
        self.pub_base = self.create_publisher(
            PointStamped, "/landing_pad/base_relative_position", 10)
        # Publisher principal para a base (classe 0).
        # Queue size 10: mantém até 10 mensagens em fila antes de descartar.

        self.pub_h = self.create_publisher(
            PointStamped, "/landing_pad/h_relative_position", 10)
        # Publisher para o marcador H (classe 1).

        self.pub = self.create_publisher(PointStamped, "/landing_pad/relative_position", 10)
        # Alias legado: espelha exatamente pub_base.
        # Mantido para compatibilidade com nós antigos que subscrevem
        # /landing_pad/relative_position.

        self.pub_front = self.create_publisher(PointStamped, "/landing_pad/front_optical_point", 10)
        self.pub_down = self.create_publisher(PointStamped, "/landing_pad/down_optical_point", 10)
        # Tópicos de debug: publicam o mesmo PointStamped mas separados
        # por câmera (frontal vs. baixo) para diagnóstico via RViz/echo.
```

#### Bloco de subscribers sincronizados

```python
        qos = rclpy.qos.qos_profile_sensor_data
        # Perfil QoS para sensores: BEST_EFFORT (não garante entrega),
        # KEEP_LAST (1 mensagem no buffer), sem histórico completo.
        # Adequado para câmeras onde perder um frame é aceitável.

        self.sub_f_rgb = Subscriber(self, Image, self.get_parameter("front_rgb").value, qos_profile=qos)
        self.sub_f_depth = Subscriber(self, Image, self.get_parameter("front_depth").value, qos_profile=qos)
        self.sub_f_info = Subscriber(self, CameraInfo, self.get_parameter("front_info").value, qos_profile=qos)
        # Três Subscribers independentes para RGB, depth e CameraInfo da câmera frontal.
        # Subscriber (message_filters) não chama callback diretamente;
        # ele alimenta o ApproximateTimeSynchronizer.

        self.ts_front = ApproximateTimeSynchronizer([self.sub_f_rgb, self.sub_f_depth, self.sub_f_info], 10, 0.10)
        # Sincronizador temporal aproximado para a câmera frontal.
        # Parâmetros:
        #   [sub_f_rgb, sub_f_depth, sub_f_info] → lista de subscribers a sincronizar
        #   10  → queue_size: buffer de 10 mensagens por tópico
        #   0.10 → slop: tolerância temporal (0.10 s = 100 ms)
        # Quando três mensagens com timestamps dentro de 100 ms são recebidas,
        # o callback é chamado com as três simultaneamente.

        self.ts_front.registerCallback(lambda rgb, depth, info: self.process(rgb, depth, info, "front"))
        # Registra a função de callback para o sincronizador frontal.
        # Usa lambda para injetar o parâmetro "front" sem alterar a assinatura de process().

        # (idem para a câmera de baixo, com tag "down")
        self.sub_d_rgb = Subscriber(self, Image, self.get_parameter("down_rgb").value, qos_profile=qos)
        self.sub_d_depth = Subscriber(self, Image, self.get_parameter("down_depth").value, qos_profile=qos)
        self.sub_d_info = Subscriber(self, CameraInfo, self.get_parameter("down_info").value, qos_profile=qos)
        self.ts_down = ApproximateTimeSynchronizer([self.sub_d_rgb, self.sub_d_depth, self.sub_d_info], 10, 0.10)
        self.ts_down.registerCallback(lambda rgb, depth, info: self.process(rgb, depth, info, "down"))
```

#### Bloco de estado de throttle

```python
        self._base_range_warn_t: float = -float('inf')
        self._h_range_warn_t: float = -float('inf')
        # Timestamps da última vez que um warning de range foi emitido,
        # inicializados em -inf para garantir que o primeiro warning
        # seja sempre emitido.
        # Throttle: emite no máximo 1 warning por segundo por classe,
        # evitando spam no log quando o pad está fora do alcance.
```

---

### 1.5 Método `_detection_to_base_link`

```python
    def _detection_to_base_link(self, depth_m, depth_header, u, v, fx, fy, cx_param, cy_param, source):
        """Convert a pixel detection at (u, v) to a PointStamped in target_frame."""
```

```python
        h_img, w_img = depth_m.shape[:2]
        u = int(np.clip(u, 0, w_img - 1))
        v = int(np.clip(v, 0, h_img - 1))
        # Garante que (u, v) está dentro dos limites da imagem.
        # np.clip: limita o valor ao intervalo [0, w-1] e [0, h-1].
        # Necessário pois o centro da bounding box pode estar ligeiramente
        # fora da imagem em casos extremos.
```

```python
        Z = robust_depth_m(depth_m, u, v, win=7)
        if not np.isfinite(Z) or Z <= 0.0:
            return None
        # Obtém a profundidade Z no ponto (u, v) com janela 7×7.
        # Se Z for NaN/Inf ou não-positivo, retorna None (detecção inválida).
        # Causas comuns: espelho, superfície refletiva, fora do range do sensor.
```

```python
        Xc = (u - cx_param) * Z / fx
        Yc = (v - cy_param) * Z / fy
        Zc = Z
        # Projeção inversa: pixel (u, v) + profundidade Z → ponto 3D no
        # referencial óptico da câmera (X direita, Y baixo, Z frente).
        # cx_param, cy_param: ponto principal (centro óptico, da CameraInfo)
        # fx, fy: distâncias focais em pixels
        # Fórmula: X_camera = (u - cx) * Z / fx
```

```python
        p_cam = PointStamped()
        p_cam.header = depth_header
        p_cam.point = Point(x=float(Xc), y=float(Yc), z=float(Zc))
        # Cria um PointStamped com o ponto no frame óptico da câmera.
        # depth_header contém frame_id (ex: "uav1/rgbd_down") e timestamp
        # da mensagem de depth, necessário para o lookup TF.
```

```python
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,      # frame destino: uav1/base_link
                p_cam.header.frame_id,  # frame origem: uav1/rgbd_down (ou front)
                rclpy.time.Time(),      # tempo: mais recente disponível
                timeout=rclpy.duration.Duration(seconds=0.05),
                # Espera no máximo 50 ms pelo transform.
                # Se não houver transform disponível em 50 ms, lança exceção.
            )
            p_base = do_transform_point(p_cam, tf)
            # Aplica o transform: rotaciona e translada p_cam do frame
            # da câmera para uav1/base_link.
        except Exception as e:
            self.get_logger().warn(f"[{source}] TF failed ...")
            return None
        # Qualquer exceção TF (transform não disponível, timeout, etc.)
        # é capturada silenciosamente (com log de warn) e retorna None.
```

```python
        front = p_base.point.x
        right = -p_base.point.y
        # base_link usa convenção NED: x=frente, y=esquerda, z=cima.
        # front = x do base_link (direção do nariz do drone)
        # right = -y do base_link (y positivo é esquerda → negativo é direita)
```

```python
        out = PointStamped()
        out.header = p_base.header
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self.target_frame
        out.point.x = float(right)    # x = deslocamento para a direita
        out.point.y = float(front)    # y = deslocamento para frente
        out.point.z = float(self.fixed_z)  # z = fixo (1.5 m por padrão)
        return out, Z
        # Retorna o PointStamped final e a profundidade bruta Z (para log).
        # A convenção x=right, y=front é específica deste sistema e deve
        # ser respeitada pelos nós consumidores (ex: base_waypoint_publisher).
```

---

### 1.6 Método `process`

```python
    def process(self, rgb_msg, depth_msg, info_msg, source):
```

```python
        fx = info_msg.k[0]   # focal length X (pixels)
        fy = info_msg.k[4]   # focal length Y (pixels)
        cx = info_msg.k[2]   # principal point X (pixels)
        cy = info_msg.k[5]   # principal point Y (pixels)
        # A matriz K (3×3) no CameraInfo é armazenada como vetor de 9 elementos:
        #   K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
        # Índices: fx=k[0], cx=k[2], fy=k[4], cy=k[5]

        if fx <= 0.0 or fy <= 0.0:
            return
        # Guarda de segurança: CameraInfo ainda não recebida ou inválida.
        # fx e fy zero indicam câmera não calibrada; evita divisão por zero.
```

```python
        try:
            bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding="bgr8")
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().warning(f"[{source}] cv_bridge error: {e}")
            return
        # Converte mensagens ROS para arrays NumPy.
        # "bgr8": formato esperado pelo YOLO (OpenCV usa BGR por padrão).
        # "passthrough": mantém o encoding original da depth (32FC1 ou 16UC1),
        #   sem conversão de tipo, para calcular a profundidade corretamente.
```

```python
        if depth_msg.encoding == "32FC1":
            depth_m = depth.astype(np.float32)
        elif depth_msg.encoding == "16UC1":
            depth_m = depth.astype(np.float32) * 0.001
        else:
            depth_m = depth.astype(np.float32)
        # Normaliza a profundidade para metros (float32):
        # 32FC1: já está em metros (simulação Gazebo).
        # 16UC1: está em milímetros (câmera física RealSense); divide por 1000.
        # Outros encodings: assume metros (fallback).
```

```python
        res = self.model.predict(source=bgr, conf=self.conf, verbose=False)[0]
        # Executa inferência YOLO na imagem BGR.
        # conf=self.conf: aplica o filtro de confiança mínima internamente.
        # verbose=False: suprime output do YOLO no terminal.
        # [0]: pega o resultado do primeiro (e único) frame do batch.

        if res.boxes is None or len(res.boxes) == 0:
            return
        # Se não há detecções, encerra o callback imediatamente.
        # Evita processamento desnecessário quando o pad não está visível.
```

```python
        best_by_class: dict = {}
        for b in res.boxes:
            cls = int(b.cls.item())        # ID da classe detectada (0, 1 ou 2)
            conf_val = float(b.conf.item()) # Confiança da detecção (0.0 a 1.0)
            x1, y1, x2, y2 = map(float, b.xyxy[0].tolist())
            # Coordenadas da bounding box em pixels: (x1,y1) top-left, (x2,y2) bottom-right

            if cls not in best_by_class or conf_val > best_by_class[cls][0]:
                best_by_class[cls] = (conf_val, x1, y1, x2, y2)
        # Para cada classe, mantém apenas a detecção com maior confiança.
        # Garante que apenas uma posição por classe seja publicada por frame.
        # Resultado: dict {0: (conf, x1,y1,x2,y2), 1: ..., 2: ...}
```

```python
        max_base_range_m = float(self.get_parameter("max_base_range_m").value)
        max_h_range_m = float(self.get_parameter("max_h_range_m").value)
        # Lê os parâmetros de alcance a cada callback.
        # Permite alterar os limites em tempo real via ros2 param set.
```

```python
        if self.base_class_id in best_by_class:
            conf_val, x1, y1, x2, y2 = best_by_class[self.base_class_id]
            u = int(round((x1 + x2) * 0.5))  # coordenada X do centro da bbox
            v = int(round((y1 + y2) * 0.5))  # coordenada Y do centro da bbox
            # Calcula o centro da bounding box detectada em pixels.
            # A profundidade é lida no centro (ponto mais representativo do objeto).
```

```python
            result = self._detection_to_base_link(
                depth_m, depth_msg.header, u, v, fx, fy, cx, cy, source
            )
            if result is not None:
                out_base, Z = result
                right = out_base.point.x
                front = out_base.point.y
                if not range_filter_check(right, front, max_base_range_m):
                    # Detecção fora do alcance: emite warning throttled (1/s)
                    _now = time.monotonic()
                    if _now - self._base_range_warn_t >= 1.0:
                        _range = math.hypot(right, front)
                        self.get_logger().warning(
                            f"[{source}] BASE range={_range:.2f}m > ..."
                        )
                        self._base_range_warn_t = _now
                else:
                    self.pub_base.publish(out_base)
                    # Publica em /landing_pad/base_relative_position
                    self.pub.publish(out_base)
                    # Publica no alias legado /landing_pad/relative_position
                    if source == "front":
                        self.pub_front.publish(out_base)
                    elif source == "down":
                        self.pub_down.publish(out_base)
                    # Publica no tópico de debug separado por câmera
```

---

### 1.7 Função `main`

```python
def main():
    rclpy.init()
    # Inicializa o sistema ROS 2 (contexto global, DDS/Zenoh, etc.).
    # Deve ser chamado uma vez antes de qualquer nó ser criado.

    node = YoloPadPose()
    # Instancia o nó (declaração de parâmetros, carregamento do modelo,
    # criação de publishers/subscribers).

    node.get_logger().info("yolo_pad_pose node started.")
    # Log de inicialização confirmando que o nó está pronto.

    try:
        rclpy.spin(node)
        # Entra no loop de eventos ROS 2: processa callbacks de subscribers,
        # timers e serviços indefinidamente até ser interrompido.
    except KeyboardInterrupt:
        pass
        # Ctrl+C é tratado silenciosamente (sem traceback).
    finally:
        node.destroy_node()
        # Destrói o nó: cancela subscribers, publishers e libera recursos.
        rclpy.shutdown()
        # Finaliza o sistema ROS 2 limpo (encerra DDS/Zenoh).
```

---

## 2. `odom_tf_broadcaster.py`

Nó auxiliar que subscreve `nav_msgs/Odometry` do MAVROS e publica o
transform dinâmico correspondente em `/tf`, completando a árvore TF.

---

### 2.1 Docstring do módulo

```python
"""Broadcast TF from a nav_msgs/Odometry topic.

MAVROS publishes nav_msgs/Odometry on /uav1/mavros/local_position/odom with
header.frame_id='uav1/map' and child_frame_id='uav1/base_link', but it does
NOT automatically write these frames to /tf.  This node fills that gap so the
TF tree is connected and tools like tf2_echo, RViz and tf2-based transforms
work correctly.
...
"""
```

O MAVROS publica a odometria, mas **não escreve automaticamente em `/tf`**.
Este nó preenche essa lacuna.

---

### 2.2 Imports

```python
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
# Mensagem que representa um transform TF: translação + rotação com timestamp.

from nav_msgs.msg import Odometry
# Mensagem de odometria: pose (posição + orientação) + twist (velocidade).

import tf2_ros
# TransformBroadcaster: publica transforms em /tf.
```

---

### 2.3 `OdomTfBroadcaster.__init__`

```python
    def __init__(self):
        super().__init__('odom_tf_broadcaster')
        # Registra o nó com nome 'odom_tf_broadcaster'.

        self.declare_parameter('odom_topic', '/uav1/mavros/local_position/odom')
        # Tópico de odometria a subscrever.
        # Padrão: tópico MAVROS do drone uav1.

        self.declare_parameter('tf_parent_frame_override', '')
        # Se não-vazio, substitui o frame pai (msg.header.frame_id).
        # Útil para renomear frames sem modificar o MAVROS.

        self.declare_parameter('tf_child_frame_override', '')
        # Se não-vazio, substitui o frame filho (msg.child_frame_id).

        self.declare_parameter('use_odom_header_stamp', True)
        # True (padrão): usa o timestamp da mensagem de odometria.
        # False: usa o tempo atual do nó (rclpy.Time).
        # O padrão True é mais preciso; False pode ser útil em simulações
        # onde o relógio do simulador e o relógio do nó divergem.

        self._broadcaster = tf2_ros.TransformBroadcaster(self)
        # Cria o broadcaster de TF dinâmico.
        # Publica em /tf (QoS BEST_EFFORT por padrão).

        self._sub = self.create_subscription(
            Odometry, odom_topic, self._odom_cb, 10)
        # Subscreve o tópico de odometria com queue_size=10.
```

---

### 2.4 Callback `_odom_cb`

```python
    def _odom_cb(self, msg: Odometry):
        t = TransformStamped()
        # Cria a mensagem de transform vazia.

        if self._use_header_stamp:
            t.header.stamp = msg.header.stamp
            # Usa o timestamp da odometria (sincronizado com o sensor).
        else:
            t.header.stamp = self.get_clock().now().to_msg()
            # Usa o tempo atual do nó (útil se o timestamp da odom estiver errado).

        parent = self._parent_override or msg.header.frame_id
        # Se override não-vazio: usa o override.
        # Caso contrário: usa o frame_id da mensagem (ex: 'uav1/map').

        child = self._child_override or msg.child_frame_id or 'base_link'
        # Ordem de precedência:
        # 1. override explícito
        # 2. child_frame_id da mensagem (ex: 'uav1/base_link')
        # 3. fallback 'base_link' se a mensagem não tiver child_frame_id

        t.header.frame_id = parent    # frame pai do transform
        t.child_frame_id = child      # frame filho do transform

        pos = msg.pose.pose.position
        t.transform.translation.x = pos.x
        t.transform.translation.y = pos.y
        t.transform.translation.z = pos.z
        # Copia a translação da pose da odometria para o transform.

        ori = msg.pose.pose.orientation
        t.transform.rotation.x = ori.x
        t.transform.rotation.y = ori.y
        t.transform.rotation.z = ori.z
        t.transform.rotation.w = ori.w
        # Copia o quaternião de orientação para o transform.

        self._broadcaster.sendTransform(t)
        # Publica o transform em /tf.
        # Após esta chamada, qualquer nó pode fazer lookup_transform entre
        # 'uav1/map' e 'uav1/base_link' (incluindo yolo_pad_pose_ros2).
```

---

## 3. `capture_dataset.py`

Utilitário de captura de pares imagem+depth para rotulagem. O arquivo
encontra-se vazio na versão atual do repositório (`main`); a funcionalidade
de captura foi implementada em `dataset_capture/` (pacote separado).

O entry point `capture_dataset` no `setup.py` aponta para
`dataset_capture.capture_dataset:main`, indicando que o módulo foi movido
para o pacote `dataset_capture`.

---

## 4. `base_waypoint_publisher.py`

Nó de missão que acumula posições de bases detectadas pelo YOLO, aplica
clustering no referencial mundo e guia o drone numa missão
nearest-neighbour visitando todas as bases.

---

### 4.1 Funções auxiliares puras

#### `merge_radius_from_area`

```python
def merge_radius_from_area(area_m2: float) -> float:
    """Compute the merge radius (m) from a circular merge area (m²)."""

    if area_m2 <= 0.0:
        return 0.0
    # Garante que área negativa ou nula retorne raio 0 (sem clustering).

    return math.sqrt(area_m2 / math.pi)
    # Interpreta area_m2 como a área de um círculo e resolve para o raio:
    #   A = pi * r²  →  r = sqrt(A / pi)
    # Exemplo: area=2.25 → r = sqrt(2.25/pi) ≈ 0.846 m
    # Bases dentro de 0.846 m uma da outra são fundidas (mesma base física).
```

#### `yaw_from_quat`

```python
def yaw_from_quat(x, y, z, w):
    """Extract yaw (Z-axis rotation) from a quaternion."""
    siny_cosp = 2.0 * (w * z + x * y)
    # Numerador da fórmula atan2 para yaw.
    # Componente que mede rotação em torno do eixo Z.

    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    # Denominador da fórmula atan2 para yaw.
    # Derivado da fórmula de conversão quaternião → ângulos de Euler.

    return math.atan2(siny_cosp, cosy_cosp)
    # Retorna o yaw em radianos no intervalo (-pi, +pi].
    # atan2 é usado (em vez de asin/acos) por ser numericamente estável
    # em todos os quadrantes.
```

#### `body_to_world`

```python
def body_to_world(front, right, cur_x, cur_y, yaw):
    """Project a body-frame detection to world frame (ENU) using odom yaw."""

    wx = cur_x + front * math.cos(yaw) + right * math.sin(yaw)
    # Projeção do vetor corpo→mundo no eixo X do mapa (ENU: X = leste).
    # cos(yaw)*front: componente da frente do drone no eixo X do mapa.
    # sin(yaw)*right: componente da direita do drone no eixo X do mapa.

    wy = cur_y + front * math.sin(yaw) - right * math.cos(yaw)
    # Projeção no eixo Y do mapa (ENU: Y = norte).
    # O sinal negativo em -right*cos(yaw) é consequência da convenção ENU:
    # o eixo Y é perpendicular ao X e aponta para o norte.

    return wx, wy
    # Retorna a posição absoluta da detecção no referencial mundo (ENU).
```

#### `cluster_update`

```python
def cluster_update(bases, nx, ny, merge_radius, max_bases):
    """Update the base list with a new world-frame detection."""

    best_idx = None
    best_d = math.inf
    for i, b in enumerate(bases):
        d = math.hypot(b.x - nx, b.y - ny)
        # Distância euclidiana entre a detecção nova e a base existente.
        if d < best_d:
            best_idx = i
            best_d = d
    # Encontra a base mais próxima da nova detecção.

    if best_d <= merge_radius:
        bases[best_idx].update(nx, ny)
        return bases, 'merged'
    # Se a base mais próxima está dentro do raio de fusão, atualiza-a
    # via EMA (exponential moving average) com o novo valor.
    # → ação: 'merged'

    if len(bases) < max_bases:
        bases.append(BaseEntry(x=nx, y=ny))
        return bases, 'added'
    # Se não há base próxima E ainda há espaço (< max_bases), adiciona nova.
    # → ação: 'added'

    return bases, 'discarded'
    # Se não há espaço (já atingiu max_bases), descarta a detecção.
    # → ação: 'discarded'
```

#### `BaseEntry.update` (EMA)

```python
    def update(self, nx: float, ny: float) -> None:
        """Merge a new world-frame observation into this estimate via EMA."""
        a = self._ema_alpha   # α = 0.3 (fixo)
        self.x = (1.0 - a) * self.x + a * nx
        self.y = (1.0 - a) * self.y + a * ny
        # EMA: nova estimativa = 70% estimativa anterior + 30% nova medição.
        # α = 0.3 → suaviza o sinal, mas ainda acompanha mudanças reais.
        # Quanto maior α, mais rápido a estimativa segue novas medições.
        self.seen_count += 1
        # Incrementa o contador de observações desta base.
        # Usado por _confirmed_bases() para filtrar bases com poucas observações.
```

---

### 4.2 Classe `BaseWaypointPublisher.__init__`

#### Bloco de parâmetros

```python
        self.declare_parameter("odom_topic", "/uav1/mavros/local_position/odom")
        # Odometria do MAVROS: fornece posição (x, y) e orientação (yaw) do drone.

        self.declare_parameter("det_topic", "/landing_pad/base_relative_position")
        # Tópico de detecção publicado por yolo_pad_pose_ros2.
        # Cada mensagem contém a posição de uma base em uav1/base_link.

        self.declare_parameter("waypoints_topic", "/waypoints")
        # Tópico de waypoints consumido por my_drone_controller (drone_node).
        # Formato: PoseArray com 2 poses: [posição atual, posição alvo].

        self.declare_parameter("controller_state_topic", "/drone_controller/state_voo")
        # Estado do controlador de voo.
        # Publicação de waypoints só ocorre quando state_voo == 2 (hover/missão).

        self.declare_parameter("max_bases", 6)
        # Número máximo de bases a rastrear simultâneamente.

        self.declare_parameter("merge_area_m2", 2.25)
        # Área circular de fusão. Com 2.25 m², raio ≈ 0.846 m.
        # Duas detecções dentro desse raio são consideradas a mesma base.

        self.declare_parameter("min_seen_count", 3)
        # Mínimo de observações para uma base ser considerada "confirmada".
        # Filtra bases detectadas apenas uma vez (possíveis falsos positivos).

        self.declare_parameter("reach_tol_m", 0.10)
        # Tolerância XY (m) para considerar que o drone chegou à base.
        # 0.10 m = 10 cm de precisão de chegada.

        self.declare_parameter("dwell_s", 5.0)
        # Segundos de permanência em cada base visitada antes de partir.

        self.declare_parameter("max_detection_range_m", 6.0)
        # Rejeita detecções de bases a mais de 6 m no frame corpo.
        # Mesmo filtro aplicado por yolo_pad_pose_ros2 (defesa em profundidade).

        self.declare_parameter("max_jump_m", 2.0)
        # DEPRECATED: filtro de salto no frame corpo (substituído por clustering).
        # Mantido apenas para compatibilidade. Se > 0, emite aviso de depreciação.

        self.declare_parameter("require_all_bases", True)
        # True: aguarda todas as max_bases antes de retornar home.
        # False: retorna home quando todas as bases *conhecidas* foram visitadas.
```

#### Bloco de estado da FSM

```python
        self.fsm_state: str = "COLLECT"
        # Estado inicial da máquina de estados finita (FSM).
        # Estados possíveis: COLLECT → NAVIGATE → DWELL → RETURN_HOME → DONE

        self._dwell_start_t: float = 0.0
        # Timestamp (monotonic) de quando o estado DWELL foi iniciado.

        self._dwell_completed: bool = False
        # Guarda para garantir que o log "dwell complete" seja emitido
        # apenas uma vez por evento de dwell (evita log repetido no tick).
```

#### Bloco de ROS I/O

```python
        self.sub_odom = self.create_subscription(
            Odometry, self.odom_topic, self.cb_odom, 10)
        # Subscreve odometria: atualiza (cur_x, cur_y, cur_yaw) e captura home.

        self.sub_det = self.create_subscription(
            PointStamped, self.det_topic, self.cb_det, 10)
        # Subscreve detecções YOLO: acumula posições de bases no mapa.

        self.sub_state_voo = self.create_subscription(
            Int32,
            self.controller_state_topic,
            self.cb_state_voo,
            rclpy.qos.QoSProfile(
                depth=1,
                durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            ),
        )
        # Subscreve o estado do controlador com QoS TRANSIENT_LOCAL:
        # garante que a última mensagem publicada seja recebida mesmo que
        # este nó tenha iniciado depois do controlador já publicar o estado.
        # depth=1: mantém apenas a última mensagem no buffer.

        self.pub_waypoints = self.create_publisher(PoseArray, self.waypoints_topic, 10)
        # Publisher de waypoints: PoseArray com 2 poses [atual, alvo].

        self.timer = self.create_timer(self.publish_period_s, self.tick)
        # Timer que chama self.tick() a cada publish_period_s (padrão 0.25 s = 4 Hz).
        # O tick() é onde a FSM avança e os waypoints são publicados.
```

---

### 4.3 Callbacks

#### `cb_odom`

```python
    def cb_odom(self, msg: Odometry) -> None:
        self.have_odom = True
        # Flag que indica que pelo menos uma mensagem de odometria foi recebida.
        # Outros callbacks e tick() bloqueiam até have_odom ser True.

        self.cur_x = float(msg.pose.pose.position.x)
        self.cur_y = float(msg.pose.pose.position.y)
        # Atualiza a posição XY atual do drone no referencial mundo.

        q = msg.pose.pose.orientation
        self.cur_yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        # Extrai o yaw (heading) do quaternião da odometria.
        # Necessário para converter detecções corpo→mundo em cb_det.

        if not self._have_home:
            self._have_home = True
            self.home_x = self.cur_x
            self.home_y = self.cur_y
            # Captura a posição de decolagem como "home" na primeira mensagem.
            # Após todas as bases serem visitadas, o drone retorna aqui.
```

#### `cb_det`

```python
    def cb_det(self, msg: PointStamped) -> None:
        if not self.have_odom:
            return
        # Sem odometria não é possível converter corpo→mundo; descarta.

        if self.fsm_state == "DONE":
            return
        # Missão encerrada; para de acumular detecções.

        right = float(msg.point.x)   # deslocamento para a direita (m)
        front = float(msg.point.y)   # deslocamento para frente (m)

        det_range = math.hypot(right, front)
        if self.max_detection_range_m > 0.0 and det_range > self.max_detection_range_m:
            # Rejeita detecções muito distantes (outliers ou sensor inválido).
            return

        wx, wy = body_to_world(front, right, self.cur_x, self.cur_y, self.cur_yaw)
        # Projeta a detecção do frame corpo para o referencial mundo (ENU).

        self.bases, action = cluster_update(
            self.bases, wx, wy, self.merge_radius, self.max_bases
        )
        # Atualiza a lista de bases: 'merged', 'added' ou 'discarded'.
```

---

### 4.4 FSM `tick()`

```python
    def tick(self) -> None:
        if not self.have_odom:
            return
        if self.state_voo != 2:
            return
        # Waypoints são publicados apenas quando o controlador
        # está no estado 2 (hover/modo missão).
        # state_voo 0=WAIT, 1=TAKEOFF, 2=HOVER/MISSION, etc.
```

```python
        if self.fsm_state == "COLLECT":
            target = self._pick_nearest_unvisited()
            if target is None:
                return  # Nenhuma base confirmada; aguarda mais detecções
            self.active_target = target
            self.fsm_state = "NAVIGATE"
            self._publish_waypoint(*self.active_target)
            # Transição COLLECT → NAVIGATE ao encontrar a primeira base confirmada.
```

```python
        elif self.fsm_state == "NAVIGATE":
            target = self._pick_nearest_unvisited()
            if target is not None:
                self.active_target = target
            # Atualiza o alvo continuamente: as estimativas EMA melhoram
            # durante o voo, então o target pode ser refinado a cada tick.

            tx, ty = self.active_target
            dist = math.hypot(self.cur_x - tx, self.cur_y - ty)

            if dist <= self.reach_tol_m:
                self._mark_active_visited()
                self._dwell_start_t = time.monotonic()
                self.fsm_state = "DWELL"
                # Chegou ao alvo: marca como visitado e inicia dwell.
            else:
                self._publish_waypoint(tx, ty)
                # Ainda longe: republica o waypoint para manter o controlador
                # navegando (o controlador precisa receber /waypoints periodicamente).
```

```python
        elif self.fsm_state == "DWELL":
            elapsed = time.monotonic() - self._dwell_start_t
            if elapsed >= self.dwell_s:
                if self._all_bases_visited():
                    self.fsm_state = "RETURN_HOME"
                    self._publish_waypoint(self.home_x, self.home_y)
                    # Todas as bases visitadas: retorna ao ponto de decolagem.
                else:
                    next_t = self._pick_nearest_unvisited()
                    if next_t is None:
                        self.fsm_state = "COLLECT"
                        # Sem mais bases confirmadas: volta a coletar.
                    else:
                        self.active_target = next_t
                        self.fsm_state = "NAVIGATE"
                        self._publish_waypoint(*self.active_target)
                        # Próxima base: seleciona a mais próxima não visitada.
```

```python
        elif self.fsm_state == "RETURN_HOME":
            dist = math.hypot(self.cur_x - self.home_x, self.cur_y - self.home_y)
            if dist <= self.reach_tol_m:
                self.fsm_state = "DONE"
                # Chegou em casa: missão encerrada. Nenhum waypoint é publicado.
            else:
                self._publish_waypoint(self.home_x, self.home_y)
                # Ainda longe de casa: republica home como waypoint.
```
