# 12 — Integração ROS 2 Jazzy

Este documento descreve como executar o pacote `yolo_pad_pose` no ROS 2 Jazzy,
incluindo os parâmetros disponíveis, os tópicos e frames esperados, o ajuste
do limiar de confiança e a integração completa na sessão tmux `one_drone`.

---

## 1. Sequência de inicialização manual

A ordem das três linhas de `source` é **obrigatória**:

```bash
# Passo 1 — ROS 2 Jazzy (sempre primeiro)
source /opt/ros/jazzy/setup.bash

# Passo 2 — workspace compilado com yolo_pad_pose
source ~/ros2_ws/install/setup.bash

# Passo 3 — venv Python com Ultralytics (sempre por último)
source /home/lmnr31/venvs/yolo/bin/activate
```

> **Atenção:** carregar o venv antes do workspace pode fazer o Python do
> venv sobrescrever o Python do ROS 2, quebrando a importação de `rclpy`.

---

## 2. Executar o nó de detecção

### Modelo recomendado (train7)

```bash
python3 -m yolo_pad_pose.yolo_pad_pose_ros2 --ros-args \
  -p model_path:=/home/lmnr31/runs/detect/train7/weights/best.pt \
  -p conf:=0.7
```

### Modelo legado (train4 — referência histórica, não usar em produção)

```bash
python3 -m yolo_pad_pose.yolo_pad_pose_ros2 --ros-args \
  -p model_path:=/home/lmnr31/runs/detect/train4/weights/best.pt \
  -p conf:=0.7
```

---

## 3. Tópicos publicados

| Tópico | Tipo | Descrição |
|--------|------|-----------|
| `/landing_pad/base_relative_position` | `geometry_msgs/PointStamped` | Posição da base (classe 0) em `uav1/base_link` |
| `/landing_pad/h_relative_position` | `geometry_msgs/PointStamped` | Posição do H (classe 1) em `uav1/base_link` |
| `/landing_pad/relative_position` | `geometry_msgs/PointStamped` | Alias legado de `base_relative_position` |
| `/landing_pad/front_optical_point` | `geometry_msgs/PointStamped` | Detecção da câmera frontal (debug) |
| `/landing_pad/down_optical_point` | `geometry_msgs/PointStamped` | Detecção da câmera de baixo (debug) |

### Convenção dos campos do `PointStamped`

```
point.x  = deslocamento para a direita (starboard) em metros
point.y  = deslocamento para frente (nose) em metros
point.z  = fixed_z (padrão 1.5 m; não representa profundidade real)
frame_id = uav1/base_link
```

---

## 4. Tópicos subscritos

| Tópico | Tipo | Origem |
|--------|------|--------|
| `/uav1/rgbd_front/color/image_raw` | `sensor_msgs/Image` | Câmera RealSense frontal |
| `/uav1/rgbd_front/depth/image_raw` | `sensor_msgs/Image` | Depth câmera frontal |
| `/uav1/rgbd_front/color/camera_info` | `sensor_msgs/CameraInfo` | Parâmetros intrínsecos câmera frontal |
| `/uav1/rgbd_down/color/image_raw` | `sensor_msgs/Image` | Câmera RealSense de baixo |
| `/uav1/rgbd_down/depth/image_raw` | `sensor_msgs/Image` | Depth câmera de baixo |
| `/uav1/rgbd_down/color/camera_info` | `sensor_msgs/CameraInfo` | Parâmetros intrínsecos câmera de baixo |

---

## 5. Frames TF necessários

Para que a transformação da detecção óptica para `uav1/base_link` funcione,
a árvore TF deve conter:

```
uav1/map
  └─ uav1/base_link          ← publicado por odom_tf_broadcaster
       └─ uav1/fcu           ← publicado por tf_body_fallback.launch.py
            ├─ uav1/rgbd_down  ← publicado por tf_camera_static.launch.py
            └─ uav1/rgbd_front ← publicado por tf_camera_static.launch.py
```

Verificar a árvore TF:

```bash
ros2 run tf2_tools view_frames
# Gera frames.pdf no diretório atual
```

---

## 6. Ajuste do limiar de confiança (`conf`)

| Cenário | Valor recomendado |
|---------|-------------------|
| Voo de aproximação (pad longe) | `0.4` – `0.5` |
| Voo de pouso (pad próximo) | `0.6` – `0.7` |
| Ambiente com muitos objetos similares | `0.7` |
| Alta taxa de falsos negativos | Reduzir para `0.4` |

Alterar em tempo de execução (sem reiniciar o nó):

```bash
ros2 param set /yolo_pad_pose conf 0.5
```

---

## 7. Integração na sessão tmux `one_drone`

O arquivo `mrs_uav_gazebo_simulator/tmux/one_drone/session.yml` integra o
`yolo_pad_pose` em duas janelas: **`tf_fixes`** e **`Terminal YOLO`**.

### 7.1 Janela `tf_fixes` — Transformações de referência

Esta janela sobe três processos em painéis separados, todos aguardando o
Gazebo inicializar antes de executar:

```yaml
- tf_fixes:
    layout: tiled
    panes:
      - |
        waitForGazebo; sleep 5          # aguarda Gazebo + 5 s extra
        source ~/ros2_ws/install/setup.bash
        ros2 launch yolo_pad_pose tf_body_fallback.launch.py

      - |
        waitForGazebo; sleep 5
        source ~/ros2_ws/install/setup.bash
        ros2 launch yolo_pad_pose tf_camera_static.launch.py

      - |
        waitForGazebo; sleep 5
        source ~/ros2_ws/install/setup.bash
        ros2 run yolo_pad_pose odom_tf_broadcaster
```

#### Descrição linha por linha do primeiro painel

```bash
waitForGazebo; sleep 5
# waitForGazebo: script MRS que bloqueia até o serviço /mrs_drone_spawner
#   estar disponível (Gazebo completamente inicializado).
# sleep 5: 5 s adicionais para o drone ser spawnado e os tópicos estabilizarem.

source ~/ros2_ws/install/setup.bash
# Carrega o workspace ROS 2 compilado, tornando yolo_pad_pose disponível.

ros2 launch yolo_pad_pose tf_body_fallback.launch.py
# Inicia o static_transform_publisher:
#   uav1/base_link → uav1/fcu  (x=0, y=0, z=0, roll=0, pitch=0, yaw=0)
# Necessário quando o MRS core não está rodando (sem mrs_uav_core.launch.py).
```

#### Descrição linha por linha do segundo painel

```bash
waitForGazebo; sleep 5
# (idem ao painel anterior)

source ~/ros2_ws/install/setup.bash
# (idem)

ros2 launch yolo_pad_pose tf_camera_static.launch.py
# Inicia dois static_transform_publishers com os offsets físicos reais:
#   uav1/fcu → uav1/rgbd_down  (x=0.153, y=0.0, z=-0.129)
#   uav1/fcu → uav1/rgbd_front (x=0.181, y=0.0, z=-0.089)
# Esses valores representam a posição de cada câmera RealSense em relação
# ao FCU (Flight Control Unit) do drone F450.
```

#### Descrição linha por linha do terceiro painel

```bash
waitForGazebo; sleep 5
# (idem)

source ~/ros2_ws/install/setup.bash
# (idem)

ros2 run yolo_pad_pose odom_tf_broadcaster
# Inicia o nó OdomTfBroadcaster.
# Subscreve /uav1/mavros/local_position/odom (nav_msgs/Odometry).
# Publica o transform dinâmico uav1/map → uav1/base_link em /tf.
# Sem este nó, o TF lookup dentro de yolo_pad_pose_ros2 falharia porque
# o MAVROS não publica automaticamente esse transform no /tf.
```

---

### 7.2 Janela `Terminal YOLO` — Nó de detecção

```yaml
- Terminal YOLO:
    layout: titled    # nota: typo no session.yml original; deve ser "tiled"
    panes:
      - |
        waitForGazebo; sleep 5
        source /opt/ros/jazzy/setup.bash
        source ~/ros2_ws/install/setup.bash
        source /home/lmnr31/venvs/yolo/bin/activate
        python3 -m yolo_pad_pose.yolo_pad_pose_ros2 --ros-args \
          -p model_path:=/home/lmnr31/ros2_ws/src/models/landing_pad_yolo/best.pt \
          -p conf:=0.7 \
          -p max_base_range_m:=4.0 \
          -p max_h_range_m:=4.0
```

#### Descrição linha por linha

```bash
waitForGazebo; sleep 5
# Aguarda Gazebo inicializar + 5 s para câmeras começarem a publicar imagens.

source /opt/ros/jazzy/setup.bash
# Carrega o ROS 2 Jazzy do sistema (rosidl, rclpy, sensor_msgs, etc.).
# Obrigatório antes do workspace para evitar conflitos de versão.

source ~/ros2_ws/install/setup.bash
# Carrega o workspace compilado, disponibilizando yolo_pad_pose.

source /home/lmnr31/venvs/yolo/bin/activate
# Ativa o venv Python com Ultralytics instalado.
# A partir daqui, "python3" aponta para /home/lmnr31/venvs/yolo/bin/python3.

python3 -m yolo_pad_pose.yolo_pad_pose_ros2 --ros-args \
# Executa o módulo yolo_pad_pose_ros2 como script.
# "--ros-args" indica que os argumentos seguintes são parâmetros ROS 2.

  -p model_path:=/home/lmnr31/ros2_ws/src/models/landing_pad_yolo/best.pt \
# Define o caminho para os pesos YOLO.
# Para usar o modelo train7 (recomendado), substituir por:
#   /home/lmnr31/runs/detect/train7/weights/best.pt

  -p conf:=0.7 \
# Limiar de confiança mínima. Detecções com score < 0.7 são descartadas.
# Reduzir para 0.5 se o pad estiver sendo perdido na simulação.

  -p max_base_range_m:=4.0 \
# Rejeita detecções da base a mais de 4 m de distância horizontal do drone.
# Evita publicações espúrias quando o pad está fora do alcance efetivo.

  -p max_h_range_m:=4.0
# Idem para o marcador H.
```

> **Nota sobre `model_path` no session.yml:** o caminho aponta para
> `/home/lmnr31/ros2_ws/src/models/landing_pad_yolo/best.pt`, que deve
> ser atualizado para
> `/home/lmnr31/runs/detect/train7/weights/best.pt` para usar o modelo
> treinado com o dataset CVAT completo (ver
> [`11-dataset-e-treinamento.md`](11-dataset-e-treinamento.md)).

---

### 7.3 Janela `pad_waypoint_nn` — Missão nearest-neighbour

```yaml
- pad_waypoint_nn:
```

Esta janela está declarada sem comandos no `session.yml` atual (terminal
vazio). Para ativar a missão nearest-neighbour, adicionar:

```yaml
- pad_waypoint_nn:
    layout: tiled
    panes:
      - |
        waitForGazebo; sleep 8
        source ~/ros2_ws/install/setup.bash
        ros2 run yolo_pad_pose pad_waypoint_nn
```

---

## 8. Diagrama de integração completo

```
session.yml (one_drone)
│
├── tf_fixes (3 painéis)
│   ├── tf_body_fallback  →  uav1/base_link → uav1/fcu  (/tf estático)
│   ├── tf_camera_static  →  uav1/fcu → rgbd_down/front  (/tf estático)
│   └── odom_tf_broadcaster  →  uav1/map → uav1/base_link  (/tf dinâmico)
│
└── Terminal YOLO
    └── yolo_pad_pose_ros2
        ├── subscreve: /uav1/rgbd_front/* + /uav1/rgbd_down/*
        ├── YOLO inference (best.pt, conf=0.7)
        ├── TF lookup: camera_frame → uav1/base_link
        └── publica: /landing_pad/base_relative_position
                     /landing_pad/h_relative_position
```

---

## 9. Solução de problemas comuns

### Nó inicia mas não publica detecções

1. Verificar se as câmeras estão publicando:
   ```bash
   ros2 topic hz /uav1/rgbd_down/color/image_raw
   ```
2. Reduzir `conf` para `0.4` e observar logs.
3. Verificar se o modelo está carregado corretamente nos logs do nó:
   ```
   [INFO] Loading YOLO model: /path/to/best.pt
   ```

### Erro de TF lookup

```
TF failed camera_frame -> uav1/base_link
```

Verificar se todos os três processos da janela `tf_fixes` estão rodando:

```bash
ros2 node list | grep -E "odom_tf|static_transform"
```

### `python3: No module named ultralytics`

O venv não foi ativado antes de executar o nó:

```bash
source /home/lmnr31/venvs/yolo/bin/activate
```
