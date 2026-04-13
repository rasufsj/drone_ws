# 10 — Ambiente e Instalação

Este documento descreve como configurar o ambiente Python dedicado ao YOLO,
instalar as dependências necessárias e executar o pacote `yolo_pad_pose`
tanto de forma isolada quanto integrado ao workspace ROS 2.

---

## 1. Pré-requisitos do sistema

| Componente | Versão mínima | Observação |
|------------|---------------|------------|
| Ubuntu | 22.04 LTS | Ubuntu 24.04 também suportado |
| ROS 2 | Jazzy Jalisco | Testado em Jazzy |
| Python | 3.10+ | Incluído no Ubuntu 22.04/24.04 |
| CUDA (opcional) | 11.8+ | Para inferência em GPU; CPU funciona |

---

## 2. Criação do venv YOLO

O pacote utiliza um **ambiente virtual Python dedicado** para isolar o
Ultralytics/YOLO das dependências do sistema ROS 2:

```bash
# Criar o venv no diretório padrão do projeto
python3 -m venv /home/lmnr31/venvs/yolo

# Ativar o venv
source /home/lmnr31/venvs/yolo/bin/activate

# Atualizar o pip dentro do venv
pip install --upgrade pip
```

> **Por que um venv separado?** O ROS 2 Jazzy usa o Python do sistema
> (`/usr/bin/python3`). Instalar o Ultralytics no Python do sistema pode
> conflitar com pacotes ROS 2. O venv garante isolamento total.

---

## 3. Instalação do Ultralytics

Com o venv ativado:

```bash
# Instalar Ultralytics (inclui PyTorch CPU por padrão)
pip install ultralytics

# Para GPU NVIDIA (substitua cu118 pela versão CUDA instalada):
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
pip install ultralytics
```

Verificar a instalação:

```bash
python3 -c "from ultralytics import YOLO; print('OK')"
# Deve imprimir: OK
```

---

## 4. Dependências ROS 2

As dependências ROS 2 são instaladas via `apt` e `rosdep`:

```bash
# Dependências de sistema
sudo apt update
sudo apt install \
  ros-jazzy-cv-bridge \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-message-filters \
  python3-colcon-common-extensions

# Resolver dependências do pacote via rosdep
cd ~/ros2_ws
rosdep install --from-paths src/yolo_pad_pose --ignore-src -r -y
```

---

## 5. Build do pacote

```bash
# Compilar apenas o yolo_pad_pose
cd ~/ros2_ws
colcon build --packages-select yolo_pad_pose

# Carregar o workspace compilado
source ~/ros2_ws/install/setup.bash
```

---

## 6. Execução do nó de detecção

### 6.1 Sequência completa de sources

A ordem abaixo é obrigatória: o ROS deve ser carregado **antes** do venv,
pois o nó usa tanto `rclpy` (ROS) quanto `ultralytics` (venv).

```bash
# 1. Carregar ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# 2. Carregar o workspace compilado
source ~/ros2_ws/install/setup.bash

# 3. Ativar o venv com Ultralytics
source /home/lmnr31/venvs/yolo/bin/activate
```

### 6.2 Executar o nó principal

```bash
python3 -m yolo_pad_pose.yolo_pad_pose_ros2 --ros-args \
  -p model_path:=/home/lmnr31/runs/detect/train7/weights/best.pt \
  -p conf:=0.7
```

### 6.3 Parâmetros disponíveis

| Parâmetro | Padrão | Tipo | Descrição |
|-----------|--------|------|-----------|
| `model_path` | `best.pt` | string | Caminho para os pesos `.pt` do modelo YOLO |
| `conf` | `0.25` | float | Confiança mínima de detecção (0–1) |
| `base_class_id` | `0` | int | ID da classe YOLO para a base do pad |
| `h_class_id` | `1` | int | ID da classe YOLO para o marcador H |
| `target_frame` | `uav1/base_link` | string | Frame TF de destino para a transformação |
| `fixed_z` | `1.5` | float | Componente Z fixo no `PointStamped` publicado |
| `max_base_range_m` | `6.0` | float | Alcance máximo para publicar detecção da base (m) |
| `max_h_range_m` | `6.0` | float | Alcance máximo para publicar detecção do H (m) |
| `front_rgb` | `/uav1/rgbd_front/color/image_raw` | string | Tópico RGB da câmera frontal |
| `front_depth` | `/uav1/rgbd_front/depth/image_raw` | string | Tópico depth da câmera frontal |
| `front_info` | `/uav1/rgbd_front/color/camera_info` | string | Tópico CameraInfo da câmera frontal |
| `down_rgb` | `/uav1/rgbd_down/color/image_raw` | string | Tópico RGB da câmera de baixo |
| `down_depth` | `/uav1/rgbd_down/depth/image_raw` | string | Tópico depth da câmera de baixo |
| `down_info` | `/uav1/rgbd_down/color/camera_info` | string | Tópico CameraInfo da câmera de baixo |

### 6.4 Ajuste do limiar de confiança (`conf`)

| Valor de `conf` | Comportamento |
|-----------------|---------------|
| `0.7` (padrão usado) | Poucos falsos positivos; pode perder detecções em ângulo |
| `0.5` | Equilíbrio; recomendado para início de missão |
| `0.4` | Mais sensível; útil quando o pad está longe ou parcialmente visível |

```bash
# Exemplo com conf reduzido para maior sensibilidade
python3 -m yolo_pad_pose.yolo_pad_pose_ros2 --ros-args \
  -p model_path:=/home/lmnr31/runs/detect/train7/weights/best.pt \
  -p conf:=0.5
```

---

## 7. Execução do nó de waypoints

```bash
# Após sources (seção 6.1)
ros2 run yolo_pad_pose pad_waypoint_nn
```

---

## 8. Execução do broadcaster de TF

```bash
# Lançar TF estáticos das câmeras
ros2 launch yolo_pad_pose tf_camera_static.launch.py

# Lançar TF de fallback do corpo
ros2 launch yolo_pad_pose tf_body_fallback.launch.py

# Lançar broadcaster de TF da odometria
ros2 run yolo_pad_pose odom_tf_broadcaster
```

---

## 9. Verificação rápida após inicialização

```bash
# Confirmar que os tópicos estão sendo publicados
ros2 topic list | grep landing_pad

# Ver detecções em tempo real
ros2 topic echo /landing_pad/base_relative_position

# Verificar árvore TF
ros2 run tf2_ros tf2_echo uav1/map uav1/base_link
```
