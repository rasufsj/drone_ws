# Inventário de Nós (src) — `drone_control`

> **Objetivo:** listar e descrever os seis nós C++ do pacote `drone_control`,
> indicando papel, principais classes/funções, tópicos publicados/assinados e
> referência para o capítulo de detalhamento por blocos de código.
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.
> Última sincronização: branch `main`, 2026-04-13.

---

## Índice

| # | Arquivo | Nó ROS 2 | Papel | Doc detalhada |
|---|---------|-----------|-------|---------------|
| 1 | [`src/camera_viewer.cpp`](#1-srccamera_viewercpp) | `camera_viewer` | Visualizador multi-câmera OpenCV | [13-nos.md](13-nos.md#1-camera_viewercpp) |
| 2 | [`src/takeoff.cpp`](#2-srctakeoffcpp) | `takeoff` | Decolagem com FSM e retry | [13-nos.md](13-nos.md#2-takeoffcpp) |
| 3 | [`src/pouso.cpp`](#3-srcpousocpp) | `pouso` | Pouso de precisão em duas fases | [13-nos.md](13-nos.md#3-pousocpp) |
| 4 | [`src/drone_yaw_360.cpp`](#4-srcdrone_yaw_360cpp) | `drone_yaw_360` | Giro de 360° via YawOverride | [13-nos.md](13-nos.md#4-drone_yaw_360cpp) |
| 5 | [`src/missao_P_T.cpp`](#5-srcmissao_p_tcpp) | `missao_P_T` | Orquestrador: pouso → espera → takeoff | [13-nos.md](13-nos.md#5-missao_p_tcpp) |
| 6 | [`src/supervisor_T.cpp`](#6-srcsupervisor_tcpp) | `supervisor_T` | Supervisor de trajetória com FSM | [13-nos.md](13-nos.md#6-supervisor_tcpp) |

---

## 1. `src/camera_viewer.cpp`

### Papel / Responsabilidade

Nó de visualização que assina dois tópicos de imagem do drone (`rgbd_front` e
`rgbd_down`) e os exibe em uma janela OpenCV dividida em dois painéis lado a lado.
Atua como ferramenta de depuração e monitoramento visual em tempo real.

### Classe principal

`CameraViewer` — herda de `rclcpp::Node`.

### Principais funções

| Função | Descrição |
|--------|-----------|
| `CameraViewer()` | Construtor: declara parâmetros, cria janela OpenCV, instancia subscriptions |
| `spinRender()` | Loop principal: spin_some + renderização a 30 Hz; sai em ESC ou Ctrl+C |
| `imageCallback()` | Converte `sensor_msgs/Image` → `cv::Mat` RGB via `cv_bridge`, armazena com mutex |
| `createCanvasBGR()` | Monta canvas com dois slots (front/down), converte RGB→BGR, adiciona labels |

### Tópicos

| Tópico | Tipo | Direção |
|--------|------|---------|
| `/uav1/rgbd_front/color/image_raw` | `sensor_msgs/Image` | sub |
| `/uav1/rgbd_down/color/image_raw` | `sensor_msgs/Image` | sub |

### Parâmetros

| Parâmetro | Padrão | Descrição |
|-----------|--------|-----------|
| `window_width` | `1600` | Largura da janela em pixels |
| `window_height` | `900` | Altura da janela em pixels |

---

## 2. `src/takeoff.cpp`

### Papel / Responsabilidade

Nó de decolagem que publica um waypoint de altitude-alvo em `/waypoints` e monitora
a odometria até confirmar que o drone atingiu a altitude desejada. Implementa uma
FSM com retry para tolerância a falhas transitórias de conexão.

### Classe principal / enum

`TakeoffNode` + enum `TakeoffFSM { WAIT_FCU, PUBLISH_TAKEOFF, MONITOR }`.

### Principais funções

| Função | Descrição |
|--------|-----------|
| `TakeoffNode()` | Construtor: declara parâmetros, configura pub/sub/timer |
| `stateCallback()` | Atualiza `fcu_connected_` com status MAVROS |
| `odomCallback()` | Atualiza `current_z_`, `odom_x_`, `odom_y_` |
| `timerCallback()` | Dispatcher da FSM a cada `rate_hz` Hz |
| `publishTakeoffWaypoint()` | Publica `PoseArray` com waypoint de takeoff |
| `monitorAltitude()` | Verifica se `current_z_ >= altitude_thresh_`; retry ou shutdown |

### Tópicos

| Tópico | Tipo | Direção |
|--------|------|---------|
| `/waypoints` | `geometry_msgs/PoseArray` | pub |
| `/uav1/mavros/state` | `mavros_msgs/State` | sub |
| `/uav1/mavros/local_position/odom` | `nav_msgs/Odometry` | sub |

### Parâmetros principais

| Parâmetro | Padrão | Descrição |
|-----------|--------|-----------|
| `uav_name` | `uav1` | Namespace do UAV |
| `takeoff_altitude` | `1.75` | Altitude alvo (m) |
| `altitude_threshold` | `-1.0` | Limiar de confirmação (< 0 = `takeoff_altitude - 0.3`) |
| `use_current_xy` | `true` | Usar XY da odometria atual no waypoint |
| `max_attempts` | `3` | Tentativas máximas de publicação |

---

## 3. `src/pouso.cpp`

### Papel / Responsabilidade

Nó de pouso de precisão com FSM de duas fases (CENTER → DESCEND) que evita
oscilação de posição XY durante a descida. Suporta detecção de marcador H via
YOLO para pouso autônomo de alta precisão.

### Classe principal / enums

`PousoNode` + `enum class PousoFSM { WAIT_FCU, WAIT_ODOM, COLLECT_H, CENTER, DESCEND }`.

### Principais funções

| Função | Descrição |
|--------|-----------|
| `PousoNode()` | Construtor: declara parâmetros, pub/sub/timer |
| `stateCallback()` | Atualiza status da conexão FCU |
| `odomCallback()` | Atualiza posição e yaw atual |
| `hCallback()` | Recebe detecções YOLO do marcador H |
| `timerCallback()` | Dispatcher da FSM |
| `enterCenter()` / `runCenter()` | Fase CENTER: centraliza o drone sobre o alvo XY |
| `enterDescend()` / `runDescend()` | Fase DESCEND: desce para `landing_z` |
| `publishWaypoint()` | Publica `PoseArray` com um único waypoint |

### Tópicos

| Tópico | Tipo | Direção |
|--------|------|---------|
| `/waypoints` | `geometry_msgs/PoseArray` | pub |
| `/uav1/mavros/state` | `mavros_msgs/State` | sub |
| `/uav1/mavros/local_position/odom` | `nav_msgs/Odometry` | sub |
| `/landing_pad/h_relative_position` | `geometry_msgs/PointStamped` | sub (opcional) |

### Parâmetros principais

| Parâmetro | Padrão | Descrição |
|-----------|--------|-----------|
| `use_current_xy` | `true` | Usar XY da odometria como alvo de pouso |
| `landing_z` | `0.05` | Altitude final de pouso (m) |
| `approach_z` | `-1.0` | Altitude de centralização em CENTER (m); `-1` = atual |
| `xy_hold_tol` | `0.10` | Tolerância XY para considerar centrado (m) |
| `xy_abort_tol` | `0.5` | Limiar de deriva para abortar descida e voltar a CENTER (m) |
| `use_yolo_h` | `false` | Ativar detecção de marcador H via YOLO |

---

## 4. `src/drone_yaw_360.cpp`

### Papel / Responsabilidade

Nó que comanda o drone a realizar um giro angular acumulado (padrão: 360°) usando
o mecanismo `YawOverride` do `my_drone_controller`. Monitora a odometria, acumula
a rotação realizada e desativa o override ao atingir o ângulo alvo.

### Classe principal

`DroneYaw360OverrideAngle` — herda de `rclcpp::Node`.

### Principais funções

| Função | Descrição |
|--------|-----------|
| `DroneYaw360OverrideAngle()` | Construtor: parâmetros, pub/sub, timer a 20 Hz |
| `odom_cb()` | Extrai yaw atual via Eigen quaternion → Euler ZYX |
| `timer_cb()` | Acumula rotação, verifica meta, publica enable/disable |
| `publish_enable()` | Publica `YawOverride{enable=true, yaw_rate=…}` |
| `publish_disable()` | Publica `YawOverride{enable=false}` e faz `rclcpp::shutdown()` |
| `normalize_angle()` | Normaliza ângulo para `[-π, π]` |

### Tópicos

| Tópico | Tipo | Direção |
|--------|------|---------|
| `/uav1/yaw_override/cmd` | `drone_control/YawOverride` | pub |
| `/uav1/mavros/local_position/odom` | `nav_msgs/Odometry` | sub |

### Parâmetros

| Parâmetro | Padrão | Descrição |
|-----------|--------|-----------|
| `uav_ns` | `/uav1` | Namespace do UAV |
| `yaw_rate` | `1.0` | Taxa de giro (rad/s) |
| `yaw_tolerance` | `0.05` | Tolerância angular para considerar giro concluído (rad) |
| `yaw_target_delta` | `2π` | Ângulo total a girar (rad); negativo = CW |

---

## 5. `src/missao_P_T.cpp`

### Papel / Responsabilidade

Nó orquestrador que executa a missão padrão **P→T** (Pouso → espera → Takeoff).
Usa `fork()`/`execlp()` para lançar cada fase como subprocesso independente,
garantindo isolamento de contexto ROS 2 e retorno ao estado inicial ao terminar.

### Classe principal

`MissaoPTNode` — herda de `rclcpp::Node`.

### Principais funções

| Função | Descrição |
|--------|-----------|
| `MissaoPTNode()` | Construtor: agenda início da missão 1 s após construção |
| `startMissionAsync()` | Cancela timer e lança `runMission()` em thread background |
| `run_subprocess()` | Estático: faz `fork()`/`execlp()` e aguarda exit code do filho |
| `runMission()` | Sequência: pouso → 10 s → takeoff; abort em falha |

### Sequência de execução

```
missao_P_T
  ├─ [FASE 1] ros2 run drone_control pouso
  ├─ [FASE 2] sleep 10 s
  └─ [FASE 3] ros2 run drone_control takeoff
```

### Tópicos

Nenhum direto — o nó delega toda comunicação ROS 2 aos subprocessos `pouso` e `takeoff`.

---

## 6. `src/supervisor_T.cpp`

### Papel / Responsabilidade

Supervisor que monitora eventos de trajetória (`/trajectory_progress`,
`/trajectory_finished`) e reage automaticamente: após cada trajetória
concluída, aguarda um delay configurável e lança `missao_P_T` (ou `pouso`
local se o drone estiver na origem). A FSM suporta ciclos infinitos.

### Classe principal / enum

`SupervisorTNode` + `enum class SupervisorState { INIT, TAKING_OFF, RUN_YAW, WAIT_TRAJ, WAIT_BEFORE_MISSION, RUN_MISSION }`.

### Principais funções

| Função | Descrição |
|--------|-----------|
| `SupervisorTNode()` | Construtor: parâmetros, sub/timer a 500 ms |
| `timerCallback()` | Dispatcher principal da FSM |
| `check_trajectory()` | Verifica sinais de progresso/conclusão no estado WAIT_TRAJ |
| `poll_child()` | WNOHANG: recolhe subprocessos terminados via `waitpid` |
| `launch_child()` | Fork/execlp com argumentos extras (ex.: `use_current_xy:=true`) |
| `odomCallback()` | Atualiza posição XY atual para detecção da zona base |
| `update_base_zone()` | Verifica se o drone está na zona base e acumula hold timer |

### Tópicos

| Tópico | Tipo | Direção |
|--------|------|---------|
| `/trajectory_progress` | `std_msgs/Float32` | sub |
| `/trajectory_finished` | `std_msgs/Bool` | sub |
| `/uav1/mavros/local_position/odom` | `nav_msgs/Odometry` | sub |

### Parâmetros principais

| Parâmetro | Padrão | Descrição |
|-----------|--------|-----------|
| `wait_after_traj_done_s` | `5.0` | Delay antes de lançar missão (s) |
| `use_origin_as_base` | `true` | Pousar localmente se estiver na origem |
| `base_tol_m` | `0.20` | Raio da zona base (m) |
| `base_hold_s` | `2.0` | Tempo de permanência para confirmar zona base (s) |
| `min_relaunch_dist_m` | `0.5` | Distância mínima para permitir novo lançamento (m) |
