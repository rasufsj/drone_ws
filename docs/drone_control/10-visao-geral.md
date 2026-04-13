# Visão Geral — `drone_control`

> **Objetivo:** explicar o papel do pacote `drone_control`, sua arquitetura de
> nós, a integração com o `my_drone_controller` e o fluxo de dados entre os
> componentes.
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.
> Última sincronização: branch `main`, 2026-04-13.

---

## 1. O que é o `drone_control`

O pacote `drone_control` é a **camada de execução de missões** do sistema de
controle de drones baseado em ROS 2 / MAVROS / PX4. Enquanto o
`my_drone_controller` implementa o **controlador de trajetória de baixo nível**
(FSM de 5 estados, controlador PID por codegen MATLAB, integração MAVROS),
o `drone_control` contém os **nós de alto nível** responsáveis por:

1. **Decolagem e pouso** com confirmação sensorial (`takeoff`, `pouso`).
2. **Missões compostas** orquestradas por subprocessos (`missao_P_T`).
3. **Supervisão de trajetórias** com reação automática (`supervisor_T`).
4. **Manobras de yaw** via mecanismo de override (`drone_yaw_360`).
5. **Monitoramento visual** das câmeras embarcadas (`camera_viewer`).

---

## 2. Arquitetura do sistema

```
┌────────────────────────────────────────────────────────────────────┐
│                        drone_control                               │
│                                                                    │
│  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐               │
│  │  takeoff    │   │    pouso    │   │ drone_yaw_  │               │
│  │   (FSM)     │   │  (CENTER/   │   │    360      │               │
│  └──────┬──────┘   │  DESCEND)   │   └──────┬──────┘               │
│         │          └──────┬──────┘          │                      │
│         │    /waypoints   │          /uav1/yaw_override/cmd        │
│         └────────────┬────┘                 │                      │
│                      │             ┌────────┘                      │
│  ┌─────────────┐     │             │                               │
│  │ missao_P_T  │     │             │                               │
│  │(subprocess) ├─fork/execlp→      │                               │
│  └──────▲──────┘  takeoff/pouso    │                               │
│         │                          │                               │
│  ┌──────┴──────┐                   │                               │
│  │ supervisor_T│◄──/trajectory_finished────────────────────────────│
│  │   (FSM)     │◄──/trajectory_progress                            │
│  └─────────────┘                   │                               │
│                                    │                               │
│  ┌─────────────┐                   │                               │
│  │camera_viewer│◄──/uav1/rgbd_*/color/image_raw                    │
│  └─────────────┘                                                   │
└────────────────────────────────────────────────────────────────────┘
             │ /waypoints           │ /uav1/yaw_override/cmd
             ▼                      ▼
┌────────────────────────────────────────────────────────────────────┐
│                      my_drone_controller                           │
│                                                                    │
│  FSM: WAIT → TAKEOFF → HOVER → TRAJECTORY → LANDING                │
│  Publica: /trajectory_finished, /trajectory_progress               │
│  Assina:  /waypoints, /uav1/yaw_override/cmd                       │
└────────────────────────────────────────────────────────────────────┘
             │ MAVROS
             ▼
        PX4 / FCU (hardware ou simulação Gazebo)
```

---

## 3. Hierarquia de controle

O sistema opera em três camadas:

| Camada | Pacote | Responsabilidade |
|--------|--------|-----------------|
| **Missão** | `drone_control` | Sequências de alto nível (takeoff, pouso, missão, supervisão) |
| **Trajetória** | `my_drone_controller` | FSM de estados, controle PID, publicação de setpoints MAVROS |
| **Voo** | PX4 / FCU | Controle de atitude, motores, estabilização |

---

## 4. Integração com `my_drone_controller`

### 4.1 Tópicos de comando

O `drone_control` comanda o `my_drone_controller` publicando em dois tópicos:

#### `/waypoints` — `geometry_msgs/PoseArray`

Tópico principal de envio de waypoints. Cada `Pose` no array representa um ponto
no espaço (x, y, z) que o drone deve visitar em sequência.

```bash
# Enviar um único waypoint de hover a 1.5 m
ros2 topic pub /waypoints geometry_msgs/msg/PoseArray \
  "{header: {frame_id: 'map'}, poses: [{position: {x: 0.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}]}" \
  --once
```

#### `/uav1/yaw_override/cmd` — `drone_control/YawOverride`

Sobrescreve temporariamente o controle de yaw do controlador:

```bash
# Girar a 1 rad/s CCW por até 10 s
ros2 topic pub /uav1/yaw_override/cmd drone_control/msg/YawOverride \
  "{enable: true, yaw_rate: 1.0, timeout: 10.0}" --once
```

### 4.2 Tópicos de status

O `drone_control` **escuta** o estado do `my_drone_controller` via:

| Tópico | Tipo | Publicado por | Assinado por |
|--------|------|--------------|--------------|
| `/trajectory_finished` | `std_msgs/Bool` | `my_drone_controller` | `supervisor_T` |
| `/trajectory_progress` | `std_msgs/Float32` | `my_drone_controller` | `supervisor_T` |

> **Nota:** `/trajectory_finished` é publicado com `data=true` **somente** quando
> a trajetória é concluída. Quando uma nova trajetória começa, `data=false` é
> publicado para resetar os guards do `supervisor_T`.

### 4.3 Mensagens customizadas exportadas

O `drone_control` também exporta tipos de mensagem que o `my_drone_controller`
pode consumir para trajetórias com yaw explícito:

| Tópico | Tipo | Semântica |
|--------|------|-----------|
| `/waypoints_4d` | `drone_control/Waypoint4DArray` | Array de waypoints com posição + yaw |
| `/waypoint_goal_4d` | `drone_control/Waypoint4D` | Waypoint único com posição + yaw |

---

## 5. Fluxo típico de operação

```
1. Iniciar supervisor_T
   └─ supervisor_T aguarda /trajectory_finished

2. Enviar trajetória ao my_drone_controller
   └─ my_drone_controller publica /trajectory_progress (0 → 100)

3. Trajetória concluída
   └─ my_drone_controller publica /trajectory_finished = true
   └─ supervisor_T entra em WAIT_BEFORE_MISSION (delay configurável)

4. Após o delay, supervisor_T lança missao_P_T (ou pouso local)
   └─ missao_P_T executa: pouso → espera 10 s → takeoff

5. Loop reinicia para a próxima trajetória
```

### Exemplo de fluxo mínimo

```bash
# Terminal 1 — Supervisor
ros2 run drone_control supervisor_T

# Terminal 2 — Simular conclusão de trajetória
ros2 topic pub /trajectory_finished std_msgs/msg/Bool "data: true" --once

# Terminal 3 — Monitorar status
ros2 topic echo /trajectory_progress
```

---

## 6. Mensagens e interfaces customizadas

O pacote define três tipos de mensagem (doc detalhada em [12-mensagens.md](12-mensagens.md)):

| Mensagem | Campos | Uso |
|----------|--------|-----|
| `YawOverride` | `enable`, `yaw_rate`, `timeout` | Controle de yaw em `drone_yaw_360` |
| `Waypoint4D` | `pose`, `yaw` | Waypoint com orientação explícita |
| `Waypoint4DArray` | `header`, `waypoints[]` | Array de waypoints para trajetórias ricas |

---

## 7. Como compilar e executar

```bash
# Compilar apenas o pacote drone_control
cd ~/ros2_ws
colcon build --packages-select drone_control
source install/setup.bash

# Executar um nó individualmente
ros2 run drone_control takeoff
ros2 run drone_control pouso
ros2 run drone_control drone_yaw_360
ros2 run drone_control missao_P_T
ros2 run drone_control supervisor_T
ros2 run drone_control camera_viewer

# Executar via launch
ros2 launch drone_control supervisor_T.launch.py
ros2 launch drone_control camera_viewer.launch.py
ros2 launch yolo_pad_pose tf_camera_static.launch.py
```
