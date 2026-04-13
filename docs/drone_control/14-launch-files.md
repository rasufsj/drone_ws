# Launch Files — `drone_control`

> **Objetivo:** explicar **linha por linha** cada arquivo `launch/*.launch.py`
> do pacote `drone_control` — cada import, cada argumento, cada parâmetro,
> cada ação de evento e a ordem de execução resultante.
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.
> Última sincronização: branch `main`, 2026-04-13.

---

## 1. `camera_viewer.launch.py`

### Arquivo completo

```python
from launch import LaunchDescription          # (1)
from launch_ros.actions import Node           # (2)


def generate_launch_description():           # (3)
    camera_viewer = Node(                     # (4)
        package='drone_control',              # (5)
        executable='camera_viewer',           # (6)
        name='camera_viewer',                 # (7)
        output='screen',                      # (8)
        parameters=[{                         # (9)
            'window_width': 1600,             # (10)
            'window_height': 900,             # (11)
        }],                                   # (12)
    )

    return LaunchDescription([camera_viewer]) # (13)
```

### Linha por linha

| # | Linha | O que faz |
|---|-------|-----------|
| 1 | `from launch import LaunchDescription` | Importa `LaunchDescription`: objeto-raiz do sistema de launch do ROS 2 que contém todas as ações a executar |
| 2 | `from launch_ros.actions import Node` | Importa a ação `Node`: representa `ros2 run <pkg> <exe>` com parâmetros opcionais |
| 3 | `def generate_launch_description():` | Função obrigatória reconhecida por `ros2 launch`; deve retornar um `LaunchDescription` |
| 4 | `camera_viewer = Node(` | Declara o nó; ainda não inicia — só configura |
| 5 | `package='drone_control',` | Pacote onde o executável está instalado |
| 6 | `executable='camera_viewer',` | Binário localizado em `lib/drone_control/camera_viewer` |
| 7 | `name='camera_viewer',` | Nome ROS 2 do nó (usado em `ros2 node list` e nos logs) |
| 8 | `output='screen',` | Redireciona stdout/stderr do nó para o terminal; permite ver logs em tempo real |
| 9 | `parameters=[{` | Lista de dicionários de parâmetros; equivale a `--ros-args -p k:=v` |
| 10 | `'window_width': 1600,` | Define `window_width = 1600 px`; sobrescreve o default do executável (que também é 1600) |
| 11 | `'window_height': 900,` | Define `window_height = 900 px` |
| 12 | `}],` | Fecha o dicionário e a lista de parâmetros |
| 13 | `return LaunchDescription([camera_viewer])` | Registra o nó no launch; `ros2 launch` irá iniciá-lo ao executar este arquivo |

---

## 2. `drone_yaw_360.launch.py`

### Arquivo completo

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    drone_yaw_360 = Node(
        package='drone_control',
        executable='drone_yaw_360',
        name='drone_yaw_360',
        output='screen',
        parameters=[{
            'uav_ns': '/uav1',
            'z_hold': -1.0,
            'yaw_rate': 0.8,
            'angle': 6.283185307179586,
            'hz': 20.0,
            'ccw': True,
            'controller_node': 'drone_controller_completo',
            'auto_disable_controller': True,
        }],
    )

    return LaunchDescription([drone_yaw_360])
```

### Linha por linha

| Parâmetro | Tipo | Valor | Significado |
|-----------|------|-------|-------------|
| `uav_ns` | `string` | `'/uav1'` | Namespace do UAV; constrói os tópicos `/uav1/mavros/...` e `/uav1/yaw_override/cmd` |
| `z_hold` | `float` | `-1.0` | Altitude de hover durante o giro. `-1.0` = manter altitude atual (sem subir/descer) |
| `yaw_rate` | `float` | `0.8` | Velocidade de giro em rad/s. `0.8 rad/s ≈ 45.8°/s` → 360° em ≈7.85 s |
| `angle` | `float` | `6.2831...` | Ângulo total a girar = `2π rad` = 360° |
| `hz` | `float` | `20.0` | Frequência do timer interno (Hz). Compatível com `timer_cb` a 20 Hz no código |
| `ccw` | `bool` | `True` | `True` = anti-horário (CCW); `False` = horário (CW) |
| `controller_node` | `string` | `'drone_controller_completo'` | Nome do nó controlador a pausar durante o giro (sem barra) |
| `auto_disable_controller` | `bool` | `True` | Se `True`, pausa o controlador antes do giro e o retoma ao terminar; evita conflito de comandos de yaw |

> **Atenção:** os parâmetros `z_hold`, `hz`, `ccw`, `angle` e `auto_disable_controller`
> são passados pelo launch file mas precisam ser declarados no `drone_yaw_360.cpp`
> para ter efeito. O código atual declara `yaw_rate`, `yaw_tolerance`,
> `yaw_target_delta` e `uav_ns`. Ajuste o launch ou o código conforme necessário.

---

## 3. `mission_three_nodes.launch.py`

> ⚠️ **Executáveis ausentes:** `drone_soft_land`, `drone_activator` e `drone_go_forward`
> são referenciados por este launch file mas **não possuem arquivo-fonte em
> `drone_control/src/`**. Este launch file é mantido para referência histórica; para
> que funcione, esses executáveis devem ser fornecidos por outro pacote do workspace.

### Arquivo completo

```python
from launch import LaunchDescription                              # (1)
from launch_ros.actions import Node                              # (2)
from launch.actions import RegisterEventHandler, TimerAction     # (3)
from launch.event_handlers import OnProcessExit                  # (4)


def generate_launch_description():

    soft_land = Node(                                            # (5)
        package='drone_control',
        executable='drone_soft_land',
        name='drone_soft_land',
        output='screen'
    )

    activator = Node(                                            # (6)
        package='drone_control',
        executable='drone_activator',
        name='drone_activator',
        output='screen'
    )

    forward = Node(                                              # (7)
        package='drone_control',
        executable='drone_go_forward',
        name='drone_go_forward',
        output='screen'
    )

    camera_viewer = Node(                                        # (8)
        package='drone_control',
        executable='camera_viewer',
        name='camera_viewer',
        output='screen'
    )

    delay_start = RegisterEventHandler(                          # (9)
        OnProcessExit(                                           # (10)
            target_action=soft_land,                            # (11)
            on_exit=[                                            # (12)
                TimerAction(                                     # (13)
                    period=10.0,                                 # (14)
                    actions=[                                    # (15)
                        activator,                               # (16)
                        forward                                  # (17)
                    ]
                )
            ],
        )
    )

    return LaunchDescription([                                   # (18)
        camera_viewer,                                           # (19)
        soft_land,                                               # (20)
        delay_start                                              # (21)
    ])
```

### Linha por linha

| # | O que faz |
|---|-----------|
| 1 | `from launch import LaunchDescription` — container principal das ações |
| 2 | `from launch_ros.actions import Node` — ação de nó ROS 2 |
| 3 | `from launch.actions import RegisterEventHandler, TimerAction` — `RegisterEventHandler`: registra um handler de evento; `TimerAction`: introduz delay cronometrado antes das ações |
| 4 | `from launch.event_handlers import OnProcessExit` — handler disparado quando um processo termina |
| 5 | `soft_land = Node(executable='drone_soft_land', ...)` — nó de pouso suave; deve estar compilado no workspace |
| 6 | `activator = Node(executable='drone_activator', ...)` — nó de ativação; sobem após `soft_land` terminar + 10 s |
| 7 | `forward = Node(executable='drone_go_forward', ...)` — nó de avanço; sobe em paralelo com `activator` |
| 8 | `camera_viewer = Node(executable='camera_viewer', ...)` — sobe imediatamente; monitoramento contínuo |
| 9 | `delay_start = RegisterEventHandler(...)` — registra o handler de evento; sem isso o evento é ignorado |
| 10 | `OnProcessExit(` — construtor do handler: dispara quando `target_action` terminar |
| 11 | `target_action=soft_land,` — monitora o processo `drone_soft_land`; quando ele encerrar (qualquer exit code), dispara o handler |
| 12 | `on_exit=[...]` — lista de ações a executar quando o evento ocorrer |
| 13 | `TimerAction(` — introduz um delay; as actions internas só disparam após `period` segundos |
| 14 | `period=10.0,` — espera 10 segundos após `soft_land` terminar |
| 15 | `actions=[` — lista de ações a executar após o timer |
| 16 | `activator,` — lança `drone_activator` após os 10 s |
| 17 | `forward` — lança `drone_go_forward` em paralelo com `activator` |
| 18 | `return LaunchDescription([` — registra ações que sobem imediatamente |
| 19 | `camera_viewer,` — sobe no t=0 s (monitoramento paralelo desde o início) |
| 20 | `soft_land,` — sobe no t=0 s (primeiro nó da sequência) |
| 21 | `delay_start` — registra o handler; não lança nenhum nó agora; aguarda o evento `soft_land exit` |

### Linha do tempo

```
t=0   s    camera_viewer → sobe (monitoramento independente)
t=0   s    drone_soft_land → sobe
t=X   s    drone_soft_land TERMINA (evento OnProcessExit dispara)
t=X+10 s   TimerAction expira:
              drone_activator → sobe (paralelo)
              drone_go_forward → sobe (paralelo)
```

---

## 4. `supervisor_T.launch.py`

### Arquivo completo

```python
from launch import LaunchDescription     # (1)
from launch_ros.actions import Node      # (2)


def generate_launch_description():      # (3)
    supervisor_T = Node(                 # (4)
        package='drone_control',         # (5)
        executable='supervisor_T',       # (6)
        name='supervisor_T',             # (7)
        output='screen',                 # (8)
    )
    return LaunchDescription([supervisor_T])  # (9)
```

### Linha por linha

| # | Linha | O que faz |
|---|-------|-----------|
| 1 | `from launch import LaunchDescription` | Container do launch |
| 2 | `from launch_ros.actions import Node` | Ação de nó ROS 2 |
| 3 | `def generate_launch_description():` | Ponto de entrada do `ros2 launch` |
| 4 | `supervisor_T = Node(` | Declara o nó supervisor |
| 5 | `package='drone_control',` | Pacote do executável |
| 6 | `executable='supervisor_T',` | Binário em `lib/drone_control/supervisor_T` |
| 7 | `name='supervisor_T',` | Nome do nó no grafo ROS 2 |
| 8 | `output='screen',` | Logs visíveis no terminal; essencial para acompanhar a FSM |
| 9 | `return LaunchDescription([supervisor_T])` | Launch com um único nó; sem parâmetros — usa todos os defaults do executável |

### Parâmetros customizados via ros2 run

Como o launch file não define parâmetros, use `ros2 run` para personalizar:

```bash
ros2 run drone_control supervisor_T --ros-args \
  -p wait_after_traj_done_s:=3.0 \
  -p base_tol_m:=0.15 \
  -p base_hold_s:=2.0 \
  -p min_relaunch_dist_m:=0.3 \
  -p pouso_xy_hold_tol:=0.08 \
  -p pouso_approach_z:=1.2
```

Ou edite o launch file adicionando `parameters=[{...}]` ao `Node(...)`.

---



### Linha por linha — câmera down

| # | Linha | O que faz |
|---|-------|-----------|
| 1 | `Node(package="tf2_ros", executable="static_transform_publisher",` | Nó padrão do ROS 2 para TF estática |
| 2 | `name="tf_rgbd_down",` | Nome único; diferencia da TF da câmera front no grafo |
| 3 | `"0.153", "0.0", "-0.129",` | Posição da câmera down no frame `uav1/fcu`: x=15.3 cm (frente), y=0 cm, z=−12.9 cm (abaixo) |
| 4 | `"0.0", "0.0", "0.0",` | Sem rotação: eixo óptico da câmera down alinhado com −Z do FCU (aponta para o solo) |
| 5 | `"uav1/fcu", "uav1/rgbd_down",` | Parent=`uav1/fcu`, Child=`uav1/rgbd_down` — câmera down é filho do FCU |

### Linha por linha — câmera front

| # | Linha | O que faz |
|---|-------|-----------|
| 6 | `Node(package="tf2_ros", executable="static_transform_publisher",` | Segundo nó de TF estática |
| 7 | `name="tf_rgbd_front",` | Nome único para a câmera frontal |
| 8 | `"0.181", "0.0", "-0.089",` | Posição da câmera front: x=18.1 cm (frente), y=0, z=−8.9 cm (abaixo). Mais à frente e menos abaixo que a câmera down |
| 9 | `"0.0", "0.0", "0.0",` | Sem rotação: câmera frontal alinhada com os eixos do FCU |
| 10 | `"uav1/fcu", "uav1/rgbd_front",` | Parent=`uav1/fcu`, Child=`uav1/rgbd_front` |

### Árvore TF resultante

```
uav1/base_link             ← publicado por tf_body_fallback.launch.py
  └─ uav1/fcu              ← referencial principal do veículo (FCU = Flight Control Unit)
       ├─ uav1/rgbd_down   ← x=+15.3 cm, z=−12.9 cm (câmera nadir)
       └─ uav1/rgbd_front  ← x=+18.1 cm, z=−8.9 cm  (câmera frontal)
```

### Como verificar

```bash
# Sobe as TFs
ros2 launch yolo_pad_pose tf_camera_static.launch.py

# Verifica câmera down (deve responder imediatamente)
ros2 run tf2_ros tf2_echo uav1/fcu uav1/rgbd_down

# Verifica câmera front
ros2 run tf2_ros tf2_echo uav1/fcu uav1/rgbd_front

# Visualiza toda a árvore TF em PDF
ros2 run tf2_tools view_frames
```

---

## 7. Ordem de inicialização recomendada

```bash
# 1. MAVROS ou simulador (Gazebo + MRS UAV System)
# 2. my_drone_controller
ros2 run my_drone_controller drone_node

# 3. TF estrutural (se não publicado pelo MAVROS)
ros2 launch yolo_pad_pose tf_body_fallback.launch.py

# 4. TF das câmeras
ros2 launch yolo_pad_pose tf_camera_static.launch.py

# 5. Supervisor de missão (aguarda /trajectory_finished)
ros2 launch drone_control supervisor_T.launch.py

# 6. Visualização (opcional)
ros2 launch drone_control camera_viewer.launch.py

# 7. Enviar trajetória para o my_drone_controller
ros2 topic pub /waypoints geometry_msgs/msg/PoseArray \
  "{header: {frame_id: 'map'}, poses: [{position: {x: 3.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}]}" \
  --once
```
