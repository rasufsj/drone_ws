# Launch Files — `drone_control`

> **Objetivo:** explicar por blocos cada arquivo `launch/*.launch.py` do pacote
> `drone_control` — nós que sobem, argumentos, parâmetros, remaps, transformações
> TF e ordem típica de uso.
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.
> Última sincronização: branch `main`, 2026-04-13.

---

## 1. `camera_viewer.launch.py`

### Papel / Responsabilidade

Sobe o nó `camera_viewer` configurado para a resolução padrão de tela larga.
Launch simples de um único nó; indicado para monitoramento visual durante missões.

### Bloco 1 — Imports

```python
from launch import LaunchDescription
from launch_ros.actions import Node
```

**O que este bloco faz:**

- `LaunchDescription` — objeto que contém a lista de ações a executar.
- `Node` — ação que instancia um executável ROS 2 como nó.

### Bloco 2 — Definição do nó

```python
def generate_launch_description():
    camera_viewer = Node(
        package='drone_control',
        executable='camera_viewer',
        name='camera_viewer',
        output='screen',
        parameters=[{
            'window_width':  1600,
            'window_height': 900,
        }],
    )
    return LaunchDescription([camera_viewer])
```

**O que este bloco faz:**

- `package='drone_control'` / `executable='camera_viewer'` — localiza o binário
  em `install/drone_control/lib/drone_control/camera_viewer`.
- `output='screen'` — redireciona stdout/stderr do nó para o terminal, permitindo
  ver os logs `RCLCPP_INFO_THROTTLE` de recepção de imagens.
- `parameters=[{...}]` — passa parâmetros ROS 2 equivalentes a
  `--ros-args -p window_width:=1600 -p window_height:=900`.

### Como executar

```bash
ros2 launch drone_control camera_viewer.launch.py

# Equivalente via ros2 run
ros2 run drone_control camera_viewer --ros-args \
  -p window_width:=1600 -p window_height:=900
```

---

## 2. `drone_yaw_360.launch.py`

### Papel / Responsabilidade

Sobe o nó `drone_yaw_360` pré-configurado para realizar um giro completo de
360° (2π rad) no sentido CCW a 0.8 rad/s. Inclui parâmetros de pausa
automática do controlador principal durante o giro.

### Bloco 1 — Definição do nó com parâmetros

```python
def generate_launch_description():
    drone_yaw_360 = Node(
        package='drone_control',
        executable='drone_yaw_360',
        name='drone_yaw_360',
        output='screen',
        parameters=[{
            'uav_ns':    '/uav1',
            'z_hold':    -1.0,           # manter altitude atual
            'yaw_rate':  0.8,            # rad/s
            'angle':     6.283185307179586,  # 2π rad = 360°
            'hz':        20.0,           # taxa do timer (Hz)
            'ccw':       True,           # True = CCW, False = CW
            'controller_node':        'drone_controller_completo',
            'auto_disable_controller': True,
        }],
    )
    return LaunchDescription([drone_yaw_360])
```

**O que este bloco faz:**

- `angle = 2π` + `ccw = True` → giro CCW de 360° (uma volta completa).
- `auto_disable_controller = True` → o nó pausa o `drone_controller_completo`
  durante o giro para evitar conflito de comandos de yaw, e o retoma ao terminar.
- `z_hold = -1.0` → mantém a altitude atual durante o giro (sem subir/descer).

> **Nota:** os parâmetros `z_hold`, `hz`, `ccw`, `angle`, `controller_node` e
> `auto_disable_controller` são configurados no launch mas podem não corresponder
> exatamente à interface do executável atual — verifique se o `drone_yaw_360.cpp`
> atual declara esses nomes. O código atual usa `yaw_target_delta` e `yaw_rate`.

### Como executar

```bash
ros2 launch drone_control drone_yaw_360.launch.py

# Giro CW (horário) de 180°
ros2 run drone_control drone_yaw_360 --ros-args \
  -p yaw_rate:=0.8 \
  -p yaw_target_delta:=-3.14159
```

---

## 3. `mission_three_nodes.launch.py`

### Papel / Responsabilidade

Orquestra uma missão de três nós encadeada usando o sistema de eventos do
ROS 2 Launch: `drone_soft_land` → (10 s de espera) → `drone_activator`
+ `drone_go_forward`. O `camera_viewer` sobe em paralelo desde o início.

### Bloco 1 — Imports com eventos e timers

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
```

**O que este bloco faz:**

- `RegisterEventHandler` + `OnProcessExit` — API do ROS 2 Launch para
  reagir ao término de um processo e lançar ações em resposta.
- `TimerAction` — introduce um delay cronometrado entre o evento e as ações.

### Bloco 2 — Definição dos nós

```python
soft_land    = Node(package='drone_control', executable='drone_soft_land',
                    name='drone_soft_land',    output='screen')
activator    = Node(package='drone_control', executable='drone_activator',
                    name='drone_activator',    output='screen')
forward      = Node(package='drone_control', executable='drone_go_forward',
                    name='drone_go_forward',   output='screen')
camera_viewer = Node(package='drone_control', executable='camera_viewer',
                     name='camera_viewer',     output='screen')
```

**O que este bloco faz:**

Declara os quatro nós sem parâmetros customizados — usam os defaults dos
executáveis. Nenhum remap de tópicos está configurado.

### Bloco 3 — Encadeamento via evento

```python
delay_start = RegisterEventHandler(
    OnProcessExit(
        target_action=soft_land,
        on_exit=[
            TimerAction(
                period=10.0,
                actions=[activator, forward]
            )
        ],
    )
)

return LaunchDescription([
    camera_viewer,
    soft_land,
    delay_start
])
```

**O que este bloco faz:**

- `OnProcessExit(target_action=soft_land)` — registra um handler que dispara
  quando o processo `soft_land` termina (qualquer exit code).
- `TimerAction(period=10.0)` — após o evento, espera 10 segundos antes de
  lançar `activator` e `forward` **em paralelo**.
- `camera_viewer` e `soft_land` sobem imediatamente ao iniciar o launch;
  `activator` e `forward` sobem somente após `soft_land` terminar + 10 s.

### Linha do tempo de execução

```
t=0s      camera_viewer → sobe (independente, monitoramento contínuo)
t=0s      drone_soft_land → sobe
t=X s     drone_soft_land termina (exit)
t=X+10s   drone_activator → sobe (paralelo)
t=X+10s   drone_go_forward → sobe (paralelo)
```

> **Nota:** `drone_soft_land`, `drone_activator` e `drone_go_forward` são
> executáveis que devem estar compilados no workspace. Se não estiverem
> disponíveis, o launch falhará ao tentar lançar esses nós.

### Como executar

```bash
ros2 launch drone_control mission_three_nodes.launch.py
```

---

## 4. `supervisor_T.launch.py`

### Papel / Responsabilidade

Sobe o nó `supervisor_T` com configuração mínima (sem parâmetros fixos).
Todos os parâmetros de comportamento devem ser passados via `ros2 run` ou
adicionando `parameters=[{...}]` ao arquivo de launch se personalização for
necessária.

### Bloco 1 — Definição simples

```python
def generate_launch_description():
    supervisor_T = Node(
        package='drone_control',
        executable='supervisor_T',
        name='supervisor_T',
        output='screen',
    )
    return LaunchDescription([supervisor_T])
```

**O que este bloco faz:**

- Nenhum parâmetro configurado — o supervisor usa todos os defaults definidos
  no código (`wait_after_traj_done_s=5.0`, `base_tol_m=0.20`, etc.).
- `output='screen'` — essencial para acompanhar os logs da FSM em tempo real
  (estados, launches de subprocessos, guards de distância).

### Como executar com parâmetros customizados

Para personalizar sem modificar o launch file, use `ros2 run` diretamente:

```bash
# Via launch (defaults)
ros2 launch drone_control supervisor_T.launch.py

# Via ros2 run com parâmetros customizados
ros2 run drone_control supervisor_T --ros-args \
  -p wait_after_traj_done_s:=3.0 \
  -p base_tol_m:=0.15 \
  -p base_hold_s:=2.0 \
  -p pouso_xy_hold_tol:=0.08 \
  -p pouso_approach_z:=1.2
```

Alternativamente, editar o launch e adicionar:

```python
parameters=[{
    'wait_after_traj_done_s': 3.0,
    'base_tol_m': 0.15,
    'base_hold_s': 2.0,
}]
```

---

## 5. `tf_body_fallback.launch.py`

### Papel / Responsabilidade

Publica uma transformação TF estática de fallback estrutural:
`uav1/base_link` → `uav1/fcu` com deslocamento zero e rotação identidade.
Necessário para pacotes que esperam `uav1/fcu` como filho de `uav1/base_link`
na árvore TF do ROS 2.

### Bloco 1 — Transformação estática

```python
def generate_launch_description():
    return LaunchDescription([
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_link_to_fcu",
            arguments=[
                "0", "0", "0",    # x y z (translação zero)
                "0", "0", "0",    # roll pitch yaw (rotação zero)
                "uav1/base_link", # frame pai
                "uav1/fcu",       # frame filho
            ],
            output="screen",
        ),
    ])
```

**O que este bloco faz:**

- `static_transform_publisher` do pacote `tf2_ros` publica a TF continuamente
  com QoS `transient_local` (latched) — novos subscribers recebem a TF
  imediatamente ao se conectar.
- Translação `(0, 0, 0)` e rotação `(0, 0, 0)` → os dois frames são coincidentes.
  `uav1/fcu` é fisicamente o mesmo ponto que `uav1/base_link` nesta configuração.
- `arguments` usa a interface "legacy" do `static_transform_publisher`
  (x y z roll pitch yaw parent child) — formato suportado no ROS 2 Humble.

### Quando usar

Este launch é um **fallback** para quando a configuração principal do MAVROS ou do
simulador não publica a TF `base_link → fcu`. Em simulação com MRS UAV Gazebo
Simulator, essa TF pode já existir — verifique com:

```bash
ros2 run tf2_tools view_frames
```

### Como executar

```bash
ros2 launch drone_control tf_body_fallback.launch.py

# Verificar se a TF está sendo publicada
ros2 run tf2_ros tf2_echo uav1/base_link uav1/fcu
```

---

## 6. `tf_camera_static.launch.py`

### Papel / Responsabilidade

Publica duas transformações TF estáticas com as posições físicas medidas das
câmeras RGBD embarcadas no drone: `uav1/fcu` → `uav1/rgbd_down` e
`uav1/fcu` → `uav1/rgbd_front`. Essencial para algoritmos de visão que
precisam transformar coordenadas de imagem para o referencial do drone ou
do mapa.

### Bloco 1 — TF câmera down

```python
Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_rgbd_down",
    arguments=[
        "0.153", "0.0", "-0.129",  # x y z
        "0.0", "0.0", "0.0",       # roll pitch yaw
        "uav1/fcu",                # frame pai
        "uav1/rgbd_down",          # frame filho
    ],
    output="screen",
),
```

**O que este bloco faz:**

- Câmera **down** está 15.3 cm à frente e 12.9 cm abaixo do FCU.
- `z = -0.129` (negativo) — a câmera está abaixo do FCU no referencial ENU.
- Sem rotação — o eixo óptico da câmera está alinhado com `-Z` do FCU
  (câmera aponta para baixo, que é o esperado para câmera nadir).

### Bloco 2 — TF câmera front

```python
Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_rgbd_front",
    arguments=[
        "0.181", "0.0", "-0.089",  # x y z
        "0.0", "0.0", "0.0",       # roll pitch yaw
        "uav1/fcu",                # frame pai
        "uav1/rgbd_front",         # frame filho
    ],
    output="screen",
),
```

**O que este bloco faz:**

- Câmera **front** está 18.1 cm à frente e 8.9 cm abaixo do FCU.
- Posição mais à frente e menos abaixo que a câmera down — câmera de navegação
  frontal com campo de visão levemente inclinado para baixo.
- Sem rotação — alinhada com os eixos do FCU.

### Árvore TF resultante

```
uav1/base_link           ← TF publicada por tf_body_fallback.launch.py
  └─ uav1/fcu            ← referencial principal do veículo
       ├─ uav1/rgbd_down   (15.3 cm frente, 12.9 cm abaixo)
       └─ uav1/rgbd_front  (18.1 cm frente, 8.9 cm abaixo)
```

### Como executar e verificar

```bash
# Sobe as duas TFs das câmeras
ros2 launch drone_control tf_camera_static.launch.py

# Verificar transformações
ros2 run tf2_ros tf2_echo uav1/fcu uav1/rgbd_down
ros2 run tf2_ros tf2_echo uav1/fcu uav1/rgbd_front

# Visualizar árvore TF completa
ros2 run tf2_tools view_frames
```

---

## 7. Uso combinado das launches

### Configuração completa de visão + monitoramento

```bash
# Terminal 1 — TF estrutural (fallback)
ros2 launch drone_control tf_body_fallback.launch.py

# Terminal 2 — TF das câmeras
ros2 launch drone_control tf_camera_static.launch.py

# Terminal 3 — Visualização das câmeras
ros2 launch drone_control camera_viewer.launch.py
```

### Configuração de missão autônoma completa

```bash
# Terminal 1 — TF completo
ros2 launch drone_control tf_body_fallback.launch.py &
ros2 launch drone_control tf_camera_static.launch.py &

# Terminal 2 — Supervisor de missão (aguarda trajetórias do my_drone_controller)
ros2 launch drone_control supervisor_T.launch.py

# Terminal 3 — Monitoramento visual (opcional)
ros2 launch drone_control camera_viewer.launch.py

# Terminal 4 — Enviar trajetória para my_drone_controller (exemplo)
ros2 topic pub /waypoints geometry_msgs/msg/PoseArray \
  "{header: {frame_id: 'map'}, poses: [{position: {x: 3.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}]}" \
  --once
```

### Ordem recomendada de inicialização

1. MAVROS / FCU (ou simulador)
2. `my_drone_controller` (drone_node)
3. Launches de TF (`tf_body_fallback` + `tf_camera_static`)
4. `supervisor_T` (aguarda estado do sistema)
5. `camera_viewer` (monitoramento)
6. Envio de trajetórias
