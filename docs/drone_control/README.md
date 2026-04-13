# Documentação: `drone_control`

Bem-vindo à documentação do pacote `drone_control`. Esta documentação está organizada
em inventários separados por tipo e em capítulos explicativos, seguindo o mesmo padrão
acadêmico/didático adotado em `docs/my_drone_controller/`.

## Índice

### Inventários

| Arquivo | Conteúdo |
|---------|----------|
| [00 — Inventário Meta/Build](00-inventario-meta-build.md) | `README.md`, `package.xml`, `CMakeLists.txt`, `LICENSE` |
| [01 — Inventário Mensagens](01-inventario-msg.md) | `msg/YawOverride.msg`, `msg/Waypoint4D.msg`, `msg/Waypoint4DArray.msg` |
| [02 — Inventário Nós (src)](02-inventario-src.md) | `src/camera_viewer.cpp`, `src/takeoff.cpp`, `src/pouso.cpp`, `src/drone_yaw_360.cpp`, `src/missao_P_T.cpp`, `src/supervisor_T.cpp` |
| [03 — Inventário Launch](03-inventario-launch.md) | Todos os arquivos `launch/*.launch.py` |

### Capítulos Explicativos

| Arquivo | Conteúdo |
|---------|----------|
| [10 — Visão Geral](10-visao-geral.md) | Objetivo do pacote, arquitetura, integração com `my_drone_controller`, mapa de tópicos |
| [11 — Build e Interfaces](11-build-e-interfaces.md) | `package.xml`, `CMakeLists.txt` por blocos: dependências, targets, geração de mensagens |
| [12 — Mensagens Customizadas](12-mensagens.md) | `YawOverride`, `Waypoint4D`, `Waypoint4DArray` por blocos + exemplos de uso |
| [13 — Nós (C++)](13-nos.md) | Cada `src/*.cpp` por blocos: pub/sub, FSM, parâmetros, fluxo |
| [14 — Launch Files](14-launch-files.md) | Cada `launch/*.launch.py` por blocos: nós que sobem, argumentos, remaps, TF |

## Estrutura do pacote

```
drone_control/
├── CMakeLists.txt                        ← geração de msgs + targets C++   → [11]
├── package.xml                           ← dependências e exportações       → [11]
├── LICENSE                               ← Apache-2.0                       → [00]
├── README.md                             ← guia rápido do pacote            → [00]
├── msg/
│   ├── YawOverride.msg                   ← override de yaw rate             → [01][12]
│   ├── Waypoint4D.msg                    ← waypoint posição + yaw           → [01][12]
│   └── Waypoint4DArray.msg               ← array de Waypoint4D com header   → [01][12]
├── src/
│   ├── camera_viewer.cpp                 ← visualizador multi-câmera OpenCV → [02][13]
│   ├── takeoff.cpp                       ← decolagem via FSM                → [02][13]
│   ├── pouso.cpp                         ← pouso em duas fases (CENTER/DESCEND) → [02][13]
│   ├── drone_yaw_360.cpp                 ← giro de 360° via YawOverride     → [02][13]
│   ├── missao_P_T.cpp                    ← orquestrador: pouso → espera → decolagem → [02][13]
│   └── supervisor_T.cpp                  ← supervisor de trajetória         → [02][13]
└── launch/
    ├── camera_viewer.launch.py           ← sobe camera_viewer               → [03][14]
    ├── drone_yaw_360.launch.py           ← sobe drone_yaw_360               → [03][14]
    ├── mission_three_nodes.launch.py     ← missão de três nós encadeados    → [03][14]
    ├── supervisor_T.launch.py            ← sobe supervisor_T                → [03][14]
    ├── tf_body_fallback.launch.py        ← TF estático base_link → fcu     → [03][14]
    └── tf_camera_static.launch.py        ← TF estático fcu → câmeras        → [03][14]
```

## Tópicos principais

| Tópico | Tipo | Direção | Usado por |
|--------|------|---------|-----------|
| `/waypoints` | `geometry_msgs/PoseArray` | pub | `takeoff`, `pouso` |
| `/trajectory_progress` | `std_msgs/Float32` | sub | `supervisor_T` |
| `/trajectory_finished` | `std_msgs/Bool` | sub | `supervisor_T` |
| `/uav1/mavros/state` | `mavros_msgs/State` | sub | `takeoff`, `pouso` |
| `/uav1/mavros/local_position/odom` | `nav_msgs/Odometry` | sub | `takeoff`, `pouso`, `drone_yaw_360`, `supervisor_T` |
| `/uav1/yaw_override/cmd` | `drone_control/YawOverride` | pub | `drone_yaw_360` |
| `/uav1/rgbd_front/color/image_raw` | `sensor_msgs/Image` | sub | `camera_viewer` |
| `/uav1/rgbd_down/color/image_raw` | `sensor_msgs/Image` | sub | `camera_viewer` |

## Integração com `my_drone_controller`

O pacote `drone_control` é o **executor de missões** que opera sobre o
`my_drone_controller` (controlador de trajetória PX4/MAVROS). A comunicação
entre os dois pacotes é realizada via tópicos ROS 2:

- **`/waypoints`** (`geometry_msgs/PoseArray`) — `drone_control` publica waypoints
  que o `my_drone_controller` consome para gerar trajetórias.
- **`/trajectory_finished`** / **`/trajectory_progress`** — `my_drone_controller`
  publica o progresso; o `supervisor_T` de `drone_control` escuta para reagir.
- **`/uav1/yaw_override/cmd`** (`drone_control/YawOverride`) — `drone_yaw_360`
  publica comandos de yaw que `my_drone_controller` executa.

## Convenções adotadas nesta documentação

- Linguagem: **português**, estilo acadêmico/didático.
- Nomes de variáveis membros seguem o sufixo `_` (ex.: `fsm_`, `current_z_`).
- Tópicos específicos ao veículo são prefixados com `/uav1/`.
- Explicações por **blocos de código** em todos os arquivos relevantes.
- Última sincronização com o código-fonte: branch `main`, 2026-04-13.
