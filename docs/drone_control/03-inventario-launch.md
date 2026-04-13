# Inventário de Launch Files — `drone_control`

> **Objetivo:** listar e descrever os seis arquivos de launch do pacote
> `drone_control`, indicando quais nós sobem, argumentos configuráveis,
> remaps e transformações TF publicadas.
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.
> Última sincronização: branch `main`, 2026-04-13.

---

## Índice

| # | Arquivo | Nós que sobem | Doc detalhada |
|---|---------|--------------|---------------|
| 1 | [`camera_viewer.launch.py`](#1-camera_viewerlaunchpy) | `camera_viewer` | [14-launch-files.md](14-launch-files.md#1-camera_viewerlaunchpy) |
| 2 | [`drone_yaw_360.launch.py`](#2-drone_yaw_360launchpy) | `drone_yaw_360` | [14-launch-files.md](14-launch-files.md#2-drone_yaw_360launchpy) |
| 3 | [`mission_three_nodes.launch.py`](#3-mission_three_nodeslaunchpy) | `drone_soft_land`, `drone_activator`, `drone_go_forward`, `camera_viewer` | [14-launch-files.md](14-launch-files.md#3-mission_three_nodeslaunchpy) |
| 4 | [`supervisor_T.launch.py`](#4-supervisor_tlaunchpy) | `supervisor_T` | [14-launch-files.md](14-launch-files.md#4-supervisor_tlaunchpy) |
| 5 | [`tf_body_fallback.launch.py`](#5-tf_body_fallbacklaunchpy) | `static_transform_publisher` (base_link→fcu) | [14-launch-files.md](14-launch-files.md#5-tf_body_fallbacklaunchpy) |
| 6 | [`tf_camera_static.launch.py`](#6-tf_camera_staticlaunchpy) | `static_transform_publisher` ×2 (fcu→câmeras) | [14-launch-files.md](14-launch-files.md#6-tf_camera_staticlaunchpy) |

---

## 1. `camera_viewer.launch.py`

### Papel / Responsabilidade

Sobe o nó `camera_viewer` configurado para a resolução padrão de tela larga (1600×900).

### Nós lançados

| Nó | Pacote | Parâmetros |
|----|--------|-----------|
| `camera_viewer` | `drone_control` | `window_width=1600`, `window_height=900` |

### Como executar

```bash
ros2 launch drone_control camera_viewer.launch.py
```

---

## 2. `drone_yaw_360.launch.py`

### Papel / Responsabilidade

Sobe o nó `drone_yaw_360` pré-configurado para realizar um giro CCW de 360°
a 0.8 rad/s, com pausa automática do `drone_controller_completo` durante o giro.

### Nós lançados

| Nó | Pacote | Parâmetros notáveis |
|----|--------|---------------------|
| `drone_yaw_360` | `drone_control` | `yaw_rate=0.8`, `angle=2π rad`, `ccw=true`, `auto_disable_controller=true` |

### Como executar

```bash
ros2 launch drone_control drone_yaw_360.launch.py
```

---

## 3. `mission_three_nodes.launch.py`

### Papel / Responsabilidade

Lança uma missão encadeada de três nós usando eventos de processo do ROS 2:
`drone_soft_land` → (10 s de espera) → `drone_activator` + `drone_go_forward`.
O `camera_viewer` sobe em paralelo desde o início.

### Nós lançados e ordem

```
t=0       camera_viewer  (independente, sobe imediatamente)
t=0       drone_soft_land
t=exit(soft_land)+10s  drone_activator + drone_go_forward (paralelos)
```

### Mecanismo de encadeamento

```python
RegisterEventHandler(
    OnProcessExit(
        target_action=soft_land,
        on_exit=[TimerAction(period=10.0, actions=[activator, forward])]
    )
)
```

`OnProcessExit` dispara quando `soft_land` termina; `TimerAction` adiciona
10 s de espera antes de subir os dois nós seguintes.

### Como executar

```bash
ros2 launch drone_control mission_three_nodes.launch.py
```

> **Nota:** `drone_soft_land`, `drone_activator` e `drone_go_forward` são
> executáveis externos que precisam estar compilados e instalados no workspace.

---

## 4. `supervisor_T.launch.py`

### Papel / Responsabilidade

Sobe o nó `supervisor_T` com saída direcionada ao terminal. Não define
parâmetros fixos — todos os parâmetros do supervisor devem ser passados
via linha de comando se necessário.

### Nós lançados

| Nó | Pacote | Parâmetros |
|----|--------|-----------|
| `supervisor_T` | `drone_control` | nenhum (usa defaults do código) |

### Como executar

```bash
# Versão simples
ros2 launch drone_control supervisor_T.launch.py

# Com parâmetros customizados (via ros2 run, não via launch)
ros2 run drone_control supervisor_T --ros-args \
  -p wait_after_traj_done_s:=3.0 \
  -p base_tol_m:=0.15
```

---

## 5. `tf_body_fallback.launch.py`

### Papel / Responsabilidade

Publica uma transformação TF estática estrutural: `uav1/base_link` → `uav1/fcu`
com deslocamento zero. Serve como fallback para pacotes que esperam que o frame
`uav1/fcu` seja filho de `uav1/base_link` no TF tree.

### Nós lançados

| Nó | Frame pai | Frame filho | Translação | Rotação |
|----|-----------|------------|------------|---------|
| `tf_base_link_to_fcu` (`static_transform_publisher`) | `uav1/base_link` | `uav1/fcu` | 0, 0, 0 | 0°, 0°, 0° |

### Como executar

```bash
ros2 launch drone_control tf_body_fallback.launch.py

# Verificar no TF tree
ros2 run tf2_tools view_frames
```

---

## 6. `tf_camera_static.launch.py`

### Papel / Responsabilidade

Publica duas transformações TF estáticas com as posições físicas das câmeras
RGBD no drone: `uav1/fcu` → `uav1/rgbd_down` e `uav1/fcu` → `uav1/rgbd_front`.
Essencial para que `cv_bridge` e algoritmos de visão possam converter
coordenadas de câmera para o referencial do drone.

### Nós lançados

| Nó | Frame pai | Frame filho | Translação (x, y, z) | Rotação |
|----|-----------|------------|----------------------|---------|
| `tf_rgbd_down` | `uav1/fcu` | `uav1/rgbd_down` | 0.153, 0.0, -0.129 m | 0°, 0°, 0° |
| `tf_rgbd_front` | `uav1/fcu` | `uav1/rgbd_front` | 0.181, 0.0, -0.089 m | 0°, 0°, 0° |

### Interpretação das offsets

- Câmera **down**: 15.3 cm à frente e 12.9 cm abaixo do FCU (aponta para baixo).
- Câmera **front**: 18.1 cm à frente e 8.9 cm abaixo do FCU (aponta para frente).
- Sem rotação → os eixos da câmera estão alinhados com o frame `uav1/fcu`.

### Como executar

```bash
ros2 launch drone_control tf_camera_static.launch.py

# Verificar as transformações
ros2 run tf2_ros tf2_echo uav1/fcu uav1/rgbd_down
ros2 run tf2_ros tf2_echo uav1/fcu uav1/rgbd_front
```

### Uso combinado (TF completo)

Para um TF tree funcional em simulação, execute os dois launches de TF juntos:

```bash
# Terminal 1 — TF estrutural
ros2 launch drone_control tf_body_fallback.launch.py

# Terminal 2 — TF das câmeras
ros2 launch drone_control tf_camera_static.launch.py
```

O TF tree resultante é:

```
uav1/base_link
  └─ uav1/fcu
       ├─ uav1/rgbd_down
       └─ uav1/rgbd_front
```
