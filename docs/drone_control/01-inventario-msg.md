# Inventário de Mensagens — `drone_control`

> **Objetivo:** listar e descrever as três mensagens customizadas do pacote
> `drone_control`, indicando campos, semântica, quais nós produzem/consomem
> cada mensagem e exemplos de publicação via CLI.
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.
> Última sincronização: branch `main`, 2026-04-13.

---

## Índice

| # | Arquivo | Tipo | Doc detalhada |
|---|---------|------|---------------|
| 1 | [`msg/YawOverride.msg`](#1-msgyawoverridemsg) | Comando de yaw rate | [12-mensagens.md](12-mensagens.md#1-yawoverridemsg) |
| 2 | [`msg/Waypoint4D.msg`](#2-msgwaypoint4dmsg) | Waypoint posição + yaw | [12-mensagens.md](12-mensagens.md#2-waypoint4dmsg) |
| 3 | [`msg/Waypoint4DArray.msg`](#3-msgwaypoint4darraymsg) | Array de waypoints 4D | [12-mensagens.md](12-mensagens.md#3-waypoint4darraymsg) |

---

## 1. `msg/YawOverride.msg`

### Papel / Responsabilidade

Comando de controle de yaw enviado ao `my_drone_controller` para sobrescrever
temporariamente o controle de orientação do drone. Usado pelo nó `drone_yaw_360`
para executar rotações angulares controladas.

### Definição

```
bool enable
float32 yaw_rate
float32 timeout
```

### Campos

| Campo | Tipo | Semântica |
|-------|------|-----------|
| `enable` | `bool` | `true` → ativa o override; `false` → desativa e retorna controle ao controlador |
| `yaw_rate` | `float32` | Velocidade angular desejada em rad/s (positivo = anti-horário/CCW, negativo = horário/CW) |
| `timeout` | `float32` | Tempo máximo em segundos para o override permanecer ativo; valor de segurança caso o nó que enviou o comando encerre abruptamente |

### Quem publica / assina

| Nó | Papel |
|----|-------|
| `drone_yaw_360` (`drone_control`) | **Publica** em `/uav1/yaw_override/cmd` |
| `my_drone_controller` | **Assina** para executar o override de yaw |

### Exemplo de uso — CLI

```bash
# Ativar giro CCW a 0.5 rad/s por até 10 segundos
ros2 topic pub /uav1/yaw_override/cmd drone_control/msg/YawOverride \
  "{enable: true, yaw_rate: 0.5, timeout: 10.0}" --once

# Desativar override (retornar controle ao controlador)
ros2 topic pub /uav1/yaw_override/cmd drone_control/msg/YawOverride \
  "{enable: false, yaw_rate: 0.0, timeout: 0.0}" --once

# Monitorar o tópico
ros2 topic echo /uav1/yaw_override/cmd
```

---

## 2. `msg/Waypoint4D.msg`

### Papel / Responsabilidade

Waypoint de navegação tridimensional com orientação opcional em yaw. Representa
um ponto no espaço (x, y, z) acrescido de um ângulo de yaw absoluto. Quando `yaw = NaN`,
o controlador mantém o heading atual do drone.

### Definição

```
# 4D waypoint: position (x, y, z) plus optional absolute yaw around the Z axis.
# Convention: yaw = NaN means "no yaw provided – keep current heading".
# yaw is in radians; values are normalized to [-pi, pi] by the controller.
geometry_msgs/Pose pose
float32 yaw
```

### Campos

| Campo | Tipo | Semântica |
|-------|------|-----------|
| `pose` | `geometry_msgs/Pose` | Posição (x, y, z) e orientação quaternion do waypoint. Em uso típico, apenas `pose.position` é relevante; o quaternion pode ser identidade |
| `yaw` | `float32` | Yaw absoluto em radianos, normalizado para `[-π, π]` pelo controlador. `NaN` = sem especificação de yaw |

### Convenções

- Referencial: ENU (East-North-Up), frame `map` por padrão.
- `yaw = 0` aponta para Leste (eixo +X do ENU).
- `yaw = π/2` aponta para Norte (eixo +Y do ENU).

### Quem publica / assina

| Nó / Pacote | Papel |
|-------------|-------|
| Código externo / `my_drone_controller` | **Publica** arrays de `Waypoint4D` via `Waypoint4DArray` |
| `my_drone_controller` | **Assina** para extrair waypoints individuais |

### Exemplo de uso — CLI

```bash
# Inspecionar o tipo
ros2 interface show drone_control/msg/Waypoint4D

# Publicar um único waypoint (usado dentro de Waypoint4DArray)
ros2 topic pub /waypoints_4d drone_control/msg/Waypoint4DArray \
  "{header: {frame_id: 'map'}, waypoints: [{pose: {position: {x: 2.0, y: 1.0, z: 1.5}}, yaw: 1.5708}]}" \
  --once
```

---

## 3. `msg/Waypoint4DArray.msg`

### Papel / Responsabilidade

Array de waypoints 4D com cabeçalho ROS 2 (timestamp + frame_id). É o tipo
de mensagem de alto nível usado para enviar trajetórias completas ao
`my_drone_controller`.

### Definição

```
# Array of 4D waypoints with a common header (frame_id, timestamp).
std_msgs/Header header
drone_control/Waypoint4D[] waypoints
```

### Campos

| Campo | Tipo | Semântica |
|-------|------|-----------|
| `header` | `std_msgs/Header` | Carimbo de tempo e frame de referência (ex.: `frame_id: "map"`) |
| `waypoints` | `drone_control/Waypoint4D[]` | Sequência ordenada de waypoints a percorrer |

### Relação com `my_drone_controller`

Este tipo é o equivalente "rico" (com yaw) do `geometry_msgs/PoseArray` (`/waypoints`).
O tópico `/waypoints_4d` é assinado pelo `my_drone_controller` e tem prioridade
sobre `/waypoints` quando a rota inclui comandos de orientação.

| Tópico | Tipo | Pacote consumidor |
|--------|------|-------------------|
| `/waypoints_4d` | `drone_control/Waypoint4DArray` | `my_drone_controller` |
| `/waypoint_goal_4d` | `drone_control/Waypoint4D` | `my_drone_controller` |

### Exemplo de uso — CLI

```bash
# Enviar trajetória com dois waypoints e yaw explícito
ros2 topic pub /waypoints_4d drone_control/msg/Waypoint4DArray '{
  header: {frame_id: "map"},
  waypoints: [
    {pose: {position: {x: 1.0, y: 0.0, z: 1.5}}, yaw: 0.0},
    {pose: {position: {x: 3.0, y: 2.0, z: 1.5}}, yaw: 1.5708}
  ]
}' --once

# Monitorar waypoints chegando
ros2 topic echo /waypoints_4d

# Inspecionar o tipo completo
ros2 interface show drone_control/msg/Waypoint4DArray
```
