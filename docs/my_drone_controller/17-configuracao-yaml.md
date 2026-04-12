# 17 — `config/drone_config.yaml` — Parâmetros de Execução

> **Arquivo fonte:** `my_drone_controller/config/drone_config.yaml`
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.

---

## Visão geral

`drone_config.yaml` é o arquivo de parâmetros ROS 2 que configura em tempo de
execução todos os valores numéricos e booleanos da classe
`DroneControllerCompleto`. Ele segue o formato padrão de *params-file* do ROS 2:

```yaml
<nome_do_nó>:
  ros__parameters:
    <chave>: <valor>
```

O nó é registrado como `drone_controller_completo` (nome definido no construtor
de `DroneControllerCompleto`). Quando iniciado sem o `--params-file`, o
controlador usa os defaults definidos em `drone_config.h` (struct `DroneConfig`).

### Como carregar o arquivo

```bash
ros2 run my_drone_controller drone_node \
     --ros-args --params-file /path/to/my_drone_controller/config/drone_config.yaml
```

Ou via launch file:

```python
# my_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('my_drone_controller'),
        'config', 'drone_config.yaml'
    )
    return LaunchDescription([
        Node(
            package='my_drone_controller',
            executable='drone_node',
            parameters=[params_file],
        ),
    ])
```

---

## Bloco 1 — Altitude

```yaml
drone_controller_completo:
  ros__parameters:
    # ── Altitude ────────────────────────────────────────────────────────
    hover_altitude: 2.0
    hover_altitude_margin: 0.05
    max_altitude: 500.0
    waypoint_duration: 4.0
```

### Parâmetros do bloco

| Parâmetro | Tipo | Padrão | Unidade | Descrição |
|-----------|------|--------|---------|-----------|
| `hover_altitude` | `double` | `2.0` | m | Altitude-alvo de hover. O Estado 1 (decolagem) sobe até atingir este valor. |
| `hover_altitude_margin` | `double` | `0.05` | m | Histerese abaixo de `hover_altitude` usada para detectar chegada ao hover. |
| `max_altitude` | `double` | `500.0` | m | Teto máximo aceito em waypoints. Waypoints com Z > `max_altitude` são rejeitados. |
| `waypoint_duration` | `double` | `4.0` | s | Tempo de publicação de cada waypoint durante a execução da trajetória (Estado 3). |

### Correspondência com `load_parameters()`

```cpp
// drone_controller_completo.cpp — load_parameters()

this->declare_parameter("hover_altitude",        config_.hover_altitude);        // default = 2.0
this->declare_parameter("hover_altitude_margin", config_.hover_altitude_margin); // default = 0.05
this->declare_parameter("max_altitude",          config_.max_altitude);           // default = 500.0
this->declare_parameter("waypoint_duration",     config_.waypoint_duration);      // default = 4.0

config_.hover_altitude          = this->get_parameter("hover_altitude").as_double();
config_.hover_altitude_margin   = this->get_parameter("hover_altitude_margin").as_double();
config_.max_altitude            = this->get_parameter("max_altitude").as_double();
config_.waypoint_duration       = this->get_parameter("waypoint_duration").as_double();
```

**Uso em `fsm_takeoff.cpp`:**

```cpp
// takeoff_target_z_ = max(config_.hover_altitude, current_z_real_ + config_.takeoff_z_boost)
// Loop: publica setpoints até current_z_real_ >= takeoff_target_z_ - config_.hover_altitude_margin
```

**Uso em `fsm_trajectory.cpp`:**

```cpp
// Cada waypoint fica ativo por config_.waypoint_duration segundos antes de
// avançar para o próximo índice.
```

---

## Bloco 2 — Validação de waypoints

```yaml
    # ── Validation limits ────────────────────────────────────────────────
    max_waypoint_distance: 100.0
    min_altitude: 0.2
    land_z_threshold: 0.1
```

### Parâmetros do bloco

| Parâmetro | Tipo | Padrão | Unidade | Descrição |
|-----------|------|--------|---------|-----------|
| `max_waypoint_distance` | `double` | `100.0` | m | Distância máxima em X ou Y aceita em waypoints recebidos. |
| `min_altitude` | `double` | `0.2` | m | Altitude mínima de voo. Waypoints com Z ∈ [`land_z_threshold`, `min_altitude`) são rejeitados (não são intenção de pouso, mas estão abaixo do limite seguro). |
| `land_z_threshold` | `double` | `0.1` | m | Z abaixo do qual o drone é considerado pousado. Waypoints com Z < `land_z_threshold` são interpretados como intenção de pouso. |

### Correspondência com `load_parameters()`

```cpp
this->declare_parameter("max_waypoint_distance", config_.max_waypoint_distance); // default = 100.0
this->declare_parameter("min_altitude",          config_.min_altitude);           // default = 0.2
this->declare_parameter("land_z_threshold",      config_.land_z_threshold);       // default = 0.1

config_.max_waypoint_distance = this->get_parameter("max_waypoint_distance").as_double();
config_.min_altitude          = this->get_parameter("min_altitude").as_double();
config_.land_z_threshold      = this->get_parameter("land_z_threshold").as_double();
```

**Uso em `waypoint_validation.cpp`:**

```cpp
// Rejeita se |pose.position.x| > config_.max_waypoint_distance
// Rejeita se |pose.position.y| > config_.max_waypoint_distance
// Rejeita se z >= config_.land_z_threshold && z < config_.min_altitude
//   (zona proibida: não é pouso nem voo seguro)
```

**Log gerado em execução:**

```
⚙️  Configuração de Altitude: Mínima=0.20 m | Pouso detectado: Z < 0.10 m | Máxima=500.00 m
```

---

## Bloco 3 — Takeoff safety

```yaml
    # ── Takeoff Z boost ──────────────────────────────────────────────────
    takeoff_z_boost: 0.7

    # ── Takeoff XY sanitization ──────────────────────────────────────────
    takeoff_xy_origin_threshold_m: 0.2
    latch_pose_max_age_s: 10.0
```

### Parâmetros do bloco

| Parâmetro | Tipo | Padrão | Unidade | Descrição |
|-----------|------|--------|---------|-----------|
| `takeoff_z_boost` | `double` | `0.7` | m | Incremento mínimo em Z aplicado à altitude atual na hora do takeoff. Garante que o PX4 detecte intenção de subida e não dispare o auto-disarm. |
| `takeoff_xy_origin_threshold_m` | `double` | `0.2` | m | Raio ao redor da origem (0,0) dentro do qual o XY de takeoff é substituído pelo último *latch pose* armazenado. |
| `latch_pose_max_age_s` | `double` | `10.0` | s | Idade máxima de um *latch pose* para ser considerado válido na sanitização. |

### Correspondência com `load_parameters()`

```cpp
this->declare_parameter("takeoff_z_boost",                config_.takeoff_z_boost);               // 0.7
this->declare_parameter("takeoff_xy_origin_threshold_m",  config_.takeoff_xy_origin_threshold_m); // 0.2
this->declare_parameter("latch_pose_max_age_s",           config_.latch_pose_max_age_s);           // 10.0

config_.takeoff_z_boost                = this->get_parameter("takeoff_z_boost").as_double();
config_.takeoff_xy_origin_threshold_m  =
    this->get_parameter("takeoff_xy_origin_threshold_m").as_double();
config_.latch_pose_max_age_s           = this->get_parameter("latch_pose_max_age_s").as_double();
```

### Como `takeoff_z_boost` é usado

A altitude de takeoff é calculada **uma única vez** no momento em que o comando
chega — nunca a cada ciclo de controle (o que causaria o bug de subida infinita):

```cpp
// handle_single_takeoff_waypoint_command() — drone_controller_completo.cpp
takeoff_target_z_ = std::max(
    config_.hover_altitude,
    current_z_real_ + config_.takeoff_z_boost  // garante subida mínima de 0.7 m
);
```

Log gerado:

```
⚙️  Boost de decolagem: takeoff_z_boost=0.70 m
    (Z_alvo ≥ Z_real + takeoff_z_boost após ARM, evita auto-disarm do PX4)
```

### Como `takeoff_xy_origin_threshold_m` e `latch_pose_max_age_s` são usados

A função `sanitize_takeoff_xy()` em `drone_controller_completo.cpp` implementa
a seguinte lógica:

```cpp
// Se o XY solicitado está dentro de takeoff_xy_origin_threshold_m da origem
// E existe um latch pose com age <= latch_pose_max_age_s:
//   → substituir X e Y pelo latch pose (preservar Z do comando original)
// Caso contrário: usar o XY como recebido
if (dist_to_origin < config_.takeoff_xy_origin_threshold_m && has_latch_pose_) {
    double age = (now - last_latch_pose_time_).seconds();
    if (age <= config_.latch_pose_max_age_s) {
        // override XY
    }
}
```

Isso evita que o drone voe de volta para perto de (0,0) após um ciclo de pouso
quando o publisher da missão ainda não recebeu a posição correta.

---

## Bloco 4 — Timeouts

```yaml
    # ── Timeouts ─────────────────────────────────────────────────────────
    activation_timeout: 3.0
    command_timeout: 15.0
    landing_timeout: 3.0
    offboard_confirm_timeout: 5.0
```

### Parâmetros do bloco

| Parâmetro | Tipo | Padrão | Unidade | Descrição |
|-----------|------|--------|---------|-----------|
| `activation_timeout` | `double` | `3.0` | s | Timeout aguardando confirmação de OFFBOARD + ARM do FCU. |
| `command_timeout` | `double` | `15.0` | s | Tempo máximo que um comando pode ficar `PENDING` antes de ser marcado `TIMEOUT` pela `CommandQueue`. |
| `landing_timeout` | `double` | `3.0` | s | Tempo aguardando após detecção de pouso antes de solicitar DISARM. |
| `offboard_confirm_timeout` | `double` | `5.0` | s | Timeout aguardando o FCU confirmar modo OFFBOARD antes de enviar ARM. |

### Correspondência com `load_parameters()`

```cpp
this->declare_parameter("activation_timeout",       config_.activation_timeout);      // 3.0
this->declare_parameter("command_timeout",          config_.command_timeout);          // 15.0
this->declare_parameter("landing_timeout",          config_.landing_timeout);          // 3.0
this->declare_parameter("offboard_confirm_timeout", config_.offboard_confirm_timeout); // 5.0

config_.activation_timeout      = this->get_parameter("activation_timeout").as_double();
config_.command_timeout         = this->get_parameter("command_timeout").as_double();
config_.landing_timeout         = this->get_parameter("landing_timeout").as_double();
config_.offboard_confirm_timeout = this->get_parameter("offboard_confirm_timeout").as_double();
```

### Fluxo dos timeouts no Estado 1 (takeoff)

```
1. drone recebe waypoint de takeoff
2. começa streaming pré-ARM (INITIAL_STREAM_THRESHOLD setpoints)
3. envia SET_MODE OFFBOARD → aguarda confirmação por até offboard_confirm_timeout
4. FCU confirma OFFBOARD → streaming pós-OFFBOARD (POST_OFFBOARD_STREAM_THRESHOLD)
5. envia ARM → aguarda por activation_timeout
6. FCU confirma ARM → sobe para takeoff_target_z_
```

O `command_timeout` atua na `CommandQueue`: qualquer comando (ARM, DISARM,
TRAJECTORY…) que não receba confirmação dentro de `command_timeout` segundos é
automaticamente marcado como `TIMEOUT` e logado.

O `landing_timeout` é usado no Estado 4 (landing):

```cpp
// fsm_landing.cpp
// Detecta Z < land_z_threshold → inicia contador
// Se elapsed >= config_.landing_timeout → solicita DISARM
```

---

## Relação com `drone_config.h`

`drone_config.h` define a `struct DroneConfig` com os mesmos campos e os mesmos
valores padrão. O YAML **sobrescreve** esses valores em runtime; sem `--params-file`,
os defaults de `drone_config.h` prevalecerão.

```cpp
// drone_config.h — struct DroneConfig (trecho)
struct DroneConfig {
  double hover_altitude{2.0};          // ← mesmo valor do YAML
  double hover_altitude_margin{0.05};
  double max_altitude{500.0};
  double waypoint_duration{4.0};
  double min_altitude{0.2};
  double land_z_threshold{0.1};
  double max_waypoint_distance{100.0};
  double takeoff_z_boost{0.7};
  double takeoff_xy_origin_threshold_m{0.2};
  double latch_pose_max_age_s{10.0};
  double activation_timeout{5.0};      // ← nota: default no .h é 5.0, no .yaml é 3.0
  double command_timeout{15.0};
  double landing_timeout{3.0};
  double offboard_confirm_timeout{5.0};
};
```

> **Atenção:** `activation_timeout` tem default `5.0` em `drone_config.h` e `3.0`
> no YAML. Quando o arquivo é carregado, o YAML prevalece (`3.0`). Sem o arquivo,
> o nó usa `5.0`.

---

## Parâmetros adicionais (não presentes no YAML, configuráveis via linha de comando)

Os parâmetros abaixo existem no nó mas **não estão no arquivo YAML** — eles só
podem ser passados pela linha de comando (`--ros-args -p <chave>:=<valor>`):

| Parâmetro | Tipo | Padrão | Descrição |
|-----------|------|--------|-----------|
| `enabled` | `bool` | `true` | Liga/desliga o controlador completamente. |
| `override_active` | `bool` | `false` | Permite override externo de setpoints. |
| `monitor_waypoint_goal_rate_hz` | `double` | `5.0` | Taxa de republish do waypoint goal (Hz). |
| `monitor_waypoints_rate_hz` | `double` | `1.0` | Taxa de republish da lista de waypoints (Hz). |
| `monitor_publish_only_when_active` | `bool` | `true` | Suprime republish quando ocioso. |
| `publish_state_voo` | `bool` | `true` | Habilita publicação de `/drone_controller/state_voo`. |
| `state_voo_pub_rate_hz` | `double` | `1.0` | Taxa de republish do estado da FSM (Hz). |
| `waypoints_cmd_topic` | `string` | `/waypoints` | Tópico de entrada de comandos de trajetória. |
| `waypoints_status_topic` | `string` | `/waypoints` | Tópico de republish de status de trajetória. |
| `waypoint_goal_cmd_topic` | `string` | `/waypoint_goal` | Tópico de entrada de waypoint goal. |
| `waypoint_goal_status_topic` | `string` | `/waypoint_goal` | Tópico de republish de waypoint goal. |
