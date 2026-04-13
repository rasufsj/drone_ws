# Mensagens Customizadas — `drone_control`

> **Objetivo:** explicar por blocos de código cada arquivo `.msg` do pacote
> `drone_control`, cobrindo campos, semântica, convenções de uso e exemplos
> de publicação em C++ e via CLI.
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.
> Última sincronização: branch `main`, 2026-04-13.

---

## 1. `YawOverride.msg`

### Papel / Responsabilidade

Controla o mecanismo de **override de yaw** do `my_drone_controller`: permite que
um nó externo (como `drone_yaw_360`) assuma temporariamente o controle da
orientação do drone, sobrescrevendo o controle de yaw da FSM de trajetória.

### Definição completa

```
bool enable
float32 yaw_rate
float32 timeout
```

### Bloco 1 — Campo `enable`

```
bool enable
```

**Semântica:** liga/desliga o override.

- `enable = true` → o `my_drone_controller` para de usar o yaw calculado pela
  trajetória e aplica `yaw_rate` diretamente no setpoint de velocidade angular.
- `enable = false` → o override é desativado e o controlador retorna ao controle
  normal de yaw (heading para o próximo waypoint ou heading mantido).

**Caso de uso típico:**

```cpp
// Ativar override antes de iniciar o giro
drone_control::msg::YawOverride msg;
msg.enable = true;
msg.yaw_rate = 1.0f;  // rad/s CCW
msg.timeout = 15.0f;
pub_->publish(msg);
```

### Bloco 2 — Campo `yaw_rate`

```
float32 yaw_rate
```

**Semântica:** velocidade angular desejada em **rad/s** em torno do eixo Z (yaw).

- Positivo (`> 0`) → rotação anti-horária (CCW) vista de cima — sentido
  matemático positivo no referencial ENU.
- Negativo (`< 0`) → rotação horária (CW).
- Zero → sem rotação; usado para desativar suavemente sem mudar `enable`.

**Faixa típica:** `[-2π, +2π]` rad/s; valores muito altos podem causar
instabilidade dependendo do controlador de atitude do FCU.

### Bloco 3 — Campo `timeout`

```
float32 timeout
```

**Semântica:** tempo máximo em segundos que o override permanece ativo.
Serve como **válvula de segurança**: se o nó publicador encerrar abruptamente
(crash, Ctrl+C), o `my_drone_controller` desativa o override automaticamente
após `timeout` segundos sem receber um novo comando.

- Valor `0.0` com `enable = false` → desativa imediatamente sem timeout.
- Valor `15.0` → garante que, mesmo se `drone_yaw_360` travar, o drone não
  gira indefinidamente.

### Como `drone_yaw_360` usa esta mensagem

```cpp
// Ativar — publicado uma única vez ao início do giro
void publish_enable() {
    drone_control::msg::YawOverride msg;
    msg.enable   = true;
    msg.yaw_rate = (yaw_target_delta_ >= 0.0) ? std::abs(yaw_rate_) : -std::abs(yaw_rate_);
    msg.timeout  = 15.0f;
    pub_->publish(msg);
}

// Desativar — publicado ao atingir o ângulo acumulado
void publish_disable() {
    drone_control::msg::YawOverride msg;
    msg.enable   = false;
    msg.yaw_rate = 0.0;
    msg.timeout  = 0.0;
    pub_->publish(msg);
    rclcpp::shutdown();
}
```

### Exemplo CLI

```bash
# Ativar giro CCW a 0.8 rad/s por até 15 s
ros2 topic pub /uav1/yaw_override/cmd drone_control/msg/YawOverride \
  "{enable: true, yaw_rate: 0.8, timeout: 15.0}" --once

# Desativar imediatamente
ros2 topic pub /uav1/yaw_override/cmd drone_control/msg/YawOverride \
  "{enable: false, yaw_rate: 0.0, timeout: 0.0}" --once

# Monitorar estado do override
ros2 topic echo /uav1/yaw_override/cmd
```

---

## 2. `Waypoint4D.msg`

### Papel / Responsabilidade

Representa um **waypoint de navegação tridimensional com orientação** (yaw)
opcional. É a unidade básica de trajetória "rica" no sistema — ao contrário
do `geometry_msgs/Pose` simples em `/waypoints`, este tipo permite especificar
explicitamente a orientação angular desejada no waypoint.

### Definição completa

```
# 4D waypoint: position (x, y, z) plus optional absolute yaw around the Z axis.
# Convention: yaw = NaN means "no yaw provided – keep current heading".
# yaw is in radians; values are normalized to [-pi, pi] by the controller.
geometry_msgs/Pose pose
float32 yaw
```

### Bloco 1 — Campo `pose`

```
geometry_msgs/Pose pose
```

**Semântica:** posição (x, y, z) e orientação quaternion do waypoint no frame
de referência configurado (tipicamente `map`).

- Em uso típico, apenas `pose.position.{x,y,z}` é relevante para navegação.
- `pose.orientation` pode ser identidade `{w: 1.0}` — o yaw é controlado
  pelo campo `yaw`, não pela quaternion.
- Referencial: **ENU** (East-North-Up). `+x` → Leste, `+y` → Norte, `+z` → Cima.

### Bloco 2 — Campo `yaw`

```
float32 yaw
```

**Semântica:** ângulo de yaw absoluto em **radianos**, normalizado para `[-π, π]`
pelo controlador ao processar o waypoint.

- `yaw = 0.0` → drone aponta para Leste (eixo +X do ENU).
- `yaw = π/2 ≈ 1.5708` → drone aponta para Norte (+Y).
- `yaw = π ≈ 3.1416` → drone aponta para Oeste (-X).
- `yaw = NaN` → **convenção especial**: indica que o yaw não foi especificado;
  o controlador mantém o heading atual do drone.

**Por que NaN e não 0?**  
Zero é um valor de yaw válido (aponta para Leste). Usar `NaN` permite distinguir
"sem especificação" de "apontar para Leste", sem precisar de um campo booleano
extra.

### Exemplo C++ de criação de um Waypoint4D

```cpp
#include "drone_control/msg/waypoint4_d.hpp"
#include <cmath>
#include <limits>

// Waypoint em (3.0, 2.0, 1.5) com yaw apontando para Norte (π/2 rad)
drone_control::msg::Waypoint4D wp;
wp.pose.position.x = 3.0;
wp.pose.position.y = 2.0;
wp.pose.position.z = 1.5;
wp.pose.orientation.w = 1.0;  // quaternion identidade
wp.yaw = M_PI / 2.0f;        // 90° → Norte

// Waypoint sem yaw (manter heading atual)
drone_control::msg::Waypoint4D wp_no_yaw;
wp_no_yaw.pose.position.x = 1.0;
wp_no_yaw.pose.position.y = 0.0;
wp_no_yaw.pose.position.z = 1.5;
wp_no_yaw.yaw = std::numeric_limits<float>::quiet_NaN();
```

### Exemplo CLI

```bash
# Inspecionar o tipo
ros2 interface show drone_control/msg/Waypoint4D

# Publicar um waypoint 4D isolado (tipicamente não usado sozinho — use Waypoint4DArray)
ros2 topic pub /waypoint_goal_4d drone_control/msg/Waypoint4D \
  "{pose: {position: {x: 2.0, y: 1.0, z: 1.5}, orientation: {w: 1.0}}, yaw: 1.5708}" \
  --once
```

---

## 3. `Waypoint4DArray.msg`

### Papel / Responsabilidade

Container de alto nível para enviar uma **trajetória completa** de waypoints
com orientação ao `my_drone_controller`. O cabeçalho ROS 2 fornece o
frame de referência e o timestamp da trajetória.

### Definição completa

```
# Array of 4D waypoints with a common header (frame_id, timestamp).
std_msgs/Header header
drone_control/Waypoint4D[] waypoints
```

### Bloco 1 — Campo `header`

```
std_msgs/Header header
```

**Semântica:** metadados da trajetória.

- `header.frame_id` → frame de referência dos waypoints (ex.: `"map"`).
- `header.stamp` → timestamp de criação; o controlador pode usar para
  verificar a age da mensagem e descartar trajetórias antigas.

**Uso típico:**

```cpp
drone_control::msg::Waypoint4DArray traj;
traj.header.frame_id = "map";
traj.header.stamp    = this->now();
```

### Bloco 2 — Campo `waypoints`

```
drone_control/Waypoint4D[] waypoints
```

**Semântica:** sequência **ordenada** de waypoints 4D que o drone deve percorrer.
O controlador processa os waypoints na ordem do array: `waypoints[0]`,
`waypoints[1]`, …, `waypoints[N-1]`.

**Atenção:** um array vazio (`waypoints = []`) pode ser interpretado como
"limpar trajetória atual" pelo controlador — verifique o comportamento do
`my_drone_controller` para este caso.

### Bloco 3 — Relação com `/waypoints` (PoseArray)

| Tipo | Tópico típico | Suporte a yaw | Quando usar |
|------|--------------|---------------|-------------|
| `geometry_msgs/PoseArray` | `/waypoints` | ✗ (sem yaw) | Comandos simples de posição |
| `drone_control/Waypoint4DArray` | `/waypoints_4d` | ✓ (yaw por waypoint) | Missões com orientação explícita |

### Exemplo C++ completo

```cpp
#include "drone_control/msg/waypoint4_d_array.hpp"
#include "drone_control/msg/waypoint4_d.hpp"
#include <cmath>

// Criar trajetória de 3 waypoints com yaw crescente
drone_control::msg::Waypoint4DArray traj;
traj.header.frame_id = "map";
traj.header.stamp    = this->now();

// Waypoint 1: ir para (2, 0, 1.5) apontando para Leste
drone_control::msg::Waypoint4D wp1;
wp1.pose.position.x = 2.0;
wp1.pose.position.y = 0.0;
wp1.pose.position.z = 1.5;
wp1.pose.orientation.w = 1.0;
wp1.yaw = 0.0f;
traj.waypoints.push_back(wp1);

// Waypoint 2: ir para (2, 2, 1.5) apontando para Norte
drone_control::msg::Waypoint4D wp2;
wp2.pose.position.x = 2.0;
wp2.pose.position.y = 2.0;
wp2.pose.position.z = 1.5;
wp2.pose.orientation.w = 1.0;
wp2.yaw = static_cast<float>(M_PI / 2.0);  // 90°
traj.waypoints.push_back(wp2);

// Waypoint 3: voltar para (0, 0, 1.5) sem especificação de yaw
drone_control::msg::Waypoint4D wp3;
wp3.pose.position.x = 0.0;
wp3.pose.position.y = 0.0;
wp3.pose.position.z = 1.5;
wp3.pose.orientation.w = 1.0;
wp3.yaw = std::numeric_limits<float>::quiet_NaN();  // manter heading
traj.waypoints.push_back(wp3);

pub_->publish(traj);
```

### Exemplo CLI

```bash
# Enviar trajetória com dois waypoints
ros2 topic pub /waypoints_4d drone_control/msg/Waypoint4DArray '{
  header: {frame_id: "map"},
  waypoints: [
    {pose: {position: {x: 2.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}, yaw: 0.0},
    {pose: {position: {x: 2.0, y: 2.0, z: 1.5}, orientation: {w: 1.0}}, yaw: 1.5708}
  ]
}' --once

# Monitorar trajetórias chegando
ros2 topic echo /waypoints_4d

# Verificar o tipo completo incluindo Waypoint4D aninhado
ros2 interface show drone_control/msg/Waypoint4DArray
```

---

## 4. Relação entre as mensagens

```
Waypoint4DArray
  ├── header (std_msgs/Header)
  │     ├── stamp (builtin_interfaces/Time)
  │     └── frame_id (string)
  └── waypoints[] (Waypoint4D)
        ├── pose (geometry_msgs/Pose)
        │     ├── position (geometry_msgs/Point) ← x, y, z
        │     └── orientation (geometry_msgs/Quaternion) ← tipicamente identidade
        └── yaw (float32) ← rad; NaN = sem especificação
```

**Regras de uso:**

1. Sempre definir `header.frame_id` (tipicamente `"map"`).
2. Usar `pose.orientation.w = 1.0` (quaternion identidade) quando apenas
   a posição importa.
3. Usar `yaw = NaN` quando o heading deve ser mantido entre waypoints.
4. Normalizar yaw para `[-π, π]` antes de publicar para evitar valores
   ambíguos (o controlador também normaliza, mas é boa prática).
