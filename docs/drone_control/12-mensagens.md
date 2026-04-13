# Mensagens Customizadas — `drone_control`

> **Objetivo:** explicar **linha por linha** cada arquivo `.msg` do pacote
> `drone_control`, cobrindo cada campo, sua semântica, restrições de valor,
> quais nós produzem/consomem e exemplos completos de uso em C++ e CLI.
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.
> Última sincronização: branch `main`, 2026-04-13.

---

## 1. `YawOverride.msg`

### Definição completa do arquivo

```
bool enable
float32 yaw_rate
float32 timeout
```

### Linha 1 — `bool enable`

```
bool enable
```

**Tipo:** `bool` (1 byte, `true`/`false`).

**Semântica:** liga e desliga o mecanismo de override de yaw no `my_drone_controller`.

- `enable = true` → o controlador suspende o cálculo de yaw da FSM de trajetória
  e aplica `yaw_rate` diretamente no campo angular do setpoint MAVROS.
- `enable = false` → o override é desativado; o controlador retorna ao controle
  normal de heading (apontar para o próximo waypoint ou manter o heading atual).

**Por que `bool` e não um enum?** O override tem apenas dois estados relevantes
(ativo/inativo); um `bool` é suficiente e mais simples de publicar via CLI e código.

**Exemplo de uso no nó `drone_yaw_360`:**

```cpp
// Ativar — ao iniciar o giro
drone_control::msg::YawOverride msg;
msg.enable = true;   // ← esta linha ativa o mecanismo
pub_->publish(msg);

// Desativar — ao concluir o giro
msg.enable = false;  // ← esta linha libera o controle de volta à FSM
pub_->publish(msg);
```

### Linha 2 — `float32 yaw_rate`

```
float32 yaw_rate
```

**Tipo:** `float32` (IEEE 754 single-precision, 4 bytes).

**Semântica:** velocidade angular desejada em **rad/s** em torno do eixo Z (eixo de yaw)
no referencial ENU.

- `yaw_rate > 0` → rotação **anti-horária (CCW)** vista de cima — sentido matemático
  positivo no referencial ENU (East-North-Up). Ex.: `1.0` rad/s = ≈57°/s CCW.
- `yaw_rate < 0` → rotação **horária (CW)**. Ex.: `-0.5` rad/s = ≈29°/s CW.
- `yaw_rate = 0` → sem rotação; usado apenas quando `enable = false` para indicar
  intenção de parada.

**Faixa típica:** `[-2.0, +2.0]` rad/s. Valores acima de 2 rad/s (≈115°/s) podem
causar instabilidade no controlador de atitude do FCU (PX4) dependendo da
configuração de ganhos.

**Como `drone_yaw_360.cpp` define o sinal:**

```cpp
// Linha 103 de drone_yaw_360.cpp:
msg.yaw_rate = (yaw_target_delta_ >= 0.0) ? std::abs(yaw_rate_) : -std::abs(yaw_rate_);
//                       ↑                         ↑                      ↑
//         se yaw_target > 0 (CCW)    usa positivo            usa negativo (CW)
```

- `yaw_target_delta_ >= 0.0` → o alvo é uma rotação CCW → `yaw_rate` recebe valor positivo.
- `yaw_target_delta_ < 0.0` → o alvo é uma rotação CW → `yaw_rate` recebe valor negativo.
- `std::abs(yaw_rate_)` garante que o parâmetro de configuração pode ser dado em módulo
  independentemente do sentido.

### Linha 3 — `float32 timeout`

```
float32 timeout
```

**Tipo:** `float32` (IEEE 754 single-precision, 4 bytes), representando segundos.

**Semântica:** tempo máximo (em segundos) que o override permanece ativo após a
última mensagem recebida. Funciona como **válvula de segurança**: se o nó
publicador encerrar abruptamente (crash, Ctrl+C, kill), o `my_drone_controller`
desativa o override automaticamente após `timeout` segundos.

- `timeout = 0.0` com `enable = false` → desativa imediatamente.
- `timeout = 15.0` → garante que, mesmo se `drone_yaw_360` travar, o drone
  não gira indefinidamente por mais de 15 segundos.

**Linha no código do `drone_yaw_360.cpp`:**

```cpp
// Linha 104 — valor fixo de segurança
msg.timeout = 15.0f;
// ↑ 15 s é mais que suficiente para um giro de 360° a 0.8 rad/s (≈7.85 s)
```

### Exemplo CLI completo

```bash
# Ativar giro CCW a 0.8 rad/s com timeout de 15 s
ros2 topic pub /uav1/yaw_override/cmd drone_control/msg/YawOverride \
  "{enable: true, yaw_rate: 0.8, timeout: 15.0}" --once

# Desativar override imediatamente
ros2 topic pub /uav1/yaw_override/cmd drone_control/msg/YawOverride \
  "{enable: false, yaw_rate: 0.0, timeout: 0.0}" --once

# Monitorar estado do override em tempo real
ros2 topic echo /uav1/yaw_override/cmd
```

---

## 2. `Waypoint4D.msg`

### Definição completa do arquivo

```
# 4D waypoint: position (x, y, z) plus optional absolute yaw around the Z axis.
# Convention: yaw = NaN means "no yaw provided – keep current heading".
# yaw is in radians; values are normalized to [-pi, pi] by the controller.
geometry_msgs/Pose pose
float32 yaw
```

### Linhas 1-3 — Comentários de documentação

```
# 4D waypoint: position (x, y, z) plus optional absolute yaw around the Z axis.
# Convention: yaw = NaN means "no yaw provided – keep current heading".
# yaw is in radians; values are normalized to [-pi, pi] by the controller.
```

| Linha de comentário | O que documenta |
|--------------------|-----------------|
| `# 4D waypoint: ...` | Explica o conceito: posição 3D (x,y,z) + orientação (yaw) = 4 graus de liberdade = "4D" |
| `# Convention: yaw = NaN ...` | **Convenção crítica**: `NaN` (Not a Number) é usado como sentinela para indicar "sem yaw especificado", diferenciando de `yaw=0` que é um yaw válido (aponta para Leste) |
| `# yaw is in radians; ...` | Unidade e normalização: yaw em radianos, normalizado pelo controlador para o intervalo canônico `[-π, π]` |

### Linha 4 — `geometry_msgs/Pose pose`

```
geometry_msgs/Pose pose
```

**Tipo composto** — contém dois sub-campos:

```
geometry_msgs/Pose:
  geometry_msgs/Point position
    float64 x    ← coordenada Este (ENU), metros
    float64 y    ← coordenada Norte (ENU), metros
    float64 z    ← coordenada Cima (ENU), metros
  geometry_msgs/Quaternion orientation
    float64 x    ← componente i do quaternion
    float64 y    ← componente j do quaternion
    float64 z    ← componente k do quaternion
    float64 w    ← componente escalar do quaternion
```

**Uso prático no `drone_control`:**

- `pose.position.{x,y,z}` → coordenadas do waypoint no frame `map` (ENU).
- `pose.orientation` → tipicamente quaternion identidade `{x:0, y:0, z:0, w:1}`.
  A orientação angular é controlada pelo campo `yaw`; o quaternion em `pose` não
  é usado para controle de heading.

**Por que incluir `Pose` em vez de apenas `Point`?** Para compatibilidade com
a mensagem `geometry_msgs/PoseArray` usada em `/waypoints` e para eventuais
extensões futuras que usem a orientação completa.

### Linha 5 — `float32 yaw`

```
float32 yaw
```

**Tipo:** `float32` (IEEE 754 single-precision, 4 bytes), representando radianos.

**Semântica:** ângulo de yaw absoluto desejado no waypoint, em radianos, no
referencial ENU.

| Valor | Direção do drone (ENU) |
|-------|----------------------|
| `0.0` | Leste (+X) |
| `π/2 ≈ 1.5708` | Norte (+Y) |
| `π ≈ 3.1416` | Oeste (-X) |
| `-π/2 ≈ -1.5708` | Sul (-Y) |
| `NaN` | Manter heading atual (convenção especial) |

**Por que `NaN` e não um campo booleano separado?**

`float32` em C++ pode representar `NaN` de forma nativa:
```cpp
#include <limits>
float yaw_no_spec = std::numeric_limits<float>::quiet_NaN();
// std::isnan(yaw_no_spec) == true
```

Usar `NaN` evita adicionar um campo booleano extra (`has_yaw: bool`) que
adicionaria 1 byte de overhead e complicaria a serialização. O custo é que
o código consumidor deve verificar `std::isnan(waypoint.yaw)` antes de usar.

**Normalização:** o `my_drone_controller` normaliza o valor para `[-π, π]` ao
processar o waypoint. Portanto, `yaw = 4.71` (≡ `-π/2 + 2π`) é tratado
da mesma forma que `yaw = -1.5708`.

### Exemplo C++ completo de criação

```cpp
#include "drone_control/msg/waypoint4_d.hpp"
#include <cmath>
#include <limits>

// ── Waypoint com posição E yaw especificados ──────────────────────────────
drone_control::msg::Waypoint4D wp;

// Campo pose.position: coordenadas ENU em metros
wp.pose.position.x = 3.0;   // 3 m para Leste
wp.pose.position.y = 2.0;   // 2 m para Norte
wp.pose.position.z = 1.5;   // 1.5 m de altitude

// Campo pose.orientation: quaternion identidade (não usamos para controle de heading)
wp.pose.orientation.x = 0.0;
wp.pose.orientation.y = 0.0;
wp.pose.orientation.z = 0.0;
wp.pose.orientation.w = 1.0;

// Campo yaw: 90° = π/2 rad = drone aponta para Norte
wp.yaw = static_cast<float>(M_PI / 2.0);  // cast para float32

// ── Waypoint SEM yaw especificado (manter heading atual) ─────────────────
drone_control::msg::Waypoint4D wp_no_yaw;
wp_no_yaw.pose.position.x = 1.0;
wp_no_yaw.pose.position.y = 0.0;
wp_no_yaw.pose.position.z = 1.5;
wp_no_yaw.pose.orientation.w = 1.0;
wp_no_yaw.yaw = std::numeric_limits<float>::quiet_NaN();  // convenção "sem yaw"
```

---

## 3. `Waypoint4DArray.msg`

### Definição completa do arquivo

```
# Array of 4D waypoints with a common header (frame_id, timestamp).
std_msgs/Header header
drone_control/Waypoint4D[] waypoints
```

### Linha 1 — Comentário de documentação

```
# Array of 4D waypoints with a common header (frame_id, timestamp).
```

Documenta que o array compartilha um único header ROS 2 entre todos os waypoints.
Isso implica que todos os waypoints do array estão no mesmo frame de referência
e foram gerados no mesmo instante de tempo (ou pelo menos pelo mesmo publicador).

### Linha 2 — `std_msgs/Header header`

```
std_msgs/Header header
```

**Tipo composto:**

```
std_msgs/Header:
  builtin_interfaces/Time stamp
    int32 sec     ← segundos desde a epoch ROS
    uint32 nanosec ← nanosegundos fracionários
  string frame_id  ← identificador do frame de referência (ex.: "map")
```

**Sub-campo `stamp`:**
- Usado pelo controlador para verificar a "freshness" (idade) da mensagem.
- Se `stamp` for muito antigo, o controlador pode descartar a trajetória para
  evitar executar comandos obsoletos.
- Publicar com `this->now()` garante timestamp atual.

**Sub-campo `frame_id`:**
- Define em qual sistema de coordenadas os waypoints estão expressos.
- Valor típico: `"map"` — frame ENU fixo no mundo.
- O controlador usa `frame_id` para validar que os waypoints estão no frame
  esperado antes de executar a trajetória.

**Uso no código C++:**

```cpp
drone_control::msg::Waypoint4DArray traj;

// stamp: registra o instante de criação da trajetória
traj.header.stamp = this->now();
// frame_id: declara que os waypoints estão no frame "map" (ENU)
traj.header.frame_id = "map";
```

### Linha 3 — `drone_control/Waypoint4D[] waypoints`

```
drone_control/Waypoint4D[] waypoints
```

**Tipo:** array dinâmico (`[]`) de `drone_control/Waypoint4D`.

- `[]` sem tamanho fixo → array de tamanho variável (dinâmico). O publisher
  decide quantos waypoints incluir.
- A ordem do array é **significativa**: o controlador processa `waypoints[0]`,
  depois `waypoints[1]`, ..., `waypoints[N-1]`.
- Array vazio (`waypoints = []`) → comportamento depende do controlador;
  pode ser interpretado como "cancelar trajetória atual".

**Relação com o tipo `Waypoint4D`:**

```
Waypoint4DArray.waypoints[i]  =  Waypoint4D {
  geometry_msgs/Pose pose {
    Point    position    { x, y, z }
    Quaternion orientation { x, y, z, w }
  }
  float32 yaw
}
```

**Uso no código C++ para montar uma trajetória:**

```cpp
drone_control::msg::Waypoint4DArray traj;
traj.header.stamp    = this->now();
traj.header.frame_id = "map";

// Waypoint 1: ir para (2, 0, 1.5) apontando para Leste (yaw=0)
drone_control::msg::Waypoint4D wp1;
wp1.pose.position.x = 2.0;    // 2 m Leste
wp1.pose.position.y = 0.0;    // 0 m Norte
wp1.pose.position.z = 1.5;    // 1.5 m altitude
wp1.pose.orientation.w = 1.0; // quaternion identidade
wp1.yaw = 0.0f;               // 0 rad = Leste
traj.waypoints.push_back(wp1);

// Waypoint 2: ir para (2, 2, 1.5) apontando para Norte (yaw=π/2)
drone_control::msg::Waypoint4D wp2;
wp2.pose.position.x = 2.0;
wp2.pose.position.y = 2.0;    // 2 m Norte
wp2.pose.position.z = 1.5;
wp2.pose.orientation.w = 1.0;
wp2.yaw = static_cast<float>(M_PI / 2.0);  // 1.5708 rad = Norte
traj.waypoints.push_back(wp2);

// Waypoint 3: voltar à origem sem especificação de yaw
drone_control::msg::Waypoint4D wp3;
wp3.pose.position.x = 0.0;
wp3.pose.position.y = 0.0;
wp3.pose.position.z = 1.5;
wp3.pose.orientation.w = 1.0;
wp3.yaw = std::numeric_limits<float>::quiet_NaN();  // NaN = manter heading
traj.waypoints.push_back(wp3);

// traj agora contém 3 waypoints na sequência correta
pub_->publish(traj);
```

### Exemplo CLI completo

```bash
# Enviar trajetória de 2 waypoints com yaw explícito
ros2 topic pub /waypoints_4d drone_control/msg/Waypoint4DArray '{
  header: {frame_id: "map"},
  waypoints: [
    {pose: {position: {x: 2.0, y: 0.0, z: 1.5}, orientation: {w: 1.0}}, yaw: 0.0},
    {pose: {position: {x: 2.0, y: 2.0, z: 1.5}, orientation: {w: 1.0}}, yaw: 1.5708}
  ]
}' --once

# Monitorar trajetórias chegando
ros2 topic echo /waypoints_4d

# Inspecionar o tipo completo (mostra Waypoint4D aninhado)
ros2 interface show drone_control/msg/Waypoint4DArray
```

---

## 4. Diagrama de composição dos tipos

```
drone_control/Waypoint4DArray
├── std_msgs/Header header
│   ├── builtin_interfaces/Time stamp
│   │   ├── int32 sec          ← segundos
│   │   └── uint32 nanosec     ← nanosegundos
│   └── string frame_id        ← "map" (tipicamente)
└── drone_control/Waypoint4D[] waypoints
    └── [cada elemento]:
        ├── geometry_msgs/Pose pose
        │   ├── geometry_msgs/Point position
        │   │   ├── float64 x  ← Leste (ENU), metros
        │   │   ├── float64 y  ← Norte (ENU), metros
        │   │   └── float64 z  ← Cima (ENU), metros
        │   └── geometry_msgs/Quaternion orientation
        │       ├── float64 x  ← tipicamente 0
        │       ├── float64 y  ← tipicamente 0
        │       ├── float64 z  ← tipicamente 0
        │       └── float64 w  ← tipicamente 1 (identidade)
        └── float32 yaw        ← radianos; NaN = sem especificação

drone_control/YawOverride
├── bool    enable    ← true = ativar; false = desativar
├── float32 yaw_rate  ← rad/s (+ = CCW, − = CW)
└── float32 timeout   ← segundos; proteção contra travamentos
```
