# 18 — Headers e Contratos — `include/`

> **Diretório fonte:** `my_drone_controller/include/`
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.

---

## Visão geral

O diretório `include/` contém os **contratos de API** (declarações públicas) de
todas as classes e funções do pacote, além dos cabeçalhos gerados automaticamente
pelo MATLAB Coder. A tabela abaixo resume os headers e quem os inclui:

| Header | Quem inclui | Papel |
|--------|-------------|-------|
| `my_drone_controller/drone_controller_completo.hpp` | `main.cpp`, todos os `fsm_*.cpp`, `drone_controller_completo.cpp` | Classe principal do nó ROS 2 |
| `my_drone_controller/command_queue.hpp` | `drone_controller_completo.hpp`, `command_queue.cpp` | API de fila de auditoria |
| `my_drone_controller/waypoint_validation.hpp` | `drone_controller_completo.cpp`, `waypoint_validation.cpp` | Funções de validação de waypoints |
| `drone_config.h` | `drone_controller_completo.hpp`, `waypoint_validation.hpp` | Struct de configuração |
| `Drone_codegen.h` | `drone_controller_completo.hpp`, `Drone_codegen.cpp` | Controlador PID MATLAB codegen |
| `TrajectoryPlanner_codegen.h` | `drone_controller_completo.hpp`, `TrajectoryPlanner_codegen.cpp` | Planner polinomial MATLAB codegen |
| `rtwtypes.h` | todos os headers codegen | Tipos base MATLAB Coder |
| `rt_nonfinite.h` | `Drone_codegen.cpp`, `TrajectoryPlanner_codegen.cpp` | Constantes NaN/Inf |
| `cos.h` | `Drone_codegen.cpp` | Wrapper trigonométrico codegen |
| `sqrt.h` | `Drone_codegen.cpp` | Wrapper sqrt codegen |
| `minOrMax.h` | `Drone_codegen.cpp` | Saturação de arrays codegen |
| `mldivide.h` | `TrajectoryPlanner_codegen.cpp` | Solver linear codegen |
| `coder_array.h` | `TrajectoryPlanner_codegen.h`, `TrajectoryPlanner_codegen.cpp` | Array dinâmico codegen |
| `main_codegen.h` | `drone_controller_completo.cpp` (histórico) | Cabeçalho legado (referência ao codegen combinado) |

---

## 1. `drone_controller_completo.hpp` — API do nó ROS 2

### Papel / Responsabilidade

Declaração completa da classe `DroneControllerCompleto`, que herda de
`rclcpp::Node`. Este header é a **fronteira de API** do pacote: tudo o que os
arquivos `fsm_*.cpp` precisam acessar está declarado aqui como membro privado
(acessível via `friend` implícito por método, pois as funções FSM são métodos da
mesma classe).

### Bloco 1 — Includes do header

```cpp
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/msg/extended_state.hpp"
#include "mavros_msgs/msg/position_target.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"
#include "drone_control/msg/yaw_override.hpp"
#include "drone_control/msg/waypoint4_d.hpp"
#include "drone_control/msg/waypoint4_d_array.hpp"
#include "drone_config.h"
#include "my_drone_controller/command_queue.hpp"
#include "my_drone_controller/waypoint_validation.hpp"
#include "TrajectoryPlanner_codegen.h"
#include "Drone_codegen.h"
```

> Este bloco de includes expõe as **dependências completas** do nó: mensagens ROS 2
> padrão (`geometry_msgs`, `nav_msgs`, `std_msgs`), serviços e mensagens MAVROS
> (`mavros_msgs`), mensagens customizadas do pacote `drone_control`, a struct de
> configuração (`drone_config.h`), os subsistemas internos (`command_queue.hpp`,
> `waypoint_validation.hpp`) e os codegen do MATLAB (`TrajectoryPlanner_codegen.h`,
> `Drone_codegen.h`).

### Bloco 2 — Constantes de máscara `type_mask`

```cpp
// Bits: 1 = ignorar aquele campo no PositionTarget
static constexpr uint16_t IGNORE_VX       = (1 << 3);
static constexpr uint16_t IGNORE_VY       = (1 << 4);
static constexpr uint16_t IGNORE_VZ       = (1 << 5);
static constexpr uint16_t IGNORE_AFX      = (1 << 6);
static constexpr uint16_t IGNORE_AFY      = (1 << 7);
static constexpr uint16_t IGNORE_AFZ      = (1 << 8);
static constexpr uint16_t IGNORE_YAW      = (1 << 10);
static constexpr uint16_t IGNORE_YAW_RATE = (1 << 11);

// Máscara composta: posição + yaw_rate (ignora vel, acel, yaw fixo)
static constexpr uint16_t MASK_POS_YAWRATE = …;
// Máscara composta: posição + yaw fixo
static constexpr uint16_t MASK_POS_YAW = …;
// Apenas posição
static constexpr uint16_t MASK_POS_ONLY = …;
// Posição + velocidade + yaw fixo
static constexpr uint16_t MASK_POS_VEL_YAW = …;
```

> Cada constante `IGNORE_*` desativa um campo do `mavros_msgs::msg::PositionTarget`
> enviado ao PX4. As máscaras compostas (`MASK_*`) combinam os bits conforme o
> modo de controle: posição pura, posição+velocidade, ou posição+yaw_rate. Consulte
> **[04-drone_controller_completo-parte2-loop-e-setpoints.md](04-drone_controller_completo-parte2-loop-e-setpoints.md)**
> para exemplos de uso.

### Bloco 3 — Constantes de temporização

```cpp
/// Frequência mínima garantida de publicação de setpoints (watchdog).
static constexpr double MIN_SETPOINT_RATE_HZ = 20.0;
/// Silêncio máximo entre setpoints: 1/20 Hz = 50 ms.
static constexpr double MAX_SETPOINT_SILENCE_S = 1.0 / MIN_SETPOINT_RATE_HZ;

/// Setpoints mínimos pré-ARM antes de solicitar OFFBOARD.
/// A 100 Hz → ~200 ms de streaming contínuo.
static constexpr int INITIAL_STREAM_THRESHOLD = 20;

/// Setpoints após confirmar OFFBOARD, antes de enviar ARM.
/// A 100 Hz → 1,5 s de streaming (recomendação PX4/MAVROS).
static constexpr int POST_OFFBOARD_STREAM_THRESHOLD = 150;
```

> O PX4 rejeita ARM em OFFBOARD se não houver setpoints contínuos. `INITIAL_STREAM_THRESHOLD`
> e `POST_OFFBOARD_STREAM_THRESHOLD` implementam essa exigência. O watchdog
> (`MIN_SETPOINT_RATE_HZ`) garante que o drone nunca fique mais de 50 ms sem receber
> um setpoint enquanto em voo.

### Bloco 4 — Membros de estado da FSM

```cpp
int state_voo_;          // 0=espera, 1=decolagem, 2=hover, 3=trajetória, 4=pouso
bool controlador_ativo_;
bool pouso_em_andamento_;
bool offboard_activated_;
bool offboard_mode_confirmed_;  // FCU confirmou OFFBOARD; ARM só é enviado depois
bool arm_requested_;
bool activation_confirmed_;
rclcpp::Time activation_time_;
int cycle_count_;
int takeoff_counter_;
```

> Estes membros são o **estado interno da FSM**. `state_voo_` é o índice do estado
> atual. Os booleanos controlam subestados dentro de cada fase (ex.:
> `offboard_mode_confirmed_` garante que ARM só seja enviado após o PX4 confirmar
> OFFBOARD, evitando a sequência incorreta ARM→OFFBOARD).

### Bloco 5 — Membros de trajetória e odometria

```cpp
// Odometria (NED)
double current_z_real_, current_x_ned_, current_y_ned_, current_z_ned_;
double current_vx_ned_, current_vy_ned_, current_vz_ned_;

// Trajetória
std::vector<geometry_msgs::msg::Pose> trajectory_waypoints_;
rclcpp::Time trajectory_start_time_;
bool trajectory_started_;
int current_waypoint_idx_;

// Yaw
double current_yaw_rad_, goal_yaw_rad_;
bool using_4d_goal_, trajectory_4d_mode_;
std::vector<double> trajectory_yaws_;

// Codegen
TrajectoryPlanner_codegen planner_;
Drone_codegen drone_ctrl_;
bool planner_initialized_;
```

> `planner_` e `drone_ctrl_` são os objetos dos módulos MATLAB codegen instanciados
> como membros. `trajectory_waypoints_` armazena a sequência de poses a percorrer;
> `current_waypoint_idx_` aponta para o waypoint ativo.

### Bloco 6 — Guards anti-echo

```cpp
int skip_self_waypoint_goal_count_{0};
int skip_self_waypoints_count_{0};
```

> O nó republica os próprios tópicos para monitoramento. Para não processar essas
> mensagens como novos comandos, esses contadores incrementam antes de publicar
> e decrementam no início da callback. Veja detalhes em
> **[11-subscribers-e-callbacks.md](11-subscribers-e-callbacks.md)**.

---

## 2. `command_queue.hpp` — API da Fila de Auditoria

### Papel / Responsabilidade

Define os tipos `CommandType` (enum), `CommandStatus` (enum), `Command` (struct)
e a classe `CommandQueue`. Toda a lógica de rastreabilidade de comandos é
**contratada** aqui; a implementação está em `command_queue.cpp`.

### Bloco 1 — Tipos e struct

```cpp
enum class CommandType {
  ARM, DISARM, SET_MODE_OFFBOARD,
  TAKEOFF, HOVER, TRAJECTORY, LAND
};

enum class CommandStatus {
  PENDING,    // enviado, aguardando confirmação do FCU
  CONFIRMED,  // FCU confirmou execução
  FAILED,     // FCU reportou falha
  TIMEOUT     // sem confirmação dentro do timeout
};

struct Command {
  uint64_t id{0};
  CommandType type{CommandType::ARM};
  CommandStatus status{CommandStatus::PENDING};
  std::chrono::system_clock::time_point timestamp{};
  std::chrono::system_clock::time_point confirm_time{};
  std::map<std::string, std::string> data;

  std::string type_str() const;    // "ARM", "DISARM", ...
  std::string status_str() const;  // "PENDENTE", "CONFIRMADO", ...
};
```

> Cada `Command` carrega um `id` único, o tipo, o status atual e os timestamps de
> envio e confirmação. O campo `data` é um mapa livre para metadados adicionais
> (ex.: `"z_target" → "2.00"`).

### Bloco 2 — Classe `CommandQueue`

```cpp
class CommandQueue {
public:
  uint64_t enqueue(CommandType type,
                   const std::map<std::string, std::string> & data = {});
  bool confirm(uint64_t id, bool success = true);
  std::vector<uint64_t> check_timeouts(double timeout_seconds);
  std::vector<Command> get_history() const;
  size_t pending_count() const;
  void save_log(const std::string & filename) const;
  void cancel_all_pending();
private:
  mutable std::mutex mutex_;
  uint64_t next_id_{1};
  std::map<uint64_t, Command> pending_;
  std::vector<Command> history_;
};
```

> - `enqueue()`: adiciona à fila `pending_` e retorna o `id`.
> - `confirm()`: move da fila `pending_` para `history_` com status CONFIRMED/FAILED.
> - `check_timeouts()`: itera `pending_` e move comandos expirados para `history_` com TIMEOUT.
> - `save_log()`: persiste `history_` em arquivo estruturado.
> - `cancel_all_pending()`: marca todos os `pending_` como FAILED e os move para `history_` — usado entre ciclos de voo para limpar comandos obsoletos.

Consulte a documentação completa em
**[14-command-queue.md](14-command-queue.md)**.

---

## 3. `waypoint_validation.hpp` — API de Validação de Waypoints

### Papel / Responsabilidade

Declara dois funções livres no namespace `drone_control` que aplicam as
restrições de `DroneConfig` sobre waypoints recebidos.

### Bloco único — Declarações

```cpp
namespace drone_control {

/**
 * Valida um PoseStamped contra os limites físicos.
 * Verifica NaN/Inf e aplica os limites de distância XY e altitude de config.
 * @return true se o waypoint é seguro.
 */
bool validate_waypoint(const geometry_msgs::msg::PoseStamped & msg,
                       const DroneConfig & config);

/**
 * Wrapper para um Pose simples (sem header).
 * @return true se a pose é segura.
 */
bool validate_pose(const geometry_msgs::msg::Pose & pose,
                   const DroneConfig & config);

}  // namespace drone_control
```

> `validate_waypoint` é chamada nas callbacks de `/waypoint_goal` e `/waypoints`
> antes de qualquer processamento. `validate_pose` é chamada para cada elemento
> de `PoseArray` ao receber trajetórias. Consulte a documentação detalhada em
> **[13-waypoint-validation.md](13-waypoint-validation.md)**.

---

## 4. `drone_config.h` — Struct de Configuração

### Papel / Responsabilidade

Define `struct DroneConfig` com todos os parâmetros do controlador e seus valores
padrão *hard-coded* como inicializadores de membro C++. Em runtime,
`load_parameters()` sobrescreve esses valores com os do YAML (ou linha de comando).

### Bloco único — Struct

```cpp
namespace drone_control {

struct DroneConfig {
  // ── Altitude ──────────────────────────────────────────────────────────
  double hover_altitude{2.0};          ///< Altitude-alvo de hover [m]
  double hover_altitude_margin{0.05};  ///< Histerese para detecção de chegada [m]
  double max_altitude{500.0};          ///< Teto máximo para waypoints [m]
  double waypoint_duration{4.0};       ///< Duração de cada waypoint na trajetória [s]

  // ── Limites de segurança ───────────────────────────────────────────────
  double min_altitude{0.2};            ///< Altitude mínima de voo [m]
  double land_z_threshold{0.1};        ///< Z abaixo do qual = pousado [m]

  // ── Validação ─────────────────────────────────────────────────────────
  double max_waypoint_distance{100.0}; ///< Distância máxima XY aceita [m]

  // ── Takeoff boost ─────────────────────────────────────────────────────
  double takeoff_z_boost{0.7};

  // ── Takeoff XY sanitization ───────────────────────────────────────────
  double takeoff_xy_origin_threshold_m{0.2};
  double latch_pose_max_age_s{10.0};

  // ── Timeouts ──────────────────────────────────────────────────────────
  double activation_timeout{5.0};      // nota: YAML usa 3.0
  double command_timeout{15.0};
  double landing_timeout{3.0};
  double offboard_confirm_timeout{5.0};
};

}  // namespace drone_control
```

> **Observação:** `activation_timeout` tem default `5.0` na struct mas `3.0` no
> YAML. Quando o arquivo é carregado, o YAML prevalece. Consulte
> **[17-configuracao-yaml.md](17-configuracao-yaml.md)** para a tabela completa
> de parâmetros e sua correspondência com `load_parameters()`.

---

## 5. `Drone_codegen.h` e `TrajectoryPlanner_codegen.h`

### Papel / Responsabilidade

Cabeçalhos das classes geradas pelo MATLAB Coder (versão 23.2, gerados em
10-Mar-2026). Declaram a API que `drone_controller_completo.hpp` instancia como
membros `planner_` e `drone_ctrl_`.

### `Drone_codegen.h` — Controlador PID

```cpp
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

class Drone_codegen {
public:
  // ── API pública ────────────────────────────────────────────────────
  Drone_codegen *PositionCtrl(const double Xd[3],   // posição desejada
                               const double Vd[3],   // velocidade desejada
                               const double Ad[3]);  // aceleração desejada
  Drone_codegen *init();

  // ── Entradas (preenchidas pelo nó ROS 2) ──────────────────────────
  double r[3];      // posição real: x, y, z (NED)
  double dr[3];     // velocidade real: vx, vy, vz (NED)
  double euler[3];  // atitude: roll, pitch, yaw
  double w[3];      // velocidade angular: p, q, r

  // ── Parâmetros físicos ────────────────────────────────────────────
  double g;   // gravidade [m/s²]
  double m;   // massa [kg]
  double dt;  // passo de integração [s]

  // ── Saídas (lidas pelo nó ROS 2) ─────────────────────────────────
  double phi_des;    // roll desejado [rad]
  double theta_des;  // pitch desejado [rad]
  double psi_des;    // yaw desejado [rad]
  double zdot_des;   // velocidade vertical desejada [m/s]

  // ── Memória dos integradores/derivadores ─────────────────────────
  double r_err_sum[3];
  double phi_err_sum, theta_err_sum, psi_err_sum;
  double zdot_err_sum, zdot_err_prev;

  // ── Ganhos PID ────────────────────────────────────────────────────
  double kP_pos[3], kI_pos[3], kD_pos[3];
  double kP_phi, kD_phi;
  double kP_theta, kD_theta;
  double kP_zdot, kI_zdot, kD_zdot;

  double r_des[3];    // posição de referência interna
  double dr_des[3];   // velocidade de referência interna
  double r_err[3];    // erro de posição

private:
  Drone_codegen *handle_init();
};
```

> - `init()`: inicializa ganhos PID, memória dos integradores e parâmetros físicos.
> - `PositionCtrl(Xd, Vd, Ad)`: executa um ciclo PID com os estados reais lidos de
>   `r[]`/`dr[]`/`euler[]`/`w[]` e os setpoints desejados `(Xd, Vd, Ad)`.
>   Preenche `zdot_des` como saída principal usada pelo nó para montar o
>   `PositionTarget` enviado ao PX4.

### `TrajectoryPlanner_codegen.h` — Planner Polinomial

```cpp
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>
#include <cmath>
#include <vector>

class TrajectoryPlanner_codegen {
public:
  TrajectoryPlanner_codegen *init();

  // Retorna o setpoint interpolado no instante t_atual
  void getNextSetpoint(double t_atual,
                       double Xd[3],   // posição desejada [out]
                       double Vd[3],   // velocidade desejada [out]
                       double Ad[3]);  // aceleração desejada [out]

  std::vector<double> waypoints;       // lista plana [x0,y0,z0, x1,y1,z1, ...]
  std::vector<double> segmentTimes;    // duração de cada segmento [s]

  coder::array<double, 3U> coefficients; // coeficientes polinomiais 3D (saída de mldivide)
  int numSegments;
  double X_final[3];       // posição do último waypoint
  double hoverTime[4];
  boolean_T inHover;
  double hoverStartReal;
  double currentSegment;
  double tAccum;
};
```

> - `init()`: recebe `waypoints` e `segmentTimes` já preenchidos e chama
>   `mldivide` para resolver os coeficientes polinomiais de cada segmento.
> - `getNextSetpoint(t_atual, Xd, Vd, Ad)`: avalia o polinômio no instante
>   `t_atual` para o segmento corrente e preenche `Xd`, `Vd`, `Ad`.

Consulte a documentação detalhada em
**[15-codegen-planner-e-controlador.md](15-codegen-planner-e-controlador.md)**.

---

## 6. `rtwtypes.h` — Tipos Base MATLAB Coder

### Papel / Responsabilidade

Define os tipos com largura fixa utilizados em **todo** o código gerado pelo MATLAB
Coder 23.2, mapeando-os para os equivalentes C++ padrão. Garante portabilidade
entre x86-64 (Linux/ROS 2) e ARM.

### Bloco único — Definições de tipo

```cpp
typedef double          real_T;      // double IEEE 754 64 bits
typedef float           real32_T;    // float  IEEE 754 32 bits
typedef double          real64_T;    // alias de real_T
typedef unsigned char   boolean_T;   // booleano (0/1)
typedef int             int32_T;
typedef unsigned int    uint32_T;
typedef signed char     int8_T;
typedef unsigned char   uint8_T;
typedef short           int16_T;
typedef unsigned short  uint16_T;
typedef char            char_T;
typedef int             int_T;
typedef unsigned int    uint_T;

// 64 bits nativos Linux/x64
typedef long            int64_T;
typedef unsigned long   uint64_T;

// Limites
#define MAX_int32_T  ((int32_T)(2147483647))
#define MIN_int32_T  ((int32_T)(-2147483647-1))
// ...

#ifndef FALSE
#define FALSE (0U)
#endif
#ifndef TRUE
#define TRUE  (1U)
#endif
```

> `real_T` é o tipo numérico predominante: todos os vetores de estado
> (`r[]`, `dr[]`, etc.), coeficientes do planner e saídas do PID são `real_T`.
> `boolean_T` é usado para flags internas do codegen (ex.: `inHover`).

---

## 7. Headers matemáticos codegen: `cos.h`, `sqrt.h`, `minOrMax.h`, `mldivide.h`, `rt_nonfinite.h`

Estes cinco headers são **gerados automaticamente** pelo MATLAB Coder 23.2 como
wrappers das funções matemáticas usadas internamente pelos algoritmos codegen.
Todos incluem `rtwtypes.h` e são incluídos por `Drone_codegen.cpp` e/ou
`TrajectoryPlanner_codegen.cpp`.

### `rt_nonfinite.h` — Constantes globais NaN e Inf

```cpp
#include "rtwtypes.h"

#ifdef __cplusplus
extern "C" {
#endif

extern real_T    rtInf;
extern real_T    rtMinusInf;
extern real_T    rtNaN;
extern real32_T  rtInfF;
extern real32_T  rtMinusInfF;
extern real32_T  rtNaNF;

#ifdef __cplusplus
}
#endif
```

> Declara seis variáveis globais *extern*: três `real_T` (double) e três `real32_T`
> (float). O bloco `extern "C"` garante ligação C para compatibilidade com código
> C puro que o MATLAB Coder pode gerar. A inicialização está em `rt_nonfinite.cpp`.
>
> **Quem inclui:** `Drone_codegen.cpp` e `TrajectoryPlanner_codegen.cpp`.

### `cos.h` — Wrapper de cosseno

```cpp
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

namespace coder {
  void b_cos(double &x);  // x ← cos(x) in-place
}
```

> Declara `coder::b_cos(double &x)`, que modifica `x` *in-place* para `cos(x)`.
> O prefixo `b_` é uma convenção do MATLAB Coder para diferenciar wrappers de
> funções padrão.
>
> **Quem inclui:** `Drone_codegen.cpp` (para cálculos de roll/pitch desejados).

### `sqrt.h` — Wrapper de raiz quadrada

```cpp
namespace coder {
  void b_sqrt(double &x);  // x ← sqrt(x) in-place
}
```

> Semanticamente idêntico ao `cos.h`: `coder::b_sqrt(double &x)` calcula `sqrt(x)`
> in-place. O arquivo `sqrt.cpp` correspondente está vazio no repositório atual —
> o compilador resolve via `<cmath>` diretamente.
>
> **Quem inclui:** `Drone_codegen.cpp`.

### `minOrMax.h` — Saturação de arrays

```cpp
namespace coder {
  // Mínimo elemento a elemento entre array e escalar
  void minimum(const double x[3], double ex, double ex_data[], int ex_size[1]);
  // Máximo elemento a elemento entre array e escalar
  void maximum(const double x[3], double ex, double ex_data[], int ex_size[1]);
}
```

> Implementa saturação vetorial: `minimum(x, -12.0, ...)` limita cada componente
> de `x` a no mínimo −12 m/s; `maximum(x, 12.0, ...)` limita a no máximo 12 m/s.
> Usada pelo controlador PID para saturar velocidades de saída.
>
> **Quem inclui:** `Drone_codegen.cpp`.

### `mldivide.h` — Solver de sistema linear

```cpp
namespace coder {
  // Resolve A·x = B pelo método de eliminação gaussiana com pivotamento parcial
  // A: matriz 6×6 (armazenada linha a linha, 36 doubles)
  // B: vetor 6 (entrada e saída — sobrescrito com a solução x)
  void mldivide(const double A[36], double B[6]);
}
```

> Implementa o operador `A\B` do MATLAB para sistemas 6×6. É chamada por
> `TrajectoryPlanner_codegen` durante `init()` para resolver os coeficientes
> polinomiais de cada segmento da trajetória. O algoritmo usa eliminação
> gaussiana com pivotamento parcial (ver `mldivide.cpp`).
>
> **Quem inclui:** `TrajectoryPlanner_codegen.cpp`.

---

## 8. `coder_array.h` — Template de Array Codegen

### Papel / Responsabilidade

Template C++ gerado pelo MATLAB Coder que implementa o tipo `coder::array<T, N>`,
um array dinâmico N-dimensional compatível com os arrays do MATLAB. Usado pelo
`TrajectoryPlanner_codegen` para armazenar a matriz tridimensional de coeficientes
polinomiais (`coefficients`).

### Bloco representativo — Interface pública

```cpp
namespace coder {

template <typename T, int32_T N>
class array {
public:
  array();
  ~array();

  void set_size(...)     // redefine dimensões
  int32_T size(int32_T)  // retorna tamanho da dimensão i
  T &operator[](int32_T) // acesso por índice linear

  T * data();            // ponteiro bruto para os dados
  int32_T numel() const; // número total de elementos

  // ... operadores de cópia, atribuição
};

}  // namespace coder
```

> - `N` é o número de dimensões (ex.: `coder::array<double, 3U>` é um array 3D).
> - Internamente usa alocação dinâmica (`new`/`delete`) gerenciada pelo próprio
>   template — sem dependência de `std::vector` ou contêineres STL.
> - No `TrajectoryPlanner_codegen`, `coefficients` é um `coder::array<double, 3U>`
>   com dimensões `[6, 3, numSegments]` (6 coeficientes por eixo por segmento).

---

## Árvore de dependências entre headers

```
drone_controller_completo.hpp
├── rclcpp/rclcpp.hpp
├── geometry_msgs/…, nav_msgs/…, mavros_msgs/…, std_msgs/…
├── drone_control/msg/…
├── drone_config.h
│   └── (sem dependências internas)
├── my_drone_controller/command_queue.hpp
│   └── <mutex>, <map>, <vector>, <string>, …
├── my_drone_controller/waypoint_validation.hpp
│   ├── geometry_msgs/msg/pose.hpp
│   ├── geometry_msgs/msg/pose_stamped.hpp
│   └── drone_config.h
├── TrajectoryPlanner_codegen.h
│   ├── rtwtypes.h
│   └── coder_array.h
└── Drone_codegen.h
    └── rtwtypes.h

Drone_codegen.cpp inclui:
  ├── Drone_codegen.h
  ├── rt_nonfinite.h  → rtwtypes.h
  ├── cos.h           → rtwtypes.h
  ├── sqrt.h          → rtwtypes.h
  └── minOrMax.h      → rtwtypes.h

TrajectoryPlanner_codegen.cpp inclui:
  ├── TrajectoryPlanner_codegen.h
  ├── rt_nonfinite.h  → rtwtypes.h
  └── mldivide.h      → rtwtypes.h
```
