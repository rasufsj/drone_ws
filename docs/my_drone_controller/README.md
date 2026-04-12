# Documentação: `my_drone_controller`

Bem-vindo à documentação do pacote `my_drone_controller`. Esta documentação está organizada em blocos temáticos para facilitar a leitura e a consulta.

## Índice

| Arquivo | Conteúdo |
|---------|----------|
| [00 — Inventário Completo de Arquivos](00-inventario-completo.md) | Lista completa de todos os arquivos de `my_drone_controller/src`, `include/` e `config/` com papel, funções, relação com a FSM e **o que cada bloco de código faz** |
| [01 — Visão Geral](01-visao-geral.md) | Objetivo do nó, arquitetura ROS 2/MAVROS/PX4, módulos, FSM 5 estados e fluxo de dados |
| [02 — Build e Execução](02-build-e-execucao.md) | Dependências, compilação via `colcon`, execução, parâmetros ROS 2 e tópicos cmd vs status |
| [03 — Setup (Parte 1)](03-drone_controller_completo-parte1-setup.md) | `includes`, namespace, construtor, `load_parameters`, `setup_publishers`, `setup_subscribers`, `setup_services`, `init_variables` |
| [04 — Loop e Setpoints (Parte 2)](04-drone_controller_completo-parte2-loop-e-setpoints.md) | Funções `publishPositionTarget*`, máscaras de tipo, watchdog, `publish_hold_setpoint` e `control_loop` |
| [05 — FSM Decolagem](05-fsm_takeoff.md) | `fsm_takeoff.cpp`: streaming pré-ARM, OFFBOARD separado do ARM, confirmação, `takeoff_target_z_` fixo |
| [06 — FSM Hover](06-fsm_hover.md) | `fsm_hover.cpp`: hover 3D/4D, detecção de pouso via `ExtendedState`, transição para estado 3 |
| [07 — FSM Trajetória](07-fsm_trajectory.md) | `fsm_trajectory.cpp`: planner codegen, cálculo de yaw, detecção de waypoint atingido, latch pose, finalização |
| [08 — FSM Pouso](08-fsm_landing.md) | `fsm_landing.cpp`: `complete_landing`, `handle_state4_disarm_reset`, timeout e transição para estado 0 |
| [09 — APIs e Exemplos de Uso](09-apis-e-exemplos-de-uso.md) | Como comandar o controlador via tópicos, exemplos C++ e observabilidade |
| [10 — Simulação e tmux](10-simulacao-e-tmux.md) | Como o `session.yml` do MRS UAV Gazebo Simulator foi modificado para integrar o `drone_node` |
| [11 — Subscribers e Callbacks](11-subscribers-e-callbacks.md) | Tabela completa de subscribers, detalhamento de cada callback, guards anti-echo e exemplos C++ de publicação |
| [12 — `main.cpp` e Estado 0](12-main-e-estado0.md) | Linha a linha: ponto de entrada do executável (`main.cpp`) e estado ocioso da FSM (`fsm_state0_wait.cpp`) |
| [13 — Validação de Waypoints](13-waypoint-validation.md) | Linha a linha: `waypoint_validation.cpp` — verificações de NaN/Inf, limites de altitude e distância XY |
| [14 — Fila de Comandos](14-command-queue.md) | Linha a linha: `command_queue.hpp` + `command_queue.cpp` — rastreabilidade, histórico, timeout e log de auditoria |
| [15 — Planner e Controlador Codegen](15-codegen-planner-e-controlador.md) | Linha a linha: `TrajectoryPlanner_codegen.cpp` (polinômio) + `Drone_codegen.cpp` (PID de posição) |
| [16 — Helpers Matemáticos Codegen](16-codegen-helpers-matematicos.md) | Bloco a bloco: `cos.cpp`, `sqrt.cpp`, `minOrMax.cpp` (saturação ±12 m/s), `mldivide.cpp` (pivotamento LU), `rt_nonfinite.cpp` (NaN/Inf) |
| [17 — Configuração YAML](17-configuracao-yaml.md) | Bloco a bloco: `config/drone_config.yaml` — Altitude, Validação, Takeoff safety, Timeouts; correspondência com `load_parameters()` |
| [18 — Headers e Contratos](18-headers-e-contratos.md) | Bloco a bloco: todos os headers em `include/` — `drone_controller_completo.hpp`, `drone_config.h`, headers MATLAB codegen (`cos.h`, `sqrt.h`, `minOrMax.h`, `mldivide.h`, `rt_nonfinite.h`, `rtwtypes.h`, `coder_array.h`) e árvore de dependências |

## Resumo rápido do pacote

```
my_drone_controller/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── drone_config.yaml          ← parâmetros ROS 2 (altitudes, timeouts, etc.)  → [17]
├── include/
│   ├── drone_config.h             ← struct DroneConfig (valores padrão)            → [18]
│   ├── TrajectoryPlanner_codegen.h ← planner MATLAB/codegen                        → [18]
│   ├── Drone_codegen.h            ← controlador de posição MATLAB/codegen           → [18]
│   ├── coder_array.h              ← template de array MATLAB/codegen                → [18]
│   ├── rtwtypes.h                 ← tipos base MATLAB Coder                         → [18]
│   ├── rt_nonfinite.h             ← NaN/Inf externados (codegen)                    → [18]
│   ├── cos.h                      ← wrapper coder::b_cos (codegen)                  → [18]
│   ├── sqrt.h                     ← wrapper coder::b_sqrt (codegen)                 → [18]
│   ├── minOrMax.h                 ← saturação de arrays (codegen)                   → [18]
│   ├── mldivide.h                 ← solver LU 6×6 (codegen)                         → [18]
│   ├── main_codegen.h             ← cabeçalho legado (referência ao codegen)        → [18]
│   └── my_drone_controller/
│       ├── drone_controller_completo.hpp  ← declaração completa da classe  → [18]
│       ├── command_queue.hpp              ← fila de comandos rastreável     → [18]
│       └── waypoint_validation.hpp        ← validação de waypoints          → [18]
└── src/
    ├── main.cpp                           ← ponto de entrada               → [12]
    ├── drone_controller_completo.cpp      ← setup, callbacks, loop         → [03][04][11]
    ├── fsm_state0_wait.cpp                ← Estado 0: aguardando           → [12]
    ├── fsm_takeoff.cpp                    ← Estado 1: decolagem            → [05]
    ├── fsm_hover.cpp                      ← Estado 2: hover                → [06]
    ├── fsm_trajectory.cpp                 ← Estado 3: trajetória           → [07]
    ├── fsm_landing.cpp                    ← Estado 4: pouso                → [08]
    ├── command_queue.cpp                                                    → [14]
    ├── waypoint_validation.cpp                                              → [13]
    ├── Drone_codegen.cpp                  ← controlador PID (MATLAB codegen)  → [15]
    ├── TrajectoryPlanner_codegen.cpp      ← planner polinomial (MATLAB codegen) → [15]
    ├── cos.cpp                            ← stub vazio (MATLAB codegen)        → [16]
    ├── minOrMax.cpp                       ← saturação ±12 m/s (MATLAB codegen) → [16]
    ├── mldivide.cpp                       ← solver LU (MATLAB codegen)         → [16]
    ├── rt_nonfinite.cpp                   ← constantes NaN/Inf (MATLAB codegen) → [16]
    └── sqrt.cpp                           ← stub vazio (MATLAB codegen)        → [16]
```

## Convenções adotadas nesta documentação

- Nomes de variáveis membros seguem o sufixo `_` (ex.: `state_voo_`, `config_`).
- Nomes de tópicos ROS 2 são prefixados com `/uav1/` quando específicos ao veículo.
- Exemplos de código C++ são autocontidos e compiláveis com as dependências listadas.
- Linguagem: **português**, estilo acadêmico.
