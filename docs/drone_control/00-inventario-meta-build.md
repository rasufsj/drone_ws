# Inventário Meta/Build — `drone_control`

> **Objetivo:** listar e descrever os arquivos de metadados e configuração de build
> do pacote `drone_control` — `README.md`, `package.xml`, `CMakeLists.txt` e `LICENSE`.
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.
> Última sincronização: branch `main`, 2026-04-13.

---

## Índice

| # | Arquivo | Categoria | Doc detalhada |
|---|---------|-----------|---------------|
| 1 | [`README.md`](#1-readmemd) | Documentação | — |
| 2 | [`package.xml`](#2-packagexml) | Build/meta | [11-build-e-interfaces.md](11-build-e-interfaces.md) |
| 3 | [`CMakeLists.txt`](#3-cmakeliststxt) | Build | [11-build-e-interfaces.md](11-build-e-interfaces.md) |
| 4 | [`LICENSE`](#4-license) | Meta | — |

---

## 1. `README.md`

### Papel / Responsabilidade

Documentação de usuário do pacote `drone_control`, incluída na raiz do pacote.
Serve como referência rápida para desenvolvedores e operadores do sistema.

### Conteúdo

- Descrição do pacote e lista de nós disponíveis.
- Parâmetros de cada nó em formato de tabela.
- Comportamento detalhado do `supervisor_T` (estados, sinais de trajetória, logs esperados).
- Fluxo típico de operação com comandos de terminal.
- Testes de validação manual.

### Relação com o restante do pacote

Referencia todos os nós em `src/` e descreve os tópicos ROS 2 que os conectam
ao `my_drone_controller` (especialmente `/trajectory_finished` e `/trajectory_progress`).

---

## 2. `package.xml`

### Papel / Responsabilidade

Manifesto ROS 2 do pacote no formato 3. Define nome, versão, licença, mantenedor
e todas as dependências de build/execução necessárias para compilar e rodar os
nós em `src/` e gerar as mensagens em `msg/`.

### Bloco: Identificação

```xml
<name>drone_control</name>
<version>0.0.0</version>
<maintainer email="rasufsj@gmail.com">lmnr31</maintainer>
<license>Apache-2.0</license>
```

Identifica o pacote com nome `drone_control`, versão de desenvolvimento (`0.0.0`)
e licença Apache 2.0 — a mesma adotada pelo ecossistema ROS 2 / ament.

### Bloco: Ferramentas de build

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
```

- `ament_cmake` — sistema de build CMake do ROS 2 (substitui `catkin`).
- `rosidl_default_generators` — gera código C++/Python a partir dos arquivos `.msg`
  em `msg/` (necessário para `YawOverride`, `Waypoint4D`, `Waypoint4DArray`).

### Bloco: Dependências de execução (interfaces)

```xml
<depend>rosidl_default_runtime</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

- `rosidl_default_runtime` — runtime das interfaces geradas; garante que as
  bibliotecas de suporte estejam disponíveis ao executar os nós.
- `member_of_group` — registra o pacote como fornecedor de interfaces, permitindo
  que outros pacotes declarem `<depend>drone_control</depend>` para consumir os `.msg`.

### Bloco: Dependências de código

```xml
<depend>rclcpp</depend>
<depend>rclpy</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>
<depend>rcl_interfaces</depend>
<depend>sensor_msgs</depend>
<depend>mrs_msgs</depend>
<depend>nav_msgs</depend>
<depend>mavros_msgs</depend>
<depend>tf2</depend>
<depend>cv_bridge</depend>
<depend>image_transport</depend>
<depend>opencv</depend>
```

| Dependência | Usada por |
|------------|-----------|
| `rclcpp` | todos os nós C++ |
| `rclpy` | launch files Python |
| `geometry_msgs` | `takeoff`, `pouso` (`PoseArray`, `Pose`, `PointStamped`) |
| `std_msgs` | `supervisor_T` (`Bool`, `Float32`) |
| `sensor_msgs` | `camera_viewer` (`Image`) |
| `nav_msgs` | `takeoff`, `pouso`, `drone_yaw_360`, `supervisor_T` (`Odometry`) |
| `mavros_msgs` | `takeoff`, `pouso` (`State`, `SetMode`) |
| `mrs_msgs` | reservado para integração com MRS UAV System |
| `tf2` | `drone_yaw_360` (transformações de orientação via Eigen/quaternion) |
| `cv_bridge` | `camera_viewer` (conversão ROS Image ↔ cv::Mat) |
| `image_transport` | `camera_viewer` (transporte eficiente de imagens) |
| `opencv` | `camera_viewer` (renderização OpenCV) |

### Bloco: Dependências de execução dos launches

```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

Necessário para que os arquivos `launch/*.launch.py` sejam descobertos e
executados via `ros2 launch`.

---

## 3. `CMakeLists.txt`

### Papel / Responsabilidade

Script de build CMake que:
1. Gera código C++ a partir dos arquivos `.msg` customizados.
2. Compila os seis executáveis (targets) de `src/`.
3. Instala targets e arquivos de launch no `share/`.

Doc detalhada por blocos: [11-build-e-interfaces.md](11-build-e-interfaces.md).

### Bloco: Cabeçalho e flags

```cmake
cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
```

Requer CMake ≥ 3.8 (mínimo do ROS 2 Humble) e ativa warnings máximos para
detectar erros de compilação antecipadamente.

### Bloco: Geração de mensagens

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YawOverride.msg"
  "msg/Waypoint4D.msg"
  "msg/Waypoint4DArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)
ament_export_dependencies(rosidl_default_runtime)
```

Invoca o gerador `rosidl` para criar cabeçalhos C++ como
`drone_control/msg/yaw_override.hpp`, utilizáveis via
`#include "drone_control/msg/yaw_override.hpp"`.

### Bloco: Targets de executáveis

Cada `add_executable` + `ament_target_dependencies` + `install` define um binário:

| Target | Arquivo fonte | Deps extras |
|--------|--------------|-------------|
| `drone_yaw_360` | `src/drone_yaw_360.cpp` | `nav_msgs`, `mavros_msgs`, `Eigen3` + `rosidl_target_interfaces` |
| `takeoff` | `src/takeoff.cpp` | `geometry_msgs`, `nav_msgs`, `mavros_msgs` |
| `pouso` | `src/pouso.cpp` | `geometry_msgs`, `nav_msgs`, `mavros_msgs` |
| `missao_P_T` | `src/missao_P_T.cpp` | `rclcpp` apenas |
| `camera_viewer` | `src/camera_viewer.cpp` | `sensor_msgs`, `image_transport`, `cv_bridge`, `${OpenCV_LIBRARIES}` |
| `supervisor_T` | `src/supervisor_T.cpp` | `rclcpp`, `std_msgs`, `nav_msgs` |

O target `drone_yaw_360` requer chamada extra `rosidl_target_interfaces` para
acessar o tipo `drone_control/msg/YawOverride` gerado.

### Bloco: Instalação dos launches

```cmake
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

Copia toda a pasta `launch/` para `share/drone_control/launch/`, tornando os
arquivos descobríveis via `ros2 launch drone_control <launch_file>`.

---

## 4. `LICENSE`

### Papel / Responsabilidade

Texto completo da licença **Apache License, Version 2.0** (janeiro 2004).
Define os termos de uso, reprodução e distribuição do pacote.

### Pontos relevantes

- **Uso comercial e modificação** permitidos, desde que se mantenha a atribuição.
- **Sem garantias** de qualquer tipo (seção 7 — Disclaimer of Warranty).
- Compatible com as licenças padrão do ecossistema ROS 2 e ament.

### Relação com o pacote

O `package.xml` declara `<license>Apache-2.0</license>`, que corresponde a este arquivo.
