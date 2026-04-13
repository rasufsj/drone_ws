# Build e Interfaces — `drone_control`

> **Objetivo:** explicar por blocos o `package.xml` e o `CMakeLists.txt` do
> pacote `drone_control` — dependências, targets de compilação, geração das
> interfaces `.msg` e instalação dos artefatos.
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.
> Última sincronização: branch `main`, 2026-04-13.

---

## 1. `package.xml` — Manifesto do pacote

### Bloco 1 — Cabeçalho XML e identificação

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd"
            schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>drone_control</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="rasufsj@gmail.com">lmnr31</maintainer>
  <license>Apache-2.0</license>
```

**O que este bloco faz:**

- `format="3"` — usa o schema mais recente do `package.xml` (ROS 2 Humble+).
- `<name>drone_control</name>` — nome do pacote; deve corresponder ao `project()`
  no `CMakeLists.txt` e ao diretório do pacote.
- `<version>0.0.0</version>` — versão de desenvolvimento; atualizar para semver
  antes de publicação.
- `<license>Apache-2.0</license>` — compatível com o ecossistema ROS 2 / ament.

### Bloco 2 — Ferramentas de build

```xml
<buildtool_depend>ament_cmake</buildtool_depend>
<buildtool_depend>rosidl_default_generators</buildtool_depend>
```

**O que este bloco faz:**

- `ament_cmake` — sistema de build padrão do ROS 2 que estende o CMake com
  macros `ament_*` para instalação, export e geração de targets.
- `rosidl_default_generators` — invocado pelo CMake para transformar os
  arquivos `.msg` em código-fonte C++ e Python durante o processo de build.
  **Obrigatório** sempre que o pacote define mensagens customizadas.

### Bloco 3 — Runtime das interfaces geradas

```xml
<depend>rosidl_default_runtime</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

**O que este bloco faz:**

- `rosidl_default_runtime` — dependência de execução das bibliotecas de
  serialização/deserialização geradas (`typesupport`). Garante que outros
  pacotes que importem `drone_control` também carreguem o runtime das mensagens.
- `member_of_group` — registra o pacote no grupo de pacotes de interface, o que
  permite que `<depend>drone_control</depend>` em outros `package.xml` inclua
  automaticamente os tipos de mensagem exportados.

### Bloco 4 — Dependências de código C++

```xml
<depend>rclcpp</depend>
<depend>geometry_msgs</depend>
<depend>std_msgs</depend>
<depend>sensor_msgs</depend>
<depend>nav_msgs</depend>
<depend>mavros_msgs</depend>
<depend>mrs_msgs</depend>
<depend>tf2</depend>
<depend>cv_bridge</depend>
<depend>image_transport</depend>
<depend>opencv</depend>
```

**Mapeamento dependência → nó:**

| Dependência | Nó(s) consumidor(es) | Por quê |
|------------|---------------------|---------|
| `rclcpp` | todos | runtime C++ do ROS 2 |
| `geometry_msgs` | `takeoff`, `pouso` | `PoseArray`, `Pose`, `PointStamped` |
| `std_msgs` | `supervisor_T` | `Bool`, `Float32` |
| `sensor_msgs` | `camera_viewer` | `sensor_msgs/Image` |
| `nav_msgs` | `takeoff`, `pouso`, `drone_yaw_360`, `supervisor_T` | `Odometry` |
| `mavros_msgs` | `takeoff`, `pouso` | `State`, `SetMode` (cliente de serviço) |
| `mrs_msgs` | reservado | integração futura com MRS UAV |
| `tf2` | `drone_yaw_360` | `Eigen::Quaterniond` para extração de Euler |
| `cv_bridge` | `camera_viewer` | conversão `sensor_msgs/Image` ↔ `cv::Mat` |
| `image_transport` | `camera_viewer` | transporte eficiente (compressão) |
| `opencv` | `camera_viewer` | rendering `imshow`, `resize`, `cvtColor` |

### Bloco 5 — Dependências de execução de launch

```xml
<exec_depend>launch</exec_depend>
<exec_depend>launch_ros</exec_depend>
```

**O que este bloco faz:**

Dependências somente de execução (não afetam o build C++). Garantem que o
sistema de launch do ROS 2 esteja disponível ao executar:

```bash
ros2 launch drone_control camera_viewer.launch.py
```

---

## 2. `CMakeLists.txt` — Script de build

### Bloco 1 — Cabeçalho e flags de compilação

```cmake
cmake_minimum_required(VERSION 3.8)
project(drone_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
```

**O que este bloco faz:**

- Requer CMake ≥ 3.8 (mínimo do ROS 2 Humble).
- `project(drone_control)` — define `${PROJECT_NAME}` = `drone_control`, usado
  em todo o script como referência ao nome do pacote.
- `-Wall -Wextra -Wpedantic` — ativa warnings completos em GCC/Clang para
  detectar bugs de compilação cedo.

### Bloco 2 — Localização de pacotes

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(mrs_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
```

**O que este bloco faz:**

Cada `find_package()` localiza o pacote no sistema e importa seus targets
CMake. `REQUIRED` faz o build falhar com mensagem clara se o pacote não
estiver instalado. `Eigen3` e `OpenCV` são libraries do sistema (não ROS 2),
localizadas pela infraestrutura CMake padrão.

### Bloco 3 — Geração de interfaces customizadas

```cmake
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YawOverride.msg"
  "msg/Waypoint4D.msg"
  "msg/Waypoint4DArray.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
```

**O que este bloco faz:**

`rosidl_generate_interfaces` invoca o gerador de interfaces para os três `.msg`
definidos no pacote. O resultado é a geração automática de:

- `include/drone_control/msg/yaw_override.hpp`
- `include/drone_control/msg/waypoint4_d.hpp`
- `include/drone_control/msg/waypoint4_d_array.hpp`

e das libraries de typesupport necessárias para serialização. `DEPENDENCIES`
declara que os tipos desses arquivos `.msg` dependem de `geometry_msgs` e
`std_msgs` (necessário para `Waypoint4D` que usa `geometry_msgs/Pose`).

`ament_export_dependencies(rosidl_default_runtime)` garante que qualquer
pacote que dependa de `drone_control` também receberá o runtime das mensagens.

### Bloco 4 — Target `drone_yaw_360`

```cmake
add_executable(drone_yaw_360 src/drone_yaw_360.cpp)

ament_target_dependencies(drone_yaw_360
  rclcpp
  nav_msgs
  mavros_msgs
)

target_include_directories(drone_yaw_360 PUBLIC
  $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>
)

rosidl_target_interfaces(drone_yaw_360
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS drone_yaw_360
  DESTINATION lib/${PROJECT_NAME}
)
```

**O que este bloco faz:**

- `add_executable` cria o binário `drone_yaw_360` a partir de `src/drone_yaw_360.cpp`.
- `ament_target_dependencies` linka rclcpp, nav_msgs e mavros_msgs.
- `target_include_directories` adiciona o path dos headers do Eigen3 (necessário
  para `Eigen::Quaterniond` usado na extração de Euler).
- `rosidl_target_interfaces` linka a library de typesupport gerada para que o
  nó possa publicar `drone_control/msg/YawOverride`.
- `install` copia o binário para `lib/drone_control/` no install space.

### Bloco 5 — Targets simples (`takeoff`, `pouso`, `missao_P_T`)

```cmake
add_executable(takeoff src/takeoff.cpp)
ament_target_dependencies(takeoff
  rclcpp geometry_msgs nav_msgs mavros_msgs)
install(TARGETS takeoff DESTINATION lib/${PROJECT_NAME})

add_executable(pouso src/pouso.cpp)
ament_target_dependencies(pouso
  rclcpp geometry_msgs nav_msgs mavros_msgs)
install(TARGETS pouso DESTINATION lib/${PROJECT_NAME})

add_executable(missao_P_T src/missao_P_T.cpp)
ament_target_dependencies(missao_P_T rclcpp)
install(TARGETS missao_P_T DESTINATION lib/${PROJECT_NAME})
```

**O que este bloco faz:**

Padrão repetido para três executáveis de complexidade média. `missao_P_T` usa
apenas `rclcpp` pois não assina/publica tópicos diretamente — delega isso
aos subprocessos `pouso` e `takeoff` via `fork()`/`execlp()`.

### Bloco 6 — Target `camera_viewer`

```cmake
add_executable(camera_viewer src/camera_viewer.cpp)

ament_target_dependencies(camera_viewer
  rclcpp sensor_msgs image_transport cv_bridge)

target_link_libraries(camera_viewer
  ${OpenCV_LIBRARIES})

install(TARGETS camera_viewer
  DESTINATION lib/${PROJECT_NAME})
```

**O que este bloco faz:**

`target_link_libraries` com `${OpenCV_LIBRARIES}` é necessário para linkar
as bibliotecas OpenCV do sistema (não são pacotes ROS 2, por isso não usam
`ament_target_dependencies`). As variáveis `OpenCV_LIBRARIES` e
`OpenCV_INCLUDE_DIRS` são populadas pelo `find_package(OpenCV REQUIRED)`.

### Bloco 7 — Target `supervisor_T`

```cmake
add_executable(supervisor_T src/supervisor_T.cpp)
ament_target_dependencies(supervisor_T rclcpp std_msgs nav_msgs)
install(TARGETS supervisor_T DESTINATION lib/${PROJECT_NAME})
```

**O que este bloco faz:**

Target mais simples após `missao_P_T`. O `supervisor_T` usa apenas tópicos
padrão do ROS 2 (`Bool`, `Float32`, `Odometry`) — sem interfaces customizadas.

### Bloco 8 — Instalação dos launches

```cmake
ament_package()
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

**O que este bloco faz:**

- `ament_package()` — macro obrigatória do ament; finaliza o script registrando
  o pacote no sistema de export do ROS 2.
- `install(DIRECTORY launch …)` — copia toda a pasta `launch/` para
  `share/drone_control/launch/` no install space, tornando os arquivos de launch
  descobríveis via `ros2 launch drone_control <nome>.launch.py`.

---

## 3. Como compilar e verificar

```bash
# Build completo do pacote (e suas dependências)
cd ~/ros2_ws
colcon build --packages-select drone_control
source install/setup.bash

# Verificar que os executáveis foram instalados
ls install/drone_control/lib/drone_control/

# Verificar que os launches foram instalados
ls install/drone_control/share/drone_control/launch/

# Verificar que as interfaces foram geradas
ros2 interface list | grep drone_control

# Inspecionar um tipo de mensagem
ros2 interface show drone_control/msg/YawOverride
ros2 interface show drone_control/msg/Waypoint4D
ros2 interface show drone_control/msg/Waypoint4DArray
```

### Saída esperada após build bem-sucedido

```
install/drone_control/lib/drone_control/
  camera_viewer
  drone_yaw_360
  missao_P_T
  pouso
  supervisor_T
  takeoff

install/drone_control/share/drone_control/launch/
  camera_viewer.launch.py
  drone_yaw_360.launch.py
  mission_three_nodes.launch.py
  supervisor_T.launch.py
  tf_body_fallback.launch.py
  tf_camera_static.launch.py
```
