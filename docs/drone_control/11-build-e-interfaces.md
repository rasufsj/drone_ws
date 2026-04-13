# Build e Interfaces — `drone_control`

> **Objetivo:** explicar **linha por linha** o `package.xml` e o `CMakeLists.txt`
> do pacote `drone_control` — cada declaração de dependência, cada target de
> compilação e cada diretiva de instalação.
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

| Linha | O que faz |
|-------|-----------|
| `<?xml version="1.0"?>` | Declaração XML padrão; indica codificação UTF-8 implícita |
| `<?xml-model ...>` | Instrução de processamento que aponta para o schema XSD do ROS 2; IDEs como VS Code usam isso para validar o arquivo |
| `<package format="3">` | Abre o manifesto no formato 3 — o mais recente no ROS 2 Humble+; adiciona suporte a `<depend>` unificado |
| `<name>drone_control</name>` | Define o nome do pacote; **deve** corresponder ao `project()` no CMakeLists.txt e ao diretório no workspace |
| `<version>0.0.0</version>` | Versão semântica; `0.0.0` indica desenvolvimento ativo sem release formal |
| `<description>...</description>` | Descrição curta exibida por `ros2 pkg list --info`; ainda como TODO |
| `<maintainer email="...">lmnr31</maintainer>` | Contato do mantenedor; usado por `rosdep` e pela infraestrutura de pacotes |
| `<license>Apache-2.0</license>` | SPDX identifier da licença; deve coincidir com o arquivo `LICENSE` na raiz |

### Bloco 2 — Ferramentas de build

```xml
  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>
```

| Linha | O que faz |
|-------|-----------|
| `<buildtool_depend>ament_cmake</buildtool_depend>` | Declara dependência no sistema de build `ament_cmake`; **obrigatório** para todo pacote CMake ROS 2. Garante que as macros `ament_target_dependencies`, `ament_package`, etc. estejam disponíveis |
| `<buildtool_depend>rosidl_default_generators</buildtool_depend>` | Declara dependência no gerador de código ROSIDL; **obrigatório** para qualquer pacote que defina arquivos `.msg`, `.srv` ou `.action`. Sem isso, `rosidl_generate_interfaces()` no CMakeLists não funciona |

### Bloco 3 — Runtime de interfaces

```xml
  <depend>rosidl_default_runtime</depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
```

| Linha | O que faz |
|-------|-----------|
| `<depend>rosidl_default_runtime</depend>` | Dependência de build **e** execução das bibliotecas de serialização geradas. `<depend>` é shorthand para `<build_depend>` + `<exec_depend>`. Garante que o typesupport dos `.msg` esteja disponível em tempo de execução |
| `<member_of_group>rosidl_interface_packages</member_of_group>` | Registra o pacote no grupo de interfaces ROSIDL. Permite que outros pacotes declarem `<depend>drone_control</depend>` para receber automaticamente os tipos gerados |

### Bloco 4 — Dependências de código C++

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

| Linha | Nó(s) que usa | Por quê é necessária |
|-------|--------------|---------------------|
| `rclcpp` | todos | runtime C++ do ROS 2: `rclcpp::Node`, publishers, subscribers, timers |
| `rclpy` | launch files | interpretador Python dos arquivos `launch/*.launch.py` |
| `geometry_msgs` | `takeoff`, `pouso` | tipos `PoseArray`, `Pose`, `PointStamped` |
| `std_msgs` | `supervisor_T` | tipos `Bool`, `Float32` para tópicos de trajetória |
| `rcl_interfaces` | (infra) | tipos de parâmetros ROS 2 (`ParameterEvent`, etc.) |
| `sensor_msgs` | `camera_viewer` | tipo `Image` para recepção de frames de câmera |
| `mrs_msgs` | (reservado) | integração futura com MRS UAV System |
| `nav_msgs` | `takeoff`, `pouso`, `drone_yaw_360`, `supervisor_T` | tipo `Odometry` para posição/orientação |
| `mavros_msgs` | `takeoff`, `pouso` | tipos `State` (conexão FCU) e serviço `SetMode` |
| `tf2` | `drone_yaw_360` | usada indiretamente via Eigen para cálculo de quaternion→Euler |
| `cv_bridge` | `camera_viewer` | conversão `sensor_msgs/Image` ↔ `cv::Mat` |
| `image_transport` | `camera_viewer` | transporte eficiente de imagens (compressão, plugins) |
| `opencv` | `camera_viewer` | `cv::imshow`, `cv::resize`, `cv::cvtColor` |

### Bloco 5 — Execução dos launches

```xml
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
```

| Linha | O que faz |
|-------|-----------|
| `<exec_depend>launch</exec_depend>` | Dependência somente de execução do framework de launch do ROS 2. Fornece `LaunchDescription`, `TimerAction`, `RegisterEventHandler`, `OnProcessExit` |
| `<exec_depend>launch_ros</exec_depend>` | Extensão ROS 2 do framework de launch. Fornece a ação `Node` que encapsula `ros2 run` nos arquivos `.launch.py` |

### Bloco 6 — Dependências de teste

```xml
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
```

| Linha | O que faz |
|-------|-----------|
| `<test_depend>ament_lint_auto</test_depend>` | Dependência somente de teste; ativa os linters automáticos do ament (cpplint, flake8, etc.) quando `BUILD_TESTING=ON` |
| `<test_depend>ament_lint_common</test_depend>` | Conjunto de linters comuns do ROS 2 (copyright, cppcheck, etc.) |

### Bloco 7 — Export

```xml
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

| Linha | O que faz |
|-------|-----------|
| `<export><build_type>ament_cmake</build_type></export>` | Declara explicitamente que este pacote usa o sistema `ament_cmake`. Necessário para que ferramentas como `colcon` escolham o builder correto |
| `</package>` | Fecha o manifesto |

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

| Linha | O que faz |
|-------|-----------|
| `cmake_minimum_required(VERSION 3.8)` | Exige CMake ≥ 3.8; versão mínima para ROS 2 Humble. Garante disponibilidade de APIs modernas do CMake |
| `project(drone_control)` | Define `${PROJECT_NAME}` = `drone_control`. Esta variável é usada em todo o script nos targets, diretórios de instalação e chamadas `rosidl_generate_interfaces` |
| `if(CMAKE_COMPILER_IS_GNUCXX ...)` | Verifica se o compilador é GCC ou Clang (o bloco é ignorado em MSVC) |
| `-Wall` | Ativa todos os warnings padrão de qualidade de código |
| `-Wextra` | Ativa warnings adicionais não cobertos por `-Wall` (ex.: parâmetros não usados) |
| `-Wpedantic` | Emite warnings para código não conforme com o padrão C++ selecionado |

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

| Linha | O que faz |
|-------|-----------|
| `find_package(ament_cmake REQUIRED)` | Carrega o módulo ament_cmake; disponibiliza macros `ament_*`. `REQUIRED` faz o build parar com erro claro se não encontrado |
| `find_package(rclcpp REQUIRED)` | Importa o target CMake `rclcpp::rclcpp`; necessário para todos os executáveis C++ |
| `find_package(rosidl_default_generators REQUIRED)` | Importa o gerador de interfaces; deve vir **antes** de `rosidl_generate_interfaces()` |
| `find_package(Eigen3 REQUIRED)` | Localiza Eigen3 via CMake padrão (não é pacote ROS 2); popula `EIGEN3_INCLUDE_DIRS` |
| `find_package(OpenCV REQUIRED)` | Localiza OpenCV via CMake padrão; popula `OpenCV_LIBRARIES` e `OpenCV_INCLUDE_DIRS` |
| `find_package(rosidl_cmake REQUIRED)` | Importa utilitários de baixo nível do ROSIDL; necessário internamente pelo `rosidl_generate_interfaces` |

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

| Linha | O que faz |
|-------|-----------|
| `rosidl_generate_interfaces(${PROJECT_NAME}` | Macro que registra os arquivos `.msg` para geração de código. `${PROJECT_NAME}` = `drone_control` — nome do target gerado |
| `"msg/YawOverride.msg"` | Registra `msg/YawOverride.msg` para geração de `drone_control/msg/yaw_override.hpp` |
| `"msg/Waypoint4D.msg"` | Registra `msg/Waypoint4D.msg` → `drone_control/msg/waypoint4_d.hpp` |
| `"msg/Waypoint4DArray.msg"` | Registra `msg/Waypoint4DArray.msg` → `drone_control/msg/waypoint4_d_array.hpp`; depende de `Waypoint4D` |
| `DEPENDENCIES geometry_msgs std_msgs` | Informa que os `.msg` importam tipos de `geometry_msgs` (`Pose`) e `std_msgs` (`Header`). Necessário para que o gerador inclua os headers corretos |
| `)` | Fecha `rosidl_generate_interfaces` |
| `ament_export_dependencies(rosidl_default_runtime)` | Exporta `rosidl_default_runtime` como dependência transitiva; qualquer pacote que use `drone_control` receberá automaticamente o runtime das mensagens |

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

| Linha | O que faz |
|-------|-----------|
| `add_executable(drone_yaw_360 src/drone_yaw_360.cpp)` | Cria o target executável `drone_yaw_360` a partir de um único arquivo fonte |
| `ament_target_dependencies(drone_yaw_360 rclcpp nav_msgs mavros_msgs)` | Linka `rclcpp`, `nav_msgs` e `mavros_msgs` propagando corretamente include paths, link flags e dependências transitivas |
| `target_include_directories(... $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIRS}>)` | Adiciona os headers do Eigen3 ao path de include **somente durante o build** (não exportado). Necessário para `#include <Eigen/Geometry>` em `drone_yaw_360.cpp` |
| `rosidl_target_interfaces(drone_yaw_360 ${PROJECT_NAME} "rosidl_typesupport_cpp")` | Linka o typesupport C++ gerado para `drone_control`. Necessário para que `drone_yaw_360` possa publicar `drone_control::msg::YawOverride` |
| `install(TARGETS drone_yaw_360 DESTINATION lib/${PROJECT_NAME})` | Instala o binário em `install/drone_control/lib/drone_control/drone_yaw_360`; torna disponível via `ros2 run drone_control drone_yaw_360` |

### Bloco 5 — Targets `takeoff`, `pouso` e `missao_P_T`

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

| Linha | O que faz |
|-------|-----------|
| `add_executable(takeoff src/takeoff.cpp)` | Target para o nó de decolagem |
| `ament_target_dependencies(takeoff rclcpp geometry_msgs nav_msgs mavros_msgs)` | `geometry_msgs` → `PoseArray`; `nav_msgs` → `Odometry`; `mavros_msgs` → `State` |
| `add_executable(pouso src/pouso.cpp)` | Target para o nó de pouso |
| `ament_target_dependencies(pouso rclcpp geometry_msgs nav_msgs mavros_msgs)` | Igual ao `takeoff` mais `mavros_msgs::srv::SetMode` (cliente de serviço no `pouso`) |
| `add_executable(missao_P_T src/missao_P_T.cpp)` | Target para o orquestrador de missão |
| `ament_target_dependencies(missao_P_T rclcpp)` | Só precisa de `rclcpp` — não publica/assina tópicos diretamente; usa `fork()`/`execlp()` para delegar ao `pouso` e `takeoff` |
| `install(TARGETS ... DESTINATION lib/${PROJECT_NAME})` | Repetido para cada target; instala em `lib/drone_control/<nome>` |

### Bloco 6 — Target `camera_viewer`

```cmake
add_executable(camera_viewer src/camera_viewer.cpp)

ament_target_dependencies(camera_viewer
  rclcpp
  sensor_msgs
  image_transport
  cv_bridge
)

target_link_libraries(camera_viewer
  ${OpenCV_LIBRARIES}
)

install(TARGETS camera_viewer
  DESTINATION lib/${PROJECT_NAME}
)
```

| Linha | O que faz |
|-------|-----------|
| `add_executable(camera_viewer src/camera_viewer.cpp)` | Target para o visualizador de câmera |
| `ament_target_dependencies(... sensor_msgs image_transport cv_bridge)` | `sensor_msgs` → `Image`; `image_transport` → APIs de transporte; `cv_bridge` → conversão ROS↔OpenCV |
| `target_link_libraries(camera_viewer ${OpenCV_LIBRARIES})` | Linka as bibliotecas OpenCV do sistema (não são pacotes ROS 2, portanto não usam `ament_target_dependencies`). `${OpenCV_LIBRARIES}` é populado por `find_package(OpenCV REQUIRED)` |

### Bloco 7 — Target `supervisor_T`

```cmake
add_executable(supervisor_T src/supervisor_T.cpp)
ament_target_dependencies(supervisor_T rclcpp std_msgs nav_msgs)
install(TARGETS supervisor_T DESTINATION lib/${PROJECT_NAME})
```

| Linha | O que faz |
|-------|-----------|
| `add_executable(supervisor_T src/supervisor_T.cpp)` | Target para o supervisor de trajetória |
| `ament_target_dependencies(supervisor_T rclcpp std_msgs nav_msgs)` | `std_msgs` → `Bool`, `Float32` para os tópicos `/trajectory_finished` e `/trajectory_progress`; `nav_msgs` → `Odometry` para odometria do UAV |

### Bloco 8 — Finalização e instalação de launches

```cmake
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
```

| Linha | O que faz |
|-------|-----------|
| `if(BUILD_TESTING)` | Bloco executado apenas quando `colcon build --cmake-args -DBUILD_TESTING=ON` |
| `find_package(ament_lint_auto REQUIRED)` | Carrega os linters automáticos |
| `set(ament_cmake_copyright_FOUND TRUE)` | Suprime o linter de copyright (arquivos ainda não têm cabeçalho de licença) |
| `set(ament_cmake_cpplint_FOUND TRUE)` | Suprime o cpplint (pacote fora de repositório git separado) |
| `ament_lint_auto_find_test_dependencies()` | Descobre e configura automaticamente todos os linters disponíveis |
| `ament_package()` | **Macro obrigatória** — deve ser a última chamada ament no CMakeLists.txt. Registra o pacote no índice do workspace, configura o export de targets e cria o `package.xml` de install space |
| `install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})` | Copia toda a pasta `launch/` para `share/drone_control/launch/` no install space. Isso torna os arquivos `*.launch.py` descobríveis via `ros2 launch drone_control <nome>` |

---

## 3. Como compilar e verificar

```bash
# Compilar apenas drone_control e suas dependências
cd ~/ros2_ws
colcon build --packages-select drone_control
source install/setup.bash

# Verificar executáveis instalados
ls install/drone_control/lib/drone_control/
# Saída esperada: camera_viewer  drone_yaw_360  missao_P_T  pouso  supervisor_T  takeoff

# Verificar launches instalados
ls install/drone_control/share/drone_control/launch/
# Saída esperada: camera_viewer.launch.py  drone_yaw_360.launch.py  ...

# Verificar interfaces geradas
ros2 interface list | grep drone_control
# Saída esperada:
#   drone_control/msg/Waypoint4D
#   drone_control/msg/Waypoint4DArray
#   drone_control/msg/YawOverride

# Inspecionar estrutura de uma mensagem
ros2 interface show drone_control/msg/Waypoint4DArray
```
