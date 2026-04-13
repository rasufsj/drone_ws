# 00 — Inventário real do pacote `yolo_pad_pose`

Lista completa de todos os arquivos presentes no pacote, com seus papéis e
relações com outros pacotes do workspace.

---

## Meta / Build

| Arquivo | Papel |
|---------|-------|
| `yolo_pad_pose/README.md` | README original do pacote (stub gerado pelo `ros2 pkg create`) |
| `package.xml` | Manifesto ROS 2: dependências de execução (`rclpy`, `sensor_msgs`, `geometry_msgs`, `cv_bridge`, `message_filters`, `tf2_ros`, `tf2_geometry_msgs`, `drone_control`) e dependências de teste (`ament_copyright`, `ament_flake8`, `ament_pep257`, `python3-pytest`) |
| `setup.py` | Configuração setuptools: entry points (`yolo_pad_pose`, `pad_waypoint_nn`, `odom_tf_broadcaster`, `capture_dataset`), inclusão dos launch files em `share/` |
| `setup.cfg` | Diretórios de scripts para instalação ament Python (`script_dir`, `install_scripts`) |
| `resource/yolo_pad_pose` | Arquivo marcador do índice ament (obrigatório para `ros2 pkg list`) |

---

## Código principal

| Arquivo | Papel |
|---------|-------|
| `yolo_pad_pose/yolo_pad_pose_ros2.py` | Nó ROS 2 principal: sincroniza RGB + depth, executa inferência YOLO, converte detecção em `PointStamped` no frame `uav1/base_link` e publica em `/landing_pad/base_relative_position` e `/landing_pad/h_relative_position` |
| `yolo_pad_pose/odom_tf_broadcaster.py` | Nó auxiliar: subscreve `nav_msgs/Odometry` e publica o transform dinâmico correspondente em `/tf`, preenchendo a lacuna que o MAVROS deixa na árvore TF |
| `yolo_pad_pose/base_waypoint_publisher.py` | Nó de missão: acumula posições de bases detectadas via YOLO, aplica clustering no referencial mundo (EMA), e publica `PoseArray` em `/waypoints` guiando o drone numa missão nearest-neighbour entre as bases |
| `yolo_pad_pose/capture_dataset.py` | Script utilitário (sem nó ROS permanente): captura pares imagem+depth da câmera RealSense e os salva em disco para rotulagem |
| `yolo_pad_pose/__init__.py` | Arquivo de pacote Python (vazio) |

---

## Launch files

| Arquivo | Papel |
|---------|-------|
| `launch/tf_body_fallback.launch.py` | Publica TF estático `uav1/base_link → uav1/fcu` (fallback quando o MRS core não está ativo) |
| `launch/tf_camera_static.launch.py` | Publica TFs estáticos `uav1/fcu → uav1/rgbd_down` e `uav1/fcu → uav1/rgbd_front` com offsets físicos reais das câmeras |
| `launch/odom_tf_broadcaster.launch.py` | Sobe o nó `odom_tf_broadcaster` junto com o `pad_waypoint_supervisor` (legado) |

---

## Testes

| Arquivo | Papel |
|---------|-------|
| `test/conftest.py` | Stubs de todos os módulos ROS 2 para permitir import sem ROS instalado; configura `sys.path` |
| `test/test_range_filter.py` | Testa a função pura `range_filter_check` de `yolo_pad_pose_ros2` |
| `test/test_base_waypoint_publisher.py` | Testa helpers puros de `base_waypoint_publisher`: `merge_radius_from_area`, `body_to_world`, `cluster_update`, `BaseEntry` |
| `test/test_jump_filter_reset.py` | Testa `jump_filter_check` do módulo `pad_waypoint_supervisor` (legado) |
| `test/test_flake8.py` | Lint PEP 8 via `ament_flake8` |
| `test/test_pep257.py` | Lint de docstrings via `ament_pep257` |
| `test/test_copyright.py` | Verifica cabeçalhos de copyright via `ament_copyright` |

---

## Relação com outros pacotes

| Pacote | Relação |
|--------|---------|
| `drone_control` | Dependência declarada em `package.xml`; fornece mensagens e nós de controle de voo |
| `my_drone_controller` | Consome `/waypoints` (PoseArray) publicado por `base_waypoint_publisher`; publica `/drone_controller/state_voo` consumido por `base_waypoint_publisher` |
| `mrs_uav_gazebo_simulator` | Sessão tmux `one_drone/session.yml` inicializa este pacote nas janelas `tf_fixes` e `Terminal YOLO` |
