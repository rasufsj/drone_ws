# 14 — Launch Files e Testes

> **Explicação linha por linha** de cada launch file está em
> [`14-launch.md`](14-launch.md).

Este documento apresenta um **resumo** dos três launch files e dos sete
arquivos de teste do pacote `yolo_pad_pose`. Para a explicação linha por
linha dos módulos Python principais, consulte
[`13-modulos-principais.md`](13-modulos-principais.md).

---

## 1. Launch Files

Para a explicação detalhada (linha por linha, argumentos, parâmetros,
remappings, frames TF e exemplos de execução) de cada launch file, acesse
[`14-launch.md`](14-launch.md).

### 1.1 `tf_body_fallback.launch.py`

**Objetivo:** publicar a TF estática `uav1/base_link → uav1/fcu` quando o
MRS core não está rodando.

O launch inicia um único nó `static_transform_publisher` do pacote `tf2_ros`,
com translação e rotação nulas (offset zero). Garante que a árvore TF seja
contínua do frame de odometria até o FCU, necessário para que
`tf_camera_static.launch.py` (que parte do `uav1/fcu`) seja acessível a
partir do `uav1/base_link`.

**Quando usar:** sempre que o sistema não inclua o stack completo MRS
(`mrs_uav_core.launch.py`). Na sessão `one_drone`, é iniciado na janela
`tf_fixes`.

---

### 1.2 `tf_camera_static.launch.py`

**Objetivo:** publicar as TFs estáticas das câmeras RealSense em relação ao
FCU do drone, usando os offsets físicos medidos no hardware real.

O launch inicia dois nós `static_transform_publisher`:

| Nó | Transform | Translação (x, y, z) |
|----|-----------|----------------------|
| `tf_rgbd_down` | `uav1/fcu → uav1/rgbd_down` | (0.153, 0.0, −0.129) m |
| `tf_rgbd_front` | `uav1/fcu → uav1/rgbd_front` | (0.181, 0.0, −0.089) m |

Esses valores representam a posição de cada câmera em relação ao FCU no
frame do corpo do drone (x=frente, y=esquerda, z=cima). Com esses transforms
publicados, o `yolo_pad_pose_ros2` consegue converter qualquer detecção
óptica para o `uav1/base_link` via `tf_buffer.lookup_transform`.

**Quando usar:** sempre que as câmeras RealSense estiverem presentes. Na
sessão `one_drone`, é iniciado na janela `tf_fixes`.

---

### 1.3 `odom_tf_broadcaster.launch.py`

**Objetivo:** subir dois nós juntos — `odom_tf_broadcaster` e
`pad_waypoint_supervisor` (legado) — para simplificar o lançamento de toda
a pilha de detecção e supervisão de waypoints.

O launch instancia:
- **`odom_tf_broadcaster`**: nó do pacote `yolo_pad_pose`, com parâmetros
  `odom_topic=/uav1/mavros/local_position/odom` e
  `use_odom_header_stamp=True`.
- **`pad_waypoint_supervisor`**: nó legado de supervisão de waypoints com
  parâmetros padrão.

> **Nota de uso atual:** na sessão `one_drone`, o `odom_tf_broadcaster` é
> iniciado diretamente (`ros2 run`) na janela `tf_fixes`, e o
> `pad_waypoint_supervisor` foi substituído pelo `base_waypoint_publisher`
> (`pad_waypoint_nn`). Este launch file é mantido para compatibilidade com
> fluxos de trabalho anteriores.

---

## 2. Testes

O pacote usa **pytest** com stubs de ROS 2 definidos em `conftest.py`,
permitindo que os testes sejam executados sem uma instalação ROS 2 completa.

### Executar todos os testes

```bash
cd ~/ros2_ws
colcon test --packages-select yolo_pad_pose
colcon test-result --verbose
```

Ou diretamente com pytest (com o venv ativado):

```bash
cd yolo_pad_pose
pytest test/ -v
```

---

### 2.1 `conftest.py`

Configuração global do pytest. Adiciona o pacote ao `sys.path` e cria
**stubs mínimos** para todos os módulos ROS 2 que os módulos principais
importam (`rclpy`, `rclpy.node`, `geometry_msgs`, `nav_msgs`, `sensor_msgs`,
`cv_bridge`, `message_filters`, `ultralytics`, `tf2_ros`,
`tf2_geometry_msgs`). Isso permite importar os módulos em ambiente Python
puro sem ROS 2 instalado, mantendo os testes unitários rápidos e isolados.

---

### 2.2 `test_range_filter.py`

Testa a função pura `range_filter_check` de `yolo_pad_pose_ros2`.

Cobre os cenários:
- Filtro desabilitado (`max_range_m <= 0`)
- Detecção dentro do alcance (incluindo o limiar exato)
- Detecção além do alcance
- Coordenadas negativas (usa `hypot`, sempre positivo)
- Apenas componente X ou apenas Y não-nula

---

### 2.3 `test_base_waypoint_publisher.py`

Testa as funções puras de `base_waypoint_publisher`:
- `merge_radius_from_area`: conversão área → raio (incluindo área nula/negativa)
- `body_to_world`: projeção corpo→mundo para múltiplos yaws e posições
- `cluster_update`: lógica de fusão (`merged`/`added`/`discarded`) e
  capacidade máxima (`max_bases`)
- `BaseEntry.update`: comportamento do EMA (convergência e suavização)

---

### 2.4 `test_jump_filter_reset.py`

Testa a função `jump_filter_check` do módulo legado `pad_waypoint_supervisor`
(ainda presente no repositório via `__pycache__`). Cobre:
- Aceitação quando não há âncora
- Aceitação de saltos pequenos (dentro de `max_jump_m`)
- Rejeição de saltos grandes (abaixo do limiar de reset)
- Auto-reset após N rejeições consecutivas (`jump_reject_reset_count`)

---

### 2.5 `test_flake8.py`

Executa o linter **flake8** (via `ament_flake8`) em todos os arquivos Python
do pacote. Verifica conformidade com PEP 8 (comprimento de linha, espaços,
imports, etc.).

```bash
# Executar manualmente
ament_flake8 yolo_pad_pose/
```

---

### 2.6 `test_pep257.py`

Executa o linter **pep257** (via `ament_pep257`) para verificar as
docstrings de todos os módulos e funções públicas. O escopo cobre o pacote
raiz mas exclui o diretório `test/`.

```bash
# Executar manualmente
ament_pep257 yolo_pad_pose/
```

---

### 2.7 `test_copyright.py`

Verifica a presença de cabeçalhos de copyright em todos os arquivos Python
via `ament_copyright`. Útil para conformidade com políticas de licenciamento
em projetos de longo prazo.

```bash
# Executar manualmente
ament_copyright yolo_pad_pose/
```
