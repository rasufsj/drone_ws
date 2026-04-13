# 14 — Launch Files — Explicação Linha por Linha

Este documento explica **cada linha** dos três launch files do pacote
`yolo_pad_pose`. Para um resumo rápido e para a seção de testes, consulte
[`14-launch-e-tests.md`](14-launch-e-tests.md).

---

## Índice

- [1. `tf_body_fallback.launch.py`](#1-tf_body_fallbacklaunchpy)
- [2. `tf_camera_static.launch.py`](#2-tf_camera_staticlaunchpy)
- [3. `odom_tf_broadcaster.launch.py`](#3-odom_tf_broadcasterlaunchpy)
- [4. Exemplos de execução](#4-exemplos-de-execução)
- [5. Árvore TF completa](#5-árvore-tf-completa)

---

## 1. `tf_body_fallback.launch.py`

### Motivação

O pacote `tf2_ros` exige que a árvore TF não tenha "buracos" entre frames.
Quando o stack MRS completo (`mrs_uav_core`) não está rodando, o frame
`uav1/fcu` fica desconectado de `uav1/base_link`.  
Este launch publica uma transformação estática **zerada** entre esses dois
frames, servindo de **fallback estrutural** para fechar a árvore.

### Código completo

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Structural fallback: uav1/base_link -> uav1/fcu
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_link_to_fcu",
            arguments=[
                "0", "0", "0",    # x y z
                "0", "0", "0",    # roll pitch yaw
                "uav1/base_link", "uav1/fcu",
            ],
            output="screen",
        ),
    ])
```

### Explicação linha por linha

| Linha | Código | Explicação |
|------:|--------|------------|
| 1 | `from launch import LaunchDescription` | Importa a classe central do sistema de launch do ROS 2. `LaunchDescription` é o contêiner que agrupa todas as ações a executar. |
| 2 | `from launch_ros.actions import Node` | Importa a ação `Node`, que representa um processo ROS 2 a ser iniciado. Pertence ao pacote `launch_ros` (extensão ROS do `launch`). |
| 5 | `def generate_launch_description():` | Função obrigatória exigida pelo sistema de launch do ROS 2. É chamada automaticamente por `ros2 launch`; deve retornar um `LaunchDescription`. Não recebe argumentos. |
| 6 | `return LaunchDescription([` | Cria e retorna o objeto que descreve o que será lançado. A lista dentro dos colchetes contém as ações (nós, includes, events, etc.). |
| 7 | `# Structural fallback: uav1/base_link -> uav1/fcu` | Comentário documentando a direção da transformação: o frame **pai** é `uav1/base_link` e o frame **filho** é `uav1/fcu`. |
| 8 | `Node(` | Início da declaração do nó. Os campos seguintes configuram como ele será iniciado. |
| 9 | `package="tf2_ros",` | Nome do pacote ROS 2 que contém o executável. `tf2_ros` é a biblioteca padrão do ROS 2 para transformações de coordenadas. |
| 10 | `executable="static_transform_publisher",` | Nome do executável dentro do pacote. `static_transform_publisher` publica uma única transformação TF que **não muda com o tempo** (diferente de `tf2_ros`'s broadcaster dinâmico). |
| 11 | `name="tf_base_link_to_fcu",` | Nome do nó ROS 2 no grafo de nós. Permite identificar este processo específico com `ros2 node list` e distingui-lo de outros `static_transform_publisher` em execução. |
| 12–16 | `arguments=[...]` | Lista de argumentos de linha de comando passados ao executável. O `static_transform_publisher` aceita: `x y z roll pitch yaw frame_pai frame_filho`. |
| 13 | `"0", "0", "0",    # x y z` | Translação em metros nos eixos X, Y e Z. Todos **zero** porque, por definição, o FCU está na mesma posição que o `base_link` quando não há offset físico conhecido. |
| 14 | `"0", "0", "0",    # roll pitch yaw` | Rotação em radianos (roll, pitch, yaw). Todos **zero**: nenhuma rotação entre `base_link` e `fcu` neste fallback. |
| 15 | `"uav1/base_link", "uav1/fcu",` | Frame **pai** (`uav1/base_link`) e frame **filho** (`uav1/fcu`). A convenção no ROS 2 TF é que o frame filho é expresso no sistema de coordenadas do pai. |
| 17 | `output="screen",` | Redireciona a saída stdout/stderr do nó para o terminal onde o launch foi invocado. Útil para depuração; em produção pode-se usar `output="log"` para gravar em arquivo. |

### Argumentos declarados

Este launch **não declara** `DeclareLaunchArgument`. Todos os parâmetros
(frames e offsets) estão fixos em código. Para mudar os frames, edite o
arquivo diretamente ou crie um novo launch que inclua este como base.

### Quando usar

| Cenário | Usar? |
|---------|-------|
| Simulação sem `mrs_uav_core` | ✅ Sim |
| Hardware com MRS completo rodando | ❌ Não (o MRS já publica esta TF) |
| Testes unitários de nós que leem TF | ✅ Sim |

---

## 2. `tf_camera_static.launch.py`

### Motivação

As câmeras RealSense D435 estão montadas fisicamente no drone com offsets
em relação ao FCU (Flight Control Unit). Para que a detecção visual do
`yolo_pad_pose_ros2` possa ser reprojetada no frame do mundo, o TF tree
precisa saber **onde cada câmera está** no corpo do drone.

Este launch publica duas transformações estáticas com os offsets físicos
medidos no hardware real.

### Código completo

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # uav1/fcu -> uav1/rgbd_down
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_rgbd_down",
            arguments=[
                "0.153", "0.0", "-0.129",  # x y z
                "0.0", "0.0", "0.0",       # roll pitch yaw
                "uav1/fcu", "uav1/rgbd_down",
            ],
            output="screen",
        ),

        # uav1/fcu -> uav1/rgbd_front
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_rgbd_front",
            arguments=[
                "0.181", "0.0", "-0.089",  # x y z
                "0.0", "0.0", "0.0",       # roll pitch yaw
                "uav1/fcu", "uav1/rgbd_front",
            ],
            output="screen",
        ),
    ])
```

### Explicação linha por linha

| Linha | Código | Explicação |
|------:|--------|------------|
| 1 | `from launch import LaunchDescription` | Mesmo que `tf_body_fallback`: importa o contêiner de ações. |
| 2 | `from launch_ros.actions import Node` | Importa a ação `Node` para declarar processos ROS 2. |
| 5 | `def generate_launch_description():` | Ponto de entrada obrigatório do sistema de launch. |
| 6 | `return LaunchDescription([` | Inicia a lista com **dois** nós (câmera down e câmera front). |
| 7 | `# uav1/fcu -> uav1/rgbd_down` | Comentário: esta TF conecta o FCU ao frame da câmera voltada para baixo. |
| 8–18 | Primeiro `Node(...)` | Configura o publicador estático da câmera **rgbd_down** (câmera apontada para baixo, usada para detectar o pad de pouso). |
| 9 | `package="tf2_ros",` | Mesmo pacote padrão de TF. |
| 10 | `executable="static_transform_publisher",` | Mesmo executável; publica a TF continuamente em `/tf_static`. |
| 11 | `name="tf_rgbd_down",` | Nome único no grafo. Diferencia este nó do `tf_rgbd_front` que roda na mesma sessão de launch. |
| 12–16 | `arguments=[...]` | Offsets físicos medidos na câmera rgbd_down. |
| 13 | `"0.153", "0.0", "-0.129",  # x y z` | **x=0.153 m**: a câmera está 15,3 cm à frente do FCU. **y=0.0**: sem deslocamento lateral. **z=−0.129 m**: a câmera está 12,9 cm abaixo do FCU (sinal negativo = para baixo no frame do corpo). |
| 14 | `"0.0", "0.0", "0.0",       # roll pitch yaw` | Nenhuma rotação: a câmera está alinhada com os eixos do FCU (y da câmera = esquerda, z da câmera = para baixo — orientação ROS padrão para câmeras `camera_down`). |
| 15 | `"uav1/fcu", "uav1/rgbd_down",` | Frame pai = FCU; frame filho = `rgbd_down`. Qualquer ponto em `rgbd_down` pode ser convertido para `uav1/base_link` via lookup_transform passando por `fcu → base_link`. |
| 17 | `output="screen",` | Saída para o terminal do launch. |
| 20 | `# uav1/fcu -> uav1/rgbd_front` | Comentário: TF para a câmera frontal (auxiliar, não usada atualmente na detecção principal). |
| 21–31 | Segundo `Node(...)` | Configura o publicador estático da câmera **rgbd_front** (câmera apontada para frente). |
| 24 | `name="tf_rgbd_front",` | Nome distinto para o nó da câmera frontal. |
| 26 | `"0.181", "0.0", "-0.089",  # x y z` | **x=0.181 m**: 18,1 cm à frente do FCU. **y=0.0**: sem offset lateral. **z=−0.089 m**: 8,9 cm abaixo do FCU. A câmera frontal é mais alta que a down porque está fixada em uma posição diferente no frame do drone. |
| 27 | `"0.0", "0.0", "0.0",       # roll pitch yaw` | Sem rotação. |
| 28 | `"uav1/fcu", "uav1/rgbd_front",` | Frame pai = FCU; frame filho = `rgbd_front`. |

### Argumentos declarados

Nenhum `DeclareLaunchArgument`. Os offsets físicos são constantes de
hardware; para alterá-los, edite o arquivo diretamente.

> **Atenção:** se o hardware mudar (nova câmera, remontagem), meça o novo
> offset e atualize as strings de argumentos neste arquivo.

### Tabela de offsets

| Frame filho | x (m) | y (m) | z (m) | roll | pitch | yaw |
|-------------|-------:|------:|------:|-----:|------:|----:|
| `uav1/rgbd_down` | +0.153 | 0.0 | −0.129 | 0° | 0° | 0° |
| `uav1/rgbd_front` | +0.181 | 0.0 | −0.089 | 0° | 0° | 0° |

Convenção de eixos (frame `uav1/fcu`, padrão MRS/ROS):
- **+x** → frente do drone
- **+y** → esquerda do drone
- **+z** → cima

---

## 3. `odom_tf_broadcaster.launch.py`

### Motivação

Este launch tem dupla responsabilidade:

1. **`odom_tf_broadcaster`**: converte as mensagens de odometria
   (`nav_msgs/Odometry`) em transformações TF dinâmicas, publicando
   `odom → uav1/base_link` em tempo real conforme o drone se move.
   Isso é necessário para que os nós de detecção possam usar
   `tf_buffer.lookup_transform` sem receberem erros de "frame desconhecido".

2. **`pad_waypoint_supervisor`** *(legado)*: nó de supervisão de waypoints
   que costumava controlar a missão de aproximação ao pad. Mantido no
   launch para compatibilidade com sessões que o referenciam diretamente.

### Código completo

```python
"""Launch odom_tf_broadcaster alongside pad_waypoint_supervisor."""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    odom_tf = Node(
        package='yolo_pad_pose',
        executable='odom_tf_broadcaster',
        name='odom_tf_broadcaster',
        output='screen',
        parameters=[{
            'odom_topic': '/uav1/mavros/local_position/odom',
            'use_odom_header_stamp': True,
        }],
    )

    supervisor = Node(
        package='yolo_pad_pose',
        executable='pad_waypoint_supervisor',
        name='pad_waypoint_supervisor',
        output='screen',
    )

    return LaunchDescription([odom_tf, supervisor])
```

### Explicação linha por linha

| Linha | Código | Explicação |
|------:|--------|------------|
| 1 | `"""Launch odom_tf_broadcaster alongside pad_waypoint_supervisor."""` | **Docstring de módulo** (obrigatória para passar no `ament_pep257`). Resume em uma linha o propósito do launch. O linter `test_pep257.py` verifica sua presença. |
| 2 | `from launch import LaunchDescription` | Importa o contêiner de ações do sistema de launch do ROS 2. |
| 3 | `from launch_ros.actions import Node` | Importa a ação `Node` para declarar processos ROS 2. |
| 6 | `def generate_launch_description():` | Ponto de entrada obrigatório chamado por `ros2 launch`. |
| 7 | `odom_tf = Node(` | Armazena a configuração do nó em uma variável Python `odom_tf`, em vez de criar o `Node` diretamente na lista. Facilita a leitura e permite referenciá-lo em condições ou eventos futuros. |
| 8 | `package='yolo_pad_pose',` | Pacote ROS 2 que contém o executável. É o próprio pacote `yolo_pad_pose` (não um pacote externo como nos launches de TF estático). |
| 9 | `executable='odom_tf_broadcaster',` | Entry point definido em `setup.py`: `'odom_tf_broadcaster = yolo_pad_pose.odom_tf_broadcaster:main'`. O ROS 2 localiza o script Python pelo nome do entry point. |
| 10 | `name='odom_tf_broadcaster',` | Nome do nó no grafo ROS 2. Aparece em `ros2 node list` como `/odom_tf_broadcaster`. Permite múltiplas instâncias do mesmo executável com nomes distintos. |
| 11 | `output='screen',` | Saída para o terminal. Importante para ver os logs do broadcaster em tempo real. |
| 12–15 | `parameters=[{...}],` | Lista de dicionários de parâmetros ROS 2 passados ao nó na inicialização. Equivalente a `--ros-args -p chave:=valor` na linha de comando, mas persistente no launch. |
| 13 | `'odom_topic': '/uav1/mavros/local_position/odom',` | Parâmetro que define **qual tópico de odometria** o broadcaster assina. O valor `/uav1/mavros/local_position/odom` é o tópico publicado pelo MAVROS com a posição local do drone em relação ao frame `uav1/odom`. |
| 14 | `'use_odom_header_stamp': True,` | Quando `True`, o broadcaster usa o **timestamp do header da mensagem de odometria** (não o tempo atual do sistema) ao publicar a TF. Isso mantém a consistência temporal entre os dados sensoriais e as transformações no TF tree — crucial para sincronização com `message_filters`. |
| 18 | `supervisor = Node(` | Variável para o nó do supervisor de waypoints. |
| 19 | `package='yolo_pad_pose',` | Mesmo pacote. |
| 20 | `executable='pad_waypoint_supervisor',` | Entry point do supervisor legado. Definido em `setup.py` se ainda presente; caso contrário o launch falhará com "executable not found". |
| 21 | `name='pad_waypoint_supervisor',` | Nome no grafo ROS 2. |
| 22 | `output='screen',` | Saída para o terminal. |
| 25 | `return LaunchDescription([odom_tf, supervisor])` | Retorna a descrição final com **ambos os nós** na lista. O ROS 2 os inicia em paralelo (sem ordem garantida). Se precisar de ordem, use `RegisterEventHandler` com `OnProcessStart`. |

### Argumentos declarados

Nenhum `DeclareLaunchArgument` no arquivo. Para sobrescrever parâmetros
na linha de comando, use a sintaxe de argumentos de launch passando um
arquivo YAML ou `--ros-args` após o `--`:

```bash
# Sobrescrever odom_topic na linha de comando (via ros-args extras)
ros2 launch yolo_pad_pose odom_tf_broadcaster.launch.py \
  --ros-args -p odom_topic:=/uav1/mavros/global_position/local
```

Ou crie um arquivo de parâmetros YAML e passe via `parameters=['/path/to/params.yaml']`.

### Parâmetros do `odom_tf_broadcaster`

| Parâmetro | Tipo | Default no launch | Descrição |
|-----------|------|:-----------------:|-----------|
| `odom_topic` | `string` | `/uav1/mavros/local_position/odom` | Tópico `nav_msgs/Odometry` a assinar |
| `use_odom_header_stamp` | `bool` | `True` | Usa timestamp do header da odom para a TF publicada |

### Remappings

Nenhum `remappings` declarado. O nó assina e publica nos tópicos padrão
definidos pelos parâmetros acima.

---

## 4. Exemplos de execução

### Sequência recomendada (simulação sem MRS completo)

```bash
# Terminal 1 — sources
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Terminal 2 — fallback TF (base_link → fcu)
ros2 launch yolo_pad_pose tf_body_fallback.launch.py

# Terminal 3 — TFs estáticos das câmeras (fcu → rgbd_down / rgbd_front)
ros2 launch yolo_pad_pose tf_camera_static.launch.py

# Terminal 4 — odom broadcaster + supervisor (legado)
ros2 launch yolo_pad_pose odom_tf_broadcaster.launch.py
```

### Iniciar apenas o odom_tf_broadcaster (sem o supervisor)

Se você usa o `base_waypoint_publisher` no lugar do `pad_waypoint_supervisor`,
prefira subir o `odom_tf_broadcaster` diretamente:

```bash
ros2 run yolo_pad_pose odom_tf_broadcaster \
  --ros-args \
  -p odom_topic:=/uav1/mavros/local_position/odom \
  -p use_odom_header_stamp:=true
```

### Verificar que as TFs foram publicadas

```bash
# Listar todos os frames
ros2 run tf2_tools view_frames

# Ver uma transform específica
ros2 run tf2_ros tf2_echo uav1/base_link uav1/rgbd_down

# Ver a transform dinâmica de odometria
ros2 run tf2_ros tf2_echo uav1/odom uav1/base_link
```

### Rodar o nó YOLO após os launches de TF

```bash
source /home/lmnr31/venvs/yolo/bin/activate

python3 -m yolo_pad_pose.yolo_pad_pose_ros2 --ros-args \
  -p model_path:=/home/lmnr31/runs/detect/train7/weights/best.pt \
  -p conf:=0.7
```

---

## 5. Árvore TF completa

Com os três launches rodando, a árvore TF fica:

```
uav1/odom
  └─ uav1/base_link          ← publicada pelo odom_tf_broadcaster (dinâmica)
       └─ uav1/fcu            ← publicada pelo tf_body_fallback (estática, zerada)
            ├─ uav1/rgbd_down  ← publicada pelo tf_camera_static (+0.153, 0.0, -0.129)
            └─ uav1/rgbd_front ← publicada pelo tf_camera_static (+0.181, 0.0, -0.089)
```

O `yolo_pad_pose_ros2` chama:

```python
tf_buffer.lookup_transform("uav1/base_link", "uav1/rgbd_down", rclpy.time.Time())
```

Que percorre: `rgbd_down → fcu → base_link` usando as TFs estáticas, e
depois `base_link → odom` se necessário para publicar em frame global.
