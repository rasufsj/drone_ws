# Documentação — `yolo_pad_pose`

Pacote ROS 2 Python responsável pela **detecção visual do pad de pouso** (landing pad)
via YOLO v8 e pela **publicação de waypoints** de missão para o controlador de voo.

---

## Índice

| Arquivo | Conteúdo |
|---------|----------|
| [`00-inventario.md`](00-inventario.md) | Lista real de todos os arquivos do pacote e seus papéis |
| [`10-ambiente-instalacao.md`](10-ambiente-instalacao.md) | Instalação do venv YOLO, Ultralytics, dependências ROS 2 e execução |
| [`11-dataset-e-treinamento.md`](11-dataset-e-treinamento.md) | Problema inicial, montagem do val800, dataset CVAT→Ultralytics, treino e validação |
| [`12-ros2-integracao.md`](12-ros2-integracao.md) | Como rodar no ROS 2 Jazzy, parâmetros, tópicos/frames, ajuste do `conf` e sessão `one_drone` |
| [`13-modulos-principais.md`](13-modulos-principais.md) | Explicação **linha por linha** dos 4 módulos Python principais |
| [`14-launch.md`](14-launch.md) | Explicação **linha por linha** dos 3 launch files (argumentos, nós, parâmetros, TF tree, exemplos) |
| [`14-launch-e-tests.md`](14-launch-e-tests.md) | Resumo dos launch files e explicação dos testes automatizados |

---

## Visão rápida do pacote

```
yolo_pad_pose/
├── yolo_pad_pose/
│   ├── yolo_pad_pose_ros2.py      ← nó principal: YOLO + TF → PointStamped
│   ├── odom_tf_broadcaster.py     ← publica TF a partir da odometria
│   ├── base_waypoint_publisher.py ← FSM de missão nearest-neighbour
│   └── capture_dataset.py         ← utilitário de captura de imagens
├── launch/
│   ├── tf_body_fallback.launch.py
│   ├── tf_camera_static.launch.py
│   └── odom_tf_broadcaster.launch.py
└── test/
    ├── conftest.py
    ├── test_range_filter.py
    ├── test_base_waypoint_publisher.py
    ├── test_jump_filter_reset.py
    ├── test_flake8.py
    ├── test_pep257.py
    └── test_copyright.py
```

---

## Tópicos publicados (resumo)

| Tópico | Tipo | Origem |
|--------|------|--------|
| `/landing_pad/base_relative_position` | `PointStamped` | `yolo_pad_pose_ros2` |
| `/landing_pad/h_relative_position` | `PointStamped` | `yolo_pad_pose_ros2` |
| `/landing_pad/relative_position` | `PointStamped` | `yolo_pad_pose_ros2` (alias legado) |
| `/waypoints` | `PoseArray` | `base_waypoint_publisher` |

---

## Integração na simulação

O pacote é inicializado automaticamente pela sessão tmux
`mrs_uav_gazebo_simulator/tmux/one_drone/session.yml`.
Consulte [`12-ros2-integracao.md`](12-ros2-integracao.md) para o detalhamento
completo das janelas `tf_fixes` e `Terminal YOLO`.
