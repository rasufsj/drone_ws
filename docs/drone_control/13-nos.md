# Nós C++ — `drone_control`

> **Objetivo:** explicar cada nó C++ do pacote `drone_control` por blocos de
> código — construtor, callbacks, FSM, helpers — cobrindo pub/sub, parâmetros,
> fluxo de dados e exemplos de uso.
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.
> Última sincronização: branch `main`, 2026-04-13.

---

## 1. `camera_viewer.cpp`

### Papel / Responsabilidade

Nó de visualização em tempo real que assina dois tópicos de câmera do drone
(`rgbd_front` e `rgbd_down`) e exibe os frames lado a lado em uma janela
OpenCV. Usado como ferramenta de diagnóstico e monitoramento operacional.

### Bloco 1 — Includes e classe

```cpp
#include <rclcpp/rclcpp.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <map>
#include <mutex>

class CameraViewer : public rclcpp::Node {
private:
  std::map<std::string, cv::Mat> images_rgb_;
  std::mutex image_mutex_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;
  int window_width_{1600};
  int window_height_{900};
```

**O que este bloco faz:**

- `images_rgb_` — mapa `topic → cv::Mat` com a imagem mais recente de cada câmera.
  Usar `std::map` permite escalar para N câmeras sem mudar a lógica de renderização.
- `image_mutex_` — protege `images_rgb_` de race condition entre o callback de
  subscrição (que escreve) e o loop de renderização (que lê).
- `subs_` — vector de subscriptions; mantém os objetos vivos enquanto o nó existe.

### Bloco 2 — Construtor e subscriptions

```cpp
CameraViewer() : Node("camera_viewer") {
  declare_parameter<int>("window_width", 1600);
  declare_parameter<int>("window_height", 900);
  window_width_  = get_parameter("window_width").as_int();
  window_height_ = get_parameter("window_height").as_int();

  cv::namedWindow("Drone Cameras", cv::WINDOW_NORMAL);
  cv::resizeWindow("Drone Cameras", window_width_, window_height_);

  const std::vector<std::string> topics = {
      "/uav1/rgbd_front/color/image_raw",
      "/uav1/rgbd_down/color/image_raw",
  };

  auto qos = rclcpp::SensorDataQoS();  // Best-effort, volatile

  for (const auto &topic : topics) {
    auto sub = this->create_subscription<sensor_msgs::msg::Image>(
        topic, qos,
        [this, topic](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
          this->imageCallback(msg, topic);
        });
    subs_.push_back(sub);
  }
}
```

**O que este bloco faz:**

- `SensorDataQoS()` usa perfil **best-effort, volatile** — adequado para imagens
  de câmera onde perder um frame ocasionalmente é aceitável. Perfil `reliable`
  causaria buffer overflow em alta frequência.
- O lambda captura `topic` por valor, resolvendo qual câmera está chegando sem
  precisar comparar o header da mensagem.

### Bloco 3 — `spinRender()` — loop de renderização

```cpp
void spinRender() {
  rclcpp::WallRate rate(30);  // 30 Hz de renderização

  while (rclcpp::ok()) {
    rclcpp::spin_some(shared_from_this());  // processa callbacks pendentes

    // snapshot thread-safe das imagens
    std::map<std::string, cv::Mat> images_copy;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      images_copy = images_rgb_;
    }

    cv::Mat canvas_bgr = createCanvasBGR(images_copy);
    cv::imshow("Drone Cameras", canvas_bgr);

    int key = cv::waitKey(1) & 0xFF;
    if (key == 27) {       // ESC → shutdown
      rclcpp::shutdown();
      break;
    }

    if (!rclcpp::ok()) break;
    rate.sleep();
  }
}
```

**O que este bloco faz:**

- `WallRate(30)` — usa wall clock (não ROS clock) para evitar crash no Ctrl+C
  quando o contexto ROS 2 não está mais válido.
- `spin_some` em vez de `spin` — processa callbacks sem bloquear, permitindo
  que o loop de renderização continue rodando a 30 Hz independentemente da
  frequência de chegada de imagens.
- O **snapshot com mutex** copia o mapa antes de chamar `createCanvasBGR`,
  liberando o lock antes da operação de rendering (que pode ser lenta).

### Bloco 4 — `imageCallback()` — recepção e conversão

```cpp
void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                   const std::string &topic) {
  try {
    auto cv_ptr = cv_bridge::toCvShare(msg, "rgb8");   // não copia desnecessariamente
    const cv::Mat &rgb = cv_ptr->image;

    std::lock_guard<std::mutex> lock(image_mutex_);
    images_rgb_[topic] = rgb.clone();                  // clone para sobreviver ao shared_ptr
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge error on %s: %s", topic.c_str(), e.what());
  }
}
```

**O que este bloco faz:**

- `toCvShare` (não `toCvCopy`) evita uma cópia desnecessária — apenas cria um
  wrapper. O `.clone()` posterior é necessário para desacoplar o dado do
  `shared_ptr` da mensagem ROS 2.
- `"rgb8"` — força conversão para RGB. A conversão para BGR (exigida pelo OpenCV)
  acontece em `createCanvasBGR`.

### Bloco 5 — `createCanvasBGR()` — montagem do painel

```cpp
cv::Mat createCanvasBGR(const std::map<std::string, cv::Mat> &images_rgb) {
  cv::Mat canvas(window_height_, window_width_, CV_8UC3, cv::Scalar(0,0,0));

  struct Slot { std::string topic; std::string label; };
  const std::vector<Slot> slots = {
      {"/uav1/rgbd_front/color/image_raw", "rgbd_front"},
      {"/uav1/rgbd_down/color/image_raw",  "rgbd_down"},
  };

  const int cols   = 2,   rows   = 1;
  const int cell_w = window_width_ / cols;
  const int cell_h = window_height_ / rows;

  for (int i = 0; i < (int)slots.size(); ++i) {
    int x0 = (i % cols) * cell_w;
    int y0 = (i / cols) * cell_h;

    auto it = images_rgb.find(slots[i].topic);
    if (it == images_rgb.end() || it->second.empty()) {
      cv::putText(canvas, "waiting: " + slots[i].label, cv::Point(x0+20, y0+40),
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,0,255), 2);
      continue;
    }

    cv::Mat resized_rgb, resized_bgr;
    cv::resize(it->second, resized_rgb, cv::Size(cell_w, cell_h));
    cv::cvtColor(resized_rgb, resized_bgr, cv::COLOR_RGB2BGR);   // ROS → OpenCV
    resized_bgr.copyTo(canvas(cv::Rect(x0, y0, cell_w, cell_h)));

    cv::putText(canvas, slots[i].label, cv::Point(x0+20, y0+40),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0,255,0), 2);
  }

  // linha separadora vertical entre os dois painéis
  cv::line(canvas, cv::Point(cell_w, 0), cv::Point(cell_w, window_height_),
           cv::Scalar(200,200,200), 2);
  return canvas;
}
```

**O que este bloco faz:**

- Cria um canvas de fundo preto e divide em `cols×rows` células de tamanho igual.
- Para cada slot, exibe "waiting: ..." em vermelho se a imagem ainda não chegou —
  evita tela em branco no início.
- `cvtColor(RGB→BGR)` é necessária porque ROS 2 publica em RGB e o `imshow`
  do OpenCV espera BGR.

### Como executar

```bash
ros2 run drone_control camera_viewer
ros2 run drone_control camera_viewer --ros-args -p window_width:=1920 -p window_height:=1080
```

---

## 2. `takeoff.cpp`

### Papel / Responsabilidade

Nó de decolagem que publica um waypoint de altitude-alvo em `/waypoints` e
monitora a odometria até confirmar que o drone atingiu a altitude mínima
(`altitude_threshold`). Implementa retry automático em caso de falha.

### Bloco 1 — FSM e construtor

```cpp
enum class TakeoffFSM { WAIT_FCU, PUBLISH_TAKEOFF, MONITOR };

TakeoffNode() : Node("takeoff"), fsm_(TakeoffFSM::WAIT_FCU), ... {
  // Parâmetros
  this->declare_parameter<double>("takeoff_altitude", 1.75);
  this->declare_parameter<double>("altitude_threshold", -1.0);  // <0 → auto
  this->declare_parameter<bool>  ("use_current_xy",    true);
  this->declare_parameter<int>   ("max_attempts",      3);
  // ...

  if (altitude_thresh_ < 0.0) {
    altitude_thresh_ = std::max(takeoff_alt_ - 0.3, 0.2);  // auto threshold
  }

  waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);
  // ...
}
```

**O que este bloco faz:**

- `altitude_threshold < 0` → threshold calculado automaticamente como
  `takeoff_altitude - 0.3 m` (com piso em 0.2 m). Isso evita exigir que o
  controlador atinja exatamente a altitude configurada — uma margem de 30 cm
  é suficiente para confirmar decolagem bem-sucedida.

### Bloco 2 — `timerCallback()` — dispatcher FSM

```cpp
void timerCallback() {
  switch (fsm_) {
    case TakeoffFSM::WAIT_FCU:
      if (!fcu_connected_) {
        RCLCPP_INFO_THROTTLE(..., "Waiting for FCU connection…");
        return;
      }
      fsm_ = TakeoffFSM::PUBLISH_TAKEOFF;
      break;

    case TakeoffFSM::PUBLISH_TAKEOFF:
      if (use_current_xy_ && !odom_received_) {
        RCLCPP_INFO_THROTTLE(..., "Waiting for odometry…");
        return;
      }
      publishTakeoffWaypoint();
      attempt_start_ = this->now();
      fsm_ = TakeoffFSM::MONITOR;
      break;

    case TakeoffFSM::MONITOR:
      monitorAltitude();
      break;
  }
}
```

**O que este bloco faz:**

- **WAIT_FCU** — bloqueia até receber `mavros_msgs/State` com `connected=true`.
  Garante que o FCU está pronto antes de publicar qualquer waypoint.
- **PUBLISH_TAKEOFF** — aguarda odometria se `use_current_xy=true` (para não
  publicar waypoint em (0,0,alt) se a posição inicial for desconhecida).
- **MONITOR** — verifica periodicamente se a altitude foi atingida.

### Bloco 3 — `publishTakeoffWaypoint()` — publicação do waypoint

```cpp
void publishTakeoffWaypoint() {
  geometry_msgs::msg::PoseArray waypoints;
  waypoints.header.frame_id = frame_id_;
  waypoints.header.stamp    = this->now();

  geometry_msgs::msg::Pose pose;
  pose.position.x = use_current_xy_ ? odom_x_ : fallback_x_;
  pose.position.y = use_current_xy_ ? odom_y_ : fallback_y_;
  pose.position.z = takeoff_alt_;
  pose.orientation.w = 1.0;  // quaternion identidade

  waypoints.poses.push_back(pose);
  waypoints_pub_->publish(waypoints);
}
```

**O que este bloco faz:**

- `PoseArray` com um único `Pose` — o `my_drone_controller` interpreta isso
  como "ir para este ponto e manter hover".
- `use_current_xy_=true` (padrão) — garante que o drone sobe verticalmente
  sobre a posição atual, sem desviar lateralmente durante a decolagem.

### Bloco 4 — `monitorAltitude()` — confirmação e retry

```cpp
void monitorAltitude() {
  // Sucesso: altitude atingida
  if (current_z_ >= altitude_thresh_) {
    RCLCPP_INFO(this->get_logger(), "Takeoff successful: z=%.2f m >= %.2f m.", ...);
    rclcpp::shutdown();
    return;
  }

  // Timeout: verificar se deve retentar
  double elapsed = (this->now() - attempt_start_).seconds();
  if (elapsed >= check_after_sec_) {
    attempt_count_++;
    if (attempt_count_ >= max_attempts_) {
      RCLCPP_ERROR(this->get_logger(), "Takeoff failed after %d attempt(s).", max_attempts_);
      rclcpp::shutdown();
      return;
    }
    RCLCPP_WARN(this->get_logger(), "Altitude not reached after %.1fs. Retrying…", check_after_sec_);
    fsm_ = TakeoffFSM::PUBLISH_TAKEOFF;  // republica waypoint
  }
}
```

**O que este bloco faz:**

- Compara `current_z_` (da odometria) com `altitude_thresh_`.
- Se `check_after_sec_` (padrão 10 s) passou sem sucesso → incrementa
  `attempt_count_` e volta para `PUBLISH_TAKEOFF` (republica o waypoint).
- Após `max_attempts_` (padrão 3) falhas → shutdown com log de erro.

### Exemplo de uso

```bash
# Decolagem padrão a 1.75 m
ros2 run drone_control takeoff

# Decolagem a 2.5 m em coordenadas fixas
ros2 run drone_control takeoff --ros-args \
  -p takeoff_altitude:=2.5 \
  -p use_current_xy:=false \
  -p x:=1.0 -p y:=0.0
```

---

## 3. `pouso.cpp`

### Papel / Responsabilidade

Nó de pouso de precisão com FSM de cinco estados que implementa uma estratégia
de duas fases (CENTER → DESCEND) para evitar oscilação de posição XY durante
a descida. Suporta detecção de marcador H via YOLO para pouso autônomo.

### Bloco 1 — FSM e struct HDetection

```cpp
enum class PousoFSM { WAIT_FCU, WAIT_ODOM, COLLECT_H, CENTER, DESCEND };

struct HDetection {
  rclcpp::Time stamp;
  double right {0.0};   // deslocamento lateral para a direita (m)
  double front {0.0};   // deslocamento para frente (m)
  double range {0.0};   // distância total = hypot(right, front)
};
```

**O que este bloco faz:**

- `WAIT_FCU/WAIT_ODOM` — fases de inicialização aguardando sensores.
- `COLLECT_H` — janela de coleta de detecções YOLO antes de calcular o alvo.
- `CENTER` — centralização sobre o alvo XY na altitude de aproximação.
- `DESCEND` — descida para `landing_z` com guard de deriva.
- `HDetection` — snapshot de uma detecção YOLO com timestamp para expiração.

### Bloco 2 — `startLanding()` — cálculo do alvo e entrada em CENTER

```cpp
void startLanding() {
  computeLandingTarget();   // calcula active_land_x_, active_land_y_

  // Determinar altitude de aproximação
  approach_z_ = (approach_z_param_ >= 0.0) ? approach_z_param_ : current_z_;
  if (approach_z_ > max_approach_z_) approach_z_ = max_approach_z_;

  callAutoLand();     // envia set_mode AUTO.LAND ao FCU
  attempt_start_ = this->now();
  enterCenter();
}

void computeLandingTarget() {
  if (use_yolo_h_ && has_best_h_) {
    // Converter coordenadas câmera (right, front) para mapa (x, y) usando yaw atual
    double dx = std::cos(current_yaw_) * best_collected_h_.front
               + std::sin(current_yaw_) * best_collected_h_.right;
    double dy = std::sin(current_yaw_) * best_collected_h_.front
               - std::cos(current_yaw_) * best_collected_h_.right;
    active_land_x_ = current_x_ + dx;
    active_land_y_ = current_y_ + dy;
  } else {
    active_land_x_ = use_current_xy_ ? current_x_ : target_x_;
    active_land_y_ = use_current_xy_ ? current_y_ : target_y_;
  }
}
```

**O que este bloco faz:**

- `computeLandingTarget` realiza a transformação de referencial da detecção YOLO:
  converte `(right, front)` em `(dx, dy)` no frame `map` usando `current_yaw_`.
  Isso é necessário porque o marcador H é detectado no frame da câmera (body-relative).
- `approach_z_param_ < 0` → usa a altitude atual como ponto de aproximação
  (a drone hover onde está, depois desce). Isso evita subir/descer antes de centralizar.
- `callAutoLand()` envia o modo `AUTO.LAND` ao FCU como confirmação extra —
  o nó de pouso é o mecanismo principal, mas o FCU como fallback.

### Bloco 3 — `runCenter()` — fase de centralização

```cpp
void runCenter() {
  double dxy = std::hypot(current_x_ - active_land_x_, current_y_ - active_land_y_);

  if (dxy <= xy_hold_tol_) {
    if (!in_center_stable_) {
      in_center_stable_    = true;
      center_stable_since_ = this->now();
    }
    double stable_dur = (this->now() - center_stable_since_).seconds();
    if (stable_dur >= xy_hold_stable_s_) {
      RCLCPP_INFO(this->get_logger(),
        "✅ Centrado: dxy=%.3f m por %.1fs. Iniciando descida.", dxy, stable_dur);
      publishDescentWaypoint();
      fsm_ = PousoFSM::DESCEND;
      return;
    }
  } else {
    in_center_stable_ = false;   // reset se saiu da tolerância
  }
}
```

**O que este bloco faz:**

- `xy_hold_tol_` (padrão 0.10 m) — o drone deve estar dentro deste raio do alvo.
- `xy_hold_stable_s_` (padrão 1.0 s) — deve permanecer dentro por este tempo
  **continuamente** antes de iniciar a descida. Isso previne que uma passagem
  transiente pelo alvo dispare a descida precocemente.
- `in_center_stable_` é resetado quando o drone sai da tolerância — o timer
  de estabilidade recomeça do zero.

### Bloco 4 — `runDescend()` — fase de descida

```cpp
void runDescend() {
  double dxy  = std::hypot(current_x_ - active_land_x_, current_y_ - active_land_y_);
  bool   z_ok = (current_z_ <= landing_z_ + 0.15);
  bool   xy_ok = (dxy <= xy_hold_tol_);

  if (z_ok && xy_ok) {
    RCLCPP_INFO(this->get_logger(), "✅ Pouso concluído.");
    rclcpp::shutdown();
    return;
  }

  // Abort por deriva excessiva → voltar para CENTER
  if (dxy > xy_abort_tol_) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️  Deriva excessiva durante descida: dxy=%.3f m > %.3f m. Voltando a CENTER.",
      dxy, xy_abort_tol_);
    enterCenter();    // republica waypoint de hover e sobe para approach_z
    return;
  }
}
```

**O que este bloco faz:**

- O waypoint de descida é publicado **apenas uma vez** ao entrar em DESCEND
  (não é republicado a cada tick). Isso evita que o controlador reinicie a
  trajetória de descida cada vez que a posição XY flutua levemente.
- `xy_abort_tol_` (padrão 0.5 m) — limiar mais amplo que `xy_hold_tol_`.
  Somente deriva > 50 cm aborta a descida. Pequenas oscilações são toleradas.
- Ao abortar: `enterCenter()` publica um waypoint de hover na `approach_z_`,
  fazendo o drone subir antes de tentar centralizar novamente.

---

## 4. `drone_yaw_360.cpp`

### Papel / Responsabilidade

Nó que comanda o drone a realizar um giro angular acumulado (padrão: 2π rad = 360°)
usando o mecanismo de `YawOverride` do `my_drone_controller`. Acumula a rotação
via delta de yaw da odometria e desativa o override ao atingir o ângulo alvo.

### Bloco 1 — Construtor e parâmetros

```cpp
DroneYaw360OverrideAngle() : Node("drone_yaw_360") {
  this->declare_parameter<std::string>("uav_ns",          "/uav1");
  this->declare_parameter<double>     ("yaw_rate",        1.0);       // rad/s
  this->declare_parameter<double>     ("yaw_tolerance",   0.05);      // rad
  this->declare_parameter<double>     ("yaw_target_delta", 2*M_PI);  // rad

  pub_ = this->create_publisher<drone_control::msg::YawOverride>(
    uav_ns_ + "/yaw_override/cmd", 10);

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    uav_ns_ + "/mavros/local_position/odom", 10, ...);

  timer_ = this->create_wall_timer(50ms, ...);  // 20 Hz
}
```

**O que este bloco faz:**

- `yaw_target_delta = 2π` (positivo) → giro CCW; negativo → CW.
- Timer a 20 Hz — suficiente para acumular delta de yaw com boa resolução
  sem sobrecarregar a CPU.

### Bloco 2 — `odom_cb()` — extração do yaw via Eigen

```cpp
void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
  Eigen::Quaterniond q(
    msg->pose.pose.orientation.w,
    msg->pose.pose.orientation.x,
    msg->pose.pose.orientation.y,
    msg->pose.pose.orientation.z
  );
  auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);  // ZYX = roll, pitch, yaw
  current_yaw_ = euler[2];  // índice 2 = yaw em ZYX

  if (!odom_ok_) {
    last_yaw_ = current_yaw_;
    odom_ok_  = true;
  }
}
```

**O que este bloco faz:**

- `eulerAngles(0, 1, 2)` usa a convenção intrínseca ZYX do Eigen.
  O índice `[2]` é o yaw no referencial ENU.
- `last_yaw_` é inicializado apenas na primeira odometria recebida, não no
  construtor — garante que o delta inicial seja zero.

### Bloco 3 — `timer_cb()` — acumulação de rotação

```cpp
void timer_cb() {
  if (!odom_ok_) return;

  if (!started_) {
    yaw_initial_    = current_yaw_;
    accumulated_yaw_ = 0.0;
    started_ = true;
    publish_enable();
    return;
  }

  double delta = normalize_angle(current_yaw_ - last_yaw_);
  double dir   = (yaw_target_delta_ >= 0.0) ? 1.0 : -1.0;
  if (delta * dir < 0) delta = 0;   // descarta retrocesso por ruído

  accumulated_yaw_ += std::abs(delta);
  last_yaw_ = current_yaw_;

  if (accumulated_yaw_ >= std::abs(yaw_target_delta_) - yaw_tolerance_ && !already_disabled_) {
    publish_disable();
  }
}
```

**O que este bloco faz:**

- `normalize_angle` garante que o delta está em `[-π, π]`, evitando falsos
  grandes deltas quando o yaw passa por ±π (wrap-around).
- O guard `delta * dir < 0` descarta deltas no sentido oposto ao giro
  comandado — previne que ruído de medição ou oscilação momentânea diminua
  `accumulated_yaw_`.
- `accumulated_yaw_ += std::abs(delta)` — soma o valor absoluto do delta no
  sentido correto, permitindo que o ângulo acumulado seja comparado com
  `|yaw_target_delta_| - yaw_tolerance_`.

---

## 5. `missao_P_T.cpp`

### Papel / Responsabilidade

Nó orquestrador que executa a sequência **Pouso → espera 10 s → Takeoff** como
subprocessos independentes via `fork()`/`execlp()`. Cada fase corre em um
processo filho separado, com isolamento completo de contexto ROS 2.

### Bloco 1 — Construtor e agendamento

```cpp
MissaoPTNode() : Node("missao_P_T"), stop_requested_(false), exit_code_(0) {
  timer_ = this->create_wall_timer(
    1s, std::bind(&MissaoPTNode::startMissionAsync, this));
  RCLCPP_INFO(this->get_logger(), "🚀 missao_P_T iniciado. Sequência: pouso → 10 s → takeoff");
}
```

**O que este bloco faz:**

- O timer de 1 s atrasa o início da missão para que o executor ROS 2 possa
  inicializar completamente antes de `fork()` ser chamado. Isso evita condições
  de corrida na inicialização.

### Bloco 2 — `run_subprocess()` — lançamento de subprocesso

```cpp
static int run_subprocess(const char * executable) {
  pid_t pid = fork();
  if (pid == 0) {
    // Processo filho: substituir imagem com ros2 run drone_control <executable>
    execlp("ros2", "ros2", "run", "drone_control", executable,
           static_cast<char *>(nullptr));
    _exit(127);   // execlp só retorna em caso de erro
  }
  if (pid < 0) return -1;   // fork() falhou

  int status = 0;
  waitpid(pid, &status, 0);   // bloqueia até o filho terminar
  return WIFEXITED(status) ? WEXITSTATUS(status) : -1;
}
```

**O que este bloco faz:**

- `fork()` cria uma cópia exata do processo pai; o filho executa `execlp`
  que substitui a imagem do processo por `ros2 run drone_control <exe>`.
- `_exit(127)` no filho (não `exit()`) evita que os destrutores do processo
  pai sejam chamados no contexto do filho — importante para recursos ROS 2.
- `waitpid(..., 0)` é **bloqueante** — chamado a partir da thread de missão
  (não do executor), portanto não bloqueia o spin loop do ROS 2.
- Exit code 0 → sucesso; != 0 → falha (abortar missão).

### Bloco 3 — `runMission()` — sequência completa

```cpp
void runMission() {
  // FASE 1: pouso
  int pouso_result = run_subprocess("pouso");
  if (pouso_result != 0) {
    RCLCPP_ERROR(..., "❌ [FASE 1] pouso falhou com código %d. Takeoff cancelado.", pouso_result);
    exit_code_.store(1);
    rclcpp::shutdown();
    return;
  }

  // FASE 2: espera 10 s
  RCLCPP_INFO(..., "⏳ [FASE 2] Aguardando 10 s…");
  for (int i = 0; i < 10 && !stop_requested_.load(); ++i)
    std::this_thread::sleep_for(1s);

  // FASE 3: takeoff
  int takeoff_result = run_subprocess("takeoff");
  if (takeoff_result != 0) {
    RCLCPP_ERROR(..., "❌ [FASE 3] takeoff falhou com código %d.", takeoff_result);
    exit_code_.store(1);
  }
  rclcpp::shutdown();
}
```

**O que este bloco faz:**

- Comportamento **fail-safe**: se `pouso` falha (exit code != 0), o takeoff
  é cancelado. O drone não tenta decolar se não pousou com sucesso.
- O loop de espera de 10 s verifica `stop_requested_` a cada segundo, permitindo
  interrupção limpa se o nó for encerrado externamente.

---

## 6. `supervisor_T.cpp`

### Papel / Responsabilidade

Supervisor de ciclo infinito que reage a eventos de conclusão de trajetória.
Após cada trajetória completa, aguarda um delay configurável e lança
`missao_P_T` (ou `pouso` local se o drone estiver na origem). Inclui proteção
contra re-lançamento prematuro via guard de distância e cooldown de sinais.

### Bloco 1 — Estados da FSM

```cpp
enum class SupervisorState {
    INIT,                 // Lança takeoff ao iniciar
    TAKING_OFF,           // Aguarda término do takeoff
    RUN_YAW,              // Executa drone_yaw_360 após takeoff
    WAIT_TRAJ,            // Aguarda trajetória iniciar e terminar
    WAIT_BEFORE_MISSION,  // Delay após fim de trajetória
    RUN_MISSION,          // Executa missao_P_T ou pouso local
};

static constexpr int POST_RESET_COOLDOWN_TICKS = 2;  // 2 × 500 ms = 1 s
```

**O que este bloco faz:**

- O estado `INIT` garante que o drone decola automaticamente ao iniciar o
  supervisor — sem intervenção manual.
- `POST_RESET_COOLDOWN_TICKS` — após entrar em `WAIT_TRAJ`, o supervisor ignora
  sinais de trajetória por 2 ticks (1 s). Isso descarta mensagens "residuais"
  publicadas pelo `pouso` durante o `RUN_MISSION` que ainda estão na fila.

### Bloco 2 — Callbacks de trajetória

```cpp
void progress_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    if (state_ != SupervisorState::WAIT_TRAJ) return;   // ignorar fora do estado
    if (msg->data < 99.9f) {
        if (!trajectory_active_) {
            trajectory_active_ = true;
            trajectory_done_   = false;
            RCLCPP_INFO(..., "▶️  Nova trajetória detectada (%.1f%%).", msg->data);
        }
    } else {
        if (!trajectory_done_) {
            trajectory_done_ = true;
            RCLCPP_INFO(..., "📊 Progresso %.1f%% — trajetória concluída.", msg->data);
        }
    }
}

void finished_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (state_ != SupervisorState::WAIT_TRAJ) return;
    if (!msg->data) {
        trajectory_active_ = true;
        trajectory_done_   = false;
        RCLCPP_INFO(..., "▶️  Nova trajetória (/trajectory_finished=false).");
    } else {
        if (!trajectory_done_) {
            trajectory_done_ = true;
            RCLCPP_INFO(..., "📨 /trajectory_finished=true — concluída.");
        }
    }
}
```

**O que este bloco faz:**

- Ambos os callbacks só atuam no estado `WAIT_TRAJ` — em outros estados,
  os sinais são silenciosamente descartados.
- `99.9f` como limiar (não `100.0f`) — evita que jitter de ponto flutuante
  próximo de 100.0 cause resets espúrios.
- Os dois callbacks são complementares: `progress_callback` para o caso
  em que `my_drone_controller` publica apenas progresso, `finished_callback`
  para o caso em que publica explicitamente o booleano de conclusão.

### Bloco 3 — `check_trajectory()` — lógica de transição

```cpp
void check_trajectory() {
    if (post_reset_ticks_ > 0) {
        --post_reset_ticks_;    // consumir cooldown
        return;
    }
    if (trajectory_done_) {
        wait_start_time_ = this->now();
        state_ = SupervisorState::WAIT_BEFORE_MISSION;
        RCLCPP_INFO(..., "🏁 Trajetória concluída. Aguardando %.1f s…", wait_after_traj_done_s_);
    }
}
```

**O que este bloco faz:**

- O cooldown de `post_reset_ticks_` é decrementado a cada tick (500 ms) após
  entrar em `WAIT_TRAJ`. Somente depois que ele chega a zero os sinais de
  trajetória são processados.
- Ao detectar `trajectory_done_`, registra o `wait_start_time_` e transita
  para `WAIT_BEFORE_MISSION`.

### Bloco 4 — `launch_child()` — lançamento de subprocesso com argumentos

```cpp
pid_t launch_child(const char * executable,
                   const std::vector<std::string> & extra_args)
{
    pid_t pid = fork();
    if (pid == 0) {
        std::vector<const char *> argv = {"ros2", "run", "drone_control", executable};
        if (!extra_args.empty()) {
            argv.push_back("--ros-args");
            for (auto & a : extra_args) argv.push_back(a.c_str());
        }
        argv.push_back(nullptr);
        execvp("ros2", const_cast<char * const *>(argv.data()));
        _exit(127);
    }
    return pid;
}
```

**O que este bloco faz:**

- Permite passar parâmetros extras ao subprocesso, por exemplo:
  `["-p", "use_current_xy:=true", "-p", "xy_hold_tol:=0.10"]`.
- `execvp` (não `execlp`) — recebe vetor de argumentos em vez de lista variádica,
  necessário para construir argumentos dinamicamente.

### Bloco 5 — Guard de distância e zona base

```cpp
// Em WAIT_BEFORE_MISSION, após o delay:
if (use_origin_as_base_ && odom_received_) {
    double base_hold_elapsed = (this->now() - base_zone_enter_time_).seconds();
    bool base_ok = base_zone_entered_ && (base_hold_elapsed >= base_hold_s_);

    if (base_ok) {
        // Pouso local (use_current_xy:=true)
        std::vector<std::string> args = {"-p", "use_current_xy:=true", ...};
        child_pid_ = launch_child("pouso", args);
        current_child_exec_ = "pouso";
    } else {
        // Missão padrão
        child_pid_ = launch_child("missao_P_T", {});
        current_child_exec_ = "missao_P_T";
    }
}

// Guard de distância mínima de re-lançamento
if (min_relaunch_dist_m_ > 0.0 && last_mission_valid_) {
    double dist = std::hypot(current_x_ - last_mission_x_, current_y_ - last_mission_y_);
    if (dist < min_relaunch_dist_m_) {
        RCLCPP_WARN(..., "⛔ Missão ignorada: dist=%.2f m < min=%.2f m", dist, min_relaunch_dist_m_);
        state_ = SupervisorState::WAIT_TRAJ;
        return;
    }
}
```

**O que este bloco faz:**

- `base_zone_entered_` é gerenciado em `odom_callback` — é setado quando
  `hypot(x, y) <= base_tol_m_` continuamente por `base_hold_s_` segundos.
- O guard de distância mínima (`min_relaunch_dist_m_`) previne lançamento
  repetitivo se o drone não se moveu desde a última missão — evita pouso/decolagem
  dupla no mesmo ponto.

### Exemplo de uso completo

```bash
# Iniciar supervisor com parâmetros customizados
ros2 run drone_control supervisor_T --ros-args \
  -p wait_after_traj_done_s:=3.0 \
  -p base_tol_m:=0.15 \
  -p base_hold_s:=2.0 \
  -p min_relaunch_dist_m:=0.3

# Em outro terminal, simular conclusão de trajetória
ros2 topic pub /trajectory_finished std_msgs/msg/Bool "data: true" --once

# Monitorar estado (logs do supervisor)
ros2 topic echo /trajectory_progress
ros2 topic echo /trajectory_finished
```
