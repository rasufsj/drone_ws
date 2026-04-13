# Nós C++ — `drone_control`

> **Objetivo:** explicar **linha por linha** cada nó C++ do pacote `drone_control`,
> cobrindo cada declaração de parâmetro, cada linha de lógica de callback e
> cada transição de estado da FSM.
>
> Linguagem: **português** | Estilo: guia acadêmico de implementação.
> Última sincronização: branch `main`, 2026-04-13.

---

## 1. `camera_viewer.cpp`

### 1.1 Includes e declaração da classe

```cpp
#include <rclcpp/rclcpp.hpp>    // Node, create_subscription, etc.
#include <rclcpp/qos.hpp>       // SensorDataQoS
#include <cv_bridge/cv_bridge.hpp>          // toCvShare: sensor_msgs/Image → cv::Mat
#include <sensor_msgs/msg/image.hpp>        // sensor_msgs::msg::Image
#include <opencv2/opencv.hpp>   // imshow, resize, cvtColor, namedWindow, waitKey
#include <map>                  // std::map para armazenar imagens por tópico
#include <mutex>                // std::mutex + std::lock_guard
#include <string>               // std::string
#include <vector>               // std::vector para manter subs vivas

class CameraViewer : public rclcpp::Node {
private:
  std::map<std::string, cv::Mat> images_rgb_;   // topic → última imagem RGB recebida
  std::mutex image_mutex_;                       // protege images_rgb_ de acesso concorrente
  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> subs_;
  // Mantém shared_ptr das subscriptions vivos. Sem este vector, os shared_ptr
  // seriam liberados ao sair do construtor e as subscriptions canceladas.
  int window_width_{1600};    // sobrescrito por parâmetro ROS 2
  int window_height_{900};    // sobrescrito por parâmetro ROS 2
```

### 1.2 Construtor

```cpp
CameraViewer() : Node("camera_viewer") {
  declare_parameter<int>("window_width", 1600);    // declara param; default = 1600
  declare_parameter<int>("window_height", 900);    // declara param; default = 900
  window_width_  = get_parameter("window_width").as_int();    // lê valor final
  window_height_ = get_parameter("window_height").as_int();

  cv::namedWindow("Drone Cameras", cv::WINDOW_NORMAL);
  // WINDOW_NORMAL: janela redimensionável pelo usuário
  cv::resizeWindow("Drone Cameras", window_width_, window_height_);
  // Define tamanho inicial da janela

  const std::vector<std::string> topics = {
      "/uav1/rgbd_front/color/image_raw",   // câmera frontal RGBD
      "/uav1/rgbd_down/color/image_raw",    // câmera inferior RGBD
  };

  auto qos = rclcpp::SensorDataQoS();
  // SensorDataQoS = best-effort + volatile + depth 10
  // best-effort: OK perder frames; evita buffer overflow em alta frequência
  // volatile: não reenvia mensagens antigas para novos subscribers

  for (const auto &topic : topics) {
    auto sub = this->create_subscription<sensor_msgs::msg::Image>(
        topic, qos,
        [this, topic](const sensor_msgs::msg::Image::ConstSharedPtr msg) {
        // Lambda captura 'this' (acesso ao nó) e 'topic' por valor
          this->imageCallback(msg, topic);
        });
    subs_.push_back(sub);   // mantém o shared_ptr vivo enquanto o nó existir
  }
}
```

### 1.3 `spinRender()` — loop principal

```cpp
void spinRender() {
  rclcpp::WallRate rate(30);
  // WallRate(30) = 30 Hz de wall clock.
  // WallRate (não ROSRate) continua funcionando após rclcpp::shutdown().

  while (rclcpp::ok()) {
    rclcpp::spin_some(shared_from_this());
    // spin_some: processa todos os callbacks pendentes SEM bloquear.
    // Retorna imediatamente após esvaziar a fila.

    std::map<std::string, cv::Mat> images_copy;
    {
      std::lock_guard<std::mutex> lock(image_mutex_);
      // Adquire o mutex; liberado automaticamente ao sair do bloco {}
      images_copy = images_rgb_;
      // Copia o mapa enquanto o lock está mantido → thread-safe
    }
    // Daqui em diante, imageCallback pode escrever em images_rgb_ sem conflito.

    cv::Mat canvas_bgr = createCanvasBGR(images_copy);
    // Monta o painel com as duas câmeras (operação pode ser lenta: resize, cvtColor)

    cv::imshow("Drone Cameras", canvas_bgr);   // exibe o canvas na janela

    int key = cv::waitKey(1) & 0xFF;
    // waitKey(1): aguarda 1 ms por tecla; processa eventos de GUI da janela OpenCV.
    // Sem waitKey, a janela trava e não responde ao mouse/teclado.
    // & 0xFF: máscara para compatibilidade entre plataformas.

    if (key == 27) {        // 27 = código ASCII de ESC
      rclcpp::shutdown();
      break;
    }

    if (!rclcpp::ok()) {
      break;   // Ctrl+C durante o loop
    }
    rate.sleep();   // dorme pelo tempo restante para manter 30 Hz
  }
}
```

### 1.4 `imageCallback()` — recepção e conversão

```cpp
void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr &msg,
                   const std::string &topic) {
  RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "RX %s %ux%u encoding=%s",
      topic.c_str(), msg->width, msg->height, msg->encoding.c_str());
  // Loga no máximo 1x a cada 2000 ms; evita flood a 30 Hz.

  try {
    auto cv_ptr = cv_bridge::toCvShare(msg, "rgb8");
    // toCvShare: wrapper zero-copy (NÃO copia os bytes de imagem).
    // "rgb8": converte para RGB 8-bit se necessário.

    const cv::Mat &rgb = cv_ptr->image;
    // Referência ao cv::Mat interno; dados pertencem ao shared_ptr cv_ptr.

    std::lock_guard<std::mutex> lock(image_mutex_);
    // Adquire mutex antes de modificar images_rgb_
    images_rgb_[topic] = rgb.clone();
    // clone(): cria cópia independente dos pixels.
    // NECESSÁRIO: sem clone(), ao destruir cv_ptr os pixels seriam liberados
    // e images_rgb_[topic] apontaria para memória inválida.

  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge error on %s: %s", topic.c_str(), e.what());
    // Captura exceções de encoding inválido sem derrubar o nó.
  }
}
```

### 1.5 `createCanvasBGR()` — montagem do painel

```cpp
cv::Mat createCanvasBGR(const std::map<std::string, cv::Mat> &images_rgb) {
  cv::Mat canvas(window_height_, window_width_, CV_8UC3, cv::Scalar(0, 0, 0));
  // Canvas de fundo preto; CV_8UC3 = 8 bits unsigned, 3 canais (BGR).

  const int cols   = 2;
  const int rows   = 1;
  const int cell_w = window_width_ / cols;   // ex.: 1600/2 = 800 px por painel
  const int cell_h = window_height_ / rows;  // ex.: 900 px de altura

  for (int i = 0; i < (int)slots.size(); ++i) {
    int col = i % cols;           // 0 (esquerda) ou 1 (direita)
    int row = i / cols;           // sempre 0 (1 linha)
    int x0  = col * cell_w;      // x0=0 para slot 0, x0=800 para slot 1
    int y0  = row * cell_h;      // y0=0 para ambos

    auto it = images_rgb.find(slots[i].topic);
    if (it == images_rgb.end() || it->second.empty()) {
      cv::putText(canvas, ("waiting: " + slots[i].label),
                  cv::Point(x0 + 20, y0 + 40),
                  cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 255), 2);
      // Texto vermelho (BGR: 0,0,255) quando câmera ainda não recebeu imagem
      continue;
    }

    cv::Mat resized_rgb;
    cv::resize(it->second, resized_rgb, cv::Size(cell_w, cell_h));
    // Redimensiona a imagem para o tamanho da célula do painel.

    cv::Mat resized_bgr;
    cv::cvtColor(resized_rgb, resized_bgr, cv::COLOR_RGB2BGR);
    // Converte RGB (formato ROS 2) → BGR (formato OpenCV imshow).

    resized_bgr.copyTo(canvas(cv::Rect(x0, y0, cell_w, cell_h)));
    // Copia a imagem convertida para a célula correta do canvas.
    // cv::Rect(x0, y0, cell_w, cell_h): região de interesse (ROI).

    cv::putText(canvas, slots[i].label,
                cv::Point(x0 + 20, y0 + 40),
                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
    // Label verde (BGR: 0,255,0) sobre cada painel.
  }

  cv::line(canvas, cv::Point(cell_w, 0), cv::Point(cell_w, window_height_),
           cv::Scalar(200, 200, 200), 2);
  // Linha cinza separando os dois painéis verticalmente.

  return canvas;
}
```

---

## 2. `takeoff.cpp`

### 2.1 Enum da FSM

```cpp
enum class TakeoffFSM { WAIT_FCU, PUBLISH_TAKEOFF, MONITOR };
// WAIT_FCU       : aguarda mavros_msgs/State com connected=true
// PUBLISH_TAKEOFF: aguarda odometria (se use_current_xy=true) e publica waypoint
// MONITOR        : verifica altitude; retenta até max_attempts ou shutdown
```

### 2.2 Construtor

```cpp
TakeoffNode() : Node("takeoff"),
                fsm_(TakeoffFSM::WAIT_FCU),
                attempt_start_(rclcpp::Time(0, 0, RCL_ROS_TIME))
{
  this->declare_parameter<std::string>("uav_name", "uav1");
  // uav_name: prefixo dos tópicos MAVROS (ex.: "/uav1/mavros/state")

  this->declare_parameter<double>("takeoff_altitude", 1.75);
  // Altitude-alvo em metros (ENU)

  this->declare_parameter<double>("altitude_threshold", -1.0);
  // Limiar de confirmação. Valor < 0 → auto: max(altitude - 0.3, 0.2)

  this->declare_parameter<bool>("use_current_xy", true);
  // true: waypoint usa X/Y da odometria atual → decolagem vertical
  // false: waypoint usa parâmetros x e y

  this->declare_parameter<double>("x", 0.0);          // X quando use_current_xy=false
  this->declare_parameter<double>("y", 0.0);          // Y quando use_current_xy=false
  this->declare_parameter<std::string>("frame_id", "map");   // frame do waypoint
  this->declare_parameter<double>("check_after_sec", 10.0);  // timeout por tentativa (s)
  this->declare_parameter<int>("max_attempts", 3);           // máximo de retentativas
  this->declare_parameter<double>("rate_hz", 10.0);          // frequência da FSM (Hz)

  // Leitura dos parâmetros (podem ter sido sobrescritos por --ros-args)
  uav_name_        = this->get_parameter("uav_name").as_string();
  takeoff_alt_     = this->get_parameter("takeoff_altitude").as_double();
  altitude_thresh_ = this->get_parameter("altitude_threshold").as_double();
  use_current_xy_  = this->get_parameter("use_current_xy").as_bool();
  fallback_x_      = this->get_parameter("x").as_double();
  fallback_y_      = this->get_parameter("y").as_double();
  frame_id_        = this->get_parameter("frame_id").as_string();
  check_after_sec_ = this->get_parameter("check_after_sec").as_double();
  max_attempts_    = this->get_parameter("max_attempts").as_int();
  double rate_hz   = this->get_parameter("rate_hz").as_double();

  if (altitude_thresh_ < 0.0) {
    altitude_thresh_ = std::max(takeoff_alt_ - 0.3, 0.2);
    // Cálculo automático: 30 cm abaixo do alvo, com piso em 0.2 m.
    // "Decolagem confirmada" não precisa atingir exatamente a altitude-alvo.
  }

  waypoints_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("/waypoints", 10);
  // /waypoints: consumido pelo my_drone_controller

  state_sub_ = this->create_subscription<mavros_msgs::msg::State>(
    "/" + uav_name_ + "/mavros/state", 10,
    std::bind(&TakeoffNode::stateCallback, this, _1));
  // Assina /uav1/mavros/state para detectar quando o FCU está conectado

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/" + uav_name_ + "/mavros/local_position/odom", 10,
    std::bind(&TakeoffNode::odomCallback, this, _1));
  // Assina odometria para leitura de posição atual (x, y, z)

  auto period_ms = std::chrono::milliseconds(static_cast<int>(1000.0 / rate_hz));
  timer_ = this->create_wall_timer(period_ms, std::bind(&TakeoffNode::timerCallback, this));
  // Timer que chama timerCallback() na frequência rate_hz (padrão 10 Hz)
}
```

### 2.3 Callbacks de dados

```cpp
void stateCallback(const mavros_msgs::msg::State::SharedPtr msg)
{
  fcu_connected_ = msg->connected;
  // msg->connected = true quando MAVROS tem comunicação com o FCU.
  // Ativado pelo TAKING_OFF da FSM quando FCU responde ao heartbeat.
}

void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  current_z_     = msg->pose.pose.position.z;   // altitude atual (ENU, m)
  odom_x_        = msg->pose.pose.position.x;   // posição X atual
  odom_y_        = msg->pose.pose.position.y;   // posição Y atual
  odom_received_ = true;   // flag: já recebemos pelo menos uma leitura
}
```

### 2.4 `timerCallback()` — dispatcher da FSM

```cpp
void timerCallback()
{
  switch (fsm_) {

    case TakeoffFSM::WAIT_FCU:
      if (!fcu_connected_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 3000,
          "Waiting for FCU connection…");
        // Throttle a cada 3 s para não poluir o terminal
        return;
      }
      RCLCPP_INFO(this->get_logger(), "FCU connected.");
      fsm_ = TakeoffFSM::PUBLISH_TAKEOFF;
      // Transição: FCU pronto → preparar para publicar waypoint
      break;

    case TakeoffFSM::PUBLISH_TAKEOFF:
      if (use_current_xy_ && !odom_received_) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
          "Waiting for odometry…");
        // Se use_current_xy=true e ainda sem odometria, aguarda.
        // Sem esta guarda, o waypoint seria publicado em (0,0,alt).
        return;
      }
      publishTakeoffWaypoint();       // publica o waypoint de decolagem
      attempt_start_ = this->now();   // registra início desta tentativa
      fsm_ = TakeoffFSM::MONITOR;     // transição para monitoramento
      break;

    case TakeoffFSM::MONITOR:
      monitorAltitude();   // verifica altitude; pode retentar ou encerrar
      break;
  }
}
```

### 2.5 `publishTakeoffWaypoint()`

```cpp
void publishTakeoffWaypoint()
{
  geometry_msgs::msg::PoseArray waypoints;
  waypoints.header.frame_id = frame_id_;       // "map"
  waypoints.header.stamp    = this->now();     // timestamp atual

  geometry_msgs::msg::Pose pose;
  pose.position.x = use_current_xy_ ? odom_x_ : fallback_x_;
  // use_current_xy_=true: decolagem vertical sobre posição atual
  // use_current_xy_=false: decolagem para coordenadas fixas
  pose.position.y = use_current_xy_ ? odom_y_ : fallback_y_;
  pose.position.z    = takeoff_alt_;   // altitude-alvo (ex.: 1.75 m)
  pose.orientation.w = 1.0;            // quaternion identidade (sem rotação)

  waypoints.poses.push_back(pose);     // array com um único waypoint
  waypoints_pub_->publish(waypoints);  // envia ao my_drone_controller
}
```

### 2.6 `monitorAltitude()`

```cpp
void monitorAltitude()
{
  if (current_z_ >= altitude_thresh_) {
    // Altitude atingida → decolagem bem-sucedida
    RCLCPP_INFO(this->get_logger(),
      "Takeoff successful: z=%.2f m >= %.2f m.", current_z_, altitude_thresh_);
    rclcpp::shutdown();   // encerra o nó; exit code 0
    return;
  }

  double elapsed = (this->now() - attempt_start_).seconds();
  // Tempo desde o início desta tentativa

  if (elapsed >= check_after_sec_) {
    attempt_count_++;
    // Incrementa e verifica se esgotou as tentativas

    if (attempt_count_ >= max_attempts_) {
      RCLCPP_ERROR(this->get_logger(),
        "Takeoff failed after %d attempt(s): z=%.2f m.", max_attempts_, current_z_);
      rclcpp::shutdown();   // falha; exit code retornado via main()
      return;
    }

    RCLCPP_WARN(this->get_logger(),
      "Altitude not reached after %.1fs. Retrying (attempt %d/%d)…",
      check_after_sec_, attempt_count_ + 1, max_attempts_);
    fsm_ = TakeoffFSM::PUBLISH_TAKEOFF;
    // Volta para PUBLISH_TAKEOFF: republica o waypoint e reinicia o timer
  }
}
```

---

## 3. `pouso.cpp`

### 3.1 Enum e struct

```cpp
enum class PousoFSM { WAIT_FCU, WAIT_ODOM, COLLECT_H, CENTER, DESCEND };
// WAIT_FCU  : aguarda connected=true no /mavros/state
// WAIT_ODOM : aguarda primeira odometria
// COLLECT_H : janela de coleta de detecções YOLO (se use_yolo_h=true)
// CENTER    : hover sobre alvo XY em approach_z até estabilizar
// DESCEND   : desce para landing_z; aborta em deriva excessiva

struct HDetection {
  rclcpp::Time stamp;   // timestamp para expiração da detecção
  double right {0.0};   // deslocamento lateral direito no frame da câmera (m)
  double front {0.0};   // deslocamento frontal no frame da câmera (m)
  double range {0.0};   // = hypot(right, front): distância total ao marcador
};
```

### 3.2 `hCallback()` — recepção de detecções YOLO

```cpp
void hCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  double right = msg->point.x;   // ponto.x = componente "direita" no frame câmera
  double front = msg->point.y;   // ponto.y = componente "frente" no frame câmera
  double range = std::hypot(right, front);   // distância total ao marcador

  if (range > max_h_range_m_) {
    return;   // detecção muito distante → ruído; descartada
  }

  if (fsm_ == PousoFSM::COLLECT_H) {
    h_collect_count_++;
    if (!has_best_h_ || range < best_collected_h_.range) {
      best_collected_h_ = {this->now(), right, front, range};
      has_best_h_ = true;
      // Guarda a detecção mais próxima (menor range) da janela de coleta.
      // Estratégia prefer_closest_h=true (padrão): usa o marcador de menor distância.
    }
  }

  h_detections_.push_back({this->now(), right, front, range});
  // Adiciona ao buffer geral (sliding window)

  rclcpp::Time now = this->now();
  h_detections_.erase(
    std::remove_if(h_detections_.begin(), h_detections_.end(),
      [&](const HDetection & d) {
        return (now - d.stamp).seconds() > h_timeout_s_;
        // Remove detecções mais antigas que h_timeout_s_ (janela deslizante)
      }),
    h_detections_.end());
  // Padrão erase-remove: eficiente para vectors; remove_if marca, erase elimina.
}
```

### 3.3 `computeLandingTarget()` — cálculo do alvo de pouso

```cpp
void computeLandingTarget()
{
  if (use_yolo_h_ && has_best_h_) {
    double yaw = current_yaw_;
    // Transformação: frame câmera (right, front) → frame map (dx, dy)
    // usando a rotação do drone (yaw atual)
    double dx = std::cos(yaw) * best_collected_h_.front
               + std::sin(yaw) * best_collected_h_.right;
    // cos(yaw)*front: projeção "frente" no eixo X do mapa
    // sin(yaw)*right: projeção "direita" no eixo X do mapa
    double dy = std::sin(yaw) * best_collected_h_.front
               - std::cos(yaw) * best_collected_h_.right;
    // sin(yaw)*front: projeção "frente" no eixo Y do mapa
    // -cos(yaw)*right: sinal negativo pela convenção da rotação ENU
    active_land_x_ = current_x_ + dx;   // alvo absoluto = posição atual + offset
    active_land_y_ = current_y_ + dy;
  } else {
    active_land_x_ = use_current_xy_ ? current_x_ : target_x_;
    active_land_y_ = use_current_xy_ ? current_y_ : target_y_;
    // Fallback: usa posição atual da odometria ou parâmetros x/y fixos
  }
}
```

### 3.4 `enterCenter()` e `runCenter()`

```cpp
void enterCenter()
{
  in_center_stable_    = false;
  // Reseta flag: a estabilidade deve ser confirmada do zero nesta entrada.
  center_stable_since_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  // Reseta timer de estabilidade (valor inválido até in_center_stable_ = true).
  publishCenterWaypoint();
  // Publica waypoint de hover em approach_z sobre active_land_x_/y_.
  fsm_ = PousoFSM::CENTER;
}

void runCenter()
{
  double dxy = std::hypot(current_x_ - active_land_x_, current_y_ - active_land_y_);
  // dxy: distância planar atual→alvo XY

  if (dxy <= xy_hold_tol_) {
    if (!in_center_stable_) {
      in_center_stable_    = true;
      center_stable_since_ = this->now();
      // Primeira vez dentro da tolerância: inicia contagem de estabilidade.
    }
    double stable_dur = (this->now() - center_stable_since_).seconds();
    if (stable_dur >= xy_hold_stable_s_) {
      // Estável por tempo suficiente → iniciar descida
      publishDescentWaypoint();
      // Publica waypoint em landing_z (altitude de solo).
      fsm_ = PousoFSM::DESCEND;
      return;
    }
    // Ainda dentro da tolerância mas tempo insuficiente: aguarda mais
  } else {
    in_center_stable_ = false;
    // Saiu da tolerância XY: reseta timer de estabilidade.
    // Na próxima vez que entrar, o contador recomeça do zero.
  }

  double elapsed = (this->now() - attempt_start_).seconds();
  if (elapsed >= check_after_sec_) {
    // Timeout global de CENTER: não conseguiu centralizar
    RCLCPP_WARN(this->get_logger(),
      "⚠️  Timeout em CENTER (%.1fs): dxy=%.3f m.", check_after_sec_, dxy);
    rclcpp::shutdown();
  }
}
```

### 3.5 `runDescend()`

```cpp
void runDescend()
{
  double dxy  = std::hypot(current_x_ - active_land_x_, current_y_ - active_land_y_);
  bool   z_ok  = (current_z_ <= landing_z_ + 0.15);
  // z_ok: chegou a 15 cm acima de landing_z (margem para suavidade)
  bool   xy_ok = (dxy <= xy_hold_tol_);

  if (z_ok && xy_ok) {
    // Pouso confirmado
    RCLCPP_INFO(this->get_logger(), "✅ Pouso concluído.");
    rclcpp::shutdown();   // exit code 0 → sucesso
    return;
  }

  if (dxy > xy_abort_tol_) {
    // Deriva excessiva durante descida → abortar e voltar a CENTER
    RCLCPP_WARN(this->get_logger(),
      "⚠️  Deriva excessiva: dxy=%.3f m > %.3f m. Voltando a CENTER.",
      dxy, xy_abort_tol_);
    enterCenter();
    // enterCenter republica waypoint de hover em approach_z:
    // o drone sobe e recentraliza antes de tentar descer novamente.
    return;
  }

  // Sem abort, sem conclusão: descida em andamento.
  // NÃO republicamos o waypoint de descida a cada tick (intencional):
  // republicar causaria reset da trajetória com jitter de XY pequeno.

  double elapsed = (this->now() - attempt_start_).seconds();
  if (elapsed >= check_after_sec_) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️  Timeout aguardando pouso: Z=%.2f m.", current_z_);
    rclcpp::shutdown();
  }
}
```

---

## 4. `drone_yaw_360.cpp`

### 4.1 Construtor

```cpp
DroneYaw360OverrideAngle() : Node("drone_yaw_360") {
  this->declare_parameter<std::string>("uav_ns", "/uav1");
  // Namespace do UAV para construção dos tópicos

  this->declare_parameter<double>("yaw_rate", 1.0);
  // Velocidade de giro em rad/s (sempre usado como |valor| internamente)

  this->declare_parameter<double>("yaw_tolerance", 0.05);
  // Margem em rad para considerar giro completo.
  // Sem margem, variações de float impediriam a conclusão exata.

  this->declare_parameter<double>("yaw_target_delta", 2*M_PI);
  // Ângulo total a girar. Positivo = CCW, negativo = CW.
  // Padrão = 2π = 360°.

  uav_ns_           = this->get_parameter("uav_ns").as_string();
  yaw_rate_         = this->get_parameter("yaw_rate").as_double();
  yaw_tolerance_    = this->get_parameter("yaw_tolerance").as_double();
  yaw_target_delta_ = this->get_parameter("yaw_target_delta").as_double();

  pub_ = this->create_publisher<drone_control::msg::YawOverride>(
    uav_ns_ + "/yaw_override/cmd", 10);
  // Publica em /uav1/yaw_override/cmd

  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    uav_ns_ + "/mavros/local_position/odom", 10,
    std::bind(&DroneYaw360OverrideAngle::odom_cb, this, std::placeholders::_1));
  // Assina odometria para extrair yaw atual

  timer_ = this->create_wall_timer(50ms,
    std::bind(&DroneYaw360OverrideAngle::timer_cb, this));
  // 50 ms = 20 Hz: boa resolução para acumular delta de yaw
}
```

### 4.2 `odom_cb()` — extração de yaw via Eigen

```cpp
void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  Eigen::Quaterniond q(
    msg->pose.pose.orientation.w,   // w: componente escalar
    msg->pose.pose.orientation.x,   // x: componente i
    msg->pose.pose.orientation.y,   // y: componente j
    msg->pose.pose.orientation.z    // z: componente k
  );
  // Construtor Eigen: (w, x, y, z) — atenção: ordem diferente do ROS (x,y,z,w)

  auto euler = q.toRotationMatrix().eulerAngles(0, 1, 2);
  // toRotationMatrix(): quaternion → matriz de rotação 3x3
  // eulerAngles(0, 1, 2): decompõe em ângulos intrínsecos ZYX:
  //   euler[0] = roll  (rotação em torno de X)
  //   euler[1] = pitch (rotação em torno de Y)
  //   euler[2] = yaw   (rotação em torno de Z) ← o que queremos

  current_yaw_ = euler[2];   // extrai o yaw

  if (!odom_ok_) {
    last_yaw_ = current_yaw_;
    // Inicializa last_yaw_ APENAS na primeira odometria.
    // Razão: se inicializasse no construtor com last_yaw_=0, o primeiro delta
    // seria current_yaw_ - 0 ≠ 0 → acumulação espúria no início.
    odom_ok_ = true;
  }
}
```

### 4.3 `timer_cb()` — acumulação e controle

```cpp
void timer_cb()
{
  if (!odom_ok_) return;   // sem odometria: aguarda

  if (!started_) {
    yaw_initial_     = current_yaw_;   // yaw de referência inicial (para logs)
    last_yaw_        = current_yaw_;   // base para o primeiro delta
    accumulated_yaw_ = 0.0;            // contador de ângulo girado
    started_ = true;
    publish_enable();   // ativa YawOverride no my_drone_controller
    return;
  }

  double delta = normalize_angle(current_yaw_ - last_yaw_);
  // delta: variação de yaw entre ticks, normalizada para [-π, π].
  // Normalização evita salto de ≈-2π quando yaw passa por ±π (wrap-around).

  double dir = (yaw_target_delta_ >= 0.0) ? 1.0 : -1.0;
  // dir: direção esperada do giro (+1 CCW, -1 CW)

  if (delta * dir < 0) delta = 0;
  // Se delta e dir têm sinais opostos: drone girou "para trás".
  // Descartamos: ruído do IMU ou oscilação momentânea.
  // Nunca subtraímos do acumulado: só contamos o progresso, não o retrocesso.

  accumulated_yaw_ += std::abs(delta);
  // Soma o módulo do delta (sempre positivo) ao acumulado.

  last_yaw_ = current_yaw_;   // atualiza base para o próximo delta

  if (accumulated_yaw_ >= std::abs(yaw_target_delta_) - yaw_tolerance_
      && !already_disabled_) {
    // accumulated ≥ alvo - tolerância: giro concluído
    publish_disable();   // desativa YawOverride e chama rclcpp::shutdown()
  }
}
```

### 4.4 `normalize_angle()`

```cpp
double normalize_angle(double a)
{
  while (a > M_PI)  a -= 2 * M_PI;
  // Se a > π: subtrai 2π; ex.: 3.5 → 3.5 - 2π ≈ -2.78 rad
  while (a < -M_PI) a += 2 * M_PI;
  // Se a < -π: adiciona 2π; ex.: -3.5 → -3.5 + 2π ≈ 2.78 rad
  return a;
  // Mantém o ângulo em [-π, π].
  // Caso de uso crítico: ao cruzar ±π, a diferença "crua" seria ≈ 2π
  // mas a rotação real foi mínima.
}
```

---

## 5. `missao_P_T.cpp`

### 5.1 Includes

```cpp
#include <rclcpp/rclcpp.hpp>    // Node, timer, logger
#include <sys/types.h>          // pid_t
#include <sys/wait.h>           // waitpid, WIFEXITED, WEXITSTATUS
#include <unistd.h>             // fork, execlp, _exit
#include <thread>               // std::thread
#include <chrono>               // std::chrono_literals (1s)
#include <atomic>               // std::atomic<bool>, std::atomic<int>
```

### 5.2 Construtor

```cpp
MissaoPTNode()
: Node("missao_P_T"),
  stop_requested_(false),   // atomic<bool>: false inicialmente
  exit_code_(0)             // atomic<int>: 0 = sucesso inicialmente
{
  timer_ = this->create_wall_timer(
    1s,   // delay de 1 s antes de começar
    std::bind(&MissaoPTNode::startMissionAsync, this));
  // Por que 1 s? Garante que o executor ROS 2 terminou de inicializar.
  // fork() durante a inicialização do DDS (rmw) pode causar deadlocks.
}

~MissaoPTNode()
{
  stop_requested_.store(true);   // sinaliza thread de missão para parar
  if (mission_thread_.joinable()) mission_thread_.join();
  // join() aguarda a thread terminar limpa antes de destruir o objeto.
}
```

### 5.3 `startMissionAsync()`

```cpp
void startMissionAsync()
{
  timer_->cancel();
  // Cancela: evita que o timer dispare novamente (single-shot).

  mission_thread_ = std::thread(&MissaoPTNode::runMission, this);
  // Lança runMission() em thread separada.
  // runMission() chama waitpid() BLOQUEANTE — não pode rodar no callback do timer,
  // pois bloquearia o executor ROS 2 e impediria logs e shutdown.
}
```

### 5.4 `run_subprocess()` — fork/exec

```cpp
static int run_subprocess(const char * executable) {
  pid_t pid = fork();
  // fork(): duplica o processo. Retorna:
  //   0   no processo filho
  //   PID do filho no processo pai
  //  -1   em caso de erro

  if (pid == 0) {
    // ── Processo filho ────────────────────────────────────────────────
    execlp("ros2", "ros2", "run", "drone_control", executable,
           static_cast<char *>(nullptr));
    // execlp: substitui a imagem do processo filho.
    // Argumentos passados: "ros2" "run" "drone_control" "<executable>"
    // static_cast<char*>(nullptr): sentinela de fim da lista de argumentos.
    // Se execlp tiver sucesso → esta linha nunca é alcançada.
    _exit(127);
    // _exit (não exit): não chama destrutores C++ do processo pai no filho.
    // 127 = código padrão de "command not found" no shell.
  }

  if (pid < 0) return -1;   // fork() falhou

  // ── Processo pai ──────────────────────────────────────────────────────
  int status = 0;
  waitpid(pid, &status, 0);
  // Bloqueia até o filho terminar.
  // Chamado de mission_thread_ (NÃO do executor) → não bloqueia o ROS 2.

  if (WIFEXITED(status)) {
    return WEXITSTATUS(status);
    // WIFEXITED: true se filho terminou normalmente (não por sinal).
    // WEXITSTATUS: extrai os 8 bits menos significativos como exit code.
  }
  return -1;   // filho morto por sinal ou outro término anormal
}
```

### 5.5 `runMission()` — sequência P→T

```cpp
void runMission()
{
  // ── FASE 1: pouso ─────────────────────────────────────────────────────
  int pouso_result = run_subprocess("pouso");
  // Bloqueia até o nó pouso terminar (pode levar dezenas de segundos)

  if (pouso_result != 0) {
    RCLCPP_ERROR(this->get_logger(),
      "❌ [FASE 1] pouso falhou com código %d. Takeoff cancelado.", pouso_result);
    exit_code_.store(1);
    rclcpp::shutdown();
    return;
    // FAIL-SAFE: nunca decolar se o pouso falhou.
  }

  // ── FASE 2: espera 10 s ────────────────────────────────────────────────
  for (int i = 10; i > 0; --i) {
    if (stop_requested_.load()) {
      // Verifica a cada segundo se foi solicitada interrupção (Ctrl+C, shutdown)
      exit_code_.store(1);
      rclcpp::shutdown();
      return;
    }
    std::this_thread::sleep_for(1s);   // dorme 1 s × 10 iterações = 10 s
  }

  // ── FASE 3: takeoff ────────────────────────────────────────────────────
  int takeoff_result = run_subprocess("takeoff");
  // Bloqueia até o nó takeoff terminar

  if (takeoff_result != 0) {
    RCLCPP_WARN(this->get_logger(),
      "⚠️  [FASE 3] takeoff encerrou com código %d.", takeoff_result);
    exit_code_.store(1);   // exit code != 0: supervisor registra aviso
  }

  rclcpp::shutdown();   // encerra o nó após a sequência completa
}
```

---

## 6. `supervisor_T.cpp`

### 6.1 Enum de estados

```cpp
enum class SupervisorState {
    INIT,                 // Dispara takeoff automático ao iniciar
    TAKING_OFF,           // Poll WNOHANG do processo takeoff
    RUN_YAW,              // Poll WNOHANG do processo drone_yaw_360
    WAIT_TRAJ,            // Aguarda sinais de trajetória
    WAIT_BEFORE_MISSION,  // Delay configurável pós-trajetória
    RUN_MISSION,          // Poll WNOHANG de missao_P_T ou pouso local
};

static constexpr int POST_RESET_COOLDOWN_TICKS = 2;
// 2 × 500 ms = 1 s de cooldown ao entrar em WAIT_TRAJ.
// Descarta sinais residuais do missao_P_T que chegam via DDS com atraso.
```

### 6.2 `odom_callback()` — rastreamento da zona base

```cpp
void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    current_x_ = msg->pose.pose.position.x;   // X atual
    current_y_ = msg->pose.pose.position.y;   // Y atual
    odom_received_ = true;

    if (use_origin_as_base_) {
        const double dist = std::hypot(current_x_, current_y_);
        // dist: distância da origem (0,0)

        if (dist <= base_tol_m_) {
            if (!base_zone_entered_) {
                base_zone_entered_    = true;
                base_zone_enter_time_ = this->now();
                // Início do timer de estabilidade da zona base.
                // O drone deve permanecer dentro de base_tol_m_ por base_hold_s_
                // para autorizar pouso local.
            }
            // Se já estava na zona: timer continua contando naturalmente.
        } else {
            if (base_zone_entered_) {
                base_zone_entered_ = false;
                // Saiu da zona: reseta o timer.
                // Próxima entrada reiniciará a contagem.
            }
        }
    }
}
```

### 6.3 Callbacks de trajetória

```cpp
void progress_callback(const std_msgs::msg::Float32::SharedPtr msg) {
    if (state_ != SupervisorState::WAIT_TRAJ) return;
    // Guard: ignora sinais fora do estado correto

    if (msg->data < 99.9f) {
        if (!trajectory_active_) {
            trajectory_active_ = true;
            trajectory_done_   = false;
            // Nova trajetória detectada; marca como ativa, reseta conclusão.
        }
        // threshold 99.9f (não 100.0f): previne reset por jitter float
    } else {
        if (!trajectory_done_) {
            trajectory_active_ = true;
            trajectory_done_   = true;
            // Progresso ≥ 99.9%: trajetória concluída.
        }
    }
}

void finished_callback(const std_msgs::msg::Bool::SharedPtr msg) {
    if (state_ != SupervisorState::WAIT_TRAJ) return;

    if (!msg->data) {
        // false: nova trajetória iniciando (reset)
        if (!trajectory_active_) {
            trajectory_active_ = true;
            trajectory_done_   = false;
        }
    } else {
        // true: trajetória concluída
        if (!trajectory_done_) {
            trajectory_active_ = true;
            trajectory_done_   = true;
        }
    }
}
```

### 6.4 `check_trajectory()`

```cpp
void check_trajectory() {
    if (post_reset_ticks_ > 0) {
        --post_reset_ticks_;
        // Decrementa cooldown a cada tick (500 ms).
        if (trajectory_active_ || trajectory_done_) {
            reset_trajectory_guards();
            // Descarta sinais que chegaram durante o cooldown
            // (residuais do missao_P_T anterior ainda na fila DDS).
        }
        return;
    }

    if (!trajectory_done_) return;   // aguarda conclusão

    // Trajetória concluída → transição para delay pré-missão
    reset_trajectory_guards();
    wait_start_time_ = this->now();
    state_ = SupervisorState::WAIT_BEFORE_MISSION;
}
```

### 6.5 `check_wait_before_mission()`

```cpp
void check_wait_before_mission() {
    const double elapsed = (this->now() - wait_start_time_).seconds();

    if (elapsed < wait_after_traj_done_s_) {
        return;   // delay ainda não expirou
    }

    // Guard: evita re-lançamento muito próximo da última missão
    if (min_relaunch_dist_m_ > 0.0 && last_mission_valid_ && odom_received_) {
        const double d = std::hypot(current_x_ - last_mission_x_,
                                    current_y_ - last_mission_y_);
        if (d < min_relaunch_dist_m_) {
            // Drone não se moveu o suficiente: ignora esta missão
            post_reset_ticks_ = POST_RESET_COOLDOWN_TICKS;
            state_ = SupervisorState::WAIT_TRAJ;
            return;
        }
    }

    // Verificar se está na zona base (e estável)
    bool at_base = false;
    if (use_origin_as_base_ && odom_received_ && base_zone_entered_) {
        const double stable_s = (this->now() - base_zone_enter_time_).seconds();
        if (stable_s >= base_hold_s_) {
            at_base = true;   // estável na zona base: pouso local autorizado
        } else {
            return;   // ainda acumulando estabilidade: aguarda
        }
    }

    // Lança executável escolhido
    child_pid_ = at_base ? fork_exec_pouso_local() : fork_exec("missao_P_T");

    if (child_pid_ > 0) {
        if (odom_received_) {
            last_mission_x_     = current_x_;   // salva posição para guard
            last_mission_y_     = current_y_;
            last_mission_valid_ = true;
        }
        state_ = SupervisorState::RUN_MISSION;
    }
    // child_pid_ <= 0: fork falhou → permanece em WAIT_BEFORE_MISSION e retenta
}
```

### 6.6 `fork_exec_pouso_local()` — pouso com argumentos dinâmicos

```cpp
pid_t fork_exec_pouso_local() {
    // Constrói strings "param:=valor" no stack antes do fork()
    char xy_hold_tol_arg[64];
    char xy_hold_stable_s_arg[64];
    char xy_abort_tol_arg[64];
    char approach_z_arg[64];

    snprintf(xy_hold_tol_arg, sizeof(xy_hold_tol_arg),
             "xy_hold_tol:=%.4f", pouso_xy_hold_tol_);
    // Ex.: "xy_hold_tol:=0.1000" — parâmetro passado ao nó pouso

    snprintf(xy_hold_stable_s_arg, sizeof(xy_hold_stable_s_arg),
             "xy_hold_stable_s:=%.4f", pouso_xy_hold_stable_s_);
    // Ex.: "xy_hold_stable_s:=1.0000"

    snprintf(xy_abort_tol_arg, sizeof(xy_abort_tol_arg),
             "xy_abort_tol:=%.4f", pouso_xy_abort_tol_);
    // Ex.: "xy_abort_tol:=0.5000"

    snprintf(approach_z_arg, sizeof(approach_z_arg),
             "approach_z:=%.4f", pouso_approach_z_);
    // Ex.: "approach_z:=-1.0000" (negativo = usar altitude atual)

    pid_t pid = fork();
    if (pid == 0) {
        execlp("ros2", "ros2", "run", "drone_control", "pouso",
               "--ros-args",
               "-p", "use_current_xy:=true",    // pousa na posição atual
               "-p", xy_hold_tol_arg,
               "-p", xy_hold_stable_s_arg,
               "-p", xy_abort_tol_arg,
               "-p", approach_z_arg,
               static_cast<char *>(nullptr));
        _exit(1);
    }
    return pid;
}
```

### 6.7 `check_mission()` — recolhimento com WNOHANG

```cpp
void check_mission() {
    int status = 0;
    pid_t result = waitpid(child_pid_, &status, WNOHANG);
    // WNOHANG: retorna imediatamente.
    //   result == 0: filho ainda em execução → retorna sem bloquear
    //   result > 0:  filho terminou → status contém o código de saída
    //   result < 0:  erro (filho já foi recolhido, ou PID inválido)

    if (result == 0) {
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
            "[RUN_MISSION] Aguardando %s (PID %d)…",
            current_child_exec_.c_str(), static_cast<int>(child_pid_));
        return;   // filho ainda rodando: volta no próximo tick (500 ms)
    }

    // Filho terminou
    child_pid_ = -1;   // limpa PID; -1 = nenhum filho ativo

    std_msgs::msg::Bool done_msg;
    done_msg.data = true;
    mission_cycle_done_pub_->publish(done_msg);
    // Publica /mission_cycle_done=true para outros nós saberem que
    // um ciclo completo de missão terminou.

    reset_trajectory_guards();
    post_reset_ticks_ = POST_RESET_COOLDOWN_TICKS;
    // Ativa cooldown de 1 s para descartar sinais residuais de trajetória
    // que podem ter sido gerados pelo pouso do missao_P_T.

    state_ = SupervisorState::WAIT_TRAJ;
    // Volta a aguardar a próxima trajetória do my_drone_controller.
}
```
