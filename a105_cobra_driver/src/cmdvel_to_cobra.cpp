// cmdvel_to_cobra.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <cmath>
#include <mutex>
#include <string>
#include <thread>
#include <atomic>
#include <vector>
#include <cstring>
#include <chrono>

class CmdVelToCobra : public rclcpp::Node {
public:
  CmdVelToCobra() : Node("cmdvel_to_cobra") {
    // --- params ---
    port_       = declare_parameter<std::string>("port", "/dev/ttyACM1");
    baud_       = declare_parameter<int>("baud", 115200);
    track_      = declare_parameter<double>("track", 0.228);     // м
    tpm_        = declare_parameter<double>("tpm", 1000.0);      // ticks per meter
    rate_hz_    = declare_parameter<double>("rate_hz", 30.0);
    poll_hz_    = declare_parameter<double>("poll_hz", 5.0);
    deadman_s_  = declare_parameter<double>("deadman_s", 0.3);
    odom_frame_ = declare_parameter<std::string>("odom_frame", "odom");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");

    // --- serial open raw ---
    fd_ = open_serial(port_.c_str(), baud_);
    if (fd_ < 0) throw std::runtime_error("serial open failed");
    RCLCPP_INFO(get_logger(), "Serial %s @ %d (raw, no EOL)", port_.c_str(), baud_);

    // --- init controller (чистый JSON, БЕЗ \r\n) ---
    send_json("{\"T\":131,\"cmd\":1}");            // telemetry on
    send_json("{\"T\":142,\"cmd\":200}");          // telemetry period ms
    send_json("{\"T\":143,\"cmd\":0}");            // echo off
    send_json("{\"T\":12}");                       // motors enable

    // --- ROS I/O ---
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg){
        std::lock_guard<std::mutex> lk(m_);
        last_v_ = msg->linear.x;
        last_w_ = msg->angular.z;
        last_cmd_time_ = now();
        have_cmd_ = true;
      });

    // --- timers ---
    cmd_timer_ = create_wall_timer(std::chrono::milliseconds(int(1000.0/rate_hz_)),
                  std::bind(&CmdVelToCobra::tick_cmd, this));
    poll_timer_ = create_wall_timer(std::chrono::milliseconds(int(1000.0/std::max(0.1, poll_hz_))),
                  std::bind(&CmdVelToCobra::tick_poll, this));

    // --- RX thread ---
    rx_run_.store(true);
    rx_thread_ = std::thread([this]{ rx_loop(); });
  }

  ~CmdVelToCobra() override {
    rx_run_.store(false);
    if (rx_thread_.joinable()) rx_thread_.join();
    if (fd_ >= 0) ::close(fd_);
  }

private:
  // -------- serial ----------
  static speed_t baud_to_speed(int b){
    switch(b){ case 9600: return B9600; case 19200: return B19200; case 38400: return B38400;
      case 57600: return B57600; case 115200: return B115200;
#ifdef B230400
      case 230400: return B230400;
#endif
      default: return B115200; }
  }
  int open_serial(const char* dev, int baud){
    int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0){ RCLCPP_ERROR(get_logger(), "open(%s): %s", dev, std::strerror(errno)); return -1; }
    fcntl(fd, F_SETFL, 0); // blocking
    termios tio{};
    if (tcgetattr(fd, &tio) != 0){ ::close(fd); return -1; }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    cfsetispeed(&tio, baud_to_speed(baud));
    cfsetospeed(&tio, baud_to_speed(baud));
    tio.c_cc[VMIN]=0; tio.c_cc[VTIME]=1; // 100ms read timeout
    if (tcsetattr(fd, TCSANOW, &tio) != 0){ ::close(fd); return -1; }
    return fd;
  }
  void send_json(const std::string& s){
    // строго только JSON. НИ одного \r или \n.
    RCLCPP_INFO(get_logger(), "[TX] %s", s.c_str());
    ssize_t n = ::write(fd_, s.data(), s.size());
    (void)n;
  }

  // -------- TX logic ----------
  void tick_cmd(){
    const bool expired = (now() - last_cmd_time_).seconds() > deadman_s_;
    double v=0.0, w=0.0;
    {
      std::lock_guard<std::mutex> lk(m_);
      if (!expired && have_cmd_){ v = last_v_; w = last_w_; }
    }
    if (expired && (last_deadman_state_.exchange(true) == false)) {
      RCLCPP_WARN(get_logger(), "Deadman: no cmd_vel for %.2fs → sending zeros", deadman_s_);
    }
    if (!expired && (last_deadman_state_.exchange(false) == true)) {
      RCLCPP_INFO(get_logger(), "Deadman cleared: cmd_vel active");
    }

    char buf[96];
    std::snprintf(buf, sizeof(buf), "{\"T\":13,\"X\":%.3f,\"Z\":%.3f}", v, w);
    send_json(buf);
  }
  void tick_poll(){
    send_json(std::string("{\"T\":130}")); // разовый запрос статуса
  }

  // -------- RX parsing + ODOM ----------
  static bool extract_int(const std::string& s, const char* key, int &out){
    std::string pat = std::string("\"")+key+"\":";
    size_t p = s.find(pat);
    if (p==std::string::npos) return false;
    p += pat.size();
    while (p<s.size() && (s[p]==' ')) ++p;
    char* end=nullptr;
    long v = std::strtol(s.c_str()+p, &end, 10);
    if (end==nullptr) return false;
    out = static_cast<int>(v);
    return true;
  }

  void rx_loop(){
    std::string buf; buf.reserve(4096);
    auto last = now();
    while (rx_run_.load()){
      char tmp[512];
      ssize_t r = ::read(fd_, tmp, sizeof(tmp));
      if (r <= 0) continue;
      buf.append(tmp, tmp + r);

      // вычленяем полноценные JSON-объекты по балансировке {}
      size_t i=0;
      while (i < buf.size()){
        while (i<buf.size() && buf[i] != '{') ++i;
        if (i>=buf.size()) break;
        int depth = 0;
        size_t j = i;
        for (; j<buf.size(); ++j){
          if (buf[j]=='{') ++depth;
          else if (buf[j]=='}'){
            --depth;
            if (depth==0){
              std::string js = buf.substr(i, j-i+1);
              // --- RAW RX LOG ---
              RCLCPP_INFO(get_logger(), "[RX] %s", js.c_str());
              handle_json(js, last);
              i = j+1;
              break;
            }
          }
        }
        if (j>=buf.size()) break;
      }
      if (i>0) buf.erase(0, i);
      if (buf.size() > 16384) buf.erase(0, buf.size()-1024);
    }
  }

  void handle_json(const std::string& js, rclcpp::Time &last_time){
    int tcode=0;
    if (!extract_int(js, "T", tcode)) return;

    // лог интересных пакетов сводкой
    if (tcode == 1001) {
      int odl=0, odr=0;
      if (!extract_int(js, "odl", odl)) return;
      if (!extract_int(js, "odr", odr)) return;

      const auto tnow = now();
      const double dt = (tnow - last_time).seconds();
      if (dt <= 0.0) { last_time = tnow; return; }

      const double Dl = (odl - last_left_ticks_) / tpm_;
      const double Dr = (odr - last_right_ticks_) / tpm_;
      last_left_ticks_  = odl;
      last_right_ticks_ = odr;

      if (!have_odo_){ have_odo_ = true; last_time = tnow; return; }

      const double dS = 0.5 * (Dl + Dr);
      const double dTh = (Dr - Dl) / track_;
      const double th_mid = yaw_ + 0.5 * dTh;
      x_   += dS * std::cos(th_mid);
      y_   += dS * std::sin(th_mid);
      yaw_ += dTh;

      const double v = dS / dt;
      const double w = dTh / dt;

      // опциональные поля телеметрии
      int m1=0,m2=0,m3=0,m4=0, vraw=0;
      bool has_m = extract_int(js, "M1", m1) | extract_int(js, "M2", m2) |
                   extract_int(js, "M3", m3) | extract_int(js, "M4", m4);
      bool has_v = extract_int(js, "v", vraw);

      RCLCPP_INFO(get_logger(),
        "[ODOM] x=%.3f y=%.3f yaw=%.3f  v=%.3f m/s w=%.3f rad/s  ticks(L,R)=(%d,%d)%s%s",
        x_, y_, yaw_, v, w, odl, odr,
        has_m ? " M[*]=yes" : "",
        has_v ? " vraw=yes" : "");

      nav_msgs::msg::Odometry od;
      od.header.stamp = tnow;
      od.header.frame_id = odom_frame_;
      od.child_frame_id  = base_frame_;
      od.pose.pose.position.x = x_;
      od.pose.pose.position.y = y_;
      od.pose.pose.position.z = 0.0;
      double half = 0.5 * yaw_;
      od.pose.pose.orientation.z = std::sin(half);
      od.pose.pose.orientation.w = std::cos(half);
      od.twist.twist.linear.x  = v;
      od.twist.twist.angular.z = w;
      odom_pub_->publish(od);

      last_time = tnow;
      return;
    }

    if (tcode == 1000 || tcode == 130) {
      // статус/пинг — просто отметим
      RCLCPP_INFO(get_logger(), "[STAT] packet T=%d", tcode);
      return;
    }
  }

  // -------- helpers ----------
  rclcpp::Time now() { return get_clock()->now(); }

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr cmd_timer_, poll_timer_;

  // cmd state
  std::mutex m_;
  double last_v_{0.0}, last_w_{0.0};
  std::atomic<bool> last_deadman_state_{false};
  rclcpp::Time last_cmd_time_{0,0,RCL_ROS_TIME};
  bool have_cmd_{false};

  // odom state
  std::atomic<bool> rx_run_{false};
  std::thread rx_thread_;
  int fd_{-1};
  int last_left_ticks_{0}, last_right_ticks_{0};
  bool have_odo_{false};
  double x_{0.0}, y_{0.0}, yaw_{0.0};

  // params
  std::string port_, odom_frame_, base_frame_;
  int baud_{115200};
  double track_{0.228}, tpm_{1000.0}, rate_hz_{30.0}, poll_hz_{5.0}, deadman_s_{0.3};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToCobra>());
  rclcpp::shutdown();
  return 0;
}
