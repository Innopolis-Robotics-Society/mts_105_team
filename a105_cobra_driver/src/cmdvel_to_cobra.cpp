// # обычный режим
// ros2 launch a105_cobra_driver cobra.launch.py

// # тестовый сценарий
// ros2 run a105_cobra_driver cmdvel_to_cobra --ros-args -p port:=/dev/ttyUSB0 -p baud:=115200 -p test:=true

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <cmath>
#include <mutex>
#include <string>
#include <cstdio>
#include <cstring>

class CmdVelToCobra : public rclcpp::Node {
public:
  CmdVelToCobra() : Node("cmdvel_to_cobra") {
    // параметры
    port_       = declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_       = declare_parameter<int>("baud", 115200);
    track_m_    = declare_parameter<double>("track", 0.228);
    wheel_d_m_  = declare_parameter<double>("wheel_d", 0.0745);
    max_rpm_    = declare_parameter<double>("max_rpm", 180.0);
    rate_hz_    = declare_parameter<double>("rate_hz", 30.0);
    deadman_s_  = declare_parameter<double>("deadman_s", 0.3);
    test_       = declare_parameter<bool>("test", false);

    fd_ = open_serial(port_.c_str(), baud_);
    if (fd_ < 0) throw std::runtime_error("serial open failed");
    RCLCPP_INFO(get_logger(), "Serial %s @ %d", port_.c_str(), baud_);

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, [this](geometry_msgs::msg::Twist::SharedPtr msg){
        std::lock_guard<std::mutex> lk(m_);
        last_v_ = msg->linear.x;
        last_w_ = msg->angular.z;
        last_msg_time_ = now();
        have_cmd_ = true;
      });

    timer_ = create_wall_timer(std::chrono::milliseconds(int(1000.0 / rate_hz_)),
      std::bind(&CmdVelToCobra::tick, this));

    if (test_) {
      test_t0_ = now();
      test_timer_ = create_wall_timer(std::chrono::milliseconds(50),
        std::bind(&CmdVelToCobra::test_script, this));
    }
  }

  ~CmdVelToCobra() override { if (fd_ >= 0) ::close(fd_); }

private:
  static speed_t baud_to_speed(int b){
    switch (b) {
      case 9600: return B9600; case 19200: return B19200; case 38400: return B38400;
      case 57600: return B57600; case 115200: return B115200;
#ifdef B230400
      case 230400: return B230400;
#endif
      default: return B115200;
    }
  }
  int open_serial(const char* dev, int baud){
    int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) { RCLCPP_ERROR(get_logger(), "open(%s): %s", dev, std::strerror(errno)); return -1; }
    fcntl(fd, F_SETFL, 0); // blocking
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) { ::close(fd); return -1; }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    cfsetispeed(&tio, baud_to_speed(baud));
    cfsetospeed(&tio, baud_to_speed(baud));
    tio.c_cc[VMIN]=0; tio.c_cc[VTIME]=1; // 100ms
    if (tcsetattr(fd, TCSANOW, &tio) != 0) { ::close(fd); return -1; }
    return fd;
  }

  // cmd_vel → (L,R) в десятых об/мин
  void cmd_to_rpm10(double v, double w, int &L, int &R){
    const double r = wheel_d_m_ * 0.5;
    const double vl = v - w * (track_m_ * 0.5);
    const double vr = v + w * (track_m_ * 0.5);
    const double wl = vl / r;
    const double wr = vr / r;
    const double rpml = wl * 60.0 / (2.0*M_PI);
    const double rpmr = wr * 60.0 / (2.0*M_PI);
    auto clamp=[&](double x){ return std::max(-max_rpm_, std::min(max_rpm_, x)); };
    L = int(std::round(clamp(rpml) * 10.0));
    R = int(std::round(clamp(rpmr) * 10.0));
  }

  // JSON helpers
  void send_raw(const char* s){ ::write(fd_, s, std::strlen(s)); ::write(fd_, "\n", 1); }
  void send_lr(int L, int R){
    char buf[64];
    std::snprintf(buf, sizeof(buf), "{\"T\":1,\"L\":%d,\"R\":%d}", L, R);
    send_raw(buf);
  }
  void send_cmd13(double vx, double wz){
    char buf[96];
    std::snprintf(buf, sizeof(buf), "{\"T\":13,\"X\":%.3f,\"Z\":%.3f}", vx, wz);
    send_raw(buf);
  }
  void send_led(int io4, int io5){
    char buf[96];
    std::snprintf(buf, sizeof(buf), "{\"T\":132,\"IO4\":%d,\"IO5\":%d}", io4, io5);
    send_raw(buf);
  }

  void tick(){
    if (test_) return; // тестовый сценарий сам шлёт команды
    bool expired = (now() - last_msg_time_).seconds() > deadman_s_;
    double v=0.0, w=0.0;
    {
      std::lock_guard<std::mutex> lk(m_);
      if (!expired && have_cmd_) { v = last_v_; w = last_w_; }
    }
    int L=0, R=0;
    cmd_to_rpm10(v, w, L, R);
    send_lr(L, R);
  }

  void test_script(){
    const double t = (now() - test_t0_).seconds();
    // 0–1 c: мигаем 5 Гц
    if (t < 1.0) { int on = (int(std::floor(t*10)) & 1) ? 255 : 0; send_led(on,on); send_cmd13(0,0); return; }
    // 1–3 c: вперёд 0.3 м/с
    if (t < 3.0) { send_led(255,255); send_cmd13(0.30, 0.0); return; }
    // 3–3.5 c: стоп
    if (t < 3.5){ send_cmd13(0.0, 0.0); return; }
    // 3.5–5.5 c: назад 0.3 м/с
    if (t < 5.5){ send_cmd13(-0.30, 0.0); return; }
    // 5.5–6.0 c: стоп
    if (t < 6.0){ send_cmd13(0.0, 0.0); return; }
    // 6.0–8.0 c: поворот на месте 0.8 рад/с
    if (t < 8.0){ send_cmd13(0.0, 0.8); return; }
    // 8.0–10.0 c: поворот в другую сторону
    if (t < 10.0){ send_cmd13(0.0, -0.8); return; }
    // рестарт цикла
    send_cmd13(0.0, 0.0); send_led(0,0); test_t0_ = now();
  }

  // ROS
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::TimerBase::SharedPtr timer_, test_timer_;
  std::mutex m_;
  double last_v_{0.0}, last_w_{0.0};
  rclcpp::Time last_msg_time_{0,0,RCL_ROS_TIME};
  bool have_cmd_{false};

  // серийка и кинематика
  std::string port_; int fd_{-1}, baud_{115200};
  double track_m_{0.228}, wheel_d_m_{0.0745}, max_rpm_{180.0}, rate_hz_{30.0}, deadman_s_{0.3};
  bool test_{false}; rclcpp::Time test_t0_{0,0,RCL_ROS_TIME};
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToCobra>());
  rclcpp::shutdown();
  return 0;
}
