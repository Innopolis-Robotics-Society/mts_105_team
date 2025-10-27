// src/cmdvel_simple_cobra.cpp
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <string>
#include <thread>
#include <mutex>
#include <vector>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <chrono>
#include <cerrno>
#include <cstring>

class CmdVelToCobra : public rclcpp::Node {
public:
  CmdVelToCobra() : Node("cmdvel_to_cobra") {
    port_  = declare_parameter<std::string>("port", "/dev/ttyACM1");
    baud_  = declare_parameter<int>("baud", 115200);
    track_ = declare_parameter<double>("track", 0.228);
    tpm_   = declare_parameter<double>("tpm", 1000.0);
    rate_  = declare_parameter<double>("rate", 10.0);

    fd_ = open_serial(port_.c_str(), baud_);
    if (fd_ < 0) throw std::runtime_error("serial open failed");
    RCLCPP_INFO(get_logger(), "Opened %s @ %d", port_.c_str(), baud_);
    ::tcflush(fd_, TCIFLUSH);

    // Инициализация контроллера
    send_json("{\"T\":131,\"cmd\":1}");   // telemetry on
    send_json("{\"T\":142,\"cmd\":200}"); // 200 ms period
    send_json("{\"T\":143,\"cmd\":0}");   // echo off
    send_json("{\"T\":12}");              // enable motors
    send_json("{\"T\":3,\"lineNum\":0,\"Text\":\":-)\"}");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // ROS I/O
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&CmdVelToCobra::cmd_cb, this, std::placeholders::_1));

    timer_ = create_wall_timer(
      std::chrono::milliseconds(int(1000.0 / rate_)),
      std::bind(&CmdVelToCobra::send_cmd, this));

    rx_thread_ = std::thread([this]{ rx_loop(); });
  }

  ~CmdVelToCobra() override {
    run_rx_ = false;
    if (rx_thread_.joinable()) rx_thread_.join();
    if (fd_ >= 0) ::close(fd_);
  }

private:
  // --- Serial setup ---
  static speed_t baud_to_speed(int b) {
    switch(b){
      case 9600: return B9600;
      case 19200: return B19200;
      case 38400: return B38400;
      case 57600: return B57600;
      case 115200: return B115200;
      default: return B115200;
    }
  }

  int open_serial(const char* dev, int baud){
    int fd = ::open(dev, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s): %s", dev, std::strerror(errno));
      return -1;
    }
    fcntl(fd, F_SETFL, 0);
    termios tio{};
    if (tcgetattr(fd, &tio) != 0) { ::close(fd); return -1; }
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    cfsetispeed(&tio, baud_to_speed(baud));
    cfsetospeed(&tio, baud_to_speed(baud));
    tio.c_cc[VMIN] = 0; tio.c_cc[VTIME] = 1; // 100 ms
    if (tcsetattr(fd, TCSANOW, &tio) != 0) { ::close(fd); return -1; }
    return fd;
  }

  void send_json(const std::string& s){
    ::write(fd_, s.data(), s.size());
    ::write(fd_, "\r\n", 2); // контроллер принимает CRLF
    RCLCPP_INFO(get_logger(), "[TX] %s", s.c_str());
  }

  // --- Helpers: JSON ---
  static bool extract_int(const std::string& s, const char* key, int &out){
    std::string p = std::string("\"")+key+"\":";
    size_t i = s.find(p);
    if (i == std::string::npos) return false;
    i += p.size();
    out = std::atoi(s.c_str()+i);
    return true;
  }

  void drain_json_objects(std::string &buf, std::vector<std::string> &out){
    bool in_str = false, esc = false;
    int depth = 0;
    size_t start = std::string::npos;
    for (size_t i=0; i<buf.size(); ++i){
      char c = buf[i];
      if (in_str){
        if (esc) esc = false;
        else if (c == '\\') esc = true;
        else if (c == '"') in_str = false;
        continue;
      }
      if (c == '"'){ in_str = true; continue; }
      if (c == '{'){
        if (depth == 0) start = i;
        ++depth;
      } else if (c == '}'){
        --depth;
        if (depth == 0 && start != std::string::npos){
          out.emplace_back(buf.substr(start, i - start + 1));
          start = std::string::npos;
        }
      }
    }
    if (!out.empty()){
      const std::string &last = out.back();
      size_t endpos = buf.rfind(last);
      if (endpos != std::string::npos) buf.erase(0, endpos + last.size());
    } else if (buf.size() > 65536){
      buf.erase(0, buf.size() - 1024);
    }
  }

  rclcpp::Time now(){ return this->get_clock()->now(); }

  // --- ROS cmd_vel ---
  void cmd_cb(const geometry_msgs::msg::Twist::SharedPtr msg){
    std::lock_guard<std::mutex> lk(m_);
    last_v_ = msg->linear.x;
    last_w_ = msg->angular.z;
  }

  void send_cmd(){
    double v, w;
    { std::lock_guard<std::mutex> lk(m_); v = last_v_; w = last_w_; }
    char buf[96];
    std::snprintf(buf, sizeof(buf), "{\"T\":13,\"X\":%.3f,\"Z\":%.3f}", v, w);
    send_json(buf);
  }

  // --- RX ---
  void rx_loop(){
    std::string buf; buf.reserve(8192);
    auto last_t = now();
    while (run_rx_){
      char tmp[512];
      ssize_t n = ::read(fd_, tmp, sizeof(tmp));
      if (n > 0){
        buf.append(tmp, tmp + n);
        std::vector<std::string> objs;
        drain_json_objects(buf, objs);
        for (auto &js : objs) handle_json(js, last_t);
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      }
    }
  }

  void handle_json(const std::string& js, rclcpp::Time &last_t){
    RCLCPP_INFO(get_logger(), "[RX] %s", js.c_str());

    int t=0;
    if (!extract_int(js, "T", t)) return;
    if (t != 1001) return;

    int odl=0, odr=0;
    if (!extract_int(js, "odl", odl)) return;
    if (!extract_int(js, "odr", odr)) return;

    auto tnow = now();
    if (!odom_ready_){
      last_left_ = odl; last_right_ = odr;
      odom_ready_ = true; last_t = tnow;
      return;
    }

    double dt = (tnow - last_t).seconds();
    if (dt <= 0) return;

    double Dl = (odl - last_left_) / tpm_;
    double Dr = (odr - last_right_) / tpm_;
    last_left_ = odl; last_right_ = odr;

    double dS  = 0.5*(Dl+Dr);
    double dTh = (Dr-Dl)/track_;
    double th_mid = yaw_ + 0.5*dTh;
    x_ += dS*std::cos(th_mid);
    y_ += dS*std::sin(th_mid);
    yaw_ += dTh;

    nav_msgs::msg::Odometry od;
    od.header.stamp = tnow;
    od.header.frame_id = "odom";
    od.child_frame_id  = "base_link";
    od.pose.pose.position.x = x_;
    od.pose.pose.position.y = y_;
    double half = 0.5*yaw_;
    od.pose.pose.orientation.z = std::sin(half);
    od.pose.pose.orientation.w = std::cos(half);
    od.twist.twist.linear.x  = dS/dt;
    od.twist.twist.angular.z = dTh/dt;
    odom_pub_->publish(od);

    last_t = tnow;
  }

  // --- members ---
  int fd_{-1};
  std::string port_;
  int baud_{115200};
  double track_{0.228}, tpm_{1000.0}, rate_{10.0};

  std::mutex m_;
  double last_v_{0.0}, last_w_{0.0};

  std::thread rx_thread_;
  bool run_rx_{true};

  bool odom_ready_{false};
  int last_left_{0}, last_right_{0};
  double x_{0}, y_{0}, yaw_{0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CmdVelToCobra>());
  rclcpp::shutdown();
  return 0;
}
