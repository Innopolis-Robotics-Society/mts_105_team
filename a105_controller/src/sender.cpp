#include <memory>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <cstdlib>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

class Sender : public rclcpp::Node {
public:
  Sender() : Node("cmd_vel_sim_sender") {
    const char* h = std::getenv("CMD_HOST"); host_ = h ? h : "127.0.0.1";
    const char* p = std::getenv("CMD_PORT"); port_ = p ? std::stoi(p) : 5555;

    sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);        // UDP
    addr_.sin_family = AF_INET;
    addr_.sin_port   = htons(port_);
    ::inet_pton(AF_INET, host_.c_str(), &addr_.sin_addr);

    sub_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      [this](geometry_msgs::msg::Twist::SharedPtr m){ send(m->linear.x, m->angular.z); });
  }

  ~Sender(){ if (sock_>=0) ::close(sock_); }

private:
  void send(float v, float w){
    RCLCPP_INFO(this->get_logger(), "angular z=%.3f  linear x=%.3f", w, v);
    unsigned char buf[8];
    std::memcpy(buf+0, &v, 4);
    std::memcpy(buf+4, &w, 4);
    ::sendto(sock_, buf, 8, 0, (sockaddr*)&addr_, sizeof(addr_));
}

  int sock_{-1};
  std::string host_;
  int port_{};
  sockaddr_in addr_{};
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Sender>());
  rclcpp::shutdown();
  return 0;
}
