// combo_bridge_no_lidar.cpp
// ENV: TELEMETRY_PROTO=tcp|udp (tcp), TEL_HOST=0.0.0.0, TEL_PORT=5600
// TOPICS: /wheel/odom, /imu/data_raw, /camera/depth/{image_raw,camera_info}, /camera/color/{image_raw,camera_info}

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <unordered_map>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <chrono>

struct RConfig {
  double odom_pose_big_var = 1e-3;
  double odom_twist_var_vx = 0.05;
  double odom_twist_var_vy = 1.0;
  double odom_twist_var_wz = 0.02;

  double imu_gyro_var_wx = 1.0;
  double imu_gyro_var_wy = 1.0;
  double imu_gyro_var_wz = 0.02;
};
static RConfig g_R;

static bool read_exact(int fd, void* buf, size_t n) {
  auto* p = static_cast<uint8_t*>(buf);
  size_t got = 0;
  while (got < n) {
    ssize_t r = recv(fd, p + got, n - got, 0);
    if (r <= 0) return false;
    got += static_cast<size_t>(r);
  }
  return true;
}

static inline void yaw_to_quat(float yaw, double& qx, double& qy, double& qz, double& qw) {
  qx = 0.0; qy = 0.0;
  qz = std::sin(0.5 * yaw);
  qw = std::cos(0.5 * yaw);
}

template <typename T>
static void declare_and_get(rclcpp::Node& node, const std::string& name, T& var) {
  node.declare_parameter<T>(name, var);
  node.get_parameter(name, var);
}

static int getenv_int(const char* k, int defv) {
  const char* s = std::getenv(k);
  if (!s || !*s) return defv;
  try { return std::stoi(s); } catch (...) { return defv; }
}
static std::string getenv_str(const char* k, const char* defv) {
  const char* s = std::getenv(k);
  return (s && *s) ? std::string(s) : std::string(defv);
}

class ComboBridgeNode : public rclcpp::Node {
public:
  ComboBridgeNode() : Node("wbtg_combo_pub") {
    declare_and_get(*this, "R.odom_pose_big_var", g_R.odom_pose_big_var);
    declare_and_get(*this, "R.odom_twist_var_vx", g_R.odom_twist_var_vx);
    declare_and_get(*this, "R.odom_twist_var_vy", g_R.odom_twist_var_vy);
    declare_and_get(*this, "R.odom_twist_var_wz", g_R.odom_twist_var_wz);
    declare_and_get(*this, "R.imu_gyro_var_wx",   g_R.imu_gyro_var_wx);
    declare_and_get(*this, "R.imu_gyro_var_wy",   g_R.imu_gyro_var_wy);
    declare_and_get(*this, "R.imu_gyro_var_wz",   g_R.imu_gyro_var_wz);

    declare_and_get(*this, "camera.depth.fov_deg", depth_fov_deg_);
    declare_and_get(*this, "camera.rgb.fov_deg",   rgb_fov_deg_);
    declare_and_get(*this, "camera.depth.frame_id", depth_frame_);
    declare_and_get(*this, "camera.rgb.frame_id",   rgb_frame_);

    odom_pub_  = create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", 10);
    imu_pub_   = create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", rclcpp::SensorDataQoS());
    depth_pub_ = create_publisher<sensor_msgs::msg::Image>("/camera/depth/image_raw", rclcpp::SensorDataQoS());
    rgb_pub_   = create_publisher<sensor_msgs::msg::Image>("/camera/color/image_raw", rclcpp::SensorDataQoS());
    depth_ci_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/depth/camera_info", rclcpp::SensorDataQoS());
    rgb_ci_pub_   = create_publisher<sensor_msgs::msg::CameraInfo>("/camera/color/camera_info",  rclcpp::SensorDataQoS());

    proto_ = getenv_str("TELEMETRY_PROTO", "tcp");
    host_  = getenv_str("TEL_HOST", "0.0.0.0");
    port_  = static_cast<uint16_t>(getenv_int("TEL_PORT", 5600));

    if (proto_ == "udp") {
      setup_udp_receiver();
      RCLCPP_INFO(get_logger(), "INPUT UDP %s:%u", host_.c_str(), port_);
      timer_ = create_wall_timer(std::chrono::milliseconds(2), [this]{ this->udp_tick(); });
    } else {
      setup_tcp_listener();
      RCLCPP_INFO(get_logger(), "INPUT TCP %s:%u", host_.c_str(), port_);
      timer_ = create_wall_timer(std::chrono::milliseconds(2), [this]{ this->tcp_tick(); });
    }
  }

private:
  static sensor_msgs::msg::CameraInfo make_ci(uint32_t w, uint32_t h, double fov_deg, const rclcpp::Time& stamp, const std::string& frame){
    sensor_msgs::msg::CameraInfo ci;
    ci.header.stamp = stamp;
    ci.header.frame_id = frame;
    ci.width = w; ci.height = h;
    ci.distortion_model = "plumb_bob";
    ci.d.assign(5, 0.0);

    const double fov = fov_deg * M_PI / 180.0;
    const double fx = static_cast<double>(w) / (2.0 * std::tan(0.5 * fov));
    const double fy = fx;
    const double cx = (static_cast<double>(w) - 1.0) * 0.5;
    const double cy = (static_cast<double>(h) - 1.0) * 0.5;

    ci.k = {fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0};
    ci.r = {1.0,0.0,0.0,
            0.0,1.0,0.0,
            0.0,0.0,1.0};
    ci.p = {fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0};
    return ci;
  }

  // TCP
  void setup_tcp_listener() {
    tcp_srv_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (tcp_srv_ < 0) { perror("socket"); std::exit(1); }
    int on = 1;
    setsockopt(tcp_srv_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port_);
    if (host_ == "0.0.0.0" || host_.empty()) a.sin_addr.s_addr = htonl(INADDR_ANY);
    else {
      in_addr ip{};
      if (inet_pton(AF_INET, host_.c_str(), &ip) != 1) { perror("inet_pton"); std::exit(1); }
      a.sin_addr = ip;
    }
    if (bind(tcp_srv_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) < 0) { perror("bind"); std::exit(1); }
    if (listen(tcp_srv_, 1) < 0) { perror("listen"); std::exit(1); }
    fcntl(tcp_srv_, F_SETFL, O_NONBLOCK);
    tcp_cli_ = -1;
  }

  void tcp_tick() {
    if (tcp_cli_ < 0) {
      sockaddr_in addr{}; socklen_t alen = sizeof(addr);
      int fd = ::accept(tcp_srv_, reinterpret_cast<sockaddr*>(&addr), &alen);
      if (fd >= 0) {
        tcp_cli_ = fd;
        fcntl(tcp_cli_, F_SETFL, 0);
        RCLCPP_INFO(get_logger(), "client connected");
      }
      return;
    }
    uint32_t sz = 0;
    if (!read_exact(tcp_cli_, &sz, 4)) { ::close(tcp_cli_); tcp_cli_ = -1; RCLCPP_INFO(get_logger(), "client disconnected"); return; }
    if (sz == 0 || sz > (64u << 20)) { RCLCPP_WARN(get_logger(), "bad size %u", sz); ::close(tcp_cli_); tcp_cli_ = -1; return; }
    std::vector<uint8_t> buf(sz);
    if (!read_exact(tcp_cli_, buf.data(), sz)) { ::close(tcp_cli_); tcp_cli_ = -1; RCLCPP_INFO(get_logger(), "client disconnected"); return; }
    handle_payload(buf.data(), buf.size());
  }

  // UDP + CHNK
  void setup_udp_receiver() {
    udp_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (udp_fd_ < 0) { perror("socket"); std::exit(1); }
    int on = 1; setsockopt(udp_fd_, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));
    int rcvbuf = 4 * 1024 * 1024; setsockopt(udp_fd_, SOL_SOCKET, SO_RCVBUF, &rcvbuf, sizeof(rcvbuf));

    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port_);
    if (host_ == "0.0.0.0" || host_.empty()) a.sin_addr.s_addr = htonl(INADDR_ANY);
    else {
      in_addr ip{};
      if (inet_pton(AF_INET, host_.c_str(), &ip) != 1) { perror("inet_pton"); std::exit(1); }
      a.sin_addr = ip;
    }
    if (bind(udp_fd_, reinterpret_cast<sockaddr*>(&a), sizeof(a)) < 0) { perror("bind"); std::exit(1); }
    fcntl(udp_fd_, F_SETFL, O_NONBLOCK);
  }

  struct ChunkAgg {
    uint32_t total_len = 0;
    uint16_t count = 0;
    std::vector<std::vector<uint8_t>> parts;
    std::vector<bool> got;
    std::chrono::steady_clock::time_point t0;
  };

  void udp_tick() {
    for (;;) {
      uint8_t buf[65536];
      sockaddr_in from{}; socklen_t flen = sizeof(from);
      ssize_t n = recvfrom(udp_fd_, buf, sizeof(buf), 0, reinterpret_cast<sockaddr*>(&from), &flen);
      if (n <= 0) break;
      if (n >= 4 && std::memcmp(buf, "CHNK", 4) == 0) handle_chunk(buf, static_cast<size_t>(n));
      else handle_payload(buf, static_cast<size_t>(n));
    }
    cleanup_old_chunks();
  }

  void handle_chunk(const uint8_t* pkt, size_t len) {
    if (len < 4 + 4 + 4 + 2 + 2) return;
    const uint8_t* p = pkt + 4;
    uint32_t msg_id = 0, total = 0; uint16_t idx = 0, count = 0;
    std::memcpy(&msg_id, p + 0, 4);
    std::memcpy(&total,  p + 4, 4);
    std::memcpy(&idx,    p + 8, 2);
    std::memcpy(&count,  p + 10, 2);
    const uint8_t* data = pkt + 4 + 4 + 4 + 2 + 2;
    size_t dlen = len - (4 + 4 + 4 + 2 + 2);
    if (count == 0 || idx >= count) return;
    if (total == 0 || total > (64u << 20)) return;

    auto& ag = chunks_[msg_id];
    if (ag.parts.empty()) {
      ag.total_len = total; ag.count = count;
      ag.parts.resize(count); ag.got.assign(count, false); ag.t0 = std::chrono::steady_clock::now();
    }
    if (ag.count != count || ag.total_len != total) return;

    ag.parts[idx].assign(data, data + dlen);
    ag.got[idx] = true;

    bool complete = true;
    for (bool g : ag.got) { if (!g) { complete = false; break; } }
    if (!complete) return;

    std::vector<uint8_t> cat; cat.reserve(ag.total_len);
    for (uint16_t i = 0; i < ag.count; ++i) cat.insert(cat.end(), ag.parts[i].begin(), ag.parts[i].end());
    chunks_.erase(msg_id);
    if (cat.size() != ag.total_len) return;
    handle_payload(cat.data(), cat.size());
  }

  void cleanup_old_chunks() {
    using clock = std::chrono::steady_clock;
    const auto now = clock::now();
    const auto ttl = std::chrono::seconds(2);
    for (auto it = chunks_.begin(); it != chunks_.end(); ) {
      if (now - it->second.t0 > ttl) it = chunks_.erase(it);
      else ++it;
    }
  }

  // Parser
  void handle_payload(const uint8_t* data, size_t len) {
    if (len < 4) return;
    const rclcpp::Time stamp = this->get_clock()->now();

    // WBTG: 9 float (x,y,th, vx,vy,vth, wx,wy,wz)
    if (std::memcmp(data, "WBTG", 4) == 0) {
      if (len < 4 + 9*sizeof(float)) return;
      const float* f = reinterpret_cast<const float*>(data + 4);
      float x=f[0], y=f[1], th=f[2];
      float vx=f[3], vy=f[4], vth=f[5];
      float wx=f[6], wy=f[7], wz=f[8];

      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = "odom";
      odom.child_frame_id  = "base_link";
      double qx,qy,qz,qw; yaw_to_quat(th, qx,qy,qz,qw);
      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation.x = qx;
      odom.pose.pose.orientation.y = qy;
      odom.pose.pose.orientation.z = qz;
      odom.pose.pose.orientation.w = qw;
      std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
      odom.pose.covariance[0]  = g_R.odom_pose_big_var;
      odom.pose.covariance[7]  = g_R.odom_pose_big_var;
      odom.pose.covariance[14] = g_R.odom_pose_big_var;
      odom.pose.covariance[21] = g_R.odom_pose_big_var;
      odom.pose.covariance[28] = g_R.odom_pose_big_var;
      odom.pose.covariance[35] = g_R.odom_pose_big_var;

      odom.twist.twist.linear.x  = vx;
      odom.twist.twist.linear.y  = vy;
      odom.twist.twist.angular.z = vth;
      std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
      odom.twist.covariance[0]  = g_R.odom_twist_var_vx;
      odom.twist.covariance[7]  = g_R.odom_twist_var_vy;
      odom.twist.covariance[35] = g_R.odom_twist_var_wz;
      odom_pub_->publish(odom);

      sensor_msgs::msg::Imu imu;
      imu.header.stamp = stamp;
      imu.header.frame_id = "imu_link";
      imu.orientation_covariance[0] = -1.0;
      imu.linear_acceleration_covariance[0] = -1.0;
      imu.angular_velocity.x = wx;
      imu.angular_velocity.y = wy;
      imu.angular_velocity.z = wz;
      imu.angular_velocity_covariance = {
        g_R.imu_gyro_var_wx, 0.0, 0.0,
        0.0, g_R.imu_gyro_var_wy, 0.0,
        0.0, 0.0, g_R.imu_gyro_var_wz
      };
      imu_pub_->publish(imu);
      return;
    }

    // WBTD: depth (dw,dh,ds,minR,maxR) + payload
    if (std::memcmp(data, "WBTD", 4) == 0) {
      auto le16 = [](const uint8_t* p){ uint16_t v; std::memcpy(&v,p,2); return v; };
      auto le32 = [](const uint8_t* p){ uint32_t v; std::memcpy(&v,p,4); return v; };
      auto le32f= [](const uint8_t* p){ float    v; std::memcpy(&v,p,4); return v; };
      if (len < 4 + 2 + 2 + 4 + 4 + 4) return;

      const uint16_t dw  = le16(data + 4);
      const uint16_t dh  = le16(data + 6);
      const int32_t  ds  = static_cast<int32_t>(le32(data + 8));   // bytes per pixel (2 or 4)
      const float    minR= le32f(data + 12);
      const float    maxR= le32f(data + 16);

      const size_t hdr = 4 + 2 + 2 + 4 + 4 + 4;
      const size_t payload = static_cast<size_t>(dw) * static_cast<size_t>(dh) * static_cast<size_t>(ds);
      if (hdr + payload > len) return;
      const uint8_t* pimg = data + hdr;

      sensor_msgs::msg::Image img;
      img.header.stamp = stamp;
      img.header.frame_id = depth_frame_;
      img.width = dw; img.height = dh;
      img.is_bigendian = 0;

      if (ds == 2) {
        img.encoding = "16UC1";
        img.step = static_cast<uint32_t>(dw) * 2u;
        img.data.assign(pimg, pimg + payload);
      } else if (ds == 4) {
        img.encoding = "32FC1";
        img.step = static_cast<uint32_t>(dw) * 4u;
        img.data.assign(pimg, pimg + payload);
      } else {
        RCLCPP_WARN(this->get_logger(), "Unsupported depth BPP ds=%d", ds);
        return;
      }

      depth_pub_->publish(img);
      depth_ci_pub_->publish(make_ci(dw, dh, depth_fov_deg_, stamp, depth_frame_));
      return;
    }

    // WBTR: BGRA8
    if (std::memcmp(data, "WBTR", 4) == 0) {
      if (len < 4 + 2 + 2 + 4) return;
      uint16_t dw, dh; int32_t ds;
      std::memcpy(&dw, data + 4, 2);
      std::memcpy(&dh, data + 6, 2);
      std::memcpy(&ds, data + 8, 4);
      size_t hdr = 4 + 2 + 2 + 4;
      size_t need = hdr + static_cast<size_t>(dw) * dh * 4u;
      if (need > len) return;

      sensor_msgs::msg::Image img;
      img.header.stamp = stamp;
      img.header.frame_id = rgb_frame_;
      img.width = dw; img.height = dh;
      img.encoding = "bgra8";
      img.is_bigendian = 0;
      img.step = static_cast<sensor_msgs::msg::Image::_step_type>(dw * 4u);
      const uint8_t* pimg = data + hdr;
      img.data.assign(pimg, pimg + (dw * dh * 4u));
      rgb_pub_->publish(img);
      rgb_ci_pub_->publish(make_ci(dw, dh, rgb_fov_deg_, stamp, rgb_frame_));
      return;
    }
  }

  // pubs
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr depth_ci_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_ci_pub_;

  double depth_fov_deg_{87.0}, rgb_fov_deg_{87.0};
  std::string depth_frame_{"depth_camera"};
  std::string rgb_frame_{"rgb_camera"};

  std::string proto_;
  std::string host_;
  uint16_t port_{5600};

  int tcp_srv_{-1};
  int tcp_cli_{-1};
  int udp_fd_{-1};
  std::unordered_map<uint32_t, ChunkAgg> chunks_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ComboBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
