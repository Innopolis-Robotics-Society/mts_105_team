#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <vector>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <string>

// ---------------------- R-config (ковариации) ----------------------
struct RConfig {
  // Odometry pose
  double odom_pose_big_var = 1e-3;

  // Odometry twist: [vx, vy, wz]
  double odom_twist_var_vx = 0.05;
  double odom_twist_var_vy = 1.0;
  double odom_twist_var_wz = 0.02;

  // IMU gyro: [wx, wy, wz]
  double imu_gyro_var_wx = 1.0;
  double imu_gyro_var_wy = 1.0;
  double imu_gyro_var_wz = 0.02;

  // IMU accel: [ax, ay, az]  (НОВОЕ)
  double imu_acc_var_ax = 0.2;
  double imu_acc_var_ay = 0.2;
  double imu_acc_var_az = 0.2;

  // lidar
  float scan_range_min = 0.02f;
  float scan_range_max = 8.0f;
  float scan_fov_deg   = 360.0f;
  float scan_rate_hz   = 15.0f;
};

static RConfig g_R;
// ------------------------------------------------------------------

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

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("wbtg_combo_pub");

  // Параметры для ковариаций
  declare_and_get(*node, "R.odom_pose_big_var",    g_R.odom_pose_big_var);
  declare_and_get(*node, "R.odom_twist_var_vx",    g_R.odom_twist_var_vx);
  declare_and_get(*node, "R.odom_twist_var_vy",    g_R.odom_twist_var_vy);
  declare_and_get(*node, "R.odom_twist_var_wz",    g_R.odom_twist_var_wz);
  declare_and_get(*node, "R.imu_gyro_var_wx",      g_R.imu_gyro_var_wx);
  declare_and_get(*node, "R.imu_gyro_var_wy",      g_R.imu_gyro_var_wy);
  declare_and_get(*node, "R.imu_gyro_var_wz",      g_R.imu_gyro_var_wz);
  declare_and_get(*node, "R.imu_acc_var_ax",       g_R.imu_acc_var_ax); // NEW
  declare_and_get(*node, "R.imu_acc_var_ay",       g_R.imu_acc_var_ay); // NEW
  declare_and_get(*node, "R.imu_acc_var_az",       g_R.imu_acc_var_az); // NEW
  declare_and_get(*node, "R.scan_range_min",       g_R.scan_range_min);
  declare_and_get(*node, "R.scan_range_max",       g_R.scan_range_max);
  declare_and_get(*node, "R.scan_fov_deg",         g_R.scan_fov_deg);
  declare_and_get(*node, "R.scan_rate_hz",         g_R.scan_rate_hz);

  // Publishers
  auto odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("/wheel/odom", rclcpp::QoS(10));
  auto imu_pub  = node->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", rclcpp::SensorDataQoS());
  auto scan_pub = node->create_publisher<sensor_msgs::msg::LaserScan>("/scan", rclcpp::SensorDataQoS());

  // TCP endpoint
  const char* h = std::getenv("TEL_HOST");
  std::string host = h ? h : "0.0.0.0";
  const char* p = std::getenv("TEL_PORT");
  int port = p ? std::stoi(p) : 5600;

  int srv = socket(AF_INET, SOCK_STREAM, 0);
  if (srv < 0) { perror("socket"); return 1; }
  int on = 1;
  setsockopt(srv, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

  sockaddr_in a{};
  a.sin_family = AF_INET;
  a.sin_port   = htons(static_cast<uint16_t>(port));
  in_addr ip{};
  if (host == "0.0.0.0" || host.empty()) a.sin_addr.s_addr = htonl(INADDR_ANY);
  else {
    if (inet_pton(AF_INET, host.c_str(), &ip) != 1) { perror("inet_pton"); return 1; }
    a.sin_addr = ip;
  }
  if (bind(srv, reinterpret_cast<sockaddr*>(&a), sizeof(a)) < 0) { perror("bind"); return 1; }
  if (listen(srv, 1) < 0) { perror("listen"); return 1; }

  RCLCPP_INFO(node->get_logger(), "TCP %s:%d", host.c_str(), port);

  while (rclcpp::ok()) {
    int fd = accept(srv, nullptr, nullptr);
    if (fd < 0) continue;
    RCLCPP_INFO(node->get_logger(), "client connected");

    while (rclcpp::ok()) {
      uint32_t sz = 0;
      if (!read_exact(fd, &sz, 4)) break;

      // минимальный размер: WBTA => 4(magic)+12*4 floats +4(N) = 56; WBTG => 44
      if (sz < 44) { RCLCPP_WARN(node->get_logger(), "packet too small: %u", sz); break; }

      std::vector<uint8_t> buf(sz);
      if (!read_exact(fd, buf.data(), sz)) break;

      // magic
      if (sz < 8) { RCLCPP_WARN(node->get_logger(), "packet too small for magic"); break; }
      char magic[5] = {0,0,0,0,0};
      std::memcpy(magic, buf.data(), 4);

      // формат
      int m_floats = 0; // число float перед полем N
      bool has_acc = false;
      if (std::memcmp(magic, "WBTA", 4) == 0) { m_floats = 12; has_acc = true; }
      else if (std::memcmp(magic, "WBTG", 4) == 0) { m_floats = 9; has_acc = false; }
      else {
        RCLCPP_WARN(node->get_logger(), "unknown magic: '%.4s'", magic);
        break;
      }

      size_t header_floats_bytes = static_cast<size_t>(m_floats) * sizeof(float);
      if (sz < 4 + header_floats_bytes + 4) {
        RCLCPP_WARN(node->get_logger(), "packet too small for header floats: %u", sz);
        break;
      }

      // извлечь float[]
      float f[12] = {0};
      std::memcpy(f, buf.data() + 4, header_floats_bytes);

      // распаковка
      float x   = f[0], y   = f[1], th  = f[2];
      float vx  = f[3], vy  = f[4], vth = f[5];
      float ax  = 0.0f, ay  = 0.0f, az  = 0.0f;
      float wx, wy, wz;

      if (has_acc) {
        ax = f[6]; ay = f[7]; az = f[8];
        wx = f[9]; wy = f[10]; wz = f[11];
      } else {
        wx = f[6]; wy = f[7];  wz = f[8];
      }

      // N и массив дальностей
      uint32_t N = 0;
      size_t off_N = 4 + header_floats_bytes;
      std::memcpy(&N, buf.data() + off_N, 4);

      size_t need = off_N + 4ull + static_cast<size_t>(N) * sizeof(float);
      if (need > buf.size()) {
        RCLCPP_WARN(node->get_logger(), "bad ranges size: N=%u, packet=%u", N, sz);
        break;
      }
      const float* ranges = reinterpret_cast<const float*>(buf.data() + off_N + 4);

      auto finite = [](float v){ return std::isfinite(v); };
      if (!(finite(x) && finite(y) && finite(th) &&
            finite(vx) && finite(vth) &&
            finite(wx) && finite(wy) && finite(wz))) {
        RCLCPP_WARN(node->get_logger(), "NaN/Inf in incoming data, dropping frame");
        continue;
      }

      const rclcpp::Time stamp = node->get_clock()->now();

      // ---------------- /wheel/odom ----------------
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = stamp;
      odom.header.frame_id = "odom";
      odom.child_frame_id  = "base_link";

      double qx, qy, qz, qw;
      yaw_to_quat(th, qx, qy, qz, qw);

      std::fill(odom.pose.covariance.begin(), odom.pose.covariance.end(), 0.0);
      odom.pose.covariance[0]  = g_R.odom_pose_big_var;
      odom.pose.covariance[7]  = g_R.odom_pose_big_var;
      odom.pose.covariance[14] = g_R.odom_pose_big_var;
      odom.pose.covariance[21] = g_R.odom_pose_big_var;
      odom.pose.covariance[28] = g_R.odom_pose_big_var;
      odom.pose.covariance[35] = g_R.odom_pose_big_var;

      odom.pose.pose.position.x = x;
      odom.pose.pose.position.y = y;
      odom.pose.pose.position.z = 0.0;
      odom.pose.pose.orientation.x = qx;
      odom.pose.pose.orientation.y = qy;
      odom.pose.pose.orientation.z = qz;
      odom.pose.pose.orientation.w = qw;

      std::fill(odom.twist.covariance.begin(), odom.twist.covariance.end(), 0.0);
      odom.twist.covariance[0]  = g_R.odom_twist_var_vx;
      odom.twist.covariance[7]  = g_R.odom_twist_var_vy;
      odom.twist.covariance[35] = g_R.odom_twist_var_wz;

      odom.twist.twist.linear.x  = vx;
      odom.twist.twist.linear.y  = vy;
      odom.twist.twist.linear.z  = 0.0;
      odom.twist.twist.angular.x = 0.0;
      odom.twist.twist.angular.y = 0.0;
      odom.twist.twist.angular.z = vth;
      odom_pub->publish(odom);

      // ---------------- /imu/data_raw ----------------
      sensor_msgs::msg::Imu imu;
      imu.header.stamp = stamp;
      imu.header.frame_id = "imu_link";

      // ориентацию не даём
      imu.orientation_covariance[0] = -1.0;

      imu.angular_velocity.x = wx;
      imu.angular_velocity.y = wy;
      imu.angular_velocity.z = wz;
      imu.angular_velocity_covariance = {
        g_R.imu_gyro_var_wx, 0.0, 0.0,
        0.0, g_R.imu_gyro_var_wy, 0.0,
        0.0, 0.0, g_R.imu_gyro_var_wz
      };

      // ЛИНЕЙНЫЕ УСКОРЕНИЯ — НОВОЕ
      imu.linear_acceleration.x = std::isfinite(ax) ? ax : 0.0;
      imu.linear_acceleration.y = std::isfinite(ay) ? ay : 0.0;
      imu.linear_acceleration.z = std::isfinite(az) ? az : 0.0;
      imu.linear_acceleration_covariance = {
        g_R.imu_acc_var_ax, 0.0, 0.0,
        0.0, g_R.imu_acc_var_ay, 0.0,
        0.0, 0.0, g_R.imu_acc_var_az
      };

      imu_pub->publish(imu);

      // ---------------- /scan ----------------
      sensor_msgs::msg::LaserScan scan;
      scan.header.stamp = stamp;
      scan.header.frame_id = "lidar_link";
      scan.range_min = g_R.scan_range_min;
      scan.range_max = g_R.scan_range_max;

      const float FOV = g_R.scan_fov_deg * static_cast<float>(M_PI) / 180.0f;
      scan.angle_min = -0.5f * FOV;
      scan.angle_max =  0.5f * FOV;
      scan.angle_increment = (N > 1) ? (scan.angle_max - scan.angle_min) / static_cast<float>(N - 1) : FOV;
      scan.time_increment = 0.0f;
      scan.scan_time = 1.0f / std::max(1.0f, g_R.scan_rate_hz);

      scan.ranges.resize(N);
      if (N) std::memcpy(scan.ranges.data(), ranges, N * sizeof(float));
      scan_pub->publish(scan);
    }

    close(fd);
    RCLCPP_INFO(node->get_logger(), "client disconnected");
  }

  close(srv);
  rclcpp::shutdown();
  return 0;
}
