#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include <cmath>
#include <vector>
#include <array>
#include <atomic>
#include <thread>
#include <mutex>
#include <cstring>
#include <algorithm>
#include <limits>

using sensor_msgs::msg::LaserScan;

static constexpr uint8_t HEADER = 0x54;
static constexpr uint8_t VERLEN = 0x2C;
static constexpr int FRAME_LEN = 47;
static constexpr int POINTS_PER_PACK = 12;
static constexpr int BYTES_PER_POINT = 3;

static uint8_t sum8(const uint8_t* data, size_t n) {
  uint32_t s = 0; for (size_t i=0;i<n;i++) s += data[i]; return uint8_t(s & 0xFF);
}
static uint8_t crc8_maxim(const uint8_t* data, size_t n) {
  uint8_t crc = 0;
  for (size_t i=0;i<n;i++) {
    crc ^= data[i];
    for (int k=0;k<8;k++) crc = (crc & 1) ? (crc >> 1) ^ 0x8C : (crc >> 1);
  }
  return crc;
}
static bool checksum_ok(const uint8_t* frame) {
  const uint8_t tail = frame[FRAME_LEN-1];
  return sum8(frame, FRAME_LEN-1) == tail || crc8_maxim(frame, FRAME_LEN-1) == tail;
}

static speed_t baud_to_speed(int b) {
  switch (b) {
    case 115200: return B115200;
#ifdef B230400
    case 230400: return B230400;
#endif
#ifdef B460800
    case 460800: return B460800;
#endif
    default: return B115200;
  }
}

class LD19ToLaserScan : public rclcpp::Node {
public:
  explicit LD19ToLaserScan(const rclcpp::NodeOptions& options = rclcpp::NodeOptions())
  : Node("ld19_to_laserscan", options) {
    port_         = this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
    baud_         = this->declare_parameter<int>("baud", 230400);
    frame_id_     = this->declare_parameter<std::string>("frame_id", "lidar_link");
    inc_deg_      = this->declare_parameter<double>("inc_deg", 1.0);
    range_min_    = this->declare_parameter<double>("range_min", 0.05);
    range_max_    = this->declare_parameter<double>("range_max", 8.0);
    angle_offset_ = this->declare_parameter<double>("angle_offset_deg", 0.0) * M_PI/180.0;
    clockwise_    = this->declare_parameter<bool>("clockwise", false);

    angle_inc_ = inc_deg_ * M_PI/180.0;
    bins_n_ = std::max(1, static_cast<int>(std::lround((2.0*M_PI) / angle_inc_)));

    auto qos = rclcpp::SensorDataQoS().reliable().keep_last(1);
    pub_ = this->create_publisher<LaserScan>("scan", qos);

    open_serial();
    running_ = true;
    th_ = std::thread(&LD19ToLaserScan::reader_loop, this);
  }

  ~LD19ToLaserScan() override {
    running_ = false;
    if (th_.joinable()) th_.join();
    if (fd_ >= 0) ::close(fd_);
  }

private:
  void open_serial() {
    fd_ = ::open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_FATAL(this->get_logger(), "open(%s) failed: %s", port_.c_str(), std::strerror(errno));
      throw std::runtime_error("serial open failed");
    }
    fcntl(fd_, F_SETFL, 0); // blocking
    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) throw std::runtime_error("tcgetattr failed");
    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    cfsetispeed(&tio, baud_to_speed(baud_));
    cfsetospeed(&tio, baud_to_speed(baud_));
    tio.c_cc[VMIN]  = 0;
    tio.c_cc[VTIME] = 2; // 200 ms read timeout
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) throw std::runtime_error("tcsetattr failed");
    RCLCPP_INFO(this->get_logger(), "Serial %s @ %d opened", port_.c_str(), baud_);
  }

  bool sync_and_read_frame(uint8_t* out_frame) {
    static std::vector<uint8_t> buf;
    uint8_t tmp[128];
    int n = ::read(fd_, tmp, sizeof(tmp));
    if (n > 0) buf.insert(buf.end(), tmp, tmp + n);

    // search header and extract frames
    for (size_t i = 0; i + 1 < buf.size(); /* i is updated inside */) {
      if (buf[i] == HEADER && buf[i+1] == VERLEN) {
        if (buf.size() - i < static_cast<size_t>(FRAME_LEN)) break; // need more data
        std::memcpy(out_frame, buf.data() + i, FRAME_LEN);
        if (checksum_ok(out_frame)) {
          buf.erase(buf.begin(), buf.begin() + i + FRAME_LEN);
          return true;
        } else {
          // drop this HEADER and continue searching
          buf.erase(buf.begin(), buf.begin() + i + 1);
          i = 0;
          continue;
        }
      }
      ++i;
    }

    if (buf.size() > 2048) buf.erase(buf.begin(), buf.begin() + 1024);
    return false;
  }

  inline double transform_angle_rad(double a_deg) const {
    double a = clockwise_ ? -a_deg : a_deg;
    double th = std::fmod(a * M_PI/180.0 + angle_offset_, 2.0*M_PI);
    if (th < 0) th += 2.0*M_PI;
    return th;
  }

  void publish_scan(const std::vector<std::array<double,3>>& pts) {
    // persistent buffers with fixed size; copy into ROS message to avoid move-related DDS issues
    static std::vector<float> ranges_fixed, intens_fixed;

    const float INF = std::numeric_limits<float>::infinity();
    if (ranges_fixed.size() != static_cast<size_t>(bins_n_)) ranges_fixed.assign(bins_n_, INF);
    else std::fill(ranges_fixed.begin(), ranges_fixed.end(), INF);

    if (intens_fixed.size() != static_cast<size_t>(bins_n_)) intens_fixed.assign(bins_n_, 0.0f);
    else std::fill(intens_fixed.begin(), intens_fixed.end(), 0.0f);

    for (const auto& p : pts) {
      double th = p[0], r = p[1]; double inten = p[2];
      if (r < range_min_ || r > range_max_) continue;
      int idx = static_cast<int>(std::floor(th / angle_inc_));
      idx = std::clamp(idx, 0, bins_n_ - 1);
      if (r < ranges_fixed[idx]) { ranges_fixed[idx] = static_cast<float>(r); intens_fixed[idx] = static_cast<float>(inten); }
    }

    auto now = this->get_clock()->now();
    LaserScan msg;
    msg.header.stamp = now;
    msg.header.frame_id = frame_id_;
    msg.angle_min = 0.0f;
    msg.angle_increment = static_cast<float>(angle_inc_);
    msg.angle_max = static_cast<float>(msg.angle_min + (bins_n_ - 1) * angle_inc_);

    double now_s = now.seconds();
    double scan_time = (last_pub_s_ > 0.0) ? (now_s - last_pub_s_) : 0.0;
    last_pub_s_ = now_s;
    msg.scan_time = static_cast<float>(std::max(0.0, scan_time));
    msg.time_increment = (bins_n_ > 0 && msg.scan_time > 0.f) ? static_cast<float>(msg.scan_time / bins_n_) : 0.0f;

    msg.range_min = static_cast<float>(range_min_);
    msg.range_max = static_cast<float>(range_max_);

    msg.ranges      = ranges_fixed;   // copy
    msg.intensities = intens_fixed;   // copy

    if (msg.ranges.size() != msg.intensities.size()) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "ranges(%zu) != intensities(%zu)", msg.ranges.size(), msg.intensities.size());
      return;
    }

    pub_->publish(msg);
  }

  void reader_loop() {
    std::vector<std::array<double,3>> curr; // (theta_rad, r_m, inten)
    double last_deg = -1.0;

    uint8_t frame[FRAME_LEN];
    while (running_) {
      if (!sync_and_read_frame(frame)) {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        continue;
      }
      if (frame[0] != HEADER || frame[1] != VERLEN) continue;

      const uint16_t start_angle = uint16_t(frame[4] | (frame[5] << 8));
      const double s_deg = (start_angle % 36000) / 100.0;
      const uint16_t end_angle_raw = uint16_t(frame[42] | (frame[43] << 8));
      const double e_deg = (end_angle_raw % 36000) / 100.0;
      const double diff = std::fmod(e_deg - s_deg + 360.0, 360.0);
      const double step = (POINTS_PER_PACK > 1) ? (diff / (POINTS_PER_PACK - 1)) : 0.0;

      int off = 6;
      for (int i=0;i<POINTS_PER_PACK;i++) {
        const uint16_t dist_mm = uint16_t(frame[off] | (frame[off+1] << 8));
        const uint8_t  inten   = frame[off+2];
        off += BYTES_PER_POINT;

        const double a_deg = std::fmod(s_deg + i*step, 360.0);
        if (dist_mm == 0 || dist_mm == 0xFFFF || inten == 0) continue;

        const double th = transform_angle_rad(a_deg);
        const double r_m = dist_mm / 1000.0;

        // wrap detection in transformed space 0..360
        const double deg_norm = std::fmod((th * 180.0 / M_PI) + 360.0, 360.0);
        const bool wrap = (last_deg >= 270.0 && deg_norm < 90.0);
        if (wrap && !curr.empty()) {
          publish_scan(curr);
          curr.clear();
        }
        curr.push_back({th, r_m, static_cast<double>(inten)});
        last_deg = deg_norm;
      }
    }
  }

  // params
  std::string port_, frame_id_;
  int fd_{-1}, baud_{230400};
  double inc_deg_{1.0}, angle_inc_{}, range_min_{0.05}, range_max_{8.0}, angle_offset_{0.0};
  bool clockwise_{false};
  int bins_n_{};

  // runtime
  rclcpp::Publisher<LaserScan>::SharedPtr pub_;
  std::thread th_;
  std::atomic<bool> running_{false};
  double last_pub_s_{-1.0};
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions opts;
  opts.use_intra_process_comms(false);  // disable intra-process
  try {
    auto node = std::make_shared<LD19ToLaserScan>(opts);
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    std::fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
