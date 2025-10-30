#include <rclcpp/rclcpp.hpp>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <chrono>
#include <iostream>

namespace {
constexpr uint8_t REG_SMPLRT_DIV     = 0x19;
constexpr uint8_t REG_CONFIG         = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG    = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG   = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2  = 0x1D;
constexpr uint8_t REG_INT_PIN_CFG    = 0x37;
constexpr uint8_t REG_INT_ENABLE     = 0x38;
constexpr uint8_t REG_ACCEL_XOUT_H   = 0x3B; // +14 bytes to GZ_L
constexpr uint8_t REG_USER_CTRL      = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1     = 0x6B;
constexpr uint8_t REG_WHO_AM_I       = 0x75;

inline int16_t be16(const uint8_t *p) {
  return static_cast<int16_t>((p[0] << 8) | p[1]);
}
} // namespace

class Mpu6500TermNode : public rclcpp::Node {
public:
  Mpu6500TermNode() : Node("mpu6500_term_node") {
    // параметры аналогичны Python
    device_     = declare_parameter<std::string>("device", "");
    bus_        = declare_parameter<int>("bus", 0);
    dev_        = declare_parameter<int>("dev", 0);
    speed_hz_   = declare_parameter<int>("speed", 1'000'000);
    rate_hz_    = declare_parameter<double>("rate", 100.0);
    gyro_dps_   = declare_parameter<int>("gyro_range", 2000);
    accel_g_    = declare_parameter<int>("accel_range", 16);
    dlpf_       = declare_parameter<int>("dlpf", 3);
    smpldiv_    = declare_parameter<int>("smplrt_div", 4);
    raw_        = declare_parameter<bool>("raw", false);

    if (device_.empty()) {
      device_ = "/dev/spidev" + std::to_string(bus_) + "." + std::to_string(dev_);
    }

    if (!open_spi()) {
      throw std::runtime_error("open SPI failed");
    }
    if (!init_mpu()) {
      throw std::runtime_error("MPU init failed");
    }

    period_ = std::chrono::duration<double>(1.0 / std::max(1e-3, rate_hz_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period_),
      std::bind(&Mpu6500TermNode::tick, this));

    // заголовок как в Python
    if (raw_) {
      std::cout << "ax ay az gx gy gz (raw)\n";
    } else {
      std::cout << "ax[m/s^2] ay[m/s^2] az[m/s^2] gx[rad/s] gy[rad/s] gz[rad/s]\n";
    }
    std::cout.flush();
  }

  ~Mpu6500TermNode() override {
    if (fd_ >= 0) ::close(fd_);
  }

private:
  bool open_spi() {
    fd_ = ::open(device_.c_str(), O_RDWR);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s) failed: %s", device_.c_str(), std::strerror(errno));
      return false;
    }
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0) {
      RCLCPP_ERROR(get_logger(), "SPI set mode failed: %s", std::strerror(errno));
      return false;
    }
    if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
      RCLCPP_ERROR(get_logger(), "SPI set bits failed: %s", std::strerror(errno));
      return false;
    }
    if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz_) < 0) {
      RCLCPP_ERROR(get_logger(), "SPI set speed failed: %s", std::strerror(errno));
      return false;
    }
    return true;
  }

  bool wr(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F), val };
    struct spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<unsigned long>(tx);
    tr.len = 2;
    tr.speed_hz = speed_hz_;
    tr.bits_per_word = 8;
    int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
    return ret >= 1;
  }

  bool rn(uint8_t reg, uint8_t *dst, size_t n) {
    std::vector<uint8_t> tx(n + 1, 0x00), rx(n + 1, 0x00);
    tx[0] = reg | 0x80;
    struct spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    tr.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    tr.len = tx.size();
    tr.speed_hz = speed_hz_;
    tr.bits_per_word = 8;
    int ret = ioctl(fd_, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 1) return false;
    std::memcpy(dst, rx.data() + 1, n);
    return true;
  }

  bool init_mpu() {
    wr(REG_PWR_MGMT_1, 0x80);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    if (!wr(REG_PWR_MGMT_1, 0x01)) return false; // PLL X gyro
    if (!wr(REG_USER_CTRL, 0x10)) return false;  // disable I2C

    uint8_t who = 0;
    if (!rn(REG_WHO_AM_I, &who, 1)) return false;
    if (!(who == 0x70 || who == 0x71)) {
      RCLCPP_WARN(get_logger(), "WHO_AM_I=0x%02X", who);
    }

    if (!wr(REG_CONFIG, dlpf_ & 0x07)) return false;
    if (!wr(REG_SMPLRT_DIV, smpldiv_ & 0xFF)) return false;

    uint8_t gcfg = 3 << 3; // default 2000 dps
    if      (gyro_dps_ == 250)  gcfg = 0 << 3;
    else if (gyro_dps_ == 500)  gcfg = 1 << 3;
    else if (gyro_dps_ == 1000) gcfg = 2 << 3;
    if (!wr(REG_GYRO_CONFIG, gcfg)) return false;

    uint8_t acfg = 3 << 3; // default 16 g
    if      (accel_g_ == 2)  acfg = 0 << 3;
    else if (accel_g_ == 4)  acfg = 1 << 3;
    else if (accel_g_ == 8)  acfg = 2 << 3;
    if (!wr(REG_ACCEL_CONFIG, acfg)) return false;

    if (!wr(REG_ACCEL_CONFIG2, dlpf_ & 0x07)) return false;
    if (!wr(REG_INT_PIN_CFG, 0x00)) return false;
    if (!wr(REG_INT_ENABLE,  0x00)) return false;

    // шкалы
    switch (accel_g_) {
      case 2:  accel_lsb_per_g_ = 16384.0; break;
      case 4:  accel_lsb_per_g_ = 8192.0;  break;
      case 8:  accel_lsb_per_g_ = 4096.0;  break;
      default: accel_lsb_per_g_ = 2048.0;  break; // 16g
    }
    switch (gyro_dps_) {
      case 250:  gyro_lsb_per_dps_ = 131.0;  break;
      case 500:  gyro_lsb_per_dps_ = 65.5;   break;
      case 1000: gyro_lsb_per_dps_ = 32.8;   break;
      default:   gyro_lsb_per_dps_ = 16.4;   break; // 2000
    }
    return true;
  }

  void tick() {
    uint8_t b[14];
    if (!rn(REG_ACCEL_XOUT_H, b, 14)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "spi read failed");
      return;
    }
    int16_t ax = be16(b + 0);
    int16_t ay = be16(b + 2);
    int16_t az = be16(b + 4);
    int16_t gx = be16(b + 8);
    int16_t gy = be16(b + 10);
    int16_t gz = be16(b + 12);

    if (raw_) {
      std::cout << ax << ' ' << ay << ' ' << az << ' '
                << gx << ' ' << gy << ' ' << gz << '\n';
    } else {
      constexpr double G = 9.80665;
      constexpr double DEG2RAD = M_PI / 180.0;
      const double ax_si = ax/accel_lsb_per_g_*G;
      const double ay_si = ay/accel_lsb_per_g_*G;
      const double az_si = az/accel_lsb_per_g_*G;
      const double gx_si = (gx/gyro_lsb_per_dps_)*DEG2RAD;
      const double gy_si = (gy/gyro_lsb_per_dps_)*DEG2RAD;
      const double gz_si = (gz/gyro_lsb_per_dps_)*DEG2RAD;
      std::cout.setf(std::ios::fixed); std::cout.precision(5);
      std::cout << ax_si << ' ' << ay_si << ' ' << az_si << ' '
                << gx_si << ' ' << gy_si << ' ' << gz_si << '\n';
    }
    std::cout.flush();
  }

  // params
  std::string device_;
  int bus_, dev_, speed_hz_, gyro_dps_, accel_g_, dlpf_, smpldiv_;
  double rate_hz_{}, accel_lsb_per_g_{}, gyro_lsb_per_dps_{};
  bool raw_{};

  // io
  int fd_{-1};

  // timer
  std::chrono::duration<double> period_{};
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Mpu6500TermNode>());
  } catch (const std::exception &e) {
    std::fprintf(stderr, "fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
