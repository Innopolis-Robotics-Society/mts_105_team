#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cmath>
#include <vector>
#include <string>

namespace {
constexpr uint8_t REG_SMPLRT_DIV     = 0x19;
constexpr uint8_t REG_CONFIG         = 0x1A;
constexpr uint8_t REG_GYRO_CONFIG    = 0x1B;
constexpr uint8_t REG_ACCEL_CONFIG   = 0x1C;
constexpr uint8_t REG_ACCEL_CONFIG2  = 0x1D;
constexpr uint8_t REG_INT_PIN_CFG    = 0x37;
constexpr uint8_t REG_INT_ENABLE     = 0x38;
constexpr uint8_t REG_ACCEL_XOUT_H   = 0x3B; // 14 bytes from here
constexpr uint8_t REG_USER_CTRL      = 0x6A;
constexpr uint8_t REG_PWR_MGMT_1     = 0x6B;
constexpr uint8_t REG_WHO_AM_I       = 0x75;

inline int16_t be16(const uint8_t *p) {
  return static_cast<int16_t>((p[0] << 8) | p[1]);
}
}

class Mpu6500SpiNode : public rclcpp::Node {
public:
  Mpu6500SpiNode() : Node("mpu6500_spi_node") {
    device_         = this->declare_parameter<std::string>("device", "/dev/spidev0.0");
    frame_id_       = this->declare_parameter<std::string>("frame_id", "imu_link");
    rate_hz_        = this->declare_parameter<int>("rate_hz", 200);
    spi_speed_hz_   = this->declare_parameter<int>("spi_speed_hz", 1000000); // 1 MHz
    gyro_dps_       = this->declare_parameter<int>("gyro_range_dps", 2000);  // 250|500|1000|2000
    accel_g_        = this->declare_parameter<int>("accel_range_g", 16);     // 2|4|8|16
    dlpf_cfg_       = this->declare_parameter<int>("dlpf_cfg", 3);  // 0..6
    smplrt_div_     = this->declare_parameter<int>("smplrt_div", 4);
    double gav = this->declare_parameter<double>("gyro_variance", 0.02);
    double aav = this->declare_parameter<double>("accel_variance", 0.04);

    if (!open_spi()) {
      throw std::runtime_error("SPI open/config failed");
    }
    if (!init_mpu()) {
      throw std::runtime_error("MPU init failed");
    }

    imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());

    // set covariances
    gyro_cov_.fill(0.0);
    accel_cov_.fill(0.0);
    gyro_cov_[0] = gyro_cov_[4] = gyro_cov_[8] = gav;
    accel_cov_[0] = accel_cov_[4] = accel_cov_[8] = aav;

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(1000 / std::max(1, rate_hz_)),
        std::bind(&Mpu6500SpiNode::poll_and_publish, this));
  }

  ~Mpu6500SpiNode() override {
    if (fd_ >= 0) ::close(fd_);
  }

private:
  bool open_spi() {
    fd_ = ::open(device_.c_str(), O_RDWR);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "open(%s) failed", device_.c_str());
      return false;
    }
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0) return false;
    if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) return false;
    if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed_hz_) < 0) return false;
    return true;
  }

  bool write_reg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F), val };
    struct spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<unsigned long>(tx);
    tr.len = 2;
    tr.speed_hz = spi_speed_hz_;
    tr.bits_per_word = 8;
    return ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) >= 1;
  }

  bool read_regs(uint8_t reg, uint8_t *dst, size_t n) {
    std::vector<uint8_t> tx(n + 1, 0x00), rx(n + 1, 0x00);
    tx[0] = reg | 0x80; // read
    struct spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    tr.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    tr.len = tx.size();
    tr.speed_hz = spi_speed_hz_;
    tr.bits_per_word = 8;
    if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 1) return false;
    std::memcpy(dst, rx.data() + 1, n);
    return true;
  }

  bool init_mpu() {
    // reset
    write_reg(REG_PWR_MGMT_1, 0x80);
    rclcpp::sleep_for(std::chrono::milliseconds(100));
    // clock: PLL with X axis gyro
    if (!write_reg(REG_PWR_MGMT_1, 0x01)) return false;
    // disable I2C interface for SPI
    if (!write_reg(REG_USER_CTRL, 0x10)) return false;

    // WHO_AM_I check: 0x70 (MPU6500) or 0x71 (MPU9250)
    uint8_t who = 0;
    if (!read_regs(REG_WHO_AM_I, &who, 1)) return false;
    if (!(who == 0x70 || who == 0x71)) {
      RCLCPP_WARN(get_logger(), "WHO_AM_I=0x%02X not expected", who);
    }

    // DLPF
    write_reg(REG_CONFIG, dlpf_cfg_ & 0x07);
    // sample rate = gyro_rate/(1+SMPLRT_DIV). With DLPF on, gyro_rate=1kHz.
    write_reg(REG_SMPLRT_DIV, smplrt_div_ & 0xFF);

    // gyro range
    uint8_t gcfg = 0;
    if      (gyro_dps_ == 250)  gcfg = 0 << 3;
    else if (gyro_dps_ == 500)  gcfg = 1 << 3;
    else if (gyro_dps_ == 1000) gcfg = 2 << 3;
    else                        gcfg = 3 << 3; // 2000
    write_reg(REG_GYRO_CONFIG, gcfg);

    // accel range
    uint8_t acfg = 0;
    if      (accel_g_ == 2)  acfg = 0 << 3;
    else if (accel_g_ == 4)  acfg = 1 << 3;
    else if (accel_g_ == 8)  acfg = 2 << 3;
    else                     acfg = 3 << 3; // 16 g
    write_reg(REG_ACCEL_CONFIG, acfg);

    write_reg(REG_ACCEL_CONFIG2, (dlpf_cfg_ & 0x07));

    // latched interrupts off, data ready off
    write_reg(REG_INT_PIN_CFG, 0x00);
    write_reg(REG_INT_ENABLE,  0x00);
    compute_scales();
    return true;
  }

  void compute_scales() {
    // LSB/FS
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
  }

  void poll_and_publish() {
    uint8_t buf[14];
    if (!read_regs(REG_ACCEL_XOUT_H, buf, 14)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *this->get_clock(), 5000, "read failed");
      return;
    }

    int16_t ax = be16(buf + 0);
    int16_t ay = be16(buf + 2);
    int16_t az = be16(buf + 4);
    int16_t gx = be16(buf + 8);
    int16_t gy = be16(buf + 10);
    int16_t gz = be16(buf + 12);

    sensor_msgs::msg::Imu msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    // accel in m/s^2
    constexpr double G = 9.80665;
    msg.linear_acceleration.x = static_cast<double>(ax) / accel_lsb_per_g_ * G;
    msg.linear_acceleration.y = static_cast<double>(ay) / accel_lsb_per_g_ * G;
    msg.linear_acceleration.z = static_cast<double>(az) / accel_lsb_per_g_ * G;

    // gyro in rad/s
    constexpr double DEG2RAD = M_PI / 180.0;
    msg.angular_velocity.x = (static_cast<double>(gx) / gyro_lsb_per_dps_) * DEG2RAD;
    msg.angular_velocity.y = (static_cast<double>(gy) / gyro_lsb_per_dps_) * DEG2RAD;
    msg.angular_velocity.z = (static_cast<double>(gz) / gyro_lsb_per_dps_) * DEG2RAD;

    // orientation unknown
    msg.orientation_covariance[0] = -1.0;
    // covariances
    for (int i = 0; i < 9; ++i) {
      msg.angular_velocity_covariance[i] = gyro_cov_[i];
      msg.linear_acceleration_covariance[i] = accel_cov_[i];
    }

    imu_pub_->publish(std::move(msg));
  }

  // params
  std::string device_, frame_id_;
  int rate_hz_, spi_speed_hz_, gyro_dps_, accel_g_, dlpf_cfg_, smplrt_div_;
  double accel_lsb_per_g_{16384.0}, gyro_lsb_per_dps_{16.4};
  std::array<double,9> gyro_cov_{}, accel_cov_{};

  // io
  int fd_{-1};

  // ros
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Mpu6500SpiNode>());
  } catch (const std::exception &e) {
    fprintf(stderr, "fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
