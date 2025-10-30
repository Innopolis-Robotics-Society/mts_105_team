#include <rclcpp/rclcpp.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <string>
#include <vector>
#include <cmath>

// LSM6DS3 register map (subset)
static constexpr uint8_t REG_WHO_AM_I     = 0x0F; // expect 0x69
static constexpr uint8_t REG_CTRL1_XL     = 0x10;
static constexpr uint8_t REG_CTRL2_G      = 0x11;
static constexpr uint8_t REG_CTRL3_C      = 0x12;
static constexpr uint8_t REG_OUTX_L_G     = 0x22; // 6 bytes gyro
static constexpr uint8_t REG_OUTX_L_A     = 0x28; // 6 bytes accel
static constexpr double  PI               = 3.14159265358979323846;

static inline int16_t le16(const uint8_t lo, const uint8_t hi) {
  return static_cast<int16_t>((static_cast<uint16_t>(hi) << 8) | lo);
}

class Lsm6ds3ImuNode : public rclcpp::Node {
public:
  Lsm6ds3ImuNode() : Node("a105_imu_yes") {
    device_                = declare_parameter<std::string>("device", "/dev/spidev0.0");
    spi_speed_hz_          = declare_parameter<int>("spi_speed_hz", 1'000'000);
    spi_mode_              = static_cast<uint8_t>(declare_parameter<int>("spi_mode", 3));
    frame_id_              = declare_parameter<std::string>("frame_id", "imu_link");
    odr_hz_                = declare_parameter<int>("odr_hz", 104);
    accel_range_g_         = declare_parameter<int>("accel_range_g", 4);
    gyro_range_dps_        = declare_parameter<int>("gyro_range_dps", 245);
    bias_samples_          = declare_parameter<int>("bias_samples", 200);
    publish_rate_hz_       = declare_parameter<int>("publish_rate_hz", 100);
    lin_acc_stddev_        = declare_parameter<double>("linear_accel_stddev", 0.03);
    ang_vel_stddev_        = declare_parameter<double>("angular_velocity_stddev", 0.02);

    openSpi();
    initChip();
    calibrateBias();

    pub_ = create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", rclcpp::SensorDataQoS());

    double dt = 1.0 / std::max(1, publish_rate_hz_);
    timer_ = create_wall_timer(std::chrono::duration<double>(dt),
                               [this]() { this->publishOnce(); });
  }

  ~Lsm6ds3ImuNode() override {
    if (fd_ >= 0) ::close(fd_);
  }

private:
  void openSpi() {
    fd_ = ::open(device_.c_str(), O_RDWR);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "open(%s) failed: %s", device_.c_str(), std::strerror(errno));
      throw std::runtime_error("SPI open failed");
    }
    // Mode
    if (ioctl(fd_, SPI_IOC_WR_MODE, &spi_mode_) < 0 || ioctl(fd_, SPI_IOC_RD_MODE, &spi_mode_) < 0) {
      RCLCPP_FATAL(get_logger(), "SPI set/read mode failed");
      throw std::runtime_error("SPI mode");
    }
    // Bits per word = 8
    uint8_t bits = 8;
    if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0 || ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits) < 0) {
      RCLCPP_FATAL(get_logger(), "SPI bits-per-word failed");
      throw std::runtime_error("SPI bits");
    }
    // Speed
    uint32_t hz = spi_speed_hz_;
    if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &hz) < 0 || ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &hz) < 0) {
      RCLCPP_FATAL(get_logger(), "SPI set/read speed failed");
      throw std::runtime_error("SPI speed");
    }
  }

  void writeReg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg & 0x7F), val };
    uint8_t rx[2] = {0,0};
    struct spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<unsigned long>(tx);
    tr.rx_buf = reinterpret_cast<unsigned long>(rx);
    tr.len = 2;
    tr.speed_hz = spi_speed_hz_;
    tr.bits_per_word = 8;
    tr.cs_change = 0;
    if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
      RCLCPP_ERROR(get_logger(), "SPI write reg 0x%02X failed", reg);
    }
  }

  uint8_t readReg(uint8_t reg) {
    uint8_t tx[2] = { static_cast<uint8_t>(reg | 0x80), 0x00 };
    uint8_t rx[2] = {0,0};
    struct spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<unsigned long>(tx);
    tr.rx_buf = reinterpret_cast<unsigned long>(rx);
    tr.len = 2;
    tr.speed_hz = spi_speed_hz_;
    tr.bits_per_word = 8;
    tr.cs_change = 0;
    if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
      RCLCPP_ERROR(get_logger(), "SPI read reg 0x%02X failed", reg);
      return 0;
    }
    return rx[1];
  }

  void readBurst(uint8_t start_reg, uint8_t *buf, size_t n) {
    std::vector<uint8_t> tx(n + 1, 0);
    std::vector<uint8_t> rx(n + 1, 0);
    tx[0] = static_cast<uint8_t>(start_reg | 0x80); // read

    struct spi_ioc_transfer tr{};
    tr.tx_buf = reinterpret_cast<unsigned long>(tx.data());
    tr.rx_buf = reinterpret_cast<unsigned long>(rx.data());
    tr.len = static_cast<uint32_t>(n + 1);
    tr.speed_hz = spi_speed_hz_;
    tr.bits_per_word = 8;
    tr.cs_change = 0;

    if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
      RCLCPP_ERROR(get_logger(), "SPI burst read from 0x%02X failed", start_reg);
      std::memset(buf, 0, n);
      return;
    }
    std::memcpy(buf, rx.data() + 1, n);
  }

  void initChip() {
    // WHO_AM_I
    uint8_t who = readReg(REG_WHO_AM_I);
    if (who != 0x69) {
      RCLCPP_WARN(get_logger(), "WHO_AM_I=0x%02X (expected 0x69). Check wiring and chip.", who);
    }

    // Reset then enable IF_INC + BDU
    writeReg(REG_CTRL3_C, 0x01);  // SW_RESET
    rclcpp::sleep_for(std::chrono::milliseconds(50));
    writeReg(REG_CTRL3_C, 0b01000100); // BDU=1 (bit6), IF_INC=1 (bit2)

    // ODR encoding
    uint8_t odr_code = odrToCode(odr_hz_); // [7:4]
    // Accel FS
    uint8_t fs_xl = accelFsToBits(accel_range_g_); // [3:2]
    // BW_XL left 00
    writeReg(REG_CTRL1_XL, static_cast<uint8_t>((odr_code << 4) | (fs_xl << 2) | 0b00));

    // Gyro FS
    uint8_t fs_g = gyroFsToBits(gyro_range_dps_);  // [4:3]
    writeReg(REG_CTRL2_G, static_cast<uint8_t>((odr_code << 4) | (fs_g << 3)));

    // Precompute scales
    accel_scale_si_ = accelScaleSI(accel_range_g_);      // m/s^2 per LSB
    gyro_scale_si_  = gyroScaleSI(gyro_range_dps_);      // rad/s per LSB
  }

  static uint8_t odrToCode(int hz) {
    // 12,26,52,104,208,416,833,1660 Hz
    if (hz >= 1660) return 0b1001;
    if (hz >= 833)  return 0b1000;
    if (hz >= 416)  return 0b0111;
    if (hz >= 208)  return 0b0110;
    if (hz >= 104)  return 0b0101;
    if (hz >= 52)   return 0b0100;
    if (hz >= 26)   return 0b0011;
    return 0b0010; // 12.5 Hz
  }

  static uint8_t accelFsToBits(int g) {
    switch (g) {
      case 2:  return 0b00;
      case 4:  return 0b10;
      case 8:  return 0b11;
      case 16: return 0b01;
      default: return 0b10; // 4g
    }
  }

  static uint8_t gyroFsToBits(int dps) {
    switch (dps) {
      case 245:  return 0b00;
      case 500:  return 0b01;
      case 1000: return 0b10;
      case 2000: return 0b11;
      default:   return 0b00; // 245 dps
    }
  }

  static double accelScaleSI(int g) {
    // LSB sensitivities (datasheet): 2g=0.061 mg/LSB, 4g=0.122, 8g=0.244, 16g=0.488
    double mg_per_lsb;
    switch (g) {
      case 2:  mg_per_lsb = 0.061; break;
      case 4:  mg_per_lsb = 0.122; break;
      case 8:  mg_per_lsb = 0.244; break;
      case 16: mg_per_lsb = 0.488; break;
      default: mg_per_lsb = 0.122; break;
    }
    return (mg_per_lsb * 1e-3) * 9.80665; // m/s^2 per LSB
  }

  static double gyroScaleSI(int dps) {
    // LSB sensitivities (mdps/LSB): 245=8.75, 500=17.50, 1000=35, 2000=70
    double mdps_per_lsb;
    switch (dps) {
      case 245:  mdps_per_lsb = 8.75;  break;
      case 500:  mdps_per_lsb = 17.50; break;
      case 1000: mdps_per_lsb = 35.0;  break;
      case 2000: mdps_per_lsb = 70.0;  break;
      default:   mdps_per_lsb = 8.75;  break;
    }
    double dps_per_lsb = mdps_per_lsb * 1e-3;
    return dps_per_lsb * (PI / 180.0); // rad/s per LSB
  }

  void calibrateBias() {
    RCLCPP_INFO(get_logger(), "Gyro bias calibration: %d samples", bias_samples_);
    double sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;
    uint8_t gbuf[6];
    for (int i = 0; i < bias_samples_; ++i) {
      readBurst(REG_OUTX_L_G, gbuf, 6);
      int16_t gx = le16(gbuf[0], gbuf[1]);
      int16_t gy = le16(gbuf[2], gbuf[3]);
      int16_t gz = le16(gbuf[4], gbuf[5]);
      sum_gx += gx;
      sum_gy += gy;
      sum_gz += gz;
      rclcpp::sleep_for(std::chrono::milliseconds(10));
    }
    bias_gx_ = sum_gx / bias_samples_;
    bias_gy_ = sum_gy / bias_samples_;
    bias_gz_ = sum_gz / bias_samples_;
    RCLCPP_INFO(get_logger(), "Gyro bias (raw LSB): gx=%.2f gy=%.2f gz=%.2f", bias_gx_, bias_gy_, bias_gz_);
  }

  void publishOnce() {
    uint8_t abuf[6], gbuf[6];
    readBurst(REG_OUTX_L_G, gbuf, 6);
    readBurst(REG_OUTX_L_A, abuf, 6);

    int16_t gx_lsb = le16(gbuf[0], gbuf[1]);
    int16_t gy_lsb = le16(gbuf[2], gbuf[3]);
    int16_t gz_lsb = le16(gbuf[4], gbuf[5]);

    int16_t ax_lsb = le16(abuf[0], abuf[1]);
    int16_t ay_lsb = le16(abuf[2], abuf[3]);
    int16_t az_lsb = le16(abuf[4], abuf[5]);

    double wx = (gx_lsb - bias_gx_) * gyro_scale_si_;
    double wy = (gy_lsb - bias_gy_) * gyro_scale_si_;
    double wz = (gz_lsb - bias_gz_) * gyro_scale_si_;

    double ax = ax_lsb * accel_scale_si_;
    double ay = ay_lsb * accel_scale_si_;
    double az = az_lsb * accel_scale_si_;

    auto msg = sensor_msgs::msg::Imu();
    msg.header.stamp = this->get_clock()->now();
    msg.header.frame_id = frame_id_;

    msg.angular_velocity.x = wx;
    msg.angular_velocity.y = wy;
    msg.angular_velocity.z = wz;

    msg.linear_acceleration.x = ax;
    msg.linear_acceleration.y = ay;
    msg.linear_acceleration.z = az;

    msg.orientation_covariance[0] = -1.0;

    const double av_var = ang_vel_stddev_ * ang_vel_stddev_;
    const double la_var = lin_acc_stddev_ * lin_acc_stddev_;
    msg.angular_velocity_covariance[0] = av_var;
    msg.angular_velocity_covariance[4] = av_var;
    msg.angular_velocity_covariance[8] = av_var;
    msg.linear_acceleration_covariance[0] = la_var;
    msg.linear_acceleration_covariance[4] = la_var;
    msg.linear_acceleration_covariance[8] = la_var;

    pub_->publish(std::move(msg));
  }

  // Params
  std::string device_;
  int spi_speed_hz_;
  uint8_t spi_mode_;
  std::string frame_id_;
  int odr_hz_;
  int accel_range_g_;
  int gyro_range_dps_;
  int bias_samples_;
  int publish_rate_hz_;
  double lin_acc_stddev_;
  double ang_vel_stddev_;

  // SPI
  int fd_{-1};

  // Scales and bias
  double accel_scale_si_{0.0};
  double gyro_scale_si_{0.0};
  double bias_gx_{0.0}, bias_gy_{0.0}, bias_gz_{0.0};

  // ROS
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Lsm6ds3ImuNode>());
  } catch (const std::exception &e) {
    fprintf(stderr, "Fatal: %s\n", e.what());
  }
  rclcpp::shutdown();
  return 0;
}
