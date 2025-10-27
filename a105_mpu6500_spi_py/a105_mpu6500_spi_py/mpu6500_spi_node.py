#!/usr/bin/env python3
import math, time, spidev
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

REG_SMPLRT_DIV, REG_CONFIG, REG_GYRO_CONFIG, REG_ACCEL_CONFIG = 0x19,0x1A,0x1B,0x1C
REG_ACCEL_CONFIG2, REG_INT_PIN_CFG, REG_INT_ENABLE = 0x1D,0x37,0x38
REG_ACCEL_XOUT_H, REG_USER_CTRL, REG_PWR_MGMT_1, REG_WHO_AM_I = 0x3B,0x6A,0x6B,0x75

G = 9.80665
DEG2RAD = math.pi/180.0

def be16(hi, lo):
    v = (hi<<8)|lo
    return v-65536 if v & 0x8000 else v

class Mpu6500Node(Node):
    def __init__(self):
        super().__init__('mpu6500_spi_node')

        self.declare_parameter('bus', 0)
        self.declare_parameter('dev', 0)
        self.declare_parameter('speed', 1_000_000)
        self.declare_parameter('rate', 200.0)
        self.declare_parameter('gyro_range', 2000)
        self.declare_parameter('accel_range', 16)
        self.declare_parameter('dlpf', 3)
        self.declare_parameter('smplrt_div', 4)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('gyro_variance', 0.02)
        self.declare_parameter('accel_variance', 0.04)

        bus         = int(self.get_parameter('bus').value)
        dev         = int(self.get_parameter('dev').value)
        speed       = int(self.get_parameter('speed').value)
        self.rate   = float(self.get_parameter('rate').value)
        self.frame  = str(self.get_parameter('frame_id').value)
        self.gvar   = float(self.get_parameter('gyro_variance').value)
        self.avar   = float(self.get_parameter('accel_variance').value)

        gyro_dps    = int(self.get_parameter('gyro_range').value)
        accel_g     = int(self.get_parameter('accel_range').value)
        dlpf        = int(self.get_parameter('dlpf').value)
        smpldiv     = int(self.get_parameter('smplrt_div').value)

        self.acc_lsb = {2:16384.0, 4:8192.0, 8:4096.0}.get(accel_g, 2048.0)
        self.gyr_lsb = {250:131.0, 500:65.5, 1000:32.8}.get(gyro_dps, 16.4)

        self.spi = spidev.SpiDev()
        self.spi.open(bus, dev)         # /dev/spidev0.0
        self.spi.mode = 0
        self.spi.max_speed_hz = speed
        self.spi.bits_per_word = 8

        self._init_device(gyro_dps, accel_g, dlpf, smpldiv)

        self.pub = self.create_publisher(Imu, 'imu/data_raw', qos_profile_sensor_data)
        period = 1.0/max(self.rate, 1e-3)
        self.timer = self.create_timer(period, self._tick)

    def _w(self, reg, val): self.spi.xfer2([reg & 0x7F, val & 0xFF])
    def _rn(self, reg, n):  return self.spi.xfer2([reg | 0x80] + [0]*n)[1:]

    def _init_device(self, gyro_dps, accel_g, dlpf, smpldiv):
        self._w(REG_PWR_MGMT_1, 0x80); time.sleep(0.1)
        self._w(REG_PWR_MGMT_1, 0x01)
        self._w(REG_USER_CTRL, 0x10)
        who = self._rn(REG_WHO_AM_I, 1)[0]
        if who not in (0x70, 0x71):
            self.get_logger().warn(f'WHO_AM_I=0x{who:02X}')

        self._w(REG_CONFIG, dlpf & 0x07)
        self._w(REG_SMPLRT_DIV, smpldiv & 0xFF)
        gcfg = {250:0,500:1,1000:2}.get(gyro_dps,3) << 3
        self._w(REG_GYRO_CONFIG, gcfg)
        acfg = {2:0,4:1,8:2}.get(accel_g,3) << 3
        self._w(REG_ACCEL_CONFIG, acfg)
        self._w(REG_ACCEL_CONFIG2, dlpf & 0x07)
        self._w(REG_INT_PIN_CFG, 0x00); self._w(REG_INT_ENABLE, 0x00)

    def _tick(self):
        try:
            b = self._rn(REG_ACCEL_XOUT_H, 14)
        except Exception as e:
            self.get_logger().throttle(5.0).error(f'spi read failed: {e}')
            return

        ax = be16(b[0],b[1]); ay = be16(b[2],b[3]); az = be16(b[4],b[5])
        gx = be16(b[8],b[9]); gy = be16(b[10],b[11]); gz = be16(b[12],b[13])

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame

        msg.linear_acceleration.x = ax/self.acc_lsb*G
        msg.linear_acceleration.y = ay/self.acc_lsb*G
        msg.linear_acceleration.z = az/self.acc_lsb*G


        msg.angular_velocity.x = (gx/self.gyr_lsb)*DEG2RAD
        msg.angular_velocity.y = (gy/self.gyr_lsb)*DEG2RAD
        msg.angular_velocity.z = (gz/self.gyr_lsb)*DEG2RAD

        msg.orientation_covariance[0] = -1.0
        for i in (0,4,8):
            msg.angular_velocity_covariance[i] = self.gvar
            msg.linear_acceleration_covariance[i] = self.avar

        self.pub.publish(msg)

    def destroy_node(self):
        try: self.spi.close()
        except Exception: pass
        super().destroy_node()

def main():
    rclpy.init()
    node = Mpu6500Node()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
