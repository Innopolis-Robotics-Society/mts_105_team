#!/usr/bin/env python3
import json, math, threading, time, serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class CmdVelToCobra(Node):
    def __init__(self):
        super().__init__('cmdvel_to_cobra_py')

        # params
        self.declare_parameter('port', '/dev/ttyACM1')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('track', 0.228)   # m
        self.declare_parameter('tpm', 1000.0)    # ticks per meter
        self.declare_parameter('rate', 10.0)     # Hz, send T=13
        self.declare_parameter('poll_hz', 5.0)   # Hz, send T=130
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud').value)
        self.track = float(self.get_parameter('track').value)
        self.tpm = float(self.get_parameter('tpm').value)
        self.rate = float(self.get_parameter('rate').value)
        self.poll_hz = float(self.get_parameter('poll_hz').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # serial
        self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
        self.get_logger().info(f"Serial open {self.port}@{self.baud}")

        # init controller (CRLF)
        self._jsend({"T":131, "cmd":1})   # telemetry on
        self._jsend({"T":142, "cmd":200}) # telemetry period ms
        self._jsend({"T":143, "cmd":0})   # echo off
        self._jsend({"T":12})             # motors enable
        self._jsend({"T":3, "lineNum":0, "Text":":-)"})
        time.sleep(0.1)

        # ROS IO
        self.sub = self.create_subscription(Twist, 'cmd_vel', self._cmd_cb, 10)
        self.pub = self.create_publisher(Odometry, 'odom', 10)

        self.timer_cmd  = self.create_timer(1.0/max(1e-6,self.rate), self._tick_cmd)
        self.timer_poll = self.create_timer(1.0/max(1e-6,self.poll_hz), self._tick_poll)

        # RX thread
        self._rx_run = True
        self._rx_th = threading.Thread(target=self._rx_loop, daemon=True)
        self._rx_th.start()

        # state
        self._v = 0.0
        self._w = 0.0
        self._last_left = 0
        self._last_right = 0
        self._odom_ready = False
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_time = self.get_clock().now()

    # ---- helpers ----
    def _jsend(self, obj):
        s = json.dumps(obj)
        self.ser.write((s + "\r\n").encode('utf-8'))
        self.get_logger().info(f"[TX] {s}")

    def _cmd_cb(self, msg: Twist):
        self._v = msg.linear.x
        self._w = msg.angular.z

    def _tick_cmd(self):
        self._jsend({"T":13, "X":self._v, "Z":self._w})

    def _tick_poll(self):
        self._jsend({"T":130})

    # ---- RX ----
    def _rx_loop(self):
        buf = ""
        while self._rx_run:
            try:
                chunk = self.ser.read(512)
                if not chunk:
                    continue
                buf += chunk.decode('utf-8', errors='ignore')
                # простая сборка по балансировке скобок
                while True:
                    start = buf.find('{')
                    if start < 0:
                        buf = ""  # мусор до '{'
                        break
                    end = buf.find('}', start)
                    if end < 0:
                        # ждём продолжение
                        if start > 0:
                            buf = buf[start:]  # обрежем мусор слева
                        break
                    js = buf[start:end+1]
                    buf = buf[end+1:]
                    self._handle_json(js)
            except Exception as e:
                self.get_logger().error(f"RX error: {e}")
                time.sleep(0.1)

    def _handle_json(self, js: str):
        self.get_logger().info(f"[RX] {js}")
        try:
            data = json.loads(js)
        except Exception:
            return
        if data.get("T") != 1001:
            return

        # поддерживаем два формата: абсолюты (odl/odr) или дельты (dl/dr)
        tnow = self.get_clock().now()
        dt = (tnow - self._last_time).nanoseconds / 1e9
        if dt < 0:
            self._last_time = tnow
            return

        Dl = Dr = None
        if "odl" in data and "odr" in data:
            odl = int(data["odl"]); odr = int(data["odr"])
            if not self._odom_ready:
                self._last_left = odl; self._last_right = odr
                self._odom_ready = True
                self._last_time = tnow
                return
            Dl = (odl - self._last_left) / max(1e-6, self.tpm)
            Dr = (odr - self._last_right) / max(1e-6, self.tpm)
            self._last_left = odl; self._last_right = odr
        elif "dl" in data and "dr" in data:
            Dl = float(data["dl"]) / max(1e-6, self.tpm)
            Dr = float(data["dr"]) / max(1e-6, self.tpm)
            if not self._odom_ready:
                self._odom_ready = True

        if Dl is None or Dr is None:
            return
        if dt == 0.0:
            self._last_time = tnow
            return

        dS  = 0.5 * (Dl + Dr)
        dTh = (Dr - Dl) / max(1e-9, self.track)
        thm = self._yaw + 0.5*dTh
        self._x   += dS * math.cos(thm)
        self._y   += dS * math.sin(thm)
        self._yaw += dTh

        od = Odometry()
        od.header.stamp = tnow.to_msg()
        od.header.frame_id = self.odom_frame
        od.child_frame_id  = self.base_frame
        od.pose.pose.position.x = self._x
        od.pose.pose.position.y = self._y
        od.pose.pose.orientation.z = math.sin(self._yaw/2.0)
        od.pose.pose.orientation.w = math.cos(self._yaw/2.0)
        od.twist.twist.linear.x  = dS / dt
        od.twist.twist.angular.z = dTh / dt
        self.pub.publish(od)

        self.get_logger().info(f"[ODOM] dt={dt:.3f} Dl={Dl:.4f} Dr={Dr:.4f} x={self._x:.3f} y={self._y:.3f} yaw={self._yaw:.3f}")
        self._last_time = tnow


def main():
    rclpy.init()
    node = CmdVelToCobra()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node._rx_run = False
    time.sleep(0.1)
    node.destroy_node()
    rclpy.shutdown()
