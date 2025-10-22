"""
udp_diff — контроллер для Webots.

Функционал:
- Принимает команды скоростей linear_x, angular_z по сети (UDP, порт 5555).
- Управляет моторами робота через дифференциальное управление.
- Считает одометрию по энкодерам.
- Отправляет телеметрию (одометрия, GPS, lidar scan, gyro) по сети (TCP/UDP, порт 5600).
- Записывает результаты заезда (start/finish) в CSV-файл.
- Ведёт лог состояния.
- Включает стереокамеры и камеру глубины (stereo_left, stereo_right, depth_camera).

Физические ограничения:
- v ∈ [-0.3, 0.3] м/с *SPEEDUP
- w ∈ [-1.0, 1.0] рад/с *SPEEDUP
- ограничение ускорений для плавного движения
"""

import socket
import struct
import time
import math
import os
import sys
from controller import Supervisor

# === Константы и переменные окружения ===
CMD_LISTEN_PORT = int(os.getenv("CMD_LISTEN_PORT", "5555"))
TELEMETRY_HOST  = os.getenv("TELEMETRY_HOST", "127.0.0.1")
TELEMETRY_PORT  = int(os.getenv("TELEMETRY_PORT", "5600"))
TELEMETRY_PROTO = os.getenv("TELEMETRY_PROTO", "udp").lower()

TIME_STEP_MS = 32
MAX_VEL = 12.0
WHEEL_BASE = 0.25
WHEEL_RADIUS = 0.035

# УСКОРЕНИЕ
SPEEDUP = int(os.getenv("SPEEDUP", "2"))

# Реалистичные ограничения движения:
BASE_MAX_LINEAR = 0.5  # м/с
BASE_MAX_ANGULAR = 1.0  # рад/с
BASE_MAX_LINEAR_ACC = 0.05  # м/с²
BASE_MAX_ANGULAR_ACC = 0.2  # рад/с²

# Ускоренные ограничения движения:
MAX_LINEAR = BASE_MAX_LINEAR * SPEEDUP
MAX_ANGULAR = BASE_MAX_ANGULAR * SPEEDUP
MAX_LINEAR_ACC = BASE_MAX_LINEAR_ACC * SPEEDUP
MAX_ANGULAR_ACC = BASE_MAX_ANGULAR_ACC * SPEEDUP

# Тайминг старта/финиша
ARENA = float(os.getenv("ARENA", "8"))
OFFSET = float(os.getenv("OFFSET", "0.045"))
WALL_COUNT = float(os.getenv("WALL_COUNT", "16"))
START_CMD_V_THRESH = float(os.getenv("START_CMD_V_THRESH", "0.01"))
START_CMD_W_THRESH = float(os.getenv("START_CMD_W_THRESH", "0.01"))
START_MOVE_DIST = float(os.getenv("START_MOVE_DIST", "0.02"))
FINISH_CX = float(os.getenv("FINISH_CX", "0.0"))
FINISH_CY = float(os.getenv("FINISH_CY", "0.0"))
STOP_ON_FINISH = os.getenv("STOP_ON_FINISH", "pause").lower()

FINISH_HALF_SIZE = ARENA / WALL_COUNT - OFFSET
RESULTS_FILE = os.getenv("RESULTS_FILE", os.path.join(os.getcwd(), "results.csv"))

# ---- helpers ----
def _env_str(k, d): v=os.environ.get(k); return v if v is not None else d
def _env_int(k, d):
    try: return int(os.environ.get(k, str(d)))
    except: return d


# ---- wire formats ----
DEPTH_MAGIC=b"WBTD"; DEPTH_HDR="<HHi f f"   # dw, dh, ds, minR, maxR
RGB_MAGIC=b"WBTR";   RGB_HDR  ="<HHi"       # dw, dh, ds
CHNK_MAGIC=b"CHNK";  CHNK_HDR ="<I I H H"   # msg_id, total_len, idx, count

class UdpDiffController:
    def __init__(self):
        self.robot = Supervisor()
        self.ts = int(self.robot.getBasicTimeStep()) if hasattr(self.robot,'getBasicTimeStep') else 16

        # devices
        self.rgb = None
        self.depth = None

        # stream controls
        self.depth_enable = _env_int("DEPTH_ENABLE", 1) == 1
        self.depth_send_every = max(1, _env_int("DEPTH_SEND_EVERY", 3))
        self.depth_downsample = max(1, _env_int("DEPTH_DOWNSAMPLE", 2))

        self.rgb_enable = _env_int("RGB_ENABLE", 0) == 1
        self.rgb_send_every = max(1, _env_int("RGB_SEND_EVERY", 6))
        self.rgb_downsample = max(1, _env_int("RGB_DOWNSAMPLE", 2))

        # udp chunking
        self.chunking = _env_int("CHUNKING", 1) == 1
        self.max_dgram = max(512, _env_int("MAX_DGRAM", 1200))

        # Моторы
        self.LF = self._pick_motor("left_front_motor")
        self.RF = self._pick_motor("right_front_motor")
        self.LR = self._pick_motor("left_rear_motor")
        self.RR = self._pick_motor("right_rear_motor")

        # Энкодеры
        self.lf_enc = self._enable_sensor("left_front_encoder")
        self.lr_enc = self._enable_sensor("left_rear_encoder")
        self.rf_enc = self._enable_sensor("right_front_encoder")
        self.rr_enc = self._enable_sensor("right_rear_encoder")

        # Датчики
        self.gps = self._setup_gps()
        self.gyro = self._setup_gyro()

        # Одометрия
        self.last_lf = self.last_lr = self.last_rf = self.last_rr = None
        self.odom_x = self.odom_y = self.odom_th = 0.0

        # Скорости (для плавного изменения)
        self.current_v = 0.0
        self.current_w = 0.0

        # Состояние заезда
        self.started = False
        self.finished = False
        self.init_gps = None
        self.start_pos = None
        self.finish_pos = None
        self.start_time = None
        self.finish_time = None

        # Сеть
        self.sock_in = self._setup_cmd_socket()
        self.sock_out = self._setup_telemetry_socket()

    # ================================================================
    # Устройства
    # ================================================================
    def _pick_motor(self, name: str):
        try:
            dev = self.robot.getDevice(name)
            dev.setPosition(float("inf"))
            dev.setVelocity(0.0)
            print(f"[udp_diff] picked motor '{name}'")
            return dev
        except Exception:
            return None

    def _enable_sensor(self, name: str):
        try:
            dev = self.robot.getDevice(name)
            dev.enable(TIME_STEP_MS)
            return dev
        except Exception:
            return None

    def _setup_gps(self):
        try:
            gps = self.robot.getDevice("gps")
            gps.enable(TIME_STEP_MS)
            print("[udp_diff] gps enabled")
            return gps
        except Exception:
            return None

    def _setup_gyro(self):
        try:
            gyro = self.robot.getDevice("gyro")
            gyro.enable(TIME_STEP_MS)
            print("[udp_diff] gyro enabled")
            return gyro
        except Exception:
            return None

    def _next_attempt(self, path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                n = sum(1 for _ in f) - 1
                return max(1, n + 1)
        except Exception:
            return 1

    def _write_result_csv(self, elapsed, start_t, finish_t, sx, sy, fx, fy, status: str = None):
        path = RESULTS_FILE
        header = "timestamp_iso,attempt,elapsed_s,start_x,start_y,finish_x,finish_y,start_t,finish_t,status\n"
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        if not os.path.exists(path):
            with open(path, "w", encoding="utf-8", newline="") as f:
                f.write(header)
        attempt = self._next_attempt(path)
        ts = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(finish_t))
        with open(path, "a", encoding="utf-8", newline="") as f:
            f.write(
                f"{ts},{attempt},{elapsed:.3f},{sx:.3f},{sy:.3f},{fx:.3f},{fy:.3f},{start_t:.3f},{finish_t:.3f},{status}\n")
        print(f"[udp_diff] saved result attempt={attempt} elapsed={elapsed:.3f}s status={status} → {path}")

    def _setup_devices(self):
        # RGB
        try:
            self.rgb = self.robot.getDevice('rgb')
            if self.rgb: self.rgb.enable(self.ts)
        except Exception: self.rgb = None
        # Depth
        try:
            self.depth = self.robot.getDevice('depth')
            if self.depth: self.depth.enable(self.ts)
        except Exception: self.depth = None

        if self.rgb:
            fov = self.rgb.getFov() if hasattr(self.rgb,'getFov') else 0.0
            print(f"[udp_diff] rgb: {self.rgb.getWidth()}x{self.rgb.getHeight()} fov={fov:.3f}", flush=True)
        if self.depth:
            fov = self.depth.getFov() if hasattr(self.depth,'getFov') else 0.0
            mn  = self.depth.getMinRange() if hasattr(self.depth,'getMinRange') else 0.0
            mx  = self.depth.getMaxRange() if hasattr(self.depth,'getMaxRange') else 0.0
            print(f"[udp_diff] depth: {self.depth.getWidth()}x{self.depth.getHeight()} fov={fov:.3f} min={mn:.2f} max={mx:.2f}", flush=True)
    

    # ---------- packing ----------
    def _pack_depth(self):
        w = self.depth.getWidth(); h = self.depth.getHeight(); ds = self.depth_downsample
        dw = (w + ds - 1)//ds; dh = (h + ds - 1)//ds
        buf = self.depth.getRangeImage()
        minR = self.depth.getMinRange() if hasattr(self.depth,'getMinRange') else 0.0
        maxR = self.depth.getMaxRange() if hasattr(self.depth,'getMaxRange') else 0.0
        out = bytearray()
        out += DEPTH_MAGIC
        out += struct.pack(DEPTH_HDR, dw, dh, ds, float(minR), float(maxR))
        for v in range(0, h, ds):
            base = v * w
            for u in range(0, w, ds):
                z = buf[base + u]
                if not (z > 0.0) or math.isinf(z) or z != z:
                    mm = 0
                else:
                    mm = int(min(65535, round(z * 1000.0)))
                out += struct.pack("<H", mm)
        return bytes(out)

    def _pack_rgb(self):
        w = self.rgb.getWidth(); h = self.rgb.getHeight(); ds = self.rgb_downsample
        dw = (w + ds - 1)//ds; dh = (h + ds - 1)//ds
        raw = self.rgb.getImage()   # BGRA
        out = bytearray()
        out += RGB_MAGIC
        out += struct.pack(RGB_HDR, dw, dh, ds)
        stride = w * 4
        for v in range(0, h, ds):
            row = v * stride
            for u in range(0, w, ds):
                i = row + u * 4
                out += raw[i:i+4]
        return bytes(out)

    # ================================================================
    # Сеть
    # ================================================================
    def _setup_cmd_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except Exception:
            pass
        sock.bind(("0.0.0.0", CMD_LISTEN_PORT))
        sock.setblocking(False)
        return sock

    def _setup_telemetry_socket(self):
        if TELEMETRY_PROTO == "udp":
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            print(f"[udp_diff] listen cmd on 0.0.0.0:{CMD_LISTEN_PORT} ; TX UDP → {TELEMETRY_HOST}:{TELEMETRY_PORT}")
            return sock
        else:            
            while True:
                try:
                    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    sock.settimeout(2.0)
                    sock.connect((TELEMETRY_HOST, TELEMETRY_PORT))
                    break
                except Exception as e:
                    sock.close()
                    print(f"[udp_diff] TCP connect retry to {TELEMETRY_HOST}:{TELEMETRY_PORT} ({e})")
                    time.sleep(0.5)
            sock.settimeout(None)
            print(f"[udp_diff] listen cmd on 0.0.0.0:{CMD_LISTEN_PORT} ; TX TCP → {TELEMETRY_HOST}:{TELEMETRY_PORT}")
            return sock

    def _send(self, payload: bytes):
        if self.proto == "tcp":
            try:
                self.tx.sendall(struct.pack("<I", len(payload)) + payload)
            except Exception as e:
                sys.stderr.write(f"[udp_diff] TCP send failed: {e}\n"); sys.stderr.flush()
            return

        # UDP
        if (not self.chunking) or len(payload) <= self.max_dgram:
            try:
                self.tx.sendto(payload, (self.host, self.port))
            except Exception as e:
                sys.stderr.write(f"[udp_diff] send failed: {e}\n"); sys.stderr.flush()
            return

        # chunk
        msg_id = next(self.msg_id)
        header_len = 4 + struct.calcsize(CHNK_HDR)
        max_payload = max(1, self.max_dgram - header_len)
        count = (len(payload) + max_payload - 1) // max_payload
        total = len(payload)
        for idx in range(count):
            part = payload[idx*max_payload:(idx+1)*max_payload]
            chunk = bytearray()
            chunk += CHNK_MAGIC
            chunk += struct.pack(CHNK_HDR, msg_id, total, idx, count)
            chunk += part
            try:
                self.tx.sendto(chunk, (self.host, self.port))
            except Exception as e:
                sys.stderr.write(f"[udp_diff] send(ch{idx}/{count}) failed: {e}\n"); sys.stderr.flush()



    # ================================================================
    # Одометрия
    # ================================================================
    def update_encoder_odometry(self):
        a_lf = self.lf_enc.getValue() if self.lf_enc else None
        a_lr = self.lr_enc.getValue() if self.lr_enc else None
        a_rf = self.rf_enc.getValue() if self.rf_enc else None
        a_rr = self.rr_enc.getValue() if self.rr_enc else None
        if None in (a_lf, a_lr, a_rf, a_rr):
            return
        if self.last_lf is None:
            self.last_lf, self.last_lr, self.last_rf, self.last_rr = a_lf, a_lr, a_rf, a_rr
            return

        d_lf = (a_lf - self.last_lf) * WHEEL_RADIUS
        d_lr = (a_lr - self.last_lr) * WHEEL_RADIUS
        d_rf = (a_rf - self.last_rf) * WHEEL_RADIUS
        d_rr = (a_rr - self.last_rr) * WHEEL_RADIUS
        self.last_lf, self.last_lr, self.last_rf, self.last_rr = a_lf, a_lr, a_rf, a_rr

        d_left = 0.5 * (d_lf + d_lr)
        d_right = 0.5 * (d_rf + d_rr)
        ds = 0.5 * (d_left + d_right)
        dth = (d_right - d_left) / WHEEL_BASE

        th_mid = self.odom_th + 0.5 * dth
        self.odom_x += ds * math.cos(th_mid)
        self.odom_y += ds * math.sin(th_mid)
        self.odom_th += dth

    # ================================================================
    # Управление движением
    # ================================================================
    def diff_drive(self, v: float, w: float):
        # Ограничение диапазона
        v = max(-MAX_LINEAR, min(MAX_LINEAR, v))
        w = max(-MAX_ANGULAR, min(MAX_ANGULAR, w))

        # Ограничение ускорений
        dv = v - self.current_v
        dw = w - self.current_w
        max_dv = MAX_LINEAR_ACC * (TIME_STEP_MS / 1000.0)
        max_dw = MAX_ANGULAR_ACC * (TIME_STEP_MS / 1000.0)
        if abs(dv) > max_dv:
            v = self.current_v + max_dv * (1 if dv > 0 else -1)
        if abs(dw) > max_dw:
            w = self.current_w + max_dw * (1 if dw > 0 else -1)

        self.current_v, self.current_w = v, w

        # Пересчёт в скорости колёс
        v_l = v - (w * WHEEL_BASE / 2)
        v_r = v + (w * WHEEL_BASE / 2)
        k = MAX_VEL / max(1.0, abs(v_l), abs(v_r))
        vl, vr = v_l * k, v_r * k
        for m in [self.LF, self.LR]:
            if m: m.setVelocity(vl)
        for m in [self.RF, self.RR]:
            if m: m.setVelocity(vr)

    # ================================================================
    # Основной цикл
    # ================================================================
    def run(self):
        linear_x = angular_z = 0.0
        last_log = 0.0

        try:
            while self.robot.step(TIME_STEP_MS) != -1:
                # Приём команд
                try:
                    data, _ = self.sock_in.recvfrom(1024)
                    if len(data) >= 8:
                        linear_x, angular_z = struct.unpack("<2f", data[:8])

                except BlockingIOError:
                    pass

                self.diff_drive(linear_x, angular_z)
                self.update_encoder_odometry()

                # GPS
                gx = gy = None
                if self.gps:
                    try:
                        px, py, pz = self.gps.getValues()
                        gx, gy = float(px), float(py)
                        if self.init_gps is None:
                            self.init_gps = (gx, gy)
                    except Exception:
                        pass

                    # --- старт таймера: по реальному сдвигу (GPS) или по команде при отсутствии GPS ---
                    now = time.time()

                # Gyro
                wx = wy = wz = 0.0
                if self.gyro:
                    try:
                        wx, wy, wz = self.gyro.getValues()
                    except Exception:
                        pass

                if not self.started:
                    moved_cmd = (abs(linear_x) > START_CMD_V_THRESH) or (abs(angular_z) > START_CMD_W_THRESH)
                    moved_gps = False
                    if gx is not None and self.init_gps is not None:
                        moved_gps = math.hypot(gx - self.init_gps[0], gy - self.init_gps[1]) >= START_MOVE_DIST
                    if moved_cmd or moved_gps:
                        self.started = True
                        self.start_time = now
                        if gx is not None:
                            self.start_pos = (gx, gy)
                            print(f"[udp_diff] timing START at t={self.start_time:.3f}s ; pos x={gx:.3f} y={gy:.3f}")
                        else:
                            print(f"[udp_diff] timing START at t={self.start_time:.3f}s")

                # --- финиш: вход в центральный квадрат 2×2 клеток по X–Y ---
                if self.started and not self.finished and gx is not None:
                    if abs(gx - FINISH_CX) <= FINISH_HALF_SIZE and abs(gy - FINISH_CY) <= FINISH_HALF_SIZE:
                        self.finished = True
                        self.finish_time = now
                        self.finish_pos = (gx, gy)
                        elapsed = self.finish_time - self.start_time
                        sx, sy = self.start_pos if self.start_pos is not None else (gx, gy)
                        self._write_result_csv(elapsed, self.start_time, self.finish_time, sx, sy, gx, gy,status="finish")
                        print(
                            f"[udp_diff] timing FINISH at t={self.finish_time:.3f}s ; elapsed={elapsed:.3f}s ; pos x={gx:.3f} y={gy:.3f}")

                        # остановить моторы
                        if self.LF: self.LF.setVelocity(0.0)
                        if self.RF: self.RF.setVelocity(0.0)
                        if self.LR: self.LR.setVelocity(0.0)
                        if self.RR: self.RR.setVelocity(0.0)

                        # остановить симуляцию
                        try:
                            if STOP_ON_FINISH == "quit":
                                self.robot.simulationQuit(0)
                            elif STOP_ON_FINISH == "pause":
                                self.robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
                        except Exception:
                            pass
                        break

                if self.depth and self.depth_enable and (self.step % self.depth_send_every == 0):
                    try: self._send(self._pack_depth())
                    except Exception as e: sys.stderr.write(f"[udp_diff] depth pack/send error: {e}\n")

                if self.rgb and self.rgb_enable and (self.step % self.rgb_send_every == 0):
                    try: self._send(self._pack_rgb())
                    except Exception as e: sys.stderr.write(f"[udp_diff] rgb pack/send error: {e}\n")


                # lidar
                # ranges = []
                # if self.lidar:
                #     try:
                #         vals = self.lidar.getRangeImage()
                #         for r in vals:
                #             if r != r or r == float("inf") or r <= 0.0:
                #                 ranges.append(8.0)
                #             else:
                #                 ranges.append(float(r))
                #     except Exception as e:
                #         print(f"[udp_diff] lidar read error: {e}")

                # # Telemetry (no cameras)
                # n = len(ranges)
                payload = b"WBTG" + struct.pack(  # b"WBT2" + struct.pack(
                    "<9f",  # "<6f",
                    self.odom_x, self.odom_y, self.odom_th,  # одометрия
                    self.current_v, 0.0, self.current_w,  # скорости
                    wx, wy, wz  # гироскоп
                )
                # payload += struct.pack("<I", n)
                # if n:
                #     payload += struct.pack(f"<{n}f", *ranges)

                try:
                    if TELEMETRY_PROTO == "udp":
                        self.sock_out.sendto(payload, (TELEMETRY_HOST, TELEMETRY_PORT))
                    else:
                        self.sock_out.sendall(struct.pack("<I", len(payload)) + payload)
                except Exception:
                    if TELEMETRY_PROTO == "tcp":
                        try:
                            self.sock_out.close()
                        except:
                            pass
                        self.sock_out = self._setup_telemetry_socket()

                # Компактное логирование
                # now = time.time()
                # if now - last_log > 1.0:
                #     if ranges:
                #         front = ranges[len(ranges) // 2]
                #         right = ranges[len(ranges) // 4]
                #         left = ranges[3 * len(ranges) // 4]
                #         lidar_info = f"n={n},front={front:.2f},left={left:.2f},right={right:.2f}"
                #     else:
                #         lidar_info = "n=0"

                #     print(
                #         f"[udp_diff] ODOM(x={self.odom_x:.3f},y={self.odom_y:.3f},θ={math.degrees(self.odom_th):.1f}°) "
                #         f"CMD(v={linear_x:.2f},w={angular_z:.2f}) "
                #         f"LIDAR({lidar_info}) "
                #         f"Gyro: wx={wx:.3f}, wy={wy:.3f}, wz={wz:.3f} "
                #         f"TX[{TELEMETRY_PROTO.upper()}→{TELEMETRY_HOST}:{TELEMETRY_PORT}]"
                #     )
                #     last_log = now

        except KeyboardInterrupt:
            print("[udp_diff] KeyboardInterrupt: shutting down gracefully...")
        finally:
            try:
                self.sock_in.close()
            except:
                pass
            try:
                self.sock_out.close()
            except:
                pass
            print("[udp_diff] sockets closed, controller terminated")

if __name__ == "__main__":
    controller = UdpDiffController()
    try:
        controller.run()
    except (KeyboardInterrupt, SystemExit):
        print("[udp_diff] Stopping controller...")
    finally:
        controller.sock_in.close()
        controller.sock_out.close()
        print("[udp_diff] Resources closed, exiting.")
