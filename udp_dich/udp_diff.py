# -*- coding: utf-8 -*-
"""
udp_diff_unified.py — единый контроллер Webots.

Функционал:
- Принимает команды v,w по UDP: 0.0.0.0:CMD_LISTEN_PORT
- Дифференциальное управление 4 колёсами
- Плавные ускорения, скоростные лимиты
- Одометрия по энкодерам (x,y,theta)
- Тайминг старта/финиша по GPS и зона финиша
- Гироскоп
- Depth обязателен, RGB опционален
- Телеметрия по UDP или TCP:
    • "WBTG" (odometry, cmd speeds, gyro)
    • "WBTD"/"WBTR" с чанками "CHNK" при необходимости

ENV:
  TELEMETRY_PROTO=tcp|udp (udp)
  TELEMETRY_HOST=127.0.0.1
  TELEMETRY_PORT=5600
  CMD_LISTEN_PORT=5555

  SPEEDUP=2
  ARENA=8 OFFSET=0.045 WALL_COUNT=16 FINISH_CX=0 FINISH_CY=0 STOP_ON_FINISH=pause|quit|none
  START_CMD_V_THRESH=0.01 START_CMD_W_THRESH=0.01 START_MOVE_DIST=0.02
  RESULTS_FILE=./results.csv

  DEPTH_ENABLE=1 DEPTH_SEND_EVERY=3 DEPTH_DOWNSAMPLE=2
  RGB_ENABLE=0   RGB_SEND_EVERY=6   RGB_DOWNSAMPLE=2
  CHUNKING=1 MAX_DGRAM=1200
"""

import os, sys, math, struct, socket, select, time, itertools, traceback
from controller import Supervisor

# ---- wire formats ----
DEPTH_MAGIC=b"WBTD"; DEPTH_HDR="<HHi f f"   # dw, dh, ds, minR, maxR
RGB_MAGIC=b"WBTR";   RGB_HDR  ="<HHi"       # dw, dh, ds
CHNK_MAGIC=b"CHNK";  CHNK_HDR ="<I I H H"   # msg_id, total_len, idx, count

def _env_str(k, d): v=os.environ.get(k); return v if v is not None else d
def _env_int(k, d):
    try: return int(os.environ.get(k, str(d)))
    except: return d
def _env_float(k, d):
    try: return float(os.environ.get(k, str(d)))
    except: return d

class UdpDiffController:
    # --- кинематика и лимиты ---
    WHEEL_BASE   = 0.25   # м
    WHEEL_RADIUS = 0.035  # м
    MAX_WHEEL_ANG = 12.0  # рад/с ограничение мотор-контроллера в Webots

    BASE_MAX_LINEAR      = 0.5   # м/с
    BASE_MAX_ANGULAR     = 1.0   # рад/с
    BASE_MAX_LINEAR_ACC  = 0.05  # м/с^2
    BASE_MAX_ANGULAR_ACC = 0.2   # рад/с^2

    def __init__(self):
        # --- Webots ---
        self.robot = Supervisor()
        self.ts = int(self.robot.getBasicTimeStep()) if hasattr(self.robot,'getBasicTimeStep') else 16
        self.dt = self.ts / 1000.0
        self.step = 0

        # --- ускорение симуляции ---
        self.SPEEDUP = _env_int("SPEEDUP", 2)
        self.MAX_LINEAR      = self.BASE_MAX_LINEAR      * self.SPEEDUP
        self.MAX_ANGULAR     = self.BASE_MAX_ANGULAR     * self.SPEEDUP
        self.MAX_LINEAR_ACC  = self.BASE_MAX_LINEAR_ACC  * self.SPEEDUP
        self.MAX_ANGULAR_ACC = self.BASE_MAX_ANGULAR_ACC * self.SPEEDUP

        # --- сеть ---
        self.proto = _env_str("TELEMETRY_PROTO", "udp").lower()
        self.host  = _env_str("TELEMETRY_HOST", "127.0.0.1")
        self.port  = _env_int("TELEMETRY_PORT", 5600)
        self.cmd_listen_port = _env_int("CMD_LISTEN_PORT", 5555)

        self.chunking  = _env_int("CHUNKING", 1) == 1
        self.max_dgram = max(512, _env_int("MAX_DGRAM", 1200))
        self.msg_id = itertools.count(1)

        self.tx = None          # телеметрия
        self.cmd_rx = None      # приём команд

        # --- финиш тайминг ---
        self.ARENA = _env_float("ARENA", 8.0)
        self.OFFSET = _env_float("OFFSET", 0.045)
        self.WALL_COUNT = _env_float("WALL_COUNT", 16.0)
        self.START_CMD_V_THRESH = _env_float("START_CMD_V_THRESH", 0.01)
        self.START_CMD_W_THRESH = _env_float("START_CMD_W_THRESH", 0.01)
        self.START_MOVE_DIST    = _env_float("START_MOVE_DIST", 0.02)
        self.FINISH_CX = _env_float("FINISH_CX", 0.0)
        self.FINISH_CY = _env_float("FINISH_CY", 0.0)
        self.STOP_ON_FINISH = _env_str("STOP_ON_FINISH", "pause").lower()
        self.FINISH_HALF_SIZE = self.ARENA / self.WALL_COUNT - self.OFFSET
        self.RESULTS_FILE = _env_str("RESULTS_FILE", os.path.join(os.getcwd(), "results.csv"))

        # --- стримы ---
        self.depth_enable      = _env_int("DEPTH_ENABLE", 1) == 1
        self.depth_send_every  = max(1, _env_int("DEPTH_SEND_EVERY", 3))
        self.depth_downsample  = max(1, _env_int("DEPTH_DOWNSAMPLE", 2))
        self.rgb_enable        = _env_int("RGB_ENABLE", 0) == 1
        self.rgb_send_every    = max(1, _env_int("RGB_SEND_EVERY", 6))
        self.rgb_downsample    = max(1, _env_int("RGB_DOWNSAMPLE", 2))

        # --- устройства ---
        self.rgb = None
        self.depth = None
        self.gps = None
        self.gyro = None

        # моторы и энкодеры
        self.LF = self._pick_motor("left_front_motor")
        self.RF = self._pick_motor("right_front_motor")
        self.LR = self._pick_motor("left_rear_motor")
        self.RR = self._pick_motor("right_rear_motor")

        self.lf_enc = self._enable_sensor("left_front_encoder")
        self.lr_enc = self._enable_sensor("left_rear_encoder")
        self.rf_enc = self._enable_sensor("right_front_encoder")
        self.rr_enc = self._enable_sensor("right_rear_encoder")

        self.last_lf = self.last_lr = self.last_rf = self.last_rr = None
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_th = 0.0

        self.current_v = 0.0
        self.current_w = 0.0

        # тайминг заезда
        self.started = False
        self.finished = False
        self.init_gps = None
        self.start_pos = None
        self.finish_pos = None
        self.start_time = None
        self.finish_time = None

        # сокеты
        self._setup_comm()
        self._setup_cmd_listener()

        # датчики
        self._setup_devices()

        print(f"[udp_diff] READY proto={self.proto} dst={self.host}:{self.port} | "
              f"cmd@udp://0.0.0.0:{self.cmd_listen_port} | chunking={self.chunking} max_dgram={self.max_dgram}",
              flush=True)

    # --- устройства ---
    def _pick_motor(self, name):
        try:
            dev = self.robot.getDevice(name)
            dev.setPosition(float("inf"))
            dev.setVelocity(0.0)
            print(f"[udp_diff] motor '{name}' OK")
            return dev
        except Exception:
            print(f"[udp_diff] motor '{name}' missing")
            return None

    def _enable_sensor(self, name):
        try:
            dev = self.robot.getDevice(name)
            dev.enable(self.ts)
            return dev
        except Exception:
            return None

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
        # GPS
        try:
            self.gps = self.robot.getDevice("gps")
            if self.gps: self.gps.enable(self.ts)
        except Exception: self.gps = None
        # Gyro
        try:
            self.gyro = self.robot.getDevice("gyro")
            if self.gyro: self.gyro.enable(self.ts)
        except Exception: self.gyro = None

        if self.rgb:
            fov = self.rgb.getFov() if hasattr(self.rgb,'getFov') else 0.0
            print(f"[udp_diff] rgb: {self.rgb.getWidth()}x{self.rgb.getHeight()} fov={fov:.3f}")
        if self.depth:
            fov = self.depth.getFov() if hasattr(self.depth,'getFov') else 0.0
            mn  = self.depth.getMinRange() if hasattr(self.depth,'getMinRange') else 0.0
            mx  = self.depth.getMaxRange() if hasattr(self.depth,'getMaxRange') else 0.0
            print(f"[udp_diff] depth: {self.depth.getWidth()}x{self.depth.getHeight()} fov={fov:.3f} min={mn:.2f} max={mx:.2f}")
        if self.gps:  print("[udp_diff] gps enabled")
        if self.gyro: print("[udp_diff] gyro enabled")

    # --- сеть ---
    def _setup_comm(self):
        if self.proto == "udp":
            self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.tx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            print(f"[udp_diff] TX UDP → {self.host}:{self.port}")
            return
        # TCP с ретраями
        while True:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2.0)
                s.connect((self.host, self.port))
                s.settimeout(None)
                self.tx = s
                print(f"[udp_diff] TX TCP → {self.host}:{self.port}")
                break
            except Exception as e:
                try: s.close()
                except: pass
                print(f"[udp_diff] TCP connect retry to {self.host}:{self.port} ({e})")
                time.sleep(0.5)

    def _setup_cmd_listener(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try: s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        except Exception: pass
        s.bind(("0.0.0.0", self.cmd_listen_port))
        s.setblocking(False)
        self.cmd_rx = s
        print(f"[udp_diff] listen cmd on 0.0.0.0:{self.cmd_listen_port}")

    def _send(self, payload: bytes):
        if self.proto == "tcp":
            try:
                self.tx.sendall(struct.pack("<I", len(payload)) + payload)
            except Exception as e:
                sys.stderr.write(f"[udp_diff] TCP send failed: {e}\n"); sys.stderr.flush()
                try: self.tx.close()
                except: pass
                self._setup_comm()
            return
        # UDP без чанков
        if (not self.chunking) or len(payload) <= self.max_dgram:
            try:
                self.tx.sendto(payload, (self.host, self.port))
            except Exception as e:
                sys.stderr.write(f"[udp_diff] UDP send failed: {e}\n"); sys.stderr.flush()
            return
        # UDP чанки
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
                sys.stderr.write(f"[udp_diff] send(ch {idx+1}/{count}) failed: {e}\n"); sys.stderr.flush()

    # --- упаковка кадров ---
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

    # --- одометрия ---
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

        d_lf = (a_lf - self.last_lf) * self.WHEEL_RADIUS
        d_lr = (a_lr - self.last_lr) * self.WHEEL_RADIUS
        d_rf = (a_rf - self.last_rf) * self.WHEEL_RADIUS
        d_rr = (a_rr - self.last_rr) * self.WHEEL_RADIUS
        self.last_lf, self.last_lr, self.last_rf, self.last_rr = a_lf, a_lr, a_rf, a_rr

        d_left  = 0.5 * (d_lf + d_lr)
        d_right = 0.5 * (d_rf + d_rr)
        ds  = 0.5 * (d_left + d_right)
        dth = (d_right - d_left) / self.WHEEL_BASE

        th_mid = self.odom_th + 0.5 * dth
        self.odom_x += ds * math.cos(th_mid)
        self.odom_y += ds * math.sin(th_mid)
        self.odom_th += dth

    # --- дифпривод ---
    def diff_drive(self, v: float, w: float):
        # лимиты
        v = max(-self.MAX_LINEAR,  min(self.MAX_LINEAR,  v))
        w = max(-self.MAX_ANGULAR, min(self.MAX_ANGULAR, w))
        # аксель
        dv = v - self.current_v
        dw = w - self.current_w
        max_dv = self.MAX_LINEAR_ACC  * self.dt
        max_dw = self.MAX_ANGULAR_ACC * self.dt
        if abs(dv) > max_dv: v = self.current_v + max_dv * (1 if dv > 0 else -1)
        if abs(dw) > max_dw: w = self.current_w + max_dw * (1 if dw > 0 else -1)
        self.current_v, self.current_w = v, w

        # в скорости колёс
        v_l = v - (w * self.WHEEL_BASE * 0.5)
        v_r = v + (w * self.WHEEL_BASE * 0.5)
        w_l = v_l / self.WHEEL_RADIUS  # рад/с
        w_r = v_r / self.WHEEL_RADIUS

        # масштаб до лимита моторов
        scale = max(1.0, abs(w_l)/self.MAX_WHEEL_ANG, abs(w_r)/self.MAX_WHEEL_ANG)
        w_l /= scale; w_r /= scale

        for m in (self.LF, self.LR):
            if m: m.setVelocity(w_l)
        for m in (self.RF, self.RR):
            if m: m.setVelocity(w_r)

    # --- CSV ---
    def _next_attempt(self, path):
        try:
            with open(path, "r", encoding="utf-8") as f:
                n = sum(1 for _ in f) - 1
                return max(1, n + 1)
        except Exception:
            return 1

    def _write_result_csv(self, elapsed, start_t, finish_t, sx, sy, fx, fy, status=None):
        path = self.RESULTS_FILE
        header = "timestamp_iso,attempt,elapsed_s,start_x,start_y,finish_x,finish_y,start_t,finish_t,status\n"
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        if not os.path.exists(path):
            with open(path, "w", encoding="utf-8", newline="") as f: f.write(header)
        attempt = self._next_attempt(path)
        ts = time.strftime("%Y-%m-%dT%H:%M:%S", time.localtime(finish_t))
        with open(path, "a", encoding="utf-8", newline="") as f:
            f.write(f"{ts},{attempt},{elapsed:.3f},{sx:.3f},{sy:.3f},{fx:.3f},{fy:.3f},{start_t:.3f},{finish_t:.3f},{status}\n")
        print(f"[udp_diff] saved result attempt={attempt} elapsed={elapsed:.3f}s status={status} → {path}")

    # --- команды ---
    def _recv_cmd(self):
        try:
            data, _ = self.cmd_rx.recvfrom(1024)
            if len(data) >= 8:
                return struct.unpack("<2f", data[:8])  # v, w
        except BlockingIOError:
            pass
        except Exception as e:
            sys.stderr.write(f"[udp_diff] cmd rx error: {e}\n")
        return None

    # --- основной цикл ---
    def run(self):
        linear_x = 0.0
        angular_z = 0.0
        try:
            while self.robot.step(self.ts) != -1:
                self.step += 1
                now = time.time()

                cmd = self._recv_cmd()
                if cmd is not None:
                    linear_x, angular_z = cmd

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

                # Gyro
                wx = wy = wz = 0.0
                if self.gyro:
                    try:
                        wx, wy, wz = self.gyro.getValues()
                    except Exception:
                        pass

                # старт таймера
                if not self.started:
                    moved_cmd = (abs(linear_x) > self.START_CMD_V_THRESH) or (abs(angular_z) > self.START_CMD_W_THRESH)
                    moved_gps = False
                    if gx is not None and self.init_gps is not None:
                        moved_gps = math.hypot(gx - self.init_gps[0], gy - self.init_gps[1]) >= self.START_MOVE_DIST
                    if moved_cmd or moved_gps:
                        self.started = True
                        self.start_time = now
                        if gx is not None:
                            self.start_pos = (gx, gy)
                            print(f"[udp_diff] START t={self.start_time:.3f}s pos x={gx:.3f} y={gy:.3f}")
                        else:
                            print(f"[udp_diff] START t={self.start_time:.3f}s")

                # финиш
                if self.started and not self.finished and gx is not None:
                    in_cx = abs(gx - self.FINISH_CX) <= self.FINISH_HALF_SIZE
                    in_cy = abs(gy - self.FINISH_CY) <= self.FINISH_HALF_SIZE
                    if in_cx and in_cy:
                        self.finished = True
                        self.finish_time = now
                        self.finish_pos = (gx, gy)
                        elapsed = self.finish_time - self.start_time
                        sx, sy = self.start_pos if self.start_pos else (gx, gy)
                        self._write_result_csv(elapsed, self.start_time, self.finish_time, sx, sy, gx, gy, status="finish")
                        print(f"[udp_diff] FINISH t={self.finish_time:.3f}s elapsed={elapsed:.3f}s pos x={gx:.3f} y={gy:.3f}")
                        # стоп
                        for m in (self.LF, self.RF, self.LR, self.RR):
                            if m: m.setVelocity(0.0)
                        try:
                            if self.STOP_ON_FINISH == "quit":
                                self.robot.simulationQuit(0)
                            elif self.STOP_ON_FINISH == "pause":
                                self.robot.simulationSetMode(Supervisor.SIMULATION_MODE_PAUSE)
                        except Exception:
                            pass
                        break

                # отправка кадров
                if self.depth and self.depth_enable and (self.step % self.depth_send_every == 0):
                    try: self._send(self._pack_depth())
                    except Exception as e: sys.stderr.write(f"[udp_diff] depth send error: {e}\n")
                if self.rgb and self.rgb_enable and (self.step % self.rgb_send_every == 0):
                    try: self._send(self._pack_rgb())
                    except Exception as e: sys.stderr.write(f"[udp_diff] rgb send error: {e}\n")

                # компактная телеметрия WBTG
                payload = b"WBTG" + struct.pack(
                    "<9f",
                    self.odom_x, self.odom_y, self.odom_th,
                    self.current_v, 0.0, self.current_w,
                    wx, wy, wz
                )
                self._send(payload)

        except KeyboardInterrupt:
            print("[udp_diff] KeyboardInterrupt")
        finally:
            try: self.cmd_rx.close()
            except: pass
            try: self.tx.close()
            except: pass
            print("[udp_diff] sockets closed, controller terminated")

def main():
    c = UdpDiffController()
    c.run()

if __name__ == "__main__":
    main()

