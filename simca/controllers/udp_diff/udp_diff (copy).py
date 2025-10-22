# -*- coding: utf-8 -*-
"""
udp_diff_retry_chunked.py — контроллер с:
  • RGB-D (depth обяз., RGB опц.)
  • Безопасной отправкой по UDP (чанки) или TCP (поток)
  • TCP connect с ретраями (как в твоём diff)
  • Отдельным UDP-сокетом для приёма команд на 0.0.0.0:CMD_LISTEN_PORT

ENV:
  TELEMETRY_PROTO=tcp|udp (udp)
  TELEMETRY_HOST=127.0.0.1
  TELEMETRY_PORT=5600
  CMD_LISTEN_PORT=5555              # слушаем команды на UDP (не блокирующе)

  # Depth:
  DEPTH_ENABLE=1
  DEPTH_SEND_EVERY=3
  DEPTH_DOWNSAMPLE=2

  # RGB:
  RGB_ENABLE=0
  RGB_SEND_EVERY=6
  RGB_DOWNSAMPLE=2

  # UDP chunking:
  CHUNKING=1
  MAX_DGRAM=1200
"""
import os, sys, math, struct, socket, select, time, itertools, traceback

try:
    from controller import Robot
except Exception:
    # для линтера
    class Robot: pass

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
        self.robot = Robot()
        self.ts = int(self.robot.getBasicTimeStep()) if hasattr(self.robot,'getBasicTimeStep') else 16

        # net
        self.proto = _env_str("TELEMETRY_PROTO", "udp").lower()
        self.host  = _env_str("TELEMETRY_HOST", "127.0.0.1")
        self.port  = _env_int("TELEMETRY_PORT", 5600)
        self.cmd_listen_port = _env_int("CMD_LISTEN_PORT", 5555)

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

        # sockets
        self.tx = None
        self.cmd_rx = None

        # devices
        self.rgb = None
        self.depth = None

        self.step = 0
        self.msg_id = itertools.count(1)

        self._setup_comm()          # включает TCP retry (см. ниже)
        self._setup_cmd_listener()  # UDP на 0.0.0.0:CMD_LISTEN_PORT
        self._setup_devices()

        print(f"[udp_diff] READY: proto={self.proto} dst={self.host}:{self.port} | "
              f"cmd@udp://0.0.0.0:{self.cmd_listen_port} | chunking={self.chunking} max_dgram={self.max_dgram}",
              flush=True)

    # ---------- sockets ----------
    def _setup_comm(self):
        if self.proto == "udp":
            self.tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.tx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            print(f"[udp_diff] TX UDP → {self.host}:{self.port}", flush=True)
            return

        # TCP with retry loop (как в твоём diff)
        while True:
            try:
                s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                s.settimeout(2.0)
                s.connect((self.host, self.port))
                s.settimeout(None)
                self.tx = s
                print(f"[udp_diff] TX TCP → {self.host}:{self.port}", flush=True)
                break
            except Exception as e:
                try: s.close()
                except: pass
                print(f"[udp_diff] TCP connect retry to {self.host}:{self.port} ({e})", flush=True)
                time.sleep(0.5)

    def _setup_cmd_listener(self):
        # простой неблокирующий UDP сокет под ваши команды (приём)
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind(("0.0.0.0", self.cmd_listen_port))
            s.setblocking(False)
            self.cmd_rx = s
            print(f"[udp_diff] listen cmd on 0.0.0.0:{self.cmd_listen_port}", flush=True)
        except Exception as e:
            print(f"[udp_diff] cmd listen failed: {e}", file=sys.stderr, flush=True)

    # ---------- devices ----------
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

    # ---------- send ----------
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

    # ---------- loop ----------
    def _poll_cmd(self):
        if not self.cmd_rx: return
        try:
            r,_,_ = select.select([self.cmd_rx], [], [], 0.0)
            if not r: return
            data, addr = self.cmd_rx.recvfrom(2048)
            # тут обработай свои команды; пример: pong
            if data == b'ping':
                self.tx.sendto(b'pong', (self.host, self.port)) if self.proto=='udp' else self.tx.sendall(struct.pack("<I",4)+b'pong')
        except Exception:
            pass

    def run(self):
        while True:
            if hasattr(self.robot,'step') and self.robot.step(self.ts) == -1:
                break
            self.step += 1
            self._poll_cmd()

            if self.depth and self.depth_enable and (self.step % self.depth_send_every == 0):
                try: self._send(self._pack_depth())
                except Exception as e: sys.stderr.write(f"[udp_diff] depth pack/send error: {e}\n")

            if self.rgb and self.rgb_enable and (self.step % self.rgb_send_every == 0):
                try: self._send(self._pack_rgb())
                except Exception as e: sys.stderr.write(f"[udp_diff] rgb pack/send error: {e}\n")

def main():
    c = UdpDiffController()
    c.run()

if __name__ == "__main__":
    main()
