#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RGB-D Receiver for Webots udp_diff_* controllers
- Works with UDP (with/without chunking) and TCP
- Parses:
    * WBTR (RGB)  : 'WBTR' + <HHi> + raw BGRA bytes
    * WBTD (Depth): 'WBTD' + <HHi f f> + uint16 mm array
- Shows two OpenCV windows: "RGB" and "Depth (m)"
Dependencies: pip install numpy opencv-python
Usage examples:
  # UDP
  python3 receiver_rgbd.py --proto udp --host 0.0.0.0 --port 5600
  # TCP (connects to controller)
  python3 receiver_rgbd.py --proto tcp --host 127.0.0.1 --port 5600
"""

import argparse
import socket
import struct
import collections
import sys
import time

import numpy as np
import cv2

MAGIC_RGB  = b"WBTR"
HDR_RGB    = "<HHi"       # dw, dh, ds
MAGIC_DEPTH= b"WBTD"
HDR_DEPTH  = "<HHi f f"   # dw, dh, ds, minR, maxR
MAGIC_CHNK = b"CHNK"
HDR_CHNK   = "<I I H H"   # msg_id, total_len, idx, count

def parse_rgb(payload: bytes):
    # WBTR + <HHi> + pixels(BGRA)
    dw, dh, ds = struct.unpack_from(HDR_RGB, payload, 4)
    off = 4 + struct.calcsize(HDR_RGB)
    raw = payload[off: off + dw*dh*4]
    if len(raw) != dw*dh*4:
        raise ValueError("RGB payload size mismatch")
    img_bgra = np.frombuffer(raw, dtype=np.uint8).reshape((dh, dw, 4))
    img_bgr  = cv2.cvtColor(img_bgra, cv2.COLOR_BGRA2BGR)
    return img_bgr

def parse_depth(payload: bytes):
    # WBTD + <HHi f f> + uint16[dw*dh] (mm)
    dw, dh, ds, minR, maxR = struct.unpack_from(HDR_DEPTH, payload, 4)
    off = 4 + struct.calcsize(HDR_DEPTH)
    raw = payload[off: off + dw*dh*2]
    if len(raw) != dw*dh*2:
        raise ValueError("Depth payload size mismatch")
    mm = np.frombuffer(raw, dtype=np.uint16).reshape((dh, dw))
    depth_m = mm.astype(np.float32) / 1000.0  # to meters
    # visualize: clamp & normalize for display
    vis = depth_m.copy()
    # set zeros (invalid) to maxR for nicer appearance
    vis[vis <= 0] = maxR if maxR > 0 else vis.max(initial=5.0)
    vis = np.clip(vis, 0, maxR if maxR>0 else vis.max(initial=5.0))
    vis_norm = (vis / vis.max(initial=1.0) * 255.0).astype(np.uint8)
    vis_color = cv2.applyColorMap(255 - vis_norm, cv2.COLORMAP_TURBO)
    return depth_m, vis_color

def recv_all(sock, n):
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("TCP connection closed")
        buf += chunk
    return bytes(buf)

def tcp_loop(host, port):
    print(f"[receiver] TCP connect to {host}:{port}")
    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.connect((host, port))
    s.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

    rgb_img = None
    depth_vis = None

    while True:
        # framing: <u32 length> + payload
        hdr = recv_all(s, 4)
        (length,) = struct.unpack("<I", hdr)
        payload = recv_all(s, length)

        if payload.startswith(MAGIC_RGB):
            rgb_img = parse_rgb(payload)
        elif payload.startswith(MAGIC_DEPTH):
            _, depth_vis = parse_depth(payload)

        # draw windows
        if rgb_img is not None:
            cv2.imshow("RGB", rgb_img)
        if depth_vis is not None:
            cv2.imshow("Depth (m)", depth_vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    s.close()
    cv2.destroyAllWindows()

def udp_loop(host, port):
    print(f"[receiver] UDP listen on {host}:{port}")
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((host, port))

    # reassembly for chunking
    pending = collections.defaultdict(lambda: {"parts": {}, "count": None, "total": None, "last": time.time()})
    rgb_img = None
    depth_vis = None

    while True:
        pkt, addr = s.recvfrom(65535)

        if pkt.startswith(MAGIC_CHNK):
            msg_id, total, idx, count = struct.unpack_from(HDR_CHNK, pkt, 4)
            part = pkt[4 + struct.calcsize(HDR_CHNK):]
            entry = pending[msg_id]
            entry["parts"][idx] = part
            entry["count"] = count
            entry["total"] = total
            entry["last"] = time.time()

            if len(entry["parts"]) == count:
                # reassemble
                payload = b"".join(entry["parts"][i] for i in range(count))
                del pending[msg_id]
            else:
                # not complete yet
                payload = None
        else:
            payload = pkt

        if not payload:
            # clean stale entries occasionally
            now = time.time()
            stale = [mid for mid,e in pending.items() if now - e["last"] > 5.0]
            for mid in stale: del pending[mid]
            continue

        if payload.startswith(MAGIC_RGB):
            rgb_img = parse_rgb(payload)
        elif payload.startswith(MAGIC_DEPTH):
            _, depth_vis = parse_depth(payload)

        if rgb_img is not None:
            cv2.imshow("RGB", rgb_img)
        if depth_vis is not None:
            cv2.imshow("Depth (m)", depth_vis)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    s.close()
    cv2.destroyAllWindows()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--proto", choices=["udp", "tcp"], default="udp")
    ap.add_argument("--host", default="0.0.0.0", help="UDP bind host or TCP connect host")
    ap.add_argument("--port", type=int, default=5600)
    args = ap.parse_args()

    if args.proto == "udp":
        udp_loop(args.host, args.port)
    else:
        tcp_loop(args.host, args.port)

if __name__ == "__main__":
    main()
