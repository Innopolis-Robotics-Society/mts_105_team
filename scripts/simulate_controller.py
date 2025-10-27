#!/usr/bin/env python3
import os, sys, json, time, threading, serial

PORT = "/tmp/ttyACM1"   # второй конец
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0)
lock = threading.Lock()
state = {
    "enabled": False, "echo": False, "period_ms": 200,
    "odl": 0, "odr": 0, "v": 1021, "m": [0,0,0,0]
}
last_tx = time.time()

def send(js):
    b = json.dumps(js, separators=(",",":")).encode("ascii")
    with lock: ser.write(b)  # без \r\n

def telemetry():
    global last_tx
    while True:
        now = time.time()
        if state["enabled"] and (now - last_tx) >= state["period_ms"]/1000.0:
            last_tx = now
            send({"T":1001,"M1":state["m"][0],"M2":state["m"][1],
                  "M3":state["m"][2],"M4":state["m"][3],
                  "odl":state["odl"],"odr":state["odr"],"v":state["v"]})
        time.sleep(0.002)

def rx_loop():
    buf = bytearray()
    depth = 0
    while True:
        chunk = ser.read(512)
        if not chunk:
            time.sleep(0.001); continue
        buf += chunk
        i = 0
        while i < len(buf):
            while i < len(buf) and buf[i] != ord('{'):
                i += 1
            if i >= len(buf): break
            d = 0; j = i
            while j < len(buf):
                if buf[j] == ord('{'): d += 1
                elif buf[j] == ord('}'):
                    d -= 1
                    if d == 0:
                        frame = bytes(buf[i:j+1])
                        handle(frame)
                        i = j + 1
                        break
                j += 1
            if j >= len(buf): break
        if i > 0:
            del buf[:i]
        if len(buf) > 16384:
            del buf[:-1024]

def handle(frame):
    try:
        js = json.loads(frame.decode("ascii"))
    except Exception:
        return
    t = js.get("T")
    # init / echo / period / enable motors
    if t == 131 and js.get("cmd") == 1:
        state["enabled"] = True
    elif t == 142:
        state["period_ms"] = int(js.get("cmd", 200))
    elif t == 143:
        state["echo"] = bool(js.get("cmd", 0))
    elif t == 12:
        pass
    # drive cmd
    elif t == 13:
        vx = float(js.get("X",0.0)); wz = float(js.get("Z",0.0))
        # примитивная модель: обновляем «моторы» и одометрию
        ml = int(500*vx - 200*wz); mr = int(500*vx + 200*wz)
        state["m"] = [ml, mr, ml, mr]
        state["odl"] += max(-5, min(5, ml//100))
        state["odr"] += max(-5, min(5, mr//100))
    # poll
    elif t == 130:
        send({"T":1001,"M1":state["m"][0],"M2":state["m"][1],
              "M3":state["m"][2],"M4":state["m"][3],
              "odl":state["odl"],"odr":state["odr"],"v":state["v"]})
    # optional echo
    if state["echo"]:
        send(js)

if __name__ == "__main__":
    threading.Thread(target=telemetry, daemon=True).start()
    rx_loop()
