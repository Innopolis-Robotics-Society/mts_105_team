
#!/usr/bin/env python3
# teleop_sequence_with_logs.py
import time, json, sys, argparse, signal, threading, queue
from datetime import datetime
import serial

def now():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def jsend(ser, obj, txlog=None):
    s = json.dumps(obj)
    ser.write((s + "\r\n").encode("utf-8"))
    if txlog is not None:
        msg = f"[{now()}] [TX] {s}"
        print(msg)
        if txlog: print(msg, file=txlog, flush=True)

def safe_stop(ser, txlog=None):
    try:
        jsend(ser, {"T":13, "X":0, "Z":0}, txlog)
        jsend(ser, {"T":132, "IO1":0, "IO2":0}, txlog)  # LED off
        jsend(ser, {"T":999}, txlog)                    # reset emergency
    except Exception:
        pass

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dev", default="/dev/ttyACM1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--vx", type=float, default=0.20)
    ap.add_argument("--blink_hz", type=float, default=4.0)
    ap.add_argument("--rate", type=float, default=10.0, help="cmd publish Hz")
    ap.add_argument("--poll_hz", type=float, default=5.0, help="status poll Hz (T=130)")
    ap.add_argument("--log", default="controller.log")
    args = ap.parse_args()

    ser = serial.Serial(args.dev, args.baud, timeout=0.05)
    LOG = open(args.log, "a", buffering=1)

    # --- RX-поток: печать и запись в файл ---
    rx_run = True
    def rx_loop():
        while rx_run:
            try:
                line = ser.readline()
                if not line:
                    continue
                s = line.decode("utf-8", "ignore").strip()
                msg = f"[{now()}] [RX] {s}"
                print(msg)
                print(msg, file=LOG, flush=True)
            except Exception:
                break
    rx_th = threading.Thread(target=rx_loop, daemon=True)
    rx_th.start()

    def on_sig(*_):
        nonlocal rx_run
        rx_run = False
        time.sleep(0.05)
        safe_stop(ser, LOG)
        try: ser.close()
        except: pass
        LOG.close()
        sys.exit(0)

    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    # --- инициализация ---
    jsend(ser, {"T":131, "cmd":1}, LOG)    # telemetry stream on
    jsend(ser, {"T":142, "cmd":200}, LOG)  # telemetry period ms
    jsend(ser, {"T":143, "cmd":0}, LOG)    # echo off
    jsend(ser, {"T":12}, LOG)              # motors enable
    time.sleep(0.1)

    # --- сценарий 30 c: FWD 5 → BWD 5 → BLINK 5 → FWD 5 → BWD 5 → BLINK 5 ---
    phase = [
        ("move",  +args.vx, 0.0, 5.0),
        ("move",  -args.vx, 0.0, 5.0),
        ("blink", 0.0,      0.0, 5.0),
        ("move",  +args.vx, 0.0, 5.0),
        ("move",  -args.vx, 0.0, 5.0),
        ("blink", 0.0,      0.0, 5.0),
    ]

    dt_cmd  = 1.0 / max(1e-3, args.rate)
    dt_poll = 1.0 / max(1e-3, args.poll_hz)
    next_poll = time.monotonic()

    try:
        for kind, vx, wz, dur in phase:
            t0 = time.monotonic()
            if kind == "move":
                while (time.monotonic() - t0) < dur:
                    jsend(ser, {"T":13, "X":vx, "Z":wz}, LOG)
                    # запрос статуса по расписанию
                    now_t = time.monotonic()
                    if now_t >= next_poll:
                        jsend(ser, {"T":130}, LOG)   # one-shot status request
                        next_poll = now_t + dt_poll
                    time.sleep(dt_cmd)
                jsend(ser, {"T":13, "X":0.0, "Z":0.0}, LOG)  # стоп после участка

            elif kind == "blink":
                period = 1.0 / max(0.1, args.blink_hz)
                on = False
                while (time.monotonic() - t0) < dur:
                    on = not on
                    val = 255 if on else 0
                    jsend(ser, {"T":132, "IO1":val, "IO2":val}, LOG)
                    # поддерживаем нулевую скорость
                    jsend(ser, {"T":13, "X":0.0, "Z":0.0}, LOG)
                    now_t = time.monotonic()
                    if now_t >= next_poll:
                        jsend(ser, {"T":130}, LOG)
                        next_poll = now_t + dt_poll
                    time.sleep(period/2.0)

        safe_stop(ser, LOG)
    finally:
        rx_run = False
        time.sleep(0.05)
        try: ser.close()
        except: pass
        LOG.close()

if name == "main":
    main()
