#!/usr/bin/env python3
# cobra_sim_pty.py — простой симулятор контроллера Cobra по сериалу (PTY)
import os, sys, json, time, argparse, math, signal
from datetime import datetime

def ts(): return datetime.now().strftime("%H:%M:%S.%f")[:-3]

def extract_json_objects(buf: bytearray):
    """Разбор потока по балансировке скобок вне строк."""
    out = []
    depth = 0
    start = None
    in_str = False
    esc = False
    for i,b in enumerate(buf):
        c = chr(b)
        if in_str:
            if esc: esc = False
            elif c == '\\': esc = True
            elif c == '"': in_str = False
            continue
        if c == '"': in_str = True; continue
        if c == '{':
            if depth == 0: start = i
            depth += 1
        elif c == '}':
            depth -= 1
            if depth == 0 and start is not None:
                out.append(bytes(buf[start:i+1]).decode('utf-8', 'ignore'))
                start = None
    # обрезаем разобранную часть
    if out:
        last = out[-1].encode('utf-8')
        pos = buf.rfind(last)
        if pos != -1: del buf[:pos+len(last)]
    elif len(buf) > 65536:
        del buf[:len(buf)-1024]
    return out

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tpm", type=float, default=1000.0, help="ticks per meter")
    ap.add_argument("--track", type=float, default=0.228, help="колея, м")
    ap.add_argument("--wheel_d", type=float, default=0.0745, help="диаметр колеса, м (для T=1)")
    ap.add_argument("--link", default="", help="создать symlink на PTY slave, напр. /tmp/ttyACM1")
    ap.add_argument("--start_telemetry", action="store_true", help="включить телеметрию при старте (T=131,cmd=1)")
    ap.add_argument("--period_ms", type=int, default=200, help="период телеметрии по умолчанию")
    args = ap.parse_args()

    # создаём PTY
    master_fd, slave_fd = os.openpty()
    slave_path = os.ttyname(slave_fd)
    if args.link:
        try:
            if os.path.islink(args.link) or os.path.exists(args.link):
                os.unlink(args.link)
            os.symlink(slave_path, args.link)
            slave_path = args.link
        except Exception as e:
            print(f"[{ts()}] symlink failed: {e}", file=sys.stderr)
    print(f"[{ts()}] PTY ready. Use port: {slave_path}", flush=True)

    # состояние «контроллера»
    telemetry_on = bool(args.start_telemetry)
    period_ms = int(args.period_ms)
    tpm = float(args.tpm)
    track = float(args.track)
    wheel_r = float(args.wheel_d) * 0.5

    # модель одометрии
    odl = 0
    odr = 0
    vx = 0.0
    wz = 0.0
    # для T=1 (L,R в десятых об/мин)
    rpm10_L = 0
    rpm10_R = 0
    use_cmd13 = True  # последняя команда — T=13, иначе T=1

    # «моторные» величины для телеметрии
    M1=M2=M3=M4=0
    v_code = 1021  # как в примерах логов

    # настройка неблокирующего чтения
    os.set_blocking(master_fd, False)

    running = True
    def on_sig(*_): 
        nonlocal running
        running = False
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    rx_buf = bytearray()
    next_telemetry = time.monotonic() + period_ms/1000.0
    last_update = time.monotonic()

    def send(obj):
        s = json.dumps(obj, separators=(',',':'))
        os.write(master_fd, (s + "\r\n").encode('utf-8'))
        print(f"[{ts()}] [TX] {s}")

    print(f"[{ts()}] Simulator started. Telemetry={'ON' if telemetry_on else 'OFF'} period={period_ms}ms")

    while running:
        now = time.monotonic()
        dt = max(0.0, now - last_update)
        last_update = now

        # интеграция одометрии
        if use_cmd13:
            vl = vx - wz * (track * 0.5)
            vr = vx + wz * (track * 0.5)
        else:
            # T=1: RPM*10 -> м/с
            rl = (rpm10_L/10.0) / 60.0 * 2.0*math.pi
            rr = (rpm10_R/10.0) / 60.0 * 2.0*math.pi
            vl = rl * wheel_r
            vr = rr * wheel_r
        odl += int(round(vl * dt * tpm))
        odr += int(round(vr * dt * tpm))

        # телеметрия по таймеру
        if telemetry_on and now >= next_telemetry:
            # простая имитация токов моторов по модулю скорости
            M1 = M2 = int(min(800, abs(vl)*800))
            M3 = M4 = int(min(800, abs(vr)*800))
            send({"T":1001,"M1":M1,"M2":M2,"M3":M3,"M4":M4,"odl":odl,"odr":odr,"v":v_code})
            next_telemetry = now + period_ms/1000.0

        # приём входящих данных
        try:
            chunk = os.read(master_fd, 4096)
            if chunk:
                rx_buf.extend(chunk)
                for js in extract_json_objects(rx_buf):
                    print(f"[{ts()}] [RX] {js}")
                    try:
                        obj = json.loads(js)
                    except Exception:
                        continue
                    T = obj.get("T", None)

                    # команды управления
                    if T == 13:
                        vx = float(obj.get("X", 0.0))
                        wz = float(obj.get("Z", 0.0))
                        use_cmd13 = True
                    elif T == 1:
                        rpm10_L = int(obj.get("L", 0))
                        rpm10_R = int(obj.get("R", 0))
                        use_cmd13 = False

                    # сервисные
                    elif T == 12:
                        # enable motors — можно подтвердить телеметрией
                        pass
                    elif T == 131:
                        telemetry_on = bool(int(obj.get("cmd", 1)))
                    elif T == 142:
                        # период телеметрии в мс
                        newp = int(obj.get("cmd", period_ms))
                        period_ms = max(20, newp)
                        next_telemetry = time.monotonic() + period_ms/1000.0
                    elif T == 143:
                        # echo off/on — игнорируем, т.к. логируем сами
                        pass
                    elif T == 130:
                        # разовый ответ статуса
                        M1 = M2 = int(min(800, abs(vx)*800))
                        M3 = M4 = int(min(800, abs(vx)*800))
                        send({"T":1001,"M1":M1,"M2":M2,"M3":M3,"M4":M4,"odl":odl,"odr":odr,"v":v_code})
                    elif T == 999:
                        # ресет аварии — ничего не делаем тут
                        pass
            else:
                time.sleep(0.002)
        except BlockingIOError:
            time.sleep(0.002)
        except OSError:
            break

    print(f"[{ts()}] Simulator stopped.")

if __name__ == "__main__":
    main()
# python3 src/scripts/simulate_controller.py --link /tmp/ttyACM1
# ros2 run a105_cobra_driver cmdvel_to_cobra --ros-args -p port:=/tmp/ttyACM1
# ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}' -r 5
# ros2 topic echo /odom
