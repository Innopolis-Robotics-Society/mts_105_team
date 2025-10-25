#!/usr/bin/env bash
# usage: ./teleop_serial.sh [/dev/ttyACM1] [115200]

set -euo pipefail

DEV="${1:-/dev/ttyACM1}"
BAUD="${2:-115200}"

# --- serial setup ---
if [ ! -e "$DEV" ]; then
  echo "no device $DEV" >&2; exit 1
fi
stty -F "$DEV" "$BAUD" -echo -icrnl -ixon -ixoff -crtscts raw

# --- helpers ---
send() { printf '%s\r\n' "$1" > "$DEV"; }
cleanup() {
  # stop robot and restore tty
  send '{"T":13,"X":0,"Z":0}'
  send '{"T":999}'   # reset emergency flag (на всякий случай)
  stty -F "$DEV" sane || true
  kill "$READER_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

# --- start background reader ---
# печатаем входящие строки с пометкой [RX]
( stdbuf -oL tr -d '\r' < "$DEV" | sed -u 's/^/[RX] /' ) &
READER_PID=$!

# --- init controller (безопасные дефолты) ---
send '{"T":12}'                    # MOTOR_ENABLED
send '{"T":131,"cmd":1}'           # включить поток телеметрии
send '{"T":142,"cmd":200}'         # период телеметрии ~200 мс
send '{"T":143,"cmd":0}'           # echo off (чтоб не дублировались команды)
send '{"T":130}'                   # разовый запрос базовой телеметрии

# --- teleop loop ---
X=0.0
Z=0.0
PERIOD=0.10   # сек
LAST_SEND=$(date +%s%3N)

# включаем неканоничный ввод с немедленным возвратом символов
stty -icanon -echo min 0 time 1

echo "[INFO] keys: W/S/A/D, SPACE=stop, E=EMG, M=motors on, Q=quit"

while true; do
  # читаем одну клавишу, если есть
  if IFS= read -rsn1 key; then
    case "$key" in
      [Ww]) X=0.20; Z=0.0 ;;
      [Ss]) X=-0.20; Z=0.0 ;;
      [Aa]) X=0.0;  Z=0.60 ;;
      [Dd]) X=0.0;  Z=-0.60 ;;
      " ")  X=0.0;  Z=0.0 ;;
      [Ee]) send '{"T":0}'; X=0.0; Z=0.0 ;;   # EMERGENCY STOP
      [Mm]) send '{"T":12}' ;;                # re-enable motors
      [Qq]) exit 0 ;;
      *) : ;;
    esac
  fi

  # периодически посылать текущую команду скорости
  now=$(date +%s%3N)
  dt=$(( now - LAST_SEND ))
  if [ "$dt" -ge $(( PERIOD*1000 )) ]; then
    # форматируем числа с точкой
    Xs=$(printf '%.3f' "$X")
    Zs=$(printf '%.3f' "$Z")
    send "{\"T\":13,\"X\":$Xs,\"Z\":$Zs}"
    LAST_SEND=$now
  fi
done
