#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_BAUD="115200"

PORT="${1:-${PORT:-}}"
BAUD="${2:-${BAUD:-$DEFAULT_BAUD}}"

if [[ "${PORT:-}" == "-h" || "${PORT:-}" == "--help" ]]; then
  echo "Usage: ./connect.sh [PORT] [BAUD]"
  echo "Examples:"
  echo "  ./connect.sh"
  echo "  ./connect.sh /dev/cu.usbmodem4827E2FD33FC2"
  echo "  ./connect.sh /dev/cu.usbmodem4827E2FD33FC2 115200"
  exit 0
fi

if command -v arduino-cli >/dev/null 2>&1; then
  ARDUINO_CLI="$(command -v arduino-cli)"
elif [[ -x "/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli" ]]; then
  ARDUINO_CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
else
  echo "Error: arduino-cli not found." >&2
  echo "Install it with Homebrew or use Arduino IDE 2.x." >&2
  exit 1
fi

if [[ -z "${PORT}" ]]; then
  PORT="$(ls /dev/cu.usbmodem* 2>/dev/null | head -n 1 || true)"
fi

if [[ -z "${PORT}" ]]; then
  PORT="$("$ARDUINO_CLI" board list | awk '/arduino:esp32:nano_nora/ {print $1; exit}')"
fi

if [[ -z "${PORT}" ]]; then
  echo "Error: could not detect serial port." >&2
  "$ARDUINO_CLI" board list >&2 || true
  exit 1
fi

if [[ "$PORT" != /dev/* ]]; then
  echo "Error: detected '$PORT', which is not a serial device path." >&2
  echo "Pass a UART device explicitly, e.g. ./connect.sh /dev/cu.usbmodemXXXX" >&2
  exit 1
fi

echo "Using CLI : $ARDUINO_CLI"
echo "Using port: $PORT"
echo "Using baud: $BAUD"
echo

echo "Connecting... (Ctrl+C to quit)"
"$ARDUINO_CLI" monitor -p "$PORT" -c "baudrate=$BAUD"
