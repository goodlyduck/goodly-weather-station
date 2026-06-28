#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_BAUD="115200"

PORT="${1:-${PORT:-}}"
BAUD="${2:-${BAUD:-$DEFAULT_BAUD}}"

# Detect OS
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" || "$OSTYPE" == "cygwin" ]]; then
  IS_WINDOWS=true
else
  IS_WINDOWS=false
fi

if [[ "${PORT:-}" == "-h" || "${PORT:-}" == "--help" ]]; then
  echo "Usage: ./connect.sh [PORT] [BAUD]"
  echo "Examples:"
  if [[ "$IS_WINDOWS" == true ]]; then
    echo "  ./connect.sh"
    echo "  ./connect.sh COM3"
    echo "  ./connect.sh COM3 115200"
  else
    echo "  ./connect.sh"
    echo "  ./connect.sh /dev/cu.usbmodem4827E2FD33FC2"
    echo "  ./connect.sh /dev/cu.usbmodem4827E2FD33FC2 115200"
  fi
  exit 0
fi

# Find arduino-cli
if command -v arduino-cli >/dev/null 2>&1; then
  ARDUINO_CLI="$(command -v arduino-cli)"
elif [[ -x "/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli" ]]; then
  ARDUINO_CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
elif [[ -x "$HOME/AppData/Local/Arduino15/arduino-cli.exe" ]]; then
  ARDUINO_CLI="$HOME/AppData/Local/Arduino15/arduino-cli.exe"
else
  echo "Error: arduino-cli not found." >&2
  if [[ "$IS_WINDOWS" == true ]]; then
    echo "Install Arduino IDE 2.x or download arduino-cli from https://arduino.cc/en/software" >&2
  else
    echo "Install it with Homebrew or use Arduino IDE 2.x." >&2
  fi
  exit 1
fi

# Auto-detect port if not provided
if [[ -z "${PORT}" ]]; then
  if [[ "$IS_WINDOWS" == true ]]; then
    # On Windows, parse arduino-cli board list with exact COM matching.
    BOARD_LIST="$($ARDUINO_CLI board list 2>/dev/null || true)"
    PORT="$(printf '%s\n' "$BOARD_LIST" | awk '/arduino:esp32:nano_nora/ {for (i=1; i<=NF; i++) if ($i ~ /^COM[0-9]+$/) {print $i; exit}}')"
    if [[ -z "${PORT}" ]]; then
      PORT="$(printf '%s\n' "$BOARD_LIST" | awk 'toupper($1) ~ /^COM[0-9]+$/ {print toupper($1); exit}')"
    fi
  else
    # On Unix-like systems, look for macOS/Linux serial ports
    PORT="$(ls /dev/cu.usbmodem* 2>/dev/null | head -n 1 || true)"
    if [[ -z "${PORT}" ]]; then
      PORT="$(ls /dev/ttyUSB* 2>/dev/null | head -n 1 || true)"
    fi
  fi
fi

# If still no port, use arduino-cli to detect it
if [[ -z "${PORT}" ]]; then
  PORT="$("$ARDUINO_CLI" board list | awk '/arduino:esp32:nano_nora/ {print $1; exit}' || true)"
fi

if [[ -z "${PORT}" ]]; then
  echo "Error: could not detect serial port." >&2
  "$ARDUINO_CLI" board list >&2 || true
  exit 1
fi

# Validate port format
if [[ "$IS_WINDOWS" == true ]]; then
  if ! [[ "$PORT" =~ ^COM[0-9]+$ ]]; then
    echo "Error: detected '$PORT', which is not a valid COM port on Windows." >&2
    echo "Expected format: COM1, COM2, COM3, etc." >&2
    exit 1
  fi
else
  if ! [[ "$PORT" =~ ^/dev/ ]]; then
    echo "Error: detected '$PORT', which is not a serial device path." >&2
    echo "Pass a UART device explicitly, e.g. ./connect.sh /dev/cu.usbmodemXXXX" >&2
    exit 1
  fi
fi

echo "Using CLI : $ARDUINO_CLI"
echo "Using port: $PORT"
echo "Using baud: $BAUD"
echo

echo "Connecting... (Ctrl+C to quit)"
"$ARDUINO_CLI" monitor -p "$PORT" -c "baudrate=$BAUD"
