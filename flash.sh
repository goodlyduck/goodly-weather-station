#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$ROOT_DIR/goodly_weather_station"
FQBN="${FQBN:-arduino:esp32:nano_nora}"
PORT="${1:-${PORT:-}}"

# Detect OS
if [[ "$OSTYPE" == "msys" || "$OSTYPE" == "win32" || "$OSTYPE" == "cygwin" ]]; then
  IS_WINDOWS=true
else
  IS_WINDOWS=false
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
if [[ -z "$PORT" ]]; then
  if [[ "$IS_WINDOWS" == true ]]; then
    # On Windows, parse arduino-cli board list with exact COM matching.
    BOARD_LIST="$($ARDUINO_CLI board list 2>/dev/null || true)"
    PORT="$(printf '%s\n' "$BOARD_LIST" | awk '/arduino:esp32:nano_nora/ {for (i=1; i<=NF; i++) if ($i ~ /^COM[0-9]+$/) {print $i; exit}}')"
    if [[ -z "$PORT" ]]; then
      PORT="$(printf '%s\n' "$BOARD_LIST" | awk 'toupper($1) ~ /^COM[0-9]+$/ {print toupper($1); exit}')"
    fi
  else
    # On Unix-like systems, use arduino-cli to detect
    PORT="$("$ARDUINO_CLI" board list | awk '/arduino:esp32:nano_nora/ {print $1; exit}' || true)"
  fi
fi

# If still no port, use arduino-cli to detect it
if [[ -z "$PORT" ]]; then
  PORT="$("$ARDUINO_CLI" board list | awk '/arduino:esp32:nano_nora/ {print $1; exit}' || true)"
fi

if [[ -z "$PORT" ]]; then
  echo "Error: could not detect Nano ESP32 serial port." >&2
  echo "Connected boards:" >&2
  "$ARDUINO_CLI" board list >&2
  echo >&2
  if [[ "$IS_WINDOWS" == true ]]; then
    echo "Usage: ./flash.sh COM3" >&2
  else
    echo "Usage: ./flash.sh /dev/cu.usbmodemXXXX" >&2
  fi
  exit 1
fi

echo "Using CLI: $ARDUINO_CLI"
echo "Using board: $FQBN"
echo "Using port : $PORT"
echo

echo "Compiling..."
"$ARDUINO_CLI" compile --fqbn "$FQBN" "$SKETCH_DIR"

echo
echo "Uploading..."
"$ARDUINO_CLI" upload --fqbn "$FQBN" --port "$PORT" "$SKETCH_DIR"

echo
echo "Done."
