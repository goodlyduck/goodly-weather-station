#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SKETCH_DIR="$ROOT_DIR/goodly_weather_station"
FQBN="${FQBN:-arduino:esp32:nano_nora}"
PORT="${1:-${PORT:-}}"

if command -v arduino-cli >/dev/null 2>&1; then
  ARDUINO_CLI="$(command -v arduino-cli)"
elif [[ -x "/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli" ]]; then
  ARDUINO_CLI="/Applications/Arduino IDE.app/Contents/Resources/app/lib/backend/resources/arduino-cli"
else
  echo "Error: arduino-cli not found." >&2
  echo "Install it with Homebrew or use Arduino IDE 2.x." >&2
  exit 1
fi

if [[ -z "$PORT" ]]; then
  PORT="$("$ARDUINO_CLI" board list | awk '/arduino:esp32:nano_nora/ {print $1; exit}')"
fi

if [[ -z "$PORT" ]]; then
  echo "Error: could not detect Nano ESP32 serial port." >&2
  echo "Connected boards:" >&2
  "$ARDUINO_CLI" board list >&2
  echo >&2
  echo "Usage: ./flash.sh /dev/cu.usbmodemXXXX" >&2
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
