#!/usr/bin/env bash
set -euo pipefail

MQTT_HOST="${MQTT_HOST:-mqtt.home.arpa}"
MQTT_PORT="${MQTT_PORT:-1883}"
MQTT_USER="${MQTT_USER:-ha}"
MQTT_QOS="${MQTT_QOS:-1}"
TOPIC_ROOT="${TOPIC_ROOT:-voda/septik}"
MQTT_PASS_FILE="${MQTT_PASS_FILE:-$HOME/.zalevaci-nadrz/mqtt_password}"
MQTT_PASS="${MQTT_PASS:-}"

CMD_TOPICS=(
  "cmd/reboot"
  "cmd/webapp/start"
  "cmd/webapp/stop"
  "cmd/debug/start"
  "cmd/debug/stop"
  "cmd/log/level"
  "cmd/ota/start"
  "cmd/ota/confirm"
)

ensure_dependencies() {
  if ! command -v mosquitto_pub >/dev/null 2>&1; then
    echo "Chyba: 'mosquitto_pub' neni nainstalovan." >&2
    exit 1
  fi
}

load_or_ask_password() {
  if [[ -n "$MQTT_PASS" ]]; then
    return
  fi

  local pass_dir
  pass_dir="$(dirname "$MQTT_PASS_FILE")"
  mkdir -p "$pass_dir"

  if [[ -s "$MQTT_PASS_FILE" ]]; then
    MQTT_PASS="$(<"$MQTT_PASS_FILE")"
    return
  fi

  read -r -s -p "Zadej MQTT heslo pro uzivatele '$MQTT_USER': " MQTT_PASS
  echo

  if [[ -z "$MQTT_PASS" ]]; then
    echo "Heslo nesmi byt prazdne." >&2
    exit 1
  fi

  printf '%s' "$MQTT_PASS" >"$MQTT_PASS_FILE"
  chmod 600 "$MQTT_PASS_FILE"
  echo "Heslo ulozeno do: $MQTT_PASS_FILE"
}

clear_retained_topic() {
  local topic="$1"
  echo "Cistim retained: $topic"
  mosquitto_pub \
    -h "$MQTT_HOST" \
    -p "$MQTT_PORT" \
    -u "$MQTT_USER" \
    -P "$MQTT_PASS" \
    -q "$MQTT_QOS" \
    -r \
    -n \
    -t "$topic"
}

main() {
  ensure_dependencies
  load_or_ask_password

  echo "Broker: $MQTT_HOST:$MQTT_PORT"
  echo "User: $MQTT_USER"
  echo "Topic root: $TOPIC_ROOT"

  for suffix in "${CMD_TOPICS[@]}"; do
    clear_retained_topic "$TOPIC_ROOT/$suffix"
  done

  echo "Hotovo: retained command topiky byly vycisteny."
}

main "$@"
