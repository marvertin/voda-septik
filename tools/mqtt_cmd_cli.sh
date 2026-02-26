#!/usr/bin/env bash
set -euo pipefail

MQTT_HOST="${MQTT_HOST:-mqtt.home.arpa}"
MQTT_PORT="${MQTT_PORT:-1883}"
MQTT_USER="${MQTT_USER:-ha}"
MQTT_QOS="${MQTT_QOS:-1}"
TOPIC_ROOT="${TOPIC_ROOT:-voda/septik}"
MQTT_PASS_FILE="${MQTT_PASS_FILE:-$HOME/.zalevaci-nadrz/mqtt_password}"

ensure_dependencies() {
  if ! command -v mosquitto_pub >/dev/null 2>&1; then
    echo "Chyba: 'mosquitto_pub' neni nainstalovan." >&2
    exit 1
  fi
}

load_or_ask_password() {
  local pass_dir
  pass_dir="$(dirname "$MQTT_PASS_FILE")"
  mkdir -p "$pass_dir"

  if [[ -s "$MQTT_PASS_FILE" ]]; then
    MQTT_PASS="$(<"$MQTT_PASS_FILE")"
    return
  fi

  read -r -s -p "Zadej MQTT heslo pro uzivatele '$MQTT_USER': " MQTT_PASS
  echo

  if [[ -z "${MQTT_PASS}" ]]; then
    echo "Heslo nesmi byt prazdne." >&2
    exit 1
  fi

  printf '%s' "$MQTT_PASS" >"$MQTT_PASS_FILE"
  chmod 600 "$MQTT_PASS_FILE"
  echo "Heslo ulozeno do: $MQTT_PASS_FILE"
}

publish_cmd() {
  local topic="$1"
  local payload="${2:-1}"

  echo
  echo "Spoustim: mosquitto_pub -d -h $MQTT_HOST -p $MQTT_PORT -u $MQTT_USER -q $MQTT_QOS -t $topic -m $payload"
  mosquitto_pub -d \
    -h "$MQTT_HOST" \
    -p "$MQTT_PORT" \
    -u "$MQTT_USER" \
    -P "$MQTT_PASS" \
    -q "$MQTT_QOS" \
    -t "$topic" \
    -m "$payload"
}

print_header() {
  cat <<EOF

MQTT Command CLI
----------------
Host:       $MQTT_HOST
Port:       $MQTT_PORT
User:       $MQTT_USER
Topic root: $TOPIC_ROOT
Pass file:  $MQTT_PASS_FILE
EOF
}

menu() {
  cat <<'EOF'

Vyber akci:
  1) reboot
  2) webapp/start
  3) webapp/stop
  4) debug/start
  5) debug/stop
  6) log/level
  7) custom cmd topic
  8) konec
EOF
}

handle_choice() {
  local choice="$1"
  local value=""
  local topic=""

  case "$choice" in
    1)
      publish_cmd "$TOPIC_ROOT/cmd/reboot" "1"
      ;;
    2)
      publish_cmd "$TOPIC_ROOT/cmd/webapp/start" "1"
      ;;
    3)
      publish_cmd "$TOPIC_ROOT/cmd/webapp/stop" "1"
      ;;
    4)
      publish_cmd "$TOPIC_ROOT/cmd/debug/start" "1"
      ;;
    5)
      publish_cmd "$TOPIC_ROOT/cmd/debug/stop" "1"
      ;;
    6)
      read -r -p "Zadej tag (napr. mqtt_cmd nebo *): " topic
      [[ -n "$topic" ]] || { echo "Tag nesmi byt prazdny."; return; }
      read -r -p "Zadej uroven (NONE|ERROR|WARN|INFO|DEBUG|VERBOSE): " value
      [[ -n "$value" ]] || { echo "Uroven nesmi byt prazdna."; return; }
      publish_cmd "$TOPIC_ROOT/cmd/log/level" "$topic=$value"
      ;;
    7)
      read -r -p "Zadej cmd topic cast za '$TOPIC_ROOT/cmd/' (napr. webapp/start): " topic
      [[ -n "$topic" ]] || { echo "Topic nesmi byt prazdny."; return; }
      read -r -p "Zadej payload (default 1): " value
      value="${value:-1}"
      publish_cmd "$TOPIC_ROOT/cmd/$topic" "$value"
      ;;
    8)
      echo "Konec."
      exit 0
      ;;
    *)
      echo "Neplatna volba."
      ;;
  esac
}

main() {
  ensure_dependencies
  load_or_ask_password

  while true; do
    print_header
    menu
    read -r -p "Volba [1-8]: " choice
    handle_choice "$choice"
  done
}

main "$@"
