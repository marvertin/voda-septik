#!/usr/bin/env bash
set -euo pipefail

MQTT_HOST="${MQTT_HOST:-mqtt.home.arpa}"
MQTT_PORT="${MQTT_PORT:-1883}"
MQTT_USER="${MQTT_USER:-ha}"
MQTT_QOS="${MQTT_QOS:-1}"
TOPIC_ROOT="${TOPIC_ROOT:-voda/septik}"
MQTT_PASS_FILE="${MQTT_PASS_FILE:-$HOME/.zalevaci-nadrz/mqtt_password}"
HTTP_PORT="${OTA_HTTP_PORT:-8000}"
BIN_PATH_DEFAULT="${OTA_BIN_PATH:-build/zalevaci-nadrz.bin}"
OTA_WAIT_TIMEOUT_SEC="${OTA_WAIT_TIMEOUT_SEC:-300}"

SERVER_PID=""

cleanup() {
  if [[ -n "$SERVER_PID" ]]; then
    kill "$SERVER_PID" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

require_cmd() {
  local cmd="$1"
  if ! command -v "$cmd" >/dev/null 2>&1; then
    echo "Chyba: chybi prikaz '$cmd'." >&2
    exit 1
  fi
}

wait_for_ota_boot() {
  local timeout_sec="$1"
  local line topic payload
  local started_at=$SECONDS

  echo
  echo "Sleduji OTA eventy (timeout ${timeout_sec}s)..."

  coproc OTA_SUB {
    mosquitto_sub \
      -h "$MQTT_HOST" \
      -p "$MQTT_PORT" \
      -u "$MQTT_USER" \
      -P "$MQTT_PASS" \
      -v \
      -t "$TOPIC_ROOT/system/ota/#" \
      -t "$TOPIC_ROOT/system/boot_mode" \
      -t "$TOPIC_ROOT/diag/fw_version"
  }

  while true; do
    if IFS= read -r -t 1 line <&"${OTA_SUB[0]}"; then
      echo "[MQTT] $line"
      topic="${line%% *}"
      payload="${line#* }"

      if [[ "$topic" == "$TOPIC_ROOT/system/boot_mode" && "$payload" == "ota" ]]; then
        echo "Detekovan boot_mode=ota (novy OTA firmware nabootovan)."
        kill "$OTA_SUB_PID" >/dev/null 2>&1 || true
        wait "$OTA_SUB_PID" 2>/dev/null || true
        return 0
      fi
    fi

    if (( SECONDS - started_at >= timeout_sec )); then
      echo "Timeout: neobjevil se system/boot_mode=ota do ${timeout_sec}s." >&2
      kill "$OTA_SUB_PID" >/dev/null 2>&1 || true
      wait "$OTA_SUB_PID" 2>/dev/null || true
      return 1
    fi
  done
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

  if [[ -z "$MQTT_PASS" ]]; then
    echo "Heslo nesmi byt prazdne." >&2
    exit 1
  fi

  printf '%s' "$MQTT_PASS" >"$MQTT_PASS_FILE"
  chmod 600 "$MQTT_PASS_FILE"
  echo "Heslo ulozeno do: $MQTT_PASS_FILE"
}

default_local_ip() {
  local ip
  ip="$(ipconfig getifaddr en0 2>/dev/null || true)"
  if [[ -z "$ip" ]]; then
    ip="$(ipconfig getifaddr en1 2>/dev/null || true)"
  fi
  echo "$ip"
}

pub_cmd() {
  local topic="$1"
  local payload="$2"

  echo "-> mosquitto_pub topic=$topic payload=$payload"
  mosquitto_pub -d \
    -h "$MQTT_HOST" \
    -p "$MQTT_PORT" \
    -u "$MQTT_USER" \
    -P "$MQTT_PASS" \
    -q "$MQTT_QOS" \
    -t "$topic" \
    -m "$payload"
}

start_http_server() {
  local directory="$1"
  local port="$2"

  echo "Startuji HTTP server: $directory na portu $port"
  python3 -m http.server "$port" --bind 0.0.0.0 --directory "$directory" >/tmp/ota_http_server.log 2>&1 &
  SERVER_PID="$!"
  sleep 1

  if ! kill -0 "$SERVER_PID" >/dev/null 2>&1; then
    echo "HTTP server se nepodarilo spustit. Log: /tmp/ota_http_server.log" >&2
    exit 1
  fi
}

stop_http_server() {
  if [[ -n "$SERVER_PID" ]]; then
    echo "Zastavuji HTTP server (PID $SERVER_PID)..."
    kill "$SERVER_PID" >/dev/null 2>&1 || true
    pkill -TERM -P "$SERVER_PID" >/dev/null 2>&1 || true

    sleep 0.5
    if kill -0 "$SERVER_PID" >/dev/null 2>&1; then
      echo "HTTP server stale bezi, posilam KILL..."
      kill -9 "$SERVER_PID" >/dev/null 2>&1 || true
      pkill -KILL -P "$SERVER_PID" >/dev/null 2>&1 || true
    fi

    wait "$SERVER_PID" 2>/dev/null || true
    SERVER_PID=""
  fi
}

main() {
  require_cmd mosquitto_pub
  require_cmd mosquitto_sub
  require_cmd python3

  load_or_ask_password

  local bin_path
  read -r -p "Cesta k firmware bin [${BIN_PATH_DEFAULT}]: " bin_path
  bin_path="${bin_path:-$BIN_PATH_DEFAULT}"

  if [[ ! -f "$bin_path" ]]; then
    echo "Soubor neexistuje: $bin_path" >&2
    exit 1
  fi

  local bin_abs
  bin_abs="$(cd "$(dirname "$bin_path")" && pwd)/$(basename "$bin_path")"

  local host_ip
  host_ip="$(default_local_ip)"
  read -r -p "IP adresa tohoto stroje pro ESP [${host_ip}]: " input_ip
  host_ip="${input_ip:-$host_ip}"

  if [[ -z "$host_ip" ]]; then
    echo "IP adresa je povinna." >&2
    exit 1
  fi

  local http_port
  read -r -p "HTTP port [${HTTP_PORT}]: " input_port
  http_port="${input_port:-$HTTP_PORT}"

  start_http_server "$(dirname "$bin_abs")" "$http_port"

  local firmware_url
  firmware_url="http://${host_ip}:${http_port}/$(basename "$bin_abs")"

  echo
  echo "Firmware URL pro OTA: $firmware_url"
  pub_cmd "$TOPIC_ROOT/cmd/ota/start" "$firmware_url"

  if ! wait_for_ota_boot "$OTA_WAIT_TIMEOUT_SEC"; then
    stop_http_server
    exit 1
  fi

  echo
  echo "OTA reboot byl detekovan."
  printf "Je nove FW v poradku a chces ho POTVRDIT? [y/N]: "
  if [[ -r /dev/tty ]]; then
    read -r confirm </dev/tty
  else
    read -r confirm
  fi

  case "${confirm}" in
    y|Y|yes|YES)
      pub_cmd "$TOPIC_ROOT/cmd/ota/confirm" "1"
      echo "Firmware potvrzen pres MQTT."
      ;;
    *)
      echo "Firmware NEPOTVRZEN."
      ;;
  esac

  echo "Pockam 0.5 s a poslu reboot command..."
  sleep 0.5
  pub_cmd "$TOPIC_ROOT/cmd/reboot" "1"
  echo "Reboot command odeslan."

  stop_http_server
}

main "$@"
