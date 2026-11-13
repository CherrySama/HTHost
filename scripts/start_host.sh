#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
python_bin="${HTHOST_PYTHON:-$HOME/.conda/envs/panthera/bin/python}"
chrome_bin="${HTHOST_CHROME:-google-chrome}"
state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/hthost"
chrome_profile="${XDG_CACHE_HOME:-$HOME/.cache}/hthost/chrome-swiftshader"
web_url="http://127.0.0.1:5173/"
bridge_url="http://127.0.0.1:5174/api/panthera/connection"

fail() {
  printf '%s\n' "$*" >&2
  exit 1
}

for command in curl flock node setsid "$chrome_bin"; do
  command -v "$command" >/dev/null || fail "找不到命令 $command"
done
[[ -x "$python_bin" ]] || fail "找不到 Panthera Python 环境：$python_bin"
[[ -x node_modules/.bin/vite ]] || fail "请先在项目目录运行 npm ci"

umask 077
mkdir -p -- "$state_dir" "$chrome_profile"
exec 9>"$state_dir/start.lock"
flock -n 9 || fail "HTHost 正在启动，请稍候"

fetch_local() {
  curl --noproxy '*' --silent --show-error --max-time 1 "$1" 2>/dev/null
}

is_frontend() {
  [[ "$1" == *'<title>Panthera Studio</title>'* ]]
}

is_bridge() {
  "$python_bin" -c 'import json, sys
try:
    data = json.load(sys.stdin)
    assert data["state"] in ("disconnected", "connecting", "connected", "homing", "calibrating", "error")
except (ValueError, TypeError, KeyError, AssertionError):
    sys.exit(1)' <<< "$1"
}

started_pids=()
cleanup() {
  local pid
  for pid in "${started_pids[@]}"; do
    kill -TERM "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

start_service() {
  local label="$1" url="$2" check="$3" log="$4"
  shift 4
  local response pid
  if response=$(fetch_local "$url"); then
    "$check" "$response" || fail "$url 已被其他服务占用"
    printf '复用%s\n' "$label"
    return
  fi

  setsid "$@" </dev/null >"$log" 2>&1 9>&- &
  pid=$!
  started_pids+=("$pid")
  for ((attempt = 0; attempt < 60; attempt++)); do
    kill -0 "$pid" 2>/dev/null || fail "$label 启动失败，日志在 $log"
    if response=$(fetch_local "$url") && "$check" "$response"; then
      printf '%s已启动（PID %s）\n' "$label" "$pid"
      return
    fi
    sleep 0.25
  done
  fail "$label 启动超时，日志在 $log"
}

start_service "真机桥接" "$bridge_url" is_bridge "$state_dir/bridge.log" \
  "$python_bin" -u scripts/panthera_bridge.py
start_service "网页前端" "$web_url" is_frontend "$state_dir/frontend.log" \
  node_modules/.bin/vite --host 127.0.0.1 --port 5173 --strictPort

setsid "$chrome_bin" --user-data-dir="$chrome_profile" --no-first-run \
  --no-default-browser-check --use-gl=angle --use-angle=swiftshader \
  --enable-unsafe-swiftshader --new-window "$web_url" \
  </dev/null >>"$state_dir/chrome.log" 2>&1 9>&- &
printf 'Panthera Studio %s\n日志目录 %s\n' "$web_url" "$state_dir"
trap - EXIT
