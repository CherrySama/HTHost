#!/usr/bin/env bash
set -euo pipefail

cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.."
project_dir=$(pwd -P)
state_dir="${XDG_STATE_HOME:-$HOME/.local/state}/hthost"
bridge_url="http://127.0.0.1:5174/api/panthera/connection"

fail() {
  printf '%s\n' "$*" >&2
  exit 1
}

for command in curl flock python3 ss; do
  command -v "$command" >/dev/null || fail "找不到命令 $command"
done

umask 077
mkdir -p -- "$state_dir"
exec 9>"$state_dir/start.lock"
flock -n 9 || fail "HTHost 正在启动或停止，请稍候"

listener_pid() {
  local listeners
  listeners=$(ss -Hltnp "sport = :$1") || fail "无法查询端口 $1"
  [[ -n "$listeners" ]] || return 0
  if [[ "$listeners" == *$'\n'* || ! "$listeners" =~ pid=([0-9]+), ]]; then
    fail "无法确认端口 $1 的进程身份"
  fi
  local pid="${BASH_REMATCH[1]}"
  [[ "${listeners#*pid=}" != *pid=* ]] || fail "端口 $1 有多个进程持有"
  printf '%s\n' "$pid"
}

verify_process() {
  local pid="$1" entry="$2" argument
  [[ "$(readlink -f "/proc/$pid/cwd")" == "$project_dir" ]] || return 1
  while IFS= read -r -d '' argument; do
    case "$argument" in
      "$entry"|"./$entry"|"$project_dir/$entry") return 0 ;;
    esac
  done < "/proc/$pid/cmdline"
  return 1
}

frontend=$(listener_pid 5173)
bridge=$(listener_pid 5174)
if [[ -n "$frontend" ]] && ! verify_process "$frontend" node_modules/.bin/vite; then
  fail "端口 5173 的进程不是此目录的 HTHost 前端"
fi
if [[ -n "$bridge" ]] && ! verify_process "$bridge" scripts/panthera_bridge.py; then
  fail "端口 5174 的进程不是此目录的 HTHost 桥接"
fi

bridge_request() {
  curl --noproxy '*' --silent --show-error --max-time 5 "$@" "$bridge_url"
}

json_field() {
  python3 -c 'import json, sys; print(json.load(sys.stdin).get(sys.argv[1]) or "")' "$1"
}

if [[ -n "$bridge" ]]; then
  status=$(bridge_request) || fail "无法读取真机状态，服务仍在运行"
  state=$(json_field state <<< "$status") || fail "无法解析真机状态，服务仍在运行"
  if [[ "$state" == "connected" ]]; then
    payload=$(python3 -c 'import json, sys; print(json.dumps({"session": json.load(sys.stdin)["session"]}))' <<< "$status") ||
      fail "无法确认真机会话，服务仍在运行"
    response=$(bridge_request -X DELETE -H 'Content-Type: application/json' --data "$payload") ||
      fail "无法确认回零请求，服务仍在运行"
    state=$(json_field state <<< "$response") || fail "无法确认回零请求，服务仍在运行"
    [[ "$state" == "homing" ]] ||
      fail "$(json_field message <<< "$response")。服务仍在运行。"
  fi

  if [[ "$state" == "homing" ]]; then
    printf '等待机械臂回零并断开…\n'
    deadline=$((SECONDS + 90))
    while [[ "$state" == "homing" && $SECONDS -lt $deadline ]]; do
      sleep 0.25
      status=$(bridge_request) || fail "回零期间无法读取反馈，服务仍在运行"
      state=$(json_field state <<< "$status") || fail "无法解析回零状态，服务仍在运行"
    done
    [[ "$state" == "disconnected" ]] ||
      fail "回零未确认。$(json_field message <<< "$status") 服务仍在运行。"
  elif [[ "$state" != "disconnected" && "$state" != "error" ]]; then
    fail "真机当前状态为 $state，请等待连接或校准完成后重试"
  fi
fi

stop_service() {
  local pid="$1" entry="$2" port="$3" label="$4" current
  if [[ -z "$pid" ]]; then
    printf '%s已停止\n' "$label"
    return
  fi
  current=$(listener_pid "$port")
  if [[ -n "$current" ]]; then
    [[ "$current" == "$pid" ]] && verify_process "$pid" "$entry" ||
      fail "$label 进程已变化，请重新运行脚本"
    kill -TERM "$pid" || fail "无法停止$label"
    for ((attempt = 0; attempt < 40; attempt++)); do
      [[ -n "$(listener_pid "$port")" ]] || break
      sleep 0.25
    done
    [[ -z "$(listener_pid "$port")" ]] || fail "$label 端口尚未释放，请查看服务日志"
  fi
  printf '%s已停止\n' "$label"
}

stop_service "$bridge" scripts/panthera_bridge.py 5174 "真机桥接"
stop_service "$frontend" node_modules/.bin/vite 5173 "网页前端"
