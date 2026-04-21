#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CASES_FILE="${CASES_FILE:-$ROOT_DIR/tests/reverse-cases.tsv}"

LEGACY_IPERF_BIN="${LEGACY_IPERF_BIN:-$ROOT_DIR/src/iperf}"
IPERF3_BIN="${IPERF3_BIN:-$(command -v iperf3 || true)}"
RUN_LEGACY="${RUN_LEGACY:-1}"
RUN_IPERF3="${RUN_IPERF3:-1}"
FAIL_ON_MISSING_BIN="${FAIL_ON_MISSING_BIN:-0}"
KEEP_TMP="${KEEP_TMP:-0}"
HOST="${HOST:-127.0.0.1}"
PORT_BASE="${PORT_BASE:-55001}"
CLIENT_TIMEOUT_SEC="${CLIENT_TIMEOUT_SEC:-20}"
SERVER_START_DELAY_SEC="${SERVER_START_DELAY_SEC:-1}"

if command -v timeout >/dev/null 2>&1; then
  TIMEOUT_BIN="timeout"
elif command -v gtimeout >/dev/null 2>&1; then
  TIMEOUT_BIN="gtimeout"
else
  TIMEOUT_BIN=""
fi

if [[ ! -f "$CASES_FILE" ]]; then
  echo "Cases file not found: $CASES_FILE" >&2
  exit 1
fi

TMPDIR_RUN="$(mktemp -d "${TMPDIR:-/tmp}/iperf-reverse-matrix.XXXXXX")"
if [[ "$KEEP_TMP" != "1" ]]; then
  trap 'rm -rf "$TMPDIR_RUN"' EXIT
fi

failures=0
case_count=0
port="$PORT_BASE"

cleanup_server() {
  local pid="$1"
  if kill -0 "$pid" >/dev/null 2>&1; then
    kill "$pid" >/dev/null 2>&1 || true
    wait "$pid" >/dev/null 2>&1 || true
  fi
}

run_with_timeout() {
  local timeout_sec="$1"
  shift

  if [[ -n "$TIMEOUT_BIN" ]]; then
    "$TIMEOUT_BIN" "$timeout_sec" "$@"
    return $?
  fi

  if command -v perl >/dev/null 2>&1; then
    perl -e 'my $t=shift @ARGV; alarm $t; exec @ARGV;' "$timeout_sec" "$@"
    return $?
  fi

  "$@"
  return $?
}

run_case() {
  local suite="$1"
  local case_name="$2"
  local server_args="$3"
  local client_args="$4"
  local expect_exit="$5"
  local client_expect_regex="$6"
  local bin=""

  if [[ "$suite" == "legacy" ]]; then
    if [[ "$RUN_LEGACY" != "1" ]]; then
      return 0
    fi
    bin="$LEGACY_IPERF_BIN"
  elif [[ "$suite" == "iperf3" ]]; then
    if [[ "$RUN_IPERF3" != "1" ]]; then
      return 0
    fi
    bin="$IPERF3_BIN"
  else
    echo "Unknown suite '$suite' in $CASES_FILE" >&2
    failures=$((failures + 1))
    return 0
  fi

  if [[ -z "$bin" || ! -x "$bin" ]]; then
    local hint=""
    if [[ "$suite" == "iperf3" ]]; then
      hint="Install iperf3 first (macOS: 'brew install iperf3', Ubuntu/Debian: 'sudo apt-get install -y iperf3')."
    elif [[ "$suite" == "legacy" ]]; then
      hint="Build the legacy binary first with './configure && make' or set LEGACY_IPERF_BIN."
    fi
    if [[ "$FAIL_ON_MISSING_BIN" == "1" ]]; then
      echo "FAIL [$suite/$case_name] missing executable: $bin"
      [[ -n "$hint" ]] && echo "  hint: $hint"
      failures=$((failures + 1))
    else
      echo "SKIP [$suite/$case_name] missing executable: $bin"
      [[ -n "$hint" ]] && echo "  hint: $hint"
    fi
    return 0
  fi

  case_count=$((case_count + 1))
  local prefix="$TMPDIR_RUN/${suite}_${case_name}"
  local server_log="${prefix}.server.log"
  local client_log="${prefix}.client.log"

  local server_cmd=("$bin" -s -p "$port")
  if [[ -n "$server_args" ]]; then
    # shellcheck disable=SC2206
    local sa=( $server_args )
    server_cmd+=("${sa[@]}")
  fi

  local client_cmd=("$bin" -c "$HOST" -p "$port" -t 1)
  if [[ "$suite" == "iperf3" ]]; then
    client_cmd+=(-J)
  fi
  if [[ -n "$client_args" ]]; then
    # shellcheck disable=SC2206
    local ca=( $client_args )
    client_cmd+=("${ca[@]}")
  fi

  echo "RUN  [$suite/$case_name] port=$port"
  "${server_cmd[@]}" >"$server_log" 2>&1 &
  local server_pid=$!
  sleep "$SERVER_START_DELAY_SEC"

  set +e
  run_with_timeout "$CLIENT_TIMEOUT_SEC" "${client_cmd[@]}" >"$client_log" 2>&1
  local rc=$?
  set -e

  cleanup_server "$server_pid"

  local ok=1
  if [[ "$rc" -ne "$expect_exit" ]]; then
    ok=0
    echo "FAIL [$suite/$case_name] expected exit $expect_exit, got $rc"
  fi

  if [[ -n "$client_expect_regex" ]]; then
    if ! grep -Eq "$client_expect_regex" "$client_log"; then
      ok=0
      echo "FAIL [$suite/$case_name] missing regex: $client_expect_regex"
    fi
  fi

  if [[ "$ok" -eq 1 ]]; then
    echo "PASS [$suite/$case_name]"
  else
    failures=$((failures + 1))
    echo "  client log: $client_log"
    echo "  server log: $server_log"
  fi

  port=$((port + 1))
}

while IFS='|' read -r suite case_name server_args client_args expect_exit client_expect_regex; do
  [[ -z "$suite" || "${suite:0:1}" == "#" ]] && continue
  run_case "$suite" "$case_name" "$server_args" "$client_args" "$expect_exit" "$client_expect_regex"
done <"$CASES_FILE"

echo
echo "Executed cases: $case_count"
echo "Failures: $failures"
if [[ "$KEEP_TMP" == "1" || "$failures" -ne 0 ]]; then
  echo "Logs: $TMPDIR_RUN"
fi

if [[ "$failures" -ne 0 ]]; then
  exit 1
fi
