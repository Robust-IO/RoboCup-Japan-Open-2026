#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "用法：./start_dynamic.sh [N:=数量] 或 [--N 数量]；默认 N=6，始终从 session1 开始。"
  echo "前 N 轮正常执行；后续轮收到 Instruction 后等待 5 秒发送 Give_up。N=0 表示全部放弃。"
  exit 0
fi
N=6
if (( $# )); then
  if (( $# == 1 )) && [[ "$1" == N:=* ]]; then
    N="${1#N:=}"
  elif (( $# == 2 )) && [[ "$1" == "--N" ]]; then
    N="$2"
  else
    echo "不再接受起始轮次；请使用 N:=数量 或 --N 数量（默认 6）。" >&2
    exit 2
  fi
fi
if [[ ! "$N" =~ ^(0|[1-9][0-9]*)$ ]] || (( ${#N} > 10 )) || (( N > 4294967295 )); then
  echo "N 必须为 0 至 4294967295 的整数。" >&2
  exit 2
fi
exec bash "$ROOT/start_competition.sh" --execute --from-session 1 --N "$N" --dynamic-instructions
