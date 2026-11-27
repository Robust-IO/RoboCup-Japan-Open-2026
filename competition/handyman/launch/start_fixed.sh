#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ "${1:-}" == "--help" || "${1:-}" == "-h" ]]; then
  echo "用法：./start_fixed.sh [1-6]；默认从 session1 开始，继续运行到 session6。"
  echo "先运行本命令，再让 Unity 从对应 session 的初始位置开始。q 回车或 Ctrl+C 退出。"
  exit 0
fi
if (( $# > 1 )) || [[ ! "${1:-1}" =~ ^[1-6]$ ]]; then
  echo "起始 session 必须为 1 至 6，且只能传入一个编号。" >&2
  exit 2
fi
exec bash "$ROOT/start_competition.sh" --execute --from-session "${1:-1}" 
