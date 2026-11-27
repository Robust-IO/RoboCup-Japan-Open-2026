#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if (( $# )); then echo "此文件固定从 session6 启动，无需参数。" >&2; exit 2; fi
exec bash "$ROOT/start_fixed.sh" 6
