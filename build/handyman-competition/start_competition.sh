#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export HANDYMAN_COMPETITION_ROOT="$ROOT"
/usr/bin/python3 "$ROOT/resolve_package_paths.py"
if [ -e "$ROOT/PACKAGING_IN_PROGRESS" ]; then echo "比赛包尚未通过检查" >&2; exit 1; fi
if [ "$#" -eq 0 ]; then set -- --execute --from-session1-initial; fi
# Explicit help is read-only; only a real start retires the previous owner.
if [[ " ${*} " != *" --help "* && " ${*} " != *" -h "* ]]; then
  /usr/bin/python3 "$ROOT/competition_cleanup.py" --execute
  # Apply the complete validated task set only after old owners have exited.
  bash "$ROOT/configure_tasks.sh" --apply
fi
exec /usr/bin/python3 "$ROOT/competition_launcher.py" "$@"
