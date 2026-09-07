#!/usr/bin/env bash
set -euo pipefail

# Safe by default: QARM_HARDWARE must be explicitly set on the board. The
# service itself will still reject enable/gravity/MOVEJ until a real controller
# adapter is installed.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
venv_python="$repo_root/.venv/bin/python"
if [[ ! -x "$venv_python" ]]; then
  printf 'workspace virtual environment not found: %s\n' "$venv_python" >&2
  exit 1
fi
exec "$venv_python" platform/server/qarm_control_server.py "$@"
