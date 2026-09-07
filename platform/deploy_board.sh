#!/usr/bin/env bash
set -euo pipefail

# Build the browser and local control service in this checkout.
# Run this script from the development board workspace.
start_server=1
case "${1:-}" in
  "") ;;
  --no-start) start_server=0 ;;
  *)
    printf 'usage: %s [--no-start]\n' "$0" >&2
    exit 2
    ;;
esac

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
if [[ ! -x .venv/bin/python ]]; then
  printf '%s\n' 'missing .venv; create the workspace virtual environment first' >&2
  exit 1
fi
if [[ ! -d platform/node_modules ]]; then
  (cd platform && npm ci)
fi
(cd platform && npm run build)

if (( start_server )); then
  echo "Starting Qarm platform on http://127.0.0.1:8090 (hardware mode is explicit)."
  QARM_HARDWARE=1 QARM_PLATFORM_PORT=8090 \
    .venv/bin/python platform/server/qarm_control_server.py
else
  echo "Platform bundle built locally; start with QARM_HARDWARE=1 ./platform/run_server.sh"
fi
