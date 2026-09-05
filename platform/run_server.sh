#!/usr/bin/env bash
set -euo pipefail

# Safe by default: QARM_HARDWARE must be explicitly set to 1 on the controller
# computer. The service itself will still reject enable/gravity/MOVEJ until a
# real controller adapter is installed.
repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root"
exec python3 platform/server/qarm_control_server.py "$@"
