#!/usr/bin/env bash
set -euo pipefail

# Build the browser once on the development computer, copy the self-hosted
# bundle and server to the board, then optionally start it there.  No Node.js
# runtime is required on the board.
board_user="${QARM_BOARD_USER:-HwHiAiUser}"
board_host="${QARM_BOARD_HOST:-192.168.10.102}"
remote_root="${QARM_BOARD_ROOT:-/home/${board_user}/qarm-platform}"
start_server=1
[[ "${1:-}" == "--no-start" ]] && start_server=0

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$repo_root/platform"
npm ci
npm run build

ssh "${board_user}@${board_host}" "mkdir -p '${remote_root}/platform/server' '${remote_root}/platform/dist' '${remote_root}/config' '${remote_root}/description'"
rsync -az --delete dist/ "${board_user}@${board_host}:${remote_root}/platform/dist/"
rsync -az server/qarm_control_server.py "${board_user}@${board_host}:${remote_root}/platform/server/"
rsync -az run_server.sh "${board_user}@${board_host}:${remote_root}/platform/"
rsync -az "$repo_root/config/joint_map.json" "$repo_root/config/calibration_pose.json" "${board_user}@${board_host}:${remote_root}/config/"
rsync -az "$repo_root/description/qmini_arm.urdf" "${board_user}@${board_host}:${remote_root}/description/"

if (( start_server )); then
  echo "Starting Qarm platform on http://${board_host}:8090 (hardware mode is explicit)."
  ssh -t "${board_user}@${board_host}" "cd '${remote_root}' && QARM_HARDWARE=1 QARM_PLATFORM_PORT=8090 QARM_CONFIG='${remote_root}/config/joint_map.json' python3 platform/server/qarm_control_server.py"
else
  echo "Deployed to ${board_user}@${board_host}:${remote_root}; start with QARM_HARDWARE=1 ./platform/run_server.sh"
fi
