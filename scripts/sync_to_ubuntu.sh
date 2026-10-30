#!/usr/bin/env bash
# Safely mirror this source workspace to the Ubuntu robot over SSH.
# Default target: zhouweihua@192.168.64.2:~/ros2
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
REMOTE_USER="${ROS2_REMOTE_USER:-zhouweihua}"
REMOTE_HOST="${ROS2_REMOTE_HOST:-192.168.64.2}"
REMOTE_DIR="${ROS2_REMOTE_DIR:-~/ros2}"
REMOTE="${REMOTE_USER}@${REMOTE_HOST}"

usage() {
  cat <<'EOF'
Usage: ./scripts/sync_to_ubuntu.sh [--delete]

Synchronizes project sources to the Ubuntu robot. By default, it never removes
files that exist only on the robot. Use --delete only after confirming the
remote directory is exclusively managed by this project.

Optional environment overrides:
  ROS2_REMOTE_USER, ROS2_REMOTE_HOST, ROS2_REMOTE_DIR
EOF
}

DELETE_ARGS=()
case "${1:-}" in
  '') ;;
  --delete) DELETE_ARGS=(--delete) ;;
  -h|--help) usage; exit 0 ;;
  *) usage >&2; exit 2 ;;
esac

echo "Checking SSH connection to ${REMOTE}..."
ssh -o ConnectTimeout=10 "${REMOTE}" "mkdir -p ${REMOTE_DIR}"

echo "Syncing ${PROJECT_ROOT} to ${REMOTE}:${REMOTE_DIR}/"
rsync -avz --human-readable \
  --exclude='.git/' \
  --exclude='build/' \
  --exclude='install/' \
  --exclude='log/' \
  --exclude='.venv-vision/' \
  --exclude='.venv-opencv/' \
  --exclude='.pio/' \
  --exclude='.pytest_cache/' \
  --exclude='__pycache__/' \
  --exclude='*.pyc' \
  "${DELETE_ARGS[@]}" \
  "${PROJECT_ROOT}/" "${REMOTE}:${REMOTE_DIR}/"

echo "Sync complete. On Ubuntu, build with:"
echo "  cd ${REMOTE_DIR} && source /opt/ros/\$ROS_DISTRO/setup.bash && colcon build --symlink-install"
