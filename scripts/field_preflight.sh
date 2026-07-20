#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-transport}"
WORKSPACE_ROOT="$(
  cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd
)"
CONFIG_FILE="${FIELD_CONFIG:-${WORKSPACE_ROOT}/config/field.env}"

if [[ -f "${CONFIG_FILE}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${CONFIG_FILE}"
  set +a
fi

failures=0

check_value() {
  local name="$1"
  if [[ -z "${!name:-}" ]]; then
    printf 'FAIL: %s is not set\n' "${name}"
    failures=$((failures + 1))
  else
    printf 'OK:   %s is set\n' "${name}"
  fi
}

check_file() {
  local label="$1"
  local path="$2"
  if [[ -f "${path}" ]]; then
    printf 'OK:   %s: %s\n' "${label}" "${path}"
  else
    printf 'FAIL: %s not found: %s\n' "${label}" "${path}"
    failures=$((failures + 1))
  fi
}

# Mission mode expects a depth camera driver publishing over ROS rather
# than a local V4L webcam, so the /dev/video check does not apply.
if [[ "${MODE}" != "mission" ]]; then
  camera_index="${CAMERA_INDEX:-0}"
  if [[ -e "/dev/video${camera_index}" ]]; then
    printf 'OK:   webcam device: /dev/video%s\n' "${camera_index}"
  else
    printf 'FAIL: webcam device missing: /dev/video%s\n' "${camera_index}"
    failures=$((failures + 1))
  fi
fi

if python3 -c 'import rclpy, laspy, lazrs' >/dev/null 2>&1; then
  printf 'OK:   ROS Python and LAS/LAZ imports\n'
else
  printf 'FAIL: ROS Python or LAS/LAZ import is unavailable\n'
  failures=$((failures + 1))
fi

trimble_scan_directory="${TRIMBLE_SCAN_DIRECTORY:-/tmp/trimble_scans}"
if [[ -d "${trimble_scan_directory}" ]]; then
  printf 'OK:   Trimble scan directory: %s\n' "${trimble_scan_directory}"
else
  printf 'FAIL: Trimble scan directory missing: %s\n' "${trimble_scan_directory}"
  failures=$((failures + 1))
fi

needs_spot_sdk=false
if [[ "${MODE}" == "mission" ]]; then
  needs_spot_sdk=true
fi
if [[ "${ROBOT_GOAL_BACKEND:-}" == "spot_sdk" ||
      "${SPOT_LOCALIZATION:-false}" == "true" ||
      "${EAP_LIDAR:-false}" == "true" ]]; then
  needs_spot_sdk=true
fi

if [[ "${needs_spot_sdk}" == "true" ]]; then
  if python3 -c 'import bosdyn.client' >/dev/null 2>&1; then
    printf 'OK:   Boston Dynamics SDK (bosdyn-client) import\n'
  else
    printf 'FAIL: bosdyn-client is unavailable; install requirements-field.txt\n'
    failures=$((failures + 1))
  fi
  check_value SPOT_IP
fi

# The EAP lidar is the backbone of the occupancy map, so a missing point
# cloud service means no frontier exploration at all.
if [[ "${MODE}" == "mission" || "${EAP_LIDAR:-false}" == "true" ]]; then
  if python3 -c 'from bosdyn.client.point_cloud import PointCloudClient' \
      >/dev/null 2>&1; then
    printf 'OK:   Spot EAP point cloud client is importable\n'
  else
    printf 'FAIL: bosdyn PointCloudClient is unavailable; the EAP lidar cannot be read\n'
    failures=$((failures + 1))
  fi
  if [[ -n "${SPOT_IP:-}" ]] && command -v ping >/dev/null 2>&1; then
    if ping -c 1 -W 2 "${SPOT_IP}" >/dev/null 2>&1; then
      printf 'OK:   Spot is reachable at %s\n' "${SPOT_IP}"
    else
      printf 'FAIL: Spot did not answer at %s\n' "${SPOT_IP}"
      failures=$((failures + 1))
    fi
  fi
fi

if [[ "${MODE}" == "mission" || "${MODE}" == "full" ]]; then
  check_file 'YOLO model' "${MODEL_PATH:-}"
  check_file 'dataset configuration' "${DATASET_PATH:-}"
  if [[ -f "${DATASET_PATH:-}" ]]; then
    if python3 - "${DATASET_PATH}" <<'PY'
import sys
import yaml

with open(sys.argv[1], encoding='utf-8') as stream:
    dataset = yaml.safe_load(stream) or {}
if not dataset.get('names'):
    raise SystemExit(1)
PY
    then
      printf 'OK:   dataset contains class names\n'
    else
      printf 'FAIL: dataset names list is empty\n'
      failures=$((failures + 1))
    fi
  fi
fi

if ((failures > 0)); then
  printf '\nPreflight failed with %d problem(s).\n' "${failures}"
  exit 1
fi

printf '\nPreflight passed for %s mode.\n' "${MODE}"
