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

# SLAM mode is self-contained: the OAK depth camera plus RTAB-Map, with no
# Spot, no Trimble, and no YOLO. Check only what that path needs, then stop
# before the Spot/Trimble/webcam checks that do not apply.
if [[ "${MODE}" == "slam" ]]; then
  if ros2 pkg prefix rtabmap_slam >/dev/null 2>&1; then
    printf 'OK:   rtabmap_slam is installed\n'
  else
    printf 'FAIL: rtabmap_slam not found; install ros-jazzy-rtabmap-ros\n'
    failures=$((failures + 1))
  fi
  if python3 -c 'import rclpy' >/dev/null 2>&1; then
    printf 'OK:   ROS Python (rclpy) import\n'
  else
    printf 'FAIL: rclpy is unavailable\n'
    failures=$((failures + 1))
  fi
  # The OAK driver may still be starting, so a missing topic is a warning,
  # not a failure: the operator can launch the driver and SLAM in any order.
  for topic in "${RGB_TOPIC:-/oak/rgb/image}" \
               "${DEPTH_TOPIC:-/oak/rgb/depth}" \
               "${DEPTH_CAMERA_INFO_TOPIC:-/oak/rgb/camera_info}" \
               "${DEPTH_ODOM_TOPIC:-/oak/odom}"; do
    if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
      printf 'OK:   OAK topic advertised: %s\n' "${topic}"
    else
      printf 'WARN: OAK topic not yet advertised: %s\n' "${topic}"
    fi
  done
  if ((failures > 0)); then
    printf '\nPreflight failed with %d problem(s).\n' "${failures}"
    exit 1
  fi
  printf '\nPreflight passed for slam mode.\n'
  exit 0
fi

# Mission mode expects a depth camera driver publishing over ROS rather
# than a local V4L webcam, so the /dev/video check does not apply. Demo
# mode runs no camera at all.
if [[ "${MODE}" != "mission" && "${MODE}" != "demo" ]]; then
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
# Demo mode invents the site but drives the real robot, so it needs the
# SDK exactly as the field mission does.
if [[ "${MODE}" == "mission" || "${MODE}" == "demo" ]]; then
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

# Demo mode: no model and no sensors to check, but the virtual site has
# to parse before the robot is standing in a field waiting for it.
if [[ "${MODE}" == "demo" ]]; then
  demo_site_path="${DEMO_SITE_PATH:-}"
  if [[ -z "${demo_site_path}" ]]; then
    demo_site_path="$(
      ros2 pkg prefix defect_detection 2>/dev/null
    )/share/defect_detection/config/demo_site.yaml"
  fi
  check_file 'virtual site' "${demo_site_path}"
  if [[ -f "${demo_site_path}" ]]; then
    if site_summary=$(python3 - "${demo_site_path}" <<'PY'
import sys

from defect_detection.simulation.virtual_site import load_site

site = load_site(sys.argv[1])
if not len(site.points):
    raise SystemExit(1)
print(
    f'"{site.name}": {len(site.points)} points, '
    f'{len(site.defects)} planted defects'
)
PY
    )
    then
      printf 'OK:   virtual site %s\n' "${site_summary}"
    else
      printf 'FAIL: virtual site failed to load\n'
      failures=$((failures + 1))
    fi
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
