#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 [transport|full] [--sim gazebo|synthetic]"
  echo
  echo "  transport|full   Deployment mode on the real robot (default: transport)."
  echo "  --sim gazebo     Run the same pipeline against a Gazebo simulation."
  echo "  --sim synthetic  Run against the pure-Python synthetic world (no GPU)."
  echo
  echo "Examples:"
  echo "  $0 full                  # real robot, full pipeline"
  echo "  $0 full --sim gazebo     # identical pipeline, simulated in Gazebo"
  echo "  $0 --sim synthetic       # quick demo without Gazebo"
  exit 2
}

MODE="transport"
SIM="none"
while [[ $# -gt 0 ]]; do
  case "$1" in
    transport|full) MODE="$1"; shift ;;
    --sim) [[ $# -ge 2 ]] || usage; SIM="$2"; shift 2 ;;
    --sim=*) SIM="${1#--sim=}"; shift ;;
    -h|--help) usage ;;
    *) echo "Unknown argument: $1"; usage ;;
  esac
done
if [[ "${SIM}" != "none" && "${SIM}" != "gazebo" && "${SIM}" != "synthetic" ]]; then
  echo "Invalid --sim value: ${SIM}"
  usage
fi

WORKSPACE_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export WORKSPACE_ROOT
export FIELD_CONFIG="${FIELD_CONFIG:-${WORKSPACE_ROOT}/config/field.env}"

# Simulation runs without hardware and without credentials, so field.env
# is optional there; on the real robot it is required.
if [[ -f "${FIELD_CONFIG}" ]]; then
  set -a
  # shellcheck disable=SC1090
  source "${FIELD_CONFIG}"
  set +a
elif [[ "${SIM}" == "none" ]]; then
  echo "Missing ${FIELD_CONFIG}"
  echo "Copy config/field.env.example to config/field.env and edit it."
  exit 1
fi

source /opt/ros/jazzy/setup.bash
if [[ ! -f "${WORKSPACE_ROOT}/install/setup.bash" ]]; then
  echo "Workspace is not built. Run: colcon build --symlink-install"
  exit 1
fi
# shellcheck disable=SC1091
source "${WORKSPACE_ROOT}/install/setup.bash"

export ROS_LOG_DIR="${ROS_LOG_DIR:-${WORKSPACE_ROOT}/log/field}"
export YOLO_CONFIG_DIR="${YOLO_CONFIG_DIR:-${WORKSPACE_ROOT}/.runtime/ultralytics}"
mkdir -p "${ROS_LOG_DIR}" "${YOLO_CONFIG_DIR}"

cd "${WORKSPACE_ROOT}"

if [[ "${SIM}" != "none" ]]; then
  # Fresh sim state: stale anchors or defect stores would offset the twin.
  DEMO_ROOT=/tmp/synthetic_demo
  rm -rf "${DEMO_ROOT}"
  mkdir -p "${DEMO_ROOT}/trimble_scans"

  if [[ "${SIM}" == "gazebo" ]]; then
    if ! command -v gz >/dev/null 2>&1 \
        || ! ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
      echo "Gazebo (gz-sim) or ros_gz is not installed. Install with:"
      echo "  sudo apt install ros-jazzy-ros-gz"
      exit 1
    fi
    echo "Simulation mode: Gazebo (world + robot + camera simulated)"
    exec ros2 launch defect_detection gazebo_sim.launch.xml \
      rviz:="${ENABLE_RVIZ:-true}" \
      demo_root:="${DEMO_ROOT}"
  fi

  echo "Simulation mode: synthetic (pure Python, no Gazebo required)"
  exec ros2 launch defect_detection synthetic_demo.launch.xml \
    rviz:="${ENABLE_RVIZ:-true}" \
    demo_root:="${DEMO_ROOT}"
fi

"${WORKSPACE_ROOT}/scripts/field_preflight.sh" "${MODE}"

detector=false
fusion="${FUSION:-false}"
if [[ "${MODE}" == "full" ]]; then
  detector=true
  if [[ "${OAK_DEPTH_NAVIGATION:-false}" != "true" ]]; then
    fusion=true
  fi
fi

exec ros2 launch pointcloud_bridge full_pipeline.launch.xml \
  lidar_input_topic:="${LIDAR_INPUT_TOPIC:-/lidar/raw}" \
  pointcloud_topic:="${POINTCLOUD_TOPIC:-/lidar/points}" \
  publish_camera:="${PUBLISH_CAMERA:-false}" \
  camera_index:="${CAMERA_INDEX:-0}" \
  camera_frame:="${CAMERA_FRAME:-camera_optical_frame}" \
  lidar_frame:="${LIDAR_FRAME:-lidar}" \
  dataset_path:="${DATASET_PATH:-}" \
  model_path:="${MODEL_PATH:-}" \
  image_topic:="${IMAGE_TOPIC:-/ros2_image}" \
  detections_2d_topic:="${DETECTIONS_2D_TOPIC:-/detections_2d}" \
  detections_3d_topic:="${DETECTIONS_3D_TOPIC:-/detections_3d}" \
  calibration_path:="${CALIBRATION_PATH:-}" \
  detector:="${detector}" \
  fusion:="${fusion}" \
  oak_depth_navigation:="${OAK_DEPTH_NAVIGATION:-false}" \
  oak_depth_topic:="${OAK_DEPTH_TOPIC:-/oak/rgb/depth}" \
  oak_camera_info_topic:="${OAK_CAMERA_INFO_TOPIC:-/oak/rgb/camera_info}" \
  oak_depth_minimum_confidence:="${OAK_DEPTH_MINIMUM_CONFIDENCE:-0.50}" \
  oak_depth_bbox_padding_px:="${OAK_DEPTH_BBOX_PADDING_PX:-4}" \
  oak_depth_default_bbox_size_m:="${OAK_DEPTH_DEFAULT_BBOX_SIZE_M:-0.20}" \
  oak_localization:="${OAK_LOCALIZATION:-false}" \
  oak_odom_topic:="${OAK_ODOM_TOPIC:-/oak/odom}" \
  oak_odom_frame:="${OAK_ODOM_FRAME:-oak_odom}" \
  oak_odom_base_frame:="${OAK_ODOM_BASE_FRAME:-base_link}" \
  oak_odom_use_message_frame_ids:="${OAK_ODOM_USE_MESSAGE_FRAME_IDS:-true}" \
  visualization:=true \
  rviz:="${ENABLE_RVIZ:-true}" \
  autonomous_navigation:="${AUTONOMOUS_NAVIGATION:-false}" \
  autonomous_navigation_enabled:="${AUTONOMOUS_NAVIGATION_ENABLED:-false}" \
  navigation_base_frame:="${NAVIGATION_BASE_FRAME:-body}" \
  navigation_priority_config:="${NAVIGATION_PRIORITY_CONFIG:-}" \
  trimble_scan_watcher:="${TRIMBLE_SCAN_WATCHER:-true}" \
  trimble_scan_directory:="${TRIMBLE_SCAN_DIRECTORY:-/tmp/trimble_scans}" \
  trimble_scan_topic:="${TRIMBLE_SCAN_TOPIC:-/trimble/x7/scan_points}" \
  trimble_scan_frame:="${TRIMBLE_SCAN_FRAME:-map}" \
  trimble_windows_bridge:="${TRIMBLE_WINDOWS_BRIDGE:-false}" \
  trimble_windows_url:="${TRIMBLE_WINDOWS_URL:-http://127.0.0.1:8765}" \
  trimble_reference_scan_on_start:="${TRIMBLE_REFERENCE_SCAN_ON_START:-true}" \
  scan_decision:="${SCAN_DECISION:-true}" \
  scan_confidence_threshold:="${SCAN_CONFIDENCE_THRESHOLD:-0.65}" \
  scan_min_detections:="${SCAN_MIN_DETECTIONS:-1}" \
  scan_cooldown_sec:="${SCAN_COOLDOWN_SEC:-60.0}" \
  digital_twin_map:="${DIGITAL_TWIN_MAP:-true}" \
  frontier_planner:="${FRONTIER_PLANNER:-true}" \
  frame_anchor:="${FRAME_ANCHOR:-true}" \
  robot_world_frame:="${ROBOT_WORLD_FRAME:-oak_odom}" \
  anchor_store_path:="${ANCHOR_STORE_PATH:-/tmp/digital_twin_anchor.yaml}" \
  auto_anchor_on_first_scan:="${AUTO_ANCHOR_ON_FIRST_SCAN:-true}" \
  infrastructure_planner:="${INFRASTRUCTURE_PLANNER:-true}" \
  infrastructure_goal_cooldown_sec:="${INFRASTRUCTURE_GOAL_COOLDOWN_SEC:-20.0}" \
  robot_goal_bridge:="${ROBOT_GOAL_BRIDGE:-false}" \
  robot_goal_backend:="${ROBOT_GOAL_BACKEND:-dry_run}" \
  spot_command_url:="${SPOT_COMMAND_URL:-}" \
  spot_ip:="${SPOT_IP:-}" \
  spot_username:="${SPOT_USERNAME:-}" \
  spot_password:="${SPOT_PASSWORD:-}" \
  spot_command_frame:="${SPOT_COMMAND_FRAME:-odom}" \
  spot_goal_duration_sec:="${SPOT_GOAL_DURATION_SEC:-30.0}" \
  spot_arrival_timeout_sec:="${SPOT_ARRIVAL_TIMEOUT_SEC:-45.0}" \
  spot_auto_power_on:="${SPOT_AUTO_POWER_ON:-false}" \
  spot_stand_before_move:="${SPOT_STAND_BEFORE_MOVE:-true}" \
  arrival_check_source:="${ARRIVAL_CHECK_SOURCE:-tf}" \
  arrival_base_frame:="${ARRIVAL_BASE_FRAME:-body}" \
  arrival_position_tolerance_m:="${ARRIVAL_POSITION_TOLERANCE_M:-0.35}" \
  arrival_yaw_tolerance_rad:="${ARRIVAL_YAW_TOLERANCE_RAD:-0.45}" \
  arrival_stable_sec:="${ARRIVAL_STABLE_SEC:-1.5}" \
  defect_map:="${DEFECT_MAP:-true}" \
  pointcloud_monitor:=false \
  image_monitor:=false
