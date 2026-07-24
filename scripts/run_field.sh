#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 [mission|demo|slam|transport|full] [--sim gazebo|synthetic]"
  echo
  echo "  mission          The field pipeline: the depth camera builds the"
  echo "                   map, the planner visits defects"
  echo "                   first and frontiers otherwise, the Trimble X7 is"
  echo "                   triggered at each stop, and the robot walks back"
  echo "                   to its start pose before waiting for the E57."
  echo "  slam             OAK-only depth-camera RGBD SLAM: RTAB-Map owns the"
  echo "                   map->odom->base_link chain, building its own internal"
  echo "                   map with loop closure from the depth camera alone."
  echo "                   No Spot, no Trimble; starts fresh at (0,0,0)."
  echo "  demo             The same mission on the real robot and the real"
  echo "                   X7, but the site is invented: a predefined 3D"
  echo "                   point cloud anchored at the robot's start pose"
  echo "                   stands in for the depth camera."
  echo "                   Run it in a large, empty, open area."
  echo "  transport|full   Reduced bringup for bench work on the real robot."
  echo "  --sim gazebo     Run the same pipeline against a Gazebo simulation."
  echo "  --sim synthetic  Run against the pure-Python synthetic world (no GPU)."
  echo
  echo "Examples:"
  echo "  $0 mission                   # the field pipeline on the real robot"
  echo "  $0 demo                      # real robot, virtual site"
  echo "  $0 mission --sim synthetic   # same loop, no hardware"
  echo "  $0 mission --sim gazebo      # same loop, simulated in Gazebo"
  exit 2
}

MODE="mission"
SIM="none"
while [[ $# -gt 0 ]]; do
  case "$1" in
    transport|full|mission|demo|slam) MODE="$1"; shift ;;
    # The old name for what is now the only field pipeline.
    coverage) MODE="mission"; shift ;;
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
elif [[ "${SIM}" == "none" && "${MODE}" != "slam" ]]; then
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

# Mission mode: defects first, frontiers as the fallback, a scan at every
# stop, one accumulated map fused from both sensors, and a managed return
# to the start pose at the end.
MISSION_ARGS=()
if [[ "${MODE}" == "mission" ]]; then
  MISSION_ARGS+=(
    scan_mode:=coverage
    prefer_defect_rescans:=true
    planner_hold_for_scan:=true
    digital_twin_accumulate:=true
    digital_twin_use_tf_scan_origin:=true
    mission_manager:=true
  )
fi

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
      demo_root:="${DEMO_ROOT}" \
      "${MISSION_ARGS[@]}"
  fi

  echo "Simulation mode: synthetic (pure Python, no Gazebo required)"
  exec ros2 launch defect_detection synthetic_demo.launch.xml \
    rviz:="${ENABLE_RVIZ:-true}" \
    demo_root:="${DEMO_ROOT}" \
    "${MISSION_ARGS[@]}"
fi

# SLAM mode: the OAK depth camera is the whole localization and mapping
# stack. RTAB-Map owns map->odom->base_link and builds its own internal
# occupancy grid with loop closure. No Spot, no Trimble, no field.env.
if [[ "${MODE}" == "slam" ]]; then
  "${WORKSPACE_ROOT}/scripts/field_preflight.sh" slam

  exec ros2 launch defect_detection rgbd_slam.launch.xml \
    rgb_topic:="${RGB_TOPIC:-/oak/rgb/image}" \
    depth_topic:="${DEPTH_TOPIC:-/oak/rgb/depth}" \
    camera_info_topic:="${DEPTH_CAMERA_INFO_TOPIC:-/oak/rgb/camera_info}" \
    odom_topic:="${DEPTH_ODOM_TOPIC:-/oak/odom}" \
    base_frame:="${SLAM_BASE_FRAME:-base_link}" \
    camera_optical_frame:="${SLAM_CAMERA_OPTICAL_FRAME:-oak_rgb_camera_optical_frame}" \
    cam_x:="${SLAM_CAM_X:-0.0}" \
    cam_y:="${SLAM_CAM_Y:-0.0}" \
    cam_z:="${SLAM_CAM_Z:-0.0}" \
    cam_roll:="${SLAM_CAM_ROLL:-0.0}" \
    cam_pitch:="${SLAM_CAM_PITCH:-0.0}" \
    cam_yaw:="${SLAM_CAM_YAW:-0.0}" \
    rtabmap_args:="${SLAM_RTABMAP_ARGS---delete_db_on_start}" \
    grid_resolution:="${SLAM_GRID_RESOLUTION:-0.05}" \
    grid_range_max:="${SLAM_GRID_RANGE_MAX:-5.0}" \
    rtabmap_viz:="${SLAM_RTABMAP_VIZ:-false}" \
    rviz:="${ENABLE_RVIZ:-false}"
fi

"${WORKSPACE_ROOT}/scripts/field_preflight.sh" "${MODE}"

# Demo mode: the robot, its localization, its motion, and the X7 are all
# real; only the site is invented. A predefined 3D point cloud anchored
# at the robot's start pose replaces the depth camera,
# so the whole mission loop runs in an empty open area.
if [[ "${MODE}" == "demo" ]]; then
  DEMO_ROOT="${DEMO_ROOT:-/tmp/demo_site}"
  mkdir -p "${DEMO_ROOT}"

  cat <<'WARNING'

  DEMO MODE
  The occupancy map will contain a site that is not there. Nothing in
  this stack knows about anything that actually is. Before enabling
  motion, confirm:
    * the area is open and empty for the site footprint plus the
      excursion leash below;
    * a spotter is holding the e-stop;
    * Spot's own obstacle avoidance is enabled.

WARNING

  exec ros2 launch defect_detection demo_site.launch.xml \
    site_path:="${DEMO_SITE_PATH:-$(ros2 pkg prefix defect_detection)/share/defect_detection/config/demo_site.yaml}" \
    site_density:="${DEMO_SITE_DENSITY:-1.0}" \
    sensor_range_m:="${DEMO_SENSOR_RANGE_M:-20.0}" \
    detection_range_m:="${DEMO_DETECTION_RANGE_M:-8.0}" \
    detection_fov_deg:="${DEMO_DETECTION_FOV_DEG:-90.0}" \
    anchor_to_start_pose:="${DEMO_ANCHOR_TO_START_POSE:-true}" \
    anchor_x:="${DEMO_ANCHOR_X:-0.0}" \
    anchor_y:="${DEMO_ANCHOR_Y:-0.0}" \
    anchor_yaw_deg:="${DEMO_ANCHOR_YAW_DEG:-0.0}" \
    demo_root:="${DEMO_ROOT}" \
    mission_max_excursion_m:="${DEMO_MAX_EXCURSION_M:-20.0}" \
    min_scan_separation_m:="${DEMO_MIN_SCAN_SEPARATION_M:-4.0}" \
    scan_cooldown_sec:="${DEMO_SCAN_COOLDOWN_SEC:-45.0}" \
    spot_localization:="${SPOT_LOCALIZATION:-true}" \
    spot_frame:="${SPOT_FRAME:-vision}" \
    spot_ip:="${SPOT_IP:-}" \
    spot_username:="${SPOT_USERNAME:-}" \
    spot_password:="${SPOT_PASSWORD:-}" \
    robot_world_frame:="${ROBOT_WORLD_FRAME:-vision}" \
    navigation_base_frame:="${NAVIGATION_BASE_FRAME:-body}" \
    robot_goal_bridge:="${ROBOT_GOAL_BRIDGE:-false}" \
    robot_goal_backend:="${ROBOT_GOAL_BACKEND:-spot_sdk}" \
    spot_command_frame:="${SPOT_COMMAND_FRAME:-vision}" \
    trimble_windows_bridge:="${TRIMBLE_WINDOWS_BRIDGE:-true}" \
    trimble_windows_url:="${TRIMBLE_WINDOWS_URL:-http://127.0.0.1:8765}" \
    trimble_reference_scan_on_start:="${TRIMBLE_REFERENCE_SCAN_ON_START:-true}" \
    rviz:="${ENABLE_RVIZ:-true}"
fi

detector=false
depth_navigation="${DEPTH_NAVIGATION:-false}"
depth_localization="${DEPTH_LOCALIZATION:-false}"
scan_mode="${SCAN_MODE:-coverage}"
prefer_defect_rescans="${PREFER_DEFECT_RESCANS:-true}"
planner_hold_for_scan="${PLANNER_HOLD_FOR_SCAN:-true}"
accumulate="${DIGITAL_TWIN_ACCUMULATE:-true}"
use_tf_scan_origin="${DIGITAL_TWIN_USE_TF_SCAN_ORIGIN:-true}"
spot_localization="${SPOT_LOCALIZATION:-false}"
mission_manager="${MISSION_MANAGER:-false}"
map_depth_enabled="${MAP_DEPTH_ENABLED:-false}"
robot_world_frame="${ROBOT_WORLD_FRAME:-vision}"
defect_map="${DEFECT_MAP:-true}"
fused_localization="${FUSED_LOCALIZATION:-false}"
if [[ "${MODE}" == "full" ]]; then
  detector=true
elif [[ "${MODE}" == "mission" ]]; then
  # The field pipeline. Spot's vision frame localizes the mission, the
  # depth camera builds the occupancy map, YOLO finds defects worth a
  # closer Trimble scan, and the mission manager walks the robot home
  # when there is nothing left to visit.
  detector=true
  depth_navigation="${DEPTH_NAVIGATION:-true}"
  depth_localization=false
  spot_localization="${SPOT_LOCALIZATION:-true}"
  mission_manager="${MISSION_MANAGER:-true}"
  map_depth_enabled="${MAP_DEPTH_ENABLED:-true}"
  defect_map="${DEFECT_MAP:-true}"
fi

# Fused localization: an EKF blends Spot vision pose + navX IMU + depth
# VIO. When on, the Spot bridge streams odometry only and the EKF owns
# the world->base TF, and the navX bridge is brought up by default.
if [[ "${fused_localization}" == "true" ]]; then
  spot_publish_tf=false
  navx_imu="${NAVX_IMU:-true}"
else
  spot_publish_tf=true
  navx_imu="${NAVX_IMU:-false}"
fi

exec ros2 launch pointcloud_bridge full_pipeline.launch.xml \
  cloud_input_topic:="${CLOUD_INPUT_TOPIC:-/cloud/raw}" \
  pointcloud_topic:="${POINTCLOUD_TOPIC:-/cloud/points}" \
  publish_camera:="${PUBLISH_CAMERA:-false}" \
  camera_index:="${CAMERA_INDEX:-0}" \
  camera_frame:="${CAMERA_FRAME:-camera_optical_frame}" \
  cloud_frame:="${CLOUD_FRAME:-cloud}" \
  dataset_path:="${DATASET_PATH:-}" \
  model_path:="${MODEL_PATH:-}" \
  image_topic:="${IMAGE_TOPIC:-/ros2_image}" \
  detections_2d_topic:="${DETECTIONS_2D_TOPIC:-/detections_2d}" \
  detections_3d_topic:="${DETECTIONS_3D_TOPIC:-/detections_3d}" \
  detector:="${detector}" \
  depth_navigation:="${depth_navigation}" \
  depth_topic:="${DEPTH_TOPIC:-/oak/rgb/depth}" \
  depth_camera_info_topic:="${DEPTH_CAMERA_INFO_TOPIC:-/oak/rgb/camera_info}" \
  depth_minimum_confidence:="${DEPTH_MINIMUM_CONFIDENCE:-0.50}" \
  depth_bbox_padding_px:="${DEPTH_BBOX_PADDING_PX:-4}" \
  depth_default_bbox_size_m:="${DEPTH_DEFAULT_BBOX_SIZE_M:-0.20}" \
  depth_localization:="${depth_localization}" \
  depth_odom_topic:="${DEPTH_ODOM_TOPIC:-/oak/odom}" \
  depth_odom_frame:="${DEPTH_ODOM_FRAME:-depth_odom}" \
  depth_odom_base_frame:="${DEPTH_ODOM_BASE_FRAME:-body}" \
  depth_odom_use_message_frame_ids:="${DEPTH_ODOM_USE_MESSAGE_FRAME_IDS:-true}" \
  visualization:=true \
  rviz:="${ENABLE_RVIZ:-true}" \
  navigation_base_frame:="${NAVIGATION_BASE_FRAME:-body}" \
  trimble_scan_watcher:="${TRIMBLE_SCAN_WATCHER:-false}" \
  trimble_scan_directory:="${TRIMBLE_SCAN_DIRECTORY:-/tmp/trimble_scans}" \
  trimble_scan_topic:="${TRIMBLE_SCAN_TOPIC:-/trimble/x7/scan_points}" \
  trimble_scan_frame:="${TRIMBLE_SCAN_FRAME:-map}" \
  trimble_windows_bridge:="${TRIMBLE_WINDOWS_BRIDGE:-false}" \
  trimble_windows_url:="${TRIMBLE_WINDOWS_URL:-http://127.0.0.1:8765}" \
  trimble_reference_scan_on_start:="${TRIMBLE_REFERENCE_SCAN_ON_START:-true}" \
  scan_decision:="${SCAN_DECISION:-true}" \
  scan_mode:="${scan_mode}" \
  scan_confidence_threshold:="${SCAN_CONFIDENCE_THRESHOLD:-0.65}" \
  scan_min_detections:="${SCAN_MIN_DETECTIONS:-1}" \
  scan_cooldown_sec:="${SCAN_COOLDOWN_SEC:-60.0}" \
  min_scan_separation_m:="${MIN_SCAN_SEPARATION_M:-10.0}" \
  digital_twin_map:="${DIGITAL_TWIN_MAP:-true}" \
  digital_twin_accumulate:="${accumulate}" \
  digital_twin_use_tf_scan_origin:="${use_tf_scan_origin}" \
  map_cloud_enabled:="${MAP_CLOUD_ENABLED:-false}" \
  map_cloud_topic:="${MAP_CLOUD_TOPIC:-/cloud/points}" \
  map_cloud_max_range_m:="${MAP_CLOUD_MAX_RANGE_M:-40.0}" \
  map_cloud_sensor_frame:="${MAP_CLOUD_SENSOR_FRAME:-}" \
  map_depth_enabled:="${map_depth_enabled}" \
  map_depth_points_topic:="${MAP_DEPTH_POINTS_TOPIC:-/depth/points}" \
  map_depth_max_range_m:="${MAP_DEPTH_MAX_RANGE_M:-5.0}" \
  map_depth_sensor_frame:="${MAP_DEPTH_SENSOR_FRAME:-}" \
  frame_anchor:="${FRAME_ANCHOR:-false}" \
  robot_world_frame:="${robot_world_frame}" \
  anchor_store_path:="${ANCHOR_STORE_PATH:-/tmp/digital_twin_anchor.yaml}" \
  auto_anchor_on_first_scan:="${AUTO_ANCHOR_ON_FIRST_SCAN:-true}" \
  infrastructure_planner:="${INFRASTRUCTURE_PLANNER:-true}" \
  infrastructure_goal_cooldown_sec:="${INFRASTRUCTURE_GOAL_COOLDOWN_SEC:-20.0}" \
  prefer_defect_rescans:="${prefer_defect_rescans}" \
  planner_hold_for_scan:="${planner_hold_for_scan}" \
  planner_scan_wait_timeout_sec:="${PLANNER_SCAN_WAIT_TIMEOUT_SEC:-300.0}" \
  spot_localization:="${spot_localization}" \
  spot_frame:="${SPOT_FRAME:-vision}" \
  spot_odom_topic:="${SPOT_ODOM_TOPIC:-/spot/odom}" \
  spot_publish_tf:="${spot_publish_tf}" \
  fused_localization:="${fused_localization}" \
  navx_imu:="${navx_imu}" \
  navx_mode:="${NAVX_MODE:-relay}" \
  navx_imu_topic:="${NAVX_IMU_TOPIC:-/navx/imu}" \
  navx_input_imu_topic:="${NAVX_INPUT_IMU_TOPIC:-/navx/imu_raw}" \
  navx_serial_port:="${NAVX_SERIAL_PORT:-/dev/ttyACM0}" \
  navx_serial_baud:="${NAVX_SERIAL_BAUD:-115200}" \
  mission_manager:="${mission_manager}" \
  mission_summary_path:="${MISSION_SUMMARY_PATH:-/tmp/mission_summary.yaml}" \
  mission_max_stations:="${MISSION_MAX_STATIONS:-0}" \
  mission_max_excursion_m:="${MISSION_MAX_EXCURSION_M:-30.0}" \
  mission_duration_sec:="${MISSION_DURATION_SEC:-0.0}" \
  mission_home_position_tolerance_m:="${MISSION_HOME_POSITION_TOLERANCE_M:-1.0}" \
  trimble_scan_timeout_sec:="${TRIMBLE_SCAN_TIMEOUT_SEC:-300.0}" \
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
  defect_map:="${defect_map}"
