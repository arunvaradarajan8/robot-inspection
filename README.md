# Infrastructure Defect Detection

ROS 2 workspace for USB-camera defect detection, generic point-cloud fusion,
Trimble X7 scan ingestion, digital-twin markers, infrastructure inspection
planning, and optional robot navigation backends.

## Pipeline

```text
USB camera -> /ros2_image -> YOLO -> /detections_2d
                                      |
                                      v
                           scan_decision_node
                                      |
                                      v
                         /digital_twin/scan_required

Trimble X7 LAS/LAZ folder -> /trimble/x7/scan_points
                                      |
                                      v
                /digital_twin/map + /digital_twin/defect_markers
                                      |
                                      v
              frame anchor + infrastructure inspection goals
                                      |
                                      v
                   dry-run, Nav2, or external Spot command bridge
```

Generic live or simulated point clouds can still be bridged:

```text
/lidar/raw -> pointcloud_bridge -> /lidar/points -> /detections_3d
```

For the field robot, navigation perception is intended to use a Luxonis
OAK-D Pro W class camera: IR illumination, wide FOV stereo, and OV9282 global
shutter stereo sensors. In that mode:

```text
OAK RGB image -> YOLO -> /detections_2d
OAK depth + camera_info + /detections_2d -> /detections_3d
OAK visual odometry/VSLAM -> /oak/odom -> oak_odom -> body TF
/detections_3d -> digital twin defect markers -> inspection/rescan goals
```

The OAK path is enabled with:

```text
OAK_DEPTH_NAVIGATION=true
OAK_LOCALIZATION=true
IMAGE_TOPIC=/oak/rgb/image_raw
OAK_DEPTH_TOPIC=/oak/rgb/depth
OAK_CAMERA_INFO_TOPIC=/oak/rgb/camera_info
OAK_ODOM_TOPIC=/oak/odom
ROBOT_WORLD_FRAME=oak_odom
NAVIGATION_BASE_FRAME=body
```

The depth image must be aligned to the RGB image used by YOLO. Topic names
depend on the `depthai_ros` launch file, so confirm them with `ros2 topic list`
on the Jetson and update `config/field.env` if needed.

Mission localization intentionally uses OAK odometry/VSLAM rather than Spot
odom. The OAK localization bridge republishes `/oak/odom` as TF, normally
`oak_odom -> body`. The first Trimble reference scan anchors the digital twin
`map` frame to `oak_odom`, so the planner computes goals from OAK-estimated
motion. Spot still uses its internal low-level balance/motor control to walk,
but the high-level inspection localization source is OAK.

For best results, configure the OAK odometry/VSLAM output so its child frame is
the robot body frame, or provide a calibrated static transform from the OAK
camera frame to `body`.

## Simulation (No Hardware)

The full pipeline runs end to end without any hardware. The same field
script switches between the real robot and simulation — only the arguments
change:

```bash
./scripts/run_field.sh full                  # real robot deployment
./scripts/run_field.sh full --sim gazebo     # same pipeline, Gazebo sim
./scripts/run_field.sh --sim synthetic       # pure-Python sim (no Gazebo)
```

Both simulation modes share one world definition
(`defect_detection/simulation/world_constants.py`): a concrete wall with
three defects (crack, spalling, exposed rebar), two pillars, and a ground
plane. The Gazebo world SDF, the synthetic LAS scans, and the ground-truth
detections are all generated from it, so they can never drift apart.

**Gazebo mode** (`--sim gazebo`, requires `sudo apt install
ros-jazzy-ros-gz`): Gazebo Harmonic simulates the robot, its RGBD camera,
and the inspection site with physics and rendering. `ros_gz_bridge` maps
the simulated camera and odometry onto the exact topics the field pipeline
uses; ground-truth sim nodes stand in for YOLO and the Trimble X7, and a
goal driver converts planner goals into `cmd_vel`. Everything runs on
`/clock` sim time.

**Synthetic mode** (`--sim synthetic`): one Python node renders the same
world analytically (camera, depth, detections, robot, scanner) — no GPU or
Gazebo needed. There is also a convenience wrapper that builds first:
`./scripts/run_synthetic_demo.sh`.

In both modes the real production nodes walk the full autonomy loop:

```text
camera + detections
  -> OAK depth fusion (3D detections)
  -> scan decision requests a Trimble scan
  -> sim X7 writes a LAS file; the scan watcher publishes it
  -> frame anchor locks the digital twin map frame
  -> occupancy map -> frontier + infrastructure planners publish goals
  -> robot goal bridge (dry run) -> robot drives to the goal
  -> arrival verified over TF -> rescan
```

RViz shows the camera view, 3D defect markers, the Trimble scan cloud, the
occupancy map, and the goal arrows as the loop runs. Sim state lives under
`/tmp/synthetic_demo` and is wiped on each start. To edit the world, change
`world_constants.py` and regenerate the Gazebo SDF with
`python3 scripts/generate_gazebo_world.py`.

## Requirements

- ROS 2 Jazzy
- Python 3.12
- OpenCV and `cv_bridge`
- Ultralytics for YOLO
- DepthAI ROS publishing OAK RGB/depth/camera_info topics
- `laspy` and `lazrs` for LAS/LAZ scan ingestion

Install field Python dependencies:

```bash
python3 -m pip install --user --break-system-packages -r requirements-field.txt
```

Build:

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Field Setup

Create a machine-specific config:

```bash
cp config/field.env.example config/field.env
```

Edit camera, model, calibration, Nav2, and Trimble scan-folder settings. The X7
workflow assumes scans are written as completed `.las` or `.laz` files into the
configured folder.

Run transport/digital-twin bringup:

```bash
./scripts/run_field.sh transport
```

Run full detection/fusion once the YOLO engine and calibration are ready:

```bash
./scripts/run_field.sh full
```

## Efficient X7 Scan Gate

The scan decision node prevents wasting X7 scans when detections are absent,
stale, or low confidence. It publishes:

- `/digital_twin/scan_required` (`std_msgs/Bool`)
- `/digital_twin/scan_reason` (`std_msgs/String`)

Defaults:

```text
scan_confidence_threshold:=0.65
scan_min_detections:=1
scan_cooldown_sec:=60.0
```

The Trimble scan watcher defaults to `trimble_require_scan_request:=true`, so
it only ingests the next completed scan after a high-confidence request.

For the first station/reference scan, enable the Perspective bridge. It requests
a reference scan on startup even when there are no detections:

```bash
ros2 launch pointcloud_bridge full_pipeline.launch.xml \
  trimble_windows_bridge:=true \
  trimble_windows_url:=http://PERSPECTIVE_HOST_IP:8765 \
  trimble_reference_scan_on_start:=true
```

## Digital Twin And Robot Motion

The X7 scan watcher publishes `/trimble/x7/scan_points`. The occupancy builder
turns that into `/digital_twin/map`.

The frame anchor node records the robot pose when the reference scan arrives and
publishes the transform from the digital-twin `map` frame to the robot world
frame. This is the coordinate glue that lets defect markers and scan stations be
converted into robot-relative goals. It persists:

- `/tmp/digital_twin_anchor.yaml`

The defect map node persists AI markers to YAML and republishes them as:

- `/digital_twin/defect_markers`
- `/digital_twin/rescan_goals`

The infrastructure planner prefers defect rescan goals first, then falls back to
map-frontier exploration goals. It publishes:

- `/infrastructure/inspection_goal`
- `/infrastructure/planner_status`

The robot goal bridge subscribes to `/infrastructure/inspection_goal`. It is off
by default so the stack can propose goals without moving hardware. Enable one of
these backends when the field command path is ready:

```text
ROBOT_GOAL_BRIDGE=true
ROBOT_GOAL_BACKEND=dry_run  # no motion, publishes arrival for software tests
ROBOT_GOAL_BACKEND=nav2     # send NavigateToPose goals
ROBOT_GOAL_BACKEND=http     # POST goals to an external Spot SDK command service
ROBOT_GOAL_BACKEND=spot_sdk # command Spot directly with the Boston Dynamics SDK
```

For Spot, the intended first hardware path is to use Spot-native localization and
mobility for walking, while this ROS stack handles inspection goals, scan
coordination, AI markers, and digital-twin updates.

Direct Spot SDK control requires the Jetson to reach Spot on the mission LAN and
the Boston Dynamics Python SDK to be installed from `requirements-field.txt`.
Configure:

```text
ROBOT_GOAL_BRIDGE=true
ROBOT_GOAL_BACKEND=spot_sdk
SPOT_IP=192.168.80.3
SPOT_USERNAME=...
SPOT_PASSWORD=...
SPOT_COMMAND_FRAME=odom
SPOT_AUTO_POWER_ON=false
SPOT_STAND_BEFORE_MOVE=true
```

Leave `SPOT_AUTO_POWER_ON=false` unless the tablet/operator workflow explicitly
allows the payload to power motors. The backend acquires a lease, optionally
commands stand, sends an SE2 trajectory goal, and publishes waypoint arrival
when the SDK trajectory command completes.

## Perspective Control Host

The Trimble X7 side is coordinated by a Windows Perspective control host. This
can be a Windows tablet or Windows laptop running Trimble Perspective and the
checked-in bridge app. The Jetson stays responsible for ROS 2, OAK-D
perception, AI detections, robot goals, and digital-twin processing.

```text
Windows tablet or Windows laptop
  -> Trimble Perspective controls the X7
  -> Perspective bridge app handles Start/Stop/status/scan-file transfer
  -> Jetson runs ROS 2, OAK-D, YOLO, planner, and digital twin
```

Install Python 3.12 for Windows, then install the bridge dependencies:

```text
tools\trimble_perspective_bridge\Install Windows Dependencies.bat
```

Launch the Windows/Tkinter bridge:

```powershell
py tools\trimble_perspective_bridge\windows_app.py
```

or double-click:

```text
tools\trimble_perspective_bridge\Launch Trimble Bridge.bat
```

Press `Start` in the app to SSH into the Jetson, build the ROS workspace, launch
the autonomy/digital-twin stack, and wait for the Jetson to report ready. Press
`Stop + Download Twin` to stop the Jetson ROS launch and copy configured
digital-twin outputs back to the control host.

The app also listens for Jetson scan requests, optionally launches Perspective,
watches the Perspective export folder, and prepares a Jetson-sized `.las` or
`.laz` copy before transfer. Full-resolution raw scans stay on the Perspective
host by default; this keeps Wi-Fi transfer practical.

Recommended Wi-Fi starting point:

```text
Jetson max points: 500000
Remote twin paths: /tmp/digital_twin_defects.yaml;/tmp/digital_twin_anchor.yaml
```

## Tests

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select defect_detection pointcloud_bridge
colcon test-result --verbose
```
