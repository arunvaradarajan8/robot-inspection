# Autonomous Infrastructure Inspection

ROS 2 workspace for autonomous infrastructure inspection on Boston Dynamics
Spot: the robot explores a site on its own, walks up to defects it spots,
triggers a Trimble X7 scan at every stop, and returns to where it started.

## The Pipeline

```text
depth camera ──┬─► occupancy map ─► frontier goals ───────────┐
               │                                              │
               └─► YOLO ─► 3D defects ─► defect standoff goals ┤
                                     (preferred)              │
                                                       inspection goal
                                                              │
                                                              ▼
                                                    Spot SE2 walk command
                                                    (Spot's own onboard
                                                     obstacle avoidance runs)
                                                              │
                                                              ▼
                                              arrival verified over TF
                                                              │
                                                              ▼
                                          app triggers an X7 scan; the robot
                                          stands still until it finishes
                                                              │
                                                              ▼
                                       mission ends ─► walk back to the start
                                                    ─► wait for the E57 upload
```

Three properties define this design:

**The Trimble never closes the loop.** The X7 is triggered by the app and
writes its scans to its own SD card. Nothing it produces feeds localization,
mapping, or planning during the mission. The operator uploads the E57 at the
end, once the robot is home.

**The depth camera builds the map.** Its point cloud is fused into one
accumulated occupancy grid; a cell is only unknown when the camera has never
seen it, so the frontier planner explores real terrain instead of blind spots.
(The simulator and demo feed a long-range synthetic cloud through the same
map source in place of a real camera.)

**Defects come first.** When YOLO has found something, the planner walks to a
standoff distance from it so the X7 captures it closely. Only when there is
nothing to revisit does it fall back to frontier exploration.

Run it:

```bash
cp config/field.env.example config/field.env   # set SPOT_IP + credentials
./scripts/run_field.sh mission                 # real robot
./scripts/run_field.sh mission --sim synthetic # same loop, no hardware
```

Motion stays disabled until `ROBOT_GOAL_BRIDGE=true` and
`ROBOT_GOAL_BACKEND=spot_sdk` are set in `config/field.env`; before that the
stack proposes goals without moving hardware.

## Hardware

**Spot.** The robot's world pose comes over the Boston Dynamics SDK via
`spot_localization_bridge`, and SE2 walk commands go back the same way. Spot's
own onboard obstacle avoidance runs underneath every command.

The mission runs in Spot's **vision** frame rather than `odom`. Vision is
drift-corrected against Spot's own cameras, and the accuracy of the walk home
after a long excursion follows directly from that choice. Set
`SPOT_FRAME=odom` for the smoother but drifting alternative.

**Localization (optional fusion).** Spot's vision frame alone already walks
the robot home. For a steadier estimate you can fuse Spot's pose, a **navX
IMU**, and the depth camera's **visual odometry** through a `robot_localization`
EKF (`FUSED_LOCALIZATION=true`). When on, `spot_localization_bridge` streams
odometry only and the EKF owns the `vision -> body` transform; the navX bridge
(`navx_imu_bridge`) publishes `sensor_msgs/Imu` either by relaying an existing
Imu topic or by reading a navX over USB/UART. See the EKF inputs in
`config/ekf.yaml` and `launch/fused_localization.launch.xml`, and verify the
TF tree on the Jetson before relying on it.

**Depth camera.** Any depth camera works; the defaults match a Luxonis OAK
running `depthai_ros`. It supplies RGB for YOLO, aligned depth to turn 2D
boxes into 3D defect positions, and a point cloud for the near field of the
occupancy map. This workspace does not launch the camera driver — point
`DEPTH_TOPIC`, `DEPTH_CAMERA_INFO_TOPIC`, and `MAP_DEPTH_POINTS_TOPIC` at
whatever your driver publishes. If the driver only publishes a depth image,
run `depth_image_proc` to produce the point cloud.

**Trimble X7.** Not controlled directly. Trimble Perspective runs it from a
Windows tablet or laptop, and the checked-in bridge app exposes an HTTP API so
the Jetson can request a scan and learn when it finished.

**Compute.** The Jetson runs ROS 2, perception, planning, and the Spot SDK
clients. The Windows host runs Perspective and the bridge app.

## Mission Lifecycle

`mission_manager` owns the mission and is what makes the run finite:

| State | What happens |
|---|---|
| `EXPLORING` | The planner picks goals; every arrival is recorded as a breadcrumb. |
| `RETURNING` | Exploration is gated off and the recorded stations are replayed in reverse, ending at the start pose. |
| `AWAITING_UPLOAD` | The robot is home. The mission stays open until the E57 is filed. |
| `COMPLETE` | Mission summary written. |

Exploration ends on whichever comes first: no frontier left, the station limit,
the excursion limit (`MISSION_MAX_EXCURSION_M`, default 30 m), a duration cap,
or an operator command on `/mission/return_home`. A mission summary — start
pose, every station, timings — is written to `MISSION_SUMMARY_PATH`.

The planner and the mission manager both publish to
`/infrastructure/inspection_goal`, and `/mission/allow_exploration` guarantees
only one of them is doing so at a time.

## Scan Triggering

`scan_decision_node` runs in `coverage` mode by default, meaning it requests a
scan wherever the planner parked the robot — the planner already decided the
stop was worth making. Scans are still paced by `SCAN_COOLDOWN_SEC` and
`MIN_SCAN_SEPARATION_M`, so the robot must move meaningfully between stations.
Set `SCAN_MODE=detection` to additionally require live high-confidence
detections.

While a scan runs the robot must stand still. Because nothing comes back from
the X7 over ROS, the Windows bridge reports completion on
`/digital_twin/scan_complete`, and that is what releases the planner. A
`TRIMBLE_SCAN_TIMEOUT_SEC` safeguard releases it anyway if the report never
arrives.

## Simulation

```bash
./scripts/run_field.sh mission --sim synthetic  # pure Python, no GPU
./scripts/run_field.sh mission --sim gazebo     # needs ros-jazzy-ros-gz
```

Both modes run the real production nodes and share one world definition
(`defect_detection/simulation/world_constants.py`): a concrete wall with three
defects, two pillars, and a ground plane. The Gazebo world SDF, the synthetic
scans, and the ground-truth detections are all generated from it, so they
cannot drift apart.

In sim the simulated world cloud feeds the map's long-range cloud source in
place of a depth camera. The simulated
X7 still writes a LAS file, and — exactly as in the field — nothing reads it
back; it only stands in for the scanner's SD card. Sim state lives under
`/tmp/synthetic_demo` and is wiped on each start.

## Demo Mode

```bash
./scripts/run_field.sh demo
```

The real robot, the real localization, the real Trimble X7 — an invented site.
`virtual_site_node` loads a predefined 3D point cloud, anchors it at the
robot's pose when the demo launched, and plays it back as if a sensor were
sweeping it: range-limited and occluded from the robot's live TF pose, so the
occupancy map fills in as the robot walks and the frontier planner still has
somewhere to go. Defects planted in the site are published as 3D detections
once the robot is close enough and facing them, which sends it to a standoff
pose and triggers a real X7 scan on arrival.

Everything downstream is the field pipeline unmodified — same occupancy map,
planner, mission manager, goal bridge, scan trigger, and walk home. Only the
depth camera and YOLO are absent.

The site is a YAML file (`src/defect_detection/config/demo_site.yaml`) whose
coordinates are metres relative to the robot at launch: `+x` ahead, `+y` left.
It describes walls, boxes, cylinders, and ground patches that get sampled into
a cloud, plus the defect list. Point `cloud_path:` at a `.las`/`.laz`/`.pcd`/
`.ply` instead to demo against a real captured scan, with the defect list
authored on top of it. Set `DEMO_SITE_PATH` in `config/field.env` to use your
own; the rest of the `DEMO_*` settings there control sensor range, detection
range, anchoring, and the excursion leash.

> **Safety.** The occupancy map contains a site that is not there, and nothing
> in the stack knows about anything that is. Run it in a large open area —
> empty for the site's footprint plus `DEMO_MAX_EXCURSION_M` — with a spotter
> on the e-stop. Spot's own onboard obstacle avoidance still runs underneath
> the SE2 commands, but it is the only thing that does. Motion stays disabled
> until `ROBOT_GOAL_BRIDGE=true`, as in the field.

## Depth-Camera RGBD SLAM (OAK-only)

```bash
sudo apt install ros-jazzy-rtabmap-ros
./scripts/run_field.sh slam        # or the launch file directly, below
```

The field and demo pipelines never localize *against* their map: the depth
camera's `/oak/odom` is relayed straight to TF (`depth_localization_bridge`)
and the occupancy grid (`pointcloud_to_occupancy`) is only *drawn from* that
pose — open-loop dead reckoning that drifts, with `map` owned by Spot's
vision frame. This mode is the opposite: the **OAK camera is the sole
localization and mapping authority**. [RTAB-Map](http://introlab.github.io/rtabmap/)
takes the RGBD stream plus `/oak/odom`, runs loop closure, and owns the whole
transform chain, publishing its own internal map that corrects the pose:

```text
map --(rtabmap, loop-closure corrected)--> odom
    --(depth_localization_bridge from /oak/odom)--> base_link
    --(static extrinsics)--> oak_rgb_camera_optical_frame
```

Outputs are `/rtabmap/grid_map` (2D `OccupancyGrid`), `/rtabmap/cloud_map`
(3D `PointCloud2`), and the loop-closure-corrected pose. The map starts empty
at the origin on every run (`--delete_db_on_start`); clear `SLAM_RTABMAP_ARGS`
to keep `~/.ros/rtabmap.db` and relocalize into a previously built map instead.

Launch it directly to set the camera mount pose (`base_link ->` optical frame),
which you **must** provide for your rig — the identity default produces a
geometrically wrong map:

```bash
ros2 launch defect_detection rgbd_slam.launch.xml \
  cam_x:=0.20 cam_z:=0.15 cam_roll:=-1.5708 cam_yaw:=-1.5708 \
  rtabmap_viz:=true
```

The same values are available to `run_field.sh slam` as `SLAM_CAM_X`,
`SLAM_CAM_ROLL`, … environment variables.

**OAK stream requirements.** RTAB-Map reconstructs from the RGB camera model,
so the depth image must be registered/aligned to the RGB camera
(`setDepthAlign(RGB)` in the DepthAI pipeline), depth and RGB must share one
resolution, RGB must be `rgb8`/`bgr8`, and `/oak/rgb/camera_info` must match
that resolution. Misaligned depth is the most common cause of a warped map.

This mode runs no Spot, no Trimble, and no YOLO, so it needs neither
`config/field.env` nor the Boston Dynamics SDK — only `ros-jazzy-rtabmap-ros`
and a depth camera driver publishing the `/oak/*` topics.

## Perspective Control Host

```text
Windows tablet or laptop
  -> Trimble Perspective controls the X7
  -> bridge app: Start/Stop, scan trigger, scan-finished, E57 upload
  -> Jetson runs ROS 2, the depth camera, YOLO, planning, and the digital twin
```

Install Python 3.12 for Windows, then:

```text
tools\trimble_perspective_bridge\Install Windows Dependencies.bat
py tools\trimble_perspective_bridge\windows_app.py
```

`Start Mission` SSHes into the Jetson, builds the workspace, and launches the
stack. `Scan Finished` marks a scan complete manually if Perspective's export
folder is not being watched. `Upload E57 + Finish` files the scanner's E57 into
`mission_output_dir` and closes the mission on the robot.

Scan file transfer to the Jetson is **off** by default (`auto_transfer`), since
no scan feeds the live loop any more. Turn it on only to debug the legacy
ingest path.

## Requirements

Full setup for both machines — Jetson and the Windows scanner host — is in
**[docs/INSTALL.md](docs/INSTALL.md)**. Two things bite people there: the
Jetson needs NVIDIA's PyTorch build rather than the PyPI one, and the TensorRT
engine must be built on the Jetson itself.

- ROS 2 Jazzy, Python 3.12
- OpenCV and `cv_bridge`
- Ultralytics for YOLO
- A depth camera driver publishing RGB, aligned depth, and `camera_info`
- `bosdyn-client` for Spot state and motion commands
- `robot_localization` and `pyserial`, only for the optional fused localization
- `laspy` / `lazrs` only for the legacy LAS ingest path and the simulator

```bash
python3 -m pip install --user --break-system-packages -r requirements-field.txt
cd ~/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
```

## Legacy Paths

These are kept in the repo but default **off** and are wired to nothing in the
mission loop. They belong to the earlier design in which Trimble scans built
the map:

- `trimble_scan_watcher` — ingested LAS/LAZ from a watched folder
- `frame_anchor_node` — anchored the twin's `map` frame to the robot world
  frame. Unnecessary now: the map is built directly in Spot's world frame.
- `tools/coverage_mission/` — a standalone non-ROS mission controller that
  still implements the old X7-closes-the-loop design
- `depth_localization_bridge` — depth-camera VIO as the localization source
- `image_publisher` — USB webcam source, for bench work without a depth camera

## Tests

```bash
source /opt/ros/jazzy/setup.bash
colcon test --packages-select defect_detection pointcloud_bridge
colcon test-result --verbose
```
