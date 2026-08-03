# Autonomous Infrastructure Inspection

ROS 2 workspace for autonomous infrastructure inspection on Boston Dynamics
Spot: the robot maps a site on its own with the depth camera, works out the
best places to scan from the map it built, triggers a Trimble X7 scan at each,
and returns to where it started.

## The Pipeline

```text
Oak-D depth camera ─► occupancy map ─┬─► PHASE 1  frontier goals
                                     │   (explore a bounded radius,
                                     │    fastest-payoff frontier first)
                                     │
                                     └─► PHASE 2  scan-vantage goals
                                         (rank free cells by openness ×
                                          centrality, scan the best few)
                                                       │
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
                                                       ▼ (phase 2 only)
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

**The depth camera builds the map — nothing else.** The Oak-D point cloud is
fused into one accumulated occupancy grid; a cell is only unknown when the
camera has never seen it, so the frontier planner explores real terrain instead
of blind spots. (The simulator and demo feed a synthetic cloud through the same
map source in place of a real camera.)

**Explore first, then scan — no object detection.** Where the robot goes
depends only on the shape of the map. Phase 1 sweeps a bounded radius, always
choosing the frontier with the best payoff-per-metre so mapping is
time-efficient. Phase 2 runs once the map is as complete as it will get: it
scores every free cell the robot could stand on by *openness* (clearance and
sightlines for the tripod) times *centrality* (closeness to the mapped
structure) and takes the X7 scans at the best few well-separated cells.

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

**Spot.** Spot is used for **motion only** — SE2 walk commands go over the
Boston Dynamics SDK, and Spot's own onboard obstacle avoidance runs underneath
every command. Spot's **leg/joint-encoder (kinematic) odometry is deliberately
not used for localization at all** (`SPOT_LOCALIZATION=false`); the
`spot_localization_bridge` that would read it stays off.

**Localization is the depth camera.** The robot's world pose comes entirely
from the Oak-D's **visual-inertial odometry** on `/oak/odom`, republished as
the `depth_odom -> body` TF by `depth_localization_bridge`
(`DEPTH_LOCALIZATION=true`, `ROBOT_WORLD_FRAME=depth_odom`). The occupancy map,
the planner, the scan planner, and the walk-home all live in that camera frame.
Your OAK launch must publish odometry on `/oak/odom` (on-device VIO or an
RTAB-Map/VIO node) or the robot will not be localized.

> The optional `robot_localization` EKF (`FUSED_LOCALIZATION=true`, with a
> **navX IMU** via `navx_imu_bridge` and `config/ekf.yaml`) is **off** and
> would fuse Spot's kinematic pose back in — enable it only if you deliberately
> want Spot's odometry in the estimate again.

> **Autonomous motion caveat:** goals are computed in `depth_odom`, but Spot
> only accepts move commands in its own frame, and with Spot localization off
> there is no TF linking the two. So `ROBOT_GOAL_BRIDGE=true`/`spot_sdk` needs
> that link added first; run supervised (walk Spot by tablet) until then.

**Depth camera (Oak-D Pro).** The only sensor that builds the map. Any depth
camera works; the defaults match a Luxonis OAK running `depthai_ros`. It
supplies the point cloud that fills in the occupancy grid. This workspace does
not launch the camera driver — point `MAP_DEPTH_POINTS_TOPIC` at whatever your
driver publishes. If the driver only publishes a depth image, run
`depth_image_proc` to produce the point cloud.

**Trimble X7.** Not controlled directly. Trimble Perspective runs it from a
Windows tablet or laptop, and the checked-in bridge app exposes an HTTP API so
the Jetson can request a scan and learn when it finished.

**Compute.** The Jetson runs ROS 2, perception, planning, and the Spot SDK
clients. The Windows host runs Perspective and the bridge app.

## Mission Lifecycle

`mission_manager` owns the mission and is what makes the run finite:

| State | What happens |
|---|---|
| `EXPLORING` | The frontier planner picks goals; every arrival is recorded as a breadcrumb. |
| `SCANNING` | Exploration is gated off; the scan planner drives to its chosen vantages, parks in SCAN at each, and waits for the operator to release it to the next. |
| `RETURNING` | The recorded stations are replayed in reverse, ending at the start pose. |
| `AWAITING_UPLOAD` | The robot is home. The mission stays open until the E57 is filed. |
| `COMPLETE` | Mission summary written. |

Exploration ends on whichever comes first: no frontier left inside the
exploration radius, the station limit, the excursion limit
(`MISSION_MAX_EXCURSION_M`, default 40 m), a duration cap, or an operator
command on `/mission/return_home`. The mission then enters `SCANNING`; when the
scan planner reports done on `/mission/scanning_complete`, the robot walks home.
The scan phase is operator-paced, so by default the mission waits as long as the
operator needs (`MISSION_SCANNING_TIMEOUT_SEC=0`). A mission summary — start
pose, every station, timings — is written to `MISSION_SUMMARY_PATH`.

The frontier planner, the scan planner, and the mission manager all publish to
`/infrastructure/inspection_goal`. `/mission/allow_exploration` (frontier
planner) and `/mission/start_scanning` (scan planner) guarantee only one of
them is driving at a time.

## Scan Planning

Once exploration ends, `scan_planner` scores every free cell on the finished
map by *openness* (the free-space fraction within `SCAN_OPENNESS_RADIUS_M`,
i.e. clearance and broad sightlines for the tripod) times *centrality*
(closeness to the mapped structure's centroid, with `SCAN_CENTRALITY_SCALE_M`
falloff). It picks the best `SCAN_MAX_STATIONS` cells that are at least
`SCAN_MIN_SEPARATION_M` apart, so the scans cover different parts of the site,
then drives to each, parks in SCAN, and triggers the X7. No object detection is
involved.

At each vantage the robot holds position in SCAN and **waits for a human** to
release it to the next scan location — it does not advance on a filesystem
export or a timer. The operator presses **"Next scan location"** in the Windows
console (`tools/trimble_perspective_bridge/windows_app.py`, tk button or the
browser dashboard); that is delivered over ROS on `/digital_twin/scan_complete`,
which advances the scan planner to the next vantage. The wait is unbounded by
default (`SCAN_WAIT_TIMEOUT_SEC=0`, `TRIMBLE_SCAN_TIMEOUT_SEC=0`); set either
above zero to add a safety auto-release. To restore the old hands-off behaviour
where a finished Perspective export releases the robot on its own, enable
*"Release robot automatically when a new export appears"* in the console
settings (`advance_on_file_export`).

## Simulation

```bash
./scripts/run_field.sh mission --sim synthetic  # pure Python, no GPU
./scripts/run_field.sh mission --sim gazebo     # needs ros-jazzy-ros-gz
```

Both modes run the real production nodes and share one world definition
(`defect_detection/simulation/world_constants.py`): a concrete wall, two
pillars, and a ground plane. The Gazebo world SDF and the synthetic cloud are
generated from it, so they cannot drift apart.

In sim the simulated world cloud feeds the map's cloud source in place of a
depth camera, the robot explores it, and the scan planner then picks vantages
and scans. The simulated X7 writes a LAS file and — exactly as in the field —
nothing reads it back; it only stands in for the scanner's SD card. Sim state
lives under `/tmp/synthetic_demo` and is wiped on each start.

## Demo Mode

```bash
./scripts/run_field.sh demo
```

The real robot, the real localization, the real Trimble X7 — an invented site.
`virtual_site_node` loads a predefined 3D point cloud, anchors it at the
robot's pose when the demo launched, and plays it back as if a sensor were
sweeping it: range-limited and occluded from the robot's live TF pose, so the
occupancy map fills in as the robot walks and the frontier planner still has
somewhere to go. Once exploration ends, the scan planner picks vantages on the
built map and triggers a real X7 scan at each.

Everything downstream is the field pipeline unmodified — same occupancy map,
frontier planner, scan planner, mission manager, goal bridge, and walk home.
Only the depth camera is absent.

The site is a YAML file (`src/defect_detection/config/demo_site.yaml`) whose
coordinates are metres relative to the robot at launch: `+x` ahead, `+y` left.
It describes walls, boxes, cylinders, and ground patches that get sampled into
a cloud. Point `cloud_path:` at a `.las`/`.laz`/`.pcd`/`.ply` instead to demo
against a real captured scan. Set `DEMO_SITE_PATH` in `config/field.env` to use
your own; the rest of the `DEMO_*` settings there control sensor range,
anchoring, the exploration radius, and the excursion leash.

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
  -> Jetson runs ROS 2, the depth camera, mapping, planning, and the digital twin
```

Install Python 3.12 for Windows, then:

```text
tools\trimble_perspective_bridge\Install Windows Dependencies.bat
py tools\trimble_perspective_bridge\windows_app.py
```

`Start Mission` SSHes into the Jetson, builds the workspace, and launches the
stack. `Next scan location` releases the robot from SCAN so it drives to the
next vantage — this is how the operator paces the scan phase. `Upload E57 +
Finish` files the scanner's E57 into `mission_output_dir` and closes the mission
on the robot.

Scan file transfer to the Jetson is **off** by default (`auto_transfer`), since
no scan feeds the live loop any more. Turn it on only to debug the legacy
ingest path.

## Requirements

Full setup for both machines — Jetson and the Windows scanner host — is in
**[docs/INSTALL.md](docs/INSTALL.md)**.

- ROS 2 Jazzy, Python 3.12
- OpenCV and `cv_bridge`
- NumPy (the frontier and scan planners are vectorized over the occupancy grid)
- A depth camera driver publishing a point cloud (and `camera_info`)
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
