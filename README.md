# Infrastructure Defect Detection

ROS 2 workspace for autonomous infrastructure inspection on Boston Dynamics
Spot: the robot explores a site on its own, walks up to defects it spots,
triggers a Trimble X7 scan at every stop, and returns to where it started.

## The Pipeline

```text
Spot EAP lidar  ──┐
                  ├─► fused occupancy map ─► frontier goals ──┐
depth camera    ──┘                                           │
      │                                                       ▼
      └─► YOLO ─► 3D defects ─► defect standoff goals ─► inspection goal
                                     (preferred)              │
                                                              ▼
                                                    Spot SE2 walk command
                                                    (EAP feeds Spot's own
                                                     obstacle avoidance)
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

**Both sensors build one map.** The EAP lidar gives long range and 360°
coverage; the depth camera fills the lidar's near-field blind spot and catches
low or thin obstacles. A cell is only unknown when *neither* sensor has seen
it, so the frontier planner explores real terrain instead of blind spots.

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

**Spot + EAP (Enhanced Autonomy Payload).** The EAP does two jobs. Its lidar
feeds Spot's own onboard obstacle avoidance while walking, and this stack also
pulls that lidar cloud over the Boston Dynamics SDK
(`bosdyn.client.point_cloud.PointCloudClient`) and publishes it as
`/eap/points` for the occupancy map. Spot's world pose comes from the same SDK
via `spot_localization_bridge`.

The mission runs in Spot's **vision** frame rather than `odom`. Vision is
drift-corrected against Spot's own cameras, and the accuracy of the walk home
after a long excursion follows directly from that choice. Set
`SPOT_FRAME=odom` for the smoother but drifting alternative.

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

In sim the simulated world cloud stands in for the EAP lidar. The simulated
X7 still writes a LAS file, and — exactly as in the field — nothing reads it
back; it only stands in for the scanner's SD card. Sim state lives under
`/tmp/synthetic_demo` and is wiped on each start.

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
- `bosdyn-client` for Spot state, the EAP lidar, and motion commands
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
