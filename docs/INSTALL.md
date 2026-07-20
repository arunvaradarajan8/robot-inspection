# Installation Guide

Two computers run this system. The split is not arbitrary — it follows from
what each vendor's software will run on.

| | Jetson (on Spot) | Windows host (tablet or laptop) |
|---|---|---|
| Runs | ROS 2, YOLO inference, planning, Spot SDK | Trimble Perspective + the bridge app |
| Talks to | Spot (EAP lidar, pose, motion), the depth camera | The Trimble X7, the Jetson over HTTP/SSH |
| Why there | CUDA/TensorRT for inference; must ride the robot | **Perspective is Windows-only** — this is the whole reason a second machine exists |

The Windows host does *no* perception and *no* autonomy. It is a scanner
controller and an operator console. If it goes offline mid-mission the robot
keeps mapping and navigating; it just stops getting scans.

---

## Part 1 — Jetson

Assumes JetPack 6 (Ubuntu 22.04/24.04 base) on an Orin-class module. Check
what you have first, because every later choice depends on it:

```bash
cat /etc/nv_tegra_release
sudo apt install -y python3-pip git curl
```

### 1.1 ROS 2 Jazzy

```bash
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-jazzy-ros-base ros-dev-tools
sudo apt install -y \
  ros-jazzy-cv-bridge \
  ros-jazzy-vision-msgs \
  ros-jazzy-nav2-msgs \
  ros-jazzy-tf2-ros \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-message-filters \
  ros-jazzy-rviz2

echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc
source /opt/ros/jazzy/setup.bash
```

> If JetPack pins you to Ubuntu 22.04, Jazzy has no binaries for it. Either
> move to a JetPack release on 24.04, or run the stack in the OSRF `ros:jazzy`
> container with `--runtime nvidia`. Do not try to source-build Jazzy on the
> Jetson — it is hours of work and breaks on the next JetPack update.

### 1.2 YOLO inference

This is the part people get wrong. **Do not `pip install torch`** — PyPI wheels
are CPU-only x86 builds and will either fail to install or silently run YOLO on
the CPU at about 2 FPS.

```bash
# NVIDIA's Jetson-specific PyTorch. Match the URL to your JetPack version:
#   https://developer.download.nvidia.com/compute/redist/jp/
sudo apt install -y libopenblas-base libopenmpi-dev
pip3 install --no-cache-dir \
  --index-url https://pypi.jetson-ai-lab.dev/jp6/cu126 torch torchvision

# Ultralytics, without letting it pull its own torch
pip3 install --no-deps ultralytics
pip3 install opencv-python numpy pyyaml pillow tqdm psutil matplotlib pandas
```

Verify CUDA is actually live before going further:

```bash
python3 -c "import torch; print(torch.__version__, torch.cuda.is_available())"
```

If that prints `False`, stop and fix it — nothing downstream will perform.

### 1.3 Build the TensorRT engine

`config/field.env` points `MODEL_PATH` at `models/yolov11m.engine`. **That file
is not in the repo and cannot be** — a TensorRT engine is built for one exact
GPU, TensorRT version, and JetPack. An engine built anywhere else will not
load. Build it on the Jetson itself from your trained `.pt`:

```bash
cd ~/ros2_ws/src/defect_detection/models
yolo export model=yolov11m.pt format=engine half=True device=0 imgsz=640
```

Expect 10–20 minutes. Rebuild it after any JetPack or TensorRT upgrade.

### 1.4 Spot SDK and workspace dependencies

```bash
cd ~/ros2_ws
pip3 install --user -r requirements-field.txt   # bosdyn-client, laspy, lazrs
```

### 1.5 Depth camera driver

The workspace does **not** launch your camera — it subscribes to topics. For a
Luxonis OAK:

```bash
sudo apt install -y ros-jazzy-depthai-ros
echo 'SUBSYSTEM=="usb", ATTRS{idVendor}=="03e7", MODE="0666"' \
  | sudo tee /etc/udev/rules.d/80-movidius.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

The occupancy map needs a **point cloud**, not just a depth image. If your
driver publishes only a depth image, add:

```bash
sudo apt install -y ros-jazzy-depth-image-proc
```

and run `depth_image_proc/point_cloud_xyz` to produce `MAP_DEPTH_POINTS_TOPIC`.

### 1.6 Build and configure

```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

cp config/field.env.example config/field.env
# Set SPOT_IP, SPOT_USERNAME, SPOT_PASSWORD, and confirm the DEPTH_* topics
# match `ros2 topic list` with your camera driver running.
```

Confirm the topic names rather than trusting the defaults:

```bash
ros2 topic list | grep -E "rgb|depth|points"
```

### 1.7 Verify

```bash
./scripts/field_preflight.sh mission
```

It checks the Spot SDK import, the EAP point cloud client, reachability of
`SPOT_IP`, and the model/dataset files. Then dry-run the whole loop with no
hardware at all:

```bash
./scripts/run_field.sh mission --sim synthetic
```

---

## Part 2 — Windows host

Runs Trimble Perspective and the bridge app. Nothing else.

### 2.1 Prerequisites

1. **Trimble Perspective**, installed and already paired with your X7 over its
   Wi-Fi. Confirm you can scan from Perspective alone before adding software —
   debugging the bridge and the scanner pairing at once is miserable.
2. **Python 3.12 for Windows** from python.org. Tick *Add python.exe to PATH*.

### 2.2 Bridge app

```text
cd tools\trimble_perspective_bridge
"Install Windows Dependencies.bat"
py windows_app.py
```

or double-click `Launch Trimble Bridge.bat`.

### 2.3 Configure

In the app's config tab:

| Setting | Value |
|---|---|
| `jetson_host` / `jetson_user` | The Jetson's IP and login |
| `jetson_workspace` | `~/ros2_ws` |
| `jetson_mode` | `mission` |
| `export_dir` | Perspective's export folder — watched to detect scan completion |
| `mission_output_dir` | Where the end-of-mission E57 is filed |
| `auto_transfer` | **false** — scans stay on the scanner |

The Jetson needs `TRIMBLE_WINDOWS_URL` in `config/field.env` pointing at this
machine's IP on port 8765.

### 2.4 Firewall

Windows will silently block the Jetson's requests otherwise:

```powershell
New-NetFirewallRule -DisplayName "Trimble Bridge" -Direction Inbound `
  -LocalPort 8765 -Protocol TCP -Action Allow
```

---

## Part 3 — Network

All three devices on one LAN:

| Link | Port | Direction |
|---|---|---|
| Jetson → Spot | 443 (gRPC) | SDK: pose, EAP lidar, motion |
| Jetson → Windows | 8765 | Scan requests, status polling |
| Windows → Jetson | 22 | SSH start/stop |
| Windows ↔ X7 | Trimble Wi-Fi | Perspective controls the scanner |

Set the same `ROS_DOMAIN_ID` (default 42) anywhere you run ROS.

Test both directions before the robot leaves the bench:

```bash
# On the Jetson
ping <spot-ip> && ping <windows-ip>
curl http://<windows-ip>:8765/status
```

```powershell
# On Windows
ssh <user>@<jetson-ip> "echo ok"
```

---

## Part 4 — First mission

1. Windows: launch Perspective, then the bridge app.
2. Jetson: `./scripts/run_field.sh mission` with `ROBOT_GOAL_BRIDGE=false`.
   The stack proposes goals but **does not move the robot**. Walk Spot by
   tablet and confirm `/digital_twin/map` grows and `/mission/state` advances.
3. Confirm the sensors independently:
   ```bash
   ros2 topic hz /eap/points
   ros2 topic hz /depth/points
   ros2 topic echo /mission/state --once
   ```
4. Only once all of that is clean, set `ROBOT_GOAL_BRIDGE=true` and
   `ROBOT_GOAL_BACKEND=spot_sdk`. Keep the tablet E-Stop in hand.
5. At the end: robot returns to start → state reaches `AWAITING_UPLOAD` → pull
   the E57 off the X7's SD card → **Upload E57 + Finish** in the app.

---

## Troubleshooting

**`torch.cuda.is_available()` is False** — you have a CPU wheel. Uninstall and
reinstall from NVIDIA's Jetson index (§1.2).

**Engine fails to load** — built on different hardware or a different TensorRT.
Rebuild on this Jetson (§1.3).

**`bosdyn PointCloudClient is unavailable`** — `bosdyn-client` missing, or your
Spot has no EAP. Without it there is no occupancy map and no exploration.

**Robot stands still at every station for 5 minutes** — the Windows bridge is
not reporting scan completion. Check `curl http://<windows-ip>:8765/status`
returns a rising `scans_completed`. The 5 minutes is
`TRIMBLE_SCAN_TIMEOUT_SEC` expiring.

**Map has a hole under the robot** — depth camera not contributing. Check
`MAP_DEPTH_POINTS_TOPIC` exists and that TF resolves from the map frame to the
camera frame.

**Robot misses the start pose on return** — you are probably on `odom`. Set
`SPOT_FRAME=vision`.
