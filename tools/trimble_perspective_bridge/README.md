# Trimble Perspective Bridge App

This is the Windows companion control app for the Jetson ROS pipeline. The
Windows tablet or laptop runs Trimble Perspective and this Tkinter bridge app;
the Jetson stays focused on ROS 2, OAK-D localization, AI detection, robot
goals, and digital-twin processing.

It provides:

- A small interactive Tkinter UI.
- One-button `Start` to SSH into the Jetson, build, and launch ROS.
- One-button `Stop + Download Twin` to stop ROS and copy digital-twin outputs
  back to the control host.
- HTTP endpoints the Jetson can call:
  - `POST /scan_request`
  - `POST /waypoint_arrived`
  - `POST /jetson_ready`
  - `GET /health`
- Optional launch of Trimble Perspective where the host OS supports it.
- Optional export command hook if Trimble provides a CLI/API later.
- Automatic watch/copy of completed `.las`, `.laz`, or `.e57` files from a
  Perspective export folder to a Jetson-accessible scan folder.
- Wi-Fi-friendly LAS/LAZ reduction before transfer so full raw scans can stay
  on the Perspective/control host.

## Install On Windows Tablet Or Laptop

1. Install Python 3.12 for Windows and enable `Add python.exe to PATH`.
2. Copy this repository, or at least `tools\trimble_perspective_bridge`, onto
   the Windows machine.
3. Double-click:

```text
tools\trimble_perspective_bridge\Install Windows Dependencies.bat
```

Then launch the app with:

```text
tools\trimble_perspective_bridge\Launch Trimble Bridge.bat
```

You can also run it manually:

```powershell
py tools\trimble_perspective_bridge\windows_app.py
```

Configure:

- `Jetson host/IP`, `Jetson SSH user`, `Jetson workspace`.
- `Windows IP for Jetson` / control host IP, the address the Jetson can use to
  reach this app.
- `Perspective EXE`: path to Trimble Perspective.
- `Optional export command`: leave blank unless you have an automation command.
- `Perspective export folder`: where Perspective writes completed scans.
- `Reduced scan folder`: where the app writes Jetson-sized LAS/LAZ copies.
- `Jetson max points`: point cap for the transferred scan. Use `500000` to
  start on Wi-Fi.
- `Jetson scan folder`: usually a network share such as
  `\\JETSON\trimble_scans` or a synced folder.
- `Local twin folder`: where the app downloads digital-twin artifacts on stop.

Press `Start` once. The app runs the default remote command:

```text
cd ~/ros2_ws
colcon build --symlink-install --packages-select defect_detection pointcloud_bridge
TRIMBLE_WINDOWS_BRIDGE=true TRIMBLE_WINDOWS_URL=http://CONTROL_HOST_IP:8765 ./scripts/run_field.sh full
```

Press `Stop + Download Twin` to stop ROS and copy compact configured artifacts
such as `/tmp/digital_twin_defects.yaml` back to Windows. Full raw scans should
stay on the Windows/Perspective machine unless you deliberately add them to
`Remote twin paths`.

The scan phase is operator-paced. When the robot reaches a chosen vantage it
parks in SCAN and waits: the Jetson requests a scan and the app shows/logs it,
but the robot does **not** move on until you press `Next scan location` (the
mission-tab button or the browser dashboard). Take the X7 scan in Perspective,
then press it to release the robot to the next vantage. A new export appearing
in the watched folder is logged but, by default, does not release the robot on
its own — tick `Release robot automatically when a new export appears` in
Settings to restore that hands-off behaviour.

## Jetson URL

Launch the Jetson bridge node with the control host address:

```bash
ros2 launch pointcloud_bridge full_pipeline.launch.xml \
  trimble_windows_bridge:=true \
  trimble_windows_url:=http://CONTROL_HOST_IP:8765
```
