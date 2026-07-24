# Coverage Mission (no ROS) — LEGACY

> **This implements the earlier design, not the current pipeline.** Here the
> Trimble X7 closes the loop: scans are transferred back mid-mission and turned
> into the occupancy map that drives exploration. The ROS stack no longer works
> that way — the depth camera builds the map, and the X7 is
> trigger-only with its scans staying on the SD card until the end. See the
> repository README.
>
> Kept as a hardware-light fallback for sites where the Jetson is unavailable.
> Do not treat it as the reference for the field pipeline.

Standalone Python controller that maximizes the terrain scanned by a
Trimble X7 mounted on Spot. Requires only the Boston Dynamics SDK and a
network path to the Windows computer running Trimble Perspective + the
bridge app (`tools/trimble_perspective_bridge`).

```text
this script (any computer: the Perspective laptop works fine)
  |-- bosdyn SDK --> Spot: pose (vision/odom frame), SE2 walk commands
  |                  (Spot's onboard obstacle avoidance runs underneath)
  |-- HTTP --------> Perspective bridge: POST /scan_request
  `-- watches -----> shared scan folder for the completed .las/.laz
```

The mission loop is sequential, so the robot naturally stands still
while the X7 scans:

1. Reference scan at the start pose; that pose anchors the map frame
   (Perspective registers later scans into the first station's frame).
2. The scan becomes a 2D occupancy map; every later scan merges in.
3. Walk to the farthest frontier (free cell bordering unscanned area).
4. Scan, merge, repeat — until no frontier remains or the station limit
   is reached.

## Install

```bash
python3 -m pip install -r requirements.txt
```

## Run

```bash
python3 mission.py \
  --spot-ip 192.168.80.3 \
  --bridge-url http://192.168.1.50:8765 \
  --scan-dir "C:/TrimbleScans"      # the bridge's scan delivery folder
```

Credentials come from `--spot-user/--spot-pass` or
`BOSDYN_CLIENT_USERNAME`/`BOSDYN_CLIENT_PASSWORD`. Keep the tablet
E-Stop active; the script refuses to run if Spot is estopped, and only
powers motors itself with `--power-on`. In the bridge app, point the
"Jetson scan folder" at the `--scan-dir` folder (any shared folder — no
Jetson involved).

Useful knobs: `--min-scan-separation` (m between stations),
`--min-frontier-distance`, `--max-stations`, `--frame odom|vision`,
`--scan-timeout`, `--walk-speed`, `--scan-yaw-offset` (fixed yaw between
the X7 export frame and the robot body at the reference scan).

## Outputs

`coverage_mission_output/` (override with `--output-dir`):

- `coverage_map.pgm` — the accumulated map (white free, black walls,
  gray unscanned), updated after every scan
- `mission.json` — anchor pose, every scan station, coverage stats

## Test without hardware

```bash
python3 mission.py --dry-run --scan-dir /tmp/fake_scans --bridge-url ''
```

The robot teleports between goals; drop `.las` files into the scan
folder (or let `test_mission_dry_run.py` do it) to drive the loop.
