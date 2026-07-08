#!/usr/bin/env bash
# Build and launch the full pipeline on synthetic data. No hardware needed.
#
#   ./scripts/run_synthetic_demo.sh              # build + launch with RViz
#   ./scripts/run_synthetic_demo.sh --no-rviz    # headless (topics only)
#   ./scripts/run_synthetic_demo.sh --no-build   # skip colcon build
set -euo pipefail

WS_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEMO_ROOT=/tmp/synthetic_demo
RVIZ=true
BUILD=true

for arg in "$@"; do
  case "$arg" in
    --no-rviz) RVIZ=false ;;
    --no-build) BUILD=false ;;
    *) echo "Unknown option: $arg" >&2; exit 1 ;;
  esac
done

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash

# Fresh demo state: stale anchors or defect stores from a previous run
# would offset the digital twin.
rm -rf "$DEMO_ROOT"
mkdir -p "$DEMO_ROOT/trimble_scans"

cd "$WS_ROOT"
if [ "$BUILD" = true ]; then
  colcon build --packages-select defect_detection --symlink-install
fi
# shellcheck disable=SC1091
source "$WS_ROOT/install/setup.bash"

echo
echo "Synthetic demo starting. Watch for this sequence:"
echo "  1. Synthetic camera publishes imagery with 3 defects (crack,"
echo "     spalling, exposed rebar) on a concrete wall."
echo "  2. OAK depth fusion turns them into 3D detections (markers in RViz)."
echo "  3. Scan decision requests a Trimble scan; the synthetic X7 writes a"
echo "     LAS file; the scan watcher publishes the scan cloud."
echo "  4. Frame anchor locks the digital twin; the occupancy map appears."
echo "  5. Planners publish inspection goals; the simulated robot drives"
echo "     to them; the goal bridge verifies arrival over TF and rescans."
echo

exec ros2 launch defect_detection synthetic_demo.launch.xml rviz:="$RVIZ"
