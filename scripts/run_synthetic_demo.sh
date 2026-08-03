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
echo "  1. The synthetic world cloud builds the Oak-D occupancy map."
echo "  2. The frontier planner publishes exploration goals; the simulated"
echo "     robot drives to them and maps the bounded radius."
echo "  3. Exploration ends; the scan planner ranks map vantages by"
echo "     openness x centrality and drives to the best few."
echo "  4. At each vantage the robot parks in SCAN and waits to be released"
echo "     to the next scan location. On real hardware the operator presses"
echo "     'Next scan location'; here the synthetic X7 writes a LAS and"
echo "     auto-releases the robot as a stand-in for that press."
echo "  5. The mission manager walks the robot back to its start pose."
echo

exec ros2 launch defect_detection synthetic_demo.launch.xml rviz:="$RVIZ"
