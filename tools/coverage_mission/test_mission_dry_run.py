#!/usr/bin/env python3
"""End-to-end dry run of the coverage mission with a simulated X7.

No hardware, no bosdyn: the robot teleports between goals and every
"scan" is a synthetic LAS file of a walled 30x20m site, limited to the
scanner range around the current station. Verifies that the mission
anchors, explores frontiers, respects station separation, accumulates
the map, and terminates.

  python3 test_mission_dry_run.py
"""
import math
import sys
import tempfile
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

from coverage_map import CoverageMap  # noqa: E402
from mission import CoverageMission, parse_args  # noqa: E402
from spot_robot import DryRunRobot  # noqa: E402
from trimble import TrimbleClient  # noqa: E402

# Rectangular site: walls at x=+-15, y=+-10, robot starts at the center.
WALL_X = 15.0
WALL_Y = 10.0
POINT_SPACING = 0.05
WALL_HEIGHTS = (0.3, 0.6, 0.9)


def wall_points():
    points = []
    for x in np.arange(-WALL_X, WALL_X + POINT_SPACING, POINT_SPACING):
        for z in WALL_HEIGHTS:
            points.append((x, -WALL_Y, z))
            points.append((x, WALL_Y, z))
    for y in np.arange(-WALL_Y, WALL_Y + POINT_SPACING, POINT_SPACING):
        for z in WALL_HEIGHTS:
            points.append((-WALL_X, y, z))
            points.append((WALL_X, y, z))
    return np.asarray(points, dtype=np.float64)


WORLD = wall_points()


def write_las(points, path):
    import laspy

    header = laspy.LasHeader(point_format=3, version='1.2')
    header.scales = (0.001, 0.001, 0.001)
    las = laspy.LasData(header)
    las.x = points[:, 0]
    las.y = points[:, 1]
    las.z = points[:, 2]
    las.write(str(path))


class SimulatedX7(TrimbleClient):
    """Writes a range-limited synthetic scan whenever one is requested."""

    def __init__(self, scan_dir, robot, scan_range=12.0):
        super().__init__(scan_dir, bridge_url='', stable_age_sec=0.0,
                         poll_period_sec=0.1, log=lambda *_: None)
        self.robot = robot
        self.scan_range = scan_range
        self.scan_count = 0

    def request_scan(self, reason, extra=None):
        x, y, _ = self.robot.pose()  # anchor is the origin in dry run
        distances = np.hypot(WORLD[:, 0] - x, WORLD[:, 1] - y)
        visible = WORLD[distances <= self.scan_range]
        self.scan_count += 1
        path = self.scan_dir / f'sim_scan_{self.scan_count:03d}.las'
        write_las(visible, path)
        print(f'[sim x7] {reason}: wrote {len(visible)} points')


def main():
    start = time.monotonic()
    with tempfile.TemporaryDirectory(prefix='coverage_mission_') as tmp:
        scan_dir = Path(tmp) / 'scans'
        output_dir = Path(tmp) / 'output'
        args = parse_args([
            '--dry-run',
            '--scan-dir', str(scan_dir),
            '--output-dir', str(output_dir),
            '--max-range', '12.0',
            '--min-scan-separation', '5.0',
            '--min-frontier-distance', '2.0',
            '--max-stations', '12',
            '--scan-timeout', '30',
        ])
        robot = DryRunRobot()
        trimble = SimulatedX7(scan_dir, robot, scan_range=12.0)
        coverage_map = CoverageMap(
            resolution=args.resolution,
            min_z=args.min_z,
            max_z=args.max_z,
            max_range_m=args.max_range,
        )
        mission = CoverageMission(robot, trimble, coverage_map, args)
        mission.run()

        stations = mission.stations
        stats = coverage_map.coverage_stats()
        site_area = (2 * WALL_X) * (2 * WALL_Y)
        checks = [
            ('anchored at origin', mission.anchor == (0.0, 0.0, 0.0)),
            ('multiple stations scanned', len(stations) >= 3),
            ('stations respect separation', all(
                math.hypot(a['x'] - b['x'], a['y'] - b['y']) >= 5.0
                for i, a in enumerate(stations)
                for b in stations[i + 1:]
            )),
            ('covered most of the site',
             stats['covered_m2'] >= 0.9 * site_area),
            ('map image written',
             (output_dir / 'coverage_map.pgm').is_file()),
            ('mission summary written',
             (output_dir / 'mission.json').is_file()),
        ]
        print()
        failures = 0
        for label, passed in checks:
            print(f'{"PASS" if passed else "FAIL"}: {label}')
            failures += 0 if passed else 1
        print(
            f'\n{len(stations)} stations, {stats["covered_m2"]:.0f} m^2 '
            f'covered of {site_area:.0f} m^2 site, '
            f'{time.monotonic() - start:.1f}s'
        )
        return 1 if failures else 0


if __name__ == '__main__':
    sys.exit(main())
