#!/usr/bin/env python3
"""Autonomous Spot + Trimble X7 terrain-coverage mission. No ROS.

Loop:
  1. Take a reference scan where the robot starts; that pose anchors the
     map frame (Perspective registers later scans into the same frame).
  2. Build/extend the 2D coverage map from the scan.
  3. Walk to the farthest frontier (free cells bordering unscanned area).
  4. Scan there, merge, repeat until no frontier remains.

Spot walks with SE2 trajectory commands in its native vision/odom frame
(Spot's onboard perception and obstacle avoidance run underneath); the X7 is
triggered over HTTP through the Windows Perspective bridge, which drops
each completed LAS into a shared folder this script watches.

Example:
  python3 mission.py --spot-ip 192.168.80.3 \
      --bridge-url http://192.168.1.50:8765 \
      --scan-dir /mnt/trimble_scans

Test the full loop without any hardware:
  python3 mission.py --dry-run --scan-dir /tmp/fake_scans --bridge-url ''
"""
import argparse
import json
import math
import os
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from coverage_map import CoverageMap, read_las_xyz  # noqa: E402
from spot_robot import DryRunRobot, SpotRobot  # noqa: E402
from trimble import TrimbleClient  # noqa: E402


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def se2_compose(a, b):
    ax, ay, ayaw = a
    bx, by, byaw = b
    cos_yaw = math.cos(ayaw)
    sin_yaw = math.sin(ayaw)
    return (
        ax + cos_yaw * bx - sin_yaw * by,
        ay + sin_yaw * bx + cos_yaw * by,
        wrap_angle(ayaw + byaw),
    )


def se2_inverse(a):
    ax, ay, ayaw = a
    cos_yaw = math.cos(ayaw)
    sin_yaw = math.sin(ayaw)
    return (
        -(cos_yaw * ax + sin_yaw * ay),
        -(-sin_yaw * ax + cos_yaw * ay),
        wrap_angle(-ayaw),
    )


def rotate_points_xy(points, yaw):
    if yaw == 0.0:
        return points
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    rotated = points.copy()
    rotated[:, 0] = cos_yaw * points[:, 0] - sin_yaw * points[:, 1]
    rotated[:, 1] = sin_yaw * points[:, 0] + cos_yaw * points[:, 1]
    return rotated


class CoverageMission:

    def __init__(self, robot, trimble, coverage_map, args, log=print):
        self.robot = robot
        self.trimble = trimble
        self.map = coverage_map
        self.args = args
        self.log = log
        self.anchor = None  # world pose of the map origin (reference scan)
        self.stations = []  # map-frame (x, y, yaw) of every scan station
        self.failed_goals = []
        self.output_dir = Path(args.output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

    def world_to_map(self, pose):
        return se2_compose(se2_inverse(self.anchor), pose)

    def map_to_world(self, pose):
        return se2_compose(self.anchor, pose)

    def robot_map_pose(self):
        return self.world_to_map(self.robot.pose())

    def take_scan(self, reason):
        """Request an X7 scan at the current pose and merge it."""
        self.trimble.report_status('Scanning', reason)
        self.trimble.request_scan(reason)
        scan_path = self.trimble.wait_for_scan(
            timeout_sec=self.args.scan_timeout,
        )
        points = read_las_xyz(scan_path, max_points=self.args.max_points)
        points = rotate_points_xy(points, self.args.scan_yaw_offset)
        station = self.robot_map_pose()
        used = self.map.add_scan(points, station[:2])
        self.stations.append(
            {'x': station[0], 'y': station[1], 'yaw': station[2],
             'scan_file': scan_path.name, 'points_used': int(used)},
        )
        stats = self.map.coverage_stats()
        self.log(
            f'Scan {len(self.stations)} merged ({used} points); '
            f'covered area: {stats["covered_m2"]:.0f} m^2'
        )
        self.save_outputs()

    def save_outputs(self):
        self.map.save_pgm(self.output_dir / 'coverage_map.pgm')
        summary = {
            'anchor_world_pose': dict(
                zip(('x', 'y', 'yaw'), self.anchor or (0.0, 0.0, 0.0)),
            ),
            'frame': self.robot.frame,
            'stations': self.stations,
            'coverage': self.map.coverage_stats(),
        }
        with open(self.output_dir / 'mission.json', 'w',
                  encoding='utf-8') as stream:
            json.dump(summary, stream, indent=2)

    def station_too_close(self, map_xy):
        return any(
            math.hypot(map_xy[0] - s['x'], map_xy[1] - s['y'])
            < self.args.min_scan_separation
            for s in self.stations
        )

    def run(self):
        self.robot.connect()
        self.trimble.report_status(
            'Mission Start', f'coverage mission ({self.robot.frame} frame)',
        )

        # The reference scan pins the map frame to the starting pose.
        self.anchor = self.robot.pose()
        self.log(
            'Anchored map frame at world pose '
            f'x={self.anchor[0]:.2f}, y={self.anchor[1]:.2f}, '
            f'yaw={self.anchor[2]:.2f}'
        )
        self.take_scan('reference scan (station 1)')

        while len(self.stations) < self.args.max_stations:
            robot_map = self.robot_map_pose()
            frontier = self.map.select_frontier(
                robot_map[:2],
                min_distance_m=self.args.min_frontier_distance,
                excluded=self.failed_goals,
                exclusion_radius_m=self.args.goal_exclusion_radius,
            )
            if frontier is None:
                self.log('No frontier left: terrain coverage is complete.')
                self.trimble.report_status(
                    'Mission Complete', 'no unscanned frontier remains',
                )
                break

            if self.station_too_close(frontier):
                # Scanning there would duplicate an existing station.
                self.failed_goals.append(frontier)
                continue

            yaw = math.atan2(
                frontier[1] - robot_map[1],
                frontier[0] - robot_map[0],
            )
            goal_map = (frontier[0], frontier[1], yaw)
            goal_world = self.map_to_world(goal_map)
            self.log(
                f'Next frontier (map): x={goal_map[0]:.2f}, '
                f'y={goal_map[1]:.2f} '
                f'({math.hypot(goal_map[0] - robot_map[0], goal_map[1] - robot_map[1]):.1f}m away)'
            )
            self.trimble.report_status(
                'Navigating',
                f'to frontier x={goal_map[0]:.1f}, y={goal_map[1]:.1f}',
            )

            try:
                self.robot.goto(*goal_world)
            except Exception as error:
                self.log(f'Trajectory command failed: {error}')
                self.failed_goals.append(frontier)
                continue

            distance_error, _ = self.robot.arrival_error(*goal_world)
            if distance_error > self.args.arrival_tolerance:
                self.log(
                    f'Did not reach the frontier ({distance_error:.2f}m '
                    'short); excluding it and picking another'
                )
                self.failed_goals.append(frontier)
                continue

            station_map = self.robot_map_pose()
            if self.station_too_close(station_map[:2]):
                self.log(
                    'Arrived, but a previous station is within '
                    f'{self.args.min_scan_separation:.1f}m; skipping scan '
                    'and excluding this frontier'
                )
                self.failed_goals.append(frontier)
                continue

            self.take_scan(f'coverage station {len(self.stations) + 1}')
        else:
            self.log(f'Reached the station limit ({self.args.max_stations}).')
            self.trimble.report_status(
                'Mission Complete', 'station limit reached',
            )

        self.save_outputs()
        self.log(
            f'Mission done: {len(self.stations)} station(s), outputs in '
            f'{self.output_dir}'
        )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='Spot + Trimble X7 autonomous terrain coverage '
                    '(no ROS required)',
    )
    parser.add_argument('--spot-ip', default=os.environ.get('SPOT_IP', ''))
    parser.add_argument(
        '--spot-user', default=os.environ.get('SPOT_USERNAME'),
    )
    parser.add_argument(
        '--spot-pass', default=os.environ.get('SPOT_PASSWORD'),
    )
    parser.add_argument(
        '--frame', choices=('vision', 'odom'), default='vision',
        help='Spot localization frame; vision drifts least (default)',
    )
    parser.add_argument(
        '--bridge-url',
        default=os.environ.get('TRIMBLE_WINDOWS_URL', ''),
        help='Perspective bridge, e.g. http://192.168.1.50:8765 '
             '(empty: operator starts scans manually)',
    )
    parser.add_argument(
        '--scan-dir', required=True,
        help='Folder where completed LAS/LAZ scans appear (the bridge\'s '
             'delivery folder or the Perspective export folder)',
    )
    parser.add_argument('--output-dir', default='coverage_mission_output')
    parser.add_argument('--resolution', type=float, default=0.10)
    parser.add_argument('--max-range', type=float, default=80.0)
    parser.add_argument('--min-z', type=float, default=-0.25)
    parser.add_argument('--max-z', type=float, default=1.20)
    parser.add_argument('--max-points', type=int, default=500000)
    parser.add_argument(
        '--min-frontier-distance', type=float, default=2.0,
        help='Ignore frontiers closer than this to the robot (m)',
    )
    parser.add_argument(
        '--min-scan-separation', type=float, default=5.0,
        help='Never scan within this distance of a previous station (m)',
    )
    parser.add_argument('--arrival-tolerance', type=float, default=1.0)
    parser.add_argument('--goal-exclusion-radius', type=float, default=1.5)
    parser.add_argument('--max-stations', type=int, default=25)
    parser.add_argument(
        '--scan-timeout', type=float, default=900.0,
        help='Seconds to wait for each scan file (X7 scans take minutes)',
    )
    parser.add_argument(
        '--scan-yaw-offset', type=float, default=0.0,
        help='Fixed yaw (rad) between the X7 scan frame and the robot '
             'body at the reference scan; rotates ingested points',
    )
    parser.add_argument('--walk-speed', type=float, default=0.6)
    parser.add_argument(
        '--power-on', action='store_true',
        help='Power on Spot motors (otherwise the tablet operator does)',
    )
    parser.add_argument(
        '--yes', action='store_true',
        help='Skip the confirmation prompt before moving the robot',
    )
    parser.add_argument(
        '--dry-run', action='store_true',
        help='No robot: teleport between goals to test the scan/map loop',
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    if args.dry_run:
        robot = DryRunRobot()
    else:
        if not args.spot_ip:
            print('error: --spot-ip (or SPOT_IP) is required without '
                  '--dry-run', file=sys.stderr)
            return 2
        if not args.yes:
            answer = input(
                f'Spot at {args.spot_ip} will stand and walk autonomously. '
                'An external E-Stop (tablet) must be active. Continue? '
                '[y/N] '
            )
            if answer.strip().lower() not in {'y', 'yes'}:
                print('Aborted.')
                return 1
        robot = SpotRobot(
            args.spot_ip,
            username=args.spot_user,
            password=args.spot_pass,
            frame=args.frame,
            walk_speed_mps=args.walk_speed,
            power_on=args.power_on,
        )

    trimble = TrimbleClient(args.scan_dir, bridge_url=args.bridge_url)
    coverage_map = CoverageMap(
        resolution=args.resolution,
        min_z=args.min_z,
        max_z=args.max_z,
        max_range_m=args.max_range,
    )
    mission = CoverageMission(robot, trimble, coverage_map, args)

    try:
        mission.run()
    except KeyboardInterrupt:
        print('\nInterrupted; stopping the mission safely.')
        trimble.report_status('Mission Aborted', 'operator interrupt')
    except (RuntimeError, TimeoutError) as error:
        print(f'Mission error: {error}', file=sys.stderr)
        trimble.report_status('Mission Error', str(error))
        return 1
    finally:
        robot.shutdown()
        mission.save_outputs()
    return 0


if __name__ == '__main__':
    sys.exit(main())
