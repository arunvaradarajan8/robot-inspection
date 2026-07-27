import math
import time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


UNKNOWN = -1
FREE = 0
OCCUPIED = 100


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def box_fraction(mask, radius_cells):
    """Fraction of True cells in a square window around every cell.

    Uses an integral image so the whole map is scored in one pass, which
    keeps vantage selection cheap even on a large grid. The window is a
    (2*radius+1) square; near the map edge it shrinks to what fits.
    """
    height, width = mask.shape
    integral = np.zeros((height + 1, width + 1), dtype=np.int64)
    integral[1:, 1:] = np.cumsum(np.cumsum(mask.astype(np.int64), axis=0), axis=1)

    rows = np.arange(height)
    cols = np.arange(width)
    y0 = np.clip(rows - radius_cells, 0, height)
    y1 = np.clip(rows + radius_cells + 1, 0, height)
    x0 = np.clip(cols - radius_cells, 0, width)
    x1 = np.clip(cols + radius_cells + 1, 0, width)

    top = integral[y0][:, x1] - integral[y0][:, x0]
    bottom = integral[y1][:, x1] - integral[y1][:, x0]
    counts = bottom - top
    areas = (y1 - y0)[:, None] * (x1 - x0)[None, :]
    areas = np.maximum(areas, 1)
    return counts / areas


class ScanPlanner(Node):
    """Choose where to take the X7 scans once the map is built.

    This runs after exploration, when the occupancy map is as complete as
    it is going to get. It scores every free cell the robot could stand on
    by two cheap, map-only measures:

      openness  - how much free space surrounds the cell, so the tripod has
                  clearance and a broad line of sight;
      centrality- how close the cell is to the centre of the mapped
                  structure, so a single scan captures as much of it as
                  possible.

    The best few well-separated cells become the scan stations. The node
    drives to each in turn, triggers a scan, waits for it to finish, then
    reports that scanning is complete so the mission can walk home. No
    object detection is involved: the stations come only from the shape of
    the Oak-D map.
    """

    def __init__(self):
        super().__init__('scan_planner')

        self.declare_parameter('map_topic', '/digital_twin/map')
        self.declare_parameter('goal_topic', '/infrastructure/inspection_goal')
        self.declare_parameter('status_topic', '/infrastructure/planner_status')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('start_scanning_topic', '/mission/start_scanning')
        self.declare_parameter(
            'scanning_complete_topic',
            '/mission/scanning_complete',
        )
        self.declare_parameter(
            'waypoint_arrived_topic',
            '/digital_twin/waypoint_arrived',
        )
        self.declare_parameter('scan_required_topic', '/digital_twin/scan_required')
        self.declare_parameter('scan_reason_topic', '/digital_twin/scan_reason')
        self.declare_parameter('scan_complete_topic', '/digital_twin/scan_complete')
        # How many scan stations to pick, and how far apart they must be so
        # the scans cover different parts of the site.
        self.declare_parameter('max_scan_stations', 3)
        self.declare_parameter('min_scan_separation_m', 8.0)
        # A vantage must be at least this open (free fraction within the
        # openness radius) to be considered, so the robot never parks the
        # tripod in a tight spot.
        self.declare_parameter('openness_radius_m', 3.0)
        self.declare_parameter('min_openness', 0.6)
        # Falloff of the centrality term, in metres from the structure
        # centroid. Larger keeps far-but-open vantages competitive.
        self.declare_parameter('centrality_scale_m', 10.0)
        # Timeouts so a stuck drive or a scan that never reports back cannot
        # strand the mission in the scan phase.
        self.declare_parameter('leg_timeout_sec', 90.0)
        self.declare_parameter('scan_timeout_sec', 300.0)
        self.declare_parameter('scan_settle_sec', 2.0)

        self.map_topic = self.get_parameter(
            'map_topic'
        ).get_parameter_value().string_value
        goal_topic = self.get_parameter(
            'goal_topic'
        ).get_parameter_value().string_value
        status_topic = self.get_parameter(
            'status_topic'
        ).get_parameter_value().string_value
        self.target_frame = self.get_parameter(
            'target_frame'
        ).get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame'
        ).get_parameter_value().string_value
        start_scanning_topic = self.get_parameter(
            'start_scanning_topic'
        ).get_parameter_value().string_value
        scanning_complete_topic = self.get_parameter(
            'scanning_complete_topic'
        ).get_parameter_value().string_value
        waypoint_arrived_topic = self.get_parameter(
            'waypoint_arrived_topic'
        ).get_parameter_value().string_value
        scan_required_topic = self.get_parameter(
            'scan_required_topic'
        ).get_parameter_value().string_value
        scan_reason_topic = self.get_parameter(
            'scan_reason_topic'
        ).get_parameter_value().string_value
        scan_complete_topic = self.get_parameter(
            'scan_complete_topic'
        ).get_parameter_value().string_value
        self.max_scan_stations = self.get_parameter(
            'max_scan_stations'
        ).get_parameter_value().integer_value
        self.min_scan_separation = self.get_parameter(
            'min_scan_separation_m'
        ).get_parameter_value().double_value
        self.openness_radius = self.get_parameter(
            'openness_radius_m'
        ).get_parameter_value().double_value
        self.min_openness = self.get_parameter(
            'min_openness'
        ).get_parameter_value().double_value
        self.centrality_scale = self.get_parameter(
            'centrality_scale_m'
        ).get_parameter_value().double_value
        self.leg_timeout = self.get_parameter(
            'leg_timeout_sec'
        ).get_parameter_value().double_value
        self.scan_timeout = self.get_parameter(
            'scan_timeout_sec'
        ).get_parameter_value().double_value
        self.scan_settle = self.get_parameter(
            'scan_settle_sec'
        ).get_parameter_value().double_value

        if self.max_scan_stations <= 0:
            raise ValueError('max_scan_stations must be greater than zero')
        if self.centrality_scale <= 0.0:
            raise ValueError('centrality_scale_m must be greater than zero')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_publisher = self.create_publisher(PoseStamped, goal_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.scan_required_publisher = self.create_publisher(
            Bool,
            scan_required_topic,
            10,
        )
        self.scan_reason_publisher = self.create_publisher(
            String,
            scan_reason_topic,
            10,
        )
        self.scanning_complete_publisher = self.create_publisher(
            Bool,
            scanning_complete_topic,
            10,
        )

        self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            1,
        )
        self.create_subscription(
            Bool,
            start_scanning_topic,
            self.start_scanning_callback,
            10,
        )
        self.create_subscription(
            String,
            waypoint_arrived_topic,
            self.waypoint_arrived_callback,
            10,
        )
        self.create_subscription(
            Bool,
            scan_complete_topic,
            self.scan_complete_callback,
            10,
        )

        self.latest_map = None
        # None until the mission asks us to scan. Then a list of remaining
        # (x, y, yaw) stations; empty list means scanning finished.
        self.stations = None
        # 'driving' (heading to the station) or 'scanning' (parked, waiting
        # for the X7 to finish).
        self.phase = None
        self.phase_since = 0.0
        self.arrived = False
        self.finished = False

        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(
            'Scan planner ready; waiting for the mission to finish exploring. '
            f'stations<={self.max_scan_stations}, '
            f'separation>={self.min_scan_separation:.1f}m'
        )

    # ---- inputs -------------------------------------------------------

    def map_callback(self, grid):
        self.latest_map = grid

    def start_scanning_callback(self, message):
        if not message.data or self.stations is not None or self.finished:
            return
        if self.latest_map is None:
            self.publish_status('cannot pick scan stations yet: no map received')
            return
        self.stations = self.select_scan_stations(self.latest_map)
        if not self.stations:
            self.publish_status(
                'no suitable scan vantage found; scanning phase is a no-op'
            )
            self.report_complete()
            return
        self.publish_status(
            f'scan phase: {len(self.stations)} station(s) chosen from the map'
        )
        self.begin_next_leg()

    def waypoint_arrived_callback(self, _message):
        if self.stations is not None and self.phase == 'driving':
            self.arrived = True

    def scan_complete_callback(self, message):
        if message.data and self.phase == 'scanning':
            self.publish_status('scan finished; moving to the next station')
            self.stations.pop(0)
            self.begin_next_leg()

    # ---- station scoring ----------------------------------------------

    def select_scan_stations(self, grid):
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        if width == 0 or height == 0:
            return []

        data = np.asarray(grid.data, dtype=np.int16).reshape((height, width))
        free = data == FREE
        occupied = data == OCCUPIED
        if not free.any() or not occupied.any():
            return []

        radius_cells = max(1, int(round(self.openness_radius / resolution)))
        openness = box_fraction(free, radius_cells)

        occ_ys, occ_xs = np.nonzero(occupied)
        structure_x = origin_x + (occ_xs.mean() + 0.5) * resolution
        structure_y = origin_y + (occ_ys.mean() + 0.5) * resolution

        candidate = free & (openness >= self.min_openness)
        cand_ys, cand_xs = np.nonzero(candidate)
        if cand_ys.size == 0:
            return []

        world_x = origin_x + (cand_xs + 0.5) * resolution
        world_y = origin_y + (cand_ys + 0.5) * resolution
        dist = np.hypot(world_x - structure_x, world_y - structure_y)
        centrality = np.exp(-dist / self.centrality_scale)
        scores = openness[cand_ys, cand_xs] * centrality

        order = np.argsort(scores)[::-1]
        yaw_to_structure = np.arctan2(
            structure_y - world_y,
            structure_x - world_x,
        )

        stations = []
        for index in order:
            x = float(world_x[index])
            y = float(world_y[index])
            if any(
                math.hypot(x - sx, y - sy) < self.min_scan_separation
                for sx, sy, _ in stations
            ):
                continue
            stations.append((x, y, float(yaw_to_structure[index])))
            if len(stations) >= self.max_scan_stations:
                break
        return stations

    # ---- driving and scanning -----------------------------------------

    def begin_next_leg(self):
        if not self.stations:
            self.report_complete()
            return
        self.phase = 'driving'
        self.phase_since = time.monotonic()
        self.arrived = False
        self.publish_goal(self.stations[0])

    def tick(self):
        if self.stations is None or self.finished:
            return
        now = time.monotonic()
        if self.phase == 'driving':
            if self.arrived or self.at_station(self.stations[0]):
                self.trigger_scan()
            elif now - self.phase_since > self.leg_timeout:
                self.publish_status(
                    'drive to scan station timed out; skipping it'
                )
                self.stations.pop(0)
                self.begin_next_leg()
        elif self.phase == 'scanning':
            if now - self.phase_since > self.scan_timeout:
                self.publish_status(
                    'scan station timed out waiting for the X7; skipping it'
                )
                self.stations.pop(0)
                self.begin_next_leg()

    def trigger_scan(self):
        if self.phase == 'scanning':
            return
        self.phase = 'scanning'
        self.phase_since = time.monotonic()
        x, y, _ = self.stations[0]
        reason = String()
        reason.data = (
            f'X7 scan at chosen vantage x={x:.2f}, y={y:.2f} '
            f'({len(self.stations)} station(s) remaining)'
        )
        self.scan_reason_publisher.publish(reason)
        self.scan_required_publisher.publish(Bool(data=True))
        self.publish_status(reason.data)

    def report_complete(self):
        self.finished = True
        self.stations = []
        self.phase = None
        self.scanning_complete_publisher.publish(Bool(data=True))
        self.publish_status('scan phase complete')

    def at_station(self, station):
        pose = self.robot_position()
        if pose is None:
            return False
        return math.hypot(station[0] - pose[0], station[1] - pose[1]) <= 1.0

    def robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        return translation.x, translation.y

    def publish_goal(self, station):
        goal = PoseStamped()
        goal.header.frame_id = self.target_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = station[0]
        goal.pose.position.y = station[1]
        (
            goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w,
        ) = quaternion_from_yaw(station[2])
        self.goal_publisher.publish(goal)
        self.publish_status(
            f'driving to scan station x={station[0]:.2f}, y={station[1]:.2f}'
        )

    def publish_status(self, message):
        status = String()
        status.data = message
        self.status_publisher.publish(status)
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = ScanPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
