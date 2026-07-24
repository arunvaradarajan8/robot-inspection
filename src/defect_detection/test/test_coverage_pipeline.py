import os
import time
from types import SimpleNamespace

import numpy as np
import rclpy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header, String

from defect_detection.digital_twin.infrastructure_planner import (
    InfrastructurePlanner,
)
from defect_detection.digital_twin.mission_manager import (
    AWAITING_UPLOAD,
    EXPLORING,
    RETURNING,
    MissionManager,
)
from defect_detection.digital_twin.pointcloud_to_occupancy import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    PointCloudToOccupancy,
)
from defect_detection.digital_twin.scan_decision_node import ScanDecisionNode


def make_cloud(points):
    header = Header()
    header.frame_id = 'map'
    return point_cloud2.create_cloud_xyz32(
        header,
        np.asarray(points, dtype=np.float32),
    )


def grid_value_at(message, x, y):
    info = message.info
    cell_x = int((x - info.origin.position.x) / info.resolution)
    cell_y = int((y - info.origin.position.y) / info.resolution)
    return message.data[cell_y * info.width + cell_x]


def source_named(node, name):
    return next(source for source in node.sources if source.name == name)


def test_coverage_scan_decision_waits_for_waypoint_arrivals():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = ScanDecisionNode()
    try:
        node.mode = 'coverage'
        node.request_initial_scan = True

        should_scan, reason = node.scan_decision(100.0)
        assert should_scan
        assert 'initial coverage reference scan' in reason

        node.last_scan_request_time = 100.0
        should_scan, reason = node.scan_decision(200.0)
        assert not should_scan
        assert 'waypoint arrival' in reason

        # Arrival during the cooldown is retained, not dropped.
        node.arrival_pending = True
        should_scan, reason = node.scan_decision(110.0)
        assert not should_scan
        assert 'cooldown' in reason
        assert node.arrival_pending

        should_scan, reason = node.scan_decision(200.0)
        assert should_scan
        assert 'coverage waypoint reached' in reason
        assert not node.arrival_pending
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_coverage_scan_decision_consumes_arrival_when_too_close():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = ScanDecisionNode()
    try:
        node.mode = 'coverage'
        node.last_scan_request_time = 100.0
        node.scan_positions = [(0.0, 0.0)]
        node.arrival_pending = True

        should_scan, reason = node.scan_decision(300.0, 1.0, 0.0)
        assert not should_scan
        assert 'too close' in reason
        assert not node.arrival_pending

        node.arrival_pending = True
        should_scan, _ = node.scan_decision(300.0, 50.0, 0.0)
        assert should_scan
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_occupancy_accumulation_keeps_walls_from_earlier_stations():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = PointCloudToOccupancy()
    try:
        node.use_tf_scan_origin = False
        published = []
        node.publisher = SimpleNamespace(publish=published.append)
        cloud = source_named(node, 'cloud')

        # Station 1 at the configured origin sees a wall at x=2.
        wall_one = [(2.0, y / 10.0, 0.5) for y in range(-10, 11)]
        node.cloud_callback(cloud, make_cloud(wall_one))
        assert len(published) == 1
        assert grid_value_at(published[0], 2.0, 0.0) == OCCUPIED

        # Station 2 further along sees a different wall at x=8.
        node.scan_origin_x = 6.0
        cloud.last_update = 0.0
        wall_two = [(8.0, y / 10.0, 0.5) for y in range(-10, 11)]
        node.cloud_callback(cloud, make_cloud(wall_two))
        assert len(published) == 2

        merged = published[1]
        assert grid_value_at(merged, 8.0, 0.0) == OCCUPIED
        # The wall from station 1 stays on the accumulated map even
        # though station 2 never saw it.
        assert grid_value_at(merged, 2.0, 0.0) == OCCUPIED
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_depth_rays_do_not_erase_cloud_walls():
    """A near-field depth ray must not punch through a wall the cloud source saw."""
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = PointCloudToOccupancy()
    try:
        node.use_tf_scan_origin = False
        published = []
        node.publisher = SimpleNamespace(publish=published.append)
        cloud = source_named(node, 'cloud')
        depth = source_named(node, 'depth')

        # The long-range cloud source sees a wall at x=2.
        node.cloud_callback(
            cloud,
            make_cloud([(2.0, y / 10.0, 0.5) for y in range(-10, 11)]),
        )
        assert grid_value_at(published[-1], 2.0, 0.0) == OCCUPIED

        # The depth camera reports a return past that wall, so its ray
        # crosses the wall cell on the way out.
        node.cloud_callback(
            depth,
            make_cloud([(4.0, y / 10.0, 0.5) for y in range(-3, 4)]),
        )
        merged = published[-1]
        assert grid_value_at(merged, 2.0, 0.0) == OCCUPIED
        assert grid_value_at(merged, 4.0, 0.0) == OCCUPIED
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_either_sensor_clears_unknown_so_it_stops_being_a_frontier():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = PointCloudToOccupancy()
    try:
        node.use_tf_scan_origin = False
        published = []
        node.publisher = SimpleNamespace(publish=published.append)
        depth = source_named(node, 'depth')

        # Only the depth camera observes this patch; the cloud source never does.
        node.cloud_callback(
            depth,
            make_cloud([(3.0, y / 10.0, 0.5) for y in range(-5, 6)]),
        )
        merged = published[-1]
        # Cells between the origin and the return are free, not unknown,
        # so the frontier planner will not treat them as unexplored.
        assert grid_value_at(merged, 1.5, 0.0) == FREE
        assert grid_value_at(merged, 1.5, 0.0) != UNKNOWN
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_decimation_respects_the_per_source_budget():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = PointCloudToOccupancy()
    try:
        cloud = source_named(node, 'cloud')
        cloud.max_points = 50
        points = np.column_stack(
            [
                np.linspace(1.0, 20.0, 5000),
                np.zeros(5000),
                np.full(5000, 0.5),
            ]
        )
        kept = node.filter_points(cloud, points, 0.0, 0.0)
        assert len(kept) <= cloud.max_points
        # A uniform stride keeps both ends of the scan, unlike truncation.
        assert kept[0][0] < 2.0
        assert kept[-1][0] > 18.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_planner_holds_until_the_scanner_reports_it_finished():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = InfrastructurePlanner()
    try:
        assert not node.scan_hold_active(time.monotonic())

        node.waypoint_arrived_callback(String())
        assert node.hold_state == 'decision'
        assert node.scan_hold_active(time.monotonic())

        scan_required = Bool()
        scan_required.data = True
        node.scan_required_callback(scan_required)
        assert node.hold_state == 'scan'

        # Nothing comes back from the X7 over ROS; the Windows bridge
        # reports completion instead, and that is what releases the hold.
        node.scan_complete_callback(Bool(data=True))
        assert node.hold_state is None

        # A declined scan decision releases the hold after settling.
        node.waypoint_arrived_callback(String())
        declined = Bool()
        declined.data = False
        node.scan_required_callback(declined)
        assert node.hold_state == 'decision'
        node.hold_since -= node.scan_decision_settle + 1.0
        node.scan_required_callback(declined)
        assert node.hold_state is None

        # A scan that never completes cannot park the robot forever.
        node.hold_state = 'scan'
        node.hold_since = time.monotonic() - node.scan_wait_timeout - 1.0
        assert not node.scan_hold_active(time.monotonic())
        assert node.hold_state is None
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_planner_stops_exploring_when_the_mission_manager_says_so():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = InfrastructurePlanner()
    try:
        assert node.allow_exploration

        node.allow_exploration_callback(Bool(data=False))
        assert not node.allow_exploration
        # plan_tick must return before touching TF or the map.
        node.latest_map = None
        node.plan_tick()

        node.allow_exploration_callback(Bool(data=True))
        assert node.allow_exploration
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_mission_manager_retraces_breadcrumbs_in_reverse():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = MissionManager()
    try:
        node.breadcrumb_min_spacing = 1.0
        node.start_pose = (0.0, 0.0, 0.0)
        for station in [(3.0, 0.0, 0.0), (6.0, 0.0, 0.0), (9.0, 0.0, 0.0)]:
            node.record_station(station)
        assert len(node.breadcrumbs) == 3

        node.begin_return('test')
        assert node.state == RETURNING
        # Newest station first, start pose last.
        assert [round(x, 1) for x, _, _ in node.return_queue] == [
            9.0,
            6.0,
            3.0,
            0.0,
        ]
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_mission_manager_ends_exploration_on_excursion_limit():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = MissionManager()
    try:
        node.start_pose = (0.0, 0.0, 0.0)
        node.max_excursion = 10.0
        node.max_stations = 0
        node.mission_duration = 0.0

        assert node.end_condition((5.0, 0.0, 0.0)) is None
        reason = node.end_condition((11.0, 0.0, 0.0))
        assert reason is not None
        assert 'excursion limit' in reason
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_mission_manager_waits_for_the_e57_upload_before_completing():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = MissionManager()
    try:
        node.summary_path = node.summary_path.with_name('test_mission.yaml')
        assert node.state == EXPLORING

        node.start_pose = (0.0, 0.0, 0.0)
        node.begin_return('test')
        node.return_queue = []
        node.return_tick()
        # Home, but the mission stays open until the scanner's E57 lands.
        assert node.state == AWAITING_UPLOAD

        node.upload_complete_callback(Bool(data=True))
        assert node.state == 'COMPLETE'
    finally:
        node.destroy_node()
        rclpy.shutdown()
