import math
import os
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import OccupancyGrid
import rclpy
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header

from defect_detection.digital_twin.infrastructure_planner import (
    InfrastructurePlanner,
)
from defect_detection.digital_twin.mission_manager import (
    AWAITING_UPLOAD,
    EXPLORING,
    RETURNING,
    SCANNING,
    MissionManager,
)
from defect_detection.digital_twin.pointcloud_to_occupancy import (
    FREE,
    OCCUPIED,
    UNKNOWN,
    PointCloudToOccupancy,
)
from defect_detection.digital_twin.scan_planner import ScanPlanner


def make_cloud(points):
    header = Header()
    header.frame_id = 'map'
    return point_cloud2.create_cloud_xyz32(
        header,
        np.asarray(points, dtype=np.float32),
    )


def make_grid(array, resolution=1.0, origin=(0.0, 0.0)):
    grid = OccupancyGrid()
    grid.header.frame_id = 'map'
    height, width = array.shape
    grid.info.width = width
    grid.info.height = height
    grid.info.resolution = resolution
    grid.info.origin.position.x = float(origin[0])
    grid.info.origin.position.y = float(origin[1])
    grid.info.origin.orientation.w = 1.0
    grid.data = array.reshape(-1).astype(int).tolist()
    return grid


def grid_value_at(message, x, y):
    info = message.info
    cell_x = int((x - info.origin.position.x) / info.resolution)
    cell_y = int((y - info.origin.position.y) / info.resolution)
    return message.data[cell_y * info.width + cell_x]


def source_named(node, name):
    return next(source for source in node.sources if source.name == name)


# ---- occupancy grid ----------------------------------------------------


def test_occupancy_accumulation_keeps_walls_from_earlier_stations():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = PointCloudToOccupancy()
    try:
        node.use_tf_scan_origin = False
        published = []
        node.publisher = SimpleNamespace(publish=published.append)
        cloud = source_named(node, 'cloud')

        wall_one = [(2.0, y / 10.0, 0.5) for y in range(-10, 11)]
        node.cloud_callback(cloud, make_cloud(wall_one))
        assert len(published) == 1
        assert grid_value_at(published[0], 2.0, 0.0) == OCCUPIED

        node.scan_origin_x = 6.0
        cloud.last_update = 0.0
        wall_two = [(8.0, y / 10.0, 0.5) for y in range(-10, 11)]
        node.cloud_callback(cloud, make_cloud(wall_two))
        assert len(published) == 2

        merged = published[1]
        assert grid_value_at(merged, 8.0, 0.0) == OCCUPIED
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

        node.cloud_callback(
            cloud,
            make_cloud([(2.0, y / 10.0, 0.5) for y in range(-10, 11)]),
        )
        assert grid_value_at(published[-1], 2.0, 0.0) == OCCUPIED

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

        node.cloud_callback(
            depth,
            make_cloud([(3.0, y / 10.0, 0.5) for y in range(-5, 6)]),
        )
        merged = published[-1]
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
        assert kept[0][0] < 2.0
        assert kept[-1][0] > 18.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---- frontier exploration ----------------------------------------------


def two_cluster_grid():
    """A map with a near frontier column (x=10) and a far one (x=25)."""
    width, height = 30, 7
    data = np.full((height, width), UNKNOWN, dtype=int)
    data[:, 0:11] = FREE   # known free region; x=10 borders the unknown east
    data[:, 25] = FREE     # a far free column, unknown on both sides
    return make_grid(data)


def test_frontier_prefers_the_near_cluster_over_a_far_one():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = InfrastructurePlanner()
    try:
        node.exploration_radius = 100.0
        node.min_frontier_cluster_cells = 4
        node.home = (5.0, 3.0)
        goal = node.select_frontier(two_cluster_grid(), 5.0, 3.0)
        assert goal is not None
        # Both clusters hold seven cells, so the closer one wins on utility.
        assert goal[0] < 15.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_frontier_respects_the_exploration_radius():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = InfrastructurePlanner()
    try:
        node.min_frontier_cluster_cells = 4
        node.home = (27.0, 3.0)
        # From a home by the far column, a tight radius hides the near one.
        node.exploration_radius = 6.0
        goal = node.select_frontier(two_cluster_grid(), 27.0, 3.0)
        assert goal is not None
        assert goal[0] > 20.0
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_frontier_returns_none_on_a_fully_known_map():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = InfrastructurePlanner()
    try:
        node.home = (1.0, 1.0)
        data = np.full((5, 5), FREE, dtype=int)
        assert node.select_frontier(make_grid(data), 1.0, 1.0) is None
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


# ---- scan vantage selection --------------------------------------------


def test_scan_stations_are_open_central_and_separated():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = ScanPlanner()
    try:
        node.max_scan_stations = 2
        node.min_scan_separation = 5.0
        node.min_openness = 0.4
        node.openness_radius = 1.5
        node.centrality_scale = 8.0

        width, height = 24, 24
        data = np.full((height, width), FREE, dtype=int)
        data[11:13, 11:13] = OCCUPIED  # a compact structure to inspect
        grid = make_grid(data)

        stations = node.select_scan_stations(grid)
        assert 1 <= len(stations) <= 2
        for x, y, _ in stations:
            # Stations sit on free space, never on the structure itself.
            assert grid_value_at(grid, x, y) == FREE
        if len(stations) == 2:
            (x0, y0, _), (x1, y1, _) = stations
            assert math.hypot(x0 - x1, y0 - y1) >= node.min_scan_separation
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_scan_stations_empty_without_structure():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = ScanPlanner()
    try:
        data = np.full((10, 10), FREE, dtype=int)
        assert node.select_scan_stations(make_grid(data)) == []
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ---- mission lifecycle -------------------------------------------------


def test_mission_manager_scans_before_returning_home():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = MissionManager()
    try:
        node.scan_phase_enabled = True
        node.start_pose = (0.0, 0.0, 0.0)

        node.begin_scanning('excursion limit reached')
        assert node.state == SCANNING

        node.scanning_complete_callback(Bool(data=True))
        assert node.state == RETURNING
        # The wrap-up reason is why exploration ended, not the hand-off.
        assert node.end_reason == 'excursion limit reached'
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
        assert node.state == AWAITING_UPLOAD

        node.upload_complete_callback(Bool(data=True))
        assert node.state == 'COMPLETE'
    finally:
        node.destroy_node()
        rclpy.shutdown()
