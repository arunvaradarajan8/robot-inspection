import math
import time

from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


UNKNOWN = -1
FREE = 0
OCCUPIED = 100


def bresenham(x0, y0, x1, y1):
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    error = dx + dy
    x = x0
    y = y0

    while True:
        yield x, y
        if x == x1 and y == y1:
            break
        doubled = 2 * error
        if doubled >= dy:
            error += dy
            x += sx
        if doubled <= dx:
            error += dx
            y += sy


class CloudSource:
    """One sensor feeding the shared occupancy grid.

    A long-range cloud source and the depth camera see the site very
    differently, so each keeps its own height band, trusted range, and
    decimation budget.
    """

    def __init__(
        self,
        name,
        topic,
        min_z,
        max_z,
        max_range,
        max_points,
        sensor_frame,
        update_period,
    ):
        self.name = name
        self.topic = topic
        self.min_z = min_z
        self.max_z = max_z
        self.max_range = max_range
        self.max_points = max_points
        self.sensor_frame = sensor_frame
        self.update_period = update_period
        self.last_update = 0.0

    def due(self, now):
        return now - self.last_update >= self.update_period


class PointCloudToOccupancy(Node):
    """Fuse every observing sensor into one accumulated occupancy grid.

    The depth camera is the field sensor; an optional long-range cloud
    source (used by the simulator and demo) can be fused alongside it. A
    cell only stays unknown when no sensor has seen it, so the frontier
    planner stops chasing blind spots instead of terrain.
    """

    def __init__(self):
        super().__init__('pointcloud_to_occupancy')

        self.declare_parameter('map_topic', '/digital_twin/map')
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('padding_m', 2.0)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'body')
        # Merge every observation into one persistent grid. Turning this
        # off makes each cloud produce a standalone map, which is only
        # useful for one-shot debugging.
        self.declare_parameter('accumulate', True)
        # Raytrace from each sensor's own pose rather than a fixed origin.
        self.declare_parameter('use_tf_scan_origin', True)
        self.declare_parameter('scan_origin_x', 0.0)
        self.declare_parameter('scan_origin_y', 0.0)

        # Optional long-range cloud source: sparse, sees walls far away.
        # The launch layer turns this off in the field (the depth camera is
        # the only sensor) and on for the simulator and demo, which feed
        # their world cloud in through it.
        self.declare_parameter('cloud_enabled', True)
        self.declare_parameter('cloud_topic', '/cloud/points')
        self.declare_parameter('cloud_min_z', -0.25)
        self.declare_parameter('cloud_max_z', 1.20)
        self.declare_parameter('cloud_max_range_m', 40.0)
        self.declare_parameter('cloud_max_points_per_update', 40000)
        self.declare_parameter('cloud_sensor_frame', '')
        self.declare_parameter('cloud_update_period_sec', 0.5)

        # Depth camera: short range, dense. The field map's backbone.
        self.declare_parameter('depth_enabled', True)
        self.declare_parameter('depth_topic', '/depth/points')
        self.declare_parameter('depth_min_z', -0.25)
        self.declare_parameter('depth_max_z', 1.20)
        # Beyond a few metres stereo depth is too noisy to write free
        # space with, so returns past this range are dropped.
        self.declare_parameter('depth_max_range_m', 5.0)
        self.declare_parameter('depth_max_points_per_update', 20000)
        self.declare_parameter('depth_sensor_frame', '')
        self.declare_parameter('depth_update_period_sec', 0.5)

        map_topic = self.get_parameter(
            'map_topic'
        ).get_parameter_value().string_value
        self.resolution = self.get_parameter(
            'resolution'
        ).get_parameter_value().double_value
        self.padding = self.get_parameter(
            'padding_m'
        ).get_parameter_value().double_value
        self.target_frame = self.get_parameter(
            'target_frame'
        ).get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame'
        ).get_parameter_value().string_value
        self.accumulate = self.get_parameter(
            'accumulate'
        ).get_parameter_value().bool_value
        self.use_tf_scan_origin = self.get_parameter(
            'use_tf_scan_origin'
        ).get_parameter_value().bool_value
        self.scan_origin_x = self.get_parameter(
            'scan_origin_x'
        ).get_parameter_value().double_value
        self.scan_origin_y = self.get_parameter(
            'scan_origin_y'
        ).get_parameter_value().double_value

        if self.resolution <= 0.0:
            raise ValueError('resolution must be greater than zero')

        self.tf_buffer = None
        self.tf_listener = None
        if self.use_tf_scan_origin:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        # The accumulated grid: (min_x, min_y, numpy array) or None.
        self.grid = None
        self.grid_min_x = 0.0
        self.grid_min_y = 0.0

        self.publisher = self.create_publisher(OccupancyGrid, map_topic, 1)
        self.sources = []
        for name in ('cloud', 'depth'):
            source = self.build_source(name)
            if source is None:
                continue
            self.sources.append(source)
            self.create_subscription(
                PointCloud2,
                source.topic,
                self.make_callback(source),
                qos_profile_sensor_data,
            )

        if not self.sources:
            self.get_logger().warning(
                'No cloud sources are enabled; the occupancy map will stay empty'
            )
        self.get_logger().info(
            'Building occupancy grid '
            f'{map_topic} from: '
            + ', '.join(
                f'{source.name} ({source.topic}, <={source.max_range:.0f}m)'
                for source in self.sources
            )
        )

    def build_source(self, name):
        def value(suffix):
            return self.get_parameter(f'{name}_{suffix}').get_parameter_value()

        if not value('enabled').bool_value:
            return None
        max_z = value('max_z').double_value
        min_z = value('min_z').double_value
        if max_z <= min_z:
            raise ValueError(f'{name}_max_z must be greater than {name}_min_z')
        max_points = value('max_points_per_update').integer_value
        if max_points <= 0:
            raise ValueError(
                f'{name}_max_points_per_update must be greater than zero'
            )
        return CloudSource(
            name=name,
            topic=value('topic').string_value,
            min_z=min_z,
            max_z=max_z,
            max_range=value('max_range_m').double_value,
            max_points=max_points,
            # An empty sensor frame means "raytrace from the robot body",
            # which is right for a sensor mounted over the body centre.
            sensor_frame=value('sensor_frame').string_value or self.base_frame,
            update_period=value('update_period_sec').double_value,
        )

    def make_callback(self, source):
        def callback(cloud):
            self.cloud_callback(source, cloud)
        return callback

    # ---- grid management ----------------------------------------------

    def snap_down(self, value):
        return math.floor(value / self.resolution) * self.resolution

    def snap_up(self, value):
        return math.ceil(value / self.resolution) * self.resolution

    def ensure_bounds(self, min_x, min_y, max_x, max_y):
        """Grow the accumulated grid so the given extent fits inside it."""
        min_x = self.snap_down(min_x - self.padding)
        min_y = self.snap_down(min_y - self.padding)
        max_x = self.snap_up(max_x + self.padding)
        max_y = self.snap_up(max_y + self.padding)

        if self.grid is None:
            width = max(1, int(round((max_x - min_x) / self.resolution)))
            height = max(1, int(round((max_y - min_y) / self.resolution)))
            self.grid = np.full((height, width), UNKNOWN, dtype=np.int8)
            self.grid_min_x = min_x
            self.grid_min_y = min_y
            return

        height, width = self.grid.shape
        current_max_x = self.grid_min_x + width * self.resolution
        current_max_y = self.grid_min_y + height * self.resolution
        new_min_x = min(min_x, self.grid_min_x)
        new_min_y = min(min_y, self.grid_min_y)
        new_max_x = max(max_x, current_max_x)
        new_max_y = max(max_y, current_max_y)

        if (
            new_min_x == self.grid_min_x
            and new_min_y == self.grid_min_y
            and new_max_x == current_max_x
            and new_max_y == current_max_y
        ):
            # Common case once the site is mapped: reuse the grid in place
            # instead of reallocating it on every update.
            return

        new_width = max(1, int(round((new_max_x - new_min_x) / self.resolution)))
        new_height = max(1, int(round((new_max_y - new_min_y) / self.resolution)))
        grown = np.full((new_height, new_width), UNKNOWN, dtype=np.int8)
        offset_x = int(round((self.grid_min_x - new_min_x) / self.resolution))
        offset_y = int(round((self.grid_min_y - new_min_y) / self.resolution))
        grown[
            offset_y:offset_y + height,
            offset_x:offset_x + width,
        ] = self.grid
        self.grid = grown
        self.grid_min_x = new_min_x
        self.grid_min_y = new_min_y

    def world_to_cell(self, x, y):
        return (
            int(math.floor((x - self.grid_min_x) / self.resolution)),
            int(math.floor((y - self.grid_min_y) / self.resolution)),
        )

    def cell_in_bounds(self, cell):
        height, width = self.grid.shape
        return 0 <= cell[0] < width and 0 <= cell[1] < height

    # ---- integration ---------------------------------------------------

    def scan_origin(self, source):
        if not self.use_tf_scan_origin:
            return self.scan_origin_x, self.scan_origin_y
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                source.sensor_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as error:
            self.get_logger().warning(
                f'Cannot locate {source.sensor_frame} in {self.target_frame} '
                f'for the {source.name} raytrace origin: {error}',
                throttle_duration_sec=10.0,
            )
            return None
        translation = transform.transform.translation
        return translation.x, translation.y

    def filter_points(self, source, points, origin_x, origin_y):
        z_mask = (points[:, 2] >= source.min_z) & (points[:, 2] <= source.max_z)
        distance = np.hypot(points[:, 0] - origin_x, points[:, 1] - origin_y)
        points = points[z_mask & (distance <= source.max_range)]
        if len(points) <= source.max_points:
            return points
        # Uniform stride preserves the spatial spread of the scan; taking
        # the first N returns would map only one slice of the field of view.
        stride = int(np.ceil(len(points) / source.max_points))
        return points[::stride]

    def cloud_callback(self, source, cloud):
        now = time.monotonic()
        if not source.due(now):
            return

        points = point_cloud2.read_points_numpy(
            cloud,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )
        points = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        if points.size == 0:
            return

        origin = self.scan_origin(source)
        if origin is None:
            return
        origin_x, origin_y = origin

        obstacles = self.filter_points(source, points, origin_x, origin_y)
        if len(obstacles) == 0:
            self.get_logger().warning(
                f'No {source.name} points remained after map filtering',
                throttle_duration_sec=10.0,
            )
            return

        source.last_update = now
        if not self.accumulate:
            self.grid = None

        self.integrate(obstacles, origin_x, origin_y)
        self.publish_map(cloud.header.stamp, source, len(obstacles))

    def integrate(self, obstacles, origin_x, origin_y):
        self.ensure_bounds(
            min(float(np.min(obstacles[:, 0])), origin_x),
            min(float(np.min(obstacles[:, 1])), origin_y),
            max(float(np.max(obstacles[:, 0])), origin_x),
            max(float(np.max(obstacles[:, 1])), origin_y),
        )

        origin_cell = self.world_to_cell(origin_x, origin_y)
        occupied_cells = set()
        for point in obstacles:
            end_cell = self.world_to_cell(point[0], point[1])
            if not self.cell_in_bounds(end_cell):
                continue
            for cell in bresenham(
                origin_cell[0],
                origin_cell[1],
                end_cell[0],
                end_cell[1],
            ):
                if not self.cell_in_bounds(cell):
                    break
                # Never punch a hole through an obstacle another sensor -
                # or an earlier observation - already reported. A depth
                # ray skimming a wall must not erase that wall.
                if self.grid[cell[1], cell[0]] != OCCUPIED:
                    self.grid[cell[1], cell[0]] = FREE
            occupied_cells.add(end_cell)

        for x, y in occupied_cells:
            self.grid[y, x] = OCCUPIED

    def publish_map(self, stamp, source, point_count):
        height, width = self.grid.shape
        message = OccupancyGrid()
        message.header.stamp = stamp
        message.header.frame_id = self.target_frame
        message.info.resolution = self.resolution
        message.info.width = width
        message.info.height = height
        message.info.origin.position.x = float(self.grid_min_x)
        message.info.origin.position.y = float(self.grid_min_y)
        message.info.origin.orientation.w = 1.0
        message.data = self.grid.reshape(-1).astype(int).tolist()
        self.publisher.publish(message)
        self.get_logger().info(
            f'Published occupancy grid {width}x{height} after {point_count} '
            f'{source.name} points',
            throttle_duration_sec=5.0,
        )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudToOccupancy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
