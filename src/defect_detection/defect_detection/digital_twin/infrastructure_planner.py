import math
import time
from collections import deque

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


def yaw_to_pose(pose, yaw):
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = quaternion_from_yaw(yaw)


def unknown_adjacent(unknown):
    """True where any of the 8 neighbours is unknown.

    A frontier is free space that borders the unexplored map edge, so we
    grow the unknown mask by one cell and intersect it with free space.
    """
    padded = np.pad(unknown, 1, mode='constant', constant_values=False)
    out = np.zeros_like(unknown)
    for dy in range(3):
        for dx in range(3):
            if dy == 1 and dx == 1:
                continue
            out |= padded[dy:dy + unknown.shape[0], dx:dx + unknown.shape[1]]
    return out


def cluster_cells(mask):
    """Group the True cells of a boolean mask into 8-connected clusters.

    Frontier cells sit on the boundary between free and unknown space, so
    they are sparse and this flood fill stays cheap even on a large grid.
    Returns a list of clusters, each a list of (x, y) cell indices.
    """
    visited = np.zeros_like(mask)
    clusters = []
    ys, xs = np.nonzero(mask)
    height, width = mask.shape
    for start_x, start_y in zip(xs.tolist(), ys.tolist()):
        if visited[start_y, start_x]:
            continue
        queue = deque([(start_x, start_y)])
        visited[start_y, start_x] = True
        component = []
        while queue:
            x, y = queue.popleft()
            component.append((x, y))
            for dx in (-1, 0, 1):
                for dy in (-1, 0, 1):
                    if dx == 0 and dy == 0:
                        continue
                    nx, ny = x + dx, y + dy
                    if (
                        0 <= nx < width
                        and 0 <= ny < height
                        and mask[ny, nx]
                        and not visited[ny, nx]
                    ):
                        visited[ny, nx] = True
                        queue.append((nx, ny))
        clusters.append(component)
    return clusters


class InfrastructurePlanner(Node):
    """Time-efficient frontier explorer for the Oak-D occupancy map.

    The map is built from the depth camera alone. Each planning tick the
    planner finds the frontier - free cells that border unexplored space -
    clusters it, and drives to the cluster with the best cost/utility
    trade-off: a large frontier (much to reveal) that is close by (little
    time to reach). Exploration is capped to a radius around the mission
    start so the robot sweeps roughly a fixed area and then declares the
    map done, which hands the mission over to the scan phase.

    Nothing here uses object detection; where the robot goes depends only
    on the shape of the map it has built so far.
    """

    def __init__(self):
        super().__init__('infrastructure_inspection_planner')

        self.declare_parameter('enabled', True)
        self.declare_parameter('map_topic', '/digital_twin/map')
        self.declare_parameter('goal_topic', '/infrastructure/inspection_goal')
        self.declare_parameter('status_topic', '/infrastructure/planner_status')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('goal_cooldown_sec', 20.0)
        self.declare_parameter('planning_period_sec', 5.0)
        # Ignore frontiers within this distance of the robot: it is already
        # standing there and moving a fraction of a metre reveals nothing.
        self.declare_parameter('min_frontier_distance_m', 1.0)
        # Cap exploration to a disc of this radius around the mission start
        # pose. Frontiers outside it are left alone, so the robot maps a
        # bounded area instead of chasing the map edge indefinitely.
        self.declare_parameter('exploration_radius_m', 40.0)
        # Drop specks of frontier (sensor noise, lone cells). A real map
        # edge is many cells long.
        self.declare_parameter('min_frontier_cluster_cells', 4)
        # Cost/utility trade-off. utility = cluster_size * exp(-distance /
        # travel_decay_m). A larger decay tolerates longer drives for a big
        # payoff; a smaller one keeps the robot greedy and nearby (faster).
        self.declare_parameter('travel_decay_m', 8.0)
        # The mission manager pauses exploration (scan phase, walking home)
        # by publishing False here.
        self.declare_parameter(
            'allow_exploration_topic',
            '/mission/allow_exploration',
        )

        self.enabled = self.get_parameter(
            'enabled'
        ).get_parameter_value().bool_value
        map_topic = self.get_parameter(
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
        self.goal_cooldown = self.get_parameter(
            'goal_cooldown_sec'
        ).get_parameter_value().double_value
        planning_period = self.get_parameter(
            'planning_period_sec'
        ).get_parameter_value().double_value
        self.min_frontier_distance = self.get_parameter(
            'min_frontier_distance_m'
        ).get_parameter_value().double_value
        self.exploration_radius = self.get_parameter(
            'exploration_radius_m'
        ).get_parameter_value().double_value
        self.min_frontier_cluster_cells = self.get_parameter(
            'min_frontier_cluster_cells'
        ).get_parameter_value().integer_value
        self.travel_decay = self.get_parameter(
            'travel_decay_m'
        ).get_parameter_value().double_value
        allow_exploration_topic = self.get_parameter(
            'allow_exploration_topic'
        ).get_parameter_value().string_value

        if self.travel_decay <= 0.0:
            raise ValueError('travel_decay_m must be greater than zero')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_publisher = self.create_publisher(PoseStamped, goal_topic, 10)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self.map_callback,
            1,
        )
        self.latest_map = None
        self.last_goal_time = 0.0
        self.last_goal_key = None
        # The mission start pose, latched the first time the robot is
        # localized; the exploration radius is measured from here.
        self.home = None
        # Explore until the mission manager says otherwise. Missions that
        # run without a mission manager keep exploring indefinitely.
        self.allow_exploration = True

        self.allow_exploration_subscription = self.create_subscription(
            Bool,
            allow_exploration_topic,
            self.allow_exploration_callback,
            10,
        )

        self.timer = self.create_timer(
            max(0.5, planning_period),
            self.plan_tick,
        )

        self.get_logger().info(
            f'Infrastructure planner enabled={self.enabled}; '
            f'map={map_topic}, goal={goal_topic}, '
            f'exploration_radius={self.exploration_radius:.1f}m'
        )

    def map_callback(self, grid):
        self.latest_map = grid

    def allow_exploration_callback(self, message):
        if message.data == self.allow_exploration:
            return
        self.allow_exploration = message.data
        self.publish_status(
            'exploration resumed by the mission manager'
            if message.data
            else 'exploration paused by the mission manager'
        )

    def plan_tick(self):
        if not self.enabled or not self.allow_exploration:
            return
        now = time.monotonic()
        if now - self.last_goal_time < self.goal_cooldown:
            return
        robot = self.robot_position()
        if robot is None:
            return
        if self.home is None:
            self.home = robot
            self.get_logger().info(
                f'Latched mission start: x={robot[0]:.2f}, y={robot[1]:.2f}; '
                f'exploring within {self.exploration_radius:.1f}m of it'
            )

        if self.latest_map is None:
            self.publish_status('waiting for digital twin occupancy map')
            return
        frontier = self.select_frontier(self.latest_map, robot[0], robot[1])
        if frontier is None:
            self.publish_status('no frontier found')
            return
        goal = self.make_frontier_goal(self.latest_map, frontier, robot)
        self.publish_goal(goal, 'map frontier')

    def robot_position(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as error:
            self.publish_status(
                f'waiting for localization: {self.target_frame}->{self.base_frame}: {error}'
            )
            return None
        translation = transform.transform.translation
        return translation.x, translation.y

    def select_frontier(self, grid, robot_x, robot_y):
        """Return the world (x, y) of the best frontier to drive to.

        Best = highest cost/utility score across every frontier cluster
        inside the exploration radius, where a big, nearby frontier wins.
        Returns None when nothing worth visiting remains.
        """
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y
        if width == 0 or height == 0:
            return None

        data = np.asarray(grid.data, dtype=np.int16).reshape((height, width))
        frontier = (data == FREE) & unknown_adjacent(data == UNKNOWN)
        if not frontier.any():
            return None

        home_x, home_y = self.home if self.home is not None else (robot_x, robot_y)
        best = None
        best_score = -math.inf
        for cluster in cluster_cells(frontier):
            if len(cluster) < self.min_frontier_cluster_cells:
                continue
            cx = sum(x for x, _ in cluster) / len(cluster)
            cy = sum(y for _, y in cluster) / len(cluster)
            world_x = origin_x + (cx + 0.5) * resolution
            world_y = origin_y + (cy + 0.5) * resolution
            if self.exploration_radius > 0.0:
                from_home = math.hypot(world_x - home_x, world_y - home_y)
                if from_home > self.exploration_radius:
                    continue
            distance = math.hypot(world_x - robot_x, world_y - robot_y)
            if distance < self.min_frontier_distance:
                continue
            score = len(cluster) * math.exp(-distance / self.travel_decay)
            if score > best_score:
                best_score = score
                # Aim at the cluster cell nearest its own centroid so the
                # goal lands on real free space, not an interpolated point.
                nearest = min(
                    cluster,
                    key=lambda cell: (cell[0] - cx) ** 2 + (cell[1] - cy) ** 2,
                )
                best = (
                    origin_x + (nearest[0] + 0.5) * resolution,
                    origin_y + (nearest[1] + 0.5) * resolution,
                )
        return best

    def make_frontier_goal(self, grid, frontier, robot):
        x, y = frontier
        yaw = math.atan2(y - robot[1], x - robot[0])
        goal = PoseStamped()
        goal.header.frame_id = grid.header.frame_id or self.target_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        yaw_to_pose(goal.pose, yaw)
        return goal

    def publish_goal(self, goal, reason):
        key = (
            reason,
            round(goal.pose.position.x, 2),
            round(goal.pose.position.y, 2),
        )
        if key == self.last_goal_key:
            return
        self.last_goal_key = key
        self.last_goal_time = time.monotonic()
        self.goal_publisher.publish(goal)
        self.publish_status(
            f'published {reason} goal: '
            f'x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}'
        )

    def publish_status(self, message):
        status = String()
        status.data = message
        self.status_publisher.publish(status)
        self.get_logger().info(message)


def main(args=None):
    rclpy.init(args=args)
    node = InfrastructurePlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
