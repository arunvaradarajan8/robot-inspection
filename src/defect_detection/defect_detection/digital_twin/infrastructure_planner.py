import math
import time

from geometry_msgs.msg import PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


UNKNOWN = -1
FREE = 0


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_to_pose(pose, yaw):
    (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    ) = quaternion_from_yaw(yaw)


class InfrastructurePlanner(Node):

    def __init__(self):
        super().__init__('infrastructure_inspection_planner')

        self.declare_parameter('enabled', True)
        self.declare_parameter('map_topic', '/digital_twin/map')
        self.declare_parameter('rescan_goals_topic', '/digital_twin/rescan_goals')
        self.declare_parameter('goal_topic', '/infrastructure/inspection_goal')
        self.declare_parameter('status_topic', '/infrastructure/planner_status')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('standoff_distance_m', 1.5)
        self.declare_parameter('min_frontier_distance_m', 1.0)
        self.declare_parameter('goal_cooldown_sec', 20.0)
        self.declare_parameter('planning_period_sec', 5.0)
        self.declare_parameter('prefer_defect_rescans', True)
        # Stop publishing goals between a waypoint arrival (or scan
        # request) and the scanner reporting that it finished, so the
        # robot stands still for the duration of the X7 scan.
        self.declare_parameter('hold_for_scan', True)
        self.declare_parameter('scan_required_topic', '/digital_twin/scan_required')
        # The X7 keeps its scans on its own SD card, so nothing comes back
        # over ROS to signal the end of a scan. The Windows bridge reports
        # completion on this topic instead.
        self.declare_parameter('scan_complete_topic', '/digital_twin/scan_complete')
        self.declare_parameter(
            'waypoint_arrived_topic',
            '/digital_twin/waypoint_arrived',
        )
        # The mission manager takes over goal publishing when the mission
        # stops exploring and starts walking home.
        self.declare_parameter(
            'allow_exploration_topic',
            '/mission/allow_exploration',
        )
        self.declare_parameter('scan_wait_timeout_sec', 300.0)
        self.declare_parameter('scan_decision_settle_sec', 5.0)

        self.enabled = self.get_parameter(
            'enabled'
        ).get_parameter_value().bool_value
        map_topic = self.get_parameter(
            'map_topic'
        ).get_parameter_value().string_value
        rescan_goals_topic = self.get_parameter(
            'rescan_goals_topic'
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
        self.standoff_distance = self.get_parameter(
            'standoff_distance_m'
        ).get_parameter_value().double_value
        self.min_frontier_distance = self.get_parameter(
            'min_frontier_distance_m'
        ).get_parameter_value().double_value
        self.goal_cooldown = self.get_parameter(
            'goal_cooldown_sec'
        ).get_parameter_value().double_value
        planning_period = self.get_parameter(
            'planning_period_sec'
        ).get_parameter_value().double_value
        self.prefer_defect_rescans = self.get_parameter(
            'prefer_defect_rescans'
        ).get_parameter_value().bool_value
        self.hold_for_scan = self.get_parameter(
            'hold_for_scan'
        ).get_parameter_value().bool_value
        scan_required_topic = self.get_parameter(
            'scan_required_topic'
        ).get_parameter_value().string_value
        scan_complete_topic = self.get_parameter(
            'scan_complete_topic'
        ).get_parameter_value().string_value
        waypoint_arrived_topic = self.get_parameter(
            'waypoint_arrived_topic'
        ).get_parameter_value().string_value
        allow_exploration_topic = self.get_parameter(
            'allow_exploration_topic'
        ).get_parameter_value().string_value
        self.scan_wait_timeout = self.get_parameter(
            'scan_wait_timeout_sec'
        ).get_parameter_value().double_value
        self.scan_decision_settle = self.get_parameter(
            'scan_decision_settle_sec'
        ).get_parameter_value().double_value

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
        self.rescan_subscription = self.create_subscription(
            PoseArray,
            rescan_goals_topic,
            self.rescan_goals_callback,
            10,
        )
        self.latest_map = None
        self.rescan_goals = []
        self.last_goal_time = 0.0
        self.last_goal_key = None
        # None, 'decision' (arrived; scan decision pending) or
        # 'scan' (scan requested; waiting for the scan to be ingested).
        self.hold_state = None
        self.hold_since = 0.0
        # Explore until the mission manager says otherwise. Missions that
        # run without a mission manager keep exploring indefinitely.
        self.allow_exploration = True

        self.allow_exploration_subscription = self.create_subscription(
            Bool,
            allow_exploration_topic,
            self.allow_exploration_callback,
            10,
        )

        if self.hold_for_scan:
            self.scan_required_subscription = self.create_subscription(
                Bool,
                scan_required_topic,
                self.scan_required_callback,
                10,
            )
            self.scan_complete_subscription = self.create_subscription(
                Bool,
                scan_complete_topic,
                self.scan_complete_callback,
                10,
            )
            self.arrival_subscription = self.create_subscription(
                String,
                waypoint_arrived_topic,
                self.waypoint_arrived_callback,
                10,
            )

        self.timer = self.create_timer(
            max(0.5, planning_period),
            self.plan_tick,
        )

        self.get_logger().info(
            f'Infrastructure planner enabled={self.enabled}; '
            f'map={map_topic}, rescans={rescan_goals_topic}, goal={goal_topic}'
        )

    def map_callback(self, grid):
        self.latest_map = grid

    def rescan_goals_callback(self, poses):
        self.rescan_goals = list(poses.poses)

    def waypoint_arrived_callback(self, _message):
        # An arrival usually precedes a scan request; wait for the scan
        # decision before moving on so the X7 is not dragged mid-scan.
        self.hold_state = 'decision'
        self.hold_since = time.monotonic()

    def scan_required_callback(self, message):
        now = time.monotonic()
        if message.data:
            self.hold_state = 'scan'
            self.hold_since = now
        elif (
            self.hold_state == 'decision'
            and now - self.hold_since > self.scan_decision_settle
        ):
            # The scan decision declined to scan here; resume exploring.
            self.hold_state = None

    def scan_complete_callback(self, message):
        if message.data and self.hold_state is not None:
            self.hold_state = None
            self.publish_status('scan finished; resuming exploration')

    def allow_exploration_callback(self, message):
        if message.data == self.allow_exploration:
            return
        self.allow_exploration = message.data
        self.publish_status(
            'exploration resumed by the mission manager'
            if message.data
            else 'exploration paused by the mission manager'
        )

    def scan_hold_active(self, now):
        if self.hold_state is None:
            return False
        if now - self.hold_since > self.scan_wait_timeout:
            self.publish_status(
                'scan hold timed out; resuming exploration'
            )
            self.hold_state = None
            return False
        return True

    def plan_tick(self):
        if not self.enabled or not self.allow_exploration:
            return
        now = time.monotonic()
        if self.scan_hold_active(now):
            self.publish_status('holding position for X7 scan')
            return
        if now - self.last_goal_time < self.goal_cooldown:
            return
        robot = self.robot_position()
        if robot is None:
            return

        if self.prefer_defect_rescans:
            goal = self.next_rescan_goal(robot)
            if goal is not None:
                self.publish_goal(goal, 'defect rescan')
                return

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

    def next_rescan_goal(self, robot):
        if not self.rescan_goals:
            return None
        best = None
        best_distance = math.inf
        for pose in self.rescan_goals:
            distance = math.hypot(
                pose.position.x - robot[0],
                pose.position.y - robot[1],
            )
            if distance < best_distance:
                best_distance = distance
                best = pose
        if best is None:
            return None

        dx = best.position.x - robot[0]
        dy = best.position.y - robot[1]
        distance = math.hypot(dx, dy)
        if distance <= 0.01:
            return None
        standoff = min(self.standoff_distance, max(0.0, distance - 0.3))
        goal_x = best.position.x - standoff * dx / distance
        goal_y = best.position.y - standoff * dy / distance
        yaw = math.atan2(best.position.y - goal_y, best.position.x - goal_x)

        goal = PoseStamped()
        goal.header.frame_id = self.target_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = goal_x
        goal.pose.position.y = goal_y
        goal.pose.position.z = 0.0
        yaw_to_pose(goal.pose, yaw)
        return goal

    def select_frontier(self, grid, robot_x, robot_y):
        data = grid.data
        width = grid.info.width
        height = grid.info.height
        resolution = grid.info.resolution
        origin_x = grid.info.origin.position.x
        origin_y = grid.info.origin.position.y

        best = None
        best_score = -math.inf
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if data[y * width + x] != FREE:
                    continue
                if not self.has_unknown_neighbor(data, width, x, y):
                    continue
                world_x = origin_x + (x + 0.5) * resolution
                world_y = origin_y + (y + 0.5) * resolution
                distance = math.hypot(world_x - robot_x, world_y - robot_y)
                if distance < self.min_frontier_distance:
                    continue
                score = distance
                if score > best_score:
                    best_score = score
                    best = world_x, world_y
        return best

    @staticmethod
    def has_unknown_neighbor(data, width, x, y):
        offsets = [
            (-1, -1), (0, -1), (1, -1),
            (-1, 0), (1, 0),
            (-1, 1), (0, 1), (1, 1),
        ]
        return any(data[(y + dy) * width + x + dx] == UNKNOWN for dx, dy in offsets)

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
