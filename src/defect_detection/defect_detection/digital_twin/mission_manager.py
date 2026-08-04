from pathlib import Path
import math
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


EXPLORING = 'EXPLORING'
SCANNING = 'SCANNING'
RETURNING = 'RETURNING'
AWAITING_UPLOAD = 'AWAITING_UPLOAD'
COMPLETE = 'COMPLETE'


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def yaw_from_quaternion(rotation):
    siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cosy_cosp = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z)
    return math.atan2(siny_cosp, cosy_cosp)


class MissionManager(Node):
    """Own the mission lifecycle and bring the robot home when it ends.

    The inspection planner only ever proposes the next place to look. This
    node decides when looking is finished. It then hands control to the
    scan planner, which picks the best vantages on the finished map and
    takes the X7 scans there; once scanning reports done, it walks the
    robot back along the stations it already visited to the pose it started
    from, and holds the mission open until the operator has uploaded the
    Trimble E57 that stayed on the scanner's SD card during the run.

    The lifecycle is EXPLORING -> SCANNING -> RETURNING -> AWAITING_UPLOAD
    -> COMPLETE.
    """

    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'body')
        self.declare_parameter('goal_topic', '/infrastructure/inspection_goal')
        self.declare_parameter(
            'waypoint_arrived_topic',
            '/digital_twin/waypoint_arrived',
        )
        self.declare_parameter(
            'planner_status_topic',
            '/infrastructure/planner_status',
        )
        self.declare_parameter(
            'allow_exploration_topic',
            '/mission/allow_exploration',
        )
        self.declare_parameter('state_topic', '/mission/state')
        self.declare_parameter('return_home_topic', '/mission/return_home')
        self.declare_parameter('upload_complete_topic', '/mission/upload_complete')
        # Scan phase: when exploration ends, hand off to the scan planner
        # rather than walking straight home. Turn off to scan-less missions
        # that explore and return.
        self.declare_parameter('scan_phase_enabled', True)
        self.declare_parameter('start_scanning_topic', '/mission/start_scanning')
        self.declare_parameter(
            'scanning_complete_topic',
            '/mission/scanning_complete',
        )
        # The scan phase is operator-paced: the robot parks at each vantage
        # and waits for a human to release it to the next one, so there is no
        # fixed wall-clock budget. 0 disables the whole-phase timeout and lets
        # the mission wait for the operator to work through every station. Set
        # it above zero only as a safety net to walk home if the scan planner
        # never reports back.
        self.declare_parameter('scanning_timeout_sec', 0.0)
        self.declare_parameter('summary_path', '/tmp/mission_summary.yaml')
        # Mission end conditions. Any one of them ends exploration.
        self.declare_parameter('max_stations', 0)
        self.declare_parameter('max_excursion_m', 30.0)
        self.declare_parameter('mission_duration_sec', 0.0)
        self.declare_parameter('no_frontier_patience_sec', 60.0)
        # Return-leg behaviour.
        self.declare_parameter('breadcrumb_min_spacing_m', 2.0)
        self.declare_parameter('return_leg_timeout_sec', 90.0)
        self.declare_parameter('home_position_tolerance_m', 1.0)
        self.declare_parameter('tick_period_sec', 1.0)

        self.target_frame = self.get_parameter(
            'target_frame'
        ).get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame'
        ).get_parameter_value().string_value
        goal_topic = self.get_parameter(
            'goal_topic'
        ).get_parameter_value().string_value
        waypoint_arrived_topic = self.get_parameter(
            'waypoint_arrived_topic'
        ).get_parameter_value().string_value
        planner_status_topic = self.get_parameter(
            'planner_status_topic'
        ).get_parameter_value().string_value
        allow_exploration_topic = self.get_parameter(
            'allow_exploration_topic'
        ).get_parameter_value().string_value
        state_topic = self.get_parameter(
            'state_topic'
        ).get_parameter_value().string_value
        return_home_topic = self.get_parameter(
            'return_home_topic'
        ).get_parameter_value().string_value
        upload_complete_topic = self.get_parameter(
            'upload_complete_topic'
        ).get_parameter_value().string_value
        self.scan_phase_enabled = self.get_parameter(
            'scan_phase_enabled'
        ).get_parameter_value().bool_value
        start_scanning_topic = self.get_parameter(
            'start_scanning_topic'
        ).get_parameter_value().string_value
        scanning_complete_topic = self.get_parameter(
            'scanning_complete_topic'
        ).get_parameter_value().string_value
        self.scanning_timeout = self.get_parameter(
            'scanning_timeout_sec'
        ).get_parameter_value().double_value
        self.summary_path = Path(
            self.get_parameter('summary_path').get_parameter_value().string_value
        ).expanduser()
        self.max_stations = self.get_parameter(
            'max_stations'
        ).get_parameter_value().integer_value
        self.max_excursion = self.get_parameter(
            'max_excursion_m'
        ).get_parameter_value().double_value
        self.mission_duration = self.get_parameter(
            'mission_duration_sec'
        ).get_parameter_value().double_value
        self.no_frontier_patience = self.get_parameter(
            'no_frontier_patience_sec'
        ).get_parameter_value().double_value
        self.breadcrumb_min_spacing = self.get_parameter(
            'breadcrumb_min_spacing_m'
        ).get_parameter_value().double_value
        self.return_leg_timeout = self.get_parameter(
            'return_leg_timeout_sec'
        ).get_parameter_value().double_value
        self.home_position_tolerance = self.get_parameter(
            'home_position_tolerance_m'
        ).get_parameter_value().double_value
        tick_period = self.get_parameter(
            'tick_period_sec'
        ).get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.goal_publisher = self.create_publisher(PoseStamped, goal_topic, 10)
        self.state_publisher = self.create_publisher(String, state_topic, 10)
        self.allow_exploration_publisher = self.create_publisher(
            Bool,
            allow_exploration_topic,
            10,
        )
        self.start_scanning_publisher = self.create_publisher(
            Bool,
            start_scanning_topic,
            10,
        )
        self.create_subscription(
            Bool,
            scanning_complete_topic,
            self.scanning_complete_callback,
            10,
        )
        self.create_subscription(
            String,
            waypoint_arrived_topic,
            self.waypoint_arrived_callback,
            10,
        )
        self.create_subscription(
            String,
            planner_status_topic,
            self.planner_status_callback,
            10,
        )
        self.create_subscription(
            Bool,
            return_home_topic,
            self.return_home_callback,
            10,
        )
        self.create_subscription(
            Bool,
            upload_complete_topic,
            self.upload_complete_callback,
            10,
        )

        self.state = EXPLORING
        self.mission_started = time.monotonic()
        self.start_pose = None
        self.breadcrumbs = []
        self.stations = []
        self.return_queue = []
        self.return_goal_sent = 0.0
        self.return_goal_pending = False
        self.no_frontier_since = None
        self.end_reason = None
        self.scanning_started = 0.0

        self.timer = self.create_timer(max(0.1, tick_period), self.tick)
        self.get_logger().info(
            f'Mission manager started; max_excursion={self.max_excursion:.1f}m, '
            f'max_stations={self.max_stations or "unlimited"}, '
            f'summary={self.summary_path}'
        )

    # ---- inputs -------------------------------------------------------

    def waypoint_arrived_callback(self, _message):
        pose = self.robot_pose()
        if pose is None:
            return
        if self.state == EXPLORING:
            self.record_station(pose)
        elif self.state == RETURNING:
            self.return_goal_pending = False

    def planner_status_callback(self, message):
        if self.state != EXPLORING:
            return
        # The planner says this when the map has no reachable unknown edge
        # left; see infrastructure_planner.plan_tick.
        if 'no frontier found' in message.data:
            if self.no_frontier_since is None:
                self.no_frontier_since = time.monotonic()
        else:
            self.no_frontier_since = None

    def return_home_callback(self, message):
        if message.data and self.state == EXPLORING:
            self.begin_return('operator requested return home')

    def scanning_complete_callback(self, message):
        if message.data and self.state == SCANNING:
            self.begin_return('scanning complete')

    def upload_complete_callback(self, message):
        if message.data and self.state == AWAITING_UPLOAD:
            self.state = COMPLETE
            self.write_summary()
            self.get_logger().info('E57 upload confirmed; mission complete')

    # ---- state machine ------------------------------------------------

    def tick(self):
        self.publish_state()
        self.allow_exploration_publisher.publish(
            Bool(data=self.state == EXPLORING)
        )

        if self.state == EXPLORING:
            self.explore_tick()
        elif self.state == SCANNING:
            self.scanning_tick()
        elif self.state == RETURNING:
            self.return_tick()

    def explore_tick(self):
        pose = self.robot_pose()
        if pose is None:
            return
        if self.start_pose is None:
            self.start_pose = pose
            self.get_logger().info(
                f'Recorded mission start pose: x={pose[0]:.2f}, y={pose[1]:.2f}'
            )
            return

        reason = self.end_condition(pose)
        if reason is not None:
            if self.scan_phase_enabled:
                self.begin_scanning(reason)
            else:
                self.begin_return(reason)

    def end_condition(self, pose):
        if self.max_stations > 0 and len(self.stations) >= self.max_stations:
            return f'station limit reached ({self.max_stations})'

        if self.max_excursion > 0.0:
            distance = math.hypot(
                pose[0] - self.start_pose[0],
                pose[1] - self.start_pose[1],
            )
            if distance >= self.max_excursion:
                return (
                    f'excursion limit reached ({distance:.1f}m from start, '
                    f'limit {self.max_excursion:.1f}m)'
                )

        if self.mission_duration > 0.0:
            elapsed = time.monotonic() - self.mission_started
            if elapsed >= self.mission_duration:
                return f'mission duration reached ({elapsed:.0f}s)'

        if (
            self.no_frontier_since is not None
            and time.monotonic() - self.no_frontier_since
            >= self.no_frontier_patience
        ):
            return 'no frontier remaining'

        return None

    def begin_scanning(self, reason):
        self.end_reason = reason
        self.state = SCANNING
        self.scanning_started = time.monotonic()
        self.get_logger().info(
            f'Exploration finished ({reason}); handing off to the scan '
            'planner to capture the X7 scans'
        )

    def scanning_tick(self):
        # Keep asking the scan planner to run; it ignores repeats once it is
        # underway, and a late-starting node still catches the request.
        self.start_scanning_publisher.publish(Bool(data=True))
        if (
            self.scanning_timeout > 0.0
            and time.monotonic() - self.scanning_started >= self.scanning_timeout
        ):
            self.get_logger().warning(
                'Scan phase timed out; walking home without waiting for it'
            )
            self.begin_return('scan phase timed out')

    def begin_return(self, reason):
        # Preserve the reason exploration ended; the scan phase completing
        # is a transition, not the reason the mission wrapped up.
        if self.end_reason is None:
            self.end_reason = reason
        self.state = RETURNING
        # Retrace the stations in reverse, then finish at the start pose.
        self.return_queue = list(reversed(self.breadcrumbs))
        if self.start_pose is not None:
            self.return_queue.append(self.start_pose)
        self.return_goal_pending = False
        self.get_logger().info(
            f'Returning home: {reason}; '
            f'{len(self.return_queue)} leg(s) to retrace'
        )

    def return_tick(self):
        now = time.monotonic()
        if self.return_goal_pending:
            if now - self.return_goal_sent < self.return_leg_timeout:
                return

            self.get_logger().warning(
                'Return leg timed out; skipping to the next one'
            )
            self.return_goal_pending = False

        if not self.return_queue:
            self.finish_return()
            return

        target = self.return_queue.pop(0)
        pose = self.robot_pose()
        if pose is not None and not self.return_queue:
            distance = math.hypot(target[0] - pose[0], target[1] - pose[1])
            if distance <= self.home_position_tolerance:
                self.finish_return()
                return

        self.publish_return_goal(target, pose)
        self.return_goal_sent = now
        self.return_goal_pending = True

    def finish_return(self):
        self.state = AWAITING_UPLOAD
        self.write_summary()
        self.get_logger().info(
            'Back at the mission start pose. Waiting for the Trimble E57 '
            'upload before closing the mission.'
        )

    # ---- helpers haha 67 ------------------------------------------------------

    def robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as error:
            self.get_logger().warning(
                f'Waiting for localization '
                f'{self.target_frame}->{self.base_frame}: {error}',
                throttle_duration_sec=10.0,
            )
            return None
        translation = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        return translation.x, translation.y, yaw

    def record_station(self, pose):
        self.stations.append(
            {
                'x': float(pose[0]),
                'y': float(pose[1]),
                'yaw': float(pose[2]),
                'arrived_sec': time.time(),
            }
        )
        if self.breadcrumbs:
            last = self.breadcrumbs[-1]
            spacing = math.hypot(pose[0] - last[0], pose[1] - last[1])
            if spacing < self.breadcrumb_min_spacing:
                return
        self.breadcrumbs.append(pose)
        self.get_logger().info(
            f'Recorded breadcrumb {len(self.breadcrumbs)}: '
            f'x={pose[0]:.2f}, y={pose[1]:.2f}'
        )

    def publish_return_goal(self, target, pose):
        goal = PoseStamped()
        goal.header.frame_id = self.target_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(target[0])
        goal.pose.position.y = float(target[1])
        # Face the direction of travel so the robot walks forwards home;
        # fall back to the recorded heading when the pose is unknown.
        if pose is not None:
            yaw = math.atan2(target[1] - pose[1], target[0] - pose[0])
        else:
            yaw = float(target[2])
        (
            goal.pose.orientation.x,
            goal.pose.orientation.y,
            goal.pose.orientation.z,
            goal.pose.orientation.w,
        ) = quaternion_from_yaw(yaw)
        self.goal_publisher.publish(goal)
        self.get_logger().info(
            f'Return goal: x={target[0]:.2f}, y={target[1]:.2f} '
            f'({len(self.return_queue)} leg(s) remaining)'
        )

    def publish_state(self):
        message = String()
        message.data = self.state
        self.state_publisher.publish(message)

    def write_summary(self):
        summary = {
            'state': self.state,
            'end_reason': self.end_reason,
            'started_sec': time.time() - (time.monotonic() - self.mission_started),
            'duration_sec': time.monotonic() - self.mission_started,
            'start_pose': None,
            'stations': self.stations,
            'breadcrumbs': [
                {'x': float(x), 'y': float(y), 'yaw': float(yaw)}
                for x, y, yaw in self.breadcrumbs
            ],
        }
        if self.start_pose is not None:
            summary['start_pose'] = {
                'x': float(self.start_pose[0]),
                'y': float(self.start_pose[1]),
                'yaw': float(self.start_pose[2]),
            }
        try:
            self.summary_path.parent.mkdir(parents=True, exist_ok=True)
            with self.summary_path.open('w', encoding='utf-8') as stream:
                yaml.safe_dump({'mission': summary}, stream, sort_keys=True)
        except OSError as error:
            self.get_logger().warning(f'Cannot write mission summary: {error}')
            return
        self.get_logger().info(f'Wrote mission summary to {self.summary_path}')


def main(args=None):
    rclpy.init(args=args)
    node = MissionManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
