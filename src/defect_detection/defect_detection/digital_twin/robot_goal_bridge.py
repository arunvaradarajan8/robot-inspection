import json
import math
import os
import time
from urllib import error as urlerror
from urllib import request as urlrequest

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from std_msgs.msg import String
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


def post_json(url, payload, timeout_sec):
    data = json.dumps(payload).encode('utf-8')
    request = urlrequest.Request(
        url,
        data=data,
        headers={'content-type': 'application/json'},
        method='POST',
    )
    with urlrequest.urlopen(request, timeout=timeout_sec) as response:
        return response.status, response.read().decode('utf-8')


def compose_body_goal_in_spot_frame(spot_x, spot_y, spot_yaw, dx, dy, dyaw):
    """Re-express a body-relative goal in Spot's command frame.

    (spot_x, spot_y, spot_yaw) is where Spot's own snapshot says the body is
    in its command frame; (dx, dy, dyaw) is the goal relative to the body as
    computed from the camera localization. Composing the two yields the SE2
    goal Spot's firmware will accept, without ever feeding Spot's pose back
    into our localization or the map.
    """
    goal_x = spot_x + math.cos(spot_yaw) * dx - math.sin(spot_yaw) * dy
    goal_y = spot_y + math.sin(spot_yaw) * dx + math.cos(spot_yaw) * dy
    return goal_x, goal_y, normalize_angle(spot_yaw + dyaw)


class RobotGoalBridge(Node):

    def __init__(self):
        super().__init__('infrastructure_robot_goal_bridge')

        self.declare_parameter('goal_topic', '/infrastructure/inspection_goal')
        self.declare_parameter('waypoint_arrived_topic', '/digital_twin/waypoint_arrived')
        self.declare_parameter('status_topic', '/infrastructure/navigation_status')
        self.declare_parameter('backend', 'dry_run')
        self.declare_parameter('navigate_action', '/navigate_to_pose')
        self.declare_parameter('spot_command_url', '')
        self.declare_parameter('spot_ip', '')
        self.declare_parameter('spot_username', '')
        self.declare_parameter('spot_password', '')
        self.declare_parameter('spot_command_frame', 'odom')
        self.declare_parameter('spot_goal_duration_sec', 30.0)
        self.declare_parameter('spot_arrival_timeout_sec', 45.0)
        self.declare_parameter('spot_auto_power_on', False)
        self.declare_parameter('spot_stand_before_move', True)
        self.declare_parameter('spot_auto_return_lease', True)
        self.declare_parameter('arrival_check_source', 'tf')
        self.declare_parameter('arrival_base_frame', 'body')
        self.declare_parameter('arrival_position_tolerance_m', 0.35)
        self.declare_parameter('arrival_yaw_tolerance_rad', 0.45)
        self.declare_parameter('arrival_stable_sec', 1.5)
        self.declare_parameter('arrival_check_period_sec', 0.25)
        self.declare_parameter('http_timeout_sec', 3.0)
        self.declare_parameter('dry_run_arrival_delay_sec', 3.0)

        goal_topic = self.get_parameter(
            'goal_topic'
        ).get_parameter_value().string_value
        waypoint_arrived_topic = self.get_parameter(
            'waypoint_arrived_topic'
        ).get_parameter_value().string_value
        status_topic = self.get_parameter(
            'status_topic'
        ).get_parameter_value().string_value
        navigate_action = self.get_parameter(
            'navigate_action'
        ).get_parameter_value().string_value
        self.backend = self.get_parameter(
            'backend'
        ).get_parameter_value().string_value
        self.spot_command_url = self.get_parameter(
            'spot_command_url'
        ).get_parameter_value().string_value.rstrip('/')
        self.spot_ip = self.get_parameter(
            'spot_ip'
        ).get_parameter_value().string_value
        self.spot_username = self.get_parameter(
            'spot_username'
        ).get_parameter_value().string_value
        self.spot_password = self.get_parameter(
            'spot_password'
        ).get_parameter_value().string_value
        self.spot_command_frame = self.get_parameter(
            'spot_command_frame'
        ).get_parameter_value().string_value
        self.spot_goal_duration = self.get_parameter(
            'spot_goal_duration_sec'
        ).get_parameter_value().double_value
        self.spot_arrival_timeout = self.get_parameter(
            'spot_arrival_timeout_sec'
        ).get_parameter_value().double_value
        self.spot_auto_power_on = self.get_parameter(
            'spot_auto_power_on'
        ).get_parameter_value().bool_value
        self.spot_stand_before_move = self.get_parameter(
            'spot_stand_before_move'
        ).get_parameter_value().bool_value
        self.spot_auto_return_lease = self.get_parameter(
            'spot_auto_return_lease'
        ).get_parameter_value().bool_value
        self.arrival_check_source = self.get_parameter(
            'arrival_check_source'
        ).get_parameter_value().string_value
        self.arrival_base_frame = self.get_parameter(
            'arrival_base_frame'
        ).get_parameter_value().string_value
        self.arrival_position_tolerance = self.get_parameter(
            'arrival_position_tolerance_m'
        ).get_parameter_value().double_value
        self.arrival_yaw_tolerance = self.get_parameter(
            'arrival_yaw_tolerance_rad'
        ).get_parameter_value().double_value
        self.arrival_stable_sec = self.get_parameter(
            'arrival_stable_sec'
        ).get_parameter_value().double_value
        self.arrival_check_period = self.get_parameter(
            'arrival_check_period_sec'
        ).get_parameter_value().double_value
        self.http_timeout = self.get_parameter(
            'http_timeout_sec'
        ).get_parameter_value().double_value
        self.dry_run_arrival_delay = self.get_parameter(
            'dry_run_arrival_delay_sec'
        ).get_parameter_value().double_value

        self.arrival_publisher = self.create_publisher(
            String,
            waypoint_arrived_topic,
            10,
        )
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            goal_topic,
            self.goal_callback,
            10,
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.navigation_client = ActionClient(self, NavigateToPose, navigate_action)
        self.active_goal = None
        self.active_goal_started_sec = None
        self.arrival_stable_since = None
        self.arrival_timer = None
        self.dry_run_timer = None
        self.spot = None
        self.spot_command_client = None
        self.spot_state_client = None
        self.spot_lease_keepalive = None

        self.get_logger().info(
            f'Robot goal bridge listening on {goal_topic}; backend={self.backend}'
        )

    def goal_callback(self, goal):
        if self.active_goal is not None:
            self.publish_status('busy; ignoring new inspection goal')
            return

        self.active_goal = goal
        self.active_goal_started_sec = time.monotonic()
        self.arrival_stable_since = None
        self.publish_status(
            f'goal received: {goal.header.frame_id} '
            f'x={goal.pose.position.x:.2f}, y={goal.pose.position.y:.2f}'
        )
        if self.backend == 'nav2':
            self.send_nav2_goal(goal)
        elif self.backend == 'http':
            self.send_http_goal(goal)
        elif self.backend == 'spot_sdk':
            self.send_spot_sdk_goal(goal)
        else:
            self.get_logger().warning(
                'Dry-run navigation backend active; no robot motion command sent'
            )
            self.dry_run_timer = self.create_timer(
                self.dry_run_arrival_delay,
                self.dry_run_arrived,
            )

    def send_nav2_goal(self, goal):
        if not self.navigation_client.server_is_ready():
            self.finish_goal('nav2 unavailable', arrived=False)
            return
        request = NavigateToPose.Goal()
        request.pose = goal
        future = self.navigation_client.send_goal_async(request)
        future.add_done_callback(self.nav2_goal_response)

    def nav2_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.finish_goal(f'nav2 request failed: {error}', arrived=False)
            return
        if not goal_handle.accepted:
            self.finish_goal('nav2 rejected goal', arrived=False)
            return
        goal_handle.get_result_async().add_done_callback(self.nav2_result)

    def nav2_result(self, future):
        try:
            wrapped = future.result()
            self.confirm_arrival_or_finish(
                f'nav2 completed with status {wrapped.status}'
            )
        except Exception as error:
            self.finish_goal(f'nav2 failed: {error}', arrived=False)

    def send_http_goal(self, goal):
        if not self.spot_command_url:
            self.finish_goal('spot_command_url is not configured', arrived=False)
            return
        payload = {
            'frame_id': goal.header.frame_id,
            'position': {
                'x': goal.pose.position.x,
                'y': goal.pose.position.y,
                'z': goal.pose.position.z,
            },
            'orientation': {
                'x': goal.pose.orientation.x,
                'y': goal.pose.orientation.y,
                'z': goal.pose.orientation.z,
                'w': goal.pose.orientation.w,
            },
        }
        try:
            status, body = post_json(
                self.spot_command_url + '/navigate_to_pose',
                payload,
                self.http_timeout,
            )
            self.confirm_arrival_or_finish(
                f'spot http command accepted: {status} {body}'
            )
        except (OSError, urlerror.URLError, TimeoutError) as error:
            self.finish_goal(f'spot http command failed: {error}', arrived=False)

    def send_spot_sdk_goal(self, goal):
        try:
            self.ensure_spot_connected()
            goal_x, goal_y, goal_heading = self.spot_goal_coordinates(goal)
            command_id = self.command_spot_to_goal(goal_x, goal_y, goal_heading)
            self.wait_for_spot_goal(command_id)
        except Exception as error:
            self.finish_goal(f'spot sdk command failed: {error}', arrived=False)

    def spot_goal_coordinates(self, goal):
        """Express the goal as (x, y, heading) in Spot's command frame.

        With camera-only localization there is no TF linking the camera's
        world frame (depth_odom) to Spot's own frames, so the goal cannot be
        transformed directly. Instead the goal is made body-relative with
        the camera TF -- where the camera says the robot is -- and Spot's
        transform snapshot, read once at command time, re-expresses that
        displacement in Spot's command frame. Spot's pose is never fed back
        into localization or the map; it is only the dialect the motion
        command must be spoken in.

        If a direct TF to the command frame does exist (Spot localization
        turned back on), it is preferred, matching the old behaviour.
        """
        if goal.header.frame_id == self.spot_command_frame:
            return (
                goal.pose.position.x,
                goal.pose.position.y,
                yaw_from_quaternion(goal.pose.orientation),
            )

        if self.tf_buffer.can_transform(
            self.spot_command_frame,
            goal.header.frame_id,
            rclpy.time.Time(),
            timeout=Duration(seconds=0.25),
        ):
            transform = self.tf_buffer.lookup_transform(
                self.spot_command_frame,
                goal.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
            pose = do_transform_pose(goal.pose, transform)
            self.publish_status(
                f'goal transformed by TF into {self.spot_command_frame}'
            )
            return (
                pose.position.x,
                pose.position.y,
                yaw_from_quaternion(pose.orientation),
            )

        dx, dy, dyaw = self.body_relative_goal(goal)
        spot_x, spot_y, spot_yaw = self.spot_body_se2()
        self.publish_status(
            'camera-only goal: body-relative '
            f'dx={dx:.2f}, dy={dy:.2f}, dyaw={dyaw:.2f} composed with '
            f"Spot's snapshot pose in {self.spot_command_frame}"
        )
        return compose_body_goal_in_spot_frame(
            spot_x, spot_y, spot_yaw, dx, dy, dyaw,
        )

    def body_relative_goal(self, goal):
        """Goal relative to the robot body, from the camera localization."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.arrival_base_frame,
                goal.header.frame_id,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            raise RuntimeError(
                f'cannot express {goal.header.frame_id} goal relative to '
                f'{self.arrival_base_frame}: camera localization TF is '
                f'missing ({error})'
            ) from error
        pose = do_transform_pose(goal.pose, transform)
        return (
            pose.position.x,
            pose.position.y,
            yaw_from_quaternion(pose.orientation),
        )

    def spot_body_se2(self):
        """Spot's body pose in its command frame, from its own snapshot.

        Read once per command to translate a body-relative goal into Spot's
        dialect. This never enters the map or the pose estimate.
        """
        from bosdyn.client.frame_helpers import (
            BODY_FRAME_NAME,
            ODOM_FRAME_NAME,
            VISION_FRAME_NAME,
            get_a_tform_b,
        )

        frame_name = (
            VISION_FRAME_NAME
            if self.spot_command_frame == 'vision'
            else ODOM_FRAME_NAME
        )
        state = self.spot_state_client.get_robot_state()
        spot_tform_body = get_a_tform_b(
            state.kinematic_state.transforms_snapshot,
            frame_name,
            BODY_FRAME_NAME,
        )
        if spot_tform_body is None:
            raise RuntimeError(
                f'Spot snapshot has no {frame_name}->body transform'
            )
        se2 = spot_tform_body.get_closest_se2_transform()
        return se2.x, se2.y, se2.angle

    def command_spot_to_goal(self, goal_x, goal_y, goal_heading):
        try:
            from bosdyn.client.robot_command import RobotCommandBuilder
        except ImportError as error:
            raise RuntimeError(
                'bosdyn-client is not installed. Install requirements-field.txt '
                'on the Jetson before using ROBOT_GOAL_BACKEND=spot_sdk.'
            ) from error

        command = RobotCommandBuilder.synchro_se2_trajectory_point_command(
            goal_x=goal_x,
            goal_y=goal_y,
            goal_heading=goal_heading,
            frame_name=self.spot_command_frame,
        )
        end_time = time.time() + self.spot_goal_duration
        command_id = self.spot_command_client.robot_command(
            command,
            end_time_secs=end_time,
        )
        self.publish_status(
            'spot command sent: '
            f'{self.spot_command_frame} '
            f'x={goal_x:.2f}, y={goal_y:.2f}, yaw={goal_heading:.2f}'
        )
        return command_id

    def ensure_spot_connected(self):
        if self.spot_command_client is not None:
            return
        if not self.spot_ip:
            raise RuntimeError('spot_ip is not configured')

        try:
            import bosdyn.client
            from bosdyn.client.lease import LeaseClient, LeaseKeepAlive
            from bosdyn.client.robot_command import (
                RobotCommandClient,
                blocking_stand,
            )
            from bosdyn.client.robot_state import RobotStateClient
        except ImportError as error:
            raise RuntimeError(
                'bosdyn-client is not installed. Install requirements-field.txt '
                'on the Jetson before using ROBOT_GOAL_BACKEND=spot_sdk.'
            ) from error

        username = self.spot_username or os.environ.get('BOSDYN_CLIENT_USERNAME')
        password = self.spot_password or os.environ.get('BOSDYN_CLIENT_PASSWORD')
        if not username or not password:
            raise RuntimeError(
                'Spot credentials are not configured. Set SPOT_USERNAME and '
                'SPOT_PASSWORD in config/field.env or BOSDYN_CLIENT_USERNAME/'
                'BOSDYN_CLIENT_PASSWORD in the environment.'
            )

        sdk = bosdyn.client.create_standard_sdk('InfrastructureInspectionClient')
        self.spot = sdk.create_robot(self.spot_ip)
        self.spot.authenticate(username, password)
        self.spot.time_sync.wait_for_sync()

        lease_client = self.spot.ensure_client(LeaseClient.default_service_name)
        self.spot_lease_keepalive = LeaseKeepAlive(
            lease_client,
            must_acquire=True,
            return_at_exit=self.spot_auto_return_lease,
        )
        self.spot_command_client = self.spot.ensure_client(
            RobotCommandClient.default_service_name
        )
        # Read-only: used once per command to express a body-relative goal
        # in Spot's frame. Never feeds localization or the map.
        self.spot_state_client = self.spot.ensure_client(
            RobotStateClient.default_service_name
        )

        if self.spot_auto_power_on:
            self.publish_status('powering on Spot motors')
            self.spot.power_on(timeout_sec=20)
        if self.spot_stand_before_move:
            self.publish_status('commanding Spot to stand')
            blocking_stand(self.spot_command_client, timeout_sec=10)
        self.publish_status(f'connected to Spot at {self.spot_ip}')

    def wait_for_spot_goal(self, command_id):
        try:
            from bosdyn.client.robot_command import block_for_trajectory_cmd
        except ImportError:
            block_for_trajectory_cmd = None

        if block_for_trajectory_cmd is None:
            time.sleep(min(self.spot_goal_duration, self.spot_arrival_timeout))
            self.confirm_arrival_or_finish('spot sdk command duration elapsed')
            return

        result = block_for_trajectory_cmd(
            self.spot_command_client,
            command_id,
            timeout_sec=self.spot_arrival_timeout,
            logger=self.get_logger(),
        )
        self.confirm_arrival_or_finish(f'spot sdk trajectory completed: {result}')

    def dry_run_arrived(self):
        if self.active_goal is None:
            return
        if self.dry_run_timer is not None:
            self.dry_run_timer.cancel()
            self.dry_run_timer = None
        self.confirm_arrival_or_finish('dry-run command elapsed')

    def confirm_arrival_or_finish(self, detail):
        if self.arrival_check_source != 'tf':
            self.finish_goal(detail)
            return
        self.publish_status(f'{detail}; verifying arrival with TF')
        if self.arrival_timer is not None:
            self.arrival_timer.cancel()
        self.arrival_timer = self.create_timer(
            max(0.05, self.arrival_check_period),
            self.arrival_check_tick,
        )

    def arrival_check_tick(self):
        if self.active_goal is None:
            self.cancel_arrival_timer()
            return

        if (
            self.active_goal_started_sec is not None
            and time.monotonic() - self.active_goal_started_sec
            > self.spot_arrival_timeout
        ):
            self.cancel_arrival_timer()
            self.finish_goal('arrival TF check timed out', arrived=False)
            return

        error = self.arrival_error(self.active_goal)
        if error is None:
            self.arrival_stable_since = None
            return
        distance_error, yaw_error = error
        self.publish_status(
            'arrival check: '
            f'distance={distance_error:.2f}m, yaw={yaw_error:.2f}rad'
        )
        if (
            distance_error <= self.arrival_position_tolerance
            and yaw_error <= self.arrival_yaw_tolerance
        ):
            if self.arrival_stable_since is None:
                self.arrival_stable_since = time.monotonic()
            if time.monotonic() - self.arrival_stable_since >= self.arrival_stable_sec:
                self.cancel_arrival_timer()
                self.finish_goal(
                    'arrival verified by OAK/TF '
                    f'(distance={distance_error:.2f}m, yaw={yaw_error:.2f}rad)'
                )
            return

        self.arrival_stable_since = None

    def arrival_error(self, goal):
        try:
            transform = self.tf_buffer.lookup_transform(
                goal.header.frame_id,
                self.arrival_base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as error:
            self.publish_status(f'waiting for arrival TF: {error}')
            return None

        position = transform.transform.translation
        dx = goal.pose.position.x - position.x
        dy = goal.pose.position.y - position.y
        distance_error = math.hypot(dx, dy)
        current_yaw = yaw_from_quaternion(transform.transform.rotation)
        goal_yaw = yaw_from_quaternion(goal.pose.orientation)
        yaw_error = abs(normalize_angle(goal_yaw - current_yaw))
        return distance_error, yaw_error

    def cancel_arrival_timer(self):
        if self.arrival_timer is not None:
            self.arrival_timer.cancel()
            self.arrival_timer = None

    def finish_goal(self, detail, arrived=True):
        self.publish_status(detail)
        if arrived:
            message = String()
            message.data = detail
            self.arrival_publisher.publish(message)
        self.active_goal = None
        self.active_goal_started_sec = None
        self.arrival_stable_since = None

    def publish_status(self, message):
        status = String()
        status.data = message
        self.status_publisher.publish(status)
        self.get_logger().info(message)

    def destroy_node(self):
        if self.spot_lease_keepalive is not None:
            self.spot_lease_keepalive.shutdown()
            self.spot_lease_keepalive = None
        super().destroy_node()


def yaw_from_quaternion(rotation):
    x = rotation.x
    y = rotation.y
    z = rotation.z
    w = rotation.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def main(args=None):
    rclpy.init(args=args)
    node = RobotGoalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
