import os
import time

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class SpotLocalizationBridge(Node):
    """Publish Spot kinematic odometry as TF and nav_msgs/Odometry.

    Polls the Boston Dynamics SDK robot state service and republishes
    Spot's own world-frame pose as the mission TF. This is the frame the
    whole stack lives in: the occupancy map accumulates in it, goals are
    computed in it, and the mission's start pose is recorded in it, so
    the accuracy of the walk home follows directly from which Spot frame
    is selected here.
    """

    def __init__(self):
        super().__init__('spot_localization_bridge')

        self.declare_parameter('spot_ip', '')
        self.declare_parameter('spot_username', '')
        self.declare_parameter('spot_password', '')
        # Spot publishes two world frames. 'odom' is smooth but drifts;
        # 'vision' is drift-corrected against Spot's own cameras, which is
        # what makes a long walk back to the start pose land accurately.
        self.declare_parameter('spot_frame', 'vision')
        self.declare_parameter('odom_frame', 'vision')
        self.declare_parameter('base_frame', 'body')
        self.declare_parameter('odom_topic', '/spot/odom')
        self.declare_parameter('publish_rate_hz', 10.0)
        self.declare_parameter('reconnect_interval_sec', 5.0)

        self.spot_ip = self.get_parameter(
            'spot_ip'
        ).get_parameter_value().string_value
        self.spot_username = self.get_parameter(
            'spot_username'
        ).get_parameter_value().string_value
        self.spot_password = self.get_parameter(
            'spot_password'
        ).get_parameter_value().string_value
        self.spot_frame = self.get_parameter(
            'spot_frame'
        ).get_parameter_value().string_value
        if self.spot_frame not in {'vision', 'odom'}:
            raise ValueError("spot_frame must be 'vision' or 'odom'")
        self.odom_frame = self.get_parameter(
            'odom_frame'
        ).get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame'
        ).get_parameter_value().string_value
        odom_topic = self.get_parameter(
            'odom_topic'
        ).get_parameter_value().string_value
        publish_rate = self.get_parameter(
            'publish_rate_hz'
        ).get_parameter_value().double_value
        self.reconnect_interval = self.get_parameter(
            'reconnect_interval_sec'
        ).get_parameter_value().double_value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, odom_topic, 10)
        self.state_client = None
        self.last_connect_attempt = 0.0
        self.timer = self.create_timer(
            1.0 / max(0.1, publish_rate),
            self.tick,
        )
        self.get_logger().info(
            f'Bridging Spot {self.spot_frame} odometry at '
            f'{self.spot_ip or "<unset>"} to '
            f'{self.odom_frame}->{self.base_frame} TF and {odom_topic}'
        )

    def connect(self):
        if not self.spot_ip:
            raise RuntimeError('spot_ip is not configured')

        try:
            import bosdyn.client
            from bosdyn.client.robot_state import RobotStateClient
        except ImportError as error:
            raise RuntimeError(
                'bosdyn-client is not installed. Install requirements-field.txt '
                'on the Jetson before using SPOT_LOCALIZATION=true.'
            ) from error

        username = self.spot_username or os.environ.get('BOSDYN_CLIENT_USERNAME')
        password = self.spot_password or os.environ.get('BOSDYN_CLIENT_PASSWORD')
        if not username or not password:
            raise RuntimeError(
                'Spot credentials are not configured. Set SPOT_USERNAME and '
                'SPOT_PASSWORD in config/field.env or BOSDYN_CLIENT_USERNAME/'
                'BOSDYN_CLIENT_PASSWORD in the environment.'
            )

        sdk = bosdyn.client.create_standard_sdk('SpotLocalizationBridge')
        robot = sdk.create_robot(self.spot_ip)
        robot.authenticate(username, password)
        robot.time_sync.wait_for_sync()
        self.state_client = robot.ensure_client(
            RobotStateClient.default_service_name
        )
        self.get_logger().info(f'Connected to Spot at {self.spot_ip}')

    def tick(self):
        if self.state_client is None:
            now = time.monotonic()
            if now - self.last_connect_attempt < self.reconnect_interval:
                return
            self.last_connect_attempt = now
            try:
                self.connect()
            except Exception as error:
                self.get_logger().warning(
                    f'Cannot connect to Spot: {error}',
                    throttle_duration_sec=10.0,
                )
                return

        try:
            state = self.state_client.get_robot_state()
        except Exception as error:
            self.get_logger().warning(
                f'Spot state request failed; reconnecting: {error}',
                throttle_duration_sec=10.0,
            )
            self.state_client = None
            return
        self.publish_odometry(state)

    def odom_tform_body(self, state):
        from bosdyn.client.frame_helpers import (
            BODY_FRAME_NAME,
            ODOM_FRAME_NAME,
            VISION_FRAME_NAME,
            get_a_tform_b,
        )

        source = (
            VISION_FRAME_NAME if self.spot_frame == 'vision' else ODOM_FRAME_NAME
        )
        return get_a_tform_b(
            state.kinematic_state.transforms_snapshot,
            source,
            BODY_FRAME_NAME,
        )

    def publish_odometry(self, state):
        pose = self.odom_tform_body(state)
        if pose is None:
            self.get_logger().warning(
                'Spot state has no odom_tform_body transform',
                throttle_duration_sec=10.0,
            )
            return
        stamp = self.get_clock().now().to_msg()

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.odom_frame
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x = float(pose.x)
        transform.transform.translation.y = float(pose.y)
        transform.transform.translation.z = float(pose.z)
        transform.transform.rotation.x = float(pose.rot.x)
        transform.transform.rotation.y = float(pose.rot.y)
        transform.transform.rotation.z = float(pose.rot.z)
        transform.transform.rotation.w = float(pose.rot.w)
        self.tf_broadcaster.sendTransform(transform)

        odometry = Odometry()
        odometry.header.stamp = stamp
        odometry.header.frame_id = self.odom_frame
        odometry.child_frame_id = self.base_frame
        odometry.pose.pose.position.x = float(pose.x)
        odometry.pose.pose.position.y = float(pose.y)
        odometry.pose.pose.position.z = float(pose.z)
        odometry.pose.pose.orientation.x = float(pose.rot.x)
        odometry.pose.pose.orientation.y = float(pose.rot.y)
        odometry.pose.pose.orientation.z = float(pose.rot.z)
        odometry.pose.pose.orientation.w = float(pose.rot.w)
        velocity = state.kinematic_state.velocity_of_body_in_odom
        odometry.twist.twist.linear.x = float(velocity.linear.x)
        odometry.twist.twist.linear.y = float(velocity.linear.y)
        odometry.twist.twist.linear.z = float(velocity.linear.z)
        odometry.twist.twist.angular.x = float(velocity.angular.x)
        odometry.twist.twist.angular.y = float(velocity.angular.y)
        odometry.twist.twist.angular.z = float(velocity.angular.z)
        self.odom_publisher.publish(odometry)


def main(args=None):
    rclpy.init(args=args)
    node = SpotLocalizationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
