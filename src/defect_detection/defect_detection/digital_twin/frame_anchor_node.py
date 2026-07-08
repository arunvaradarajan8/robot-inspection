from pathlib import Path
import math
import time

from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
import yaml


def yaw_from_quaternion(rotation):
    x = rotation.x
    y = rotation.y
    z = rotation.z
    w = rotation.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class FrameAnchorNode(Node):

    def __init__(self):
        super().__init__('digital_twin_frame_anchor')

        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_world_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('anchor_request_topic', '/digital_twin/anchor_request')
        self.declare_parameter('scan_topic', '/trimble/x7/scan_points')
        self.declare_parameter('status_topic', '/digital_twin/anchor_status')
        self.declare_parameter('store_path', '/tmp/digital_twin_anchor.yaml')
        self.declare_parameter('auto_anchor_on_first_scan', True)
        self.declare_parameter('publish_rate_hz', 5.0)

        self.map_frame = self.get_parameter(
            'map_frame'
        ).get_parameter_value().string_value
        self.robot_world_frame = self.get_parameter(
            'robot_world_frame'
        ).get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame'
        ).get_parameter_value().string_value
        anchor_request_topic = self.get_parameter(
            'anchor_request_topic'
        ).get_parameter_value().string_value
        scan_topic = self.get_parameter(
            'scan_topic'
        ).get_parameter_value().string_value
        status_topic = self.get_parameter(
            'status_topic'
        ).get_parameter_value().string_value
        self.store_path = Path(
            self.get_parameter('store_path').get_parameter_value().string_value
        ).expanduser()
        self.auto_anchor_on_first_scan = self.get_parameter(
            'auto_anchor_on_first_scan'
        ).get_parameter_value().bool_value
        publish_rate = self.get_parameter(
            'publish_rate_hz'
        ).get_parameter_value().double_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.status_publisher = self.create_publisher(String, status_topic, 10)
        self.anchor_request_subscription = self.create_subscription(
            Bool,
            anchor_request_topic,
            self.anchor_request_callback,
            10,
        )
        # Sensor-data QoS so the best-effort scan watcher publisher can
        # connect; a reliable subscription never receives those scans.
        self.scan_subscription = self.create_subscription(
            PointCloud2,
            scan_topic,
            self.scan_callback,
            qos_profile_sensor_data,
        )

        self.anchor = self.load_anchor()
        self.timer = self.create_timer(
            1.0 / max(0.1, publish_rate),
            self.publish_anchor_transform,
        )
        self.get_logger().info(
            f'Anchoring {self.map_frame} to {self.robot_world_frame}; '
            f'base={self.base_frame}, store={self.store_path}'
        )

    def load_anchor(self):
        if not self.store_path.is_file():
            return None
        with self.store_path.open('r', encoding='utf-8') as stream:
            data = yaml.safe_load(stream) or {}
        anchor = data.get('anchor')
        if anchor:
            self.publish_status('loaded anchor from disk')
        return anchor

    def save_anchor(self):
        self.store_path.parent.mkdir(parents=True, exist_ok=True)
        with self.store_path.open('w', encoding='utf-8') as stream:
            yaml.safe_dump({'anchor': self.anchor}, stream, sort_keys=True)

    def anchor_request_callback(self, message):
        if not message.data:
            return
        self.capture_anchor('manual anchor request')

    def scan_callback(self, _message):
        if self.anchor is None and self.auto_anchor_on_first_scan:
            self.capture_anchor('first Trimble reference scan')

    def capture_anchor(self, reason):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_world_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as error:
            self.publish_status(f'anchor failed: {error}')
            self.get_logger().warning(f'Cannot capture anchor: {error}')
            return

        translation = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)

        # The first scan station is the digital twin origin. Publish
        # map->robot_world as the inverse of robot_world->base at capture time,
        # so base_link is near (0, 0, 0) in map when the reference scan starts.
        inverse_yaw = -yaw
        cos_yaw = math.cos(inverse_yaw)
        sin_yaw = math.sin(inverse_yaw)
        tx = -(cos_yaw * translation.x - sin_yaw * translation.y)
        ty = -(sin_yaw * translation.x + cos_yaw * translation.y)
        tz = -translation.z

        self.anchor = {
            'map_frame': self.map_frame,
            'robot_world_frame': self.robot_world_frame,
            'base_frame': self.base_frame,
            'translation': {'x': tx, 'y': ty, 'z': tz},
            'yaw': inverse_yaw,
            'captured_reason': reason,
            'captured_sec': time.time(),
            'robot_pose_at_capture': {
                'x': translation.x,
                'y': translation.y,
                'z': translation.z,
                'yaw': yaw,
            },
        }
        self.save_anchor()
        self.publish_status(f'anchor captured: {reason}')
        self.get_logger().info(f'Captured digital twin anchor: {reason}')

    def publish_anchor_transform(self):
        if self.anchor is None:
            return

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.anchor['map_frame']
        transform.child_frame_id = self.anchor['robot_world_frame']
        translation = self.anchor['translation']
        transform.transform.translation.x = float(translation['x'])
        transform.transform.translation.y = float(translation['y'])
        transform.transform.translation.z = float(translation['z'])
        (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ) = quaternion_from_yaw(float(self.anchor['yaw']))
        self.tf_broadcaster.sendTransform(transform)

    def publish_status(self, message):
        status = String()
        status.data = message
        self.status_publisher.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = FrameAnchorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
