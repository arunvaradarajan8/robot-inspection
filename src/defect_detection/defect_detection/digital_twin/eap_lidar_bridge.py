import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header


class EapLidarBridge(Node):
    """Publish the Spot EAP lidar cloud as a ROS PointCloud2.

    Polls the Boston Dynamics point cloud service for the EAP scanner,
    transforms the points into Spot's own world frame with the snapshot
    that shipped with the cloud, and republishes them. The occupancy
    builder fuses this with the depth camera cloud, so exploration no
    longer depends on Trimble scans coming back from the field.
    """

    def __init__(self):
        super().__init__('eap_lidar_bridge')

        self.declare_parameter('spot_ip', '')
        self.declare_parameter('spot_username', '')
        self.declare_parameter('spot_password', '')
        self.declare_parameter('point_cloud_service', 'velodyne-point-cloud')
        self.declare_parameter('point_cloud_source', 'velodyne-point-cloud')
        self.declare_parameter('output_topic', '/eap/points')
        # Spot's drift-corrected world frame. Must match the frame the
        # localization bridge publishes, or the map will smear.
        self.declare_parameter('target_frame', 'vision')
        self.declare_parameter('publish_rate_hz', 5.0)
        self.declare_parameter('reconnect_interval_sec', 5.0)
        self.declare_parameter('max_points', 200000)

        self.spot_ip = self.get_parameter(
            'spot_ip'
        ).get_parameter_value().string_value
        self.spot_username = self.get_parameter(
            'spot_username'
        ).get_parameter_value().string_value
        self.spot_password = self.get_parameter(
            'spot_password'
        ).get_parameter_value().string_value
        self.point_cloud_service = self.get_parameter(
            'point_cloud_service'
        ).get_parameter_value().string_value
        self.point_cloud_source = self.get_parameter(
            'point_cloud_source'
        ).get_parameter_value().string_value
        output_topic = self.get_parameter(
            'output_topic'
        ).get_parameter_value().string_value
        self.target_frame = self.get_parameter(
            'target_frame'
        ).get_parameter_value().string_value
        publish_rate = self.get_parameter(
            'publish_rate_hz'
        ).get_parameter_value().double_value
        self.reconnect_interval = self.get_parameter(
            'reconnect_interval_sec'
        ).get_parameter_value().double_value
        self.max_points = self.get_parameter(
            'max_points'
        ).get_parameter_value().integer_value

        if self.max_points <= 0:
            raise ValueError('max_points must be greater than zero')

        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            qos_profile_sensor_data,
        )
        self.point_cloud_client = None
        self.last_connect_attempt = 0.0
        self.timer = self.create_timer(
            1.0 / max(0.1, publish_rate),
            self.tick,
        )
        self.get_logger().info(
            f'Bridging Spot EAP lidar at {self.spot_ip or "<unset>"} '
            f'({self.point_cloud_source}) to {output_topic} '
            f'in the {self.target_frame} frame'
        )

    def connect(self):
        if not self.spot_ip:
            raise RuntimeError('spot_ip is not configured')

        try:
            import bosdyn.client
            from bosdyn.client.point_cloud import PointCloudClient
        except ImportError as error:
            raise RuntimeError(
                'bosdyn-client is not installed. Install requirements-field.txt '
                'on the Jetson before using EAP_LIDAR=true.'
            ) from error

        username = self.spot_username or os.environ.get('BOSDYN_CLIENT_USERNAME')
        password = self.spot_password or os.environ.get('BOSDYN_CLIENT_PASSWORD')
        if not username or not password:
            raise RuntimeError(
                'Spot credentials are not configured. Set SPOT_USERNAME and '
                'SPOT_PASSWORD in config/field.env or BOSDYN_CLIENT_USERNAME/'
                'BOSDYN_CLIENT_PASSWORD in the environment.'
            )

        sdk = bosdyn.client.create_standard_sdk('EapLidarBridge')
        robot = sdk.create_robot(self.spot_ip)
        robot.authenticate(username, password)
        robot.time_sync.wait_for_sync()
        self.point_cloud_client = robot.ensure_client(
            self.point_cloud_service or PointCloudClient.default_service_name
        )
        self.get_logger().info(
            f'Connected to the Spot EAP point cloud service at {self.spot_ip}'
        )

    def tick(self):
        if self.point_cloud_client is None:
            now = time.monotonic()
            if now - self.last_connect_attempt < self.reconnect_interval:
                return
            self.last_connect_attempt = now
            try:
                self.connect()
            except Exception as error:
                self.get_logger().warning(
                    f'Cannot connect to the Spot EAP lidar: {error}',
                    throttle_duration_sec=10.0,
                )
                return

        try:
            responses = self.point_cloud_client.get_point_cloud_from_sources(
                [self.point_cloud_source]
            )
        except Exception as error:
            self.get_logger().warning(
                f'EAP point cloud request failed; reconnecting: {error}',
                throttle_duration_sec=10.0,
            )
            self.point_cloud_client = None
            return

        if not responses:
            self.get_logger().warning(
                f'No EAP point cloud returned for source '
                f'{self.point_cloud_source}',
                throttle_duration_sec=10.0,
            )
            return
        self.publish_cloud(responses[0].point_cloud)

    @staticmethod
    def decode_points(point_cloud):
        """Decode a bosdyn PointCloud proto into an Nx3 float array."""
        if not point_cloud.data:
            return np.empty((0, 3), dtype=np.float64)
        points = np.frombuffer(point_cloud.data, dtype=np.float32)
        usable = (points.size // 3) * 3
        return points[:usable].reshape((-1, 3)).astype(np.float64)

    def transform_points(self, point_cloud, points):
        """Move points from the sensor frame into the mission world frame."""
        from bosdyn.client.frame_helpers import get_a_tform_b

        sensor_frame = point_cloud.source.frame_name_sensor
        target_tform_sensor = get_a_tform_b(
            point_cloud.source.transforms_snapshot,
            self.target_frame,
            sensor_frame,
        )
        if target_tform_sensor is None:
            self.get_logger().warning(
                f'EAP cloud has no {self.target_frame} <- {sensor_frame} '
                'transform; dropping the scan',
                throttle_duration_sec=10.0,
            )
            return None

        rotation = np.array(
            target_tform_sensor.rotation.to_matrix(),
            dtype=np.float64,
        ).reshape((3, 3))
        translation = np.array(
            [
                target_tform_sensor.position.x,
                target_tform_sensor.position.y,
                target_tform_sensor.position.z,
            ],
            dtype=np.float64,
        )
        return points @ rotation.T + translation

    def decimate(self, points):
        if len(points) <= self.max_points:
            return points
        # Uniform stride keeps the angular structure of a spinning lidar
        # far better than taking the first max_points returns.
        stride = int(np.ceil(len(points) / self.max_points))
        return points[::stride]

    def publish_cloud(self, point_cloud):
        points = self.decode_points(point_cloud)
        if points.size == 0:
            self.get_logger().warning(
                'EAP point cloud was empty',
                throttle_duration_sec=10.0,
            )
            return

        points = self.transform_points(point_cloud, points)
        if points is None:
            return
        points = self.decimate(points)

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame
        message = point_cloud2.create_cloud_xyz32(
            header,
            points.astype(np.float32),
        )
        self.publisher.publish(message)
        self.get_logger().debug(f'Published {len(points)} EAP lidar points')


def main(args=None):
    rclpy.init(args=args)
    node = EapLidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
