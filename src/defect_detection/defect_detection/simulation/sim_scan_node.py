from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header

from defect_detection.simulation import world_model


class SimScanNode(Node):
    """Trimble X7 stand-in for the Gazebo simulation.

    When the pipeline requests a scan, waits a short "scanning" delay and
    writes a synthetic LAS file of the shared world geometry into the
    watched scan folder, so the real trimble_scan_watcher picks it up
    exactly as it would a real Perspective export. Also publishes a low
    density ambient cloud for RViz.
    """

    def __init__(self):
        super().__init__('sim_scan_node')

        self.declare_parameter('scan_required_topic', '/digital_twin/scan_required')
        self.declare_parameter('scan_topic', '/trimble/x7/scan_points')
        self.declare_parameter('scan_directory', '/tmp/synthetic_demo/trimble_scans')
        self.declare_parameter('scan_frame', 'map')
        self.declare_parameter('pointcloud_topic', '/cloud/points')
        self.declare_parameter('ambient_cloud_frame', 'odom')
        self.declare_parameter('scan_write_delay_sec', 2.0)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.scan_directory = Path(
            self.get_parameter('scan_directory').value
        ).expanduser()
        self.scan_frame = self.get_parameter('scan_frame').value
        self.ambient_frame = self.get_parameter('ambient_cloud_frame').value
        self.scan_write_delay = float(
            self.get_parameter('scan_write_delay_sec').value
        )

        self.rng = np.random.default_rng(23)
        self.scan_write_due = None
        self.scan_count = 0

        self.cloud_publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter('pointcloud_topic').value,
            qos_profile_sensor_data,
        )
        self.scan_publisher = self.create_publisher(
            PointCloud2, self.scan_topic, qos_profile_sensor_data,
        )
        self.create_subscription(
            Bool,
            self.get_parameter('scan_required_topic').value,
            self.scan_required_callback,
            10,
        )
        self.cloud_timer = self.create_timer(0.5, self.publish_environment_cloud)
        self.scan_timer = self.create_timer(0.25, self.scan_tick)

        self.get_logger().info(
            f'Simulated Trimble X7 ready; LAS output: {self.scan_directory}'
        )

    def scan_required_callback(self, message):
        if not message.data or self.scan_write_due is not None:
            return
        self.scan_write_due = time.monotonic() + self.scan_write_delay
        self.get_logger().info(
            'Scan requested; simulated Trimble X7 is scanning...'
        )

    def scan_tick(self):
        if (
            self.scan_write_due is not None
            and time.monotonic() >= self.scan_write_due
        ):
            self.scan_write_due = None
            self.emit_reference_scan()

    def publish_environment_cloud(self):
        points = world_model.structure_points(
            self.rng, density=0.25,
        ).astype(np.float32)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.ambient_frame
        self.cloud_publisher.publish(
            point_cloud2.create_cloud_xyz32(header, points)
        )

    def emit_reference_scan(self):
        points = world_model.structure_points(self.rng, density=8.0)
        self.scan_count += 1
        try:
            self.scan_directory.mkdir(parents=True, exist_ok=True)
            target = self.scan_directory / f'sim_scan_{self.scan_count:03d}.las'
            world_model.write_las(points, target)
        except ImportError:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.scan_frame
            self.scan_publisher.publish(
                point_cloud2.create_cloud_xyz32(header, points.astype(np.float32))
            )
            self.get_logger().warning(
                'laspy is not installed; published the synthetic scan '
                f'directly on {self.scan_topic} instead of writing a LAS file.'
            )
            return
        self.get_logger().info(
            f'Simulated Trimble scan written: {target} '
            f'({len(points)} points); the scan watcher will publish it.'
        )


def main(args=None):
    rclpy.init(args=args)
    node = SimScanNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
