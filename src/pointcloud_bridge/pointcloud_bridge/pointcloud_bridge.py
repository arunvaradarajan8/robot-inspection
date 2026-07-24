from copy import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


REQUIRED_FIELDS = {'x', 'y', 'z'}


def stamp_to_nanoseconds(stamp):
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def validate_timestamp(
    stamp,
    now_nanoseconds,
    max_age_sec,
    max_future_sec,
):
    stamp_nanoseconds = stamp_to_nanoseconds(stamp)
    if stamp_nanoseconds == 0:
        raise ValueError('Point cloud has a zero acquisition timestamp')

    offset_sec = (now_nanoseconds - stamp_nanoseconds) / 1e9
    if max_age_sec > 0.0 and offset_sec > max_age_sec:
        raise ValueError(
            f'Point cloud is stale ({offset_sec:.3f}s old)'
        )
    if max_future_sec >= 0.0 and offset_sec < -max_future_sec:
        raise ValueError(
            f'Point cloud timestamp is {-offset_sec:.3f}s in the future'
        )
    return offset_sec


def normalize_cloud(cloud, output_frame, receive_stamp=None):
    field_names = {field.name for field in cloud.fields}
    missing_fields = REQUIRED_FIELDS - field_names
    if missing_fields:
        missing = ', '.join(sorted(missing_fields))
        raise ValueError(f'Point cloud is missing required fields: {missing}')

    normalized = copy(cloud)
    normalized.header = copy(cloud.header)
    if output_frame:
        normalized.header.frame_id = output_frame
    if receive_stamp is not None:
        normalized.header.stamp = receive_stamp
    return normalized


class PointCloudBridge(Node):

    def __init__(self):
        super().__init__('pointcloud_bridge')

        self.declare_parameter('input_topic', '/cloud/raw')
        self.declare_parameter(
            'output_topic',
            '/cloud/points',
        )
        self.declare_parameter('output_frame', '')
        self.declare_parameter('max_cloud_age_sec', 0.5)
        self.declare_parameter('max_future_offset_sec', 0.05)
        self.declare_parameter('timestamp_mode', 'source')

        input_topic = self.get_parameter(
            'input_topic'
        ).get_parameter_value().string_value
        output_topic = self.get_parameter(
            'output_topic'
        ).get_parameter_value().string_value
        self.output_frame = self.get_parameter(
            'output_frame'
        ).get_parameter_value().string_value
        self.max_cloud_age = self.get_parameter(
            'max_cloud_age_sec'
        ).get_parameter_value().double_value
        self.max_future_offset = self.get_parameter(
            'max_future_offset_sec'
        ).get_parameter_value().double_value
        self.timestamp_mode = self.get_parameter(
            'timestamp_mode'
        ).get_parameter_value().string_value

        if self.timestamp_mode not in {'source', 'receive'}:
            raise ValueError(
                'timestamp_mode must be either "source" or "receive"'
            )

        self.publisher = self.create_publisher(
            PointCloud2,
            output_topic,
            qos_profile_sensor_data,
        )
        self.subscription = self.create_subscription(
            PointCloud2,
            input_topic,
            self.cloud_callback,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f'Bridging {input_topic} to {output_topic} '
            f'with {self.timestamp_mode} timestamps'
        )

    def cloud_callback(self, cloud):
        receive_time = self.get_clock().now()

        if self.timestamp_mode == 'source':
            try:
                validate_timestamp(
                    cloud.header.stamp,
                    receive_time.nanoseconds,
                    self.max_cloud_age,
                    self.max_future_offset,
                )
            except ValueError as error:
                self.get_logger().warning(f'Dropping point cloud: {error}')
                return
            receive_stamp = None
        else:
            receive_stamp = receive_time.to_msg()

        try:
            normalized = normalize_cloud(
                cloud,
                self.output_frame,
                receive_stamp=receive_stamp,
            )
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        self.publisher.publish(normalized)


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
