import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2DArray


def detection_confidence(detection):
    if not detection.results:
        return 0.0
    return max(float(result.hypothesis.score) for result in detection.results)


def count_high_confidence_detections(detections, threshold):
    return sum(
        1
        for detection in detections
        if detection_confidence(detection) >= threshold
    )


class ScanDecisionNode(Node):

    def __init__(self):
        super().__init__('digital_twin_scan_decision')

        self.declare_parameter('detections_topic', '/detections_2d')
        self.declare_parameter('scan_required_topic', '/digital_twin/scan_required')
        self.declare_parameter('scan_reason_topic', '/digital_twin/scan_reason')
        self.declare_parameter('confidence_threshold', 0.65)
        self.declare_parameter('min_detections', 1)
        self.declare_parameter('decision_period_sec', 2.0)
        self.declare_parameter('detection_timeout_sec', 3.0)
        self.declare_parameter('scan_cooldown_sec', 60.0)
        self.declare_parameter('min_scan_separation_m', 2.0)
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')

        detections_topic = self.get_parameter(
            'detections_topic'
        ).get_parameter_value().string_value
        scan_required_topic = self.get_parameter(
            'scan_required_topic'
        ).get_parameter_value().string_value
        scan_reason_topic = self.get_parameter(
            'scan_reason_topic'
        ).get_parameter_value().string_value
        self.confidence_threshold = self.get_parameter(
            'confidence_threshold'
        ).get_parameter_value().double_value
        self.min_detections = self.get_parameter(
            'min_detections'
        ).get_parameter_value().integer_value
        decision_period = self.get_parameter(
            'decision_period_sec'
        ).get_parameter_value().double_value
        self.detection_timeout = self.get_parameter(
            'detection_timeout_sec'
        ).get_parameter_value().double_value
        self.scan_cooldown = self.get_parameter(
            'scan_cooldown_sec'
        ).get_parameter_value().double_value
        self.min_scan_separation = self.get_parameter(
            'min_scan_separation_m'
        ).get_parameter_value().double_value
        self.target_frame = self.get_parameter(
            'target_frame'
        ).get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame'
        ).get_parameter_value().string_value

        if self.min_detections <= 0:
            raise ValueError('min_detections must be greater than zero')
        if decision_period <= 0.0:
            raise ValueError('decision_period_sec must be greater than zero')

        self.last_detection_time = None
        self.last_scan_request_time = None
        self.high_confidence_count = 0
        self.total_count = 0
        self.last_max_confidence = 0.0
        self.scan_positions = []

        self.tf_buffer = None
        self.tf_listener = None
        if self.min_scan_separation > 0.0:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_required_publisher = self.create_publisher(
            Bool,
            scan_required_topic,
            10,
        )
        self.scan_reason_publisher = self.create_publisher(
            String,
            scan_reason_topic,
            10,
        )
        # Sensor-data QoS so best-effort detection publishers (the YOLO
        # node publishes with sensor QoS) can connect.
        self.subscription = self.create_subscription(
            Detection2DArray,
            detections_topic,
            self.detections_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(decision_period, self.evaluate)
        self.get_logger().info(
            f'Gating X7 scans from {detections_topic}: '
            f'threshold={self.confidence_threshold:.2f}, '
            f'min_detections={self.min_detections}, '
            f'min_scan_separation={self.min_scan_separation:.1f}m'
        )

    def detections_callback(self, detections_msg):
        self.last_detection_time = time.monotonic()
        self.total_count = len(detections_msg.detections)
        self.high_confidence_count = count_high_confidence_detections(
            detections_msg.detections,
            self.confidence_threshold,
        )
        if detections_msg.detections:
            self.last_max_confidence = max(
                detection_confidence(detection)
                for detection in detections_msg.detections
            )
        else:
            self.last_max_confidence = 0.0

    def evaluate(self):
        now = time.monotonic()
        robot_x, robot_y = self.robot_position()
        should_scan, reason = self.scan_decision(now, robot_x, robot_y)
        self.publish_decision(should_scan, reason)
        if should_scan:
            self.last_scan_request_time = now
            if robot_x is not None:
                self.scan_positions.append((robot_x, robot_y))
            self.get_logger().info(reason)
        else:
            self.get_logger().debug(reason)

    def robot_position(self):
        if self.tf_buffer is None:
            return None, None
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as error:
            self.get_logger().warning(
                f'Cannot locate {self.base_frame} in {self.target_frame} '
                f'for scan separation check: {error}',
                throttle_duration_sec=10.0,
            )
            return None, None
        translation = transform.transform.translation
        return translation.x, translation.y

    def nearest_scan_distance(self, robot_x, robot_y):
        if not self.scan_positions:
            return None
        return min(
            math.hypot(robot_x - x, robot_y - y)
            for x, y in self.scan_positions
        )

    def scan_decision(self, now, robot_x=None, robot_y=None):
        if self.last_detection_time is None:
            return False, 'Skipping X7 scan: no detections received yet'

        age = now - self.last_detection_time
        if age > self.detection_timeout:
            return False, (
                'Skipping X7 scan: detections are stale '
                f'({age:.1f}s old)'
            )

        if self.high_confidence_count < self.min_detections:
            return False, (
                'Skipping X7 scan: no high-confidence defects '
                f'({self.high_confidence_count}/{self.total_count}, '
                f'max_confidence={self.last_max_confidence:.2f}, '
                f'threshold={self.confidence_threshold:.2f})'
            )

        if self.last_scan_request_time is not None:
            cooldown_remaining = (
                self.scan_cooldown - (now - self.last_scan_request_time)
            )
            if cooldown_remaining > 0.0:
                return False, (
                    'Skipping X7 scan: scan cooldown active '
                    f'({cooldown_remaining:.1f}s remaining)'
                )

        if self.min_scan_separation > 0.0 and robot_x is not None:
            nearest = self.nearest_scan_distance(robot_x, robot_y)
            if nearest is not None and nearest < self.min_scan_separation:
                return False, (
                    'Skipping X7 scan: too close to a previous scan '
                    f'({nearest:.1f}m away, minimum separation '
                    f'{self.min_scan_separation:.1f}m)'
                )

        return True, (
            'Requesting X7 scan: '
            f'{self.high_confidence_count} high-confidence defect(s) '
            f'from {self.total_count} detection(s)'
        )

    def publish_decision(self, should_scan, reason):
        decision = Bool()
        decision.data = should_scan
        self.scan_required_publisher.publish(decision)

        message = String()
        message.data = reason
        self.scan_reason_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = ScanDecisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
