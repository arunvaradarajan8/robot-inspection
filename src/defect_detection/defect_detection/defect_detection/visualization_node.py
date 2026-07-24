from copy import deepcopy

from builtin_interfaces.msg import Duration
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    qos_profile_sensor_data,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from vision_msgs.msg import Detection3DArray
from visualization_msgs.msg import Marker, MarkerArray


MARKER_COLORS = (
    (0.95, 0.20, 0.20),
    (0.20, 0.75, 0.95),
    (0.30, 0.90, 0.35),
    (0.95, 0.65, 0.15),
    (0.70, 0.35, 0.95),
)


def duration_from_seconds(seconds):
    whole_seconds = int(seconds)
    nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
    if nanoseconds == 1_000_000_000:
        whole_seconds += 1
        nanoseconds = 0
    return Duration(sec=whole_seconds, nanosec=nanoseconds)


def color_for_class(class_id):
    try:
        color_index = int(class_id) % len(MARKER_COLORS)
    except (TypeError, ValueError):
        text = str(class_id)
        color_index = sum(ord(character) for character in text)
        color_index %= len(MARKER_COLORS)
    return MARKER_COLORS[color_index]


def detection_label(detection):
    if not detection.results:
        return 'detection'

    result = detection.results[0].hypothesis
    class_id = result.class_id or 'unknown'
    return f'{class_id} {result.score:.2f}'


def detections_to_marker_array(
    detections_msg,
    box_alpha=0.30,
    label_height=0.15,
    lifetime_sec=0.50,
):
    markers = MarkerArray()

    clear_marker = Marker()
    clear_marker.action = Marker.DELETEALL
    markers.markers.append(clear_marker)

    lifetime = duration_from_seconds(lifetime_sec)

    for index, detection in enumerate(detections_msg.detections):
        header = detection.header
        if not header.frame_id:
            header = detections_msg.header

        class_id = (
            detection.results[0].hypothesis.class_id
            if detection.results
            else 'unknown'
        )
        red, green, blue = color_for_class(class_id)

        box_marker = Marker()
        box_marker.header = header
        box_marker.ns = 'detection_boxes'
        box_marker.id = index
        box_marker.type = Marker.CUBE
        box_marker.action = Marker.ADD
        box_marker.pose = deepcopy(detection.bbox.center)
        box_marker.scale.x = max(detection.bbox.size.x, 0.01)
        box_marker.scale.y = max(detection.bbox.size.y, 0.01)
        box_marker.scale.z = max(detection.bbox.size.z, 0.01)
        box_marker.color.r = red
        box_marker.color.g = green
        box_marker.color.b = blue
        box_marker.color.a = box_alpha
        box_marker.lifetime = lifetime
        markers.markers.append(box_marker)

        label_marker = Marker()
        label_marker.header = header
        label_marker.ns = 'detection_labels'
        label_marker.id = index
        label_marker.type = Marker.TEXT_VIEW_FACING
        label_marker.action = Marker.ADD
        label_marker.pose = deepcopy(detection.bbox.center)
        label_marker.pose.position.z += (
            detection.bbox.size.z / 2.0 + label_height
        )
        label_marker.scale.z = label_height
        label_marker.color.r = red
        label_marker.color.g = green
        label_marker.color.b = blue
        label_marker.color.a = 1.0
        label_marker.text = detection_label(detection)
        label_marker.lifetime = lifetime
        markers.markers.append(label_marker)

    return markers


class DetectionVisualizationNode(Node):

    def __init__(self):
        super().__init__('detection_visualization_node')

        self.declare_parameter(
            'pointcloud_topic',
            '/cloud/points',
        )
        self.declare_parameter(
            'visualization_pointcloud_topic',
            '/rviz/pointcloud',
        )
        self.declare_parameter(
            'detections_topic',
            '/detections_3d',
        )
        self.declare_parameter(
            'markers_topic',
            '/detection_markers',
        )
        self.declare_parameter('box_alpha', 0.30)
        self.declare_parameter('label_height', 0.15)
        self.declare_parameter('marker_lifetime_sec', 0.50)

        pointcloud_topic = self.get_parameter(
            'pointcloud_topic'
        ).get_parameter_value().string_value
        visualization_pointcloud_topic = self.get_parameter(
            'visualization_pointcloud_topic'
        ).get_parameter_value().string_value
        detections_topic = self.get_parameter(
            'detections_topic'
        ).get_parameter_value().string_value
        markers_topic = self.get_parameter(
            'markers_topic'
        ).get_parameter_value().string_value
        self.box_alpha = self.get_parameter(
            'box_alpha'
        ).get_parameter_value().double_value
        self.label_height = self.get_parameter(
            'label_height'
        ).get_parameter_value().double_value
        self.marker_lifetime_sec = self.get_parameter(
            'marker_lifetime_sec'
        ).get_parameter_value().double_value

        if not 0.0 < self.box_alpha <= 1.0:
            raise ValueError('box_alpha must be in the range (0.0, 1.0]')
        if self.label_height <= 0.0:
            raise ValueError('label_height must be greater than zero')
        if self.marker_lifetime_sec < 0.0:
            raise ValueError(
                'marker_lifetime_sec must be greater than or equal to zero'
            )

        marker_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.pointcloud_publisher = self.create_publisher(
            PointCloud2,
            visualization_pointcloud_topic,
            qos_profile_sensor_data,
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            markers_topic,
            marker_qos,
        )
        self.pointcloud_subscription = self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.pointcloud_callback,
            qos_profile_sensor_data,
        )
        self.detection_subscription = self.create_subscription(
            Detection3DArray,
            detections_topic,
            self.detections_callback,
            10,
        )

        self.get_logger().info(
            f'Publishing RViz point cloud on {visualization_pointcloud_topic} '
            f'and detection markers on {markers_topic}'
        )

    def pointcloud_callback(self, pointcloud_msg):
        self.pointcloud_publisher.publish(pointcloud_msg)

    def detections_callback(self, detections_msg):
        markers = detections_to_marker_array(
            detections_msg,
            box_alpha=self.box_alpha,
            label_height=self.label_height,
            lifetime_sec=self.marker_lifetime_sec,
        )
        self.marker_publisher.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = DetectionVisualizationNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
