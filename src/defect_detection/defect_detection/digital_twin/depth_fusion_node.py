import math

from cv_bridge import CvBridge
import message_filters
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import (
    BoundingBox3D,
    Detection2DArray,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)


def detection_confidence(detection):
    if not detection.results:
        return 0.0
    return float(detection.results[0].hypothesis.score)


def detection_class_id(detection):
    if not detection.results:
        return 'unknown'
    return detection.results[0].hypothesis.class_id or 'unknown'


def bbox_bounds(detection, image_width, image_height, padding_px=0):
    center_x = float(detection.bbox.center.position.x)
    center_y = float(detection.bbox.center.position.y)
    half_width = float(detection.bbox.size_x) / 2.0 + padding_px
    half_height = float(detection.bbox.size_y) / 2.0 + padding_px
    x1 = max(0, int(math.floor(center_x - half_width)))
    y1 = max(0, int(math.floor(center_y - half_height)))
    x2 = min(image_width, int(math.ceil(center_x + half_width)))
    y2 = min(image_height, int(math.ceil(center_y + half_height)))
    return x1, y1, x2, y2


def depth_image_to_meters(depth_image, encoding):
    if encoding in {'16UC1', 'mono16'}:
        return depth_image.astype(np.float32) / 1000.0
    if encoding in {'32FC1', 'passthrough'}:
        return depth_image.astype(np.float32)
    return depth_image.astype(np.float32)


def robust_depth(depth_meters):
    finite = depth_meters[np.isfinite(depth_meters)]
    finite = finite[finite > 0.05]
    if finite.size == 0:
        return None
    low, high = np.percentile(finite, [15, 85])
    clipped = finite[(finite >= low) & (finite <= high)]
    if clipped.size == 0:
        clipped = finite
    return float(np.median(clipped))


def project_pixel_to_camera(u, v, depth_m, camera_matrix):
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    x = (u - cx) * depth_m / fx
    y = (v - cy) * depth_m / fy
    z = depth_m
    return x, y, z


class DepthFusionNode(Node):

    def __init__(self):
        super().__init__('depth_detection_fusion')

        self.declare_parameter('detections_2d_topic', '/detections_2d')
        self.declare_parameter('depth_topic', '/oak/rgb/depth')
        self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('detections_3d_topic', '/detections_3d')
        self.declare_parameter('minimum_confidence', 0.50)
        self.declare_parameter('bbox_padding_px', 4)
        self.declare_parameter('default_bbox_size_m', 0.20)
        self.declare_parameter('sync_queue_size', 30)
        self.declare_parameter('sync_slop_sec', 0.12)

        detections_2d_topic = self.get_parameter(
            'detections_2d_topic'
        ).get_parameter_value().string_value
        depth_topic = self.get_parameter(
            'depth_topic'
        ).get_parameter_value().string_value
        camera_info_topic = self.get_parameter(
            'camera_info_topic'
        ).get_parameter_value().string_value
        detections_3d_topic = self.get_parameter(
            'detections_3d_topic'
        ).get_parameter_value().string_value
        self.minimum_confidence = self.get_parameter(
            'minimum_confidence'
        ).get_parameter_value().double_value
        self.bbox_padding_px = self.get_parameter(
            'bbox_padding_px'
        ).get_parameter_value().integer_value
        self.default_bbox_size_m = self.get_parameter(
            'default_bbox_size_m'
        ).get_parameter_value().double_value
        sync_queue_size = self.get_parameter(
            'sync_queue_size'
        ).get_parameter_value().integer_value
        sync_slop_sec = self.get_parameter(
            'sync_slop_sec'
        ).get_parameter_value().double_value

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.camera_info_subscription = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.detections_3d_publisher = self.create_publisher(
            Detection3DArray,
            detections_3d_topic,
            10,
        )

        self.detections_sub = message_filters.Subscriber(
            self,
            Detection2DArray,
            detections_2d_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            depth_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.detections_sub, self.depth_sub],
            queue_size=sync_queue_size,
            slop=sync_slop_sec,
        )
        self.synchronizer.registerCallback(self.synchronized_callback)

        self.get_logger().info(
            'Depth camera RGB-D fusion enabled. '
            f'detections={detections_2d_topic}, depth={depth_topic}, '
            f'camera_info={camera_info_topic}, output={detections_3d_topic}'
        )

    def camera_info_callback(self, message):
        self.camera_matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)

    def synchronized_callback(self, detections_msg, depth_msg):
        if self.camera_matrix is None:
            self.get_logger().warning('Waiting for depth camera_info intrinsics')
            return

        try:
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
        except Exception as error:
            self.get_logger().warning(f'Could not decode the depth image: {error}')
            return

        depth_meters = depth_image_to_meters(depth_image, depth_msg.encoding)
        height, width = depth_meters.shape[:2]
        output = Detection3DArray()
        output.header = depth_msg.header

        for detection in detections_msg.detections:
            confidence = detection_confidence(detection)
            if confidence < self.minimum_confidence:
                continue
            converted = self.convert_detection(detection, depth_meters, width, height)
            if converted is not None:
                converted.header = depth_msg.header
                output.detections.append(converted)

        self.detections_3d_publisher.publish(output)

    def convert_detection(self, detection, depth_meters, image_width, image_height):
        x1, y1, x2, y2 = bbox_bounds(
            detection,
            image_width,
            image_height,
            padding_px=self.bbox_padding_px,
        )
        if x2 <= x1 or y2 <= y1:
            return None

        depth = robust_depth(depth_meters[y1:y2, x1:x2])
        if depth is None:
            return None

        u = float(detection.bbox.center.position.x)
        v = float(detection.bbox.center.position.y)
        x, y, z = project_pixel_to_camera(u, v, depth, self.camera_matrix)

        detection_3d = Detection3D()
        result = ObjectHypothesisWithPose()
        result.hypothesis.class_id = detection_class_id(detection)
        result.hypothesis.score = detection_confidence(detection)
        result.pose.pose.position.x = x
        result.pose.pose.position.y = y
        result.pose.pose.position.z = z
        result.pose.pose.orientation.w = 1.0
        detection_3d.results.append(result)

        detection_3d.bbox = BoundingBox3D()
        detection_3d.bbox.center.position.x = x
        detection_3d.bbox.center.position.y = y
        detection_3d.bbox.center.position.z = z
        detection_3d.bbox.center.orientation.w = 1.0
        size = max(0.02, self.default_bbox_size_m)
        detection_3d.bbox.size.x = size
        detection_3d.bbox.size.y = size
        detection_3d.bbox.size.z = size
        return detection_3d


def main(args=None):
    rclpy.init(args=args)
    node = DepthFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
