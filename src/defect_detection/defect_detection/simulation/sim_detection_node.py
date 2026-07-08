import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from defect_detection.simulation.world_constants import DEFECTS, WALL_PLANE_X


def quaternion_to_matrix(rotation):
    x, y, z, w = rotation.x, rotation.y, rotation.z, rotation.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class SimDetectionNode(Node):
    """YOLO stand-in for the Gazebo simulation.

    Projects the known world defect positions into the simulated camera
    (via TF and the bridged camera intrinsics) and publishes YOLO-style
    2D detections stamped to match each depth frame, so the downstream
    OAK depth fusion synchronizer works unmodified.
    """

    def __init__(self):
        super().__init__('sim_detection_node')

        self.declare_parameter('depth_topic', '/oak/rgb/depth')
        self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('detections_2d_topic', '/detections_2d')
        self.declare_parameter('world_frame', 'odom')
        self.declare_parameter('min_pixel_size', 12.0)

        self.world_frame = self.get_parameter('world_frame').value
        self.min_pixel_size = float(self.get_parameter('min_pixel_size').value)

        self.camera_matrix = None
        self.image_width = None
        self.image_height = None
        self.rng = np.random.default_rng(11)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.publisher = self.create_publisher(
            Detection2DArray,
            self.get_parameter('detections_2d_topic').value,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            self.get_parameter('camera_info_topic').value,
            self.camera_info_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            self.get_parameter('depth_topic').value,
            self.depth_callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            'Ground-truth defect projector running (YOLO stand-in for sim).'
        )

    def camera_info_callback(self, message):
        self.camera_matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        self.image_width = message.width
        self.image_height = message.height

    def depth_callback(self, depth_msg):
        if self.camera_matrix is None:
            return

        camera_frame = depth_msg.header.frame_id
        try:
            transform = self.tf_buffer.lookup_transform(
                self.world_frame,
                camera_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return

        rotation = quaternion_to_matrix(transform.transform.rotation)
        translation = transform.transform.translation
        camera_position = np.array([translation.x, translation.y, translation.z])

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        message = Detection2DArray()
        message.header.stamp = depth_msg.header.stamp
        message.header.frame_id = camera_frame

        for class_id, lateral, height, size, confidence in DEFECTS:
            world = np.array([WALL_PLANE_X, lateral, height])
            in_camera = rotation.T @ (world - camera_position)
            if in_camera[2] < 0.6:
                continue
            u = fx * in_camera[0] / in_camera[2] + cx
            v = fy * in_camera[1] / in_camera[2] + cy
            pixel_size = fx * size / in_camera[2]
            if pixel_size < self.min_pixel_size:
                continue
            margin = pixel_size / 2.0
            if not (
                margin < u < self.image_width - margin
                and margin < v < self.image_height - margin
            ):
                continue

            detection = Detection2D()
            detection.header = message.header
            jitter = self.rng.normal(0.0, 1.5, 2)
            detection.bbox.center.position.x = float(u + jitter[0])
            detection.bbox.center.position.y = float(v + jitter[1])
            detection.bbox.size_x = float(pixel_size)
            detection.bbox.size_y = float(pixel_size)
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = class_id
            hypothesis.hypothesis.score = float(
                np.clip(confidence + self.rng.normal(0.0, 0.02), 0.05, 0.99)
            )
            detection.results.append(hypothesis)
            message.detections.append(detection)

        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SimDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
