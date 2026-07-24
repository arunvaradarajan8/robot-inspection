from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from ultralytics import YOLO
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)
import yaml


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        package_share = Path(get_package_share_directory('defect_detection'))
        self.declare_parameter(
            'dataset_path',
            str(package_share / 'models' / 'dataset.yaml'),
        )
        self.declare_parameter(
            'model_path',
            str(package_share / 'models' / 'yolov11m.engine'),
        )
        self.declare_parameter('image_topic', '/ros2_image')
        self.declare_parameter('detections_topic', '/detections_2d')

        dataset_path = Path(
            self.get_parameter(
                'dataset_path'
            ).get_parameter_value().string_value
        ).expanduser()
        model_path = Path(
            self.get_parameter(
                'model_path'
            ).get_parameter_value().string_value
        ).expanduser()
        image_topic = self.get_parameter(
            'image_topic'
        ).get_parameter_value().string_value
        detections_topic = self.get_parameter(
            'detections_topic'
        ).get_parameter_value().string_value

        if not dataset_path.is_file():
            raise FileNotFoundError(
                f'Dataset configuration not found: {dataset_path}'
            )
        if not model_path.is_file():
            raise FileNotFoundError(f'YOLO model not found: {model_path}')

        with dataset_path.open('r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
        names = data['names']

        if isinstance(names, dict):
            self.classes = [
                names[index] for index in sorted(names.keys())
            ]
        else:
            self.classes = list(names)

        self.model = YOLO(str(model_path))
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.detections_pub = self.create_publisher(
            Detection2DArray,
            detections_topic,
            qos_profile_sensor_data,
        )

    def image_callback(self, image_msg: Image):
        frame = self.bridge.imgmsg_to_cv2(
            image_msg,
            desired_encoding='bgr8',
        )

        results = self.model(frame)

        detections_msg = Detection2DArray()

        # Preserve acquisition time through inference for cloud synchronization
        detections_msg.header = image_msg.header

        if len(results) > 0:
            result = results[0]

            for box in result.boxes:
                detection = self.yolo_box_to_detection2d(box)
                detections_msg.detections.append(detection)

        self.detections_pub.publish(detections_msg)

    def yolo_box_to_detection2d(self, box):
        detection = Detection2D()

        xyxy = box.xyxy.detach().cpu().numpy().reshape(-1)
        x1, y1, x2, y2 = xyxy[:4]

        cx = float((x1 + x2) / 2.0)
        cy = float((y1 + y2) / 2.0)
        width = float(x2 - x1)
        height = float(y2 - y1)

        detection.bbox = BoundingBox2D()
        detection.bbox.center.position.x = cx
        detection.bbox.center.position.y = cy
        detection.bbox.size_x = width
        detection.bbox.size_y = height

        hypothesis = ObjectHypothesisWithPose()

        class_id = int(box.cls.detach().cpu().numpy().reshape(-1)[0])
        confidence = float(box.conf.detach().cpu().numpy().reshape(-1)[0])

        hypothesis.hypothesis.class_id = str(class_id)
        hypothesis.hypothesis.score = confidence

        detection.results.append(hypothesis)

        return detection

    def return_classes(self):
        return self.classes


def main(args=None):
    rclpy.init(args=args)

    node = YoloNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
