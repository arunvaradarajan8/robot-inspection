import base64
import json
import time
from urllib import error as urlerror
from urllib import request as urlrequest

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String
from vision_msgs.msg import Detection2DArray

try:
    import cv2
    from cv_bridge import CvBridge
except ImportError:
    cv2 = None
    CvBridge = None


def get_json(url, timeout_sec):
    with urlrequest.urlopen(url, timeout=timeout_sec) as response:
        return json.loads(response.read().decode('utf-8'))


def post_json(url, payload, timeout_sec):
    data = json.dumps(payload).encode('utf-8')
    request = urlrequest.Request(
        url,
        data=data,
        headers={'content-type': 'application/json'},
        method='POST',
    )
    with urlrequest.urlopen(request, timeout=timeout_sec) as response:
        return response.status, response.read().decode('utf-8')


class TrimbleWindowsBridge(Node):

    def __init__(self):
        super().__init__('trimble_windows_bridge')

        self.declare_parameter('windows_url', 'http://127.0.0.1:8765')
        self.declare_parameter('scan_required_topic', '/digital_twin/scan_required')
        self.declare_parameter('scan_reason_topic', '/digital_twin/scan_reason')
        self.declare_parameter('waypoint_arrived_topic', '/digital_twin/waypoint_arrived')
        self.declare_parameter(
            'inspection_goal_topic',
            '/infrastructure/inspection_goal',
        )
        self.declare_parameter(
            'navigation_status_topic',
            '/infrastructure/navigation_status',
        )
        self.declare_parameter('image_topic', '/ros2_image')
        self.declare_parameter('detections_topic', '/detections_2d')
        self.declare_parameter('camera_preview_enabled', True)
        self.declare_parameter('camera_preview_rate_hz', 2.0)
        self.declare_parameter('camera_preview_width', 640)
        self.declare_parameter('request_reference_scan_on_start', True)
        self.declare_parameter('http_timeout_sec', 3.0)
        self.declare_parameter('request_cooldown_sec', 10.0)
        # The X7 writes its scans to its own SD card, so completion never
        # arrives over ROS. Poll the Windows app instead and report it, or
        # the planner would hold position until its scan timeout on every
        # single station.
        self.declare_parameter('scan_complete_topic', '/digital_twin/scan_complete')
        self.declare_parameter('status_poll_period_sec', 2.0)
        self.declare_parameter('scan_timeout_sec', 300.0)
        # Closes the mission once the operator has filed the E57 that the
        # Trimble held on its SD card for the whole run.
        self.declare_parameter('upload_complete_topic', '/mission/upload_complete')

        self.windows_url = self.get_parameter(
            'windows_url'
        ).get_parameter_value().string_value.rstrip('/')
        scan_required_topic = self.get_parameter(
            'scan_required_topic'
        ).get_parameter_value().string_value
        scan_reason_topic = self.get_parameter(
            'scan_reason_topic'
        ).get_parameter_value().string_value
        waypoint_arrived_topic = self.get_parameter(
            'waypoint_arrived_topic'
        ).get_parameter_value().string_value
        inspection_goal_topic = self.get_parameter(
            'inspection_goal_topic'
        ).get_parameter_value().string_value
        navigation_status_topic = self.get_parameter(
            'navigation_status_topic'
        ).get_parameter_value().string_value
        image_topic = self.get_parameter(
            'image_topic'
        ).get_parameter_value().string_value
        detections_topic = self.get_parameter(
            'detections_topic'
        ).get_parameter_value().string_value
        self.camera_preview_enabled = self.get_parameter(
            'camera_preview_enabled'
        ).get_parameter_value().bool_value
        self.camera_preview_rate_hz = self.get_parameter(
            'camera_preview_rate_hz'
        ).get_parameter_value().double_value
        self.camera_preview_width = self.get_parameter(
            'camera_preview_width'
        ).get_parameter_value().integer_value
        self.request_reference_scan_on_start = self.get_parameter(
            'request_reference_scan_on_start'
        ).get_parameter_value().bool_value
        self.http_timeout = self.get_parameter(
            'http_timeout_sec'
        ).get_parameter_value().double_value
        self.request_cooldown = self.get_parameter(
            'request_cooldown_sec'
        ).get_parameter_value().double_value
        scan_complete_topic = self.get_parameter(
            'scan_complete_topic'
        ).get_parameter_value().string_value
        status_poll_period = self.get_parameter(
            'status_poll_period_sec'
        ).get_parameter_value().double_value
        self.scan_timeout = self.get_parameter(
            'scan_timeout_sec'
        ).get_parameter_value().double_value
        upload_complete_topic = self.get_parameter(
            'upload_complete_topic'
        ).get_parameter_value().string_value

        self.scan_in_flight = False
        self.scan_started = 0.0
        self.scans_completed = None
        self.upload_reported = False

        self.last_reason = ''
        self.last_scan_request_time = None
        self.last_goal = None
        self.last_detections = []
        self.last_camera_preview_time = 0.0
        self.reference_requested = False
        self.suppress_next_scan_required = False
        self.cv_bridge = CvBridge() if CvBridge is not None else None

        self.scan_request_publisher = self.create_publisher(
            Bool,
            scan_required_topic,
            10,
        )
        self.scan_complete_publisher = self.create_publisher(
            Bool,
            scan_complete_topic,
            10,
        )
        self.upload_complete_publisher = self.create_publisher(
            Bool,
            upload_complete_topic,
            10,
        )

        self.scan_subscription = self.create_subscription(
            Bool,
            scan_required_topic,
            self.scan_required_callback,
            10,
        )
        self.reason_subscription = self.create_subscription(
            String,
            scan_reason_topic,
            self.scan_reason_callback,
            10,
        )
        self.arrival_subscription = self.create_subscription(
            String,
            waypoint_arrived_topic,
            self.waypoint_arrived_callback,
            10,
        )
        self.inspection_goal_subscription = self.create_subscription(
            PoseStamped,
            inspection_goal_topic,
            self.inspection_goal_callback,
            10,
        )
        self.navigation_status_subscription = self.create_subscription(
            String,
            navigation_status_topic,
            self.navigation_status_callback,
            10,
        )
        self.detections_subscription = self.create_subscription(
            Detection2DArray,
            detections_topic,
            self.detections_callback,
            qos_profile_sensor_data,
        )
        self.image_subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0, self.startup_tick)
        self.status_timer = self.create_timer(
            max(0.5, status_poll_period),
            self.poll_scan_status,
        )
        self.get_logger().info(f'Windows Trimble bridge targeting {self.windows_url}')
        if self.camera_preview_enabled and self.cv_bridge is None:
            self.get_logger().warning(
                'Camera preview disabled: cv_bridge/OpenCV import failed'
            )
        self.post(
            '/jetson_ready',
            {
                'status': 'ros_started',
                'node': 'trimble_windows_bridge',
            },
        )
        self.post_status('Jetson Ready', 'ROS stack started')

    def startup_tick(self):
        if self.reference_requested or not self.request_reference_scan_on_start:
            return
        self.reference_requested = True
        self.open_scan_watcher_gate()
        self.post_status('Scanning', 'Requesting initial reference scan')
        self.send_scan_request(
            {
                'scan_type': 'reference',
                'reason': 'initial reference scan at station',
            },
            bypass_cooldown=True,
        )

    def scan_reason_callback(self, message):
        self.last_reason = message.data

    def scan_required_callback(self, message):
        if not message.data:
            return
        if self.suppress_next_scan_required:
            self.suppress_next_scan_required = False
            return
        self.post_status('Scanning', self.last_reason or 'Requesting defect rescan')
        self.send_scan_request(
            {
                'scan_type': 'defect_rescan',
                'reason': self.last_reason or 'high-confidence detection',
            },
        )

    def inspection_goal_callback(self, goal):
        self.last_goal = {
            'frame_id': goal.header.frame_id,
            'x': goal.pose.position.x,
            'y': goal.pose.position.y,
            'z': goal.pose.position.z,
        }
        self.post_status(
            'Navigating',
            (
                'Infrastructure inspection goal: '
                f'x={goal.pose.position.x:.2f}, '
                f'y={goal.pose.position.y:.2f}'
            ),
        )

    def navigation_status_callback(self, message):
        self.post_status('Navigating', message.data)

    def waypoint_arrived_callback(self, message):
        payload = {
            'reason': message.data,
            'goal': self.last_goal,
        }
        self.post_status('Waypoint Arrived', message.data)
        self.open_scan_watcher_gate()
        self.post('/waypoint_arrived', payload)

    def detections_callback(self, message):
        detections = []
        for detection in message.detections:
            confidence = 0.0
            class_id = 'unknown'
            if detection.results:
                hypothesis = detection.results[0].hypothesis
                confidence = float(hypothesis.score)
                class_id = hypothesis.class_id or 'unknown'
            detections.append(
                {
                    'class_id': class_id,
                    'confidence': confidence,
                    'x': float(detection.bbox.center.position.x),
                    'y': float(detection.bbox.center.position.y),
                    'width': float(detection.bbox.size_x),
                    'height': float(detection.bbox.size_y),
                }
            )
        self.last_detections = detections

    def image_callback(self, message):
        if not self.camera_preview_enabled or self.cv_bridge is None or cv2 is None:
            return
        now = time.monotonic()
        min_period = 1.0 / max(0.1, self.camera_preview_rate_hz)
        if now - self.last_camera_preview_time < min_period:
            return
        self.last_camera_preview_time = now

        try:
            frame = self.cv_bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
            preview, scale = self.make_preview(frame)
            for detection in self.last_detections:
                self.draw_detection(preview, detection, scale)
            ok, encoded = cv2.imencode('.png', preview)
            if not ok:
                return
            payload = {
                'stamp': f'{message.header.stamp.sec}.{message.header.stamp.nanosec:09d}',
                'frame_id': message.header.frame_id,
                'detections': self.last_detections,
                'image_png_base64': base64.b64encode(encoded.tobytes()).decode('ascii'),
            }
            self.post('/camera_frame', payload)
        except Exception as error:
            self.get_logger().warning(f'Camera preview failed: {error}')

    def make_preview(self, frame):
        height, width = frame.shape[:2]
        target_width = max(160, int(self.camera_preview_width))
        if width <= target_width:
            return frame.copy(), 1.0
        scale = target_width / float(width)
        target_height = max(1, int(height * scale))
        resized = cv2.resize(frame, (target_width, target_height))
        return resized, scale

    def draw_detection(self, frame, detection, scale):
        cx = float(detection['x']) * scale
        cy = float(detection['y']) * scale
        width = float(detection['width']) * scale
        height = float(detection['height']) * scale
        x1 = int(max(0, cx - width / 2.0))
        y1 = int(max(0, cy - height / 2.0))
        x2 = int(min(frame.shape[1] - 1, cx + width / 2.0))
        y2 = int(min(frame.shape[0] - 1, cy + height / 2.0))
        label = (
            f'{detection["class_id"]} '
            f'{float(detection["confidence"]):.2f}'
        )
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 20, 220), 2)
        cv2.putText(
            frame,
            label,
            (x1, max(16, y1 - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 20, 220),
            2,
            cv2.LINE_AA,
        )

    def open_scan_watcher_gate(self):
        self.suppress_next_scan_required = True
        message = Bool()
        message.data = True
        self.scan_request_publisher.publish(message)

    def send_scan_request(self, payload, bypass_cooldown=False):
        now = time.monotonic()
        if (
            not bypass_cooldown
            and self.last_scan_request_time is not None
            and now - self.last_scan_request_time < self.request_cooldown
        ):
            self.get_logger().debug('Skipping duplicate Windows scan request')
            return
        self.last_scan_request_time = now
        self.scan_in_flight = True
        self.scan_started = now
        self.post('/scan_request', payload)

    def poll_scan_status(self):
        """Track scan completion and the end-of-mission E57 upload."""
        if not self.scan_in_flight and self.upload_reported:
            return

        now = time.monotonic()
        try:
            status = get_json(self.windows_url + '/status', self.http_timeout)
        except (OSError, urlerror.URLError, TimeoutError, ValueError) as error:
            self.get_logger().warning(
                f'Cannot read Windows bridge status: {error}',
                throttle_duration_sec=15.0,
            )
            status = None

        if status is not None:
            if not self.upload_reported and status.get('uploaded_e57'):
                self.upload_reported = True
                self.upload_complete_publisher.publish(Bool(data=True))
                self.get_logger().info(
                    f'Operator uploaded the mission E57: '
                    f'{status["uploaded_e57"]}'
                )

            completed = status.get('scans_completed')
            if self.scans_completed is None:
                self.scans_completed = completed
            elif (
                self.scan_in_flight
                and completed is not None
                and completed != self.scans_completed
            ):
                self.scans_completed = completed
                self.publish_scan_complete('Trimble reported the scan finished')
                return

        if self.scan_in_flight and now - self.scan_started > self.scan_timeout:
            self.publish_scan_complete(
                f'no scan completion within {self.scan_timeout:.0f}s; '
                'releasing the robot anyway'
            )

    def publish_scan_complete(self, reason):
        self.scan_in_flight = False
        message = Bool()
        message.data = True
        self.scan_complete_publisher.publish(message)
        self.get_logger().info(f'Scan complete: {reason}')

    def post(self, path, payload):
        url = self.windows_url + path
        try:
            status, body = post_json(url, payload, self.http_timeout)
            self.get_logger().info(f'POST {path} -> {status}: {body}')
        except (OSError, urlerror.URLError, TimeoutError) as error:
            self.get_logger().warning(f'POST {path} failed: {error}')

    def post_status(self, state, detail=''):
        self.post('/process_status', {'state': state, 'detail': detail})


def main(args=None):
    rclpy.init(args=args)
    node = TrimbleWindowsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
