import math
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Bool, Header
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

from defect_detection.simulation import world_model
from defect_detection.simulation.world_constants import (
    DEFECTS,
    WALL_HEIGHT,
    WALL_LATERAL_MAX,
    WALL_LATERAL_MIN,
    WALL_PLANE_X,
)


def quaternion_from_yaw(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quaternion_from_matrix(rotation):
    trace = rotation[0, 0] + rotation[1, 1] + rotation[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rotation[2, 1] - rotation[1, 2]) / s
        y = (rotation[0, 2] - rotation[2, 0]) / s
        z = (rotation[1, 0] - rotation[0, 1]) / s
    elif (rotation[0, 0] > rotation[1, 1]) and (rotation[0, 0] > rotation[2, 2]):
        s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / s
        x = 0.25 * s
        y = (rotation[0, 1] + rotation[1, 0]) / s
        z = (rotation[0, 2] + rotation[2, 0]) / s
    elif rotation[1, 1] > rotation[2, 2]:
        s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / s
        x = (rotation[0, 1] + rotation[1, 0]) / s
        y = 0.25 * s
        z = (rotation[1, 2] + rotation[2, 1]) / s
    else:
        s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / s
        x = (rotation[0, 2] + rotation[2, 0]) / s
        y = (rotation[1, 2] + rotation[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def look_at_rotation(camera_position, target):
    forward = np.asarray(target, dtype=np.float64) - camera_position
    norm = np.linalg.norm(forward)
    if norm < 1e-6:
        forward = np.array([0.0, 1.0, 0.0])
    else:
        forward = forward / norm
    up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, up)
    right_norm = np.linalg.norm(right)
    if right_norm < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    else:
        right = right / right_norm
    down = np.cross(forward, right)
    # Optical convention: x right, y down, z forward (columns of R).
    return np.column_stack((right, down, forward))


class SyntheticFieldNode(Node):
    """Simulate the camera, detector, robot, and Trimble X7 for demos.

    Publishes synthetic OAK-D RGB/depth/camera_info, YOLO-style 2D
    detections, a live environment point cloud, and odom->base_link TF.
    Writes a synthetic LAS scan when the pipeline requests one, and drives
    the simulated robot toward planner goals so the full autonomy loop
    closes without any hardware.
    """

    def __init__(self):
        super().__init__('synthetic_field_node')

        self.declare_parameter('rate_hz', 8.0)
        self.declare_parameter('image_topic', '/ros2_image')
        self.declare_parameter('depth_topic', '/oak/rgb/depth')
        self.declare_parameter('camera_info_topic', '/oak/rgb/camera_info')
        self.declare_parameter('detections_2d_topic', '/detections_2d')
        self.declare_parameter('pointcloud_topic', '/cloud/points')
        self.declare_parameter('scan_required_topic', '/digital_twin/scan_required')
        self.declare_parameter('scan_topic', '/trimble/x7/scan_points')
        self.declare_parameter('scan_complete_topic', '/digital_twin/scan_complete')
        self.declare_parameter('scan_directory', '/tmp/synthetic_demo/trimble_scans')
        self.declare_parameter('scan_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_optical_frame')
        self.declare_parameter('inspection_goal_topic', '/infrastructure/inspection_goal')
        self.declare_parameter('robot_speed_mps', 0.7)
        # When true the robot stays parked until it sees a reference scan
        # (the legacy anchor-first flow). The frontier demo explores before
        # it scans, so it sets this false to let the robot move at once.
        self.declare_parameter('require_reference_scan', True)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 400)
        self.declare_parameter('focal_length_px', 420.0)
        self.declare_parameter('scan_write_delay_sec', 2.0)

        self.rate_hz = self.get_parameter('rate_hz').value
        self.image_topic = self.get_parameter('image_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.detections_topic = self.get_parameter('detections_2d_topic').value
        self.cloud_topic = self.get_parameter('pointcloud_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.scan_directory = Path(
            self.get_parameter('scan_directory').value
        ).expanduser()
        self.scan_frame = self.get_parameter('scan_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.robot_speed = float(self.get_parameter('robot_speed_mps').value)
        self.width = int(self.get_parameter('image_width').value)
        self.height = int(self.get_parameter('image_height').value)
        self.focal = float(self.get_parameter('focal_length_px').value)
        self.scan_write_delay = float(
            self.get_parameter('scan_write_delay_sec').value
        )

        self.cx = self.width / 2.0
        self.cy = self.height / 2.0
        self.bridge = CvBridge()
        self.rng = np.random.default_rng(7)

        # Pixel ray grid in the camera optical frame (unnormalized, z = 1).
        us = (np.arange(self.width, dtype=np.float64) - self.cx) / self.focal
        vs = (np.arange(self.height, dtype=np.float64) - self.cy) / self.focal
        ray_x, ray_y = np.meshgrid(us, vs)
        self.rays_camera = np.stack(
            (ray_x, ray_y, np.ones_like(ray_x)),
            axis=-1,
        )

        # Robot state in odom. Parked at the origin with zero yaw until the
        # first reference scan, so the frame anchor (which inverts the robot
        # pose at capture) makes map coincide with odom and the synthetic
        # scan coordinates are valid in both frames.
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.require_reference_scan = bool(
            self.get_parameter('require_reference_scan').value
        )
        self.first_scan_seen = not self.require_reference_scan
        self.active_goal = None
        self.scan_write_due = None
        self.scan_count = 0

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.image_publisher = self.create_publisher(
            Image, self.image_topic, qos_profile_sensor_data,
        )
        self.depth_publisher = self.create_publisher(
            Image, self.depth_topic, qos_profile_sensor_data,
        )
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, self.camera_info_topic, qos_profile_sensor_data,
        )
        self.detections_publisher = self.create_publisher(
            Detection2DArray, self.detections_topic, qos_profile_sensor_data,
        )
        self.cloud_publisher = self.create_publisher(
            PointCloud2, self.cloud_topic, qos_profile_sensor_data,
        )
        self.scan_publisher = self.create_publisher(
            PointCloud2, self.scan_topic, qos_profile_sensor_data,
        )
        # Stands in for the Windows bridge reporting that the X7 finished.
        # Without it the planner would hold position at every station.
        self.scan_complete_publisher = self.create_publisher(
            Bool, self.get_parameter('scan_complete_topic').value, 10,
        )

        self.create_subscription(
            Bool,
            self.get_parameter('scan_required_topic').value,
            self.scan_required_callback,
            10,
        )
        self.create_subscription(
            PointCloud2,
            self.scan_topic,
            self.scan_seen_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter('inspection_goal_topic').value,
            self.inspection_goal_callback,
            10,
        )

        self.sensor_timer = self.create_timer(1.0 / self.rate_hz, self.sensor_tick)
        self.motion_timer = self.create_timer(0.05, self.motion_tick)
        self.cloud_timer = self.create_timer(0.5, self.publish_environment_cloud)

        self.get_logger().info(
            'Synthetic field demo running: wall with '
            f'{len(DEFECTS)} defects at x={WALL_PLANE_X}m, robot parked until the '
            'first reference scan anchors the digital twin.'
        )

    # ------------------------------------------------------------------
    # Robot motion and TF
    # ------------------------------------------------------------------

    def inspection_goal_callback(self, goal):
        self.set_goal(goal, 'inspection')

    def set_goal(self, goal, source):
        position = self.goal_in_odom(goal)
        if position is None:
            return
        x, y, yaw = position
        self.active_goal = ((x, y, yaw), time.monotonic(), source)
        self.get_logger().info(
            f'Simulated robot accepted {source} goal x={x:.2f} y={y:.2f}'
        )

    def goal_in_odom(self, goal):
        yaw = 2.0 * math.atan2(goal.pose.orientation.z, goal.pose.orientation.w)
        x = goal.pose.position.x
        y = goal.pose.position.y
        frame = goal.header.frame_id or self.odom_frame
        if frame == self.odom_frame:
            return x, y, yaw
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5),
            )
        except TransformException as error:
            self.get_logger().warning(f'Cannot transform goal to odom: {error}')
            return None
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        frame_yaw = 2.0 * math.atan2(rotation.z, rotation.w)
        cos_yaw = math.cos(frame_yaw)
        sin_yaw = math.sin(frame_yaw)
        odom_x = cos_yaw * x - sin_yaw * y + translation.x
        odom_y = sin_yaw * x + cos_yaw * y + translation.y
        return odom_x, odom_y, yaw + frame_yaw

    def motion_tick(self):
        step = 0.05
        if self.first_scan_seen and self.active_goal is not None:
            (goal_x, goal_y, goal_yaw), _, _ = self.active_goal
            dx = goal_x - self.robot_x
            dy = goal_y - self.robot_y
            distance = math.hypot(dx, dy)
            if distance > 0.10:
                travel = min(self.robot_speed * step, distance)
                self.robot_x += travel * dx / distance
                self.robot_y += travel * dy / distance
                self.robot_yaw = self.slew_yaw(math.atan2(dy, dx), step)
            else:
                self.robot_yaw = self.slew_yaw(goal_yaw, step)

        self.broadcast_transforms()

    def slew_yaw(self, target_yaw, step, rate=1.2):
        error = math.atan2(
            math.sin(target_yaw - self.robot_yaw),
            math.cos(target_yaw - self.robot_yaw),
        )
        change = max(-rate * step, min(rate * step, error))
        return self.robot_yaw + change

    def camera_pose(self):
        position = np.array([self.robot_x, self.robot_y, 0.65])
        look_y = min(
            max(self.robot_y, WALL_LATERAL_MIN + 1.0),
            WALL_LATERAL_MAX - 1.0,
        )
        rotation = look_at_rotation(position, (WALL_PLANE_X, look_y, 1.5))
        return position, rotation

    def broadcast_transforms(self):
        stamp = self.get_clock().now().to_msg()

        base = TransformStamped()
        base.header.stamp = stamp
        base.header.frame_id = self.odom_frame
        base.child_frame_id = self.base_frame
        base.transform.translation.x = self.robot_x
        base.transform.translation.y = self.robot_y
        (
            base.transform.rotation.x,
            base.transform.rotation.y,
            base.transform.rotation.z,
            base.transform.rotation.w,
        ) = quaternion_from_yaw(self.robot_yaw)

        position, rotation = self.camera_pose()
        camera = TransformStamped()
        camera.header.stamp = stamp
        camera.header.frame_id = self.odom_frame
        camera.child_frame_id = self.camera_frame
        camera.transform.translation.x = float(position[0])
        camera.transform.translation.y = float(position[1])
        camera.transform.translation.z = float(position[2])
        (
            camera.transform.rotation.x,
            camera.transform.rotation.y,
            camera.transform.rotation.z,
            camera.transform.rotation.w,
        ) = quaternion_from_matrix(rotation)

        self.tf_broadcaster.sendTransform([base, camera])

    # ------------------------------------------------------------------
    # Camera rendering
    # ------------------------------------------------------------------

    def sensor_tick(self):
        stamp = self.get_clock().now().to_msg()
        position, rotation = self.camera_pose()

        depth, wall_mask, ground_mask = self.render_depth(position, rotation)
        image = self.render_rgb(depth, wall_mask, ground_mask)
        detections = self.project_defects(position, rotation)
        self.draw_defects(image, detections)

        image_msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.camera_frame
        self.image_publisher.publish(image_msg)

        depth_msg = self.bridge.cv2_to_imgmsg(
            depth.astype(np.float32), encoding='32FC1',
        )
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.camera_frame
        self.depth_publisher.publish(depth_msg)

        self.camera_info_publisher.publish(self.camera_info(stamp))
        self.detections_publisher.publish(
            self.detections_message(detections, stamp)
        )

        if (
            self.scan_write_due is not None
            and time.monotonic() >= self.scan_write_due
        ):
            self.scan_write_due = None
            self.emit_reference_scan()

    def render_depth(self, position, rotation):
        rays_world = self.rays_camera @ rotation.T
        depth = np.full((self.height, self.width), 20.0, dtype=np.float64)

        far = 1.0e6
        with np.errstate(divide='ignore', invalid='ignore'):
            # Wall plane x = WALL_PLANE_X.
            denominator = rays_world[..., 0]
            t_wall = np.where(
                denominator > 1e-6,
                (WALL_PLANE_X - position[0]) / denominator,
                far,
            )
            hit_y = position[1] + t_wall * rays_world[..., 1]
            hit_z = position[2] + t_wall * rays_world[..., 2]
            wall_mask = (
                (t_wall > 0.3)
                & (t_wall < far)
                & (hit_y >= WALL_LATERAL_MIN)
                & (hit_y <= WALL_LATERAL_MAX)
                & (hit_z >= 0.0)
                & (hit_z <= WALL_HEIGHT)
            )

            # Ground plane z = 0.
            denominator_z = rays_world[..., 2]
            t_ground = np.where(
                denominator_z < -1e-6,
                -position[2] / denominator_z,
                far,
            )
        ground_mask = (t_ground > 0.3) & (t_ground < 20.0) & ~wall_mask

        depth[wall_mask] = t_wall[wall_mask]
        depth[ground_mask] = t_ground[ground_mask]
        depth += self.rng.normal(0.0, 0.01, depth.shape)
        return np.clip(depth, 0.3, 25.0), wall_mask, ground_mask

    def render_rgb(self, depth, wall_mask, ground_mask):
        image = np.full((self.height, self.width, 3), 178, dtype=np.uint8)
        noise = self.rng.integers(-9, 9, (self.height, self.width, 1))
        wall = np.array([148, 152, 155], dtype=np.int16)
        ground = np.array([96, 104, 110], dtype=np.int16)
        image[wall_mask] = np.clip(wall + noise[wall_mask], 0, 255)
        image[ground_mask] = np.clip(ground + noise[ground_mask], 0, 255)

        # Horizontal formwork joints for texture.
        wall_rows = np.where(wall_mask.any(axis=1))[0]
        for row in wall_rows[::47]:
            image[row, wall_mask[row]] = (128, 132, 136)
        return image

    def project_defects(self, position, rotation):
        visible = []
        for class_id, lateral, height, size, confidence in DEFECTS:
            world = np.array([WALL_PLANE_X, lateral, height])
            in_camera = rotation.T @ (world - position)
            if in_camera[2] < 0.6:
                continue
            u = self.focal * in_camera[0] / in_camera[2] + self.cx
            v = self.focal * in_camera[1] / in_camera[2] + self.cy
            pixel_size = self.focal * size / in_camera[2]
            if pixel_size < 12:
                continue
            margin = pixel_size / 2.0
            if not (
                margin < u < self.width - margin
                and margin < v < self.height - margin
            ):
                continue
            jitter = self.rng.normal(0.0, 1.5, 2)
            score = float(
                np.clip(confidence + self.rng.normal(0.0, 0.02), 0.05, 0.99)
            )
            visible.append({
                'class_id': class_id,
                'u': float(u + jitter[0]),
                'v': float(v + jitter[1]),
                'size_px': float(pixel_size),
                'score': score,
            })
        return visible

    def draw_defects(self, image, detections):
        for detection in detections:
            u = int(detection['u'])
            v = int(detection['v'])
            half = int(detection['size_px'] / 2)
            if detection['class_id'] == 'crack':
                points = [(u - half, v - half)]
                segments = max(4, half // 6)
                for index in range(1, segments + 1):
                    points.append((
                        u - half + (2 * half * index) // segments,
                        v - half + (2 * half * index) // segments
                        + int(self.rng.integers(-half // 3, half // 3 + 1)),
                    ))
                cv2.polylines(
                    image,
                    [np.array(points, dtype=np.int32)],
                    False,
                    (40, 40, 45),
                    2,
                )
            elif detection['class_id'] == 'spalling':
                cv2.ellipse(
                    image, (u, v), (half, int(half * 0.7)),
                    15, 0, 360, (70, 74, 82), -1,
                )
                cv2.ellipse(
                    image, (u - half // 4, v), (half // 2, half // 3),
                    0, 0, 360, (52, 56, 64), -1,
                )
            else:  # exposed_rebar
                for offset in (-half // 3, 0, half // 3):
                    cv2.line(
                        image,
                        (u + offset, v - half),
                        (u + offset, v + half),
                        (35, 60, 110),
                        3,
                    )

    def camera_info(self, stamp):
        info = CameraInfo()
        info.header.stamp = stamp
        info.header.frame_id = self.camera_frame
        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.k = [
            self.focal, 0.0, self.cx,
            0.0, self.focal, self.cy,
            0.0, 0.0, 1.0,
        ]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [
            self.focal, 0.0, self.cx, 0.0,
            0.0, self.focal, self.cy, 0.0,
            0.0, 0.0, 1.0, 0.0,
        ]
        return info

    def detections_message(self, detections, stamp):
        message = Detection2DArray()
        message.header.stamp = stamp
        message.header.frame_id = self.camera_frame
        for entry in detections:
            detection = Detection2D()
            detection.header = message.header
            detection.bbox.center.position.x = entry['u']
            detection.bbox.center.position.y = entry['v']
            detection.bbox.size_x = entry['size_px']
            detection.bbox.size_y = entry['size_px']
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = entry['class_id']
            hypothesis.hypothesis.score = entry['score']
            detection.results.append(hypothesis)
            message.detections.append(detection)
        return message

    # ------------------------------------------------------------------
    # Environment cloud and Trimble reference scans
    # ------------------------------------------------------------------

    def publish_environment_cloud(self):
        points = world_model.structure_points(
            self.rng, density=0.25,
        ).astype(np.float32)
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.odom_frame
        self.cloud_publisher.publish(
            point_cloud2.create_cloud_xyz32(header, points)
        )

    def scan_required_callback(self, message):
        if not message.data or self.scan_write_due is not None:
            return
        self.scan_write_due = time.monotonic() + self.scan_write_delay
        self.get_logger().info(
            'Scan requested; simulated Trimble X7 is scanning...'
        )

    def scan_seen_callback(self, _message):
        if not self.first_scan_seen:
            self.first_scan_seen = True
            self.get_logger().info(
                'Reference scan published; simulated robot may now move.'
            )

    def emit_reference_scan(self):
        points = world_model.structure_points(self.rng, density=8.0)
        self.scan_count += 1
        try:
            self.scan_directory.mkdir(parents=True, exist_ok=True)
            target = self.scan_directory / f'synthetic_scan_{self.scan_count:03d}.las'
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
            self.scan_complete_publisher.publish(Bool(data=True))
            return
        self.get_logger().info(
            f'Simulated Trimble scan written: {target} ({len(points)} points). '
            'Nothing is ingested from it; the file only stands in for the '
            'scan the real X7 keeps on its SD card.'
        )
        self.scan_complete_publisher.publish(Bool(data=True))


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticFieldNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
