"""Demo mode: a virtual site standing in for the site sensors.

The robot, its localization, its motion, and the Trimble X7 are all real;
only what the robot perceives is invented. The node anchors a virtual
site at the robot's pose when the demo starts, then plays that site back
as if a sensor were sweeping it - range-limited and occluded from the
robot's live TF pose - so the occupancy map fills in as the robot walks
and the frontier planner has somewhere to go. Planted defects are
published as 3D detections once the robot is close enough and facing
them, which is what sends it to a standoff pose and triggers a scan.

Nothing else in the stack changes: the same occupancy map, planner,
mission manager, goal bridge, and Trimble bridge run as in the field.
The real world around the robot is empty, so the site it walks through
exists only in its head.
"""

import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import Buffer, TransformException, TransformListener
from vision_msgs.msg import (
    BoundingBox3D,
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)

from defect_detection.simulation.virtual_site import SiteError, load_site, sweep


def yaw_from_quaternion(rotation):
    return math.atan2(
        2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
        1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
    )


class VirtualSiteNode(Node):

    def __init__(self):
        super().__init__('virtual_site_node')

        self.declare_parameter('site_path', '')
        self.declare_parameter('site_density', 1.0)
        self.declare_parameter('site_seed', 7)
        self.declare_parameter('cloud_topic', '/demo/points')
        self.declare_parameter('detections_3d_topic', '/detections_3d')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('base_frame', 'body')
        # Where the site sits in the world. Anchoring to the robot's pose
        # at startup means the operator walks Spot to the middle of the
        # open area, launches, and the site unfolds ahead of it.
        self.declare_parameter('anchor_to_start_pose', True)
        self.declare_parameter('anchor_x', 0.0)
        self.declare_parameter('anchor_y', 0.0)
        self.declare_parameter('anchor_yaw_deg', 0.0)
        # Simulated depth sensor.
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('sensor_range_m', 20.0)
        self.declare_parameter('bearing_bin_deg', 0.4)
        self.declare_parameter('occlusion_depth_m', 0.4)
        self.declare_parameter('max_points_per_sweep', 40000)
        self.declare_parameter('range_noise_m', 0.02)
        # Detector stand-in.
        self.declare_parameter('detection_rate_hz', 2.0)
        self.declare_parameter('detection_range_m', 8.0)
        self.declare_parameter('detection_fov_deg', 90.0)
        self.declare_parameter('detection_bbox_size_m', 0.20)
        self.declare_parameter('detection_position_noise_m', 0.05)

        site_path = self.get_parameter(
            'site_path'
        ).get_parameter_value().string_value
        if not site_path:
            raise SiteError('site_path is required in demo mode')
        self.site = load_site(
            site_path,
            density=self.get_parameter('site_density').value,
            seed=int(self.get_parameter('site_seed').value),
        )

        self.target_frame = self.get_parameter(
            'target_frame'
        ).get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame'
        ).get_parameter_value().string_value
        self.anchor_to_start_pose = self.get_parameter(
            'anchor_to_start_pose'
        ).get_parameter_value().bool_value
        self.anchor_x = float(self.get_parameter('anchor_x').value)
        self.anchor_y = float(self.get_parameter('anchor_y').value)
        self.anchor_yaw = math.radians(
            float(self.get_parameter('anchor_yaw_deg').value)
        )
        self.sensor_range = float(self.get_parameter('sensor_range_m').value)
        self.bearing_bin = math.radians(
            max(0.05, float(self.get_parameter('bearing_bin_deg').value))
        )
        self.occlusion_depth = float(
            self.get_parameter('occlusion_depth_m').value
        )
        self.max_points = int(self.get_parameter('max_points_per_sweep').value)
        self.range_noise = float(self.get_parameter('range_noise_m').value)
        self.detection_range = float(
            self.get_parameter('detection_range_m').value
        )
        self.detection_half_fov = math.radians(
            float(self.get_parameter('detection_fov_deg').value)
        ) / 2.0
        self.detection_bbox_size = float(
            self.get_parameter('detection_bbox_size_m').value
        )
        self.detection_noise = float(
            self.get_parameter('detection_position_noise_m').value
        )

        self.rng = np.random.default_rng(
            int(self.get_parameter('site_seed').value) + 1
        )
        # The site placed in target_frame; None until the anchor pose is
        # known, which needs one TF lookup after localization comes up.
        self.placed = None
        self.announced_defects = set()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cloud_publisher = self.create_publisher(
            PointCloud2,
            self.get_parameter('cloud_topic').get_parameter_value().string_value,
            qos_profile_sensor_data,
        )
        self.detection_publisher = self.create_publisher(
            Detection3DArray,
            self.get_parameter(
                'detections_3d_topic'
            ).get_parameter_value().string_value,
            10,
        )
        self.create_timer(
            1.0 / max(0.1, float(self.get_parameter('publish_rate_hz').value)),
            self.publish_sweep,
        )
        self.create_timer(
            1.0 / max(0.1, float(self.get_parameter('detection_rate_hz').value)),
            self.publish_detections,
        )

        self.get_logger().warning(
            f'DEMO MODE: navigating the virtual site "{self.site.name}" '
            f'({len(self.site.points)} points, {len(self.site.defects)} '
            'planted defects). Nothing it maps is real - the area around '
            'the robot must be open and clear, and a spotter must keep '
            'the e-stop.'
        )

    # ---- anchoring ------------------------------------------------------

    def robot_pose(self):
        """The robot's (x, y, yaw) in the target frame, or None."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.25),
            )
        except TransformException as error:
            self.get_logger().warning(
                f'Waiting for {self.target_frame} -> {self.base_frame}: {error}',
                throttle_duration_sec=10.0,
            )
            return None
        translation = transform.transform.translation
        return (
            translation.x,
            translation.y,
            yaw_from_quaternion(transform.transform.rotation),
        )

    def ensure_placed(self, pose):
        if self.placed is not None:
            return True
        if self.anchor_to_start_pose:
            x, y, yaw = pose
            yaw += self.anchor_yaw
            x += self.anchor_x * math.cos(yaw) - self.anchor_y * math.sin(yaw)
            y += self.anchor_x * math.sin(yaw) + self.anchor_y * math.cos(yaw)
        else:
            x, y, yaw = self.anchor_x, self.anchor_y, self.anchor_yaw
        self.placed = self.site.transformed(x, y, yaw)
        self.get_logger().info(
            f'Virtual site anchored at x={x:.2f} y={y:.2f} '
            f'yaw={math.degrees(yaw):.1f} deg in {self.target_frame}'
        )
        return True

    # ---- simulated sensor ------------------------------------------------

    def publish_sweep(self):
        pose = self.robot_pose()
        if pose is None or not self.ensure_placed(pose):
            return

        points = sweep(
            self.placed.points,
            pose[0],
            pose[1],
            sensor_range=self.sensor_range,
            bearing_bin=self.bearing_bin,
            occlusion_depth=self.occlusion_depth,
            max_points=self.max_points,
            range_noise=self.range_noise,
            rng=self.rng,
        )
        if not len(points):
            self.get_logger().info(
                'No virtual site geometry in range; the robot is outside '
                'the site it thinks it is inspecting',
                throttle_duration_sec=15.0,
            )
            return

        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.target_frame
        self.cloud_publisher.publish(
            point_cloud2.create_cloud_xyz32(header, points.astype(np.float32))
        )

    # ---- detector stand-in ----------------------------------------------

    def publish_detections(self):
        if self.placed is None:
            return
        pose = self.robot_pose()
        if pose is None:
            return
        origin_x, origin_y, heading = pose

        message = Detection3DArray()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.target_frame

        for index, defect in enumerate(self.placed.defects):
            delta_x = defect.position[0] - origin_x
            delta_y = defect.position[1] - origin_y
            distance = math.hypot(delta_x, delta_y)
            if distance > self.detection_range or distance < 1e-6:
                continue
            bearing = math.atan2(delta_y, delta_x) - heading
            bearing = math.atan2(math.sin(bearing), math.cos(bearing))
            if abs(bearing) > self.detection_half_fov:
                continue

            detection = Detection3D()
            detection.header = message.header
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = defect.class_id
            # Confidence climbs as the robot closes in, the way a real
            # detector's does, so the planner keeps preferring the
            # defect it is already walking toward.
            proximity = 1.0 - 0.3 * min(1.0, distance / self.detection_range)
            hypothesis.hypothesis.score = float(
                np.clip(defect.confidence * proximity, 0.05, 0.99)
            )
            detection.results.append(hypothesis)

            jitter = self.rng.normal(0.0, self.detection_noise, 3)
            detection.bbox = BoundingBox3D()
            detection.bbox.center.position.x = float(defect.position[0] + jitter[0])
            detection.bbox.center.position.y = float(defect.position[1] + jitter[1])
            detection.bbox.center.position.z = float(defect.position[2] + jitter[2])
            detection.bbox.center.orientation.w = 1.0
            size = max(0.02, defect.size or self.detection_bbox_size)
            detection.bbox.size.x = size
            detection.bbox.size.y = size
            detection.bbox.size.z = size
            message.detections.append(detection)

            if index not in self.announced_defects:
                self.announced_defects.add(index)
                self.get_logger().info(
                    f'Virtual defect in view: {defect.class_id} at '
                    f'{distance:.1f} m'
                )

        if message.detections:
            self.detection_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualSiteNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
