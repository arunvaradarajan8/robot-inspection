import math

from geometry_msgs.msg import PoseStamped, Twist
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(rotation):
    siny_cosp = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y)
    cosy_cosp = 1.0 - 2.0 * (
        rotation.y * rotation.y + rotation.z * rotation.z
    )
    return math.atan2(siny_cosp, cosy_cosp)


def angle_difference(target, current):
    return math.atan2(math.sin(target - current), math.cos(target - current))


class GoalDriverNode(Node):
    """Drive the simulated robot toward planner goals with cmd_vel.

    Stands in for Spot in simulation: subscribes to inspection goals,
    looks up the robot pose over TF, and publishes a proportional
    velocity command until the goal pose is reached.
    """

    def __init__(self):
        super().__init__('goal_driver_node')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('inspection_goal_topic', '/infrastructure/inspection_goal')
        self.declare_parameter('scan_topic', '/trimble/x7/scan_points')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('linear_speed_mps', 0.7)
        self.declare_parameter('angular_speed_rps', 1.0)
        self.declare_parameter('position_tolerance_m', 0.15)
        self.declare_parameter('yaw_tolerance_rad', 0.15)
        self.declare_parameter('wait_for_first_scan', True)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.linear_speed = float(self.get_parameter('linear_speed_mps').value)
        self.angular_speed = float(self.get_parameter('angular_speed_rps').value)
        self.position_tolerance = float(
            self.get_parameter('position_tolerance_m').value
        )
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance_rad').value)
        self.first_scan_seen = not self.get_parameter('wait_for_first_scan').value

        self.active_goal = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.cmd_publisher = self.create_publisher(
            Twist, self.get_parameter('cmd_vel_topic').value, 10,
        )
        self.create_subscription(
            PoseStamped,
            self.get_parameter('inspection_goal_topic').value,
            self.inspection_goal_callback,
            10,
        )
        self.create_subscription(
            PointCloud2,
            self.get_parameter('scan_topic').value,
            self.scan_seen_callback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(0.05, self.control_tick)
        self.get_logger().info(
            'Simulated goal driver ready; robot parked until the first scan.'
        )

    def scan_seen_callback(self, _message):
        if not self.first_scan_seen:
            self.first_scan_seen = True
            self.get_logger().info(
                'Reference scan published; simulated robot may now move.'
            )

    def inspection_goal_callback(self, goal):
        self.set_goal(goal, 'inspection')

    def set_goal(self, goal, source):
        in_odom = self.goal_in_odom(goal)
        if in_odom is None:
            return
        self.active_goal = (in_odom, source)
        x, y, _ = in_odom
        self.get_logger().info(
            f'Simulated robot accepted {source} goal x={x:.2f} y={y:.2f}'
        )

    def goal_in_odom(self, goal):
        x = goal.pose.position.x
        y = goal.pose.position.y
        yaw = yaw_from_quaternion(goal.pose.orientation)
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
        frame_yaw = yaw_from_quaternion(transform.transform.rotation)
        cos_yaw = math.cos(frame_yaw)
        sin_yaw = math.sin(frame_yaw)
        odom_x = cos_yaw * x - sin_yaw * y + translation.x
        odom_y = sin_yaw * x + cos_yaw * y + translation.y
        return odom_x, odom_y, yaw + frame_yaw

    def robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
            )
        except TransformException:
            return None
        translation = transform.transform.translation
        yaw = yaw_from_quaternion(transform.transform.rotation)
        return translation.x, translation.y, yaw

    def control_tick(self):
        command = Twist()
        if self.first_scan_seen and self.active_goal is not None:
            pose = self.robot_pose()
            if pose is not None:
                command = self.compute_command(pose)
        self.cmd_publisher.publish(command)

    def compute_command(self, pose):
        robot_x, robot_y, robot_yaw = pose
        (goal_x, goal_y, goal_yaw), _ = self.active_goal
        command = Twist()

        dx = goal_x - robot_x
        dy = goal_y - robot_y
        distance = math.hypot(dx, dy)

        if distance > self.position_tolerance:
            heading = math.atan2(dy, dx)
            heading_error = angle_difference(heading, robot_yaw)
            command.angular.z = max(
                -self.angular_speed,
                min(self.angular_speed, 2.0 * heading_error),
            )
            if abs(heading_error) < 0.5:
                command.linear.x = min(
                    self.linear_speed,
                    max(0.15, 0.8 * distance),
                )
            return command

        yaw_error = angle_difference(goal_yaw, robot_yaw)
        if abs(yaw_error) > self.yaw_tolerance:
            command.angular.z = max(
                -self.angular_speed,
                min(self.angular_speed, 2.0 * yaw_error),
            )
        return command


def main(args=None):
    rclpy.init(args=args)
    node = GoalDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
