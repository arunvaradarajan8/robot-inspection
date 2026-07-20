from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


class DepthLocalizationBridge(Node):

    def __init__(self):
        super().__init__('depth_localization_bridge')

        self.declare_parameter('odom_topic', '/oak/odom')
        self.declare_parameter('odom_frame', 'depth_odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('use_message_frame_ids', True)
        self.declare_parameter('zero_z', True)

        odom_topic = self.get_parameter(
            'odom_topic'
        ).get_parameter_value().string_value
        self.odom_frame = self.get_parameter(
            'odom_frame'
        ).get_parameter_value().string_value
        self.base_frame = self.get_parameter(
            'base_frame'
        ).get_parameter_value().string_value
        self.use_message_frame_ids = self.get_parameter(
            'use_message_frame_ids'
        ).get_parameter_value().bool_value
        self.zero_z = self.get_parameter(
            'zero_z'
        ).get_parameter_value().bool_value

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            odom_topic,
            self.odom_callback,
            10,
        )
        self.get_logger().info(
            f'Using depth-camera localization from {odom_topic}; '
            f'default TF {self.odom_frame}->{self.base_frame}'
        )

    def odom_callback(self, message):
        parent_frame = self.odom_frame
        child_frame = self.base_frame
        if self.use_message_frame_ids:
            parent_frame = message.header.frame_id or parent_frame
            child_frame = message.child_frame_id or child_frame

        transform = TransformStamped()
        transform.header.stamp = message.header.stamp
        transform.header.frame_id = parent_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = (
            0.0 if self.zero_z else message.pose.pose.position.z
        )
        transform.transform.rotation = message.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = DepthLocalizationBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
