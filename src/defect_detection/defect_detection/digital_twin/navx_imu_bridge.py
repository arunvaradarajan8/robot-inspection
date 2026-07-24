"""Publish a navX AHRS/IMU as sensor_msgs/Imu for the fused-localization EKF.

The fused-localization EKF (robot_localization) blends Spot's vision-frame
pose, this IMU, and the depth camera's visual odometry. The navX supplies an
absolute, drift-corrected heading that keeps yaw from wandering over a long
excursion, which is what a clean walk home depends on.

Two ways to get the navX data in:

* ``relay`` (default) subscribes to an ``sensor_msgs/Imu`` topic that some
  other driver already publishes (a vendor navX node, or a microcontroller
  reading the board) and republishes it under this node's frame_id and
  covariances. Always correct; needs an upstream Imu publisher.

* ``serial`` reads the navX directly over USB/UART using its ASCII streaming
  protocol. Needs ``pyserial``. The board must be streaming the ASCII 'y'
  (yaw/pitch/roll) sentence. Field widths follow the published navX-MXP ASCII
  protocol; if your firmware differs, adjust ``_YPR_FIELD_WIDTHS`` and verify
  against ``ros2 topic echo /navx/imu`` on the Jetson before trusting it.
"""

import math

from geometry_msgs.msg import Quaternion
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


# navX-MXP ASCII 'y' (YPR) sentence: '!y' + yaw + pitch + roll + heading
# + '*' + 2-char hex checksum + CRLF. Each angle is a fixed-width signed
# decimal; heading is unsigned. Widths per the navX-MXP ASCII protocol.
_YPR_PREFIX = '!y'
_YPR_FIELD_WIDTHS = (7, 7, 7, 6)  # yaw, pitch, roll, compass heading


def quaternion_from_euler(roll, pitch, yaw):
    """Return a geometry_msgs/Quaternion from roll/pitch/yaw in radians."""
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


def parse_ypr_sentence(line):
    """Parse a navX ASCII 'y' sentence into (yaw, pitch, roll) in radians.

    Returns None if the line is not a well-formed, checksum-valid YPR
    sentence, so a partial or corrupt read is simply skipped.
    """
    if not line.startswith(_YPR_PREFIX) or '*' not in line:
        return None
    body, _, checksum = line.partition('*')
    checksum = checksum.strip()
    # Checksum is the XOR of every character after '!' and before '*'.
    computed = 0
    for char in body[1:]:
        computed ^= ord(char)
    try:
        if int(checksum[:2], 16) != computed:
            return None
    except ValueError:
        return None

    fields = body[len(_YPR_PREFIX):]
    expected = sum(_YPR_FIELD_WIDTHS)
    if len(fields) < expected:
        return None
    values = []
    offset = 0
    for width in _YPR_FIELD_WIDTHS:
        chunk = fields[offset:offset + width]
        offset += width
        try:
            values.append(float(chunk))
        except ValueError:
            return None
    yaw_deg, pitch_deg, roll_deg, _heading = values
    return (
        math.radians(yaw_deg),
        math.radians(pitch_deg),
        math.radians(roll_deg),
    )


class NavxImuBridge(Node):

    def __init__(self):
        super().__init__('navx_imu_bridge')

        self.declare_parameter('mode', 'relay')
        self.declare_parameter('imu_topic', '/navx/imu')
        self.declare_parameter('frame_id', 'navx_imu')
        # relay mode
        self.declare_parameter('input_imu_topic', '/navx/imu_raw')
        # serial mode
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('serial_poll_hz', 100.0)
        # Covariances (stddev on the diagonal). A negative value marks a
        # field the EKF should ignore. The 'y' sentence carries orientation
        # only, so angular velocity and acceleration default to ignored.
        self.declare_parameter('orientation_stddev', 0.05)
        self.declare_parameter('angular_velocity_stddev', -1.0)
        self.declare_parameter('linear_acceleration_stddev', -1.0)

        self.mode = self.get_parameter('mode').value
        self.frame_id = self.get_parameter('frame_id').value
        imu_topic = self.get_parameter('imu_topic').value
        self.orientation_var = self._variance('orientation_stddev')
        self.angular_var = self._variance('angular_velocity_stddev')
        self.linear_var = self._variance('linear_acceleration_stddev')

        self.publisher = self.create_publisher(Imu, imu_topic, 10)

        if self.mode == 'relay':
            self.subscription = self.create_subscription(
                Imu,
                self.get_parameter('input_imu_topic').value,
                self.relay_callback,
                10,
            )
            self.get_logger().info(
                f'Relaying {self.get_parameter("input_imu_topic").value} '
                f'to {imu_topic} as frame {self.frame_id}'
            )
        elif self.mode == 'serial':
            self.serial = self._open_serial()
            self.buffer = ''
            poll_hz = float(self.get_parameter('serial_poll_hz').value)
            self.timer = self.create_timer(1.0 / max(1.0, poll_hz), self.read_serial)
            self.get_logger().info(
                f'Reading navX at {self.get_parameter("serial_port").value} '
                f'and publishing {imu_topic} as frame {self.frame_id}'
            )
        else:
            raise ValueError("mode must be 'relay' or 'serial'")

    def _variance(self, name):
        stddev = float(self.get_parameter(name).value)
        return -1.0 if stddev < 0.0 else stddev * stddev

    def _open_serial(self):
        try:
            import serial
        except ImportError as error:
            raise RuntimeError(
                'pyserial is not installed. Run "pip3 install pyserial" on '
                'the Jetson before using navx_mode:=serial.'
            ) from error
        return serial.Serial(
            self.get_parameter('serial_port').value,
            int(self.get_parameter('serial_baud').value),
            timeout=0.0,
        )

    # ---- relay mode ----------------------------------------------------

    def relay_callback(self, message):
        message.header.frame_id = self.frame_id
        self._apply_covariances(message)
        self.publisher.publish(message)

    # ---- serial mode ---------------------------------------------------

    def read_serial(self):
        try:
            chunk = self.serial.read(512)
        except Exception as error:  # noqa: BLE001 - report and keep spinning
            self.get_logger().warning(
                f'navX serial read failed: {error}',
                throttle_duration_sec=10.0,
            )
            return
        if not chunk:
            return
        self.buffer += chunk.decode('ascii', errors='ignore')
        while '\n' in self.buffer:
            line, _, self.buffer = self.buffer.partition('\n')
            self.handle_line(line.strip('\r'))

    def handle_line(self, line):
        parsed = parse_ypr_sentence(line)
        if parsed is None:
            return
        roll, pitch, yaw = parsed[2], parsed[1], parsed[0]
        message = Imu()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        message.orientation = quaternion_from_euler(roll, pitch, yaw)
        self._apply_covariances(message)
        self.publisher.publish(message)

    # ---- shared --------------------------------------------------------

    def _apply_covariances(self, message):
        message.orientation_covariance = self._diagonal(self.orientation_var)
        message.angular_velocity_covariance = self._diagonal(self.angular_var)
        message.linear_acceleration_covariance = self._diagonal(self.linear_var)

    @staticmethod
    def _diagonal(variance):
        # robot_localization reads a -1 in the first element as "ignore this
        # sensor's data for that quantity".
        cov = [0.0] * 9
        cov[0] = variance
        cov[4] = variance
        cov[8] = variance
        return cov


def main(args=None):
    rclpy.init(args=args)
    node = NavxImuBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
