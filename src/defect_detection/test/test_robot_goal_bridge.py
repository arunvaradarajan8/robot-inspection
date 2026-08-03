import math
import os

from geometry_msgs.msg import PoseStamped, TransformStamped
import pytest
import rclpy

from defect_detection.digital_twin.robot_goal_bridge import (
    RobotGoalBridge,
    compose_body_goal_in_spot_frame,
    normalize_angle,
)


@pytest.mark.parametrize(
    ('angle', 'expected'),
    [
        (0.0, 0.0),
        (math.pi, math.pi),
        (-math.pi, -math.pi),
        (3.0 * math.pi, math.pi),
        (-3.0 * math.pi, -math.pi),
        (2.5 * math.pi, 0.5 * math.pi),
    ],
)
def test_normalize_angle(angle, expected):
    assert normalize_angle(angle) == pytest.approx(expected)


def make_transform(x, y, yaw):
    transform = TransformStamped()
    transform.header.frame_id = 'map'
    transform.child_frame_id = 'body'
    transform.transform.translation.x = x
    transform.transform.translation.y = y
    transform.transform.rotation.z = math.sin(yaw / 2.0)
    transform.transform.rotation.w = math.cos(yaw / 2.0)
    return transform


def make_goal(x, y, yaw):
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.position.x = x
    goal.pose.position.y = y
    goal.pose.orientation.z = math.sin(yaw / 2.0)
    goal.pose.orientation.w = math.cos(yaw / 2.0)
    return goal


def test_arrival_error_uses_tf_distance_and_yaw():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = RobotGoalBridge()
    try:
        now = node.get_clock().now().to_msg()
        transform = make_transform(1.0, 2.0, 0.25)
        transform.header.stamp = now
        node.tf_buffer.set_transform(transform, 'test')

        distance_error, yaw_error = node.arrival_error(make_goal(1.1, 2.2, 0.35))

        assert distance_error == pytest.approx(math.hypot(0.1, 0.2))
        assert yaw_error == pytest.approx(0.10)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_compose_body_goal_identity_spot_pose():
    # Spot at its own origin: the body-relative goal passes straight through.
    assert compose_body_goal_in_spot_frame(0.0, 0.0, 0.0, 2.0, 1.0, 0.5) == (
        pytest.approx(2.0),
        pytest.approx(1.0),
        pytest.approx(0.5),
    )


def test_compose_body_goal_rotates_with_spot_heading():
    # Spot at (10, 5) facing +90 deg: "2m ahead, 1m left" of the body lands
    # at (10 - 1, 5 + 2) in Spot's frame, heading rotated with the body.
    x, y, heading = compose_body_goal_in_spot_frame(
        10.0, 5.0, math.pi / 2.0, 2.0, 1.0, 0.25,
    )
    assert x == pytest.approx(9.0)
    assert y == pytest.approx(7.0)
    assert heading == pytest.approx(math.pi / 2.0 + 0.25)


def test_compose_body_goal_normalizes_heading():
    _, _, heading = compose_body_goal_in_spot_frame(
        0.0, 0.0, math.pi, 0.0, 0.0, math.pi,
    )
    assert -math.pi <= heading <= math.pi


def test_body_relative_goal_uses_camera_tf():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = RobotGoalBridge()
    try:
        # Camera localization: map->body has the robot at (1, 2) yaw 0.
        transform = make_transform(1.0, 2.0, 0.0)
        transform.header.stamp = node.get_clock().now().to_msg()
        node.tf_buffer.set_transform(transform, 'test')

        dx, dy, dyaw = node.body_relative_goal(make_goal(3.0, 2.0, 0.5))

        # Goal at (3, 2) is 2m straight ahead of the body.
        assert dx == pytest.approx(2.0)
        assert dy == pytest.approx(0.0)
        assert dyaw == pytest.approx(0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_arrival_error_identifies_far_pose():
    os.environ['ROS_LOG_DIR'] = '/tmp'
    rclpy.init()
    node = RobotGoalBridge()
    try:
        transform = make_transform(0.0, 0.0, 0.0)
        transform.header.stamp = node.get_clock().now().to_msg()
        node.tf_buffer.set_transform(transform, 'test')

        distance_error, yaw_error = node.arrival_error(make_goal(2.0, 0.0, 1.0))

        assert distance_error == pytest.approx(2.0)
        assert yaw_error == pytest.approx(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
