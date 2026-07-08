"""Shared synthetic-world geometry, in the robot world frame (odom).

The robot spawns at the odom origin with zero yaw, so the digital twin
anchor (which inverts the robot pose at the first reference scan) makes the
map frame coincide with odom and these coordinates are valid in both.

The Gazebo world SDF is generated from these values
(``generate_world.py``); the synthetic camera, the LAS scan writer, and the
ground-truth detection projector all import them, so the rendered scene,
the scans, and the detections can never drift apart.

This module is intentionally dependency-free so the SDF generator can run
outside a ROS environment.
"""

# Concrete wall: a vertical plane facing the robot spawn pose. The front
# face is at x = WALL_PLANE_X; the wall extends WALL_THICKNESS behind it.
WALL_PLANE_X = 6.0
WALL_LATERAL_MIN = -8.0
WALL_LATERAL_MAX = 8.0
WALL_HEIGHT = 4.0
WALL_THICKNESS = 0.3

# Ground plane z = 0 extends from behind the robot to the wall.
GROUND_X_MIN = -4.0

# Support pillars in front of the wall face.
PILLAR_LATERALS = (-6.0, 6.0)
PILLAR_RADIUS = 0.35
PILLAR_OFFSET_FROM_WALL = 0.5

# Each defect on the wall face:
# (class_id, lateral y, height z, size in meters, base confidence).
DEFECTS = (
    ('crack', -3.0, 1.6, 0.55, 0.88),
    ('spalling', 0.5, 1.1, 0.60, 0.79),
    ('exposed_rebar', 4.0, 2.1, 0.45, 0.68),
)
