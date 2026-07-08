"""Point sampling and LAS export for the shared synthetic world.

Used by the pure-Python synthetic demo (``synthetic_field_node``) and the
Gazebo simulation (``sim_scan_node``) so both produce identical Trimble
reference scans of the world described in ``world_constants``.
"""

import math

import numpy as np

from defect_detection.simulation.world_constants import (
    DEFECTS,
    GROUND_X_MIN,
    PILLAR_LATERALS,
    PILLAR_OFFSET_FROM_WALL,
    PILLAR_RADIUS,
    WALL_HEIGHT,
    WALL_LATERAL_MAX,
    WALL_LATERAL_MIN,
    WALL_PLANE_X,
    WALL_THICKNESS,
)


def structure_points(rng, density):
    wall_count = int(9000 * density)
    wall = np.column_stack((
        rng.uniform(WALL_PLANE_X, WALL_PLANE_X + WALL_THICKNESS, wall_count),
        rng.uniform(WALL_LATERAL_MIN, WALL_LATERAL_MAX, wall_count),
        rng.uniform(0.0, WALL_HEIGHT, wall_count),
    ))

    ground_count = int(7000 * density)
    ground = np.column_stack((
        rng.uniform(GROUND_X_MIN, WALL_PLANE_X, ground_count),
        rng.uniform(WALL_LATERAL_MIN - 2.0, WALL_LATERAL_MAX + 2.0, ground_count),
        rng.normal(0.0, 0.015, ground_count),
    ))

    pillars = []
    pillar_count = int(1200 * density)
    pillar_x = WALL_PLANE_X - PILLAR_OFFSET_FROM_WALL
    for pillar_y in PILLAR_LATERALS:
        angle = rng.uniform(0.0, 2.0 * math.pi, pillar_count)
        pillars.append(np.column_stack((
            pillar_x + PILLAR_RADIUS * np.cos(angle),
            pillar_y + PILLAR_RADIUS * np.sin(angle),
            rng.uniform(0.0, WALL_HEIGHT, pillar_count),
        )))

    divots = []
    for _, lateral, height, size, _ in DEFECTS:
        divot_count = int(500 * density)
        divots.append(np.column_stack((
            np.full(divot_count, WALL_PLANE_X - 0.03),
            rng.normal(lateral, size / 4.0, divot_count),
            rng.normal(height, size / 4.0, divot_count),
        )))

    points = np.vstack([wall, ground] + pillars + divots)
    return points + rng.normal(0.0, 0.008, points.shape)


def write_las(points, path):
    """Write an Nx3 point array as a LAS 1.2 file. Requires laspy."""
    import laspy

    header = laspy.LasHeader(point_format=0, version='1.2')
    header.offsets = points.min(axis=0)
    header.scales = (0.001, 0.001, 0.001)
    las = laspy.LasData(header)
    las.x = points[:, 0]
    las.y = points[:, 1]
    las.z = points[:, 2]
    las.write(str(path))
