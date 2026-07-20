import numpy as np
import pytest
from vision_msgs.msg import Detection2D

from defect_detection.digital_twin.depth_fusion_node import (
    bbox_bounds,
    depth_image_to_meters,
    project_pixel_to_camera,
    robust_depth,
)


def make_detection():
    detection = Detection2D()
    detection.bbox.center.position.x = 320.0
    detection.bbox.center.position.y = 240.0
    detection.bbox.size_x = 40.0
    detection.bbox.size_y = 20.0
    return detection


def test_bbox_bounds_clamps_to_image():
    detection = make_detection()
    assert bbox_bounds(detection, 640, 480, padding_px=4) == (
        296,
        226,
        344,
        254,
    )

    detection.bbox.center.position.x = 5.0
    detection.bbox.center.position.y = 5.0
    assert bbox_bounds(detection, 640, 480, padding_px=4) == (
        0,
        0,
        29,
        19,
    )


def test_depth_image_to_meters_handles_uint16_mm():
    depth_mm = np.array([[1000, 2500]], dtype=np.uint16)
    np.testing.assert_allclose(
        depth_image_to_meters(depth_mm, '16UC1'),
        np.array([[1.0, 2.5]], dtype=np.float32),
    )


def test_robust_depth_ignores_invalid_values():
    depth = np.array([
        [0.0, np.nan, 2.0],
        [2.1, 2.2, 20.0],
    ], dtype=np.float32)
    assert robust_depth(depth) == pytest.approx(2.1, abs=0.15)


def test_project_pixel_to_camera_uses_intrinsics():
    camera_matrix = np.array([
        [400.0, 0.0, 320.0],
        [0.0, 400.0, 240.0],
        [0.0, 0.0, 1.0],
    ])
    assert project_pixel_to_camera(320.0, 240.0, 3.0, camera_matrix) == (
        0.0,
        0.0,
        3.0,
    )
    assert project_pixel_to_camera(420.0, 140.0, 2.0, camera_matrix) == (
        0.5,
        -0.5,
        2.0,
    )
