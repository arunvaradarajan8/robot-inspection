from defect_detection.defect_detection.visualization_node import (
    detections_to_marker_array,
    duration_from_seconds,
)
import pytest
from vision_msgs.msg import (
    Detection3D,
    Detection3DArray,
    ObjectHypothesisWithPose,
)
from visualization_msgs.msg import Marker


def make_detection():
    detection = Detection3D()
    detection.header.frame_id = 'cloud'
    detection.bbox.center.position.x = 1.0
    detection.bbox.center.position.y = 2.0
    detection.bbox.center.position.z = 3.0
    detection.bbox.center.orientation.w = 1.0
    detection.bbox.size.x = 0.4
    detection.bbox.size.y = 0.5
    detection.bbox.size.z = 0.6

    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = '2'
    result.hypothesis.score = 0.875
    detection.results.append(result)
    return detection


def test_duration_from_seconds():
    duration = duration_from_seconds(1.25)

    assert duration.sec == 1
    assert duration.nanosec == 250_000_000


def test_detection_markers_include_clear_box_and_label():
    detections = Detection3DArray()
    detections.header.frame_id = 'cloud'
    detections.detections.append(make_detection())

    markers = detections_to_marker_array(
        detections,
        box_alpha=0.4,
        label_height=0.2,
        lifetime_sec=0.75,
    )

    assert len(markers.markers) == 3
    assert markers.markers[0].action == Marker.DELETEALL

    box = markers.markers[1]
    assert box.header.frame_id == 'cloud'
    assert box.type == Marker.CUBE
    assert box.pose.position.x == 1.0
    assert box.pose.position.z == 3.0
    assert box.scale.x == 0.4
    assert box.color.a == pytest.approx(0.4)
    assert box.lifetime.nanosec == 750_000_000

    label = markers.markers[2]
    assert label.type == Marker.TEXT_VIEW_FACING
    assert label.text == '2 0.88'
    assert label.pose.position.z == pytest.approx(3.5)
    assert label.scale.z == 0.2


def test_array_header_is_used_when_detection_header_is_empty():
    detections = Detection3DArray()
    detections.header.frame_id = 'cloud'
    detection = make_detection()
    detection.header.frame_id = ''
    detections.detections.append(detection)

    markers = detections_to_marker_array(detections)

    assert markers.markers[1].header.frame_id == 'cloud'
    assert markers.markers[2].header.frame_id == 'cloud'


def test_empty_detection_array_clears_stale_markers():
    markers = detections_to_marker_array(Detection3DArray())

    assert len(markers.markers) == 1
    assert markers.markers[0].action == Marker.DELETEALL
