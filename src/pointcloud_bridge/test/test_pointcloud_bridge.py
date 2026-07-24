from builtin_interfaces.msg import Time
import pytest
from sensor_msgs.msg import PointCloud2, PointField

from pointcloud_bridge.pointcloud_bridge import (
    normalize_cloud,
    validate_timestamp,
)


def make_cloud(field_names):
    cloud = PointCloud2()
    cloud.header.frame_id = 'upstream_cloud'
    cloud.fields = [
        PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32)
        for index, name in enumerate(field_names)
    ]
    return cloud


def test_normalize_cloud_overrides_frame_and_preserves_input():
    cloud = make_cloud(['x', 'y', 'z', 'intensity'])

    normalized = normalize_cloud(cloud, 'cloud')

    assert normalized.header.frame_id == 'cloud'
    assert cloud.header.frame_id == 'upstream_cloud'
    assert [field.name for field in normalized.fields] == [
        'x',
        'y',
        'z',
        'intensity',
    ]


def test_normalize_cloud_requires_xyz_fields():
    cloud = make_cloud(['x', 'y'])

    with pytest.raises(ValueError, match='z'):
        normalize_cloud(cloud, 'cloud')


def test_validate_timestamp_accepts_recent_source_time():
    stamp = Time(sec=10, nanosec=900_000_000)

    offset = validate_timestamp(
        stamp,
        now_nanoseconds=11_000_000_000,
        max_age_sec=0.5,
        max_future_sec=0.05,
    )

    assert offset == pytest.approx(0.1)


@pytest.mark.parametrize(
    ('stamp', 'message'),
    [
        (Time(), 'zero'),
        (Time(sec=10), 'stale'),
        (Time(sec=11, nanosec=100_000_000), 'future'),
    ],
)
def test_validate_timestamp_rejects_invalid_times(stamp, message):
    with pytest.raises(ValueError, match=message):
        validate_timestamp(
            stamp,
            now_nanoseconds=11_000_000_000,
            max_age_sec=0.5,
            max_future_sec=0.05,
        )


def test_normalize_cloud_can_stamp_on_receive():
    cloud = make_cloud(['x', 'y', 'z'])
    receive_stamp = Time(sec=12, nanosec=34)

    normalized = normalize_cloud(
        cloud,
        'cloud',
        receive_stamp=receive_stamp,
    )

    assert normalized.header.stamp == receive_stamp
