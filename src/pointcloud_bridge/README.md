# Point Cloud Bridge

This package gives the defect detection pipeline a stable ROS 2 point-cloud
interface. An upstream source publishes `sensor_msgs/msg/PointCloud2` on
`/cloud/raw`, and the bridge validates and republishes it on `/cloud/points`.

```text
Upstream depth camera / simulator / scan source
  -> /cloud/raw
  -> pointcloud_bridge
  -> /cloud/points
  -> defect_detection fusion_node
```

Incoming clouds must include:

- `x`, `y`, and `z` fields
- a valid acquisition timestamp when using `timestamp_mode:=source`
- the coordinate frame in which the points are expressed

Run the bridge:

```bash
ros2 launch pointcloud_bridge pointcloud_bridge.launch.xml \
  input_topic:=/actual/cloud/topic \
  output_topic:=/cloud/points \
  cloud_frame:=actual_cloud_frame
```

`timestamp_mode:=receive` replaces the source timestamp when the Jetson receives
the message. It can help bring up an upstream driver with missing timestamps,
but it should not be used for final calibration or operation.

`cloud_frame` only sets the outgoing message frame ID. It does not rotate or
translate point coordinates. Leave it equal to the upstream frame unless the
upstream points are already expressed in the configured frame.
