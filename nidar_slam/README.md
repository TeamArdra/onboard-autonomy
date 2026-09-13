# nidar_slam

Cartographer 2D SLAM bringup. Launch/config only — no nodes of its own.

```
/scan (RPLIDAR A2M12 via rplidar_ros)  ─┐
                                        ├─▶ cartographer_node ─▶ /map (via
/mavros/imu/data (frame_id=base_link) ─┘    cartographer_occupancy_grid_node)
                                             + map->odom->base_link TF
```

Feeds the existing `nidar_autonomy` `frontier_explorer_node` — no code
changes there, it already expects `/map` and a `map->base_link` TF lookup.

## What this launch file does NOT do

`launch/slam_bringup_launch.py` never starts the physical LiDAR or mavros.
Both are separate, explicit operator commands:

```sh
ros2 launch rplidar_ros rplidar_a2m12_launch.py   # the mounted sensor is an
                                                   # A2M12 -- never use
                                                   # rplidar_a3_launch.py
ros2 launch mavros apm.launch fcu_url:=udp://@192.168.144.14:14550
```

## BENCH-TEST PLACEHOLDER — NOT FLIGHT-READY

The `static_transform_publisher` node in `slam_bringup_launch.py`
(`static_tf_base_link_to_laser_BENCH_PLACEHOLDER`) publishes an **identity**
`base_link -> laser` transform: zero translation, zero rotation. This is a
placeholder only, used because the real RPLIDAR A2M12 mount offset has not
been physically measured yet. It is almost certainly wrong on the real
airframe, and any map produced while it's in effect will be systematically
skewed by whatever the true mount offset actually is.

**Before this is used for any real indoor mapping, navigation, or flight**:
measure the physical offset (translation in meters, rotation in
roll/pitch/yaw radians) from `base_link`'s origin to the A2M12's optical
center, and update the six `--x/--y/--z/--roll/--pitch/--yaw` values in
`slam_bringup_launch.py` accordingly.
