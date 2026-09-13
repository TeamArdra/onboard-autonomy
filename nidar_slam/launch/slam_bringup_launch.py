"""SLAM bringup for NIDAR AirMouse (Cartographer 2D).

Starts, in this launch file only:
  1. A static base_link -> laser transform.
  2. cartographer_ros's cartographer_node (scan + IMU in, TF + pose graph out).
  3. cartographer_ros's cartographer_occupancy_grid_node (/map out).

Does NOT start:
  - The physical LiDAR (rplidar_ros). Launch it separately, e.g.:
        ros2 launch rplidar_ros rplidar_a2m12_launch.py
    The mounted sensor is an RPLIDAR A2M12 -- use that launch file, never
    rplidar_a3_launch.py.
  - mavros. Already launched separately, per CHECKPOINT/CURRENT_STATE.md
    Section 5 (`ros2 launch mavros apm.launch fcu_url:=...`).

Keeping both of the above out of this file is deliberate: turning the
physical LiDAR on (and re-arming/flying) stays a distinct, explicit,
human-gated step, never a side effect of bringing the SLAM stack up.

*** BENCH-TEST PLACEHOLDER -- NOT FLIGHT-READY ***
The static_transform_publisher below publishes an IDENTITY base_link->laser
transform (zero translation, zero rotation: --x/--y/--z/--roll/--pitch/--yaw
all 0) because the real RPLIDAR A2M12 mount offset has not been physically
measured yet. This is only geometrically correct if the LiDAR's optical
center sits exactly at base_link's origin with no rotation, which is not
expected to be true on the real airframe. Any map produced while this
placeholder is in effect will be systematically skewed by however far the
real mount actually differs from identity. Replace the six values below
with the measured mount offset (and rename the node away from
"_BENCH_PLACEHOLDER") before relying on this for real indoor mapping,
navigation, or flight.
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    cartographer_config_dir = FindPackageShare('nidar_slam').find('nidar_slam') + '/config'
    configuration_basename = 'nidar_2d.lua'

    # *** BENCH PLACEHOLDER -- NOT flight-ready, NOT the measured mount offset. ***
    static_tf_base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_link_to_laser_BENCH_PLACEHOLDER',
        output='screen',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser',
        ],
    )

    cartographer_node = Node(
        package='cartographer_ros',
        executable='cartographer_node',
        name='cartographer_node',
        output='screen',
        arguments=[
            '-configuration_directory', cartographer_config_dir,
            '-configuration_basename', configuration_basename,
        ],
        remappings=[
            ('scan', '/scan'),
            ('imu', '/mavros/imu/data'),
        ],
    )

    cartographer_occupancy_grid_node = Node(
        package='cartographer_ros',
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node',
        output='screen',
        parameters=[{'resolution': 0.05}],
        remappings=[
            ('map', '/map'),
        ],
    )

    return LaunchDescription([
        static_tf_base_link_to_laser,
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])
