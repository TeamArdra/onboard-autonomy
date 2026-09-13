-- nidar_2d.lua
--
-- Cartographer 2D trajectory-builder configuration for NIDAR AirMouse.
--
-- Inputs (see ../launch/slam_bringup_launch.py for the actual topic
-- remappings): a single 2D sensor_msgs/LaserScan (RPLIDAR A2M12, via
-- rplidar_ros, remapped from /scan) and sensor_msgs/Imu (remapped from
-- /mavros/imu/data, which already reports header.frame_id=base_link --
-- confirmed by direct topic inspection, so tracking_frame=base_link needs
-- no separate base_link->imu_link static transform).
--
-- No external odometry is fed in (use_odometry = false) -- matches the
-- approved architecture, which only feeds /scan + /mavros/imu/data into
-- Cartographer.
--
-- provide_odom_frame = true means this node publishes BOTH map->odom and
-- odom->base_link itself. The existing nidar_autonomy frontier_explorer_node
-- already does tf_buffer.lookup_transform("map", "base_link", ...), which
-- resolves through that two-hop chain automatically -- no code change
-- needed there.
--
-- Values not called out by the approved plan are carried over unchanged
-- from cartographer_ros's own stock reference configs
-- (/opt/ros/humble/share/cartographer_ros/configuration_files/revo_lds.lua
-- for the single-LaserScan counts, backpack_2d.lua for the base_link
-- frame-naming pattern) rather than invented here.

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = false,
  use_pose_extrapolator = true,
  use_odometry = false,
  use_nav_sat = false,
  use_landmarks = false,
  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 5e-3,
  trajectory_publish_period_sec = 30e-3,
  rangefinder_sampling_ratio = 1.,
  odometry_sampling_ratio = 1.,
  fixed_frame_pose_sampling_ratio = 1.,
  imu_sampling_ratio = 1.,
  landmarks_sampling_ratio = 1.,
}

MAP_BUILDER.use_trajectory_builder_2d = true
TRAJECTORY_BUILDER_2D.use_imu_data = true

return options
