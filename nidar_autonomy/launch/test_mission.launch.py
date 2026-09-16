"""Launch file for the dev-only "test mission" (hover / find-open-space /
move / stop, repeating) -- see nidar_autonomy/test_mission_node.py and
the "First Real Test-Flight Mission" plan record for the design this
implements.

REAL FLIGHT. Does NOT launch mavros itself -- mavros's connection details
(fcu_url, baud rate, the Ethernet static-IP/port config this vehicle
uses) are environment/vehicle-specific and belong in a separate step, not
baked into this file. Start mavros first, e.g.:

    ros2 launch mavros px4.launch fcu_url:=udp://<pixhawk-ip>:14540@

then this launch file:

    ros2 launch nidar_autonomy test_mission.launch.py

This starts exactly two nodes:
  - geofence_monitor_node, pointed at /mavros/local_position/pose (the
    MTF-01-fed PX4 EKF2 output for this stage, NOT the default
    SLAM-fed VISION_POSE_TOPIC -- see geofence_monitor_node.py's own
    docstring) with placeholder arena bounds that MUST be overridden to
    the real bench/room dimensions before trusting this node's output.
  - test_mission_node, the real-flight test-mission policy itself.

Neither node is started by, or wired into, mission_state_node.py's
launch -- see topics.py's "Test-mission topics" section for the full
isolation story.
"""
from __future__ import annotations

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Placeholder bench/arena bounds -- MUST be set to the real dimensions of
# wherever this is actually run before the geofence output is trusted
# (see geofence_monitor_node.py's own "Defaults are a placeholder" note).
_DEFAULT_ARENA_HALF_EXTENT_M = "2.5"


def generate_launch_description() -> LaunchDescription:
    arena_min_x = DeclareLaunchArgument("arena_min_x", default_value=f"-{_DEFAULT_ARENA_HALF_EXTENT_M}")
    arena_max_x = DeclareLaunchArgument("arena_max_x", default_value=_DEFAULT_ARENA_HALF_EXTENT_M)
    arena_min_y = DeclareLaunchArgument("arena_min_y", default_value=f"-{_DEFAULT_ARENA_HALF_EXTENT_M}")
    arena_max_y = DeclareLaunchArgument("arena_max_y", default_value=_DEFAULT_ARENA_HALF_EXTENT_M)

    takeoff_height_m = DeclareLaunchArgument("takeoff_height_m", default_value="0.5")
    hover_duration_s = DeclareLaunchArgument("hover_duration_s", default_value="3.0")
    max_step_m = DeclareLaunchArgument("max_step_m", default_value="0.5")
    min_clearance_m = DeclareLaunchArgument("min_clearance_m", default_value="0.5")

    geofence_monitor = Node(
        package="nidar_autonomy",
        executable="geofence_monitor_node",
        name="geofence_monitor",
        output="screen",
        parameters=[
            {
                "pose_topic": "/mavros/local_position/pose",
                "arena_min_x": LaunchConfiguration("arena_min_x"),
                "arena_max_x": LaunchConfiguration("arena_max_x"),
                "arena_min_y": LaunchConfiguration("arena_min_y"),
                "arena_max_y": LaunchConfiguration("arena_max_y"),
            }
        ],
    )

    test_mission = Node(
        package="nidar_autonomy",
        executable="test_mission_node",
        name="test_mission_node",
        output="screen",
        parameters=[
            {
                "takeoff_height_m": LaunchConfiguration("takeoff_height_m"),
                "hover_duration_s": LaunchConfiguration("hover_duration_s"),
                "max_step_m": LaunchConfiguration("max_step_m"),
                "min_clearance_m": LaunchConfiguration("min_clearance_m"),
            }
        ],
    )

    return LaunchDescription(
        [
            arena_min_x,
            arena_max_x,
            arena_min_y,
            arena_max_y,
            takeoff_height_m,
            hover_duration_s,
            max_step_m,
            min_clearance_m,
            geofence_monitor,
            test_mission,
        ]
    )
