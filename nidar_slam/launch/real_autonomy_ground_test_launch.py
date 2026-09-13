"""Ground-test bringup: REAL Cartographer SLAM + the existing frontier
exploration pipeline, with NO command path to MAVROS/the flight controller.

Includes slam_bringup_launch.py (static base_link->laser TF placeholder +
cartographer_node + cartographer_occupancy_grid_node) and additionally
starts nidar_autonomy's frontier_explorer_node.

Output chain only, per the approved integration scope:

    REAL /map  ->  frontier detection  ->  A* planned path
                   (/planned_path, /explorer/status)

frontier_explorer_node never imports mavros_msgs and never calls any
mavros service -- see its own module docstring in
nidar_autonomy/frontier_explorer_node.py. Nothing in this launch file
changes that.

Does NOT start (same LiDAR-off convention as slam_bringup_launch.py, plus
the mission stack):
  - The physical LiDAR (rplidar_ros). Launch it separately, e.g.:
        ros2 launch rplidar_ros rplidar_a2m12_launch.py
    The mounted sensor is an RPLIDAR A2M12 -- never rplidar_a3_launch.py.
  - mavros. Already launched separately, per
    CHECKPOINT/CURRENT_STATE.md Section 5.
  - mission_state_node / command_node / heartbeat_node (nidar_autonomy's
    mission-control nodes). Deliberately excluded so this ground test has
    no path to ARM/DISARM or any other flight command, however indirect.

This file is additive-only: it does not modify, replace, or namespace-
collide with the /simulation/* pipeline (simulation_node.py /
mission_simulator.py), which remains fully independent and is unaffected
by anything here.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    slam_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('nidar_slam'), '/launch/slam_bringup_launch.py']
        )
    )

    frontier_explorer_node = Node(
        package='nidar_autonomy',
        executable='frontier_explorer_node',
        name='frontier_explorer_node',
        output='screen',
    )

    return LaunchDescription([
        slam_bringup,
        frontier_explorer_node,
    ])
