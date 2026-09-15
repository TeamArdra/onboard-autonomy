"""Flight Test bringup: the mock-only Mission/Test Selector's onboard
half.

Starts, in this launch file only:
  1. command_node       -- validates /gcs/command ("start"/"abort" only).
  2. hover_test_node    -- the "hover" scenario (flight_test/hover_test_node.py).
  3. multi_step_test_node -- "Test 1"/"Test 2" and future ordered-step
     scenarios (flight_test/multi_step_test_node.py,
     flight_test/scenarios.py).

MOCK-ONLY, BY CONSTRUCTION: neither test node ever imports mavros_msgs or
constructs a MAVROSFlightController -- see each node's own module
docstring and flight_test/README.md. This launch file does NOT start
mission_state_node, heartbeat_node, or mavros -- same deliberate
exclusion `real_autonomy_ground_test_launch.py` already uses for the same
reason (no path to ARM/DISARM or any other real flight command), except
here it's doubly moot: even if mission_state_node were included, nothing
in this launch file could ever reach it, because the Flight Test nodes
only ever act on a "start" whose accompanying /gcs/mission_select
selection has mission_id="flight_test" -- mission_state_node (the
Main NIDAR Competition / real ARM-capable path) explicitly ignores those
selections and only ever acts on mission_id="main_nidar" or no
selection at all (see mission_state_node.py's "MISSION SELECTION GUARD"
docstring section). Running this launch file is safe to do at any time,
on any machine, with or without a Pixhawk attached -- it never touches
mavros or /mavros/* at all.

Run:
    ros2 launch nidar_autonomy flight_test_launch.py

Then, from the GCS (or directly on the CLI for a bench-test dry run
without the GCS UI at all):
    ros2 topic pub -1 /gcs/mission_select std_msgs/String \\
        '{data: "{\\"mission_id\\": \\"flight_test\\", \\"scenario_id\\": \\"forward_backward_hover\\"}"}'
    ros2 topic pub -1 /gcs/command std_msgs/String '{data: "start"}'
    ros2 topic echo /flight_test/multi_step/status
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package="nidar_autonomy", executable="command_node", name="command_node", output="screen"),
        Node(package="nidar_autonomy", executable="hover_test_node", name="hover_test_node", output="screen"),
        Node(
            package="nidar_autonomy",
            executable="multi_step_test_node",
            name="multi_step_test_node",
            output="screen",
        ),
    ])
