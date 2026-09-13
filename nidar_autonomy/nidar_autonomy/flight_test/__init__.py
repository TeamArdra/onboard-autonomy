"""Flight Test mission framework: a mock-only, non-flight proof-of-concept
scenario family, distinct from the Main NIDAR Competition mission owned by
mission_state_node.py / state_machine.py.

MOCK-ONLY, BY CONSTRUCTION: every node in this package constructs and
drives only `mock_flight_controller.MockFlightController` -- never
`flight_command.FlightCommandClient`, never
`mavros_flight_controller.MAVROSFlightController`, never `mavros_msgs`.
See hover_test_node.py's module docstring and README.md for the full
safety rationale. This package never touches mavros, the real Pixhawk,
or /mission/state -- it is a completely separate, additive mission
selectable only via /gcs/mission_select, per the multi-mission framework.
"""
