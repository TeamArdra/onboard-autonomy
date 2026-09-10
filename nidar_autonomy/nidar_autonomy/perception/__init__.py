"""Perception subsystem: camera capture -> pluggable person-detector ->
normalized Detection contract -> /perception/detections + /perception/status.

READ-ONLY OBSERVER, same as telemetry_bridge_node.py / coverage_tracker_node.py
/ geofence_monitor_node.py / frontier_explorer_node.py -- see perception_node.py's
module docstring. This package makes no flight decisions and imports nothing
mavros-related, mission-state-related, or arming-related.

Does NOT touch /vision/survivors (nidar_airmouse.SurvivorDetection) -- that is
a separate, still-unimplemented, localized/confirmed-survivor contract owned
elsewhere. This package's Detection is a raw per-frame, per-model detection,
not a localized survivor.
"""
