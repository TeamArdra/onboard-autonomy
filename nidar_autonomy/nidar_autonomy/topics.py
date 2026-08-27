"""Shared topic name / value constants, kept in one place so the three
nodes (and any future ones) can't drift apart from each other or from
custom-gcs/docs/DATA_MODELS.md, which is the actual source of truth."""

COMMAND_TOPIC = "/gcs/command"
MISSION_STATE_TOPIC = "/mission/state"
HEARTBEAT_TOPIC = "/gcs/heartbeat"

# Internal-only topic, not part of the custom-gcs interface contract.
# command_node publishes here ONLY after validating a /gcs/command message
# is exactly "start" or "abort" -- mission_state_node listens to this, never
# to the raw /gcs/command, so an invalid/malformed message can never reach
# the state machine even if command_node's validation is ever bypassed
# elsewhere. See Hard Safety Rule 5 in this repo's CLAUDE.md.
VALIDATED_COMMAND_TOPIC = "/nidar_autonomy/validated_command"

VALID_COMMANDS = ("start", "abort")

# Per custom-gcs/docs/DATA_MODELS.md section 1.
MISSION_STATES = ("idle", "entering", "searching", "exiting", "complete", "aborted")
