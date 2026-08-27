# onboard-autonomy

Drone-side onboard autonomy for NIDAR AirMouse — SLAM, exploration/path
planning, survivor detection, mission state, and flight command issuance
to the Pixhawk. Runs on the Jetson. Counterpart to the `custom-gcs` repo
(sibling directory), which is the operator-facing GCS this repo talks to
over `rosbridge_server`.

**Read `CLAUDE.md` first** — it defines scope, the binding Drone↔GCS
interface contract (owned jointly with `custom-gcs`, not redefined here),
and hard safety rules for anything that touches real flight.

## What exists right now (Phase 0, started 2026-08-27)

- `nidar_airmouse/` — one ROS 2 message (`SurvivorDetection.msg`),
  needed because the real GCS backend expects it and fails without it.
  Not yet built/installed on the Jetson.
- `nidar_autonomy/` — three ROS 2 nodes: command handling, mission state
  reporting, heartbeat. Nothing else. Not yet built, run, or tested
  against real ROS/rosbridge (the state-machine logic has unit tests and
  they pass; the nodes themselves haven't been run yet).

## What does NOT exist yet

SLAM, survivor detection, exploration/path planning, and — the biggest
one — actually sending flight/motor commands to make the vehicle fly
autonomously. See `CLAUDE.md`'s Architecture section for the honest
status of each. None of these were started here because they're either
large algorithmic subsystems needing a real design pass, or (for flight
commands specifically) too safety-critical to scaffold speculatively —
see `CLAUDE.md`'s Hard Safety Rules.

## Related repos

- `../custom-gcs/` — the GCS this repo serves telemetry to and takes
  exactly two commands from.
- `../NIDAR-Hardware-Bringup/` — the Pixhawk↔Jetson hardware/network
  bring-up notes (MAVROS, rosbridge_server setup). Read this before
  trying to run anything here against real hardware.
