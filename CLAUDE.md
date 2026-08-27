# CLAUDE.md

Guidance for Claude Code (and any other agent or human) working in this
repository.

## Project Purpose

This repository is the **onboard autonomy system** for an autonomous
indoor search-and-rescue drone competing in **NIDAR AirMouse** (Track 1,
Problem Statement 2, National Innovation Challenge for Drone Application
and Research, 2026-27 edition). It runs on the drone's companion
computer (Jetson) and is responsible for everything the drone does by
itself during a mission: localizing and mapping a GPS-denied indoor
arena (SLAM), detecting and localizing survivors, deciding where to fly
(exploration/path planning), and actually commanding the flight
controller (Pixhawk 6x, via MAVLink/`mavros`) to do it.

This is the counterpart to the **`custom-gcs`** repository (sibling
directory), which is the human-operator-facing Ground Control Station.
**This repo and `custom-gcs` do not overlap in responsibility.**
`custom-gcs` only displays what this repo reports and can only send this
repo exactly two commands (start, abort) — see "The Drone ↔ GCS
Interface" below. Anything involving actual flight, navigation, mapping,
or detection algorithms belongs here, never there.

## Competition Context

Same competition, same constraints, as `custom-gcs` — see
`../custom-gcs/CLAUDE.md` for the full competition context (this section
is intentionally not duplicated to avoid the two repos' descriptions
drifting apart). In short: an earthquake-damaged, GPS-denied,
≤15 m × 15 m indoor arena, up to 6 survivors, ≤30 minutes, fully
autonomous, no post-flight processing, minimal operator command surface.

## Authoritative References

Same two documents as `custom-gcs`, at the same location (one level
above this repo, read-only, never copied into either repo):

- `../Mission Brief - NIDAR AirMouse.pdf`
- `../NIDAR-26-27-Rulebook-Ver_2_1.pdf`

This repo does not have its own `docs/REQUIREMENTS.md` yet. Until it
does, treat `../custom-gcs/docs/REQUIREMENTS.md` as the shared reference
for what the rules actually require — re-derive from the PDFs directly
for anything specific to onboard autonomy (SLAM accuracy, exploration
time budget, survivor detection scoring) that the GCS-side requirements
doc wouldn't have needed to extract.

## The Drone ↔ GCS Interface (binding contract, owned jointly, not by this repo alone)

**This repo must implement the drone side of the interface defined in
`../custom-gcs/docs/COMMUNICATION.md` §2 and `../custom-gcs/docs/DATA_MODELS.md`,
exactly as written there.** Do not redefine, fork, or drift from that
schema here — it is the single source of truth for both repos, per
`COMMUNICATION.md` §4 ("Interface Stability Note"). If a genuine need
appears to change it (new field, new topic), that is a joint decision
with whoever owns `custom-gcs`, not a unilateral edit on either side.

Summary of what this repo must do (full detail in the linked docs):

**Publish** (this repo → GCS, via `rosbridge_server` on this Jetson,
port 9090):
| Topic | Type | Rate |
|---|---|---|
| `/mission/state` | `std_msgs/String` (`"idle"\|"entering"\|"searching"\|"exiting"\|"complete"\|"aborted"`) | on change |
| `/slam/map` | `nav_msgs/OccupancyGrid`, full grid each publish, 1 m resolution | 1–5 Hz |
| `/vision/survivors` | custom `nidar_airmouse/SurvivorDetection` (this repo owns this message package — see `nidar_airmouse/`) | on detection |
| `/gcs/heartbeat` | custom, minimal | 1 Hz |

Note: `/mavros/battery` and `/mavros/local_position/pose` are already
published by `mavros` itself once it's running (see
`../NIDAR-Hardware-Bringup/`) — this repo does not need to republish
them, but **does** own making `/mavros/local_position/pose` actually
meaningful indoors, by feeding this repo's SLAM pose into the Pixhawk's
EKF via MAVLink `VISION_POSITION_ESTIMATE`. Until that exists, that
topic will keep reporting a meaningless static position — this is a
known, tracked gap (`../custom-gcs/docs/DECISIONS.md` D-11), not
something to work around on the GCS side.

**Subscribe** (GCS → this repo):
| Topic | Type | Values |
|---|---|---|
| `/gcs/command` | `std_msgs/String` | exactly `"start"` or `"abort"` — nothing else is a valid operator action, ever |

## Hard Safety Rules (non-negotiable, checked before every commit that touches flight)

1. **No code path in this repo may arm the vehicle or command motor
   output except as a direct, traceable consequence of mission logic
   that was itself triggered by a real `"start"` command on
   `/gcs/command`.** No debug shortcuts, no "just for testing" arm
   scripts committed to this repo, no auto-arm on node startup.
2. **Abort must preempt everything.** An `"abort"` command must always
   be able to interrupt whatever the mission state machine or path
   planner is doing, immediately, regardless of what else is running
   (SLAM, detection, video encode). This is the single most
   safety-critical behavior in this repo — see
   `../custom-gcs/docs/DECISIONS.md` D-10, which is explicitly still
   open on the GCS side pending a real latency test against this repo.
3. **Default to simulated/bench-safe.** New flight-adjacent code should
   be developed and tested against a simulator or with props removed
   before it is ever exercised against a live, propped vehicle. Treat
   "let's just try it on the real drone" as something to flag and get
   explicit human sign-off on, not a normal iteration step.
4. **Every arm, disarm, mode-change, and command-received event must be
   logged** with a timestamp, enough to reconstruct what happened after
   the fact — there is no post-flight processing step to fall back on
   (this is also a competition rule, not just good practice).
5. **Never accept or act on any command other than the two on
   `/gcs/command`.** If a message arrives on that topic that isn't
   exactly `"start"` or `"abort"`, log it and ignore it — do not attempt
   to interpret it as anything else.

## Architecture (current, minimal — expand as real subsystems land)

```
Pixhawk 6x  <--MAVLink/mavros-->  This repo (Jetson, ROS 2 Humble)  --rosbridge_server:9090-->  custom-gcs backend
```

Planned subsystems, **only the first three have any code yet** (see
"Current Project Phase"):

- **Command handling** (`nidar_autonomy/command_node.py`) — subscribes
  `/gcs/command`, validates it's exactly start/abort, drives the mission
  state machine.
- **Mission state machine** (`nidar_autonomy/mission_state_node.py`) —
  publishes `/mission/state`, owns the idle→entering→searching→exiting→
  complete/aborted transitions.
- **Heartbeat** (`nidar_autonomy/heartbeat_node.py`) — publishes
  `/gcs/heartbeat` at 1 Hz. Deliberately the simplest node in the repo —
  per `DATA_MODELS.md` §6, this should be the first thing proven working
  end-to-end against the real GCS, before anything else.
- **SLAM / mapping** — not started. Needs an algorithm decision (which
  SLAM stack, what sensor it consumes — camera? lidar? — hasn't been
  decided here yet) before any code exists.
- **Survivor detection/localization** — not started. Needs a detection
  model/pipeline decision. Note: `hawki_yolo11n.pt` and related YOLO
  artifacts exist elsewhere on this machine (`~/hawki_yolo11n.pt`,
  `~/cognizance2026/`) from a prior/different project — evaluate whether
  any of that is reusable before starting from scratch, but don't assume
  it's a fit without checking.
- **Exploration / path planning** — not started, depends on SLAM
  existing first.
- **Flight command issuance** (actually sending velocity/position
  setpoints or waypoints to `mavros` to make the vehicle fly
  autonomously) — not started. This is the highest-stakes code in the
  whole project; do not start it without an explicit, deliberate design
  pass (control mode, failsafe behavior, geofence, abort-preemption
  mechanism) — see Hard Safety Rules above.

## Current Project Phase

**Phase 0: scaffolding.** As of 2026-08-27:

- `nidar_airmouse/`: a ROS 2 package containing exactly one message
  definition, `SurvivorDetection.msg` (`int32 survivor_id`,
  `float64 x`, `float64 y`, `float64 confidence`), matching
  `custom-gcs/docs/DATA_MODELS.md` §5 exactly. This exists because the
  real `custom-gcs` backend already expects to subscribe to
  `nidar_airmouse/SurvivorDetection` and fails without it — see
  `../custom-gcs/PROGRESS.md` (2026-08-27 entry) for how that was found.
  **Not yet built/installed on the Jetson — see `nidar_airmouse/README.md`.**
- `nidar_autonomy/`: a ROS 2 Python package with three nodes
  (`command_node`, `mission_state_node`, `heartbeat_node`) implementing
  only the command-handling / state-reporting / heartbeat pieces of the
  table above. **No SLAM, no detection, no flight control yet.** The
  mission state machine currently only tracks
  idle/searching/complete/aborted in memory — it does not yet drive any
  real drone behavior, because there is no exploration/path-planning
  subsystem yet for it to drive.
- Nothing in this repo has been run against the real Pixhawk yet. It has
  not been tested at all yet, against sim or hardware — that's the
  immediate next step, not "done."

## Rules Claude Must Follow When Modifying This Repository

1. **Never write code that sends a real arm, takeoff, or motor-output
   command** without stopping and confirming with the user first, even
   if asked to "just do it" — see Hard Safety Rules above. Scaffolding,
   message/interface plumbing, state machines, and logging are fine to
   build proactively; anything that could make the real vehicle's motors
   spin is not.
2. **Do not change the Drone ↔ GCS interface** (topic names, message
   shapes) without also updating `custom-gcs/docs/COMMUNICATION.md` and
   `custom-gcs/docs/DATA_MODELS.md` in the same change — they are one
   contract described from two repos, per that repo's own rule 5.
3. **Never invent a competition requirement.** Same rule as `custom-gcs`
   — if it's not in the Mission Brief or Rulebook, it's an engineering
   decision (write it down, don't imply it's a rule) or an open
   question.
4. **Do not modify, move, or commit anything outside this repository.**
   Same reference-material rule as `custom-gcs`.
