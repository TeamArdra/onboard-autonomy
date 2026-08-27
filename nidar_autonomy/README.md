# nidar_autonomy

Phase 0 onboard autonomy nodes. **Command handling, mission state
reporting, and heartbeat only** — no SLAM, no survivor detection, no
flight control yet. See `../CLAUDE.md` for full scope and the hard
safety rules that govern anything added here later.

## Nodes

- **`command_node`** — subscribes `/gcs/command`, validates it's exactly
  `"start"` or `"abort"` (rejects and logs anything else), republishes
  sanitized commands on an internal-only topic.
- **`mission_state_node`** — listens to that internal sanitized topic
  (never the raw `/gcs/command`), owns the mission state machine
  (`state_machine.py`, pure logic, no ROS dependency), publishes
  `/mission/state` at 2 Hz.
- **`heartbeat_node`** — publishes `/gcs/heartbeat` at 1 Hz. No inputs,
  no logic. Get this one working end-to-end against the real GCS first.

## Known limitation

`mission_state_node` currently only reaches `idle` → `entering` →
`aborted`. It never reaches `searching`, `exiting`, or `complete` — those
transitions belong to the exploration/path-planning subsystem, which
doesn't exist yet. See that node's module docstring.

## Build (on the Jetson, ROS 2 Humble, colcon workspace)

```sh
mkdir -p ~/ros2_ws/src
ln -s ~/onboard-autonomy/nidar_autonomy ~/ros2_ws/src/nidar_autonomy
ln -s ~/onboard-autonomy/nidar_airmouse ~/ros2_ws/src/nidar_airmouse   # SurvivorDetection.msg, needed at runtime
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select nidar_airmouse nidar_autonomy
source install/setup.bash
```

## Run

```sh
ros2 run nidar_autonomy heartbeat_node    # do this one first
ros2 run nidar_autonomy command_node
ros2 run nidar_autonomy mission_state_node
```

Then, with `rosbridge_server` running (see `../../NIDAR-Hardware-Bringup/`)
and `custom-gcs`'s backend pointed at this Jetson, `/health` and
`/api/telemetry` should start showing `mission_state` change when you
send a real `"start"`/`"abort"` from `custom-gcs`'s `/docs` Swagger UI —
that round trip (GCS button → this repo → back into `/api/telemetry`) is
the first real integration test worth doing, before anything else here.

## Test

The state machine logic (`state_machine.py`) is pure Python, no ROS
needed to test it:

```sh
cd ~/onboard-autonomy/nidar_autonomy
PYTHONPATH=. python3 -m pytest test/ -q
```

12 tests, currently passing. The nodes themselves (`*_node.py`) are thin
ROS wrappers around this and are not yet covered by an integration test
— that needs a running `rosbridge_server` + real `ros2 run`, not just
`pytest`. Doing that (equivalent to `custom-gcs/sim`'s
`test_server_integration.py`) is a reasonable next step, not done yet.
