# Telemetry-Radio START/ABORT — Full Context

Self-contained briefing for the "start the mission over the telemetry
radio" feature. Hand this file to any engineer or AI assistant; it should
not need anything else to understand what exists, why, and how to run,
test, or extend it.

Status as of 2026-10-06: **implemented and tested in software (unit tests
+ a simulated end-to-end run on a laptop); NOT yet run on the real
Jetson/Pixhawk.** Branch `feature/telem-radio-command` in both repos
(`onboard-autonomy` and `custom-gcs`, GitHub org TeamArdra).

---

## 1. Project background (minimum you need)

- **NIDAR AirMouse** (NIDAR 2026-27, Track 1, PS2): an autonomous indoor,
  GPS-denied search-and-rescue drone. ≤15×15 m maze arena, up to 6
  survivors, ≤30 min, fully autonomous.
- **The operator may only ever send two commands: START and ABORT.** Any
  other operator input is a scored penalty. This is a hard rule across
  the whole codebase.
- Hardware:
  - Flight controller: **Pixhawk 6X, ArduCopter 4.6.3** (not PX4).
  - Companion computer: **Jetson Orin Nano**, Ubuntu 22.04, **ROS 2
    Humble**, **MAVROS**.
  - Pixhawk ↔ Jetson: **Ethernet**, MAVLink2 UDP. Pixhawk at
    `192.168.144.14:14550` (UDP server), Jetson `192.168.144.1` on `eno1`.
    MAVROS runs with `fcu_url:=udp://@192.168.144.14:14550`.
  - Telemetry radio: **MicroLR900** (900 MHz serial MAVLink radio). One
    end on a Pixhawk TELEM port, the other on the laptop via USB.
- Repos (sibling dirs under `NIDAR/`):
  - `onboard-autonomy` — Jetson-side ROS 2 code (`nidar_autonomy`
    package): command handling, mission state machine, ARM/DISARM via
    MAVROS, mapping/exploration (not flying yet), simulation.
  - `custom-gcs` — laptop GCS: FastAPI backend (talks to the Jetson's
    `rosbridge_server` on port 9090 over Wi-Fi via roslibpy) + React
    frontend.
  - `gps_denied` — older SLAM/exploration repo, being folded into
    onboard-autonomy.

## 2. Existing command path (before this feature)

```
GCS frontend --REST--> GCS backend (FastAPI) --rosbridge/Wi-Fi--> Jetson
  /gcs/command (std_msgs/String "start"|"abort")
    -> command_node.py         validates it is exactly "start" or "abort",
                               republishes on /nidar_autonomy/validated_command
    -> mission_state_node.py   state machine idle->entering on "start";
                               "start" from idle triggers a real ARM via MAVROS
                               (arm_trigger.py / flight_command.py / arming_guard.py);
                               "abort" always triggers DISARM.
```

Important existing behavior:
- A "start" when the mission is not `idle` is ignored (no double-arm).
- ABORT currently = **DISARM** (fine on bench; in flight it would drop the
  drone — redesigning abort is deliberately out of scope for now).
- Nothing in the system sends flight setpoints yet; START only arms.

## 3. The goal

Be able to **start the mission (= arm, as of today) over the MicroLR900
telemetry radio**, without depending on Wi-Fi/rosbridge. Abort over radio
comes for free (same mechanism) but its semantics were left unchanged.
Telemetry, map, and video still go over the existing Wi-Fi channels.

## 4. Design

```
Laptop: custom-gcs/tools/telem_command.py (pymavlink, sysid 255 / comp 190)
   | USB serial, 57600 baud
MicroLR900 ))))) MicroLR900
   | Pixhawk TELEM port (MAVLink2)
Pixhawk 6X (ArduPilot) -- routes the frame, does NOT execute it,
   |                      because it is addressed to mavros's (sysid, compid)
   | Ethernet / UDP
MAVROS on Jetson -- publishes every received frame raw on /uas1/mavlink_source
   |
telem_command_bridge_node.py -- decodes, checks, publishes "start"/"abort"
   |                            on /gcs/command, replies COMMAND_ACK via
   |                            /uas1/mavlink_sink (same route back)
command_node.py -> mission_state_node.py -> ARM/DISARM   (UNCHANGED)
```

Why this design:
- **Zero changes to the safety-critical path.** The bridge only publishes
  the same string the Wi-Fi GCS publishes; validation and arm logic are
  untouched (onboard-autonomy CLAUDE.md Hard Safety Rules 1 and 5).
- **No Pixhawk network changes**: reuses MAVROS's existing Ethernet link
  and ArduPilot's built-in MAVLink routing between ports.
- **No pymavlink needed on the Jetson**: the codec is hand-written pure
  Python (struct + X.25 CRC), verified byte-for-byte against pymavlink in
  tests.

### 4.1 Wire format

Request: `COMMAND_LONG` (msgid 76)
| Field | Value |
|---|---|
| target_system / target_component | MAVROS's own `system_id` / `component_id` (default assumed **1 / 240**) |
| command | `MAV_CMD_USER_1` = 31010 |
| param1 | `1.0` = start, `2.0` = abort |
| param2 | nonce (random int; reused on every resend of the same press) |
| param3 | `4242.0` magic (rejects stray MAV_CMD_USER_1 traffic) |
| param4–7, confirmation | 0 |

Reply: `COMMAND_ACK` (msgid 77), command=31010, result=0 (ACCEPTED),
from MAVROS's sysid/compid, targeted at the sender (255/190).

Rules in the bridge:
- Anything that isn't a CRC-OK COMMAND_LONG with command 31010, correct
  target, magic 4242, and param1 ∈ {1, 2} is silently ignored.
- Each (sender sysid, sender compid, nonce) is forwarded to `/gcs/command`
  **once**; resends are re-ACKed only (last 32 nonces remembered).
- Every accepted command is logged with a timestamp (Hard Safety Rule 4).

Why the target must be MAVROS's exact id: ArduPilot only forwards a
targeted message to a link where it has seen that (sysid, compid) — MAVROS
heartbeats from its own id on the Ethernet link, so that's the route.
Wrong component id → Pixhawk silently drops it → laptop tool reports
"No ACK".

## 5. Files

onboard-autonomy (`nidar_autonomy/`):
- `nidar_autonomy/telem_command_codec.py` — pure: `decode_command_long`,
  `parse_radio_command` (→ `RadioCommand(command, nonce)` or None),
  `encode_command_ack` (builds a finished MAVLink2 frame incl. checksum,
  because MAVROS's router forwards `mavlink_sink` frames as-is),
  `payload64_to_bytes` / `bytes_to_payload64` (mavros_msgs/Mavlink stores
  payload as uint64 words), `x25_crc`. Constants: `NIDAR_START`,
  `NIDAR_ABORT`, `NIDAR_MAGIC`, `MAV_CMD_USER_1`.
- `nidar_autonomy/telem_command_bridge_node.py` — rclpy node
  `telem_command_bridge_node`. Params: `own_system_id` (1),
  `own_component_id` (240), `mavlink_source_topic`
  (`/uas1/mavlink_source`), `mavlink_sink_topic` (`/uas1/mavlink_sink`).
  Registered in `setup.py` console_scripts.
- `test/test_telem_command_codec.py` — 15 tests incl. pymavlink
  cross-checks (skipped if pymavlink missing).
- `test/test_telem_command_bridge_node.py` — rclpy tests (skipped without
  ROS): forward+ACK, abort, nonce dedupe, ignoring unrelated/bad frames.
- `README.md` — run instructions section.

custom-gcs:
- `tools/telem_command.py` — laptop console tool (pymavlink + pyserial).
  Waits for the Pixhawk heartbeat, shows armed/mode, prompt commands
  `status | start | abort | quit`. `start` requires typing `START` to
  confirm. Resends (default 5×, 1 s) until ACK, then confirms the Pixhawk
  heartbeat shows ARMED/DISARMED within 5 s. Sends a 1 Hz GCS heartbeat.
  Args: `--port` (required, e.g. COM5), `--baud` 57600, `--target-sys` 1,
  `--target-comp` 240, `--retries`, `--ack-timeout`.
- `tools/README.md` — usage.
- `docs/COMMUNICATION.md` §6 — interface note (same two commands, new
  transport; no new topic).

## 6. How to run (bench, PROPS OFF)

1. Pixhawk (Mission Planner/QGC, on the port the radio is wired to,
   e.g. TELEM1): `SERIAL1_PROTOCOL = 2` (MAVLink2), `SERIAL1_BAUD = 57`.
   Reboot.
2. Jetson: start MAVROS as usual; then
   `ros2 param get /mavros system_id` and
   `ros2 param get /mavros component_id` (node name may differ — check
   `ros2 node list`). Confirm `/uas1/mavlink_source` exists
   (`ros2 topic list | grep mavlink`).
3. Jetson: `colcon build --packages-select nidar_autonomy`, source, then
   `ros2 run nidar_autonomy telem_command_bridge_node --ros-args -p own_component_id:=<id>`
   alongside `command_node` and `mission_state_node`.
4. Laptop: `pip install pymavlink pyserial`;
   `python custom-gcs/tools/telem_command.py --port COM5 --target-comp <id>`.
5. `start` → type `START` → expect "Jetson ACKed START" then "Pixhawk now
   ARMED". `abort` → DISARMED.

Tests:
- `cd onboard-autonomy/nidar_autonomy && PYTHONPATH=. python3 -m pytest test/ -q`
  (laptop: 326 passed, 5 skipped; on the Jetson with ROS sourced the node
  tests also run).

## 7. Known gaps / risks / open items

- **Not yet hardware-verified.** Unverified assumptions:
  - MAVROS's component id is 240 (MAV_COMP_ID_UDP_BRIDGE default).
  - MAVROS (ROS 2) exposes raw frames on `/uas1/mavlink_source` and
    accepts outgoing frames on `/uas1/mavlink_sink`.
  - ArduPilot routes the radio's COMMAND_LONG to the Ethernet link and the
    ACK back to the radio.
  If ACK fails but arming works, the routing back is the problem; if
  neither works, check component id and the `rostopic echo` of
  `/uas1/mavlink_source` for msgid 76.
- **GCS failsafe**: the laptop tool heartbeats as sysid 255. If
  `FS_GCS_ENABLE` ≠ 0, closing the tool while armed triggers the failsafe.
  Keep it open, or decide the failsafe policy deliberately.
- **ABORT = DISARM** (drops the drone in flight). Proper in-air abort
  (LAND/BRAKE, or a direct-to-Pixhawk abort that bypasses a hung Jetson)
  is still to be designed.
- **Architecture rule tension**: custom-gcs says the GCS never talks to
  the Pixhawk directly. The command is addressed to and executed by the
  Jetson (Pixhawk only relays), but the tool's heartbeat does reach the
  Pixhawk. Team should explicitly agree this is acceptable.
- **Security**: anyone on the same radio NetID with the magic number could
  send START. Use a unique radio NetID/encryption if the radio supports it.
- Not integrated into the React GCS UI — standalone CLI only.

## 8. Ideas for next steps

- Hardware bench test (§6) and record results in `CHECKPOINT/CURRENT_STATE.md`.
- Add the bridge node to whatever launch setup starts the other nodes.
- Optional: radio transport mode inside the FastAPI backend so the normal
  GCS START button can use the radio.
- Decide in-air ABORT semantics, then optionally a direct-to-Pixhawk
  emergency path.
