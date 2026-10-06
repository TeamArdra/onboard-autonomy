"""Pure MAVLink encode/decode for the telemetry-radio command link, kept
free of rclpy (and of pymavlink, which isn't installed on the Jetson) so
it's unit-testable -- same split as state_machine.py vs.
mission_state_node.py.

The link: laptop --MicroLR900 radio--> Pixhawk TELEM port --(ArduPilot
MAVLink routing, Ethernet)--> mavros --/uas1/mavlink_source-->
telem_command_bridge_node.py --"/gcs/command"--> command_node.py.

The laptop sends a COMMAND_LONG(MAV_CMD_USER_1) addressed to mavros's own
(sysid, compid), so ArduPilot forwards it to the Ethernet link instead of
executing it. Layout of the request:

    param1  NIDAR_START (1.0) or NIDAR_ABORT (2.0)
    param2  nonce -- the laptop resends the same nonce until it gets an
            ACK; the bridge forwards each nonce to /gcs/command only once
    param3  NIDAR_MAGIC -- guards against a stray/foreign MAV_CMD_USER_1
            being read as a mission command

This module does not decide what start/abort *do*; it only maps a frame
to exactly "start"/"abort" or None. command_node.py still validates the
string, per Hard Safety Rule 5.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from typing import Optional, Sequence

MSG_ID_COMMAND_LONG = 76
MSG_ID_COMMAND_ACK = 77
CRC_EXTRA_COMMAND_LONG = 152
CRC_EXTRA_COMMAND_ACK = 143

MAV_CMD_USER_1 = 31010
MAV_RESULT_ACCEPTED = 0
MAV_RESULT_DENIED = 2

MAVLINK2_MAGIC = 0xFD
FRAMING_OK = 1

NIDAR_START = 1.0
NIDAR_ABORT = 2.0
NIDAR_MAGIC = 4242.0

_PARAM_TO_COMMAND = {NIDAR_START: "start", NIDAR_ABORT: "abort"}

_COMMAND_LONG_FMT = "<7fHBBB"  # 33 bytes, MAVLink wire order
_COMMAND_LONG_LEN = struct.calcsize(_COMMAND_LONG_FMT)
_COMMAND_ACK_FMT = "<HBBiBB"  # command, result, progress, result_param2, target_sys, target_comp


@dataclass(frozen=True)
class CommandLong:
    params: tuple
    command: int
    target_system: int
    target_component: int
    confirmation: int


@dataclass(frozen=True)
class RadioCommand:
    command: str  # exactly "start" or "abort"
    nonce: int


def payload64_to_bytes(payload64: Sequence[int], length: int) -> bytes:
    raw = b"".join(struct.pack("<Q", int(word) & 0xFFFFFFFFFFFFFFFF) for word in payload64)
    return raw[:length]


def bytes_to_payload64(payload: bytes) -> list:
    padded = payload + b"\x00" * (-len(payload) % 8)
    return [struct.unpack_from("<Q", padded, i)[0] for i in range(0, len(padded), 8)]


def decode_command_long(payload: bytes) -> CommandLong:
    # MAVLink2 truncates trailing zero bytes -- restore them before unpacking.
    padded = payload[:_COMMAND_LONG_LEN].ljust(_COMMAND_LONG_LEN, b"\x00")
    *params, command, target_system, target_component, confirmation = struct.unpack(
        _COMMAND_LONG_FMT, padded
    )
    return CommandLong(tuple(params), command, target_system, target_component, confirmation)


def parse_radio_command(
    cmd: CommandLong, own_system: int, own_component: int
) -> Optional[RadioCommand]:
    """Map a decoded COMMAND_LONG to a RadioCommand, or None if it isn't a
    NIDAR command addressed to us."""
    if cmd.command != MAV_CMD_USER_1:
        return None
    if cmd.target_system != own_system or cmd.target_component != own_component:
        return None
    if cmd.params[2] != NIDAR_MAGIC:
        return None
    command = _PARAM_TO_COMMAND.get(cmd.params[0])
    if command is None:
        return None
    return RadioCommand(command=command, nonce=int(cmd.params[1]))


def x25_crc(data: bytes, crc: int = 0xFFFF) -> int:
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


@dataclass(frozen=True)
class Mavlink2Frame:
    """The fields mavros_msgs/Mavlink needs to send a finished MAVLink2
    frame. mavros's router forwards a frame from mavlink_sink as-is, so
    the checksum has to be computed here."""

    len: int
    seq: int
    sysid: int
    compid: int
    msgid: int
    checksum: int
    payload64: list


def encode_command_ack(
    command: int,
    result: int,
    target_system: int,
    target_component: int,
    seq: int,
    sysid: int,
    compid: int,
) -> Mavlink2Frame:
    payload = struct.pack(
        _COMMAND_ACK_FMT, command, result, 0, 0, target_system, target_component
    )
    payload = payload.rstrip(b"\x00") or b"\x00"  # MAVLink2 zero-truncation, min 1 byte
    seq &= 0xFF
    header = struct.pack(
        "<BBBBBB3s",
        len(payload),
        0,  # incompat_flags
        0,  # compat_flags
        seq,
        sysid,
        compid,
        MSG_ID_COMMAND_ACK.to_bytes(3, "little"),
    )
    crc = x25_crc(header + payload)
    crc = x25_crc(bytes([CRC_EXTRA_COMMAND_ACK]), crc)
    return Mavlink2Frame(
        len=len(payload),
        seq=seq,
        sysid=sysid,
        compid=compid,
        msgid=MSG_ID_COMMAND_ACK,
        checksum=crc,
        payload64=bytes_to_payload64(payload),
    )
