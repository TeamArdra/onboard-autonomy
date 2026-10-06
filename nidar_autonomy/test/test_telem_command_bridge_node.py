"""Integration tests for telem_command_bridge_node.py -- requires a real
rclpy context, gated with `pytest.importorskip("rclpy")` (see
test_frontier_explorer_node.py's module docstring for the pattern/rationale)."""
import struct

import pytest

pytest.importorskip("rclpy")

import rclpy  # noqa: E402
from mavros_msgs.msg import Mavlink  # noqa: E402

from nidar_autonomy.telem_command_codec import (  # noqa: E402
    MAV_CMD_USER_1,
    MSG_ID_COMMAND_ACK,
    MSG_ID_COMMAND_LONG,
    NIDAR_ABORT,
    NIDAR_MAGIC,
    NIDAR_START,
    bytes_to_payload64,
)


@pytest.fixture(autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def _make_node():
    from nidar_autonomy.telem_command_bridge_node import TelemCommandBridgeNode

    node = TelemCommandBridgeNode()
    commands, acks = [], []
    node._command_pub.publish = lambda msg: commands.append(msg.data)
    node._sink_pub.publish = lambda msg: acks.append(msg)
    return node, commands, acks


def _frame(param1=NIDAR_START, nonce=1, target_component=240, msgid=MSG_ID_COMMAND_LONG):
    payload = struct.pack(
        "<7fHBBB", param1, float(nonce), NIDAR_MAGIC, 0, 0, 0, 0,
        MAV_CMD_USER_1, 1, target_component, 0,
    ).rstrip(b"\x00")
    msg = Mavlink()
    msg.framing_status = Mavlink.FRAMING_OK
    msg.sysid, msg.compid, msg.msgid = 255, 190, msgid
    msg.len = len(payload)
    msg.payload64 = bytes_to_payload64(payload)
    return msg


def test_start_is_forwarded_and_acked():
    node, commands, acks = _make_node()
    try:
        node._on_mavlink(_frame())
        assert commands == ["start"]
        assert len(acks) == 1
        assert acks[0].msgid == MSG_ID_COMMAND_ACK
        assert (acks[0].sysid, acks[0].compid) == (1, 240)
    finally:
        node.destroy_node()


def test_abort_is_forwarded():
    node, commands, _ = _make_node()
    try:
        node._on_mavlink(_frame(param1=NIDAR_ABORT))
        assert commands == ["abort"]
    finally:
        node.destroy_node()


def test_resend_of_same_nonce_is_reacked_not_republished():
    node, commands, acks = _make_node()
    try:
        node._on_mavlink(_frame(nonce=9))
        node._on_mavlink(_frame(nonce=9))
        node._on_mavlink(_frame(nonce=10))
        assert commands == ["start", "start"]
        assert len(acks) == 3
    finally:
        node.destroy_node()


def test_unrelated_frames_are_ignored():
    node, commands, acks = _make_node()
    try:
        node._on_mavlink(_frame(target_component=1))
        node._on_mavlink(_frame(param1=5.0))
        node._on_mavlink(_frame(msgid=0))
        bad = _frame()
        bad.framing_status = Mavlink.FRAMING_BAD_CRC
        node._on_mavlink(bad)
        assert commands == [] and acks == []
    finally:
        node.destroy_node()
