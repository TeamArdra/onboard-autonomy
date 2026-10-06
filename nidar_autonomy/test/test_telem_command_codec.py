"""Unit tests for telem_command_codec.py. The pymavlink cross-checks are
skipped where pymavlink isn't installed (it isn't on the Jetson); run them
on a dev machine with `pip install pymavlink`."""
import struct

import pytest

from nidar_autonomy.telem_command_codec import (
    MAV_CMD_USER_1,
    MAV_RESULT_ACCEPTED,
    NIDAR_ABORT,
    NIDAR_MAGIC,
    NIDAR_START,
    CommandLong,
    RadioCommand,
    bytes_to_payload64,
    decode_command_long,
    encode_command_ack,
    parse_radio_command,
    payload64_to_bytes,
    x25_crc,
)

OWN_SYS, OWN_COMP = 1, 240


def _cmd(param1=NIDAR_START, nonce=7.0, magic=NIDAR_MAGIC, command=MAV_CMD_USER_1,
         target_system=OWN_SYS, target_component=OWN_COMP):
    return CommandLong(
        (param1, nonce, magic, 0.0, 0.0, 0.0, 0.0), command, target_system, target_component, 0
    )


class TestParseRadioCommand:
    def test_start(self):
        assert parse_radio_command(_cmd(), OWN_SYS, OWN_COMP) == RadioCommand("start", 7)

    def test_abort(self):
        assert parse_radio_command(_cmd(param1=NIDAR_ABORT), OWN_SYS, OWN_COMP).command == "abort"

    @pytest.mark.parametrize(
        "kwargs",
        [
            {"param1": 3.0},
            {"param1": 0.0},
            {"magic": 0.0},
            {"command": 31011},
            {"target_system": 2},
            {"target_component": 1},
            {"target_component": 0},
        ],
    )
    def test_anything_else_is_ignored(self, kwargs):
        assert parse_radio_command(_cmd(**kwargs), OWN_SYS, OWN_COMP) is None


class TestPayloadRoundTrip:
    def test_payload64_round_trip(self):
        data = bytes(range(1, 34))
        assert payload64_to_bytes(bytes_to_payload64(data), len(data)) == data

    def test_decode_restores_mavlink2_truncated_zeros(self):
        full = struct.pack("<7fHBBB", NIDAR_START, 5.0, NIDAR_MAGIC, 0, 0, 0, 0,
                           MAV_CMD_USER_1, OWN_SYS, OWN_COMP, 0)
        truncated = full.rstrip(b"\x00")
        assert len(truncated) < len(full)
        cmd = decode_command_long(truncated)
        assert cmd.command == MAV_CMD_USER_1
        assert cmd.target_component == OWN_COMP
        assert cmd.confirmation == 0
        assert parse_radio_command(cmd, OWN_SYS, OWN_COMP) == RadioCommand("start", 5)


def test_x25_known_vector():
    # CRC-16/MCRF4XX check value for "123456789".
    assert x25_crc(b"123456789") == 0x6F91


class TestAgainstPymavlink:
    @pytest.fixture
    def mav(self):
        common = pytest.importorskip("pymavlink.dialects.v20.common")
        return common

    def test_decodes_a_real_pymavlink_command_long(self, mav):
        sender = mav.MAVLink(None, srcSystem=255, srcComponent=190)
        msg = mav.MAVLink_command_long_message(
            OWN_SYS, OWN_COMP, MAV_CMD_USER_1, 0, NIDAR_START, 123, NIDAR_MAGIC, 0, 0, 0, 0
        )
        frame = msg.pack(sender)
        length = frame[1]
        payload = frame[10:10 + length]
        # Same path as the node: payload64 words in, bytes out.
        words = bytes_to_payload64(payload)
        cmd = decode_command_long(payload64_to_bytes(words, length))
        assert parse_radio_command(cmd, OWN_SYS, OWN_COMP) == RadioCommand("start", 123)

    def test_ack_frame_is_byte_identical_to_pymavlink(self, mav):
        ours = encode_command_ack(
            command=MAV_CMD_USER_1, result=MAV_RESULT_ACCEPTED, target_system=255,
            target_component=190, seq=42, sysid=OWN_SYS, compid=OWN_COMP,
        )
        sender = mav.MAVLink(None, srcSystem=OWN_SYS, srcComponent=OWN_COMP)
        sender.seq = 42
        ref = mav.MAVLink_command_ack_message(
            MAV_CMD_USER_1, MAV_RESULT_ACCEPTED, 0, 0, 255, 190
        ).pack(sender)
        ref_len = ref[1]
        assert ours.len == ref_len
        assert ours.seq == ref[4]
        assert payload64_to_bytes(ours.payload64, ours.len) == ref[10:10 + ref_len]
        assert ours.checksum == struct.unpack_from("<H", ref, 10 + ref_len)[0]

    def test_pymavlink_parses_our_ack(self, mav):
        frame = encode_command_ack(
            command=MAV_CMD_USER_1, result=MAV_RESULT_ACCEPTED, target_system=255,
            target_component=190, seq=3, sysid=OWN_SYS, compid=OWN_COMP,
        )
        raw = (
            bytes([0xFD, frame.len, 0, 0, frame.seq, frame.sysid, frame.compid])
            + frame.msgid.to_bytes(3, "little")
            + payload64_to_bytes(frame.payload64, frame.len)
            + struct.pack("<H", frame.checksum)
        )
        parsed = mav.MAVLink(None).parse_char(raw)
        assert parsed.get_type() == "COMMAND_ACK"
        assert parsed.command == MAV_CMD_USER_1
        assert parsed.result == MAV_RESULT_ACCEPTED
        assert (parsed.target_system, parsed.target_component) == (255, 190)
