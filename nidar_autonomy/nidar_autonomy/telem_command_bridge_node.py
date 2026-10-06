"""Bridges operator commands arriving over the telemetry radio (MicroLR900
on a Pixhawk TELEM port) onto the normal /gcs/command topic, so a mission
can be started without the Wi-Fi/rosbridge link.

This node never arms anything itself and never talks to a mavros service.
It only republishes exactly "start" or "abort" on /gcs/command, where
command_node.py validates it and mission_state_node.py decides what it
does -- the same path a rosbridge GCS command takes (Hard Safety Rules 1
and 5). Frame decoding lives in telem_command_codec.py.

Inputs:  mavros's raw received-frame stream (default /uas1/mavlink_source).
Outputs: /gcs/command (std_msgs/String), and a COMMAND_ACK back to the
         sender via mavros's raw send stream (default /uas1/mavlink_sink).

Parameters:
  own_system_id / own_component_id -- must equal mavros's own
      system_id/component_id: ArduPilot only forwards the radio's
      COMMAND_LONG to the Ethernet link because it is addressed to the
      (sysid, compid) mavros heartbeats from. Check on the Jetson with
      `ros2 param get /mavros system_id` / `component_id`.
  mavlink_source_topic / mavlink_sink_topic -- mavros's raw topics.
"""

from __future__ import annotations

import time
from collections import deque
from typing import Optional

import rclpy
from mavros_msgs.msg import Mavlink
from rclpy.node import Node
from std_msgs.msg import String

from .telem_command_codec import (
    FRAMING_OK,
    MAV_CMD_USER_1,
    MAV_RESULT_ACCEPTED,
    MAVLINK2_MAGIC,
    MSG_ID_COMMAND_LONG,
    RadioCommand,
    decode_command_long,
    encode_command_ack,
    parse_radio_command,
    payload64_to_bytes,
)
from .topics import COMMAND_TOPIC

_RECENT_NONCES = 32


class TelemCommandBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("telem_command_bridge_node")
        self._own_system = int(self.declare_parameter("own_system_id", 1).value)
        self._own_component = int(self.declare_parameter("own_component_id", 240).value)
        source_topic = self.declare_parameter(
            "mavlink_source_topic", "/uas1/mavlink_source"
        ).value
        sink_topic = self.declare_parameter("mavlink_sink_topic", "/uas1/mavlink_sink").value

        self._command_pub = self.create_publisher(String, COMMAND_TOPIC, 10)
        self._sink_pub = self.create_publisher(Mavlink, sink_topic, 10)
        self._subscription = self.create_subscription(
            Mavlink, source_topic, self._on_mavlink, 100
        )
        # (sender sysid, sender compid, nonce) already forwarded -- a resend
        # of the same nonce is re-ACKed but not republished.
        self._seen = deque(maxlen=_RECENT_NONCES)
        self._ack_seq = 0

        self.get_logger().info(
            f"Listening for radio commands on {source_topic} addressed to "
            f"{self._own_system}/{self._own_component}; forwarding to {COMMAND_TOPIC}"
        )

    def _on_mavlink(self, msg: Mavlink) -> None:
        if msg.msgid != MSG_ID_COMMAND_LONG or msg.framing_status != FRAMING_OK:
            return
        payload = payload64_to_bytes(msg.payload64, msg.len)
        radio_cmd = parse_radio_command(
            decode_command_long(payload), self._own_system, self._own_component
        )
        if radio_cmd is None:
            return
        self._handle(radio_cmd, msg.sysid, msg.compid)

    def _handle(self, radio_cmd: RadioCommand, sender_sys: int, sender_comp: int) -> None:
        key = (sender_sys, sender_comp, radio_cmd.nonce)
        stamp = time.strftime("%Y-%m-%dT%H:%M:%S")
        if key in self._seen:
            self.get_logger().info(
                f"[{stamp}] Radio resend of {radio_cmd.command!r} nonce={radio_cmd.nonce} "
                f"from {sender_sys}/{sender_comp}; re-ACKing, not republishing"
            )
        else:
            self._seen.append(key)
            self.get_logger().info(
                f"[{stamp}] Radio command {radio_cmd.command!r} nonce={radio_cmd.nonce} "
                f"from {sender_sys}/{sender_comp} -> {COMMAND_TOPIC}"
            )
            self._command_pub.publish(String(data=radio_cmd.command))
        self._send_ack(sender_sys, sender_comp)

    def _send_ack(self, target_sys: int, target_comp: int) -> None:
        frame = encode_command_ack(
            command=MAV_CMD_USER_1,
            result=MAV_RESULT_ACCEPTED,
            target_system=target_sys,
            target_component=target_comp,
            seq=self._ack_seq,
            sysid=self._own_system,
            compid=self._own_component,
        )
        self._ack_seq = (self._ack_seq + 1) & 0xFF
        self._sink_pub.publish(self._to_ros(frame))

    @staticmethod
    def _to_ros(frame) -> Mavlink:
        out = Mavlink()
        out.header.stamp = rclpy.clock.Clock().now().to_msg()
        out.framing_status = FRAMING_OK
        out.magic = MAVLINK2_MAGIC
        out.len = frame.len
        out.incompat_flags = 0
        out.compat_flags = 0
        out.seq = frame.seq
        out.sysid = frame.sysid
        out.compid = frame.compid
        out.msgid = frame.msgid
        out.checksum = frame.checksum
        out.payload64 = frame.payload64
        return out


def main(args: Optional[list] = None) -> None:
    rclpy.init(args=args)
    node = TelemCommandBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
