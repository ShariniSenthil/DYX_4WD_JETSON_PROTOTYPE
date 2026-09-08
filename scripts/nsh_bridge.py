#!/usr/bin/env python3
"""Permanent, reusable PX4 NSH-over-SSH bridge.

Runs on the Jetson. Transports NSH shell commands to PX4 through the
existing MAVROS connection via MAVLink SERIAL_CONTROL, exactly as verified
and documented in docs/PX4_NSH_OVER_SSH.md (2026-09-07). Does not open a
second serial connection to the FCU and does not require arming for
read-only commands (ver, listener, param show); by default it refuses to
run ANY command while armed, matching every field script that has used
this transport so far.

Dependencies (pymavlink, fastcrc, lxml) are expected at a permanent,
non-/tmp path so they survive a Jetson reboot -- install once with:

    python3 -m pip install --target ~/.local/share/nsh_bridge/deps pymavlink

Override the path with the NSH_BRIDGE_DEPS environment variable if it was
installed somewhere else.

Usage (from the Jetson, with the rover stack already running so MAVROS
owns /dev/ttyACM0):

    source /opt/ros/humble/setup.bash
    python3 scripts/nsh_bridge.py "param show EKF2_GPS_CHECK"
    python3 scripts/nsh_bridge.py "param set EKF2_GPS_CHECK 829" "param save" "param show EKF2_GPS_CHECK"
    python3 scripts/nsh_bridge.py --allow-armed "listener vehicle_status -n 1"

Each argument is one NSH command, run in sequence, waited for completion
(echoed command + a following nsh> prompt) before the next is sent.
"""
import argparse
import os
import re
import sys
import threading
import time

os.environ.setdefault("MAVLINK20", "1")

_DEFAULT_DEPS = os.path.expanduser("~/.local/share/nsh_bridge/deps")
_deps = os.environ.get("NSH_BRIDGE_DEPS", _DEFAULT_DEPS)
if os.path.isdir(_deps) and _deps not in sys.path:
    sys.path.insert(0, _deps)

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from rclpy.signals import SignalHandlerOptions  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from mavros_msgs.msg import Mavlink, State  # noqa: E402
from mavros.mavlink import convert_to_rosmsg, convert_to_bytes  # noqa: E402
from pymavlink.dialects.v20 import common as mav  # noqa: E402


class NshBridge(Node):
    """One NSH shell session over the verified SERIAL_CONTROL transport."""

    def __init__(self):
        super().__init__("nsh_bridge")
        self.encoder = mav.MAVLink(None, srcSystem=1, srcComponent=191)
        self.decoder = mav.MAVLink(None)
        self.shell_text = ""
        self.state = None
        self.rawpub = self.create_publisher(Mavlink, "/uas1/mavlink_sink", 10)
        q = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Mavlink, "/uas1/mavlink_source", self._raw_rx, q)
        self.create_subscription(
            State, "/mavros/state", self._on_state,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT),
        )

    def _on_state(self, msg):
        self.state = msg

    def _raw_rx(self, msg):
        if msg.msgid != 126:
            return
        try:
            packets = self.decoder.parse_buffer(convert_to_bytes(msg)) or []
        except Exception:
            return
        for packet in packets:
            if packet.get_type() != "SERIAL_CONTROL":
                continue
            self.shell_text += bytes(packet.data[: packet.count]).decode(errors="replace")

    def send_shell(self, text, flags=mav.SERIAL_CONTROL_FLAG_RESPOND):
        raw = text.encode()
        chunks = [raw[i : i + 70] for i in range(0, len(raw), 70)] or [b""]
        for chunk in chunks:
            packet = self.encoder.serial_control_encode(
                mav.SERIAL_CONTROL_DEV_SHELL, flags, 0, 0, len(chunk),
                list(chunk) + [0] * (70 - len(chunk)),
            )
            packet.pack(self.encoder)
            self.encoder.seq = (self.encoder.seq + 1) % 256
            self.rawpub.publish(convert_to_rosmsg(packet))

    def shell(self, command, timeout=12, retries=3):
        for attempt in range(retries):
            try:
                return self._shell_once(command, timeout)
            except RuntimeError:
                if attempt == retries - 1:
                    raise
                time.sleep(1)

    def _shell_once(self, command, timeout):
        offset = len(self.shell_text)
        self.send_shell(command + "\n")
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            result = self.shell_text[offset:]
            idx = result.find(command)
            if idx != -1 and "nsh>" in result[idx + len(command) :]:
                return re.sub(r"\x1b\[[0-9;]*[A-Za-z]", "", result)
            time.sleep(0.02)
        raise RuntimeError(f"timeout waiting for reply to: {command}")

    def close(self):
        self.send_shell("", flags=0)
        time.sleep(0.3)


def run(commands, allow_armed=False, connect_timeout=8.0):
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = NshBridge()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        end = time.monotonic() + connect_timeout
        while (node.state is None or node.rawpub.get_subscription_count() == 0) and time.monotonic() < end:
            time.sleep(0.1)
        if node.state is None:
            print("ABORT: no /mavros/state received -- is the rover stack up?")
            return 1
        if node.state.armed and not allow_armed:
            print(f"ABORT: vehicle is armed (mode={node.state.mode}); refusing to run NSH commands. "
                  f"Pass --allow-armed to override for read-only commands.")
            return 1
        print(f"connected: armed={node.state.armed} mode={node.state.mode}")
        for cmd in commands:
            print(f"\n--- {cmd} ---")
            print(node.shell(cmd))
        return 0
    finally:
        node.close()
        executor.shutdown()
        thread.join(timeout=3)
        node.destroy_node()
        rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("commands", nargs="+", help="One or more NSH commands, run in sequence")
    ap.add_argument("--allow-armed", action="store_true",
                     help="Do not abort if the vehicle is armed (read-only commands only -- "
                          "this tool never arms/disarms/moves anything itself)")
    ap.add_argument("--connect-timeout", type=float, default=8.0)
    args = ap.parse_args()
    sys.exit(run(args.commands, allow_armed=args.allow_armed, connect_timeout=args.connect_timeout))


if __name__ == "__main__":
    main()
