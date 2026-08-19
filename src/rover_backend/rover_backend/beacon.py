"""UDP discovery beacon for DYX_GCS_Frontend.

The tablet discovery screen does not poll /api/ping. It binds UDP 5002 and
expects a JSON datagram about every 2 seconds:

    {
      "type": "rover_beacon",
      "rover_id": "dyx-4wd-001",
      "rover_name": "DYX 4WD Rover",
      "ip": "192.168.3.101",
      "port": 5001,
      "version": "2.0.0",
      "uptime": 24
    }

This module is transport-only. It never starts a mission and never talks
to ROS. A send failure must not take the HTTP/Socket.IO server down.
"""

from __future__ import annotations

import ipaddress
import json
import logging
import socket
import struct
import threading
import time
from typing import Any

from rover_backend.config import settings
from rover_backend.state import rover_state

LOGGER = logging.getLogger(__name__)


def _is_usable_ipv4(value: str) -> bool:
    text = value.strip()
    if not text or text in {"0.0.0.0", "127.0.0.1", "::1"}:
        return False
    if ":" in text:
        return False
    try:
        socket.inet_aton(text)
    except OSError:
        return False
    return not text.startswith("127.")


def _detect_local_ip() -> str:
    configured = str(settings.rover_ip).strip()
    if _is_usable_ipv4(configured):
        return configured

    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.settimeout(0)
        probe.connect(("10.254.254.254", 1))
        ip = probe.getsockname()[0]
        probe.close()
        if _is_usable_ipv4(ip):
            return ip
    except OSError:
        pass

    return configured or "127.0.0.1"


def _netmask_for_ipv4(ip: str) -> str | None:
    """Read the interface netmask that owns this IPv4 address (Linux)."""

    try:
        import fcntl
    except ImportError:
        return None

    try:
        names = socket.if_nameindex()
    except OSError:
        return None

    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for _index, name in names:
            ifname = name.encode("utf-8")[:15]
            try:
                if_addr = socket.inet_ntoa(
                    fcntl.ioctl(
                        probe.fileno(),
                        0x8915,
                        struct.pack("256s", ifname),
                    )[20:24]
                )
                if_mask = socket.inet_ntoa(
                    fcntl.ioctl(
                        probe.fileno(),
                        0x891B,
                        struct.pack("256s", ifname),
                    )[20:24]
                )
            except OSError:
                continue
            if if_addr == ip:
                return if_mask
    finally:
        probe.close()

    return None


def _broadcast_targets(ip: str) -> tuple[str, ...]:
    targets = ["255.255.255.255"]

    netmask = _netmask_for_ipv4(ip)
    if netmask and _is_usable_ipv4(ip):
        try:
            network = ipaddress.IPv4Network(f"{ip}/{netmask}", strict=False)
            broadcast = str(network.broadcast_address)
            if broadcast not in targets:
                targets.append(broadcast)
            return tuple(targets)
        except ValueError:
            pass

    parts = ip.split(".")
    if len(parts) == 4 and _is_usable_ipv4(ip):
        subnet = f"{parts[0]}.{parts[1]}.{parts[2]}.255"
        if subnet not in targets:
            targets.append(subnet)
    return tuple(targets)


class RoverBeacon:
    """Broadcast the GCS discovery payload on UDP 5002."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._socket: socket.socket | None = None
        self._start_time = time.monotonic()

    @property
    def running(self) -> bool:
        thread = self._thread
        return thread is not None and thread.is_alive()

    def start(self) -> None:
        if not settings.beacon_enabled:
            LOGGER.warning("UDP discovery beacon disabled")
            return

        with self._lock:
            if self.running:
                return

            self._stop.clear()
            self._start_time = time.monotonic()
            self._thread = threading.Thread(
                target=self._loop,
                name="rover-udp-beacon",
                daemon=True,
            )
            self._thread.start()

        LOGGER.warning(
            "UDP discovery beacon: %s:%d -> UDP %d every %.1fs",
            settings.rover_ip,
            settings.backend_port,
            settings.beacon_port,
            settings.beacon_interval_sec,
        )

    def stop(self) -> None:
        self._stop.set()

        thread: threading.Thread | None
        with self._lock:
            thread = self._thread
            self._thread = None

        if thread is not None and thread.is_alive():
            thread.join(timeout=1.5)

        sock = self._socket
        self._socket = None
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

        LOGGER.info("UDP discovery beacon stopped")

    def _payload(self, ip: str) -> bytes:
        started = rover_state.section("backend").get("started_at")
        uptime = int(max(0.0, time.monotonic() - self._start_time))

        payload: dict[str, Any] = {
            "type": "rover_beacon",
            "rover_id": settings.rover_id,
            "rover_name": settings.rover_name,
            "ip": ip,
            "host": ip,
            "port": int(settings.backend_port),
            "version": settings.application_version,
            "uptime": uptime,
            "timestamp": time.time(),
        }
        if isinstance(started, str) and started:
            payload["started_at"] = started

        return json.dumps(
            payload,
            separators=(",", ":"),
            ensure_ascii=True,
        ).encode("utf-8")

    def _loop(self) -> None:
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.settimeout(1.0)
            self._socket = sock
        except OSError:
            LOGGER.exception("UDP discovery beacon socket failed")
            return

        interval = max(0.5, float(settings.beacon_interval_sec))
        port = int(settings.beacon_port)

        try:
            while not self._stop.is_set():
                try:
                    ip = _detect_local_ip()
                    message = self._payload(ip)
                    for target in _broadcast_targets(ip):
                        try:
                            sock.sendto(message, (target, port))
                        except OSError:
                            LOGGER.exception(
                                "Beacon send failed target=%s:%s",
                                target,
                                port,
                            )
                except Exception:
                    LOGGER.exception("Beacon loop iteration failed")

                self._stop.wait(interval)
        finally:
            try:
                sock.close()
            except OSError:
                pass
            if self._socket is sock:
                self._socket = None


rover_beacon = RoverBeacon()
