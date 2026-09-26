"""List Jetson serial ports for the RTK correction-source settings.

Used by the tablet to pick the LoRa radio port and the correction output port.
Stable ``/dev/serial/by-id`` names are preferred because ttyUSB/ttyACM numbers
move between boots. The flight controller is never listed: opening it beside
MAVROS would break the FCU link.
"""

from __future__ import annotations

import glob
import os
from dataclasses import dataclass
from typing import Callable, Iterable

ROLE_GNSS_RECEIVER = "gnss_receiver"
ROLE_OTHER = "other"

# Substrings of /dev/serial/by-id names.
_FLIGHT_CONTROLLER_MARKERS = ("px4", "auterion", "holybro", "pixhawk", "ardupilot")
_GNSS_RECEIVER_MARKERS = ("septentrio",)

_BARE_PATTERNS = ("/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyTHS*")


@dataclass(frozen=True, slots=True)
class SerialPortInfo:
    path: str          # what to store: by-id path when one exists
    device: str        # resolved /dev/tty* node
    label: str         # human-readable name
    role: str          # ROLE_GNSS_RECEIVER or ROLE_OTHER


def _classify(name: str) -> str | None:
    lowered = name.lower()
    if any(marker in lowered for marker in _FLIGHT_CONTROLLER_MARKERS):
        return None
    if any(marker in lowered for marker in _GNSS_RECEIVER_MARKERS):
        return ROLE_GNSS_RECEIVER
    return ROLE_OTHER


def _label_from_by_id(name: str) -> str:
    # usb-Septentrio_Septentrio_USB_Device_3804732-if04 -> readable text
    text = name
    if text.startswith("usb-"):
        text = text[4:]
    return text.replace("_", " ")


def list_serial_ports(
    *,
    by_id_dir: str = "/dev/serial/by-id",
    bare_patterns: Iterable[str] = _BARE_PATTERNS,
    glob_fn: Callable[[str], list[str]] = glob.glob,
    realpath: Callable[[str], str] = os.path.realpath,
) -> list[SerialPortInfo]:
    """Return candidate ports, by-id first, flight controller excluded."""

    ports: list[SerialPortInfo] = []
    claimed: set[str] = set()
    excluded: set[str] = set()

    for link in sorted(glob_fn(os.path.join(by_id_dir, "*"))):
        name = os.path.basename(link)
        device = realpath(link)
        role = _classify(name)
        if role is None:
            excluded.add(device)
            continue
        claimed.add(device)
        ports.append(
            SerialPortInfo(
                path=link,
                device=device,
                label=_label_from_by_id(name),
                role=role,
            )
        )

    for pattern in bare_patterns:
        for device in sorted(glob_fn(pattern)):
            device = realpath(device)
            if device in claimed or device in excluded:
                continue
            claimed.add(device)
            ports.append(
                SerialPortInfo(
                    path=device,
                    device=device,
                    label=os.path.basename(device),
                    role=ROLE_OTHER,
                )
            )

    return ports
