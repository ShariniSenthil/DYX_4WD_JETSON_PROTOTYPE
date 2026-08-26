"""Pure NMEA-GGA generation for NTRIP VRS operation."""

from __future__ import annotations

import math

from dataclasses import dataclass
from datetime import datetime
from datetime import timezone


@dataclass(frozen=True, slots=True)
class GgaSourceFix:
    """Fresh GNSS information required for one GGA sentence."""

    latitude_deg: float
    longitude_deg: float
    altitude_msl_m: float
    mavlink_fix_type: int
    satellites_visible: int
    hdop: float
    utc_epoch_sec: float


def gga_quality_from_mavlink_fix_type(
    fix_type: int,
) -> int:
    """Map MAVLink GPS fix type to NMEA GGA quality."""

    if fix_type in (2, 3):
        return 1

    if fix_type == 4:
        return 2

    if fix_type == 5:
        return 5

    if fix_type in (6, 7):
        return 4

    if fix_type == 8:
        return 1

    return 0


def _coordinate(
    value: float,
    *,
    latitude: bool,
) -> tuple[str, str]:
    if not math.isfinite(value):
        raise ValueError(
            "coordinate must be finite"
        )

    limit = 90.0 if latitude else 180.0

    if not -limit <= value <= limit:
        raise ValueError(
            "coordinate outside valid range"
        )

    absolute = abs(value)
    degrees = int(absolute)

    minutes = round(
        (
            absolute - degrees
        )
        * 60.0,
        5,
    )

    if minutes >= 60.0:
        degrees += 1
        minutes = 0.0

    if latitude:
        field = (
            f"{degrees:02d}"
            f"{minutes:08.5f}"
        )
        hemisphere = (
            "N"
            if value >= 0.0
            else "S"
        )
    else:
        field = (
            f"{degrees:03d}"
            f"{minutes:08.5f}"
        )
        hemisphere = (
            "E"
            if value >= 0.0
            else "W"
        )

    return field, hemisphere


def _checksum(
    payload: str,
) -> int:
    value = 0

    for character in payload:
        value ^= ord(character)

    return value


def format_gga_sentence(
    fix: GgaSourceFix,
) -> bytes:
    """Encode one standards-shaped ASCII GPGGA sentence."""

    if not isinstance(
        fix,
        GgaSourceFix,
    ):
        raise TypeError(
            "fix must be GgaSourceFix"
        )

    if not math.isfinite(
        fix.altitude_msl_m
    ):
        raise ValueError(
            "altitude_msl_m must be finite"
        )

    if not math.isfinite(
        fix.utc_epoch_sec
    ):
        raise ValueError(
            "utc_epoch_sec must be finite"
        )

    latitude, ns = _coordinate(
        fix.latitude_deg,
        latitude=True,
    )

    longitude, ew = _coordinate(
        fix.longitude_deg,
        latitude=False,
    )

    quality = (
        gga_quality_from_mavlink_fix_type(
            int(fix.mavlink_fix_type)
        )
    )

    satellites = max(
        0,
        min(
            99,
            int(
                fix.satellites_visible
            ),
        ),
    )

    hdop = float(fix.hdop)

    if (
        not math.isfinite(hdop)
        or hdop <= 0.0
    ):
        hdop = 99.9

    stamp = datetime.fromtimestamp(
        fix.utc_epoch_sec,
        tz=timezone.utc,
    )

    utc_field = (
        f"{stamp.hour:02d}"
        f"{stamp.minute:02d}"
        f"{stamp.second:02d}."
        f"{stamp.microsecond // 10000:02d}"
    )

    # GPSRAW.alt is MAVLink GPS_RAW_INT altitude above MSL.
    # Geoid separation is left blank rather than invented.
    payload = (
        "GPGGA,"
        f"{utc_field},"
        f"{latitude},{ns},"
        f"{longitude},{ew},"
        f"{quality},"
        f"{satellites:02d},"
        f"{hdop:.1f},"
        f"{fix.altitude_msl_m:.3f},M,"
        ",M,,"
    )

    checksum = _checksum(
        payload
    )

    return (
        f"${payload}*{checksum:02X}\r\n"
    ).encode("ascii")
