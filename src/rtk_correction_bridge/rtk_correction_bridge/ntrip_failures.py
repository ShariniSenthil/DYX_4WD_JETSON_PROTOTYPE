"""Semantic NTRIP failures shared by the worker runtime and bootstrap."""

from __future__ import annotations


class NtripAuthError(ConnectionError):
    """Caster permanently rejected the supplied NTRIP credentials."""


class NtripMountpointRejectedError(ConnectionError):
    """Caster permanently rejected or did not provide the mountpoint."""


def validate_ntrip_status_line(status_line: bytes) -> None:
    """Accept NTRIP success or raise a semantic failure.

    Permanent configuration failures are distinguished from retryable
    connection/server failures so the backend supervisor does not create
    restart storms for bad credentials or an invalid mountpoint.
    """

    if not isinstance(
        status_line,
        (bytes, bytearray, memoryview),
    ):
        raise TypeError(
            "status_line must be bytes-like"
        )

    line = bytes(status_line).strip()

    if not line:
        raise ConnectionError(
            "NTRIP response status line is empty"
        )

    if line.upper().startswith(
        b"SOURCETABLE"
    ):
        raise NtripMountpointRejectedError(
            "NTRIP caster returned source table"
        )

    parts = line.split()

    if len(parts) < 2:
        raise ConnectionError(
            "malformed NTRIP response status line"
        )

    status = parts[1]

    if status == b"200":
        return

    if status in (
        b"401",
        b"403",
    ):
        raise NtripAuthError(
            "NTRIP authentication rejected"
        )

    if status == b"404":
        raise NtripMountpointRejectedError(
            "NTRIP mountpoint rejected"
        )

    status_text = status.decode(
        "ascii",
        errors="replace",
    )

    raise ConnectionError(
        "NTRIP caster rejected connection "
        "with status %s" % status_text
    )
