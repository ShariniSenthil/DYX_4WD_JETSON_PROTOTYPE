"""Tests for semantic NTRIP response classification."""

import pytest

from rtk_correction_bridge.ntrip_failures import (
    NtripAuthError,
    NtripMountpointRejectedError,
    validate_ntrip_status_line,
)


def test_http_200_is_accepted():
    validate_ntrip_status_line(
        b"HTTP/1.1 200 OK"
    )


def test_icy_200_is_accepted():
    validate_ntrip_status_line(
        b"ICY 200 OK"
    )


@pytest.mark.parametrize(
    "status",
    [
        b"HTTP/1.1 401 Unauthorized",
        b"HTTP/1.1 403 Forbidden",
    ],
)
def test_auth_rejection_is_terminal(status):
    with pytest.raises(NtripAuthError):
        validate_ntrip_status_line(status)


def test_404_is_mountpoint_rejected():
    with pytest.raises(
        NtripMountpointRejectedError
    ):
        validate_ntrip_status_line(
            b"HTTP/1.1 404 Not Found"
        )


def test_sourcetable_is_mountpoint_rejected():
    with pytest.raises(
        NtripMountpointRejectedError
    ):
        validate_ntrip_status_line(
            b"SOURCETABLE 200 OK"
        )


def test_500_remains_retryable_connection_failure():
    with pytest.raises(
        ConnectionError
    ) as captured:
        validate_ntrip_status_line(
            b"HTTP/1.1 500 Internal Server Error"
        )

    assert not isinstance(
        captured.value,
        (
            NtripAuthError,
            NtripMountpointRejectedError,
        ),
    )


def test_malformed_status_remains_retryable():
    with pytest.raises(
        ConnectionError
    ) as captured:
        validate_ntrip_status_line(
            b"BROKEN"
        )

    assert not isinstance(
        captured.value,
        (
            NtripAuthError,
            NtripMountpointRejectedError,
        ),
    )
