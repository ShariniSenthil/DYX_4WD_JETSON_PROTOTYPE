"""Behavior tests for secure NTRIP socket establishment."""

import socket
import ssl

import pytest

from rtk_correction_bridge.ntrip_socket import (
    TLS_DISABLED,
    TLS_REQUIRED,
    create_verified_tls_context,
    open_ntrip_socket,
)


class FakeSocket:
    def __init__(self):
        self.closed = False
        self.options = []

    def setsockopt(
        self,
        level,
        option,
        value,
    ):
        self.options.append(
            (
                level,
                option,
                value,
            )
        )

    def close(self):
        self.closed = True


class FakeSecureSocket:
    pass


class FakeContext:
    def __init__(
        self,
        *,
        verify_mode=ssl.CERT_REQUIRED,
        check_hostname=True,
        fail=False,
    ):
        self.verify_mode = verify_mode
        self.check_hostname = check_hostname
        self.fail = fail
        self.calls = []
        self.secure = FakeSecureSocket()

    def wrap_socket(
        self,
        raw_socket,
        *,
        server_hostname,
    ):
        self.calls.append(
            (
                raw_socket,
                server_hostname,
            )
        )

        if self.fail:
            raise ssl.SSLError(
                "synthetic TLS failure"
            )

        return self.secure


def test_default_context_is_verified_and_tls12_or_newer():
    context = (
        create_verified_tls_context()
    )

    assert (
        context.verify_mode
        == ssl.CERT_REQUIRED
    )

    assert context.check_hostname is True

    assert (
        context.minimum_version
        >= ssl.TLSVersion.TLSv1_2
    )


def test_required_wraps_with_exact_hostname_for_sni_and_verification():
    raw = FakeSocket()
    context = FakeContext()

    def socket_factory(
        address,
        *,
        timeout,
    ):
        assert address == (
            "caster.example",
            443,
        )
        assert timeout == 7.0
        return raw

    result = open_ntrip_socket(
        host="caster.example",
        port=443,
        timeout_sec=7.0,
        tls_mode=TLS_REQUIRED,
        socket_factory=socket_factory,
        tls_context_factory=lambda: context,
    )

    assert result is context.secure

    assert context.calls == [
        (
            raw,
            "caster.example",
        )
    ]

    assert raw.closed is False


def test_plaintext_requires_explicit_disabled_and_does_not_create_tls():
    raw = FakeSocket()

    def socket_factory(
        address,
        *,
        timeout,
    ):
        return raw

    def forbidden_context():
        raise AssertionError(
            "TLS context must not be built "
            "for explicit DISABLED mode"
        )

    result = open_ntrip_socket(
        host="legacy.example",
        port=2101,
        timeout_sec=5.0,
        tls_mode=TLS_DISABLED,
        socket_factory=socket_factory,
        tls_context_factory=forbidden_context,
    )

    assert result is raw
    assert raw.closed is False


def test_tls_failure_closes_raw_socket_and_never_falls_back():
    raw = FakeSocket()

    context = FakeContext(
        fail=True
    )

    def socket_factory(
        address,
        *,
        timeout,
    ):
        return raw

    with pytest.raises(
        ssl.SSLError,
        match="synthetic TLS failure",
    ):
        open_ntrip_socket(
            host="caster.example",
            port=443,
            timeout_sec=5.0,
            tls_mode=TLS_REQUIRED,
            socket_factory=socket_factory,
            tls_context_factory=lambda: context,
        )

    assert raw.closed is True


@pytest.mark.parametrize(
    (
        "verify_mode",
        "check_hostname",
    ),
    (
        (
            ssl.CERT_NONE,
            False,
        ),
        (
            ssl.CERT_REQUIRED,
            False,
        ),
    ),
)
def test_weakened_tls_context_is_rejected(
    verify_mode,
    check_hostname,
):
    raw = FakeSocket()

    context = FakeContext(
        verify_mode=verify_mode,
        check_hostname=check_hostname,
    )

    with pytest.raises(
        RuntimeError,
        match="TLS context",
    ):
        open_ntrip_socket(
            host="caster.example",
            port=443,
            timeout_sec=5.0,
            tls_mode=TLS_REQUIRED,
            socket_factory=(
                lambda *args, **kwargs: raw
            ),
            tls_context_factory=lambda: context,
        )

    assert raw.closed is True


def test_unknown_mode_is_rejected_before_network_open():
    opened = False

    def socket_factory(
        *args,
        **kwargs,
    ):
        nonlocal opened
        opened = True
        return FakeSocket()

    with pytest.raises(
        ValueError,
        match="tls_mode",
    ):
        open_ntrip_socket(
            host="caster.example",
            port=443,
            timeout_sec=5.0,
            tls_mode="PREFERRED",
            socket_factory=socket_factory,
        )

    assert opened is False
