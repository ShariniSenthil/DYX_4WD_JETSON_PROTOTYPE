"""Secure socket establishment for NTRIP transport."""

from __future__ import annotations

import socket
import ssl

from typing import Callable


TLS_REQUIRED = "REQUIRED"
TLS_DISABLED = "DISABLED"


def create_verified_tls_context() -> ssl.SSLContext:
    """Create a server-authenticated TLS client context."""

    context = ssl.create_default_context()

    context.minimum_version = (
        ssl.TLSVersion.TLSv1_2
    )

    return context


def open_ntrip_socket(
    *,
    host: str,
    port: int,
    timeout_sec: float,
    tls_mode: str,
    socket_factory: Callable = (
        socket.create_connection
    ),
    tls_context_factory: Callable = (
        create_verified_tls_context
    ),
):
    """Open one NTRIP socket with explicit transport policy.

    REQUIRED performs verified TLS with hostname checking and SNI.
    DISABLED is explicit plaintext. There is never automatic fallback.
    """

    if tls_mode not in {
        TLS_REQUIRED,
        TLS_DISABLED,
    }:
        raise ValueError(
            "tls_mode must be REQUIRED or DISABLED"
        )

    raw_socket = socket_factory(
        (
            host,
            port,
        ),
        timeout=timeout_sec,
    )

    try:
        raw_socket.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_KEEPALIVE,
            1,
        )

        if tls_mode == TLS_DISABLED:
            return raw_socket

        context = tls_context_factory()

        # create_default_context() already configures CERT_REQUIRED and
        # hostname verification. Assert the invariant instead of silently
        # accepting a weakened injected/test context.
        if (
            context.verify_mode
            != ssl.CERT_REQUIRED
        ):
            raise RuntimeError(
                "TLS context must require "
                "server certificate verification"
            )

        if not context.check_hostname:
            raise RuntimeError(
                "TLS context must enable "
                "hostname verification"
            )

        secure_socket = context.wrap_socket(
            raw_socket,
            server_hostname=host,
        )

        # Ownership is transferred to SSLSocket.
        raw_socket = None

        return secure_socket

    except BaseException:
        if raw_socket is not None:
            try:
                raw_socket.close()
            except OSError:
                pass

        raise
