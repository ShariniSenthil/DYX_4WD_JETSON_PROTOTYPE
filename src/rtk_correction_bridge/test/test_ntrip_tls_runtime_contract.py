"""Static contract for TLS-before-auth NTRIP connection ordering."""

from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "rtk_correction_bridge"
    / "ntrip_to_px4_node.py"
)


def _connect_source():
    source = SOURCE.read_text()

    start = source.index(
        "def _connect(self):"
    )

    end = source.index(
        "def _new_parser_session(",
        start,
    )

    return source[start:end]


def test_transport_is_opened_before_basic_auth_request_is_sent():
    source = _connect_source()

    open_index = source.index(
        "open_ntrip_socket("
    )

    auth_send_index = source.index(
        "sock.sendall("
    )

    assert open_index < auth_send_index


def test_connect_failure_path_closes_socket():
    source = _connect_source()

    assert "except BaseException:" in source
    assert "sock.close()" in source


def test_no_tls_fallback_exists_in_connect():
    source = _connect_source()

    assert "socket.create_connection(" not in source
    assert "tls_mode=self.tls_mode" in source
