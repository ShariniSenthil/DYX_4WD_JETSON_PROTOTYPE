"""Static safety contract for managed VRS GGA."""

from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def _source():
    return (
        ROOT
        / "rtk_correction_bridge"
        / "ntrip_to_px4_node.py"
    ).read_text()


def test_uses_existing_mavros_sources():
    source = _source()

    assert (
        "/mavros/global_position/raw/fix"
        in source
    )

    assert (
        "/mavros/gpsstatus/gps1/raw"
        in source
    )


def test_same_authenticated_socket_sends_gga():
    source = _source()

    assert "def _maybe_send_gga(" in source
    assert "sock.sendall(" in source
    assert "format_gga_sentence(" in source


def test_disabled_path_cannot_write_gga():
    source = _source()

    start = source.index(
        "def _maybe_send_gga("
    )

    end = source.index(
        "def _gga_status("
    )

    method = source[start:end]

    assert (
        "if not self.gga_enabled:"
        in method
    )

    assert "return False" in method
