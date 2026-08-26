"""Tests for pure NMEA-GGA generation."""

from rtk_correction_bridge.nmea_gga import (
    GgaSourceFix,
    format_gga_sentence,
    gga_quality_from_mavlink_fix_type,
)


def _checksum(payload):
    value = 0

    for character in payload:
        value ^= ord(character)

    return value


def test_quality_mapping():
    assert gga_quality_from_mavlink_fix_type(3) == 1
    assert gga_quality_from_mavlink_fix_type(4) == 2
    assert gga_quality_from_mavlink_fix_type(5) == 5
    assert gga_quality_from_mavlink_fix_type(6) == 4


def test_coordinate_and_fix_fields():
    sentence = format_gga_sentence(
        GgaSourceFix(
            latitude_deg=48.1173,
            longitude_deg=11.5166666667,
            altitude_msl_m=545.4,
            mavlink_fix_type=3,
            satellites_visible=8,
            hdop=0.9,
            utc_epoch_sec=764426119.0,
        )
    ).decode("ascii")

    assert ",4807.03800,N," in sentence
    assert ",01131.00000,E," in sentence
    assert ",1,08,0.9,545.400,M,,M,," in sentence
    assert sentence.endswith("\r\n")


def test_checksum():
    sentence = format_gga_sentence(
        GgaSourceFix(
            latitude_deg=13.0827,
            longitude_deg=80.2707,
            altitude_msl_m=12.5,
            mavlink_fix_type=6,
            satellites_visible=21,
            hdop=0.7,
            utc_epoch_sec=1700000000.25,
        )
    ).decode("ascii").strip()

    payload, supplied = (
        sentence[1:].split("*")
    )

    assert (
        int(supplied, 16)
        == _checksum(payload)
    )
