from rover_backend.mission_report import MissionReportStore


def base_result(accuracy):
    return {
        "point_outcome": "COMPLETED",
        "accuracy": accuracy,
        "spray": {"attempted": False, "outcome": "DISABLED"},
    }


def canonical(accuracy):
    return MissionReportStore._canonical_point(
        index=0,
        raw_status="COMPLETED",
        result=base_result(accuracy),
        active_index=None,
        target={"coordinate_mode": "LOCAL", "x_m": 1.0, "y_m": 2.0},
    )


def test_legacy_accuracy_schema_is_unchanged_when_precision_fields_absent():
    point = canonical(
        {
            "measurement_source": "RPP_TERMINAL_RESULT",
            "available": True,
            "cross_track_error_mm": 4.2,
            "along_track_error_mm": 6.1,
            "overall_accuracy_mm": 7.4,
            "tolerance_mm": 30.0,
            "within_tolerance": True,
            "rpp_outcome": "CAPTURED",
        }
    )
    assert point["accuracy"]["overall_accuracy_mm"] == 7.4
    assert point["accuracy"]["tolerance_mm"] == 30.0
    assert "precision_certificate" not in point["accuracy"]
    assert "precision_pass" not in point["accuracy"]


def test_precision_fields_and_nested_certificate_are_preserved_verbatim():
    certificate = {
        "version": 2,
        "terminal_identity": "terminal-7",
        "precision_pass": True,
        "first_capture_pose": {"x_m": 1.0, "y_m": 2.0},
    }
    evidence = {"schema_version": 2, "currently_valid": True}
    point = canonical(
        {
            "measurement_source": "RPP_TERMINAL_RESULT",
            "available": True,
            "cross_track_error_mm": 4.2,
            "along_track_error_mm": 6.1,
            "overall_accuracy_mm": 7.4,
            "tolerance_mm": 10.0,
            "within_tolerance": True,
            "rpp_outcome": "CAPTURED",
            "precision_certificate_version": 2,
            "precision_pass": True,
            "mission_run_id": "run-7",
            "goal_instance_id": "instance-7",
            "radial_error_mm": 7.4,
            "cross_error_mm": 4.2,
            "along_error_mm": 6.1,
            "measured_speed_mps": 0.004,
            "measured_yaw_rate_radps": 0.012,
            "settle_sec": 0.30,
            "precision_certificate": certificate,
            "precision_terminal_evidence": evidence,
        }
    )
    accuracy = point["accuracy"]
    assert accuracy["precision_certificate_version"] == 2
    assert accuracy["precision_pass"] is True
    assert accuracy["mission_run_id"] == "run-7"
    assert accuracy["measured_speed_mps"] == 0.004
    assert accuracy["precision_certificate"] == certificate
    assert accuracy["precision_terminal_evidence"] == evidence

    certificate["version"] = 99
    evidence["currently_valid"] = False
    assert accuracy["precision_certificate"]["version"] == 2
    assert accuracy["precision_terminal_evidence"]["currently_valid"] is True


# ---- raw-GNSS survey truth passthrough ----
#
# accuracy["survey"] is a second, independent measurement (raw RTK GNSS vs the
# surveyed coordinate). The canonical report must carry it through untouched and
# must never let it be confused with, or substituted by, the RPP numbers.

SURVEY = {
    "measurement_source": "RAW_GNSS_SURVEY",
    "truth_frame": "geodetic_raw_gnss",
    "available": True,
    "reason": None,
    "point_id": "P0001",
    "point_index": 0,
    "target_latitude_deg": 13.19088529,
    "target_longitude_deg": 80.2231721,
    "stopped_latitude_deg": 13.190885,
    "stopped_longitude_deg": 80.223172271,
    "along_track_error_mm": 12.3,
    "cross_track_error_mm": -4.5,
    "radial_error_mm": 13.1,
    "north_error_m": -0.0032,
    "east_error_m": 0.0185,
    "stop_side": "SHORT_OF_POINT",
    "cross_track_side": "LEFT",
    "approach_bearing_deg": 281.2,
    "approach_bearing_source": "SURVEYED_SEGMENT",
    "sample_count": 18,
    "sample_trimmed_count": 2,
    "sample_scatter_m": 0.0087,
    "fix_type": 6,
    "satellites": 17,
    "horizontal_accuracy_m": 0.015,
    "tolerance_m": 0.03,
    "within_tolerance": True,
    "sign_convention": {
        "along_track_error": "positive_is_short_of_target",
        "cross_track_error": "positive_is_right_of_approach",
    },
}


def test_survey_truth_is_preserved_verbatim_and_deep_copied():
    source = {
        "measurement_source": "RPP_TERMINAL_RESULT",
        "available": True,
        "cross_track_error_mm": 4.2,
        "along_track_error_mm": 6.1,
        "overall_accuracy_mm": 7.4,
        "tolerance_mm": 30.0,
        "within_tolerance": True,
        "rpp_outcome": "CAPTURED",
        "survey": SURVEY,
    }
    point = canonical(source)
    survey = point["accuracy"]["survey"]

    assert survey == SURVEY

    # Deep-copied, not aliased: mutating the report must not reach the source,
    # including inside the nested sign_convention dict.
    survey["radial_error_mm"] = 999.9
    survey["sign_convention"]["along_track_error"] = "mutated"
    assert SURVEY["radial_error_mm"] == 13.1
    assert SURVEY["sign_convention"]["along_track_error"] == (
        "positive_is_short_of_target"
    )

    # And the RPP numbers are untouched by its presence.
    assert point["accuracy"]["overall_accuracy_mm"] == 7.4
    assert point["accuracy"]["cross_track_error_mm"] == 4.2


def test_survey_truth_survives_when_rpp_accuracy_is_unavailable():
    """The passthrough must not be gated on RPP having a usable result.

    A point RPP could not measure can still have a good physical measurement.
    This is why the passthrough sits outside the source_is_rpp block.
    """
    point = canonical(
        {
            "measurement_source": "RPP_TERMINAL_RESULT",
            "available": False,
            "cross_track_error_mm": None,
            "along_track_error_mm": None,
            "overall_accuracy_mm": None,
            "survey": SURVEY,
        }
    )
    assert point["accuracy"]["available"] is False
    assert point["accuracy"]["overall_accuracy_mm"] is None
    assert point["accuracy"]["survey"]["available"] is True
    assert point["accuracy"]["survey"]["radial_error_mm"] == 13.1


def test_unavailable_survey_keeps_its_reason_and_is_not_replaced_by_rpp():
    """An unusable survey measurement must stay visibly unusable."""
    point = canonical(
        {
            "measurement_source": "RPP_TERMINAL_RESULT",
            "available": True,
            "cross_track_error_mm": 4.2,
            "along_track_error_mm": 6.1,
            "overall_accuracy_mm": 7.4,
            "tolerance_mm": 30.0,
            "within_tolerance": True,
            "rpp_outcome": "CAPTURED",
            "survey": {
                "measurement_source": "RAW_GNSS_SURVEY",
                "available": False,
                "reason": "GNSS_NOT_RTK_FIXED",
                "radial_error_mm": None,
                "along_track_error_mm": None,
                "cross_track_error_mm": None,
            },
        }
    )
    survey = point["accuracy"]["survey"]
    assert survey["available"] is False
    assert survey["reason"] == "GNSS_NOT_RTK_FIXED"
    assert survey["radial_error_mm"] is None
    # It must NOT have inherited the RPP value.
    assert survey["radial_error_mm"] != point["accuracy"]["overall_accuracy_mm"]


def test_legacy_report_without_survey_has_no_survey_key():
    """Old reports must not grow an empty survey object that looks measured."""
    point = canonical(
        {
            "measurement_source": "RPP_TERMINAL_RESULT",
            "available": True,
            "cross_track_error_mm": 4.2,
            "along_track_error_mm": 6.1,
            "overall_accuracy_mm": 7.4,
            "rpp_outcome": "CAPTURED",
        }
    )
    assert "survey" not in point["accuracy"]


def test_non_dict_survey_is_ignored():
    for bad in ("", "RAW_GNSS_SURVEY", 0, [], None):
        point = canonical(
            {
                "measurement_source": "RPP_TERMINAL_RESULT",
                "available": True,
                "cross_track_error_mm": 4.2,
                "along_track_error_mm": 6.1,
                "overall_accuracy_mm": 7.4,
                "rpp_outcome": "CAPTURED",
                "survey": bad,
            }
        )
        assert "survey" not in point["accuracy"]
