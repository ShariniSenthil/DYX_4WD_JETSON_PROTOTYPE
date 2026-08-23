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
