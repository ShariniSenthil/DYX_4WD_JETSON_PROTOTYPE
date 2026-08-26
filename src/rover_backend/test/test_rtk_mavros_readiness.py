"""Tests for the pure MAVROS RTCM readiness gate."""

import math

import pytest

from rover_backend.rtk_mavros_readiness import (
    evaluate_mavros_rtcm_readiness,
)


def ready(**overrides):
    values = {
        "fcu_connected": True,
        "fcu_state_age_sec": 0.1,
        "rtcm_subscriber_count": 1,
        "stale_sec": 3.0,
    }
    values.update(overrides)

    return evaluate_mavros_rtcm_readiness(
        **values
    )


def test_all_conditions_true_is_ready():
    assert ready() is True


def test_disconnected_fcu_is_not_ready():
    assert ready(
        fcu_connected=False
    ) is False


def test_stale_fcu_state_is_not_ready():
    assert ready(
        fcu_state_age_sec=3.001
    ) is False


def test_exact_stale_boundary_is_ready():
    assert ready(
        fcu_state_age_sec=3.0
    ) is True


def test_no_rtcm_subscriber_is_not_ready():
    assert ready(
        rtcm_subscriber_count=0
    ) is False


def test_multiple_rtcm_subscribers_still_ready():
    assert ready(
        rtcm_subscriber_count=2
    ) is True


@pytest.mark.parametrize(
    "age",
    [
        -1.0,
        math.inf,
        math.nan,
    ],
)
def test_invalid_age_rejected(age):
    with pytest.raises(ValueError):
        ready(
            fcu_state_age_sec=age
        )


def test_bool_subscriber_count_rejected():
    with pytest.raises(TypeError):
        ready(
            rtcm_subscriber_count=True
        )


def test_invalid_stale_threshold_rejected():
    with pytest.raises(ValueError):
        ready(
            stale_sec=0.0
        )
