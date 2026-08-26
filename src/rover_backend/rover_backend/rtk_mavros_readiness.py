"""Pure MAVROS RTCM injection-readiness evaluation."""

from __future__ import annotations

import math


def evaluate_mavros_rtcm_readiness(
    *,
    fcu_connected: bool,
    fcu_state_age_sec: float,
    rtcm_subscriber_count: int,
    stale_sec: float,
) -> bool:
    """Return whether the MAVROS RTCM injection endpoint is usable.

    Readiness requires all three independent conditions:

    * MAVROS reports an FCU connection.
    * The /mavros/state observation is fresh.
    * At least one ROS subscriber exists for the RTCM injection topic.
    """

    if not isinstance(fcu_connected, bool):
        raise TypeError(
            "fcu_connected must be a bool"
        )

    if (
        isinstance(fcu_state_age_sec, bool)
        or not isinstance(
            fcu_state_age_sec,
            (int, float),
        )
    ):
        raise TypeError(
            "fcu_state_age_sec must be a finite number >= 0"
        )

    age = float(fcu_state_age_sec)

    if not math.isfinite(age) or age < 0.0:
        raise ValueError(
            "fcu_state_age_sec must be a finite number >= 0"
        )

    if (
        isinstance(rtcm_subscriber_count, bool)
        or not isinstance(
            rtcm_subscriber_count,
            int,
        )
    ):
        raise TypeError(
            "rtcm_subscriber_count must be an int >= 0"
        )

    if rtcm_subscriber_count < 0:
        raise ValueError(
            "rtcm_subscriber_count must be an int >= 0"
        )

    if (
        isinstance(stale_sec, bool)
        or not isinstance(
            stale_sec,
            (int, float),
        )
    ):
        raise TypeError(
            "stale_sec must be a finite number > 0"
        )

    stale = float(stale_sec)

    if not math.isfinite(stale) or stale <= 0.0:
        raise ValueError(
            "stale_sec must be a finite number > 0"
        )

    return bool(
        fcu_connected
        and age <= stale
        and rtcm_subscriber_count > 0
    )
