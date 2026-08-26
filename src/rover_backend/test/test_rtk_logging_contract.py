"""Safety contract for production RTK logging."""

from pathlib import Path


ROOT = (
    Path(__file__).resolve().parents[3]
)


def _read(
    relative_path: str,
) -> str:
    return (
        ROOT / relative_path
    ).read_text(
        encoding="utf-8"
    )


def test_operator_audit_logging_is_authenticated_and_structured():
    source = _read(
        "src/rover_backend/rover_backend/"
        "rtk_routes.py"
    )

    for action in (
        "PROFILE_CREATE",
        "PROFILE_UPDATE",
        "PROFILE_DELETE",
        "PROFILE_ACTIVATE",
        "ACTIVE_PROFILE_CLEAR",
        "START",
        "STOP",
    ):
        assert (
            f"RTK_AUDIT action={action} "
            in source
        )

    assert "_session.username" in source

    # Values from password-bearing request dictionaries must never be logged.
    assert "LOGGER.info(values" not in source
    assert "LOGGER.warning(values" not in source
    assert "LOGGER.error(values" not in source

    assert "LOGGER.info(changes" not in source
    assert "LOGGER.warning(changes" not in source
    assert "LOGGER.error(changes" not in source


def test_supervisor_logs_lifecycle_without_worker_config():
    source = _read(
        "src/rover_backend/rover_backend/"
        "rtk_runtime_orchestrator.py"
    )

    required = (
        "event=MAVROS_READY_CHANGE",
        "event=SPAWN_REQUEST",
        "event=STOP_WORKER",
        "event=CHILD_STARTED",
        "event=WORKER_READY",
        "event=CHILD_EXIT",
        "event=PROTOCOL_FAULT",
    )

    for value in required:
        assert value in source

    forbidden = (
        "LOGGER.info(config",
        "LOGGER.warning(config",
        "LOGGER.error(config",
        "LOGGER.exception(config",
        "LOGGER.info(worker_config",
        "LOGGER.warning(worker_config",
        "LOGGER.error(worker_config",
        "LOGGER.exception(worker_config",
    )

    for value in forbidden:
        assert value not in source


def test_control_logs_fail_closed_transitions():
    source = _read(
        "src/rover_backend/rover_backend/"
        "rtk_control_service.py"
    )

    required = (
        "event=START_FORWARD",
        "event=START_FAILED",
        "event=START_ACCEPTED",
        "event=STOP_FORWARD_FAILED",
        "event=RECONCILE",
        "event=FORCED_STOP",
    )

    for value in required:
        assert value in source
