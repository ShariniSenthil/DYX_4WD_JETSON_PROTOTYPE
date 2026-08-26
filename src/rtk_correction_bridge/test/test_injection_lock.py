"""Tests for standalone RTCM injection ownership."""

import pytest

from rtk_correction_bridge.injection_lock import (
    DEFAULT_INJECTION_LOCK_PATH,
    InjectionOwnershipConflictError,
    InjectionOwnershipLock,
)


def test_production_lock_path_matches_backend_contract():
    assert (
        DEFAULT_INJECTION_LOCK_PATH
        == "/run/lock/rover-rtk-injection.lock"
    )


def test_second_owner_is_rejected(tmp_path):
    path = tmp_path / "injection.lock"

    first = InjectionOwnershipLock(path)
    second = InjectionOwnershipLock(path)

    first.acquire_nonblocking()

    try:
        with pytest.raises(
            InjectionOwnershipConflictError
        ):
            second.acquire_nonblocking()
    finally:
        first.close()
        second.close()


def test_lock_can_be_reacquired_after_release(tmp_path):
    path = tmp_path / "injection.lock"

    first = InjectionOwnershipLock(path)
    second = InjectionOwnershipLock(path)

    first.acquire_nonblocking()
    first.close()

    try:
        second.acquire_nonblocking()
        assert second.locked is True
    finally:
        second.close()


def test_standalone_ntrip_entrypoint_fails_closed():
    """Standalone execution must never become a second RTCM owner."""

    from pathlib import Path
    import ast

    source_path = (
        Path(__file__).resolve().parents[1]
        / "rtk_correction_bridge"
        / "ntrip_to_px4_node.py"
    )

    source = source_path.read_text()
    tree = ast.parse(source)

    main_function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef)
        and node.name == "main"
    )

    calls = [
        node
        for node in ast.walk(main_function)
        if isinstance(node, ast.Call)
    ]

    # The standalone path must not acquire correction ownership.
    assert not any(
        isinstance(call.func, ast.Name)
        and call.func.id == "InjectionOwnershipLock"
        for call in calls
    )

    assert not any(
        isinstance(call.func, ast.Name)
        and call.func.id == "NtripToPx4Node"
        for call in calls
    )

    # It must explicitly fail closed.
    returns = [
        node
        for node in ast.walk(main_function)
        if isinstance(node, ast.Return)
    ]

    assert any(
        isinstance(node.value, ast.Constant)
        and node.value.value == 2
        for node in returns
    )

    assert (
        "Standalone RTK launch is disabled"
        in source
    )
