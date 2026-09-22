"""Behavior of the event-driven command/status synchronization."""

import importlib.util
import sys
import threading
import time
from pathlib import Path

MODULE_PATH = (
    Path(__file__).resolve().parents[1] / "rover_backend" / "manager_status_sync.py"
)
_spec = importlib.util.spec_from_file_location("manager_status_sync", MODULE_PATH)
manager_status_sync = importlib.util.module_from_spec(_spec)
assert _spec.loader is not None
sys.modules[_spec.name] = manager_status_sync
_spec.loader.exec_module(manager_status_sync)
ManagerStatusSync = manager_status_sync.ManagerStatusSync


def status(epoch="e1", **acks):
    base = {name: 0 for name in ("start", "pause", "resume", "next_point", "skip_point", "stop")}
    base.update(acks)
    return {"command_ack_epoch": epoch, "command_acks": base}


def test_ack_already_committed_returns_without_waiting():
    sync = ManagerStatusSync()
    sync.observe(status(resume=3))
    before = sync.mark("resume")
    # Status arrives before the Trigger future completes (the common case).
    sync.observe(status(resume=4))
    result = sync.wait_for_ack("resume", before, timeout_sec=5.0)
    assert result.synchronized and result.supported
    assert result.wait_ms < 50.0


def test_ack_arriving_after_trigger_wakes_waiter_via_condition():
    sync = ManagerStatusSync()
    sync.observe(status(start=0))
    before = sync.mark("start")

    def late_status():
        time.sleep(0.05)
        sync.observe(status(start=1))

    thread = threading.Thread(target=late_status)
    thread.start()
    result = sync.wait_for_ack("start", before, timeout_sec=5.0)
    thread.join()
    assert result.synchronized
    assert result.wait_ms < 1000.0


def test_other_command_ack_does_not_satisfy_wait():
    sync = ManagerStatusSync()
    sync.observe(status())
    before = sync.mark("skip_point")
    # SKIP may leave the state RUNNING; an unrelated status (or another
    # command's ack) must not be mistaken for the SKIP result.
    sync.observe(status(pause=1))
    result = sync.wait_for_ack("skip_point", before, timeout_sec=0.05)
    assert result.supported and not result.synchronized


def test_timeout_is_reported_not_raised():
    sync = ManagerStatusSync()
    sync.observe(status())
    before = sync.mark("stop")
    result = sync.wait_for_ack("stop", before, timeout_sec=0.02)
    assert result.supported is True
    assert result.synchronized is False
    assert result.as_dict()["command"] == "stop"


def test_older_manager_without_acks_is_not_delayed():
    sync = ManagerStatusSync()
    sync.observe({"state": "RUNNING"})
    before = sync.mark("pause")
    started = time.monotonic()
    result = sync.wait_for_ack("pause", before, timeout_sec=5.0)
    assert result.supported is False
    assert time.monotonic() - started < 0.05


def test_manager_restart_epoch_counts_as_new_domain():
    sync = ManagerStatusSync()
    sync.observe(status(epoch="old", resume=9))
    before = sync.mark("resume")
    sync.observe(status(epoch="new", resume=1))
    assert sync.wait_for_ack("resume", before, timeout_sec=0.05).synchronized


def test_restart_without_the_command_acknowledged_does_not_satisfy():
    sync = ManagerStatusSync()
    sync.observe(status(epoch="old", resume=9))
    before = sync.mark("resume")
    sync.observe(status(epoch="new", resume=0))
    assert not sync.wait_for_ack("resume", before, timeout_sec=0.02).synchronized
