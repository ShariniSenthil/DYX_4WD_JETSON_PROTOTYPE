from __future__ import annotations

from types import SimpleNamespace

from rover_backend import mission_store as mission_store_module


def _store(tmp_path, monkeypatch):
    runtime = tmp_path / "runtime"
    settings = SimpleNamespace(
        maximum_upload_bytes=5 * 1024 * 1024,
        maximum_marking_points=10_000,
        default_dummy_point_distance_m=4.0,
        row_transition_threshold_m=3.0,
        mission_runtime_file=runtime / "mission_runtime.json",
    )
    monkeypatch.setattr(mission_store_module, "settings", settings)
    return mission_store_module.MissionStore(
        tmp_path / "mission.csv",
        runtime / "mission_metadata.json",
        tmp_path / "archive",
    )


def test_completed_mission_is_archived_and_can_be_restored(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)
    uploaded = store.save(
        raw_bytes=b"latitude,longitude\n13.1,80.2\n13.2,80.3\n",
        filename="field.csv",
        extension_mode="DISABLE",
    )

    archived = store.archive_active_mission()

    assert archived is not None
    assert archived["mission_id"] == uploaded["mission_id"]
    assert not store.mission_file.exists()
    assert store.list_archived_missions()[0]["mission_id"] == uploaded["mission_id"]

    restored = store.restore_archived_mission(uploaded["mission_id"])
    assert restored["mission_id"] == uploaded["mission_id"]
    assert store.load_metadata()["checksum_sha256"] == uploaded["checksum_sha256"]


def test_archive_rejects_path_traversal_ids(tmp_path, monkeypatch):
    store = _store(tmp_path, monkeypatch)

    try:
        store.restore_archived_mission("../mission")
    except mission_store_module.MissionValidationError as error:
        assert "not found" in str(error).lower() or "invalid" in str(error).lower()
    else:
        raise AssertionError("path traversal mission ID was accepted")
