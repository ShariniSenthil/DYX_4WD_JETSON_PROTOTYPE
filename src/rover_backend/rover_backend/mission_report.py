"""Backend-owned live and terminal mission reporting.

The frontend renders this module's canonical JSON; it never calculates point
accuracy or reconstructs point outcomes.  A live report is checkpointed after
upload and after each point event.  The same schema is finalized atomically
before the active trajectory and mission source are removed.
"""

from __future__ import annotations

import copy
import json
import math
import os
import re
import tempfile
import threading
import time

from contextlib import contextmanager
from pathlib import Path
from typing import Any
from typing import Iterator

from rover_backend.config import settings
from rover_backend.mission_store import MissionValidationError
from rover_backend.mission_store import mission_store
from rover_backend.state import rover_state
from rover_backend.state import utc_now_iso


class MissionReportError(RuntimeError):
    """Raised when a mission report cannot be created or finalized."""


class StaleMissionTerminalEvent(MissionReportError):
    """Raised when a terminal event belongs to a replaced mission."""


class MissionReportStore:
    """Build the live report and retain exactly one terminal report."""

    SCHEMA_VERSION = 2
    REPORT_SOURCE = "ROVER_BACKEND"
    MEASUREMENT_SOURCE = "MISSION_MANAGER_LOCAL_SEGMENT"
    POINT_STATUSES = frozenset(
        {"PENDING", "COMPLETED", "SKIPPED", "FAILED"}
    )
    TERMINAL_POINT_STATUSES = frozenset(
        {"COMPLETED", "SKIPPED", "FAILED"}
    )
    _POINT_ID_PATTERN = re.compile(r"^P(\d+)$", re.IGNORECASE)

    def __init__(self, report_file: Path) -> None:
        self.report_file = report_file
        self._lifecycle_lock = threading.RLock()
        self._condition = threading.Condition(threading.RLock())
        self._latest_upload_unix_ns = 0

    @contextmanager
    def lifecycle_transaction(self) -> Iterator[None]:
        """Serialize upload replacement and terminal cleanup transactions."""

        with self._lifecycle_lock:
            yield

    @staticmethod
    def _atomic_write_json(
        destination: Path,
        payload: dict[str, Any],
    ) -> None:
        destination.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        encoded = (
            json.dumps(
                payload,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")

        descriptor: int | None = None
        temporary_path: str | None = None
        try:
            descriptor, temporary_path = tempfile.mkstemp(
                prefix=f".{destination.name}.",
                suffix=".tmp",
                dir=str(destination.parent),
            )
            with os.fdopen(descriptor, "wb") as handle:
                descriptor = None
                handle.write(encoded)
                handle.flush()
                os.fsync(handle.fileno())

            try:
                os.chmod(temporary_path, 0o600)
            except PermissionError:
                pass

            os.replace(temporary_path, destination)
            temporary_path = None
        finally:
            if descriptor is not None:
                try:
                    os.close(descriptor)
                except OSError:
                    pass
            if temporary_path is not None:
                try:
                    os.unlink(temporary_path)
                except OSError:
                    pass

    def _read_report_unlocked(self) -> dict[str, Any] | None:
        if not self.report_file.is_file():
            return None
        try:
            value = json.loads(self.report_file.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            return None
        return value if isinstance(value, dict) else None

    @staticmethod
    def _safe_int(value: Any, default: int = 0) -> int:
        try:
            return int(value)
        except (TypeError, ValueError):
            return default

    @staticmethod
    def _finite_float(value: Any) -> float | None:
        try:
            result = float(value)
        except (TypeError, ValueError):
            return None
        return result if math.isfinite(result) else None

    @classmethod
    def _index_from_point_id(cls, point_id: Any) -> int | None:
        match = cls._POINT_ID_PATTERN.fullmatch(str(point_id or "").strip())
        if match is None:
            return None
        number = cls._safe_int(match.group(1), 0)
        return number - 1 if number > 0 else None

    @classmethod
    def _result_index(
        cls,
        result: dict[str, Any],
        fallback_point_id: Any = None,
    ) -> int | None:
        raw_index = result.get("point_index")
        if raw_index is not None:
            index = cls._safe_int(raw_index, -1)
            if index >= 0:
                return index
        return cls._index_from_point_id(
            result.get("point_id") or fallback_point_id
        )

    @classmethod
    def _canonical_status(cls, *values: Any) -> str:
        """Map all compatibility terms to the four public statuses."""

        for value in values:
            normalised = str(value or "").strip().upper()
            if normalised in cls.POINT_STATUSES:
                return normalised
            if normalised in {
                "MARKED",
                "SPRAYED",
                "SPRAY_COMPLETED",
                "SUCCESS",
                "ACHIEVED",
            }:
                return "COMPLETED"
            if normalised in {
                "ACCURACY_FAILED",
                "SPRAY_FAILED",
                "TIMEOUT",
                "ERROR",
            }:
                return "FAILED"
        return "PENDING"

    @staticmethod
    def _first_value(
        mappings: tuple[dict[str, Any], ...],
        keys: tuple[str, ...],
    ) -> Any:
        for mapping in mappings:
            for key in keys:
                if key in mapping and mapping.get(key) is not None:
                    return mapping.get(key)
        return None

    @classmethod
    def _first_finite(
        cls,
        mappings: tuple[dict[str, Any], ...],
        keys: tuple[str, ...],
    ) -> float | None:
        return cls._finite_float(cls._first_value(mappings, keys))

    @classmethod
    def _canonical_target(
        cls,
        metadata: dict[str, Any],
        index: int,
    ) -> dict[str, Any]:
        targets_value = metadata.get("points")
        targets = targets_value if isinstance(targets_value, list) else []
        value = targets[index] if index < len(targets) else {}
        target = value if isinstance(value, dict) else {}
        mode = str(
            target.get("coordinate_mode")
            or metadata.get("coordinate_mode")
            or "UNKNOWN"
        ).strip().upper()
        if mode == "GPS":
            return {
                "coordinate_mode": "GPS",
                "latitude": cls._finite_float(target.get("latitude")),
                "longitude": cls._finite_float(target.get("longitude")),
            }
        if mode == "LOCAL":
            return {
                "coordinate_mode": "LOCAL",
                "x_m": cls._finite_float(target.get("x_m")),
                "y_m": cls._finite_float(target.get("y_m")),
            }
        return {"coordinate_mode": mode}

    @classmethod
    def _canonical_point(
        cls,
        *,
        index: int,
        raw_status: Any,
        result: dict[str, Any],
        active_index: int | None,
        target: dict[str, Any],
    ) -> dict[str, Any]:
        accuracy_value = result.get("accuracy")
        failure_value = result.get("accuracy_failure")
        accuracy = accuracy_value if isinstance(accuracy_value, dict) else {}
        failure = failure_value if isinstance(failure_value, dict) else {}
        sources = (accuracy, failure, result)

        result_status = cls._canonical_status(
            result.get("point_outcome"),
            result.get("event"),
            accuracy.get("outcome"),
        )
        status_status = cls._canonical_status(raw_status)
        status = (
            status_status
            if status_status in cls.TERMINAL_POINT_STATUSES
            else result_status
            if result_status in cls.TERMINAL_POINT_STATUSES
            else "PENDING"
        )

        cross_track_mm = cls._first_finite(
            sources,
            (
                "cross_track_error_mm",
                "capture_cross_track_error_mm",
                "xtrack_mm",
            ),
        )
        along_track_mm = cls._first_finite(
            sources,
            (
                "along_track_error_mm",
                "front_back_error_mm",
                "capture_along_track_error_mm",
                "along_error_mm",
            ),
        )
        total_accuracy_mm = cls._first_finite(
            sources,
            (
                "total_accuracy_mm",
                "radial_error_mm",
                "closest_radial_error_mm",
                "overall_accuracy_mm",
                "combined_error_mm",
                "capture_radial_error_mm",
            ),
        )
        tolerance_mm = cls._first_finite(
            sources,
            ("tolerance_mm", "accuracy_target_mm"),
        )
        within_tolerance = (
            total_accuracy_mm <= tolerance_mm
            if total_accuracy_mm is not None and tolerance_mm is not None
            else None
        )

        spray_value = result.get("spray")
        spray = spray_value if isinstance(spray_value, dict) else {}
        spray_attempted = bool(
            spray.get("attempted", result.get("spray_attempted", False))
        )
        spray_outcome = str(
            spray.get("outcome") or result.get("spray_outcome") or ""
        ).strip().upper()
        if not spray_attempted:
            spray_outcome = "NOT_ATTEMPTED"
        elif not spray_outcome or spray_outcome == "UNSPECIFIED":
            spray_outcome = "UNKNOWN"

        spray_confirmed_value = result.get("spray_confirmed")
        if spray_confirmed_value is None:
            spray_confirmed = (
                True
                if spray_outcome == "SUCCESS"
                else False
                if spray_outcome in {"FAILED", "TIMEOUT"}
                else None
            )
        else:
            spray_confirmed = bool(spray_confirmed_value)

        return {
            "point_id": f"P{index + 1:04d}",
            "point_index": index,
            "sequence": index + 1,
            "target": copy.deepcopy(target),
            "status": status,
            "is_active": bool(status == "PENDING" and active_index == index),
            "accuracy": {
                "measurement_source": cls.MEASUREMENT_SOURCE,
                "available": any(
                    value is not None
                    for value in (
                        cross_track_mm,
                        along_track_mm,
                        total_accuracy_mm,
                    )
                ),
                "cross_track_error_mm": cross_track_mm,
                "along_track_error_mm": along_track_mm,
                "total_accuracy_mm": total_accuracy_mm,
                "tolerance_mm": tolerance_mm,
                "within_tolerance": within_tolerance,
                "captured_at": cls._first_value(
                    (accuracy, failure, result),
                    ("captured_at", "received_at", "updated_at"),
                ),
            },
            "spray": {
                "attempted": spray_attempted,
                "outcome": spray_outcome,
                "confirmed": spray_confirmed,
                "reason": spray.get("reason")
                or result.get("spray_failure_reason"),
                "elapsed_sec": cls._finite_float(
                    spray.get("elapsed_sec")
                    if spray.get("elapsed_sec") is not None
                    else result.get("spray_elapsed_sec")
                ),
            },
            "reason": cls._first_value(
                (result, accuracy, failure),
                ("reason", "failure_reason"),
            ),
            "updated_at": cls._first_value(
                (result,),
                ("received_at", "completed_at", "updated_at"),
            ),
        }

    @classmethod
    def _result_map(
        cls,
        mission: dict[str, Any],
        terminal_event: dict[str, Any] | None,
    ) -> dict[int, dict[str, Any]]:
        merged: dict[int, dict[str, Any]] = {}
        backend_value = mission.get("point_results")
        if isinstance(backend_value, dict):
            for fallback_id, value in backend_value.items():
                if not isinstance(value, dict):
                    continue
                index = cls._result_index(value, fallback_id)
                if index is not None:
                    merged[index] = copy.deepcopy(value)

        manager_value = (
            terminal_event.get("point_results")
            if isinstance(terminal_event, dict)
            else None
        )
        if isinstance(manager_value, list):
            for value in manager_value:
                if not isinstance(value, dict):
                    continue
                index = cls._result_index(value)
                if index is None:
                    continue
                existing = merged.get(index, {})
                combined = copy.deepcopy(existing)
                combined.update(copy.deepcopy(value))
                for key in (
                    "event",
                    "received_at",
                    "spray_confirmed",
                    "spray_failure_reason",
                    "spray_elapsed_sec",
                ):
                    if key in existing:
                        combined[key] = copy.deepcopy(existing[key])
                if isinstance(existing.get("spray"), dict):
                    enriched_spray = copy.deepcopy(existing["spray"])
                    if isinstance(value.get("spray"), dict):
                        enriched_spray.update(copy.deepcopy(value["spray"]))
                    combined["spray"] = enriched_spray
                merged[index] = combined
        return merged

    @classmethod
    def _recount(cls, report: dict[str, Any]) -> None:
        points_value = report.get("points")
        points = points_value if isinstance(points_value, list) else []
        counts = {
            status: sum(
                isinstance(point, dict) and point.get("status") == status
                for point in points
            )
            for status in cls.POINT_STATUSES
        }
        resolved = (
            counts["COMPLETED"] + counts["SKIPPED"] + counts["FAILED"]
        )
        total = len(points)
        report["summary"] = {
            "total_points": total,
            "pending_points": counts["PENDING"],
            "completed_points": counts["COMPLETED"],
            "skipped_points": counts["SKIPPED"],
            "failed_points": counts["FAILED"],
            "resolved_points": resolved,
            "progress_percent": (
                round(100.0 * resolved / total, 2) if total else 0.0
            ),
        }

    @classmethod
    def _build_report(
        cls,
        *,
        mission: dict[str, Any],
        metadata: dict[str, Any],
        lifecycle: str,
        terminal_event: dict[str, Any] | None = None,
        cleanup: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        terminal_event = (
            terminal_event if isinstance(terminal_event, dict) else {}
        )
        manager_summary_value = terminal_event.get("mission_summary")
        if not isinstance(manager_summary_value, dict):
            manager_summary_value = terminal_event.get("summary")
        manager_summary = (
            manager_summary_value
            if isinstance(manager_summary_value, dict)
            else {}
        )

        point_status_value = manager_summary.get("point_status")
        if not isinstance(point_status_value, list):
            point_status_value = terminal_event.get("point_status")
        if not isinstance(point_status_value, list):
            point_status_value = mission.get("point_status")
        point_status = (
            list(point_status_value)
            if isinstance(point_status_value, list)
            else []
        )

        results = cls._result_map(mission, terminal_event)
        targets_value = metadata.get("points")
        targets = targets_value if isinstance(targets_value, list) else []
        total_points = max(
            0,
            cls._safe_int(metadata.get("total_points"), 0),
            cls._safe_int(mission.get("total_points"), 0),
            cls._safe_int(manager_summary.get("total_points"), 0),
            len(targets),
            len(point_status),
            max(results.keys(), default=-1) + 1,
        )

        active_index_raw = mission.get("active_point_index")
        active_index = (
            cls._safe_int(active_index_raw, -1)
            if active_index_raw is not None
            else -1
        )
        if active_index < 0:
            parsed_active_index = cls._index_from_point_id(
                mission.get("active_point_id")
            )
            active_index = (
                parsed_active_index
                if parsed_active_index is not None
                else -1
            )

        points = [
            cls._canonical_point(
                index=index,
                raw_status=(
                    point_status[index]
                    if index < len(point_status)
                    else None
                ),
                result=results.get(index, {}),
                active_index=active_index,
                target=cls._canonical_target(metadata, index),
            )
            for index in range(total_points)
        ]

        generated_at = utc_now_iso()
        mission_id = str(
            mission.get("mission_id")
            or metadata.get("mission_id")
            or ""
        ).strip()
        mission_run_id = str(
            manager_summary.get("mission_run_id")
            or terminal_event.get("mission_run_id")
            or mission.get("mission_run_id")
            or ""
        ).strip()
        report_state = str(
            manager_summary.get("state")
            or mission.get("state")
            or "EMPTY"
        ).strip().upper()

        termination = None
        disarm_confirmed = None
        warnings: list[Any] = []
        if lifecycle == "TERMINAL":
            termination = str(
                terminal_event.get("termination")
                or (
                    "COMPLETED"
                    if bool(terminal_event.get("completed", False))
                    else "STOPPED"
                )
            ).strip().upper()
            disarm_confirmed = bool(
                terminal_event.get("disarm_confirmed", False)
            )
            warnings_value = terminal_event.get("warnings")
            warnings = (
                list(warnings_value)
                if isinstance(warnings_value, list)
                else []
            )

        cleanup_payload = (
            copy.deepcopy(cleanup)
            if isinstance(cleanup, dict)
            else {
                "status": (
                    "ACTIVE"
                    if lifecycle == "LIVE"
                    else "REPORT_WRITTEN"
                ),
                "complete": False,
                "trajectory_cleared": False,
                "active_artifacts_deleted": False,
                "error": None,
                "updated_at": generated_at,
            }
        )
        report = {
            "schema_version": cls.SCHEMA_VERSION,
            "source": cls.REPORT_SOURCE,
            "lifecycle": lifecycle,
            "report_id": mission_id,
            "mission_id": mission_id,
            "mission_run_id": mission_run_id or None,
            "generated_at": generated_at,
            "state": report_state,
            "termination": termination,
            "disarm_confirmed": disarm_confirmed,
            "warnings": warnings,
            "mission": {
                "filename": metadata.get("active_filename")
                or mission.get("filename"),
                "original_filename": metadata.get("original_filename"),
                "coordinate_mode": str(
                    metadata.get("coordinate_mode")
                    or mission.get("coordinate_mode")
                    or "UNKNOWN"
                ).upper(),
                "execution_mode": mission.get("execution_mode", "AUTO"),
                "extension_mode": metadata.get("extension_mode")
                or mission.get("extension_mode"),
                "total_points": total_points,
                "uploaded_at": metadata.get("uploaded_at")
                or mission.get("uploaded_at"),
                "started_at": mission.get("started_at"),
                "completed_at": mission.get("completed_at"),
            },
            "summary": {},
            "points": points,
            "cleanup": cleanup_payload,
        }
        cls._recount(report)
        return report

    @classmethod
    def _merge_checkpoint(
        cls,
        current: dict[str, Any],
        checkpoint: dict[str, Any] | None,
    ) -> dict[str, Any]:
        """Keep immutable resolved rows across callbacks and restarts."""

        if not isinstance(checkpoint, dict):
            return current
        if checkpoint.get("mission_id") != current.get("mission_id"):
            return current
        old_points_value = checkpoint.get("points")
        new_points_value = current.get("points")
        if not isinstance(old_points_value, list) or not isinstance(
            new_points_value, list
        ):
            return current

        old_by_id = {
            str(point.get("point_id")): point
            for point in old_points_value
            if isinstance(point, dict) and point.get("point_id")
        }
        for index, new_point in enumerate(new_points_value):
            if not isinstance(new_point, dict):
                continue
            old_point = old_by_id.get(str(new_point.get("point_id")))
            if not isinstance(old_point, dict):
                continue
            old_status = str(old_point.get("status") or "").upper()
            new_status = str(new_point.get("status") or "").upper()
            if (
                old_status in cls.TERMINAL_POINT_STATUSES
                and new_status == "PENDING"
            ):
                preserved = copy.deepcopy(old_point)
                preserved["is_active"] = False
                new_points_value[index] = preserved
            elif not new_point.get("target") and old_point.get("target"):
                new_point["target"] = copy.deepcopy(old_point["target"])

        cls._recount(current)
        return current

    @classmethod
    def _canonical_persisted_report(
        cls,
        report: dict[str, Any],
    ) -> dict[str, Any]:
        """Remove private legacy evidence if a schema-v1 report is found."""

        if (
            cls._safe_int(report.get("schema_version"), 0)
            == cls.SCHEMA_VERSION
            and report.get("source") == cls.REPORT_SOURCE
            and isinstance(report.get("points"), list)
        ):
            return copy.deepcopy(report)

        metadata_value = report.get("mission_metadata")
        metadata = metadata_value if isinstance(metadata_value, dict) else {}
        summary_value = report.get("mission_summary")
        summary = summary_value if isinstance(summary_value, dict) else {}
        legacy_results_value = report.get("point_results")
        legacy_results = (
            legacy_results_value
            if isinstance(legacy_results_value, list)
            else []
        )
        mission = {
            "mission_id": report.get("mission_id"),
            "mission_run_id": report.get("mission_run_id"),
            "state": summary.get("state"),
            "execution_mode": summary.get("execution_mode", "AUTO"),
            "total_points": summary.get("total_points"),
            "point_status": summary.get("point_status", []),
            "point_results": {
                str(value.get("point_id") or index): value
                for index, value in enumerate(legacy_results)
                if isinstance(value, dict)
            },
            "started_at": summary.get("started_at"),
            "completed_at": report.get("generated_at"),
        }
        terminal_event = {
            "event": "MISSION_TERMINATED",
            "mission_run_id": report.get("mission_run_id"),
            "completed": str(report.get("termination") or "").upper()
            == "COMPLETED",
            "termination": report.get("termination"),
            "disarm_confirmed": report.get("disarm_confirmed", True),
            "warnings": report.get("warnings", []),
            "mission_summary": summary,
            "point_status": summary.get("point_status", []),
            "point_results": legacy_results,
        }
        canonical = cls._build_report(
            mission=mission,
            metadata=metadata,
            lifecycle="TERMINAL",
            terminal_event=terminal_event,
            cleanup=(
                report.get("cleanup")
                if isinstance(report.get("cleanup"), dict)
                else None
            ),
        )
        if report.get("generated_at"):
            canonical["generated_at"] = report["generated_at"]
        return canonical

    def _load_metadata_or_state(
        self,
        mission: dict[str, Any],
    ) -> dict[str, Any]:
        try:
            metadata = mission_store.load_metadata() or {}
        except (MissionValidationError, OSError):
            metadata = {}
        if not isinstance(metadata, dict):
            metadata = {}
        if not metadata and mission.get("mission_id"):
            metadata = {
                "mission_id": mission.get("mission_id"),
                "active_filename": mission.get("filename"),
                "coordinate_mode": mission.get("coordinate_mode"),
                "extension_mode": mission.get("extension_mode"),
                "total_points": mission.get("total_points", 0),
                "uploaded_at": mission.get("uploaded_at"),
            }
        return metadata

    def _read_canonical_unlocked(self) -> dict[str, Any] | None:
        report = self._read_report_unlocked()
        if report is None:
            return None
        return self._canonical_persisted_report(report)

    def live_report(self) -> dict[str, Any] | None:
        """Return a canonical report for the currently uploaded mission."""

        with self._lifecycle_lock:
            mission = rover_state.section("mission")
            if not str(mission.get("mission_id") or "").strip():
                return None
            metadata = self._load_metadata_or_state(mission)
            current = self._build_report(
                mission=mission,
                metadata=metadata,
                lifecycle="LIVE",
            )
            with self._condition:
                checkpoint = self._read_canonical_unlocked()
            if (
                isinstance(checkpoint, dict)
                and checkpoint.get("lifecycle") == "LIVE"
            ):
                current = self._merge_checkpoint(current, checkpoint)
            return current

    def terminal_report(self) -> dict[str, Any] | None:
        """Return the durable terminal report, if one exists."""

        with self._condition:
            report = self._read_canonical_unlocked()
        if report is None or report.get("lifecycle") != "TERMINAL":
            return None
        return report

    def current_report(self) -> dict[str, Any] | None:
        """Return terminal data during cleanup, otherwise live data."""

        with self._lifecycle_lock:
            mission = rover_state.section("mission")
            mission_id = str(mission.get("mission_id") or "").strip()
            with self._condition:
                persisted = self._read_canonical_unlocked()

            if isinstance(persisted, dict):
                persisted_id = str(
                    persisted.get("mission_id") or ""
                ).strip()
                if persisted.get("lifecycle") == "TERMINAL" and (
                    not mission_id or persisted_id == mission_id
                ):
                    return persisted

            if mission_id:
                metadata = self._load_metadata_or_state(mission)
                current = self._build_report(
                    mission=mission,
                    metadata=metadata,
                    lifecycle="LIVE",
                )
                if (
                    isinstance(persisted, dict)
                    and persisted.get("lifecycle") == "LIVE"
                ):
                    current = self._merge_checkpoint(current, persisted)
                return current

            return persisted

    def checkpoint_live_report(self) -> dict[str, Any] | None:
        """Atomically retain point progress for restart-safe live display."""

        with self._lifecycle_lock:
            mission = rover_state.section("mission")
            mission_id = str(mission.get("mission_id") or "").strip()
            if not mission_id:
                return None
            metadata = self._load_metadata_or_state(mission)
            current = self._build_report(
                mission=mission,
                metadata=metadata,
                lifecycle="LIVE",
            )
            with self._condition:
                persisted = self._read_canonical_unlocked()
                if (
                    isinstance(persisted, dict)
                    and persisted.get("lifecycle") == "LIVE"
                ):
                    current = self._merge_checkpoint(current, persisted)
                try:
                    self._atomic_write_json(self.report_file, current)
                except OSError as error:
                    raise MissionReportError(
                        f"Unable to checkpoint the live mission report: {error}"
                    ) from error
                self._condition.notify_all()
            return current

    def status(self) -> dict[str, Any]:
        """Return display/download availability without point data."""

        report = self.current_report()
        if report is None:
            current = rover_state.section("report")
            error = current.get("error")
            return {
                "available": False,
                "terminal_available": False,
                "status": str(
                    current.get("status")
                    or (
                        "TERMINAL_CLEANUP_FAILED"
                        if error
                        else "UNAVAILABLE"
                    )
                ).upper(),
                "mission_id": current.get("mission_id"),
                "termination": current.get("termination"),
                "cleanup_complete": False,
                "error": error,
                "report_url": None,
                "download_url": None,
                "generated_at": current.get("generated_at"),
                "updated_at": utc_now_iso(),
            }

        cleanup_value = report.get("cleanup")
        cleanup = cleanup_value if isinstance(cleanup_value, dict) else {}
        is_terminal = report.get("lifecycle") == "TERMINAL"
        status = str(
            cleanup.get("status")
            or ("READY" if is_terminal else "LIVE")
        ).upper()
        error = cleanup.get("error")
        current = rover_state.section("report")
        if (
            is_terminal
            and not bool(cleanup.get("complete", False))
            and not error
            and current.get("mission_id") == report.get("mission_id")
            and current.get("error")
        ):
            status = str(
                current.get("status") or "REPORT_UPDATE_FAILED"
            ).upper()
            error = current.get("error")

        return {
            "available": True,
            "terminal_available": is_terminal,
            "status": status,
            "mission_id": report.get("mission_id"),
            "termination": report.get("termination"),
            "cleanup_complete": bool(cleanup.get("complete", False)),
            "error": error,
            "report_url": "/api/mission/report",
            "download_url": (
                "/api/mission/report/download" if is_terminal else None
            ),
            "generated_at": report.get("generated_at"),
            "updated_at": utc_now_iso(),
        }

    def restore_state(self) -> dict[str, Any]:
        with self._condition:
            stored = self._read_report_unlocked()
            if stored is not None:
                canonical = self._canonical_persisted_report(stored)
                if canonical != stored:
                    self._atomic_write_json(self.report_file, canonical)
        report_status = self.status()
        rover_state.update("report", **report_status)
        return report_status

    def save_new_mission(self, **save_arguments: Any) -> dict[str, Any]:
        """Store a new mission and replace the preceding report."""

        with self._lifecycle_lock:
            metadata = mission_store.save(**save_arguments)
            try:
                self.report_file.unlink(missing_ok=True)
                report = self.checkpoint_live_report()
            except (OSError, MissionReportError) as error:
                reason = (
                    "New mission stored but its backend report could not be "
                    f"initialized: {error}"
                )
                rover_state.update(
                    "report",
                    status="INITIALIZE_FAILED",
                    error=reason,
                )
                raise MissionReportError(reason) from error

            self._latest_upload_unix_ns = time.time_ns()
            rover_state.update(
                "report",
                available=True,
                terminal_available=False,
                status="LIVE",
                mission_id=metadata.get("mission_id"),
                termination=None,
                cleanup_complete=False,
                error=None,
                report_url="/api/mission/report",
                download_url=None,
                generated_at=(
                    report.get("generated_at")
                    if isinstance(report, dict)
                    else None
                ),
            )
            with self._condition:
                self._condition.notify_all()
            return metadata

    def write_terminal_report(
        self,
        terminal_event: dict[str, Any],
    ) -> dict[str, Any]:
        """Finalize canonical backend results before active-file cleanup."""

        if (
            str(terminal_event.get("event") or "").strip().upper()
            != "MISSION_TERMINATED"
        ):
            raise MissionReportError(
                "Terminal report requires MISSION_TERMINATED."
            )
        if not bool(terminal_event.get("disarm_confirmed", False)):
            raise MissionReportError(
                "Mission Manager did not confirm PX4 disarm; active mission artifacts were retained."
            )

        event_timestamp = self._safe_int(
            terminal_event.get("timestamp_unix_ns"), 0
        )
        if (
            event_timestamp > 0
            and event_timestamp < self._latest_upload_unix_ns
        ):
            raise StaleMissionTerminalEvent(
                "Ignored a terminal event older than the active upload."
            )

        mission = rover_state.section("mission")
        mission_id = str(mission.get("mission_id") or "").strip()
        if not mission_id:
            raise MissionReportError(
                "No active backend mission is available for the terminal report."
            )

        summary_value = terminal_event.get("mission_summary")
        if not isinstance(summary_value, dict):
            summary_value = terminal_event.get("summary")
        manager_summary = (
            summary_value if isinstance(summary_value, dict) else {}
        )
        event_run_id = str(
            manager_summary.get("mission_run_id")
            or terminal_event.get("mission_run_id")
            or ""
        ).strip()
        state_run_id = str(mission.get("mission_run_id") or "").strip()
        if event_run_id and state_run_id and event_run_id != state_run_id:
            raise StaleMissionTerminalEvent(
                "Terminal mission run does not match the active backend mission."
            )
        if event_run_id and not state_run_id:
            rover_state.update("mission", mission_run_id=event_run_id)
            mission["mission_run_id"] = event_run_id

        metadata = self._load_metadata_or_state(mission)
        report = self._build_report(
            mission=mission,
            metadata=metadata,
            lifecycle="TERMINAL",
            terminal_event=terminal_event,
        )
        with self._condition:
            live_checkpoint = self._read_canonical_unlocked()
        if (
            isinstance(live_checkpoint, dict)
            and live_checkpoint.get("lifecycle") == "LIVE"
        ):
            report = self._merge_checkpoint(report, live_checkpoint)
        if report.get("mission_id") != mission_id:
            raise MissionReportError(
                "Terminal report mission does not match the active backend mission."
            )

        try:
            with self._condition:
                self._atomic_write_json(self.report_file, report)
                self._condition.notify_all()
        except OSError as error:
            raise MissionReportError(
                f"Unable to write the terminal mission report: {error}"
            ) from error

        rover_state.update(
            "report",
            available=True,
            terminal_available=True,
            status="REPORT_WRITTEN",
            mission_id=mission_id,
            termination=report["termination"],
            cleanup_complete=False,
            error=None,
            report_url="/api/mission/report",
            download_url="/api/mission/report/download",
            generated_at=report["generated_at"],
        )
        return report

    def update_cleanup(
        self,
        report: dict[str, Any],
        *,
        status: str,
        complete: bool,
        trajectory_cleared: bool,
        active_artifacts_deleted: bool,
        error: str | None,
    ) -> dict[str, Any]:
        updated = self._canonical_persisted_report(copy.deepcopy(report))
        updated["cleanup"] = {
            "status": str(status).strip().upper(),
            "complete": bool(complete),
            "trajectory_cleared": bool(trajectory_cleared),
            "active_artifacts_deleted": bool(active_artifacts_deleted),
            "error": error,
            "updated_at": utc_now_iso(),
        }
        try:
            with self._condition:
                self._atomic_write_json(self.report_file, updated)
                self._condition.notify_all()
        except OSError as write_error:
            reason = (
                "Unable to update terminal report cleanup status: "
                f"{write_error}"
            )
            rover_state.update(
                "report",
                mission_id=updated.get("mission_id"),
                status="REPORT_UPDATE_FAILED",
                cleanup_complete=False,
                error=reason,
            )
            with self._condition:
                self._condition.notify_all()
            raise MissionReportError(reason) from write_error

        rover_state.update(
            "report",
            available=True,
            terminal_available=True,
            status=updated["cleanup"]["status"],
            mission_id=updated.get("mission_id"),
            termination=updated.get("termination"),
            cleanup_complete=bool(complete),
            error=error,
            report_url="/api/mission/report",
            download_url="/api/mission/report/download",
            generated_at=updated.get("generated_at"),
        )
        return updated

    def expose_failure(self, reason: str) -> None:
        mission = rover_state.section("mission")
        rover_state.update(
            "report",
            mission_id=mission.get("mission_id"),
            status="TERMINAL_CLEANUP_FAILED",
            cleanup_complete=False,
            error=str(reason),
        )
        rover_state.update(
            "mission",
            terminal_cleanup_status="FAILED",
            terminal_cleanup_error=str(reason),
            message=str(reason),
        )
        with self._condition:
            self._condition.notify_all()

    def wait_for_report(
        self,
        mission_id: str,
        timeout_sec: float = 2.0,
    ) -> dict[str, Any] | None:
        deadline = time.monotonic() + max(0.0, float(timeout_sec))
        with self._condition:
            while True:
                report = self._read_canonical_unlocked()
                cleanup = (
                    report.get("cleanup")
                    if isinstance(report, dict)
                    else None
                )
                if (
                    isinstance(report, dict)
                    and report.get("mission_id") == mission_id
                    and report.get("lifecycle") == "TERMINAL"
                    and isinstance(cleanup, dict)
                    and (
                        bool(cleanup.get("complete", False))
                        or bool(cleanup.get("error"))
                    )
                ):
                    return report

                report_state = rover_state.section("report")
                if (
                    report_state.get("mission_id") == mission_id
                    and report_state.get("error")
                ):
                    return {
                        "mission_id": mission_id,
                        "cleanup": {
                            "complete": False,
                            "error": report_state.get("error"),
                        },
                    }

                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return None
                self._condition.wait(remaining)


mission_report_store = MissionReportStore(settings.mission_report_file)
