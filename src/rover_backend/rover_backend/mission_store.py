#!/usr/bin/env python3

"""Production mission CSV storage for the DYX 4WD rover.

Exactly one active mission CSV is maintained:

    /home/flash/rover_ws/missions/mission.csv

Responsibilities:

- Validate the uploaded CSV.
- Preserve the uploaded marking-point order.
- Support GPS latitude/longitude or local x/y coordinates.
- Support ENABLE or DISABLE extension mode.
- Store the frontend-selected dummy-point distance.
- Atomically replace the active mission CSV.
- Store only mission settings and summary information in JSON.
- Restore mission state after a backend restart.

This module does not calculate dummy points.
This module does not generate trajectory interpolation.
All path planning remains inside trajectory_generator.
"""

from __future__ import annotations

import csv
import hashlib
import io
import json
import math
import os
import tempfile
import threading
import uuid

from dataclasses import dataclass
from pathlib import Path
from typing import Any

from rover_backend.config import settings
from rover_backend.state import rover_state
from rover_backend.state import utc_now_iso


class MissionValidationError(ValueError):
    """Raised when an uploaded mission is invalid."""


@dataclass(
    frozen=True,
    slots=True,
)
class MissionPoint:
    """One validated original CSV marking point."""

    index: int
    first_coordinate: float
    second_coordinate: float


@dataclass(
    frozen=True,
    slots=True,
)
class ValidatedMission:
    """Validated upload ready for atomic storage."""

    stored_bytes: bytes
    coordinate_mode: str
    points: tuple[MissionPoint, ...]
    extension_mode: str
    dummy_point_distance_m: float | None
    original_filename: str


class MissionStore:
    """Validate and maintain the single active mission CSV."""

    LATITUDE_HEADERS = (
        "latitude",
        "lat",
        "gps_lat",
        "gps_latitude",
    )

    LONGITUDE_HEADERS = (
        "longitude",
        "lon",
        "lng",
        "long",
        "gps_lon",
        "gps_longitude",
    )

    VALID_EXTENSION_MODES = {
        "ENABLE",
        "DISABLE",
    }

    MINIMUM_DUMMY_DISTANCE_M = 0.10
    MAXIMUM_DUMMY_DISTANCE_M = 20.0

    MINIMUM_MARKING_POINTS = 2

    def __init__(
        self,
        mission_file: Path,
        metadata_file: Path,
    ) -> None:
        self.mission_file = mission_file
        self.metadata_file = metadata_file

        self._lock = threading.RLock()

    # ==========================================================
    # General helpers
    # ==========================================================

    @staticmethod
    def _safe_filename(
        filename: str,
    ) -> str:
        safe_name = Path(
            str(filename or "")
        ).name.strip()

        if not safe_name:
            safe_name = "mission.csv"

        if not safe_name.lower().endswith(
            ".csv"
        ):
            raise MissionValidationError(
                "The uploaded mission must be a CSV file."
            )

        return safe_name

    @staticmethod
    def _find_header(
        normalised_headers: dict[str, str],
        aliases: tuple[str, ...],
    ) -> str | None:
        for alias in aliases:
            if alias in normalised_headers:
                return normalised_headers[alias]

        return None

    @staticmethod
    def _parse_finite_float(
        value: Any,
        *,
        row_number: int,
        field_name: str,
    ) -> float:
        raw_value = str(
            value if value is not None else ""
        ).strip()

        if not raw_value:
            raise MissionValidationError(
                f"CSV row {row_number}: "
                f"{field_name} is empty."
            )

        try:
            result = float(raw_value)

        except ValueError as error:
            raise MissionValidationError(
                f"CSV row {row_number}: "
                f"{field_name} must be numeric."
            ) from error

        if not math.isfinite(result):
            raise MissionValidationError(
                f"CSV row {row_number}: "
                f"{field_name} must be finite."
            )

        return result

    @staticmethod
    def _sha256(
        data: bytes,
    ) -> str:
        return hashlib.sha256(
            data
        ).hexdigest()

    @staticmethod
    def _normalise_csv_bytes(
        text: str,
    ) -> bytes:
        """Store validated CSV as UTF-8 with consistent line endings."""

        normalised_text = (
            text.replace(
                "\r\n",
                "\n",
            )
            .replace(
                "\r",
                "\n",
            )
        )

        if not normalised_text.endswith(
            "\n"
        ):
            normalised_text += "\n"

        return normalised_text.encode(
            "utf-8"
        )

    # ==========================================================
    # Extension validation
    # ==========================================================

    def _validate_extension_settings(
        self,
        extension_mode: str,
        dummy_point_distance_m: Any,
    ) -> tuple[str, float | None]:
        mode = str(
            extension_mode or ""
        ).strip().upper()

        if mode not in self.VALID_EXTENSION_MODES:
            raise MissionValidationError(
                "extension_mode must be "
                "ENABLE or DISABLE."
            )

        if mode == "DISABLE":
            return mode, None

        # ENABLE uses the frontend value when supplied.
        # Otherwise, it uses the configured 3.5 m fallback.
        if dummy_point_distance_m in {
            None,
            "",
        }:
            dummy_distance = float(
                settings
                .default_dummy_point_distance_m
            )

        else:
            try:
                dummy_distance = float(
                    dummy_point_distance_m
                )

            except (
                TypeError,
                ValueError,
            ) as error:
                raise MissionValidationError(
                    "dummy_point_distance_m "
                    "must be numeric."
                ) from error

        if not math.isfinite(
            dummy_distance
        ):
            raise MissionValidationError(
                "dummy_point_distance_m "
                "must be finite."
            )

        if not (
            self.MINIMUM_DUMMY_DISTANCE_M
            <= dummy_distance
            <= self.MAXIMUM_DUMMY_DISTANCE_M
        ):
            raise MissionValidationError(
                "dummy_point_distance_m must be "
                f"between "
                f"{self.MINIMUM_DUMMY_DISTANCE_M:.2f} m "
                "and "
                f"{self.MAXIMUM_DUMMY_DISTANCE_M:.2f} m."
            )

        return mode, dummy_distance

    # ==========================================================
    # CSV validation
    # ==========================================================

    def validate(
        self,
        *,
        raw_bytes: bytes,
        filename: str,
        extension_mode: str,
        dummy_point_distance_m: Any = None,
    ) -> ValidatedMission:
        """Validate one frontend-uploaded mission CSV."""

        if not isinstance(
            raw_bytes,
            bytes,
        ):
            raise MissionValidationError(
                "Mission upload data is invalid."
            )

        if not raw_bytes:
            raise MissionValidationError(
                "The uploaded CSV is empty."
            )

        if (
            len(raw_bytes)
            > settings.maximum_upload_bytes
        ):
            maximum_mb = (
                settings.maximum_upload_bytes
                / (1024 * 1024)
            )

            raise MissionValidationError(
                "The uploaded CSV exceeds "
                f"{maximum_mb:.1f} MB."
            )

        if b"\x00" in raw_bytes:
            raise MissionValidationError(
                "The uploaded CSV contains "
                "invalid null characters."
            )

        safe_filename = self._safe_filename(
            filename
        )

        try:
            csv_text = raw_bytes.decode(
                "utf-8-sig"
            )

        except UnicodeDecodeError as error:
            raise MissionValidationError(
                "The CSV must use UTF-8 encoding."
            ) from error

        if not csv_text.strip():
            raise MissionValidationError(
                "The uploaded CSV contains no data."
            )

        mode, dummy_distance = (
            self._validate_extension_settings(
                extension_mode,
                dummy_point_distance_m,
            )
        )

        # Prevent a single malformed field from consuming
        # excessive memory in Python's CSV parser.
        csv.field_size_limit(
            min(
                settings.maximum_upload_bytes,
                1_000_000,
            )
        )

        reader = csv.DictReader(
            io.StringIO(
                csv_text,
                newline="",
            )
        )

        if reader.fieldnames is None:
            raise MissionValidationError(
                "The CSV has no header row."
            )

        original_headers = [
            str(header).strip()
            for header in reader.fieldnames
            if (
                header is not None
                and str(header).strip()
            )
        ]

        if not original_headers:
            raise MissionValidationError(
                "The CSV header row is empty."
            )

        normalised_header_names = [
            header.lower()
            for header in original_headers
        ]

        if (
            len(normalised_header_names)
            != len(
                set(
                    normalised_header_names
                )
            )
        ):
            raise MissionValidationError(
                "The CSV contains duplicate "
                "column names."
            )

        normalised_headers = {
            header.lower(): header
            for header in original_headers
        }

        latitude_column = (
            self._find_header(
                normalised_headers,
                self.LATITUDE_HEADERS,
            )
        )

        longitude_column = (
            self._find_header(
                normalised_headers,
                self.LONGITUDE_HEADERS,
            )
        )

        x_column = normalised_headers.get(
            "x"
        )

        y_column = normalised_headers.get(
            "y"
        )

        has_gps_columns = (
            latitude_column is not None
            and longitude_column is not None
        )

        has_local_columns = (
            x_column is not None
            and y_column is not None
        )

        if (
            has_gps_columns
            and has_local_columns
        ):
            raise MissionValidationError(
                "The CSV must contain either "
                "latitude/longitude or x/y, "
                "not both coordinate systems."
            )

        if has_gps_columns:
            coordinate_mode = "gps"
            first_column = latitude_column
            second_column = longitude_column

        elif has_local_columns:
            coordinate_mode = "local"
            first_column = x_column
            second_column = y_column

        else:
            raise MissionValidationError(
                "The CSV must contain either "
                "latitude,longitude columns "
                "or x,y columns."
            )

        points: list[MissionPoint] = []
        encountered_points: set[
            tuple[float, float]
        ] = set()

        for row_number, row in enumerate(
            reader,
            start=2,
        ):
            if row is None:
                continue

            if None in row:
                extra_values = row.get(None)

                if extra_values and any(
                    str(value).strip()
                    for value in extra_values
                    if value is not None
                ):
                    raise MissionValidationError(
                        f"CSV row {row_number} "
                        "contains more values than "
                        "the header row."
                    )

            has_any_value = any(
                value is not None
                and str(value).strip()
                for key, value in row.items()
                if key is not None
            )

            if not has_any_value:
                continue

            first_coordinate = (
                self._parse_finite_float(
                    row.get(
                        first_column
                    ),
                    row_number=row_number,
                    field_name=(
                        first_column
                    ),
                )
            )

            second_coordinate = (
                self._parse_finite_float(
                    row.get(
                        second_column
                    ),
                    row_number=row_number,
                    field_name=(
                        second_column
                    ),
                )
            )

            if coordinate_mode == "gps":
                if not (
                    -90.0
                    <= first_coordinate
                    <= 90.0
                ):
                    raise MissionValidationError(
                        f"CSV row {row_number}: "
                        "latitude must be between "
                        "-90 and 90 degrees."
                    )

                if not (
                    -180.0
                    <= second_coordinate
                    <= 180.0
                ):
                    raise MissionValidationError(
                        f"CSV row {row_number}: "
                        "longitude must be between "
                        "-180 and 180 degrees."
                    )

            point_key = (
                first_coordinate,
                second_coordinate,
            )

            if point_key in encountered_points:
                raise MissionValidationError(
                    f"CSV row {row_number}: "
                    "duplicate marking point."
                )

            encountered_points.add(
                point_key
            )

            points.append(
                MissionPoint(
                    index=len(points),
                    first_coordinate=(
                        first_coordinate
                    ),
                    second_coordinate=(
                        second_coordinate
                    ),
                )
            )

            if (
                len(points)
                > settings.maximum_marking_points
            ):
                raise MissionValidationError(
                    "The mission contains more than "
                    f"{settings.maximum_marking_points} "
                    "marking points."
                )

        if (
            len(points)
            < self.MINIMUM_MARKING_POINTS
        ):
            raise MissionValidationError(
                "The mission requires at least "
                f"{self.MINIMUM_MARKING_POINTS} "
                "marking points."
            )

        stored_bytes = (
            self._normalise_csv_bytes(
                csv_text
            )
        )

        return ValidatedMission(
            stored_bytes=stored_bytes,
            coordinate_mode=coordinate_mode,
            points=tuple(points),
            extension_mode=mode,
            dummy_point_distance_m=(
                dummy_distance
            ),
            original_filename=(
                safe_filename
            ),
        )

    # ==========================================================
    # Atomic file operations
    # ==========================================================

    @staticmethod
    def _fsync_directory(
        directory: Path,
    ) -> None:
        """Flush directory metadata when supported."""

        directory_fd: int | None = None

        try:
            directory_fd = os.open(
                str(directory),
                os.O_RDONLY,
            )

            os.fsync(
                directory_fd
            )

        except OSError:
            # Directory fsync may be unavailable during
            # Windows-side syntax checking.
            pass

        finally:
            if directory_fd is not None:
                os.close(
                    directory_fd
                )

    @classmethod
    def _atomic_write_bytes(
        cls,
        destination: Path,
        data: bytes,
    ) -> None:
        destination.parent.mkdir(
            parents=True,
            exist_ok=True,
            mode=0o700,
        )

        temporary_path: str | None = None
        file_descriptor: int | None = None

        try:
            (
                file_descriptor,
                temporary_path,
            ) = tempfile.mkstemp(
                prefix=(
                    f".{destination.name}."
                ),
                suffix=".tmp",
                dir=str(
                    destination.parent
                ),
            )

            with os.fdopen(
                file_descriptor,
                "wb",
            ) as handle:
                file_descriptor = None

                handle.write(
                    data
                )

                handle.flush()
                os.fsync(
                    handle.fileno()
                )

            try:
                os.chmod(
                    temporary_path,
                    0o600,
                )

            except PermissionError:
                pass

            os.replace(
                temporary_path,
                destination,
            )

            temporary_path = None

            cls._fsync_directory(
                destination.parent
            )

        finally:
            if file_descriptor is not None:
                try:
                    os.close(
                        file_descriptor
                    )

                except OSError:
                    pass

            if (
                temporary_path is not None
                and os.path.exists(
                    temporary_path
                )
            ):
                try:
                    os.unlink(
                        temporary_path
                    )

                except OSError:
                    pass

    @classmethod
    def _atomic_write_json(
        cls,
        destination: Path,
        value: dict[str, Any],
    ) -> None:
        encoded = (
            json.dumps(
                value,
                ensure_ascii=False,
                indent=2,
                sort_keys=True,
            )
            + "\n"
        ).encode(
            "utf-8"
        )

        cls._atomic_write_bytes(
            destination,
            encoded,
        )

    @classmethod
    def _restore_previous_file(
        cls,
        path: Path,
        previous_data: bytes | None,
    ) -> None:
        if previous_data is None:
            try:
                path.unlink(
                    missing_ok=True
                )

            except OSError:
                pass

            return

        cls._atomic_write_bytes(
            path,
            previous_data,
        )

    # ==========================================================
    # Save and restore
    # ==========================================================

    def save(
        self,
        *,
        raw_bytes: bytes,
        filename: str,
        extension_mode: str,
        dummy_point_distance_m: Any = None,
    ) -> dict[str, Any]:
        """Validate and atomically replace the active mission."""

        validated = self.validate(
            raw_bytes=raw_bytes,
            filename=filename,
            extension_mode=extension_mode,
            dummy_point_distance_m=(
                dummy_point_distance_m
            ),
        )

        checksum = self._sha256(
            validated.stored_bytes
        )

        uploaded_at = utc_now_iso()
        mission_id = uuid.uuid4().hex

        metadata: dict[str, Any] = {
            "schema_version": 1,
            "mission_id": mission_id,

            # Only one production mission CSV exists.
            "active_filename": (
                self.mission_file.name
            ),

            "original_filename": (
                validated.original_filename
            ),

            "checksum_sha256": checksum,
            "coordinate_mode": (
                validated.coordinate_mode
            ),

            "extension_mode": (
                validated.extension_mode
            ),

            "dummy_point_distance_m": (
                validated
                .dummy_point_distance_m
            ),

            "row_transition_threshold_m": (
                settings
                .row_transition_threshold_m
            ),

            "total_points": len(
                validated.points
            ),

            "uploaded_at": uploaded_at,
        }

        with self._lock:
            previous_mission = (
                self.mission_file.read_bytes()
                if self.mission_file.is_file()
                else None
            )

            previous_metadata = (
                self.metadata_file.read_bytes()
                if self.metadata_file.is_file()
                else None
            )

            try:
                self._atomic_write_bytes(
                    self.mission_file,
                    validated.stored_bytes,
                )

                self._atomic_write_json(
                    self.metadata_file,
                    metadata,
                )

            except Exception:
                # Restore the previous valid mission when either
                # part of the replacement fails.
                self._restore_previous_file(
                    self.mission_file,
                    previous_mission,
                )

                self._restore_previous_file(
                    self.metadata_file,
                    previous_metadata,
                )

                raise

        rover_state.load_mission(
            mission_id=mission_id,
            filename=self.mission_file.name,
            checksum_sha256=checksum,
            coordinate_mode=(
                validated.coordinate_mode
            ),
            extension_mode=(
                validated.extension_mode
            ),
            dummy_point_distance_m=(
                validated
                .dummy_point_distance_m
            ),
            row_transition_threshold_m=(
                settings
                .row_transition_threshold_m
            ),
            total_points=len(
                validated.points
            ),
            uploaded_at=uploaded_at,
        )

        return dict(
            metadata
        )

    def load_metadata(
        self,
    ) -> dict[str, Any] | None:
        """Load and verify the currently stored mission."""

        with self._lock:
            mission_exists = (
                self.mission_file.is_file()
            )

            metadata_exists = (
                self.metadata_file.is_file()
            )

            if (
                not mission_exists
                and not metadata_exists
            ):
                return None

            if not mission_exists:
                raise MissionValidationError(
                    "Mission metadata exists but "
                    "mission.csv is missing."
                )

            if not metadata_exists:
                raise MissionValidationError(
                    "mission.csv exists but mission "
                    "metadata is missing."
                )

            try:
                metadata_value = json.loads(
                    self.metadata_file.read_text(
                        encoding="utf-8"
                    )
                )

            except (
                OSError,
                json.JSONDecodeError,
            ) as error:
                raise MissionValidationError(
                    "Mission metadata is invalid."
                ) from error

            if not isinstance(
                metadata_value,
                dict,
            ):
                raise MissionValidationError(
                    "Mission metadata must be "
                    "a JSON object."
                )

            mission_bytes = (
                self.mission_file.read_bytes()
            )

            expected_checksum = str(
                metadata_value.get(
                    "checksum_sha256",
                    "",
                )
            ).strip()

            actual_checksum = self._sha256(
                mission_bytes
            )

            if (
                not expected_checksum
                or expected_checksum
                != actual_checksum
            ):
                raise MissionValidationError(
                    "Stored mission checksum "
                    "verification failed."
                )

            extension_mode = str(
                metadata_value.get(
                    "extension_mode",
                    "",
                )
            ).strip().upper()

            dummy_distance = metadata_value.get(
                "dummy_point_distance_m"
            )

            validated = self.validate(
                raw_bytes=mission_bytes,
                filename=self.mission_file.name,
                extension_mode=extension_mode,
                dummy_point_distance_m=(
                    dummy_distance
                ),
            )

            metadata_total = metadata_value.get(
                "total_points"
            )

            try:
                metadata_total_int = int(
                    metadata_total
                )

            except (
                TypeError,
                ValueError,
            ) as error:
                raise MissionValidationError(
                    "Stored mission point count "
                    "is invalid."
                ) from error

            if (
                metadata_total_int
                != len(
                    validated.points
                )
            ):
                raise MissionValidationError(
                    "Stored mission point count "
                    "verification failed."
                )

            return dict(
                metadata_value
            )

    def restore_state(
        self,
    ) -> dict[str, Any] | None:
        """Restore a stored mission as LOADED after restart.

        The mission is never restored as RUNNING.
        """

        metadata = self.load_metadata()

        if metadata is None:
            rover_state.clear_mission_runtime(
                retain_loaded_file=False
            )

            return None

        rover_state.load_mission(
            mission_id=str(
                metadata["mission_id"]
            ),
            filename=str(
                metadata["active_filename"]
            ),
            checksum_sha256=str(
                metadata["checksum_sha256"]
            ),
            coordinate_mode=str(
                metadata["coordinate_mode"]
            ),
            extension_mode=str(
                metadata["extension_mode"]
            ),
            dummy_point_distance_m=(
                metadata.get(
                    "dummy_point_distance_m"
                )
            ),
            row_transition_threshold_m=float(
                metadata[
                    "row_transition_threshold_m"
                ]
            ),
            total_points=int(
                metadata["total_points"]
            ),
            uploaded_at=str(
                metadata["uploaded_at"]
            ),
        )

        return metadata

    # ==========================================================
    # Reading and deletion
    # ==========================================================

    def read_active_csv(
        self,
    ) -> bytes:
        """Return the active production mission.csv."""

        with self._lock:
            if not self.mission_file.is_file():
                raise MissionValidationError(
                    "No mission.csv is stored."
                )

            return self.mission_file.read_bytes()

    def delete(
        self,
    ) -> bool:
        """Delete the active mission and its runtime metadata."""

        with self._lock:
            existed = (
                self.mission_file.exists()
                or self.metadata_file.exists()
                or settings
                .mission_runtime_file
                .exists()
            )

            for path in (
                self.mission_file,
                self.metadata_file,
                settings.mission_runtime_file,
            ):
                try:
                    path.unlink(
                        missing_ok=True
                    )

                except OSError as error:
                    raise RuntimeError(
                        f"Unable to delete {path}"
                    ) from error

        rover_state.clear_mission_runtime(
            retain_loaded_file=False
        )

        return existed


mission_store = MissionStore(
    mission_file=settings.mission_file,
    metadata_file=(
        settings.mission_metadata_file
    ),
)