"""Persistent RTK profile and desired-state ownership.

This module owns configuration persistence only. It does not start worker
processes, touch ROS, call the runtime supervisor, or expose credentials.

The stored NTRIP password is intentionally never present in public profile
snapshots. At-rest encryption/key management is a later security phase; this
store therefore enforces restrictive directory/database permissions and keeps
the secret behind the internal WorkerConfig factory boundary.
"""

from __future__ import annotations

import os
import sqlite3
import threading
import time

from dataclasses import dataclass
from pathlib import Path
from typing import Callable
from typing import Optional

from rover_backend.config import settings
from rover_backend.rtk_manager_core import (
    DesiredState,
)
from rover_backend.rtk_process_protocol import (
    ConfigValidationError,
    WORKER_CONFIG_SCHEMA_VERSION,
    WorkerConfig,
)


RTK_PROFILE_SCHEMA_VERSION = 3

DEFAULT_RTK_DATABASE_FILE = (
    settings.rtk_database_file
)


class RtkProfileStoreError(RuntimeError):
    """Base persistence error."""


class RtkProfileValidationError(
    RtkProfileStoreError,
    ValueError,
):
    """A profile violates the RTK configuration contract."""


class RtkProfileNotFoundError(
    RtkProfileStoreError
):
    """A requested RTK profile does not exist."""


class RtkProfileConflictError(
    RtkProfileStoreError
):
    """A profile name conflicts with an existing profile."""


class RtkProfileStateError(
    RtkProfileStoreError
):
    """Persisted lifecycle state cannot satisfy the requested operation."""


@dataclass(
    frozen=True,
    slots=True,
)
class RtkProfileSnapshot:
    """Credential-free persisted RTK profile."""

    profile_id: int
    name: str

    caster_host: str
    caster_port: int
    mountpoint: str
    username: str

    password_configured: bool

    rtcm_topic: str

    connect_timeout_sec: float
    socket_timeout_sec: float
    healthy_age_sec: float
    stale_reconnect_sec: float
    reconnect_delay_sec: float
    first_data_timeout_sec: float

    gga_enabled: bool
    gga_interval_sec: float
    gga_max_age_sec: float

    tls_mode: str

    max_mavros_rtcm_frame_bytes: int

    enabled: bool

    revision: int
    created_at_epoch: int
    updated_at_epoch: int


@dataclass(
    frozen=True,
    slots=True,
)
class RtkPersistedRuntimeState:
    """Persisted backend RTK ownership target."""

    active_profile_id: Optional[int]
    desired_state: DesiredState

    revision: int
    updated_at_epoch: int


def _contains_control_characters(
    value: str,
) -> bool:
    """Return True for ASCII control bytes unsafe in protocol/log fields."""

    return any(
        ord(character) < 32
        or ord(character) == 127
        for character in value
    )


def _normalise_text(
    value: object,
    name: str,
    *,
    max_chars: int,
) -> str:
    """Normalize ordinary human-readable configuration text."""

    if not isinstance(value, str):
        raise RtkProfileValidationError(
            f"{name} must be a string"
        )

    text = value.strip()

    if not text:
        raise RtkProfileValidationError(
            f"{name} must be non-empty"
        )

    if len(text) > max_chars:
        raise RtkProfileValidationError(
            f"{name} exceeds {max_chars} characters"
        )

    if _contains_control_characters(
        text
    ):
        raise RtkProfileValidationError(
            f"{name} must not contain control characters"
        )

    return text


def _normalise_protocol_token(
    value: object,
    name: str,
    *,
    max_chars: int,
) -> str:
    """Validate one whitespace-free protocol token."""

    text = _normalise_text(
        value,
        name,
        max_chars=max_chars,
    )

    if any(
        character.isspace()
        for character in text
    ):
        raise RtkProfileValidationError(
            f"{name} must not contain whitespace"
        )

    try:
        text.encode("ascii")

    except UnicodeEncodeError as error:
        raise RtkProfileValidationError(
            f"{name} must contain ASCII characters only"
        ) from error

    return text


def _validate_secret(
    value: object,
    name: str = "password",
    *,
    max_chars: int = 2048,
) -> str:
    """Validate a secret without changing any significant whitespace."""

    if not isinstance(value, str):
        raise RtkProfileValidationError(
            f"{name} must be a string"
        )

    # Password bytes/text are semantically opaque. In particular, leading
    # and trailing ordinary spaces must NOT be stripped or normalized.
    if value == "":
        raise RtkProfileValidationError(
            f"{name} must be non-empty"
        )

    if len(value) > max_chars:
        raise RtkProfileValidationError(
            f"{name} exceeds {max_chars} characters"
        )

    if _contains_control_characters(
        value
    ):
        raise RtkProfileValidationError(
            f"{name} must not contain control characters"
        )

    return value


def _normalise_name(
    value: object,
) -> str:
    return _normalise_text(
        value,
        "name",
        max_chars=128,
    )


def _coerce_desired_state(
    value: DesiredState | str,
) -> DesiredState:
    if isinstance(value, DesiredState):
        return value

    if not isinstance(value, str):
        raise RtkProfileValidationError(
            "desired_state must be STOPPED or RUNNING"
        )

    try:
        return DesiredState(
            value.strip().upper()
        )
    except ValueError as error:
        raise RtkProfileValidationError(
            "desired_state must be STOPPED or RUNNING"
        ) from error


class RtkProfileStore:
    """SQLite-backed RTK profiles and persisted desired state."""

    def __init__(
        self,
        database_file: Path,
        *,
        clock: Callable[[], float] = time.time,
    ) -> None:
        if not isinstance(
            database_file,
            Path,
        ):
            raise TypeError(
                "database_file must be a pathlib.Path"
            )

        if not callable(clock):
            raise TypeError(
                "clock must be callable"
            )

        self.database_file = database_file
        self._clock = clock
        self._lock = threading.RLock()

    # ==========================================================
    # Database setup
    # ==========================================================

    def _connect(
        self,
    ) -> sqlite3.Connection:
        connection = sqlite3.connect(
            str(self.database_file),
            timeout=10.0,
            isolation_level=None,
            check_same_thread=False,
        )

        connection.row_factory = sqlite3.Row

        connection.execute(
            "PRAGMA foreign_keys = ON"
        )

        connection.execute(
            "PRAGMA busy_timeout = 10000"
        )

        return connection

    def _now_epoch(
        self,
    ) -> int:
        try:
            value = float(
                self._clock()
            )
        except Exception as error:
            raise RtkProfileStoreError(
                "RTK persistence clock failed"
            ) from error

        if (
            value != value
            or value in {
                float("inf"),
                float("-inf"),
            }
        ):
            raise RtkProfileStoreError(
                "RTK persistence clock must be finite"
            )

        return int(value)

    def initialize(
        self,
    ) -> None:
        """Create or validate the RTK persistence schema."""

        self.database_file.parent.mkdir(
            parents=True,
            exist_ok=True,
            mode=0o700,
        )

        try:
            os.chmod(
                self.database_file.parent,
                0o700,
            )
        except OSError:
            # Some test/filesystem environments do not expose POSIX chmod.
            pass

        with self._lock:
            connection = self._connect()

            try:
                version_row = (
                    connection.execute(
                        "PRAGMA user_version"
                    ).fetchone()
                )

                existing_version = int(
                    version_row[0]
                )

                if existing_version not in {
                    0,
                    1,
                    2,
                    RTK_PROFILE_SCHEMA_VERSION,
                }:
                    raise RtkProfileStoreError(
                        "unsupported RTK profile "
                        f"schema version {existing_version}"
                    )

                connection.execute(
                    "PRAGMA journal_mode = WAL"
                )

                connection.execute(
                    "PRAGMA synchronous = NORMAL"
                )

                connection.execute(
                    "BEGIN IMMEDIATE"
                )

                try:
                    connection.execute(
                        """
                        CREATE TABLE IF NOT EXISTS rtk_profiles (
                            id INTEGER PRIMARY KEY AUTOINCREMENT,

                            name TEXT NOT NULL
                                COLLATE NOCASE
                                UNIQUE,

                            caster_host TEXT NOT NULL,
                            caster_port INTEGER NOT NULL,
                            mountpoint TEXT NOT NULL,
                            username TEXT NOT NULL,

                            password_secret TEXT NOT NULL,

                            rtcm_topic TEXT NOT NULL,

                            connect_timeout_sec REAL NOT NULL,
                            socket_timeout_sec REAL NOT NULL,
                            healthy_age_sec REAL NOT NULL,
                            stale_reconnect_sec REAL NOT NULL,
                            reconnect_delay_sec REAL NOT NULL,
                            first_data_timeout_sec REAL NOT NULL,

                            gga_enabled INTEGER NOT NULL
                                DEFAULT 0
                                CHECK(gga_enabled IN (0, 1)),
                            gga_interval_sec REAL NOT NULL
                                DEFAULT 10.0,
                            gga_max_age_sec REAL NOT NULL
                                DEFAULT 5.0,

                            tls_mode TEXT NOT NULL
                                DEFAULT 'REQUIRED'
                                CHECK(
                                    tls_mode IN (
                                        'REQUIRED',
                                        'DISABLED'
                                    )
                                ),

                            max_mavros_rtcm_frame_bytes
                                INTEGER NOT NULL,

                            enabled INTEGER NOT NULL
                                CHECK(enabled IN (0, 1)),

                            revision INTEGER NOT NULL
                                CHECK(revision >= 1),

                            created_at INTEGER NOT NULL,
                            updated_at INTEGER NOT NULL
                        )
                        """
                    )

                    # Schema v1 -> v2 adds optional GGA/VRS policy.
                    #
                    # Existing fixed-base profiles remain behaviorally
                    # unchanged because gga_enabled defaults to false.
                    if existing_version == 1:
                        connection.execute(
                            """
                            ALTER TABLE rtk_profiles
                            ADD COLUMN gga_enabled INTEGER NOT NULL
                                DEFAULT 0
                                CHECK(gga_enabled IN (0, 1))
                            """
                        )

                        connection.execute(
                            """
                            ALTER TABLE rtk_profiles
                            ADD COLUMN gga_interval_sec REAL NOT NULL
                                DEFAULT 10.0
                            """
                        )

                        connection.execute(
                            """
                            ALTER TABLE rtk_profiles
                            ADD COLUMN gga_max_age_sec REAL NOT NULL
                                DEFAULT 5.0
                            """
                        )

                    # Schema v1/v2 -> v3:
                    # fail secure. Existing profiles become REQUIRED TLS.
                    # A plaintext caster requires an explicit operator PATCH
                    # selecting tls_mode=DISABLED.
                    if existing_version in {
                        1,
                        2,
                    }:
                        connection.execute(
                            """
                            ALTER TABLE rtk_profiles
                            ADD COLUMN tls_mode TEXT NOT NULL
                                DEFAULT 'REQUIRED'
                                CHECK(
                                    tls_mode IN (
                                        'REQUIRED',
                                        'DISABLED'
                                    )
                                )
                            """
                        )

                    connection.execute(
                        """
                        CREATE TABLE IF NOT EXISTS rtk_runtime_state (
                            singleton_id INTEGER PRIMARY KEY
                                CHECK(singleton_id = 1),

                            active_profile_id INTEGER,

                            desired_state TEXT NOT NULL
                                CHECK(
                                    desired_state IN (
                                        'STOPPED',
                                        'RUNNING'
                                    )
                                ),

                            revision INTEGER NOT NULL
                                CHECK(revision >= 1),

                            updated_at INTEGER NOT NULL,

                            FOREIGN KEY(active_profile_id)
                                REFERENCES rtk_profiles(id)
                                ON DELETE SET NULL
                        )
                        """
                    )

                    now = self._now_epoch()

                    connection.execute(
                        """
                        INSERT OR IGNORE
                        INTO rtk_runtime_state (
                            singleton_id,
                            active_profile_id,
                            desired_state,
                            revision,
                            updated_at
                        )
                        VALUES (
                            1,
                            NULL,
                            'STOPPED',
                            1,
                            ?
                        )
                        """,
                        (now,),
                    )

                    connection.execute(
                        f"PRAGMA user_version = "
                        f"{RTK_PROFILE_SCHEMA_VERSION}"
                    )

                    connection.execute(
                        "COMMIT"
                    )

                except BaseException:
                    # Never mask the original initialization failure with a
                    # secondary "no transaction is active" rollback error.
                    if connection.in_transaction:
                        connection.execute(
                            "ROLLBACK"
                        )
                    raise

            finally:
                connection.close()

        if self.database_file.exists():
            try:
                os.chmod(
                    self.database_file,
                    0o600,
                )
            except OSError:
                pass

    # ==========================================================
    # Validation
    # ==========================================================

    def _validated_profile_values(
        self,
        *,
        name: object,
        caster_host: object,
        caster_port: object,
        mountpoint: object,
        username: object,
        password: object,
        rtcm_topic: object,
        connect_timeout_sec: object,
        socket_timeout_sec: object,
        healthy_age_sec: object,
        stale_reconnect_sec: object,
        reconnect_delay_sec: object,
        first_data_timeout_sec: object,
        gga_enabled: object,
        gga_interval_sec: object,
        gga_max_age_sec: object,
        tls_mode: object,
        max_mavros_rtcm_frame_bytes: object,
        enabled: object,
    ) -> dict[str, object]:
        profile_name = _normalise_name(
            name
        )

        host = _normalise_protocol_token(
            caster_host,
            "caster_host",
            max_chars=255,
        )

        point = _normalise_protocol_token(
            mountpoint,
            "mountpoint",
            max_chars=255,
        ).lstrip("/")

        if not point:
            raise RtkProfileValidationError(
                "mountpoint must be non-empty"
            )

        user = _normalise_text(
            username,
            "username",
            max_chars=255,
        )

        secret = _validate_secret(
            password,
            "password",
            max_chars=2048,
        )

        topic = _normalise_protocol_token(
            rtcm_topic,
            "rtcm_topic",
            max_chars=255,
        )

        if not isinstance(
            enabled,
            bool,
        ):
            raise RtkProfileValidationError(
                "enabled must be a bool"
            )

        try:
            config = WorkerConfig(
                schema_version=(
                    WORKER_CONFIG_SCHEMA_VERSION
                ),
                run_id=(
                    "profile-validation"
                ),
                caster_host=host,
                caster_port=caster_port,
                mountpoint=point,
                username=user,
                password=secret,
                rtcm_topic=topic,
                connect_timeout_sec=(
                    connect_timeout_sec
                ),
                socket_timeout_sec=(
                    socket_timeout_sec
                ),
                healthy_age_sec=(
                    healthy_age_sec
                ),
                stale_reconnect_sec=(
                    stale_reconnect_sec
                ),
                reconnect_delay_sec=(
                    reconnect_delay_sec
                ),
                first_data_timeout_sec=(
                    first_data_timeout_sec
                ),
                gga_enabled=gga_enabled,
                gga_interval_sec=(
                    gga_interval_sec
                ),
                gga_max_age_sec=(
                    gga_max_age_sec
                ),
                tls_mode=tls_mode,
                max_mavros_rtcm_frame_bytes=(
                    max_mavros_rtcm_frame_bytes
                ),
            )

        except ConfigValidationError as error:
            raise RtkProfileValidationError(
                str(error)
            ) from error

        return {
            "name": profile_name,
            "caster_host": config.caster_host,
            "caster_port": config.caster_port,
            "mountpoint": config.mountpoint,
            "username": config.username,
            "password_secret": config.password,
            "rtcm_topic": config.rtcm_topic,
            "connect_timeout_sec": (
                config.connect_timeout_sec
            ),
            "socket_timeout_sec": (
                config.socket_timeout_sec
            ),
            "healthy_age_sec": (
                config.healthy_age_sec
            ),
            "stale_reconnect_sec": (
                config.stale_reconnect_sec
            ),
            "reconnect_delay_sec": (
                config.reconnect_delay_sec
            ),
            "first_data_timeout_sec": (
                config.first_data_timeout_sec
            ),
            "gga_enabled": (
                config.gga_enabled
            ),
            "gga_interval_sec": (
                config.gga_interval_sec
            ),
            "gga_max_age_sec": (
                config.gga_max_age_sec
            ),
            "tls_mode": config.tls_mode,
            "max_mavros_rtcm_frame_bytes": (
                config.max_mavros_rtcm_frame_bytes
            ),
            "enabled": bool(enabled),
        }

    # ==========================================================
    # Row projection
    # ==========================================================

    @staticmethod
    def _profile_from_row(
        row: sqlite3.Row,
    ) -> RtkProfileSnapshot:
        return RtkProfileSnapshot(
            profile_id=int(
                row["id"]
            ),
            name=str(
                row["name"]
            ),
            caster_host=str(
                row["caster_host"]
            ),
            caster_port=int(
                row["caster_port"]
            ),
            mountpoint=str(
                row["mountpoint"]
            ),
            username=str(
                row["username"]
            ),
            password_configured=bool(
                str(
                    row["password_secret"]
                )
            ),
            rtcm_topic=str(
                row["rtcm_topic"]
            ),
            connect_timeout_sec=float(
                row["connect_timeout_sec"]
            ),
            socket_timeout_sec=float(
                row["socket_timeout_sec"]
            ),
            healthy_age_sec=float(
                row["healthy_age_sec"]
            ),
            stale_reconnect_sec=float(
                row["stale_reconnect_sec"]
            ),
            reconnect_delay_sec=float(
                row["reconnect_delay_sec"]
            ),
            first_data_timeout_sec=float(
                row["first_data_timeout_sec"]
            ),
            gga_enabled=bool(
                row["gga_enabled"]
            ),
            gga_interval_sec=float(
                row["gga_interval_sec"]
            ),
            gga_max_age_sec=float(
                row["gga_max_age_sec"]
            ),
            tls_mode=str(
                row["tls_mode"]
            ),
            max_mavros_rtcm_frame_bytes=int(
                row[
                    "max_mavros_rtcm_frame_bytes"
                ]
            ),
            enabled=bool(
                row["enabled"]
            ),
            revision=int(
                row["revision"]
            ),
            created_at_epoch=int(
                row["created_at"]
            ),
            updated_at_epoch=int(
                row["updated_at"]
            ),
        )

    @staticmethod
    def _runtime_from_row(
        row: sqlite3.Row,
    ) -> RtkPersistedRuntimeState:
        profile_id = row[
            "active_profile_id"
        ]

        return RtkPersistedRuntimeState(
            active_profile_id=(
                None
                if profile_id is None
                else int(profile_id)
            ),
            desired_state=DesiredState(
                str(
                    row["desired_state"]
                )
            ),
            revision=int(
                row["revision"]
            ),
            updated_at_epoch=int(
                row["updated_at"]
            ),
        )

    # ==========================================================
    # Profile CRUD
    # ==========================================================

    def create_profile(
        self,
        *,
        name: str,
        caster_host: str,
        caster_port: int,
        mountpoint: str,
        username: str,
        password: str,
        rtcm_topic: str = (
            "/mavros/gps_rtk/send_rtcm"
        ),
        connect_timeout_sec: float = 10.0,
        socket_timeout_sec: float = 1.0,
        healthy_age_sec: float = 5.0,
        stale_reconnect_sec: float = 10.0,
        reconnect_delay_sec: float = 5.0,
        first_data_timeout_sec: float = 10.0,
        gga_enabled: bool = False,
        gga_interval_sec: float = 10.0,
        gga_max_age_sec: float = 5.0,
        tls_mode: str = "REQUIRED",
        max_mavros_rtcm_frame_bytes: int = 720,
        enabled: bool = True,
    ) -> RtkProfileSnapshot:
        values = self._validated_profile_values(
            name=name,
            caster_host=caster_host,
            caster_port=caster_port,
            mountpoint=mountpoint,
            username=username,
            password=password,
            rtcm_topic=rtcm_topic,
            connect_timeout_sec=(
                connect_timeout_sec
            ),
            socket_timeout_sec=(
                socket_timeout_sec
            ),
            healthy_age_sec=(
                healthy_age_sec
            ),
            stale_reconnect_sec=(
                stale_reconnect_sec
            ),
            reconnect_delay_sec=(
                reconnect_delay_sec
            ),
            first_data_timeout_sec=(
                first_data_timeout_sec
            ),
            gga_enabled=gga_enabled,
            gga_interval_sec=(
                gga_interval_sec
            ),
            gga_max_age_sec=(
                gga_max_age_sec
            ),
            tls_mode=tls_mode,
            max_mavros_rtcm_frame_bytes=(
                max_mavros_rtcm_frame_bytes
            ),
            enabled=enabled,
        )

        now = self._now_epoch()

        with self._lock:
            connection = self._connect()

            try:
                try:
                    cursor = connection.execute(
                        """
                        INSERT INTO rtk_profiles (
                            name,
                            caster_host,
                            caster_port,
                            mountpoint,
                            username,
                            password_secret,
                            rtcm_topic,
                            connect_timeout_sec,
                            socket_timeout_sec,
                            healthy_age_sec,
                            stale_reconnect_sec,
                            reconnect_delay_sec,
                            first_data_timeout_sec,
                            gga_enabled,
                            gga_interval_sec,
                            gga_max_age_sec,
                            tls_mode,
                            max_mavros_rtcm_frame_bytes,
                            enabled,
                            revision,
                            created_at,
                            updated_at
                        )
                        VALUES (
                            ?, ?, ?, ?, ?, ?, ?,
                            ?, ?, ?, ?, ?, ?,
                            ?, ?, ?, ?, ?, ?,
                            1, ?, ?
                        )
                        """,
                        (
                            values["name"],
                            values["caster_host"],
                            values["caster_port"],
                            values["mountpoint"],
                            values["username"],
                            values["password_secret"],
                            values["rtcm_topic"],
                            values["connect_timeout_sec"],
                            values["socket_timeout_sec"],
                            values["healthy_age_sec"],
                            values["stale_reconnect_sec"],
                            values["reconnect_delay_sec"],
                            values["first_data_timeout_sec"],
                            int(
                                values["gga_enabled"]
                            ),
                            values["gga_interval_sec"],
                            values["gga_max_age_sec"],
                            values["tls_mode"],
                            values[
                                "max_mavros_rtcm_frame_bytes"
                            ],
                            int(
                                values["enabled"]
                            ),
                            now,
                            now,
                        ),
                    )

                except sqlite3.IntegrityError as error:
                    raise RtkProfileConflictError(
                        "RTK profile name already exists"
                    ) from error

                profile_id = int(
                    cursor.lastrowid
                )

                row = connection.execute(
                    """
                    SELECT *
                    FROM rtk_profiles
                    WHERE id = ?
                    """,
                    (profile_id,),
                ).fetchone()

                if row is None:
                    raise RtkProfileStoreError(
                        "created RTK profile "
                        "could not be read back"
                    )

                return self._profile_from_row(
                    row
                )

            finally:
                connection.close()

    def list_profiles(
        self,
    ) -> tuple[RtkProfileSnapshot, ...]:
        with self._lock:
            connection = self._connect()

            try:
                rows = connection.execute(
                    """
                    SELECT *
                    FROM rtk_profiles
                    ORDER BY
                        name COLLATE NOCASE,
                        id
                    """
                ).fetchall()

                return tuple(
                    self._profile_from_row(
                        row
                    )
                    for row in rows
                )

            finally:
                connection.close()

    def get_profile(
        self,
        profile_id: int,
    ) -> RtkProfileSnapshot:
        if (
            isinstance(profile_id, bool)
            or not isinstance(
                profile_id,
                int,
            )
            or profile_id <= 0
        ):
            raise RtkProfileValidationError(
                "profile_id must be a positive int"
            )

        with self._lock:
            connection = self._connect()

            try:
                row = connection.execute(
                    """
                    SELECT *
                    FROM rtk_profiles
                    WHERE id = ?
                    """,
                    (profile_id,),
                ).fetchone()

                if row is None:
                    raise RtkProfileNotFoundError(
                        "RTK profile not found"
                    )

                return self._profile_from_row(
                    row
                )

            finally:
                connection.close()

    def update_profile(
        self,
        profile_id: int,
        *,
        name: Optional[str] = None,
        caster_host: Optional[str] = None,
        caster_port: Optional[int] = None,
        mountpoint: Optional[str] = None,
        username: Optional[str] = None,
        password: Optional[str] = None,
        rtcm_topic: Optional[str] = None,
        connect_timeout_sec: Optional[
            float
        ] = None,
        socket_timeout_sec: Optional[
            float
        ] = None,
        healthy_age_sec: Optional[
            float
        ] = None,
        stale_reconnect_sec: Optional[
            float
        ] = None,
        reconnect_delay_sec: Optional[
            float
        ] = None,
        first_data_timeout_sec: Optional[
            float
        ] = None,
        gga_enabled: Optional[bool] = None,
        gga_interval_sec: Optional[
            float
        ] = None,
        gga_max_age_sec: Optional[
            float
        ] = None,
        tls_mode: Optional[str] = None,
        max_mavros_rtcm_frame_bytes: Optional[
            int
        ] = None,
        enabled: Optional[bool] = None,
    ) -> RtkProfileSnapshot:
        if (
            isinstance(profile_id, bool)
            or not isinstance(
                profile_id,
                int,
            )
            or profile_id <= 0
        ):
            raise RtkProfileValidationError(
                "profile_id must be a positive int"
            )

        now = self._now_epoch()

        with self._lock:
            connection = self._connect()

            try:
                connection.execute(
                    "BEGIN IMMEDIATE"
                )

                try:
                    row = connection.execute(
                        """
                        SELECT *
                        FROM rtk_profiles
                        WHERE id = ?
                        """,
                        (profile_id,),
                    ).fetchone()

                    if row is None:
                        raise (
                            RtkProfileNotFoundError(
                                "RTK profile not found"
                            )
                        )

                    candidate = {
                        "name": (
                            row["name"]
                            if name is None
                            else name
                        ),
                        "caster_host": (
                            row["caster_host"]
                            if caster_host is None
                            else caster_host
                        ),
                        "caster_port": (
                            row["caster_port"]
                            if caster_port is None
                            else caster_port
                        ),
                        "mountpoint": (
                            row["mountpoint"]
                            if mountpoint is None
                            else mountpoint
                        ),
                        "username": (
                            row["username"]
                            if username is None
                            else username
                        ),
                        "password": (
                            row["password_secret"]
                            if password is None
                            else password
                        ),
                        "rtcm_topic": (
                            row["rtcm_topic"]
                            if rtcm_topic is None
                            else rtcm_topic
                        ),
                        "connect_timeout_sec": (
                            row["connect_timeout_sec"]
                            if connect_timeout_sec
                            is None
                            else connect_timeout_sec
                        ),
                        "socket_timeout_sec": (
                            row["socket_timeout_sec"]
                            if socket_timeout_sec
                            is None
                            else socket_timeout_sec
                        ),
                        "healthy_age_sec": (
                            row["healthy_age_sec"]
                            if healthy_age_sec is None
                            else healthy_age_sec
                        ),
                        "stale_reconnect_sec": (
                            row["stale_reconnect_sec"]
                            if stale_reconnect_sec
                            is None
                            else stale_reconnect_sec
                        ),
                        "reconnect_delay_sec": (
                            row["reconnect_delay_sec"]
                            if reconnect_delay_sec
                            is None
                            else reconnect_delay_sec
                        ),
                        "first_data_timeout_sec": (
                            row["first_data_timeout_sec"]
                            if first_data_timeout_sec
                            is None
                            else first_data_timeout_sec
                        ),
                        "gga_enabled": (
                            bool(
                                row["gga_enabled"]
                            )
                            if gga_enabled is None
                            else gga_enabled
                        ),
                        "gga_interval_sec": (
                            row["gga_interval_sec"]
                            if gga_interval_sec is None
                            else gga_interval_sec
                        ),
                        "gga_max_age_sec": (
                            row["gga_max_age_sec"]
                            if gga_max_age_sec is None
                            else gga_max_age_sec
                        ),
                        "tls_mode": (
                            row["tls_mode"]
                            if tls_mode is None
                            else tls_mode
                        ),
                        "max_mavros_rtcm_frame_bytes": (
                            row[
                                "max_mavros_rtcm_frame_bytes"
                            ]
                            if max_mavros_rtcm_frame_bytes
                            is None
                            else max_mavros_rtcm_frame_bytes
                        ),
                        "enabled": (
                            bool(
                                row["enabled"]
                            )
                            if enabled is None
                            else enabled
                        ),
                    }

                    values = (
                        self._validated_profile_values(
                            **candidate
                        )
                    )

                    old_runtime_values = {
                        "caster_host": str(
                            row["caster_host"]
                        ),
                        "caster_port": int(
                            row["caster_port"]
                        ),
                        "mountpoint": str(
                            row["mountpoint"]
                        ),
                        "username": str(
                            row["username"]
                        ),
                        "password_secret": str(
                            row["password_secret"]
                        ),
                        "rtcm_topic": str(
                            row["rtcm_topic"]
                        ),
                        "connect_timeout_sec": float(
                            row[
                                "connect_timeout_sec"
                            ]
                        ),
                        "socket_timeout_sec": float(
                            row[
                                "socket_timeout_sec"
                            ]
                        ),
                        "healthy_age_sec": float(
                            row["healthy_age_sec"]
                        ),
                        "stale_reconnect_sec": float(
                            row[
                                "stale_reconnect_sec"
                            ]
                        ),
                        "reconnect_delay_sec": float(
                            row[
                                "reconnect_delay_sec"
                            ]
                        ),
                        "first_data_timeout_sec": float(
                            row[
                                "first_data_timeout_sec"
                            ]
                        ),
                        "gga_enabled": bool(
                            row["gga_enabled"]
                        ),
                        "gga_interval_sec": float(
                            row["gga_interval_sec"]
                        ),
                        "gga_max_age_sec": float(
                            row["gga_max_age_sec"]
                        ),
                        "tls_mode": str(
                            row["tls_mode"]
                        ),
                        "max_mavros_rtcm_frame_bytes": int(
                            row[
                                "max_mavros_rtcm_frame_bytes"
                            ]
                        ),
                        "enabled": bool(
                            row["enabled"]
                        ),
                    }

                    old_name = str(
                        row["name"]
                    )

                    runtime_changed = any(
                        old_runtime_values[key]
                        != values[key]
                        for key in old_runtime_values
                    )

                    name_changed = (
                        old_name
                        != values["name"]
                    )

                    if (
                        not runtime_changed
                        and not name_changed
                    ):
                        connection.execute(
                            "COMMIT"
                        )

                        return (
                            self._profile_from_row(
                                row
                            )
                        )

                    try:
                        connection.execute(
                            """
                            UPDATE rtk_profiles
                            SET
                                name = ?,
                                caster_host = ?,
                                caster_port = ?,
                                mountpoint = ?,
                                username = ?,
                                password_secret = ?,
                                rtcm_topic = ?,
                                connect_timeout_sec = ?,
                                socket_timeout_sec = ?,
                                healthy_age_sec = ?,
                                stale_reconnect_sec = ?,
                                reconnect_delay_sec = ?,
                                first_data_timeout_sec = ?,
                                gga_enabled = ?,
                                gga_interval_sec = ?,
                                gga_max_age_sec = ?,
                                tls_mode = ?,
                                max_mavros_rtcm_frame_bytes = ?,
                                enabled = ?,
                                revision = revision + 1,
                                updated_at = ?
                            WHERE id = ?
                            """,
                            (
                                values["name"],
                                values["caster_host"],
                                values["caster_port"],
                                values["mountpoint"],
                                values["username"],
                                values[
                                    "password_secret"
                                ],
                                values["rtcm_topic"],
                                values[
                                    "connect_timeout_sec"
                                ],
                                values[
                                    "socket_timeout_sec"
                                ],
                                values[
                                    "healthy_age_sec"
                                ],
                                values[
                                    "stale_reconnect_sec"
                                ],
                                values[
                                    "reconnect_delay_sec"
                                ],
                                values[
                                    "first_data_timeout_sec"
                                ],
                                int(
                                    values["gga_enabled"]
                                ),
                                values[
                                    "gga_interval_sec"
                                ],
                                values[
                                    "gga_max_age_sec"
                                ],
                                values["tls_mode"],
                                values[
                                    "max_mavros_rtcm_frame_bytes"
                                ],
                                int(
                                    values["enabled"]
                                ),
                                now,
                                profile_id,
                            ),
                        )

                    except sqlite3.IntegrityError as error:
                        raise (
                            RtkProfileConflictError(
                                "RTK profile name "
                                "already exists"
                            )
                        ) from error

                    runtime_row = (
                        connection.execute(
                            """
                            SELECT *
                            FROM rtk_runtime_state
                            WHERE singleton_id = 1
                            """
                        ).fetchone()
                    )

                    if runtime_row is None:
                        raise RtkProfileStoreError(
                            "RTK runtime state missing"
                        )

                    is_active = (
                        runtime_row[
                            "active_profile_id"
                        ]
                        == profile_id
                    )

                    if is_active:
                        if not bool(
                            values["enabled"]
                        ):
                            connection.execute(
                                """
                                UPDATE rtk_runtime_state
                                SET
                                    active_profile_id = NULL,
                                    desired_state = 'STOPPED',
                                    revision = revision + 1,
                                    updated_at = ?
                                WHERE singleton_id = 1
                                """,
                                (now,),
                            )

                        elif runtime_changed:
                            # Never allow edited live credentials/transport
                            # settings to silently hot-swap underneath a
                            # currently desired RUNNING worker.
                            connection.execute(
                                """
                                UPDATE rtk_runtime_state
                                SET
                                    desired_state = 'STOPPED',
                                    revision = revision + 1,
                                    updated_at = ?
                                WHERE singleton_id = 1
                                """,
                                (now,),
                            )

                    updated = connection.execute(
                        """
                        SELECT *
                        FROM rtk_profiles
                        WHERE id = ?
                        """,
                        (profile_id,),
                    ).fetchone()

                    if updated is None:
                        raise RtkProfileStoreError(
                            "updated RTK profile "
                            "could not be read back"
                        )

                    connection.execute(
                        "COMMIT"
                    )

                    return self._profile_from_row(
                        updated
                    )

                except BaseException:
                    # Preserve the original failure. Some branches commit
                    # before projecting their return snapshot, so rollback is
                    # legal only while SQLite still reports an active
                    # transaction.
                    if connection.in_transaction:
                        connection.execute(
                            "ROLLBACK"
                        )
                    raise

            finally:
                connection.close()

    def delete_profile(
        self,
        profile_id: int,
    ) -> None:
        if (
            isinstance(profile_id, bool)
            or not isinstance(
                profile_id,
                int,
            )
            or profile_id <= 0
        ):
            raise RtkProfileValidationError(
                "profile_id must be a positive int"
            )

        now = self._now_epoch()

        with self._lock:
            connection = self._connect()

            try:
                connection.execute(
                    "BEGIN IMMEDIATE"
                )

                try:
                    row = connection.execute(
                        """
                        SELECT id
                        FROM rtk_profiles
                        WHERE id = ?
                        """,
                        (profile_id,),
                    ).fetchone()

                    if row is None:
                        raise (
                            RtkProfileNotFoundError(
                                "RTK profile not found"
                            )
                        )

                    runtime_row = (
                        connection.execute(
                            """
                            SELECT active_profile_id
                            FROM rtk_runtime_state
                            WHERE singleton_id = 1
                            """
                        ).fetchone()
                    )

                    if runtime_row is None:
                        raise RtkProfileStoreError(
                            "RTK runtime state missing"
                        )

                    if (
                        runtime_row[
                            "active_profile_id"
                        ]
                        == profile_id
                    ):
                        connection.execute(
                            """
                            UPDATE rtk_runtime_state
                            SET
                                active_profile_id = NULL,
                                desired_state = 'STOPPED',
                                revision = revision + 1,
                                updated_at = ?
                            WHERE singleton_id = 1
                            """,
                            (now,),
                        )

                    connection.execute(
                        """
                        DELETE FROM rtk_profiles
                        WHERE id = ?
                        """,
                        (profile_id,),
                    )

                    connection.execute(
                        "COMMIT"
                    )

                except BaseException:
                    # Preserve the original failure. Some branches commit
                    # before projecting their return snapshot, so rollback is
                    # legal only while SQLite still reports an active
                    # transaction.
                    if connection.in_transaction:
                        connection.execute(
                            "ROLLBACK"
                        )
                    raise

            finally:
                connection.close()

    # ==========================================================
    # Active profile + desired state
    # ==========================================================

    def runtime_state(
        self,
    ) -> RtkPersistedRuntimeState:
        with self._lock:
            connection = self._connect()

            try:
                row = connection.execute(
                    """
                    SELECT *
                    FROM rtk_runtime_state
                    WHERE singleton_id = 1
                    """
                ).fetchone()

                if row is None:
                    raise RtkProfileStoreError(
                        "RTK runtime state missing"
                    )

                return self._runtime_from_row(
                    row
                )

            finally:
                connection.close()

    def set_active_profile(
        self,
        profile_id: int,
    ) -> RtkPersistedRuntimeState:
        if (
            isinstance(profile_id, bool)
            or not isinstance(
                profile_id,
                int,
            )
            or profile_id <= 0
        ):
            raise RtkProfileValidationError(
                "profile_id must be a positive int"
            )

        now = self._now_epoch()

        with self._lock:
            connection = self._connect()

            try:
                connection.execute(
                    "BEGIN IMMEDIATE"
                )

                try:
                    profile = connection.execute(
                        """
                        SELECT id, enabled
                        FROM rtk_profiles
                        WHERE id = ?
                        """,
                        (profile_id,),
                    ).fetchone()

                    if profile is None:
                        raise (
                            RtkProfileNotFoundError(
                                "RTK profile not found"
                            )
                        )

                    if not bool(
                        profile["enabled"]
                    ):
                        raise RtkProfileStateError(
                            "disabled RTK profile "
                            "cannot become active"
                        )

                    current = connection.execute(
                        """
                        SELECT *
                        FROM rtk_runtime_state
                        WHERE singleton_id = 1
                        """
                    ).fetchone()

                    if current is None:
                        raise RtkProfileStoreError(
                            "RTK runtime state missing"
                        )

                    if (
                        current[
                            "active_profile_id"
                        ]
                        == profile_id
                    ):
                        connection.execute(
                            "COMMIT"
                        )

                        return (
                            self._runtime_from_row(
                                current
                            )
                        )

                    # Switching active profile is fail-closed. A later explicit
                    # START is required; live hot-swap is never implicit.
                    connection.execute(
                        """
                        UPDATE rtk_runtime_state
                        SET
                            active_profile_id = ?,
                            desired_state = 'STOPPED',
                            revision = revision + 1,
                            updated_at = ?
                        WHERE singleton_id = 1
                        """,
                        (
                            profile_id,
                            now,
                        ),
                    )

                    updated = connection.execute(
                        """
                        SELECT *
                        FROM rtk_runtime_state
                        WHERE singleton_id = 1
                        """
                    ).fetchone()

                    connection.execute(
                        "COMMIT"
                    )

                    if updated is None:
                        raise RtkProfileStoreError(
                            "RTK runtime state missing"
                        )

                    return self._runtime_from_row(
                        updated
                    )

                except BaseException:
                    # Preserve the original failure. Some branches commit
                    # before projecting their return snapshot, so rollback is
                    # legal only while SQLite still reports an active
                    # transaction.
                    if connection.in_transaction:
                        connection.execute(
                            "ROLLBACK"
                        )
                    raise

            finally:
                connection.close()

    def clear_active_profile(
        self,
    ) -> RtkPersistedRuntimeState:
        now = self._now_epoch()

        with self._lock:
            connection = self._connect()

            try:
                connection.execute(
                    "BEGIN IMMEDIATE"
                )

                try:
                    current = connection.execute(
                        """
                        SELECT *
                        FROM rtk_runtime_state
                        WHERE singleton_id = 1
                        """
                    ).fetchone()

                    if current is None:
                        raise RtkProfileStoreError(
                            "RTK runtime state missing"
                        )

                    if (
                        current[
                            "active_profile_id"
                        ]
                        is None
                        and current[
                            "desired_state"
                        ]
                        == DesiredState.STOPPED.value
                    ):
                        connection.execute(
                            "COMMIT"
                        )

                        return (
                            self._runtime_from_row(
                                current
                            )
                        )

                    connection.execute(
                        """
                        UPDATE rtk_runtime_state
                        SET
                            active_profile_id = NULL,
                            desired_state = 'STOPPED',
                            revision = revision + 1,
                            updated_at = ?
                        WHERE singleton_id = 1
                        """,
                        (now,),
                    )

                    updated = connection.execute(
                        """
                        SELECT *
                        FROM rtk_runtime_state
                        WHERE singleton_id = 1
                        """
                    ).fetchone()

                    connection.execute(
                        "COMMIT"
                    )

                    if updated is None:
                        raise RtkProfileStoreError(
                            "RTK runtime state missing"
                        )

                    return self._runtime_from_row(
                        updated
                    )

                except BaseException:
                    # Preserve the original failure. Some branches commit
                    # before projecting their return snapshot, so rollback is
                    # legal only while SQLite still reports an active
                    # transaction.
                    if connection.in_transaction:
                        connection.execute(
                            "ROLLBACK"
                        )
                    raise

            finally:
                connection.close()

    def set_desired_state(
        self,
        desired_state: DesiredState | str,
    ) -> RtkPersistedRuntimeState:
        desired = _coerce_desired_state(
            desired_state
        )

        now = self._now_epoch()

        with self._lock:
            connection = self._connect()

            try:
                connection.execute(
                    "BEGIN IMMEDIATE"
                )

                try:
                    current = connection.execute(
                        """
                        SELECT *
                        FROM rtk_runtime_state
                        WHERE singleton_id = 1
                        """
                    ).fetchone()

                    if current is None:
                        raise RtkProfileStoreError(
                            "RTK runtime state missing"
                        )

                    current_state = DesiredState(
                        str(
                            current[
                                "desired_state"
                            ]
                        )
                    )

                    if current_state is desired:
                        connection.execute(
                            "COMMIT"
                        )

                        return (
                            self._runtime_from_row(
                                current
                            )
                        )

                    if desired is DesiredState.RUNNING:
                        active_profile_id = current[
                            "active_profile_id"
                        ]

                        if active_profile_id is None:
                            raise RtkProfileStateError(
                                "cannot request RUNNING "
                                "without an active RTK profile"
                            )

                        profile = connection.execute(
                            """
                            SELECT enabled
                            FROM rtk_profiles
                            WHERE id = ?
                            """,
                            (
                                int(
                                    active_profile_id
                                ),
                            ),
                        ).fetchone()

                        if (
                            profile is None
                            or not bool(
                                profile["enabled"]
                            )
                        ):
                            raise RtkProfileStateError(
                                "active RTK profile is "
                                "missing or disabled"
                            )

                    connection.execute(
                        """
                        UPDATE rtk_runtime_state
                        SET
                            desired_state = ?,
                            revision = revision + 1,
                            updated_at = ?
                        WHERE singleton_id = 1
                        """,
                        (
                            desired.value,
                            now,
                        ),
                    )

                    updated = connection.execute(
                        """
                        SELECT *
                        FROM rtk_runtime_state
                        WHERE singleton_id = 1
                        """
                    ).fetchone()

                    connection.execute(
                        "COMMIT"
                    )

                    if updated is None:
                        raise RtkProfileStoreError(
                            "RTK runtime state missing"
                        )

                    return self._runtime_from_row(
                        updated
                    )

                except BaseException:
                    # Preserve the original failure. Some branches commit
                    # before projecting their return snapshot, so rollback is
                    # legal only while SQLite still reports an active
                    # transaction.
                    if connection.in_transaction:
                        connection.execute(
                            "ROLLBACK"
                        )
                    raise

            finally:
                connection.close()

    # ==========================================================
    # Runtime WorkerConfig boundary
    # ==========================================================

    def build_active_worker_config(
        self,
        run_id: str,
    ) -> WorkerConfig:
        """Build one secret-bearing WorkerConfig for the active profile.

        This is the only public store operation that reconstructs the NTRIP
        password. Its result is the existing repr-safe WorkerConfig object.
        """

        with self._lock:
            connection = self._connect()

            try:
                runtime = connection.execute(
                    """
                    SELECT active_profile_id
                    FROM rtk_runtime_state
                    WHERE singleton_id = 1
                    """
                ).fetchone()

                if runtime is None:
                    raise RtkProfileStoreError(
                        "RTK runtime state missing"
                    )

                profile_id = runtime[
                    "active_profile_id"
                ]

                if profile_id is None:
                    raise RtkProfileStateError(
                        "no active RTK profile"
                    )

                row = connection.execute(
                    """
                    SELECT *
                    FROM rtk_profiles
                    WHERE id = ?
                    """,
                    (
                        int(
                            profile_id
                        ),
                    ),
                ).fetchone()

                if row is None:
                    raise RtkProfileStateError(
                        "active RTK profile missing"
                    )

                if not bool(
                    row["enabled"]
                ):
                    raise RtkProfileStateError(
                        "active RTK profile disabled"
                    )

                try:
                    return WorkerConfig(
                        schema_version=(
                            WORKER_CONFIG_SCHEMA_VERSION
                        ),
                        run_id=run_id,
                        caster_host=str(
                            row["caster_host"]
                        ),
                        caster_port=int(
                            row["caster_port"]
                        ),
                        mountpoint=str(
                            row["mountpoint"]
                        ),
                        username=str(
                            row["username"]
                        ),
                        password=str(
                            row["password_secret"]
                        ),
                        rtcm_topic=str(
                            row["rtcm_topic"]
                        ),
                        connect_timeout_sec=float(
                            row[
                                "connect_timeout_sec"
                            ]
                        ),
                        socket_timeout_sec=float(
                            row[
                                "socket_timeout_sec"
                            ]
                        ),
                        healthy_age_sec=float(
                            row[
                                "healthy_age_sec"
                            ]
                        ),
                        stale_reconnect_sec=float(
                            row[
                                "stale_reconnect_sec"
                            ]
                        ),
                        reconnect_delay_sec=float(
                            row[
                                "reconnect_delay_sec"
                            ]
                        ),
                        first_data_timeout_sec=float(
                            row[
                                "first_data_timeout_sec"
                            ]
                        ),
                        gga_enabled=bool(
                            row["gga_enabled"]
                        ),
                        gga_interval_sec=float(
                            row["gga_interval_sec"]
                        ),
                        gga_max_age_sec=float(
                            row["gga_max_age_sec"]
                        ),
                        tls_mode=str(
                            row["tls_mode"]
                        ),
                        max_mavros_rtcm_frame_bytes=int(
                            row[
                                "max_mavros_rtcm_frame_bytes"
                            ]
                        ),
                    )

                except ConfigValidationError as error:
                    raise RtkProfileStateError(
                        "active RTK profile is invalid"
                    ) from error

            finally:
                connection.close()


# Construction only. No filesystem/database work occurs until initialize().
rtk_profile_store = RtkProfileStore(
    DEFAULT_RTK_DATABASE_FILE
)
