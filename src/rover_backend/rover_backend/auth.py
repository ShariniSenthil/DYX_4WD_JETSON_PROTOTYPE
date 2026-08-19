#!/usr/bin/env python3

"""Persistent authentication for the DYX 4WD Rover Backend."""

from __future__ import annotations

import hashlib
import secrets
import sqlite3
import threading
import time

from collections.abc import Callable
from dataclasses import dataclass
from datetime import datetime
from datetime import timezone
from pathlib import Path
from typing import Any

from argon2 import PasswordHasher
from argon2.exceptions import InvalidHashError
from argon2.exceptions import VerificationError
from argon2.exceptions import VerifyMismatchError
from fastapi import APIRouter
from fastapi import Depends
from fastapi import HTTPException
from fastapi import Request
from pydantic import BaseModel
from pydantic import Field

from rover_backend.config import settings


def _epoch_now() -> int:
    return int(time.time())


def _epoch_to_iso(
    value: int,
) -> str:
    return (
        datetime.fromtimestamp(
            value,
            tz=timezone.utc,
        )
        .isoformat(timespec="seconds")
        .replace("+00:00", "Z")
    )


def _token_hash(
    token: str,
) -> str:
    return hashlib.sha256(
        token.encode("utf-8")
    ).hexdigest()


def _normalise_username(
    username: str,
) -> str:
    return username.strip().lower()


def _request_ip(
    request: Request,
) -> str:
    if request.client is None:
        return "unknown"

    return request.client.host or "unknown"


def extract_request_token(
    request: Request,
) -> str | None:
    """Read a token from Authorization or X-Rover-Token."""

    authorization = request.headers.get(
        "Authorization",
        "",
    ).strip()

    if authorization:
        scheme, separator, value = (
            authorization.partition(" ")
        )

        if (
            separator
            and scheme.lower() == "bearer"
            and value.strip()
        ):
            return value.strip()

    rover_token = request.headers.get(
        "X-Rover-Token",
        "",
    ).strip()

    if rover_token:
        return rover_token

    return None


@dataclass(
    frozen=True,
    slots=True,
)
class AuthenticatedSession:
    session_id: str
    username: str
    created_at: str
    expires_at: str
    client_ip: str
    user_agent: str


class LoginRequest(BaseModel):
    username: str = Field(
        min_length=1,
        max_length=128,
    )

    password: str = Field(
        min_length=1,
        max_length=512,
    )


class LogoutResponse(BaseModel):
    success: bool
    message: str


class AuthenticationStore:
    """SQLite-backed users, sessions and login-attempt storage."""

    def __init__(
        self,
        database_file: Path,
    ) -> None:
        self.database_file = database_file

        self._lock = threading.RLock()

        self._password_hasher = PasswordHasher(
            time_cost=3,
            memory_cost=65_536,
            parallelism=2,
            hash_len=32,
            salt_len=16,
        )

        self._revocation_callbacks: list[
            Callable[[str], None]
        ] = []

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

    def initialize(
        self,
    ) -> None:
        """Create the database and synchronize the configured user."""

        self.database_file.parent.mkdir(
            parents=True,
            exist_ok=True,
            mode=0o700,
        )

        with self._lock:
            connection = self._connect()

            try:
                connection.execute(
                    "PRAGMA journal_mode = WAL"
                )

                connection.execute(
                    "PRAGMA synchronous = NORMAL"
                )

                connection.executescript(
                    """
                    CREATE TABLE IF NOT EXISTS users (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        username TEXT NOT NULL UNIQUE,
                        password_hash TEXT NOT NULL,
                        active INTEGER NOT NULL DEFAULT 1,
                        created_at INTEGER NOT NULL,
                        updated_at INTEGER NOT NULL
                    );

                    CREATE TABLE IF NOT EXISTS sessions (
                        session_id TEXT PRIMARY KEY,
                        token_hash TEXT NOT NULL UNIQUE,
                        user_id INTEGER NOT NULL,
                        created_at INTEGER NOT NULL,
                        expires_at INTEGER NOT NULL,
                        last_seen_at INTEGER NOT NULL,
                        revoked_at INTEGER,
                        client_ip TEXT NOT NULL,
                        user_agent TEXT NOT NULL,
                        FOREIGN KEY(user_id)
                            REFERENCES users(id)
                            ON DELETE CASCADE
                    );

                    CREATE INDEX IF NOT EXISTS
                        idx_sessions_user_active
                    ON sessions(
                        user_id,
                        revoked_at,
                        expires_at
                    );

                    CREATE TABLE IF NOT EXISTS login_failures (
                        id INTEGER PRIMARY KEY AUTOINCREMENT,
                        username TEXT NOT NULL,
                        client_ip TEXT NOT NULL,
                        failed_at INTEGER NOT NULL
                    );

                    CREATE INDEX IF NOT EXISTS
                        idx_login_failures_lookup
                    ON login_failures(
                        username,
                        client_ip,
                        failed_at
                    );
                    """
                )

                self._synchronise_static_user(
                    connection
                )

                self._cleanup_expired_data(
                    connection
                )

            finally:
                connection.close()

    def _synchronise_static_user(
        self,
        connection: sqlite3.Connection,
    ) -> None:
        """Create or update the configured production user."""

        username = _normalise_username(
            settings.static_username
        )

        now = _epoch_now()

        row = connection.execute(
            """
            SELECT
                id,
                password_hash,
                active
            FROM users
            WHERE username = ?
            """,
            (username,),
        ).fetchone()

        if row is None:
            password_hash = (
                self._password_hasher.hash(
                    settings.static_password
                )
            )

            connection.execute(
                """
                INSERT INTO users (
                    username,
                    password_hash,
                    active,
                    created_at,
                    updated_at
                )
                VALUES (?, ?, 1, ?, ?)
                """,
                (
                    username,
                    password_hash,
                    now,
                    now,
                ),
            )

            return

        password_matches = False

        try:
            password_matches = (
                self._password_hasher.verify(
                    row["password_hash"],
                    settings.static_password,
                )
            )

        except (
            VerifyMismatchError,
            VerificationError,
            InvalidHashError,
        ):
            password_matches = False

        password_needs_rehash = False

        if password_matches:
            try:
                password_needs_rehash = (
                    self._password_hasher
                    .check_needs_rehash(
                        row["password_hash"]
                    )
                )

            except InvalidHashError:
                password_needs_rehash = True

        if (
            not password_matches
            or password_needs_rehash
            or int(row["active"]) != 1
        ):
            new_hash = self._password_hasher.hash(
                settings.static_password
            )

            connection.execute(
                """
                UPDATE users
                SET
                    password_hash = ?,
                    active = 1,
                    updated_at = ?
                WHERE id = ?
                """,
                (
                    new_hash,
                    now,
                    int(row["id"]),
                ),
            )

            # Password changes invalidate existing sessions.
            if not password_matches:
                connection.execute(
                    """
                    UPDATE sessions
                    SET revoked_at = ?
                    WHERE
                        user_id = ?
                        AND revoked_at IS NULL
                    """,
                    (
                        now,
                        int(row["id"]),
                    ),
                )

    # ==========================================================
    # Cleanup and rate limiting
    # ==========================================================

    def _cleanup_expired_data(
        self,
        connection: sqlite3.Connection,
    ) -> None:
        now = _epoch_now()

        failure_cutoff = (
            now
            - max(
                settings.login_failure_window_seconds,
                settings.login_lockout_seconds,
            )
            - 60
        )

        session_cutoff = (
            now
            - 7 * 24 * 60 * 60
        )

        connection.execute(
            """
            DELETE FROM login_failures
            WHERE failed_at < ?
            """,
            (failure_cutoff,),
        )

        connection.execute(
            """
            DELETE FROM sessions
            WHERE
                expires_at < ?
                OR (
                    revoked_at IS NOT NULL
                    AND revoked_at < ?
                )
            """,
            (
                now,
                session_cutoff,
            ),
        )

    def _check_login_allowed(
        self,
        connection: sqlite3.Connection,
        *,
        username: str,
        client_ip: str,
    ) -> None:
        now = _epoch_now()

        window_start = (
            now
            - settings.login_failure_window_seconds
        )

        row = connection.execute(
            """
            SELECT
                COUNT(*) AS failure_count,
                MAX(failed_at) AS latest_failure
            FROM login_failures
            WHERE
                username = ?
                AND client_ip = ?
                AND failed_at >= ?
            """,
            (
                username,
                client_ip,
                window_start,
            ),
        ).fetchone()

        failure_count = int(
            row["failure_count"] or 0
        )

        latest_failure = int(
            row["latest_failure"] or 0
        )

        if (
            failure_count
            < settings.login_failure_limit
        ):
            return

        unlock_at = (
            latest_failure
            + settings.login_lockout_seconds
        )

        if now >= unlock_at:
            connection.execute(
                """
                DELETE FROM login_failures
                WHERE
                    username = ?
                    AND client_ip = ?
                """,
                (
                    username,
                    client_ip,
                ),
            )

            return

        retry_after = max(
            1,
            unlock_at - now,
        )

        raise HTTPException(
            status_code=429,
            detail=(
                "Too many failed login attempts. "
                f"Try again after {retry_after} seconds."
            ),
            headers={
                "Retry-After": str(
                    retry_after
                )
            },
        )

    def _record_login_failure(
        self,
        connection: sqlite3.Connection,
        *,
        username: str,
        client_ip: str,
    ) -> None:
        connection.execute(
            """
            INSERT INTO login_failures (
                username,
                client_ip,
                failed_at
            )
            VALUES (?, ?, ?)
            """,
            (
                username,
                client_ip,
                _epoch_now(),
            ),
        )

    def _clear_login_failures(
        self,
        connection: sqlite3.Connection,
        *,
        username: str,
        client_ip: str,
    ) -> None:
        connection.execute(
            """
            DELETE FROM login_failures
            WHERE
                username = ?
                AND client_ip = ?
            """,
            (
                username,
                client_ip,
            ),
        )

    # ==========================================================
    # Authentication
    # ==========================================================

    def login(
        self,
        *,
        username: str,
        password: str,
        client_ip: str,
        user_agent: str,
    ) -> dict[str, Any]:
        normalised_username = (
            _normalise_username(
                username
            )
        )

        if not normalised_username:
            raise HTTPException(
                status_code=401,
                detail="Invalid username or password.",
            )

        with self._lock:
            connection = self._connect()

            try:
                connection.execute(
                    "BEGIN IMMEDIATE"
                )

                self._cleanup_expired_data(
                    connection
                )

                self._check_login_allowed(
                    connection,
                    username=normalised_username,
                    client_ip=client_ip,
                )

                user = connection.execute(
                    """
                    SELECT
                        id,
                        username,
                        password_hash,
                        active
                    FROM users
                    WHERE username = ?
                    """,
                    (
                        normalised_username,
                    ),
                ).fetchone()

                password_valid = False

                if (
                    user is not None
                    and int(user["active"]) == 1
                ):
                    try:
                        password_valid = (
                            self._password_hasher.verify(
                                user["password_hash"],
                                password,
                            )
                        )

                    except (
                        VerifyMismatchError,
                        VerificationError,
                        InvalidHashError,
                    ):
                        password_valid = False

                if not password_valid:
                    self._record_login_failure(
                        connection,
                        username=normalised_username,
                        client_ip=client_ip,
                    )

                    connection.execute(
                        "COMMIT"
                    )

                    raise HTTPException(
                        status_code=401,
                        detail=(
                            "Invalid username or password."
                        ),
                    )

                self._clear_login_failures(
                    connection,
                    username=normalised_username,
                    client_ip=client_ip,
                )

                user_id = int(
                    user["id"]
                )

                now = _epoch_now()

                # Keep only the newest allowed sessions.
                active_sessions = (
                    connection.execute(
                        """
                        SELECT session_id
                        FROM sessions
                        WHERE
                            user_id = ?
                            AND revoked_at IS NULL
                            AND expires_at > ?
                        ORDER BY created_at DESC
                        """,
                        (
                            user_id,
                            now,
                        ),
                    ).fetchall()
                )

                sessions_to_revoke = active_sessions[
                    max(
                        0,
                        settings.maximum_active_sessions
                        - 1
                    ):
                ]

                for session in sessions_to_revoke:
                    connection.execute(
                        """
                        UPDATE sessions
                        SET revoked_at = ?
                        WHERE session_id = ?
                        """,
                        (
                            now,
                            session["session_id"],
                        ),
                    )

                raw_token = secrets.token_urlsafe(
                    48
                )

                session_id = secrets.token_hex(
                    16
                )

                expires_at = (
                    now
                    + settings.session_ttl_seconds
                )

                connection.execute(
                    """
                    INSERT INTO sessions (
                        session_id,
                        token_hash,
                        user_id,
                        created_at,
                        expires_at,
                        last_seen_at,
                        revoked_at,
                        client_ip,
                        user_agent
                    )
                    VALUES (?, ?, ?, ?, ?, ?, NULL, ?, ?)
                    """,
                    (
                        session_id,
                        _token_hash(
                            raw_token
                        ),
                        user_id,
                        now,
                        expires_at,
                        now,
                        client_ip[:128],
                        user_agent[:512],
                    ),
                )

                connection.execute(
                    "COMMIT"
                )

                return {
                    "success": True,
                    "token": raw_token,
                    "token_type": "Bearer",
                    "expires_at": (
                        _epoch_to_iso(
                            expires_at
                        )
                    ),
                    "session_id": session_id,
                    "user": {
                        "username": (
                            user["username"]
                        ),
                    },
                    "rover": {
                        "id": settings.rover_id,
                        "name": settings.rover_name,
                    },
                }

            except HTTPException:
                if connection.in_transaction:
                    connection.execute(
                        "ROLLBACK"
                    )

                raise

            except sqlite3.Error as error:
                if connection.in_transaction:
                    connection.execute(
                        "ROLLBACK"
                    )

                raise HTTPException(
                    status_code=503,
                    detail=(
                        "Authentication database "
                        "is unavailable."
                    ),
                ) from error

            finally:
                connection.close()

    def authenticate_token(
        self,
        token: str,
        *,
        touch: bool = True,
    ) -> AuthenticatedSession | None:
        if not token:
            return None

        hashed_token = _token_hash(
            token
        )

        now = _epoch_now()

        with self._lock:
            connection = self._connect()

            try:
                row = connection.execute(
                    """
                    SELECT
                        sessions.session_id,
                        sessions.created_at,
                        sessions.expires_at,
                        sessions.last_seen_at,
                        sessions.client_ip,
                        sessions.user_agent,
                        users.username,
                        users.active
                    FROM sessions
                    INNER JOIN users
                        ON users.id = sessions.user_id
                    WHERE
                        sessions.token_hash = ?
                        AND sessions.revoked_at IS NULL
                        AND sessions.expires_at > ?
                    """,
                    (
                        hashed_token,
                        now,
                    ),
                ).fetchone()

                if row is None:
                    return None

                if int(row["active"]) != 1:
                    return None

                if (
                    touch
                    and now
                    - int(row["last_seen_at"])
                    >= 30
                ):
                    connection.execute(
                        """
                        UPDATE sessions
                        SET last_seen_at = ?
                        WHERE session_id = ?
                        """,
                        (
                            now,
                            row["session_id"],
                        ),
                    )

                return AuthenticatedSession(
                    session_id=(
                        row["session_id"]
                    ),
                    username=(
                        row["username"]
                    ),
                    created_at=(
                        _epoch_to_iso(
                            int(
                                row[
                                    "created_at"
                                ]
                            )
                        )
                    ),
                    expires_at=(
                        _epoch_to_iso(
                            int(
                                row[
                                    "expires_at"
                                ]
                            )
                        )
                    ),
                    client_ip=(
                        row["client_ip"]
                    ),
                    user_agent=(
                        row["user_agent"]
                    ),
                )

            finally:
                connection.close()

    def session_is_active(
        self,
        token: str,
    ) -> bool:
        return (
            self.authenticate_token(
                token,
                touch=False,
            )
            is not None
        )

    def logout(
        self,
        token: str,
    ) -> bool:
        if not token:
            return False

        hashed_token = _token_hash(
            token
        )

        revoked_session_id: str | None = None

        with self._lock:
            connection = self._connect()

            try:
                now = _epoch_now()

                row = connection.execute(
                    """
                    SELECT session_id
                    FROM sessions
                    WHERE
                        token_hash = ?
                        AND revoked_at IS NULL
                    """,
                    (
                        hashed_token,
                    ),
                ).fetchone()

                if row is None:
                    return False

                revoked_session_id = (
                    row["session_id"]
                )

                connection.execute(
                    """
                    UPDATE sessions
                    SET revoked_at = ?
                    WHERE session_id = ?
                    """,
                    (
                        now,
                        revoked_session_id,
                    ),
                )

            finally:
                connection.close()

        if revoked_session_id is not None:
            self._notify_revocation(
                revoked_session_id
            )

        return True

    # ==========================================================
    # Socket.IO revocation integration
    # ==========================================================

    def register_revocation_callback(
        self,
        callback: Callable[[str], None],
    ) -> None:
        with self._lock:
            if (
                callback
                not in self._revocation_callbacks
            ):
                self._revocation_callbacks.append(
                    callback
                )

    def _notify_revocation(
        self,
        session_id: str,
    ) -> None:
        with self._lock:
            callbacks = list(
                self._revocation_callbacks
            )

        for callback in callbacks:
            try:
                callback(
                    session_id
                )

            except Exception:
                # Logout must succeed even when a realtime
                # disconnect callback fails.
                continue


authentication_store = AuthenticationStore(
    settings.database_file
)


def require_auth(
    request: Request,
) -> AuthenticatedSession:
    """FastAPI dependency requiring a valid persistent session."""

    token = extract_request_token(
        request
    )

    if token is None:
        raise HTTPException(
            status_code=401,
            detail="Authentication token required.",
            headers={
                "WWW-Authenticate": "Bearer"
            },
        )

    session = (
        authentication_store
        .authenticate_token(token)
    )

    if session is None:
        raise HTTPException(
            status_code=401,
            detail=(
                "Session is invalid, expired "
                "or logged out."
            ),
            headers={
                "WWW-Authenticate": "Bearer"
            },
        )

    return session


auth_router = APIRouter(
    prefix="/api/auth",
    tags=["authentication"],
)


@auth_router.post(
    "/login",
)
def login(
    body: LoginRequest,
    request: Request,
) -> dict[str, Any]:
    return authentication_store.login(
        username=body.username,
        password=body.password,
        client_ip=_request_ip(
            request
        ),
        user_agent=request.headers.get(
            "User-Agent",
            "unknown",
        ),
    )


@auth_router.get(
    "/session",
)
def read_session(
    session: AuthenticatedSession = Depends(
        require_auth
    ),
) -> dict[str, Any]:
    return {
        "authenticated": True,
        "session": {
            "session_id": (
                session.session_id
            ),
            "username": (
                session.username
            ),
            "created_at": (
                session.created_at
            ),
            "expires_at": (
                session.expires_at
            ),
        },
        "rover": {
            "id": settings.rover_id,
            "name": settings.rover_name,
        },
    }


@auth_router.post(
    "/logout",
    response_model=LogoutResponse,
)
def logout(
    request: Request,
    _session: AuthenticatedSession = Depends(
        require_auth
    ),
) -> LogoutResponse:
    token = extract_request_token(
        request
    )

    if token is None:
        raise HTTPException(
            status_code=401,
            detail="Authentication token required.",
        )

    authentication_store.logout(
        token
    )

    return LogoutResponse(
        success=True,
        message="Logged out successfully.",
    )