#!/usr/bin/env python3

"""Validated runtime configuration for the DYX rover backend.

The production systemd service loads environment variables from:

    /etc/dyx-rover-backend.env

Secrets therefore remain outside the ROS workspace and outside Git.
"""

from __future__ import annotations

import ipaddress
import os

from dataclasses import dataclass
from pathlib import Path
from typing import Final


_ALLOWED_LOG_LEVELS: Final = {
    "critical",
    "error",
    "warning",
    "info",
    "debug",
}

_DEFAULT_PRIVATE_SUBNETS: Final = (
    "127.0.0.0/8",
    "192.168.3.0/24",
)


def _read_text(
    name: str,
    default: str,
    *,
    allow_empty: bool = False,
) -> str:
    """Read and validate a text environment variable."""

    value = os.getenv(name, default).strip()

    if not value and not allow_empty:
        raise RuntimeError(f"{name} must not be empty")

    return value


def _read_int(
    name: str,
    default: int,
    *,
    minimum: int | None = None,
    maximum: int | None = None,
) -> int:
    """Read an integer environment variable with limits."""

    raw_value = os.getenv(name)

    try:
        value = (
            default
            if raw_value is None
            else int(raw_value)
        )
    except ValueError as error:
        raise RuntimeError(
            f"{name} must be an integer"
        ) from error

    if minimum is not None and value < minimum:
        raise RuntimeError(
            f"{name} must be at least {minimum}"
        )

    if maximum is not None and value > maximum:
        raise RuntimeError(
            f"{name} must be at most {maximum}"
        )

    return value


def _read_float(
    name: str,
    default: float,
    *,
    minimum: float = 0.0,
) -> float:
    """Read a decimal environment variable with a minimum."""

    raw_value = os.getenv(name)

    try:
        value = (
            default
            if raw_value is None
            else float(raw_value)
        )
    except ValueError as error:
        raise RuntimeError(
            f"{name} must be a number"
        ) from error

    if value < minimum:
        raise RuntimeError(
            f"{name} must be at least {minimum}"
        )

    return value


def _read_path(
    name: str,
    default: Path,
) -> Path:
    """Read and normalize a filesystem path."""

    raw_value = os.getenv(name)

    path = (
        Path(raw_value).expanduser()
        if raw_value
        else default.expanduser()
    )

    return path.resolve(strict=False)


def _read_networks(
    name: str = "DYX_ALLOWED_SUBNETS",
) -> tuple[
    ipaddress.IPv4Network | ipaddress.IPv6Network,
    ...,
]:
    """Read the tablet networks allowed to access the backend."""

    raw_value = os.getenv(
        name,
        ",".join(_DEFAULT_PRIVATE_SUBNETS),
    )

    networks: list[
        ipaddress.IPv4Network | ipaddress.IPv6Network
    ] = []

    for subnet_value in raw_value.split(","):
        subnet_text = subnet_value.strip()

        if not subnet_text:
            continue

        try:
            network = ipaddress.ip_network(
                subnet_text,
                strict=False,
            )
        except ValueError as error:
            raise RuntimeError(
                f"Invalid network in {name}: {subnet_text}"
            ) from error

        networks.append(network)

    if not networks:
        raise RuntimeError(
            f"{name} must contain at least one network"
        )

    return tuple(networks)


def _read_cors_origins() -> tuple[str, ...]:
    """Read optional browser origins.

    Native React Native requests do not require CORS. This is retained
    only for controlled browser-based development.
    """

    raw_value = os.getenv(
        "DYX_CORS_ORIGINS",
        "",
    )

    return tuple(
        origin.strip()
        for origin in raw_value.split(",")
        if origin.strip()
    )


@dataclass(
    frozen=True,
    slots=True,
)
class Settings:
    """Immutable validated backend configuration."""

    app_version: str

    rover_id: str
    rover_name: str
    rover_ip: str

    backend_host: str
    backend_port: int
    log_level: str

    cors_origins: tuple[str, ...]

    allowed_subnets: tuple[
        ipaddress.IPv4Network | ipaddress.IPv6Network,
        ...,
    ]

    static_username: str
    static_password: str

    session_ttl_seconds: int
    maximum_active_sessions: int

    database_file: Path

    mission_file: Path
    mission_metadata_file: Path
    runtime_directory: Path

    maximum_upload_bytes: int
    extension_trigger_distance_m: float

    telemetry_broadcast_hz: float
    socket_path: str

    @property
    def application_name(self) -> str:
        """Human-readable API application name."""

        return "DYX 4WD Rover Backend"

    @property
    def application_version(self) -> str:
        """Application version expected by the REST API."""

        return self.app_version

    @property
    def service_name(self) -> str:
        """ROS and backend service identifier."""

        return "rover_backend"

    @property
    def backend_heartbeat_hz(self) -> float:
        """Rate used to publish the backend safety heartbeat."""

        return 2.0

    @property
    def marking_hold_seconds(self) -> float:
        """Legacy alias for the marking arrival-settle window."""

        # The 3-second interval belongs to spray_controller's physical spray
        # hold. Mission arrival validation is intentionally only 0.30 s.
        return 0.30

    @property
    def mission_runtime_file(self) -> Path:
        """Separate runtime-state persistence file."""

        return (
            self.runtime_directory
            / "mission_runtime.json"
        )

    @property
    def login_failure_limit(self) -> int:
        """Maximum failed login attempts within the failure window."""
        return 5

    @property
    def login_failure_window_seconds(self) -> int:
        """Time window used to count failed login attempts."""
        return 300

    @property
    def login_lockout_seconds(self) -> int:
        """Temporary login lockout duration after repeated failures."""
        return 900

    @property
    def maximum_marking_points(self) -> int:
        """Maximum number of marking points accepted in one mission."""
        return 10000


    @property
    def row_transition_threshold_m(self) -> float:
        """Distance threshold used to detect short row transitions."""
        return self.extension_trigger_distance_m


    @property
    def default_dummy_point_distance_m(self) -> float:
        """Default navigation-only dummy-point distance."""
        return 3.5

    @property
    def trajectory_prepare_timeout_seconds(self) -> float:
        """Maximum time allowed for trajectory preparation."""
        return 30.0


def load_settings() -> Settings:
    """Load all configuration and create required data folders."""

    home_directory = Path.home()

    data_directory = _read_path(
        "DYX_DATA_DIRECTORY",
        home_directory
        / ".local"
        / "share"
        / "dyx_rover",
    )

    runtime_directory = _read_path(
        "DYX_RUNTIME_DIRECTORY",
        data_directory / "runtime",
    )

    mission_file = _read_path(
        "DYX_MISSION_FILE",
        home_directory
        / "rover_ws"
        / "missions"
        / "mission.csv",
    )

    database_file = _read_path(
        "DYX_DATABASE_FILE",
        data_directory / "backend.sqlite3",
    )

    username = _read_text(
        "DYX_STATIC_USERNAME",
        "admin",
    )

    password = _read_text(
        "DYX_STATIC_PASSWORD",
        "dyx@2026",
    )

    if len(password) < 8:
        raise RuntimeError(
            "DYX_STATIC_PASSWORD must contain "
            "at least 8 characters"
        )

    log_level = _read_text(
        "DYX_LOG_LEVEL",
        "info",
    ).lower()

    if log_level not in _ALLOWED_LOG_LEVELS:
        allowed_values = ", ".join(
            sorted(_ALLOWED_LOG_LEVELS)
        )

        raise RuntimeError(
            "DYX_LOG_LEVEL must be one of: "
            f"{allowed_values}"
        )

    socket_path = _read_text(
        "DYX_SOCKET_PATH",
        "/socket.io",
    )

    if not socket_path.startswith("/"):
        socket_path = f"/{socket_path}"

    socket_path = (
        socket_path.rstrip("/")
        or "/socket.io"
    )

    settings = Settings(
        app_version="2.0.0",

        rover_id=_read_text(
            "DYX_ROVER_ID",
            "dyx-4wd-001",
        ),

        rover_name=_read_text(
            "DYX_ROVER_NAME",
            "DYX 4WD Rover",
        ),

        rover_ip=_read_text(
            "DYX_ROVER_IP",
            "192.168.3.101",
        ),

        backend_host=_read_text(
            "DYX_BACKEND_HOST",
            "0.0.0.0",
        ),

        backend_port=_read_int(
            "DYX_BACKEND_PORT",
            5001,
            minimum=1,
            maximum=65535,
        ),

        log_level=log_level,

        cors_origins=_read_cors_origins(),

        allowed_subnets=_read_networks(),

        static_username=username,

        static_password=password,

        # Ten years by default. This provides persistent tablet
        # login while retaining expires_at and ttl_s in the API.
        session_ttl_seconds=_read_int(
            "DYX_SESSION_TTL_SECONDS",
            315_360_000,
            minimum=3_600,
        ),

        maximum_active_sessions=_read_int(
            "DYX_MAXIMUM_ACTIVE_SESSIONS",
            5,
            minimum=1,
            maximum=50,
        ),

        database_file=database_file,

        mission_file=mission_file,

        mission_metadata_file=_read_path(
            "DYX_MISSION_METADATA_FILE",
            runtime_directory
            / "mission_metadata.json",
        ),

        runtime_directory=runtime_directory,

        maximum_upload_bytes=_read_int(
            "DYX_MAXIMUM_UPLOAD_BYTES",
            5 * 1024 * 1024,
            minimum=1_024,
            maximum=50 * 1024 * 1024,
        ),

        extension_trigger_distance_m=_read_float(
            "DYX_EXTENSION_TRIGGER_DISTANCE_M",
            2.0,
            minimum=0.1,
        ),

        telemetry_broadcast_hz=_read_float(
            "DYX_TELEMETRY_BROADCAST_HZ",
            5.0,
            minimum=0.5,
        ),

        socket_path=socket_path,
    )

    required_directories = {
        settings.database_file.parent,
        settings.mission_file.parent,
        settings.mission_metadata_file.parent,
        settings.runtime_directory,
    }

    for directory in required_directories:
        directory.mkdir(
            parents=True,
            exist_ok=True,
            mode=0o700,
        )

    return settings


def client_ip_is_allowed(
    client_ip_text: str | None,
) -> bool:
    """Check whether a tablet address belongs to an allowed network."""

    if not client_ip_text:
        return False

    try:
        client_ip = ipaddress.ip_address(
            client_ip_text
        )
    except ValueError:
        return False

    return any(
        client_ip in allowed_network
        for allowed_network
        in settings.allowed_subnets
    )


settings = load_settings()
