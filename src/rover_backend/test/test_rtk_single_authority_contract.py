"""Static contract tests for backend-only RTK production ownership."""

from pathlib import Path


REPO_ROOT = (
    Path(__file__).resolve().parents[3]
)


def _read(
    relative_path: str,
) -> str:
    return (
        REPO_ROOT
        / relative_path
    ).read_text(
        encoding="utf-8"
    )


def test_start_rtk_script_is_api_only():
    source = _read(
        "scripts/start_rtk.sh"
    )

    assert "/api/rtk/start" in source
    assert "X-Rover-Token" in source

    assert "ros2 run" not in source
    assert "ntrip_to_px4_node" not in source

    assert "NTRIP_MOUNTPOINT" not in source
    assert "NTRIP_USERNAME" not in source
    assert "NTRIP_PASSWORD" not in source


def test_ros_package_exposes_no_standalone_console_script():
    source = _read(
        "src/rtk_correction_bridge/setup.py"
    )

    assert (
        "ntrip_to_px4_node:main"
        not in source
    )

    assert (
        "console_scripts"
        not in source
    )


def test_stale_direct_entrypoint_fails_closed():
    source = _read(
        "src/rtk_correction_bridge/"
        "rtk_correction_bridge/"
        "ntrip_to_px4_node.py"
    )

    main_source = source[
        source.index(
            "def main(args=None):"
        ):
    ]

    assert (
        "Standalone RTK launch is disabled"
        in main_source
    )

    assert (
        "Use rover_backend /api/rtk/start."
        in main_source
    )

    assert "rclpy.init(" not in main_source
    assert "NtripToPx4Node()" not in main_source


def test_production_launch_declares_backend_owned_rtk():
    source = _read(
        "src/rover_bringup/launch/"
        "rover.launch.py"
    )

    assert (
        "RTK correction ownership is "
        "backend-managed"
        in source
    )

    assert (
        "started separately using start_rtk.sh"
        not in source
    )


def test_worker_requires_backend_owned_worker_config():
    source = _read(
        "src/rtk_correction_bridge/"
        "rtk_correction_bridge/"
        "ntrip_to_px4_node.py"
    )

    constructor = source[
        source.index(
            "def __init__(self, worker_config=None):"
        ):
        source.index(
            "def _validate_parameters(",
        )
    ]

    assert "if worker_config is None:" in constructor
    assert "worker_config is required" in constructor

    # No credential-bearing ROS/environment configuration path remains.
    assert "NTRIP_PASSWORD" not in constructor
    assert "'password'," not in constructor
    assert "'caster_host'," not in constructor
    assert "'mountpoint'," not in constructor


def test_no_stale_direct_ntrip_backup_remains():
    stale = (
        REPO_ROOT
        / "src/rtk_correction_bridge/"
        "rtk_correction_bridge/"
        "ntrip_to_px4_node.py.backup_20260710_165936"
    )

    assert not stale.exists()
