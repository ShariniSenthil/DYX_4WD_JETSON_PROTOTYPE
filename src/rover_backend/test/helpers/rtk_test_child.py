#!/usr/bin/env python3
"""Synthetic non-ROS child used only by RTK process-adapter tests."""

from __future__ import annotations

import argparse
import os
import signal
import sys
import time
from pathlib import Path


# Make the source package importable when this file is executed directly.
PACKAGE_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PACKAGE_ROOT))

from rover_backend.rtk_process_protocol import (  # noqa: E402
    WORKER_STATUS_SCHEMA_VERSION,
    WorkerStatusEvent,
    WorkerStatusKind,
    decode_worker_config,
    encode_worker_status,
    read_bounded_fd,
    write_all_fd,
)


def _arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config-fd", required=True, type=int)
    parser.add_argument("--liveness-fd", required=True, type=int)
    parser.add_argument("--status-fd", required=True, type=int)
    parser.add_argument(
        "--status-mode",
        choices=("separate", "combined", "fragmented"),
        default="separate",
    )
    parser.add_argument("--status-run-id")
    parser.add_argument("--ignore-sigterm", action="store_true")
    parser.add_argument("--exit-after-status", action="store_true")
    parser.add_argument("--exit-code", type=int, default=0)
    return parser.parse_args()


def _write_statuses(
    fd: int,
    run_id: str,
    mode: str,
) -> None:
    started = encode_worker_status(
        WorkerStatusEvent(
            WORKER_STATUS_SCHEMA_VERSION,
            run_id,
            WorkerStatusKind.STARTED,
            "CONFIG_DECODED",
        )
    )
    ready = encode_worker_status(
        WorkerStatusEvent(
            WORKER_STATUS_SCHEMA_VERSION,
            run_id,
            WorkerStatusKind.READY,
            "SYNTHETIC_READY",
        )
    )
    if mode == "combined":
        write_all_fd(fd, started + ready)
    elif mode == "fragmented":
        split = max(1, len(ready) // 2)
        write_all_fd(fd, started + ready[:split])
        time.sleep(0.05)
        write_all_fd(fd, ready[split:])
    else:
        write_all_fd(fd, started)
        write_all_fd(fd, ready)


def main() -> int:
    args = _arguments()
    if args.ignore_sigterm:
        signal.signal(signal.SIGTERM, signal.SIG_IGN)

    try:
        encoded_config = read_bounded_fd(args.config_fd)
    finally:
        os.close(args.config_fd)
    config = decode_worker_config(encoded_config)
    del encoded_config

    status_run_id = args.status_run_id or config.run_id
    try:
        _write_statuses(args.status_fd, status_run_id, args.status_mode)
    finally:
        os.close(args.status_fd)

    if args.exit_after_status:
        return args.exit_code

    # This read blocks without threads and observes parent authority loss as
    # EOF.  Normal test shutdown reaches here through SIGTERM/SIGKILL instead.
    while True:
        try:
            chunk = os.read(args.liveness_fd, 1)
        except InterruptedError:
            continue
        if not chunk:
            os.close(args.liveness_fd)
            return args.exit_code


if __name__ == "__main__":
    raise SystemExit(main())
