#!/usr/bin/env python3
"""Permanent, reusable MAVLink-FTP file puller via MAVROS's FTP plugin.

Downloads a file from the FCU's SD card (e.g. a currently-open PX4 ULog at
/fs/microsd/log/<date>/<time>.ulg) over the existing MAVLink connection, using
MAVROS's own /mavros/ftp/{open,read,close} services -- no hand-rolled MAVLink
FTP protocol needed, MAVROS already implements it.

Safe to run while armed: this only reads a file, it does not touch flight
control in any way.

Usage (from the Jetson, with the rover stack running):

    source /opt/ros/humble/setup.bash
    python3 scripts/mavros_ftp_pull.py /fs/microsd/log/2026-09-08/06_59_35.ulg \
        /home/flash/pulled_logs/06_59_35.ulg

Downloading a file that the logger is still actively writing is fine -- this
pulls exactly the byte range that existed on the remote when the download
started (the FileOpen response's reported size), so it is a consistent
snapshot even if the file keeps growing afterward.
"""
import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from mavros_msgs.srv import FileOpen, FileRead, FileClose

CHUNK = 4096


def pull(remote_path, local_path, progress_every=2.0):
    rclpy.init()
    node = Node("mavros_ftp_pull")
    open_cli = node.create_client(FileOpen, "/mavros/ftp/open")
    read_cli = node.create_client(FileRead, "/mavros/ftp/read")
    close_cli = node.create_client(FileClose, "/mavros/ftp/close")

    for name, cli in (("open", open_cli), ("read", read_cli), ("close", close_cli)):
        if not cli.wait_for_service(timeout_sec=8.0):
            print(f"ABORT: /mavros/ftp/{name} service not available -- is MAVROS running?")
            return 1

    req = FileOpen.Request(file_path=remote_path, mode=FileOpen.Request.MODE_READ)
    fut = open_cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    res = fut.result()
    if res is None or not res.success:
        print(f"ABORT: open failed for {remote_path} (r_errno={getattr(res, 'r_errno', '?')})")
        return 1
    size = res.size
    print(f"opened {remote_path}  size={size} bytes ({size/1e6:.2f} MB)")

    os.makedirs(os.path.dirname(os.path.abspath(local_path)) or ".", exist_ok=True)
    offset = 0
    last_report = time.monotonic()
    t0 = time.monotonic()
    with open(local_path, "wb") as f:
        while offset < size:
            want = min(CHUNK, size - offset)
            rreq = FileRead.Request(file_path=remote_path, offset=offset, size=want)
            rfut = read_cli.call_async(rreq)
            rclpy.spin_until_future_complete(node, rfut, timeout_sec=15.0)
            rres = rfut.result()
            if rres is None or not rres.success:
                print(f"\nABORT: read failed at offset={offset} (r_errno={getattr(rres, 'r_errno', '?')})")
                break
            data = bytes(rres.data)
            if not data:
                break
            f.write(data)
            offset += len(data)
            now = time.monotonic()
            if now - last_report >= progress_every:
                rate = offset / max(now - t0, 1e-6)
                print(f"  {offset}/{size} bytes ({100*offset/size:.1f}%)  {rate/1024:.1f} KiB/s", flush=True)
                last_report = now

    close_cli.call_async(FileClose.Request(file_path=remote_path))
    elapsed = time.monotonic() - t0
    print(f"done: wrote {offset} bytes to {local_path} in {elapsed:.1f}s ({offset/max(elapsed,1e-6)/1024:.1f} KiB/s avg)")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if offset >= size else 2


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("remote_path", help="Path on the FCU SD card, e.g. /fs/microsd/log/2026-09-08/06_59_35.ulg")
    ap.add_argument("local_path", help="Where to write the downloaded file on the Jetson")
    args = ap.parse_args()
    sys.exit(pull(args.remote_path, args.local_path))


if __name__ == "__main__":
    main()
