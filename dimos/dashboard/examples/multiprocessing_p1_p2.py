#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Example that runs the dashboard bootstrap and color-image replay in separate
processes. One process mirrors dimos/dashboard/examples/p1.py and the other
process mirrors the logging loop from dimos/dashboard/examples/p2.py.
"""

import dataclasses
import multiprocessing as mp
import os
from pathlib import Path
import sys
import threading
import time

import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb
import yaml

from dimos.dashboard.server import start_dashboard_server_thread

default_rerun_grpc_port = 9876


@dataclasses.dataclass
class RerunInfo:
    logging_id: str = os.environ.get("RERUN_ID", "dimos_main_rerun")
    grpc_port: int = int(os.environ.get("RERUN_GRPC_PORT", default_rerun_grpc_port))
    server_memory_limit: str = os.environ.get("RERUN_SERVER_MEMORY_LIMIT", "25%")
    url: str = os.environ.get(
        "RERUN_URL",
        f"rerun+http://localhost:{os.environ.get('RERUN_GRPC_PORT', default_rerun_grpc_port)!s}/proxy",
    )


rerun_info = RerunInfo()


class RerunConnection:
    def __init__(self) -> None:
        self.init_id = mp.current_process().pid
        self.stream = rr.RecordingStream(rerun_info.logging_id, recording_id=rerun_info.logging_id)
        self.stream.connect_grpc(rerun_info.url)

    def log(self, msg: str, value, **kwargs) -> None:
        if self.init_id != mp.current_process().pid:
            raise Exception(
                """Looks like you are somehow using RerunConnection to log data to rerun. However, the process/thread where you init RerunConnection is different from where you are logging. A RerunConnection object needs to be created once per process/thread."""
            )

        self.stream.log(msg, value, **kwargs)


def dashboard_process(stop_event: mp.Event) -> None:
    """Mirrors p1.py: init rerun, send blueprint, start grpc and dashboard server."""
    print("[DashboardProc] calling rr.init")
    rr.init(rerun_info.logging_id, spawn=False, recording_id=rerun_info.logging_id)
    default_blueprint = rrb.Blueprint(
        rrb.Tabs(
            rrb.Spatial3DView(
                name="Spatial3D",
                origin="/",
                line_grid=rrb.LineGrid3D(spacing=1.0, stroke_width=1.0),
            ),
        )
    )
    print("[DashboardProc] sending empty blueprint")
    rr.send_blueprint(default_blueprint)
    if not os.environ.get("RERUN_URL", None):
        print("[DashboardProc] starting rerun grpc")
        rr.serve_grpc(
            grpc_port=rerun_info.grpc_port,
            default_blueprint=default_blueprint,
            server_memory_limit=rerun_info.server_memory_limit,
        )
    thread = start_dashboard_server_thread(
        **{
            "auto_open": True,
            "terminal_commands": {"agent-spy": "htop", "lcm-spy": "dimos lcmspy"},
            "worker": 1,
        },
        rrd_url=rerun_info.url,
        keep_alive=True,
    )
    try:
        while not stop_event.is_set():
            time.sleep(0.25)
    except KeyboardInterrupt:
        print("[DashboardProc] interrupted, shutting down")
    finally:
        # Allow a brief moment for the server thread to respond before exit.
        thread.join(timeout=1)


def _log_stream(name: str, path: Path, start_event: threading.Event) -> None:
    rc = RerunConnection()
    if not path.exists():
        print(f"[LoggingProc-{name}] file {path} does not exist", file=sys.stderr)
        return

    start_event.wait()

    with path.open("r", encoding="utf-8") as f:
        for line_number, line in enumerate(f):
            if not line.strip():
                continue
            try:
                parsed = yaml.unsafe_load(line) or []
            except Exception as error:
                print(
                    f"[LoggingProc-{name}] warning: line:{line_number} could not be parsed: {error}"
                )
                continue

            try:
                payload = parsed[0] if isinstance(parsed, list) else parsed
                print(f"[LoggingProc-{name}] logging message {line_number}")
                rc.log(f"/{name}", payload.to_rerun(), strict=True)
            except Exception as error:
                print(f"[LoggingProc-{name}] error: {error}")


def logging_process() -> None:
    """Mirrors the logging loop in p2.py: replay color_image.yaml and lidar.yaml concurrently."""
    start_event = threading.Event()
    files = {
        "color_image": Path("./dimos/dashboard/support/color_image.yaml"),
        "lidar": Path("./dimos/dashboard/support/lidar.yaml"),
    }

    threads = []
    for name, path in files.items():
        t = threading.Thread(target=_log_stream, args=(name, path, start_event), name=f"log-{name}")
        threads.append(t)
        t.start()

    # Kick off both loggers at roughly the same time.
    start_event.set()

    for t in threads:
        t.join()


def main() -> None:
    mp.set_start_method("spawn", force=True)
    stop_event = mp.Event()

    dashboard_proc = mp.Process(
        target=dashboard_process, args=(stop_event,), name="p1_like_dashboard"
    )
    logger_proc = mp.Process(target=logging_process, name="p2_like_replay")

    dashboard_proc.start()
    # Give the dashboard/grpc server a moment to spin up before logging.
    time.sleep(1.5)
    logger_proc.start()

    logger_proc.join()
    stop_event.set()
    dashboard_proc.join(timeout=5)
    if dashboard_proc.is_alive():
        print("[Main] dashboard process still running, terminating")
        dashboard_proc.terminate()
        dashboard_proc.join()


if __name__ == "__main__":
    main()
