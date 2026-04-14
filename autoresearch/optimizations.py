# Copyright 2026 Dimensional Inc.
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

"""Optimizations for Go2 basic blueprint CPU usage.

This is the ONLY file the autoresearch agent should edit. `eval.py` imports
`apply()` and splices its return value into the replay subprocess.

Design:
- Each knob lives as a top-level constant with a default matching DimOS
  upstream (i.e. the baseline when the knob is "off"). Flip a value or
  toggle an `ENABLE_*` flag to activate the change.
- `apply()` composes the active knobs into `{cli_args, env, startup_code}`.
- `startup_code` is written to `_patches/sitecustomize.py` and injected via
  `PYTHONPATH`, so it runs before `dimos` imports — that's how we
  monkey-patch module-level constants like `_LCM_LOOP_TIMEOUT` without
  touching the DimOS source tree.

Baseline: all `ENABLE_*` flags False → `apply()` returns empty dict-ish
(no CLI args, no env, no patches). This matches "Re-baseline with a clean
`optimizations.py` (empty apply() returning {})" from program.md.
"""

from __future__ import annotations

import os
from pathlib import Path

PATCH_DIR = Path(__file__).parent / "_patches"


# ------------------------------------------------------------------
# KNOB 1: LCM polling timeout
# ------------------------------------------------------------------
# Source: dimos/protocol/service/lcmservice.py:56 — `_LCM_LOOP_TIMEOUT = 50` (ms)
# Each LCM service runs a dedicated thread that blocks on `lcm.handle_timeout(T)`.
# Lower T = more context switches. With ~10 LCM services and T=50ms, that's
# ~200 wakeups/sec of pure overhead.
# Safe range: 20 - 500 ms. Above ~500ms, RPC latency becomes visible.
ENABLE_LCM_TIMEOUT = False
LCM_LOOP_TIMEOUT_MS = 50  # upstream default


# ------------------------------------------------------------------
# KNOB 2: RPC call thread pool size
# ------------------------------------------------------------------
# Source: dimos/protocol/rpc/pubsubrpc.py:80 — `_call_thread_pool_max_workers = 50`
# Each module instance lazily creates a 50-worker pool for RPC call handlers.
# During replay, RPC traffic is ~zero (no skill invocations). 50 is overkill.
# Safe range: 2 - 16. Going to 1 risks deadlocking recursive RPCs.
ENABLE_RPC_POOL = False
RPC_POOL_MAX_WORKERS = 50  # upstream default


# ------------------------------------------------------------------
# KNOB 3: camera_info 1 Hz daemon thread
# ------------------------------------------------------------------
# Source: dimos/robot/unitree/go2/connection.py:355-358 — `while True: publish; sleep(1.0)`
# During replay, camera intrinsics are static and don't need to be re-published
# 1x/sec. Options:
#   "default": 1 Hz (upstream behavior)
#   "slow":    0.1 Hz (every 10s)
#   "once":    publish once on start, then exit thread
CAMERA_INFO_MODE = "default"  # "default" | "slow" | "once"


# ------------------------------------------------------------------
# KNOB 4: n_workers (DimOS worker process count)
# ------------------------------------------------------------------
# Basic blueprint has 3-4 modules; 2 workers is usually enough. Each extra
# worker is a full Python process = ~150MB RSS + forkserver overhead.
# None = leave at blueprint default (unitree-go2-basic sets 4 via
# global_config(n_workers=4)).
N_WORKERS: int | None = None


# ------------------------------------------------------------------
# KNOB 5: BLAS / OpenMP thread counts
# ------------------------------------------------------------------
# numpy / torch / lcm codec libs spawn parallel threads by default (one per
# core). For single-threaded operations these do nothing but burn CPU on
# thread-pool management. Pinning to 1 often helps on replay.
ENABLE_BLAS_PINNING = False
OMP_NUM_THREADS = 1
MKL_NUM_THREADS = 1
OPENBLAS_NUM_THREADS = 1


# ------------------------------------------------------------------
# KNOB 6: Logging verbosity
# ------------------------------------------------------------------
# Source: dimos/utils/logging_config.py:252 reads DIMOS_LOG_LEVEL env var.
# Default is INFO; every transport/module logs a few lines per second.
# WARNING or ERROR cuts stdout volume (and formatter CPU) substantially.
ENABLE_LOG_REDUCTION = False
LOG_LEVEL = "INFO"  # "DEBUG" | "INFO" | "WARNING" | "ERROR"


def _build_startup_code() -> str:
    """Assemble the monkey-patch string injected via sitecustomize.py."""
    lines: list[str] = []

    if ENABLE_LCM_TIMEOUT:
        lines.append(
            "import dimos.protocol.service.lcmservice as _lcm_mod\n"
            f"_lcm_mod._LCM_LOOP_TIMEOUT = {LCM_LOOP_TIMEOUT_MS}\n"
        )

    if ENABLE_RPC_POOL:
        lines.append(
            "import dimos.protocol.rpc.pubsubrpc as _rpc_mod\n"
            f"_rpc_mod.PubSubRPCBase._call_thread_pool_max_workers = {RPC_POOL_MAX_WORKERS}\n"
        )

    if CAMERA_INFO_MODE == "slow":
        lines.append(
            "import dimos.robot.unitree.go2.connection as _go2_conn\n"
            "import time as _t\n"
            "def _slow_cam_info(self):\n"
            "    while True:\n"
            "        self.camera_info.publish(self.camera_info_static)\n"
            "        _t.sleep(10.0)\n"
            "_go2_conn.GO2Connection.publish_camera_info = _slow_cam_info\n"
        )
    elif CAMERA_INFO_MODE == "once":
        lines.append(
            "import dimos.robot.unitree.go2.connection as _go2_conn\n"
            "def _once_cam_info(self):\n"
            "    self.camera_info.publish(self.camera_info_static)\n"
            "_go2_conn.GO2Connection.publish_camera_info = _once_cam_info\n"
        )

    return "".join(lines)


def apply() -> dict:
    """Return optimization config for the eval harness."""

    cli_args: list[str] = []
    env: dict[str, str] = {}

    if N_WORKERS is not None:
        cli_args.append(f"--n-workers={N_WORKERS}")

    if ENABLE_LOG_REDUCTION:
        env["DIMOS_LOG_LEVEL"] = LOG_LEVEL

    if ENABLE_BLAS_PINNING:
        env["OMP_NUM_THREADS"] = str(OMP_NUM_THREADS)
        env["MKL_NUM_THREADS"] = str(MKL_NUM_THREADS)
        env["OPENBLAS_NUM_THREADS"] = str(OPENBLAS_NUM_THREADS)

    startup_code = _build_startup_code()
    if startup_code:
        PATCH_DIR.mkdir(exist_ok=True)
        (PATCH_DIR / "sitecustomize.py").write_text(startup_code)
        existing = os.environ.get("PYTHONPATH", "")
        env["PYTHONPATH"] = f"{PATCH_DIR}:{existing}" if existing else str(PATCH_DIR)

    return {
        "cli_args": cli_args,
        "env": env,
        "startup_code": "",  # handled via sitecustomize
    }
