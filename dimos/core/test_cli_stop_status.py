# Copyright 2025-2026 Dimensional Inc.
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

"""Tests for `dimos status` and `dimos stop` CLI commands.

Exercises the actual typer CLI surface using CliRunner with real
subprocess PIDs, not mocks. Verifies output formatting, flag handling,
and registry lifecycle.
"""

from __future__ import annotations

from datetime import datetime, timedelta, timezone
import subprocess
import time
from typing import TYPE_CHECKING

import pytest

from dimos.core import run_registry
from dimos.core.run_registry import RunEntry, list_runs

if TYPE_CHECKING:
    from pathlib import Path


@pytest.fixture(autouse=True)
def _tmp_registry(tmp_path: Path, monkeypatch: pytest.MonkeyPatch):
    """Redirect registry to a temp dir for test isolation."""
    monkeypatch.setattr(run_registry, "REGISTRY_DIR", tmp_path)
    yield tmp_path


@pytest.fixture()
def sleeper():
    """Start a sleep subprocess, kill it on teardown."""
    procs: list[subprocess.Popen] = []

    def _make():
        p = subprocess.Popen(["sleep", "300"])
        procs.append(p)
        return p

    yield _make
    for p in procs:
        try:
            p.kill()
            p.wait(timeout=2)
        except Exception:
            pass


def _entry(run_id: str, pid: int, blueprint: str = "test", **kwargs) -> RunEntry:
    defaults = dict(
        started_at=datetime.now(timezone.utc).isoformat(),
        log_dir="/tmp/dimos-test",
        cli_args=[blueprint],
        config_overrides={},
    )
    defaults.update(kwargs)
    e = RunEntry(run_id=run_id, pid=pid, blueprint=blueprint, **defaults)
    e.save()
    return e


# ---------------------------------------------------------------------------
# STATUS
# ---------------------------------------------------------------------------


class TestStatusCLI:
    """Tests for `dimos status` command."""

    def test_status_no_instances(self):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        result = CliRunner().invoke(main, ["status"])
        assert result.exit_code == 0
        assert "No running" in result.output

    def test_status_shows_running_instance(self, sleeper):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        proc = sleeper()
        _entry("status-test-001", proc.pid, blueprint="unitree-go2")

        result = CliRunner().invoke(main, ["status"])
        assert result.exit_code == 0
        assert "status-test-001" in result.output
        assert str(proc.pid) in result.output
        assert "unitree-go2" in result.output

    def test_status_shows_uptime_minutes(self, sleeper):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        proc = sleeper()
        ago = (datetime.now(timezone.utc) - timedelta(minutes=7, seconds=30)).isoformat()
        _entry("uptime-min", proc.pid, started_at=ago)

        result = CliRunner().invoke(main, ["status"])
        assert "7m" in result.output

    def test_status_shows_uptime_hours(self, sleeper):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        proc = sleeper()
        ago = (datetime.now(timezone.utc) - timedelta(hours=3, minutes=22)).isoformat()
        _entry("uptime-hrs", proc.pid, started_at=ago)

        result = CliRunner().invoke(main, ["status"])
        assert "3h 22m" in result.output

    def test_status_shows_mcp_port(self, sleeper):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        proc = sleeper()
        _entry("mcp-port-test", proc.pid, mcp_port=9990)

        result = CliRunner().invoke(main, ["status"])
        assert "9990" in result.output

    def test_status_multiple_instances(self, sleeper):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        p1, p2 = sleeper(), sleeper()
        _entry("multi-1", p1.pid, blueprint="go2")
        _entry("multi-2", p2.pid, blueprint="g1")

        result = CliRunner().invoke(main, ["status"])
        assert "multi-1" in result.output
        assert "multi-2" in result.output
        assert "go2" in result.output
        assert "g1" in result.output

    def test_status_filters_dead_pids(self):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        _entry("dead-one", pid=2_000_000_000)  # definitely not alive

        result = CliRunner().invoke(main, ["status"])
        assert "No running" in result.output


# ---------------------------------------------------------------------------
# STOP
# ---------------------------------------------------------------------------


class TestStopCLI:
    """Tests for `dimos stop` command."""

    def test_stop_no_instances(self):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        result = CliRunner().invoke(main, ["stop"])
        assert result.exit_code == 1

    def test_stop_default_most_recent(self, sleeper):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        proc = sleeper()
        entry = _entry("stop-default", proc.pid)

        result = CliRunner().invoke(main, ["stop"])
        assert result.exit_code == 0
        assert "Stopping" in result.output
        assert "stop-default" in result.output
        time.sleep(0.3)
        assert not entry.registry_path.exists()

    def test_stop_by_pid(self, sleeper):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        p1, p2 = sleeper(), sleeper()
        e1 = _entry("by-pid-1", p1.pid)
        e2 = _entry("by-pid-2", p2.pid)

        result = CliRunner().invoke(main, ["stop", "--pid", str(p1.pid)])
        assert result.exit_code == 0
        assert "by-pid-1" in result.output
        time.sleep(0.3)
        assert not e1.registry_path.exists()
        # e2 should still exist
        assert e2.registry_path.exists()

    def test_stop_by_pid_not_found(self, sleeper):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        proc = sleeper()
        _entry("exists", proc.pid)

        result = CliRunner().invoke(main, ["stop", "--pid", "99999999"])
        assert result.exit_code == 1
        assert "no running instance" in result.output.lower()

    def test_stop_all(self, sleeper):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        p1, p2, p3 = sleeper(), sleeper(), sleeper()
        _entry("all-1", p1.pid)
        _entry("all-2", p2.pid)
        _entry("all-3", p3.pid)

        result = CliRunner().invoke(main, ["stop", "--all"])
        assert result.exit_code == 0
        assert "all-1" in result.output
        assert "all-2" in result.output
        assert "all-3" in result.output
        time.sleep(0.5)
        assert len(list_runs(alive_only=False)) == 0

    def test_stop_all_empty(self):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        result = CliRunner().invoke(main, ["stop", "--all"])
        assert result.exit_code == 0
        assert "No running" in result.output

    def test_stop_force_sends_sigkill(self, sleeper):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        proc = sleeper()
        _entry("force-kill", proc.pid)

        result = CliRunner().invoke(main, ["stop", "--force"])
        assert result.exit_code == 0
        assert "SIGKILL" in result.output
        time.sleep(0.3)
        # Process should be dead
        assert proc.poll() is not None

    def test_stop_already_dead_cleans_registry(self):
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        entry = _entry("already-dead", pid=2_000_000_000)

        CliRunner().invoke(main, ["stop", "--pid", "2000000000"])
        # Should handle gracefully — either clean or report
        assert not entry.registry_path.exists()

    def test_stop_sigterm_kills_process(self, sleeper):
        """Verify SIGTERM actually terminates the target process."""
        from typer.testing import CliRunner

        from dimos.robot.cli.dimos import main

        proc = sleeper()
        _entry("sigterm-verify", proc.pid)

        result = CliRunner().invoke(main, ["stop"])
        assert "SIGTERM" in result.output
        # sleep handles SIGTERM by dying — wait for it
        time.sleep(6)
        assert proc.poll() is not None
