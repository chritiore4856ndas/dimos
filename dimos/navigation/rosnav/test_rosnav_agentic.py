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

"""
Agentic integration test: LLM agent navigates the robot via skills.

Tests the same agentic architecture as ``unitree_g1_agentic_sim`` but
simplified for CI:

  - ROSNav (Docker, simulation mode) — the ROS2 navigation stack
  - Agent (MockModel) — deterministic tool-call playback
  - NavSkillBridge — worker-side skill module that exposes goto_global
  - AgentTestRunner — feeds messages and waits for completion
  - OdomRecorder — captures robot position for assertions

The key difference from production: NavSkillBridge is a worker-side module
that manually calls ROSNav's goto_global RPC (similar to how
NavigationSkillContainer calls NavigationInterface.set_goal). This avoids
cross-process serialization issues with DockerModule proxies.

Requires:
    - Docker with BuildKit
    - NVIDIA GPU with drivers
    - X11 display (real or virtual)

Run:
    pytest dimos/navigation/rosnav/test_rosnav_agentic.py -m slow -s
"""

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Any

from dimos_lcm.std_msgs import Bool
from langchain_core.messages import HumanMessage
from langchain_core.messages.base import BaseMessage
import pytest
from reactivex.disposable import Disposable

from dimos.agents.agent import Agent
from dimos.agents.agent_test_runner import AgentTestRunner
from dimos.agents.annotation import skill
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.core.docker_runner import DockerModule
from dimos.core.module import Module
from dimos.core.rpc_client import RPCClient
from dimos.core.stream import In
from dimos.core.transport import pLCMTransport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.nav_msgs.Path import Path as NavPath
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.base import NavigationInterface
from dimos.navigation.rosnav.rosnav_module import ROSNav

# Where we ask the agent to go.
GOAL_X = 2.0
GOAL_Y = 0.0
POSITION_TOLERANCE = 1.5  # metres

# Timeouts
ODOM_WAIT_SEC = 30
WARMUP_SEC = 10
NAV_TIMEOUT_SEC = 120

FIXTURE_DIR = Path(__file__).parent / "fixtures"

SYSTEM_PROMPT = (
    "You are a robot navigation assistant. You have access to a goto_global "
    "skill that moves the robot to (x, y) coordinates in the map frame. "
    "The robot starts at (0, 0). When the user asks you to go somewhere, "
    "call goto_global with the requested coordinates. Do not ask for "
    "clarification."
)


class FilteredAgent(Agent):
    """Agent that filters DockerModule proxies from on_system_modules.

    DockerModule proxies cannot be pickled across process boundaries
    (their LCMRPC connections don't survive serialization). We filter
    them out; worker-side skill modules provide the agent's tools instead.
    """

    @rpc
    def on_system_modules(self, modules: list[RPCClient]) -> None:
        # Filter out DockerModules - they can't be used from worker processes
        worker_modules = [m for m in modules if not isinstance(m, DockerModule)]
        super().on_system_modules(worker_modules)


class NavSkillBridge(Module):
    """Worker-side skill module that proxies navigation calls to ROSNav.

    Uses rpc_calls to access NavigationInterface.goto_global on the
    Docker-hosted ROSNav module. This is the same pattern used by
    NavigationSkillContainer in the production agentic blueprint.
    """

    # Request these RPC methods be wired at build time
    rpc_calls: list[str] = ["ROSNav.goto_global"]

    @skill
    def goto_global(self, x: float, y: float) -> str:
        """Go to map coordinates (x, y). The robot starts at (0, 0).

        Args:
            x: X coordinate in the map frame (metres).
            y: Y coordinate in the map frame (metres).

        Returns:
            Status message from the navigation module.
        """
        try:
            goto_rpc = self.get_rpc_calls("ROSNav.goto_global")
            result = goto_rpc(x, y)
            return str(result) if result else f"Navigated to ({x}, {y})"
        except Exception as e:
            return f"Navigation error: {e}"


class OdomRecorder(Module):
    """Records odom for post-test assertions."""

    color_image: In[Image]
    lidar: In[PointCloud2]
    global_pointcloud: In[PointCloud2]
    odom: In[PoseStamped]
    goal_active: In[PoseStamped]
    goal_reached: In[Bool]
    path: In[NavPath]
    cmd_vel: In[Twist]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._lock = threading.Lock()
        self._poses: list[PoseStamped] = []
        self._first_odom = threading.Event()
        self._moved_event = threading.Event()
        self._start_pose: PoseStamped | None = None

    @rpc
    def start(self) -> None:
        self._disposables.add(Disposable(self.odom.subscribe(self._on_odom)))

    def _on_odom(self, msg: PoseStamped) -> None:
        with self._lock:
            self._poses.append(msg)
            if len(self._poses) == 1:
                self._first_odom.set()
            if self._start_pose is not None and not self._moved_event.is_set():
                dx = msg.position.x - self._start_pose.position.x
                dy = msg.position.y - self._start_pose.position.y
                if math.sqrt(dx * dx + dy * dy) > 0.3:
                    self._moved_event.set()

    @rpc
    def wait_for_odom(self, timeout: float = 30.0) -> bool:
        return self._first_odom.wait(timeout)

    @rpc
    def wait_for_movement(self, timeout: float = 120.0) -> bool:
        return self._moved_event.wait(timeout)

    @rpc
    def mark_start(self) -> None:
        with self._lock:
            if self._poses:
                self._start_pose = self._poses[-1]

    @rpc
    def get_start_pose(self) -> PoseStamped | None:
        with self._lock:
            return self._start_pose

    @rpc
    def get_latest_pose(self) -> PoseStamped | None:
        with self._lock:
            return self._poses[-1] if self._poses else None

    @rpc
    def get_odom_count(self) -> int:
        with self._lock:
            return len(self._poses)

    @rpc
    def stop(self) -> None:
        pass


def _distance_2d(a: PoseStamped, b: PoseStamped) -> float:
    return math.sqrt((a.position.x - b.position.x) ** 2 + (a.position.y - b.position.y) ** 2)


def _ensure_fixture(fixture_path: Path) -> None:
    """Create the MockModel fixture if it doesn't exist."""
    fixture_path.parent.mkdir(parents=True, exist_ok=True)
    if not fixture_path.exists():
        fixture_data = {
            "responses": [
                {
                    "content": "",
                    "tool_calls": [
                        {
                            "name": "goto_global",
                            "args": {"x": GOAL_X, "y": GOAL_Y},
                            "id": "call_nav_001",
                            "type": "tool_call",
                        }
                    ],
                },
                {
                    "content": f"I've sent the robot to ({GOAL_X}, {GOAL_Y}).",
                    "tool_calls": [],
                },
            ]
        }
        fixture_path.write_text(json.dumps(fixture_data, indent=2) + "\n")


@pytest.mark.slow
def test_rosnav_agentic_goto():
    """Build agentic blueprint, send navigation command, verify robot moves.

    This test mirrors the architecture of ``unitree_g1_agentic_sim``:
    ROSNav runs in Docker, the Agent runs in a worker, and skills are
    discovered via worker-side skill modules that proxy to Docker.

    The flow:
    1. Agent receives "Go to (2, 0)" message
    2. MockModel calls goto_global(2, 0) tool
    3. NavSkillBridge.goto_global() forwards to ROSNav via RPC
    4. ROSNav sends goal to ROS2 nav stack, robot moves
    5. Test verifies displacement toward target
    """

    fixture = FIXTURE_DIR / "test_rosnav_agentic_goto.json"
    _ensure_fixture(fixture)

    agent_kwargs: dict[str, Any] = {"system_prompt": SYSTEM_PROMPT}
    if bool(os.getenv("RECORD")) or fixture.exists():
        agent_kwargs["model_fixture"] = str(fixture)

    # Collect agent history via transport taps
    history: list[BaseMessage] = []
    finished_event = threading.Event()
    agent_transport = pLCMTransport("/agent")
    finished_transport = pLCMTransport("/finished")
    agent_transport.subscribe(lambda msg: history.append(msg))
    finished_transport.subscribe(lambda _: finished_event.set())

    # Build the blueprint — mirrors unitree_g1_agentic_sim architecture
    coordinator = (
        autoconnect(
            ROSNav.blueprint(mode="simulation"),
            NavSkillBridge.blueprint(),  # Worker-side skill proxy
            FilteredAgent.blueprint(**agent_kwargs),
            AgentTestRunner.blueprint(
                messages=[HumanMessage(f"Go to map coordinates ({GOAL_X}, {GOAL_Y}).")],
            ),
            OdomRecorder.blueprint(),
        )
        .global_config(viewer="none", n_workers=4)
        .build()
    )

    try:
        recorder = coordinator.get_instance(OdomRecorder)

        # 1. Wait for sim to produce odom — mark start immediately so we
        #    capture the position before navigation begins.
        assert recorder.wait_for_odom(ODOM_WAIT_SEC), (
            f"No odom within {ODOM_WAIT_SEC}s — Unity sim may not be running."
        )
        recorder.mark_start()  # Mark IMMEDIATELY, before agent moves the robot
        start_pose = recorder.get_start_pose()
        assert start_pose is not None
        print(f"  Start: ({start_pose.position.x:.2f}, {start_pose.position.y:.2f})")

        # 2. Wait for the agent to finish (MockModel calls goto_global which blocks).
        agent_done = finished_event.wait(NAV_TIMEOUT_SEC)

        # 3. Verify agent called the right tool.
        tool_calls = []
        for msg in history:
            if hasattr(msg, "tool_calls"):
                tool_calls.extend(msg.tool_calls)

        goto_calls = [tc for tc in tool_calls if tc["name"] == "goto_global"]
        print(f"  Tool calls: {[tc['name'] for tc in tool_calls]}")

        if not agent_done:
            print(f"  ⚠️  Agent did not finish within {NAV_TIMEOUT_SEC}s")
        else:
            assert len(goto_calls) >= 1, (
                f"Agent did not call goto_global. Tool calls: {[tc['name'] for tc in tool_calls]}"
            )
            print(f"  Agent called goto_global({goto_calls[0]['args']})")

        # 4. Check if robot moved.
        recorder.wait_for_movement(30)
        end_pose = recorder.get_latest_pose()
        assert end_pose is not None

        displacement = _distance_2d(start_pose, end_pose)
        print(f"  End: ({end_pose.position.x:.2f}, {end_pose.position.y:.2f})")
        print(f"  Displacement: {displacement:.2f}m (goal: {GOAL_X}, {GOAL_Y})")
        print(f"  Odom messages: {recorder.get_odom_count()}")

        # 5. Verify text response from agent.
        text_msgs = [
            m for m in history
            if hasattr(m, "content") and m.content and not getattr(m, "tool_calls", None)
        ]
        if text_msgs:
            print(f"  Agent response: {text_msgs[-1].content[:120]}")

        # Core assertion: the agent called goto_global AND the robot moved.
        assert len(goto_calls) >= 1, "Agent never called goto_global"
        assert displacement > 0.3, (
            f"Robot only moved {displacement:.2f}m. Expected movement toward "
            f"({GOAL_X}, {GOAL_Y})."
        )
        print("  ✅ Agentic navigation test passed")

    finally:
        agent_transport.stop()
        finished_transport.stop()
        coordinator.stop()
