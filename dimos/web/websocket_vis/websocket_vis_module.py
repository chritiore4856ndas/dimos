#!/usr/bin/env python3

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

"""
WebSocket Visualization Module for Dimos navigation and mapping.

This module provides a WebSocket data server for real-time visualization.
The frontend is served from a separate HTML file.
"""

import asyncio
import os
from pathlib import Path as FilePath
import subprocess
import threading
import time
from typing import Any
import webbrowser

import anthropic
import httpx

try:
    import psutil as _psutil
except ImportError:
    _psutil = None  # type: ignore[assignment]

from dimos_lcm.std_msgs import Bool  # type: ignore[import-untyped]
from reactivex.disposable import Disposable
import socketio  # type: ignore[import-untyped]
from starlette.applications import Starlette
from starlette.responses import FileResponse, JSONResponse, RedirectResponse, Response
from starlette.routing import Route
import uvicorn

from dimos.utils.data import get_data

# Path to the frontend HTML templates and command-center build
_TEMPLATES_DIR = FilePath(__file__).parent.parent / "templates"
_DASHBOARD_HTML = _TEMPLATES_DIR / "rerun_dashboard.html"
_MISSION_CONTROL_HTML = _TEMPLATES_DIR / "mission_control.html"
_COMMAND_CENTER_DIR = (
    FilePath(__file__).parent.parent / "command-center-extension" / "dist-standalone"
)

from dimos.core.core import rpc
from dimos.core.global_config import GlobalConfig, global_config
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.mapping.occupancy.gradient import gradient
from dimos.mapping.occupancy.inflation import simple_inflate
from dimos.mapping.types import LatLon
from dimos.msgs.geometry_msgs import PoseStamped, Twist, TwistStamped, Vector3
from dimos.msgs.nav_msgs import OccupancyGrid, Path
from dimos.utils.logging_config import setup_logger

from .optimized_costmap import OptimizedCostmapEncoder

logger = setup_logger()

_browser_open_lock = threading.Lock()
_browser_opened = False

# Spy tool definitions: name → (module_path, port)
# lcmspy and agentspy run in-process and stream via SocketIO — no textual-serve needed.
# dtop is served via textual-serve as an interactive web terminal.
_SPY_TOOLS: dict[str, tuple[str, int]] = {
    "dtop": ("dimos.utils.cli.dtop", 8001),
}


class WebsocketVisModule(Module):
    """
    WebSocket-based visualization module for real-time navigation data.

    This module provides a web interface for visualizing:
    - Robot position and orientation
    - Navigation paths
    - Costmaps
    - Interactive goal setting via mouse clicks

    Inputs:
        - robot_pose: Current robot position
        - path: Navigation path
        - global_costmap: Global costmap for visualization

    Outputs:
        - click_goal: Goal position from user clicks
    """

    # LCM inputs
    odom: In[PoseStamped]
    gps_location: In[LatLon]
    path: In[Path]
    global_costmap: In[OccupancyGrid]

    # LCM outputs
    goal_request: Out[PoseStamped]
    gps_goal: Out[LatLon]
    explore_cmd: Out[Bool]
    stop_explore_cmd: Out[Bool]
    cmd_vel: Out[Twist]
    movecmd_stamped: Out[TwistStamped]

    def __init__(
        self,
        port: int = 7779,
        cfg: GlobalConfig = global_config,
        **kwargs: Any,
    ) -> None:
        """Initialize the WebSocket visualization module.

        Args:
            port: Port to run the web server on
            cfg: Optional global config for viewer settings
        """
        super().__init__(**kwargs)
        self._global_config = cfg

        self.port = port
        self._uvicorn_server_thread: threading.Thread | None = None
        self.sio: socketio.AsyncServer | None = None
        self.app = None
        self._broadcast_loop = None
        self._broadcast_thread = None
        self._uvicorn_server: uvicorn.Server | None = None

        self.vis_state = {}  # type: ignore[var-annotated]
        self.state_lock = threading.Lock()
        self.costmap_encoder = OptimizedCostmapEncoder(chunk_size=64)

        # Track GPS goal points for visualization
        self.gps_goal_points: list[dict[str, float]] = []

        # Spy tool subprocesses (dtop only — lcmspy/agentspy stream in-process)
        self._tool_procs: dict[str, subprocess.Popen[bytes]] = {}
        self._tool_ports: dict[str, int] = {}

        # In-process monitors
        self._agent_monitor: Any = None
        self._lcm_spy: Any = None
        self._lcm_stats_thread: threading.Thread | None = None
        self._lcm_stats_stop: threading.Event = threading.Event()

        logger.info(
            f"WebSocket visualization module initialized on port {port}, GPS goal tracking enabled"
        )

    def _start_broadcast_loop(self) -> None:
        def websocket_vis_loop() -> None:
            self._broadcast_loop = asyncio.new_event_loop()  # type: ignore[assignment]
            asyncio.set_event_loop(self._broadcast_loop)
            try:
                self._broadcast_loop.run_forever()  # type: ignore[attr-defined]
            except Exception as e:
                logger.error(f"Broadcast loop error: {e}")
            finally:
                self._broadcast_loop.close()  # type: ignore[attr-defined]

        self._broadcast_thread = threading.Thread(target=websocket_vis_loop, daemon=True)  # type: ignore[assignment]
        self._broadcast_thread.start()  # type: ignore[attr-defined]

    def _launch_spy_tools(self) -> None:
        """Launch dtop as a textual-serve web instance."""
        import sys as _sys

        python = _sys.executable  # use the active venv python
        for name, (module_path, port) in _SPY_TOOLS.items():
            try:
                script = (
                    f"from textual_serve.server import Server; "
                    f"Server(command='{python} -m {module_path}', port={port}, "
                    f"title='{name}').serve()"
                )
                proc = subprocess.Popen(
                    [python, "-c", script],
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    env={**os.environ, "PYTHONPATH": str(FilePath(__file__).parents[3])},
                )
                self._tool_procs[name] = proc
                self._tool_ports[name] = port
                logger.info(f"Launched {name} on port {port} (pid {proc.pid})")
            except Exception as e:
                logger.warning(f"Failed to launch {name}: {e}")

    def _stop_spy_tools(self) -> None:
        """Terminate all spy tool subprocesses."""
        for name, proc in self._tool_procs.items():
            try:
                proc.terminate()
                proc.wait(timeout=3)
                logger.info(f"Stopped {name} (pid {proc.pid})")
            except Exception:
                proc.kill()
                logger.warning(f"Force-killed {name} (pid {proc.pid})")
        self._tool_procs.clear()
        self._tool_ports.clear()

    @rpc
    def start(self) -> None:
        super().start()

        self._create_server()

        self._start_broadcast_loop()

        self._uvicorn_server_thread = threading.Thread(target=self._run_uvicorn_server, daemon=True)
        self._uvicorn_server_thread.start()

        # Launch dtop textual-serve (interactive terminal in Dev Tools panel)
        self._launch_spy_tools()

        # In-process monitors: stream agent messages + LCM stats via SocketIO
        self._start_agent_monitor()
        self._start_lcm_stats_publisher()

        # Always open mission control dashboard in browser
        url = f"http://localhost:{self.port}/mission-control"
        logger.info(f"Dimensional Mission Control: {url}")

        global _browser_opened
        with _browser_open_lock:
            if not _browser_opened:
                try:
                    webbrowser.open_new_tab(url)
                    _browser_opened = True
                except Exception as e:
                    logger.debug(f"Failed to open browser: {e}")

        try:
            unsub = self.odom.subscribe(self._on_robot_pose)
            self._disposables.add(Disposable(unsub))
        except Exception:
            ...

        try:
            unsub = self.gps_location.subscribe(self._on_gps_location)
            self._disposables.add(Disposable(unsub))
        except Exception:
            ...

        try:
            unsub = self.path.subscribe(self._on_path)
            self._disposables.add(Disposable(unsub))
        except Exception:
            ...

        try:
            unsub = self.global_costmap.subscribe(self._on_global_costmap)
            self._disposables.add(Disposable(unsub))
        except Exception:
            ...

    @rpc
    def stop(self) -> None:
        if getattr(self, "_ws_stopped", False):
            return
        self._ws_stopped = True

        self._stop_spy_tools()
        self._stop_monitors()

        if self._uvicorn_server:
            self._uvicorn_server.should_exit = True

        if self.sio and self._broadcast_loop and not self._broadcast_loop.is_closed():

            async def _disconnect_all() -> None:
                await self.sio.disconnect()

            asyncio.run_coroutine_threadsafe(_disconnect_all(), self._broadcast_loop)

        if self._broadcast_loop and not self._broadcast_loop.is_closed():
            self._broadcast_loop.call_soon_threadsafe(self._broadcast_loop.stop)

        if self._broadcast_thread and self._broadcast_thread.is_alive():
            self._broadcast_thread.join(timeout=1.0)

        if self._uvicorn_server_thread and self._uvicorn_server_thread.is_alive():
            self._uvicorn_server_thread.join(timeout=2.0)

        super().stop()

    @rpc
    def set_gps_travel_goal_points(self, points: list[LatLon]) -> None:
        json_points = [{"lat": x.lat, "lon": x.lon} for x in points]
        self.vis_state["gps_travel_goal_points"] = json_points
        self._emit("gps_travel_goal_points", json_points)

    def _create_server(self) -> None:
        # Create SocketIO server
        self.sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")

        async def serve_mission_control(request):  # type: ignore[no-untyped-def]
            """Always serve mission control dashboard (direct access)."""
            return FileResponse(_MISSION_CONTROL_HTML, media_type="text/html")

        async def serve_index(request):  # type: ignore[no-untyped-def]
            """Serve appropriate HTML based on viewer mode."""
            # If running native Rerun, redirect to standalone command center
            if self._global_config.viewer != "rerun-web":
                return RedirectResponse(url="/command-center")

            # Serve mission control dashboard (falls back to legacy if missing)
            if _MISSION_CONTROL_HTML.exists():
                return FileResponse(_MISSION_CONTROL_HTML, media_type="text/html")
            return FileResponse(_DASHBOARD_HTML, media_type="text/html")

        async def serve_legacy_dashboard(request):  # type: ignore[no-untyped-def]
            """Serve the legacy two-panel Rerun dashboard."""
            return FileResponse(_DASHBOARD_HTML, media_type="text/html")

        async def serve_command_center(request):  # type: ignore[no-untyped-def]
            """Serve the command center 2D visualization (built React app)."""
            index_file = get_data("command_center.html")
            if index_file.exists():
                return FileResponse(index_file, media_type="text/html")
            else:
                return Response(
                    content="Command center not built. Run: cd dimos/web/command-center-extension && npm install && npm run build:standalone",
                    status_code=503,
                    media_type="text/plain",
                )

        async def serve_health(request):  # type: ignore[no-untyped-def]
            """Health check endpoint for the mission control dashboard."""
            return JSONResponse({"status": "ok", "port": self.port})

        async def serve_services(request):  # type: ignore[no-untyped-def]
            """Return running spy tool services and their ports."""
            services = {}
            for name, port in self._tool_ports.items():
                proc = self._tool_procs.get(name)
                alive = proc is not None and proc.poll() is None
                services[name] = {"port": port, "url": f"http://localhost:{port}/", "alive": alive}
            return JSONResponse(services)

        async def serve_system(request):  # type: ignore[no-untyped-def]
            """Return system stats (CPU, memory, top processes) for the htop panel."""
            if _psutil is None:
                return JSONResponse({"error": "psutil not installed"}, status_code=503)
            cpu_per = _psutil.cpu_percent(percpu=True)
            mem = _psutil.virtual_memory()
            swap = _psutil.swap_memory()
            procs = []
            for p in _psutil.process_iter(
                ["pid", "name", "cpu_percent", "memory_percent", "status"]
            ):
                try:
                    procs.append(p.info)
                except (_psutil.NoSuchProcess, _psutil.AccessDenied):
                    pass
            procs.sort(key=lambda x: x.get("cpu_percent") or 0, reverse=True)
            return JSONResponse(
                {
                    "cpu_per": cpu_per,
                    "cpu_avg": sum(cpu_per) / len(cpu_per) if cpu_per else 0,
                    "mem": {"total": mem.total, "used": mem.used, "percent": mem.percent},
                    "swap": {"total": swap.total, "used": swap.used, "percent": swap.percent},
                    "procs": procs[:30],
                }
            )

        _MCP_URL = "http://localhost:9990/mcp"

        async def serve_chat(request):  # type: ignore[no-untyped-def]
            """POST /api/chat — proxy chat messages through Claude with MCP tool use."""
            try:
                body = await request.json()
            except Exception:
                return JSONResponse({"error": "Invalid JSON body"})

            message: str = body.get("message", "")
            history: list[dict] = body.get("history", [])

            if not message:
                return JSONResponse({"error": "message is required"})

            # Fetch MCP tools
            tools: list[dict] = []
            try:
                async with httpx.AsyncClient(timeout=5.0) as client:
                    mcp_resp = await client.post(
                        _MCP_URL,
                        json={"jsonrpc": "2.0", "id": 1, "method": "tools/list", "params": {}},
                        headers={"Content-Type": "application/json"},
                    )
                    mcp_data = mcp_resp.json()
                    raw_tools = mcp_data.get("result", {}).get("tools", [])
                    tools = [
                        {
                            "name": t["name"],
                            "description": t.get("description", ""),
                            "input_schema": t.get(
                                "inputSchema", {"type": "object", "properties": {}}
                            ),
                        }
                        for t in raw_tools
                    ]
            except Exception as e:
                logger.debug(f"MCP tools/list failed: {e}")
                tools = []

            def emit_agent(msg_type: str, style: str, content: str) -> None:
                ts = time.strftime("%H:%M:%S")
                self._emit(
                    "agent_message",
                    {
                        "timestamp": time.time(),
                        "timestamp_str": ts,
                        "type": msg_type,
                        "style": style,
                        "content": content,
                    },
                )

            messages = [*list(history), {"role": "user", "content": message}]

            # Emit user message to agent feed
            emit_agent("Human ", "green", message)

            # First Claude call
            try:
                client_kw: dict = {
                    "model": "claude-opus-4-6",
                    "max_tokens": 4096,
                    "messages": messages,
                }
                if tools:
                    client_kw["tools"] = tools
                loop = asyncio.get_event_loop()
                first_resp = await loop.run_in_executor(
                    None,
                    lambda: anthropic.Anthropic().messages.create(**client_kw),
                )
            except Exception as e:
                logger.warning(f"Claude API error: {e}")
                return JSONResponse({"error": f"Claude API error: {e}"})

            tool_calls_log: list[dict] = []

            # Handle tool use blocks
            tool_use_blocks = [b for b in first_resp.content if b.type == "tool_use"]
            if tool_use_blocks:
                tool_results = []
                async with httpx.AsyncClient(timeout=30.0) as client:
                    for tb in tool_use_blocks:
                        args_str = str(tb.input)[:120]
                        emit_agent("Tool  ", "blue", f"{tb.name}({args_str})")
                        try:
                            call_resp = await client.post(
                                _MCP_URL,
                                json={
                                    "jsonrpc": "2.0",
                                    "id": 2,
                                    "method": "tools/call",
                                    "params": {"name": tb.name, "arguments": tb.input},
                                },
                                headers={"Content-Type": "application/json"},
                            )
                            call_data = call_resp.json()
                            result = call_data.get("result", call_data.get("error", {}))
                        except Exception as e:
                            result = {"error": str(e)}

                        result_str = str(result)[:200]
                        emit_agent("Tool  ", "red", f"{tb.name}() → {result_str}")
                        tool_calls_log.append(
                            {"name": tb.name, "input": tb.input, "result": result}
                        )
                        tool_results.append(
                            {
                                "type": "tool_result",
                                "tool_use_id": tb.id,
                                "content": str(result),
                            }
                        )

                # Second Claude call with tool results
                messages2 = [
                    *messages,
                    {"role": "assistant", "content": first_resp.content},
                    {"role": "user", "content": tool_results},
                ]
                try:
                    second_resp = await loop.run_in_executor(
                        None,
                        lambda: anthropic.Anthropic().messages.create(
                            model="claude-opus-4-6",
                            max_tokens=4096,
                            messages=messages2,
                        ),
                    )
                    text_blocks = [b for b in second_resp.content if b.type == "text"]
                    response_text = "\n".join(b.text for b in text_blocks)
                except Exception as e:
                    logger.warning(f"Claude second API call error: {e}")
                    return JSONResponse({"error": f"Claude API error (second call): {e}"})
            else:
                text_blocks = [b for b in first_resp.content if b.type == "text"]
                response_text = "\n".join(b.text for b in text_blocks)

            if response_text:
                emit_agent("Agent ", "yellow", response_text[:300])

            return JSONResponse({"response": response_text, "tool_calls": tool_calls_log})

        routes = [
            Route("/", serve_index),
            Route("/mission-control", serve_mission_control),
            Route("/health", serve_health),
            Route("/api/services", serve_services),
            Route("/api/system", serve_system),
            Route("/api/chat", serve_chat, methods=["POST"]),
            Route("/legacy", serve_legacy_dashboard),
            Route("/command-center", serve_command_center),
        ]

        starlette_app = Starlette(routes=routes)

        self.app = socketio.ASGIApp(self.sio, starlette_app)

        # Register SocketIO event handlers
        @self.sio.event  # type: ignore[untyped-decorator]
        async def connect(sid, environ) -> None:  # type: ignore[no-untyped-def]
            with self.state_lock:
                current_state = dict(self.vis_state)

            # Include GPS goal points in the initial state
            if self.gps_goal_points:
                current_state["gps_travel_goal_points"] = self.gps_goal_points

            # Force full costmap update on new connection
            self.costmap_encoder.last_full_grid = None

            await self.sio.emit("full_state", current_state, room=sid)  # type: ignore[union-attr]
            logger.info(
                f"Client {sid} connected, sent state with {len(self.gps_goal_points)} GPS goal points"
            )

        @self.sio.event  # type: ignore[untyped-decorator]
        async def click(sid, position) -> None:  # type: ignore[no-untyped-def]
            goal = PoseStamped(
                position=(position[0], position[1], 0),
                orientation=(0, 0, 0, 1),  # Default orientation
                frame_id="world",
            )
            self.goal_request.publish(goal)
            logger.info(
                "Click goal published", x=round(goal.position.x, 3), y=round(goal.position.y, 3)
            )

        @self.sio.event  # type: ignore[untyped-decorator]
        async def gps_goal(sid: str, goal: dict[str, float]) -> None:
            logger.info(f"Received GPS goal: {goal}")

            # Publish the goal to LCM
            self.gps_goal.publish(LatLon(lat=goal["lat"], lon=goal["lon"]))

            # Add to goal points list for visualization
            self.gps_goal_points.append(goal)
            logger.info(f"Added GPS goal to list. Total goals: {len(self.gps_goal_points)}")

            # Emit updated goal points back to all connected clients
            if self.sio is not None:
                await self.sio.emit("gps_travel_goal_points", self.gps_goal_points)
            logger.debug(
                f"Emitted gps_travel_goal_points with {len(self.gps_goal_points)} points: {self.gps_goal_points}"
            )

        @self.sio.event  # type: ignore[untyped-decorator]
        async def start_explore(sid: str) -> None:
            logger.info("Starting exploration")
            self.explore_cmd.publish(Bool(data=True))

        @self.sio.event  # type: ignore[untyped-decorator]
        async def stop_explore(sid) -> None:  # type: ignore[no-untyped-def]
            logger.info("Stopping exploration")
            self.stop_explore_cmd.publish(Bool(data=True))

        @self.sio.event  # type: ignore[untyped-decorator]
        async def clear_gps_goals(sid: str) -> None:
            logger.info("Clearing all GPS goal points")
            self.gps_goal_points.clear()
            if self.sio is not None:
                await self.sio.emit("gps_travel_goal_points", self.gps_goal_points)
            logger.info("GPS goal points cleared and updated clients")

        @self.sio.event  # type: ignore[untyped-decorator]
        async def move_command(sid: str, data: dict[str, Any]) -> None:
            # Publish Twist if transport is configured
            if self.cmd_vel and self.cmd_vel.transport:
                twist = Twist(
                    linear=Vector3(data["linear"]["x"], data["linear"]["y"], data["linear"]["z"]),
                    angular=Vector3(
                        data["angular"]["x"], data["angular"]["y"], data["angular"]["z"]
                    ),
                )
                self.cmd_vel.publish(twist)

            # Publish TwistStamped if transport is configured
            if self.movecmd_stamped and self.movecmd_stamped.transport:
                twist_stamped = TwistStamped(
                    ts=time.time(),
                    frame_id="base_link",
                    linear=Vector3(data["linear"]["x"], data["linear"]["y"], data["linear"]["z"]),
                    angular=Vector3(
                        data["angular"]["x"], data["angular"]["y"], data["angular"]["z"]
                    ),
                )
                self.movecmd_stamped.publish(twist_stamped)

    def _start_agent_monitor(self) -> None:
        """Run AgentMessageMonitor in-process; forward each message as agent_message SocketIO event."""
        try:
            from dimos.utils.cli.agentspy.agentspy import AgentMessageMonitor

            self._agent_monitor = AgentMessageMonitor()
            self._agent_monitor.subscribe(self._on_agent_message)
            self._agent_monitor.start()
            logger.info("AgentMessageMonitor started (streaming via SocketIO)")
        except Exception as e:
            logger.warning(f"Failed to start AgentMessageMonitor: {e}")

    def _on_agent_message(self, entry: Any) -> None:
        try:
            from dimos.utils.cli.agentspy.agentspy import (
                format_message_content,
                format_timestamp,
                get_message_type_and_style,
            )

            msg_type, style = get_message_type_and_style(entry.message)
            self._emit(
                "agent_message",
                {
                    "timestamp": entry.timestamp,
                    "timestamp_str": format_timestamp(entry.timestamp),
                    "type": msg_type.strip(),
                    "style": style,
                    "content": format_message_content(entry.message),
                },
            )
        except Exception as e:
            logger.debug(f"agent_message emit error: {e}")

    def _start_lcm_stats_publisher(self) -> None:
        """Run GraphLCMSpy in-process; broadcast lcm_stats every 1 s via SocketIO."""
        try:
            from dimos.utils.cli.lcmspy.lcmspy import GraphLCMSpy

            self._lcm_spy = GraphLCMSpy(autoconf=True, graph_log_window=1.0)
            self._lcm_spy.start()
            self._lcm_stats_stop.clear()
            self._lcm_stats_thread = threading.Thread(
                target=self._lcm_stats_loop, daemon=True, name="lcm-stats-pub"
            )
            self._lcm_stats_thread.start()
            logger.info("LCMStatsPublisher started (streaming via SocketIO)")
        except Exception as e:
            logger.warning(f"Failed to start LCMStatsPublisher: {e}")

    def _lcm_stats_loop(self) -> None:
        while not self._lcm_stats_stop.is_set():
            try:
                with self._lcm_spy._topic_lock:
                    topics = list(self._lcm_spy.topic.values())
                topics.sort(key=lambda t: t.total_traffic(), reverse=True)
                topic_list = [
                    {
                        "name": t.name,
                        "freq": round(t.freq(5.0), 2),
                        "kbps": round(t.kbps(5.0), 2),
                        "kbps_hr": t.kbps_hr(5.0),
                        "total_hr": t.total_traffic_hr(),
                    }
                    for t in topics
                ]
                self._emit(
                    "lcm_stats",
                    {
                        "total": {
                            "freq": round(self._lcm_spy.freq(5.0), 2),
                            "kbps": round(self._lcm_spy.kbps(5.0), 2),
                        },
                        "topics": topic_list,
                    },
                )
            except Exception as e:
                logger.debug(f"lcm_stats publish error: {e}")
            self._lcm_stats_stop.wait(1.0)

    def _stop_monitors(self) -> None:
        self._lcm_stats_stop.set()
        if self._lcm_stats_thread and self._lcm_stats_thread.is_alive():
            self._lcm_stats_thread.join(timeout=2.0)
        if self._lcm_spy is not None:
            try:
                self._lcm_spy.stop()
            except Exception:
                pass
        if self._agent_monitor is not None:
            try:
                self._agent_monitor.stop()
            except Exception:
                pass

    def _run_uvicorn_server(self) -> None:
        config = uvicorn.Config(
            self.app,  # type: ignore[arg-type]
            host="0.0.0.0",
            port=self.port,
            log_level="error",  # Reduce verbosity
        )
        self._uvicorn_server = uvicorn.Server(config)
        self._uvicorn_server.run()

    def _on_robot_pose(self, msg: PoseStamped) -> None:
        pose_data = {"type": "vector", "c": [msg.position.x, msg.position.y, msg.position.z]}
        self.vis_state["robot_pose"] = pose_data
        self._emit("robot_pose", pose_data)

    def _on_gps_location(self, msg: LatLon) -> None:
        pose_data = {"lat": msg.lat, "lon": msg.lon}
        self.vis_state["gps_location"] = pose_data
        self._emit("gps_location", pose_data)

    def _on_path(self, msg: Path) -> None:
        points = [[pose.position.x, pose.position.y] for pose in msg.poses]
        path_data = {"type": "path", "points": points}
        self.vis_state["path"] = path_data
        self._emit("path", path_data)

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        costmap_data = self._process_costmap(msg)
        self.vis_state["costmap"] = costmap_data
        self._emit("costmap", costmap_data)

    def _process_costmap(self, costmap: OccupancyGrid) -> dict[str, Any]:
        """Convert OccupancyGrid to visualization format."""
        costmap = gradient(simple_inflate(costmap, 0.1), max_distance=1.0)
        grid_data = self.costmap_encoder.encode_costmap(costmap.grid)

        return {
            "type": "costmap",
            "grid": grid_data,
            "origin": {
                "type": "vector",
                "c": [costmap.origin.position.x, costmap.origin.position.y, 0],
            },
            "resolution": costmap.resolution,
            "origin_theta": 0,  # Assuming no rotation for now
        }

    def _emit(self, event: str, data: Any) -> None:
        if self._broadcast_loop and not self._broadcast_loop.is_closed():
            asyncio.run_coroutine_threadsafe(self.sio.emit(event, data), self._broadcast_loop)


websocket_vis = WebsocketVisModule.blueprint

__all__ = ["WebsocketVisModule", "websocket_vis"]
