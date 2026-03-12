# Key Files Map

## Dashboard (Web Layer)

| File | What |
|------|------|
| `dimos/web/templates/mission_control.html` | Dashboard UI — HTML/CSS/JS single file, 3x3 grid layout |
| `dimos/web/websocket_vis/websocket_vis_module.py` | Backend — Starlette server, SocketIO, routes, spy tool lifecycle, `/api/chat` Claude proxy |
| `dimos/web/websocket_vis/test_mission_control.py` | Tests — routes, LCMSpy, AgentMonitor, event shapes, spy lifecycle |
| `dimos/web/templates/rerun_dashboard.html` | Legacy 2-panel dashboard (kept as fallback) |

## Agent System

| File | What |
|------|------|
| `dimos/agents/agent.py` | Main Agent module — LangChain/LangGraph, In/Out streams, skill discovery via RPC |
| `dimos/agents/vlm_agent.py` | VLMAgent — vision-language queries (camera stream + text → AIMessage) |
| `dimos/agents/system_prompt.py` | Default system prompt for "Daneel" (Go2 agent) |
| `dimos/agents/mcp/mcp_server.py` | MCP server on port 9990 — exposes skills as JSON-RPC tools |
| `dimos/agents/mcp/mcp_client.py` | MCP client for tool discovery + invocation |
| `dimos/agents/skills/light_skill.py` | Hackathon skill — Sonoff smart plug control (on/off) |
| `dimos/agents/skills/navigation.py` | Navigation skills (navigate_with_text, tag_location, etc.) |
| `dimos/agents/skills/person_follow.py` | Person follow skill |

## Perception (for People Intelligence)

| File | What |
|------|------|
| `dimos/perception/detection/person_tracker.py` | PersonTracker module — YOLO detections → 3D target pose |
| `dimos/perception/detection/module2D.py` | 2D detection module (YOLO, configurable) |
| `dimos/perception/detection/module3D.py` | 3D detection module (depth + 2D → 3D objects) |
| `dimos/perception/detection/reid/` | Re-identification — embedding-based person matching |
| `dimos/perception/spatial_perception.py` | Spatial memory — ChromaDB + CLIP vector DB |

## Temporal Memory (for Activity Tracking + Query)

| File | What |
|------|------|
| `dimos/perception/experimental/temporal_memory/temporal_memory.py` | VideoRAG-style temporal memory — entity rosters, rolling summaries, VLM analysis |
| `dimos/perception/experimental/temporal_memory/entity_graph_db.py` | SQLite graph DB — relations, distances, semantic edges between entities |
| `dimos/perception/experimental/temporal_memory/clip_filter.py` | CLIP-based keyframe selection (adaptive) |
| `dimos/perception/experimental/temporal_memory/temporal_utils/` | Utility functions for temporal processing |

## CLI & Monitoring Tools

| File | What |
|------|------|
| `dimos/utils/cli/agentspy/agentspy.py` | AgentMessageMonitor — subscribes to `/agent` LCM topic, callback dispatch |
| `dimos/utils/cli/lcmspy/lcmspy.py` | GraphLCMSpy — LCM traffic monitor (per-topic freq, bandwidth, totals) |
| `dimos/utils/cli/dtop.py` | System resource monitor (Textual TUI) |
| `dimos/utils/cli/human/humancli.py` | Human CLI for agent interaction |
| `dimos/robot/cli/dimos.py` | Main CLI entry point — `dimos run/list/status/stop/restart` |

## Robot Blueprints (what we run)

| File | What |
|------|------|
| `dimos/robot/unitree/go2/blueprints/basic/unitree_go2_basic.py` | Go2 basic blueprint (replay-friendly) |
| `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic_mcp.py` | Go2 with MCP agent |
| `dimos/robot/unitree/go2/blueprints/basic/unitree_go2_fleet.py` | Go2 fleet (multi-robot) |

## Ports

| Service | Port |
|---------|------|
| Mission Control + SocketIO | 7779 |
| Command Center (2D map) | 7779/command-center |
| Rerun 3D viewer | 9090 |
| MCP server | 9990 |
| dtop (textual-serve) | 8001 |
