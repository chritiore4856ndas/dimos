# Mission Control Dashboard — Spec

## What We're Building

A unified browser dashboard that gives real-time visibility into the robot's state, messages,
LCM traffic, and agent activity — all from a single page on port 7779.

---

## Architecture Decision: Spy Tools in Web

### The Problem with textual-serve
The original plan was to run `lcmspy` and `agentspy` as `textual-serve` subprocesses on
ports 8001/8002/8003 and embed them as iframes.

**Why that's problematic:**
- **CORS**: iframes on port 7779 loading content from 8001/8002/8003 is cross-origin.
  `textual-serve` does not set `Access-Control-Allow-Origin` headers → browsers block.
- **UX**: xterm.js terminal emulator inside an iframe is hard to size, style, or integrate
  with the dashboard's visual language.
- **Dependency**: `textual-serve` may not be installed; fails silently.

### Chosen Approach: Native SocketIO Streaming

Both tools already consume LCM internally:
- `AgentMessageMonitor` subscribes to the `/agent` LCM topic via `PickleLCM`
- `LCMSpy` subscribes to all topics via `LCMService`

**We run them in-process inside `WebsocketVisModule`** and forward their output as SocketIO
events over the existing port 7779 connection. Dashboard panels render native HTML.

**textual-serve is kept** only for the Dev Tools tab as an optional full-terminal experience
(power users). It's opt-in and degrades gracefully if not installed.

---

## Routes

| Route | Description |
|-------|-------------|
| `GET /` | Mission control (if `viewer=rerun-web`) or redirect to `/command-center` |
| `GET /mission-control` | Always serves mission control dashboard |
| `GET /legacy` | Old two-panel rerun dashboard |
| `GET /command-center` | 2D map (built React app) |
| `GET /health` | `{"status": "ok", "port": N}` |
| `GET /api/services` | Spy tool subprocess status (for Dev Tools tab) |

---

## SocketIO Events

### Server → Client

| Event | Payload | When |
|-------|---------|------|
| `full_state` | `{robot_pose?, path?, costmap?}` | On client connect |
| `robot_pose` | `{x, y, z, qx, qy, qz, qw, stamp}` | Each odom update |
| `path` | `{poses: [{x,y},...]}` | Each path update |
| `costmap_chunk` | `{chunk_index, data, ...}` | Incremental costmap |
| `costmap_full` | `{width, height, resolution, data}` | Full costmap reset |
| `agent_message` | `{timestamp, type, content, tool_calls?}` | Each agent LCM message |
| `lcm_stats` | `{total: {freq, kbps}, topics: [{name, freq, kbps, total_bytes},...]}` | Every 1s |
| `skill_invocation` | `{timestamp, name, args, result?, error?, duration_ms?}` | Phase 2 |

### Client → Server

| Event | Payload | Action |
|-------|---------|--------|
| `click` | `[x, y]` | Publish nav goal to LCM |
| `gps_goal` | `{lat, lon}` | Publish GPS goal to LCM |
| `start_explore` | — | Publish explore command |
| `stop_explore` | — | Publish stop explore command |
| `cmd_vel` | `{linear_x, angular_z}` | Publish twist command |

---

## Dashboard Layout

### Operations Tab (default)

```
+------------------+-------------------------+---------------------+
| Command Center   |  Rerun 3D Viewer        |  Agent Feed         |
| (2D map iframe)  |  (9090 iframe)          |  (native HTML,      |
|                  |  spans full height      |   SocketIO stream)  |
+------------------+                         +---------------------+
| Camera + Chat    |                         |  Skills Feed        |
| (5555 iframe)    |                         |  + Query Bar        |
+------------------+-------------------------+---------------------+
```

### Dev Tools Tab

```
+-------------------------------+-------------------------------+
|  LCM Stats                    |  Agent Messages               |
|  (native HTML table,          |  (native HTML log,            |
|   lcm_stats SocketIO)         |   agent_message SocketIO)     |
+-------------------------------+-------------------------------+
|  Human CLI                    |  MCP Status                   |
|  (textual-serve iframe        |  (skills list from 9990,      |
|   if available, port 8003)    |   polls /mcp/tools/list)      |
+-------------------------------+-------------------------------+
```

### Full-screen Tabs
- **Rerun 3D** — full viewport (port 9090)
- **Command Center** — full viewport 2D map

---

## Port Map

| Service | Port | How |
|---------|------|-----|
| Mission Control + SocketIO server | 7779 | Starlette + python-socketio |
| Command Center (2D map) | 7779/command-center | Same server |
| Rerun 3D viewer | 9090 | iframe (external) |
| Camera + agent chat | 5555 | iframe (external) |
| MCP server | 9990 | JSON-RPC polling |
| Human CLI (optional) | 8003 | textual-serve subprocess, iframe in Dev Tools |

---

## In-Process Monitors (New)

### `AgentStreamMixin`

Runs `AgentMessageMonitor` in the same process as `WebsocketVisModule`. On each new
agent message, emits `agent_message` via SocketIO.

**Lifecycle**: started in `WebsocketVisModule.start()`, stopped in `stop()`.

### `LCMStatsPublisher`

Wraps `LCMSpy` (already has `start()`/`stop()`). Every 1 second, reads
`spy.topic` dict and emits `lcm_stats` via SocketIO.

**Runs in**: background thread with asyncio bridge (`run_coroutine_threadsafe`).

---

## textual-serve (Optional, Dev Tools Only)

If `textual-serve` is installed, `WebsocketVisModule` launches `humancli` on port 8003.
`/api/services` reports its status. Dashboard embeds it in the Dev Tools Human CLI panel.

`lcmspy` and `agentspy` are **NOT** launched via textual-serve anymore — native streaming
replaces them.

---

## Dashboard Header

- **DimOS logo** + "Mission Control" title
- **SocketIO dot**: green (connected) / yellow (reconnecting) / red (disconnected)
- **LCM health dot**: green (>1 topic active) / yellow (0 topics) — from `lcm_stats`
- **Agent status**: last message type + time ago — from `agent_message`
- **Skill count**: polled from MCP `tools/list`
- **Clock**: live HH:MM:SS

---

## Implementation Phases

### Phase 1 (done)
- Routes, HTML shell, auto-launch spy tools via textual-serve, `/api/services`

### Phase 2 (next)
- `LCMStatsPublisher` in-process, native LCM Stats panel in Dev Tools
- `AgentStreamMixin` in-process, native Agent Feed panel in Operations
- Remove `lcmspy`/`agentspy` from textual-serve launch list
- Keep `humancli` textual-serve for Dev Tools

### Phase 3
- Real-time skill invocation stream (`skill_invocation` SocketIO event from MCP server)
- Skills feed panel with timestamp, name, args, result, duration

### Phase 4
- Person intelligence: YOLO + VLM activity classification per crop
- Person cards panel: name, last seen, activity, timeline
- Wire to temporal memory entity graph

### Phase 5
- Query bar → temporal memory `query()` skill
- Room layout overlay on 2D map

---

## Key Files

| File | Purpose |
|------|---------|
| `dimos/web/templates/mission_control.html` | Dashboard HTML/CSS/JS (single file) |
| `dimos/web/websocket_vis/websocket_vis_module.py` | Server, routes, SocketIO, spy monitor lifecycle |
| `dimos/utils/cli/lcmspy/lcmspy.py` | `LCMSpy` / `GraphLCMSpy` — data model |
| `dimos/utils/cli/agentspy/agentspy.py` | `AgentMessageMonitor` — LCM subscriber |
| `dimos/agents/mcp/mcp_server.py` | MCP server (port 9990) |
| `dimos/utils/cli/human/humanclianim.py` | Human CLI (textual-serve, port 8003) |

---

## Non-Goals

- No auth/access control (local dev tool only)
- No mobile layout
- No persistence of agent messages or LCM stats across page reloads
- No alerting or thresholds on LCM bandwidth
