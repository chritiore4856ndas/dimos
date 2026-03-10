# DimOS Mission Control — Hackathon Plan

## What We're Building
A unified **mission control dashboard** that composites all DimOS monitoring tools into a single browser UI. The robot's eyes, brain, and memory — all visible at once.

## Current State

### What's Done (Phase 1 — complete)

1. **`mission_control.html`** — Full dashboard at `dimos/web/templates/mission_control.html`
2. **Routing in `websocket_vis_module.py`**:
   - `http://localhost:7779/` → mission control (when `--viewer rerun-web`) or command center redirect
   - `http://localhost:7779/mission-control` → **always serves mission control** (any viewer mode)
   - `http://localhost:7779/legacy` → old two-panel dashboard
   - `http://localhost:7779/command-center` → standalone 2D map
   - `http://localhost:7779/health` → JSON health check
   - `http://localhost:7779/api/services` → JSON of running spy tool ports + status
3. **Auto-launch spy tools** — `WebsocketVisModule.start()` spawns `lcmspy`, `agentspy`, `humancli` as `textual-serve` subprocesses on fixed ports (8001, 8002, 8003). No manual terminal launching needed.
4. **Auto-embed** — Dashboard polls `/api/services` every 5s, embeds iframes into the correct panels as tools come alive.
5. **SocketIO connection status** — Dashboard connects to the DimOS SocketIO server. Header dot is green/yellow/red for connected/connecting/disconnected (live, not faked).
6. **Iframe health monitoring** — All iframes show a loading state. If a service isn't reachable after 8s, an error overlay appears with the URL.
7. **Clean shutdown** — `stop()` terminates all spy tool subprocesses.

### Dashboard Layout

#### Operations Tab (main view)
```
+----------------+------------------------+---------------------+
|                |                        |                     |
|  Command       |   Rerun 3D View        |   Agent Spy         |
|  Center        |   (spans full          |   (auto-embedded)   |
|  (2D map)      |    height)             |                     |
|                |                        +---------------------+
+----------------+                        |                     |
|                |                        |   Skills Feed       |
|  Camera &      |                        |   + Query Bar       |
|  Chat          |                        |                     |
|  (port 5555)   |                        |                     |
+----------------+------------------------+---------------------+
```

#### Dev Tools Tab
```
+--------------------------+--------------------------+
|  LCM Spy (port 8001)     |  Agent Spy (port 8002)   |
|  (auto-embedded)         |  (auto-embedded)         |
+--------------------------+--------------------------+
|  Human CLI (port 8003)   |  MCP Server Status       |
|  (auto-embedded)         |  (skills list from 9990) |
+--------------------------+--------------------------+
```

#### Full-screen tabs
- **Rerun 3D** — full viewport rerun viewer
- **Command Center** — full viewport 2D map

### Port Map
| Service | Port | How |
|---------|------|-----|
| Mission Control dashboard | 7779 | Starlette + SocketIO |
| Command Center (2D map) | 7779/command-center | Same server, iframe |
| Rerun 3D viewer | 9090 | iframe |
| Camera + agent chat | 5555 | iframe |
| MCP server (skills) | 9990/mcp | JSON-RPC API |
| lcmspy web | 8001 | Auto-launched textual-serve, iframe |
| agentspy web | 8002 | Auto-launched textual-serve, iframe |
| humancli web | 8003 | Auto-launched textual-serve, iframe |

### Dashboard Features
- **Header**: DimOS branding, live SocketIO connection status, skill count from MCP, live clock
- **Tab system**: Alt+1-4 keyboard shortcuts
- **Skills feed**: Live log of skill invocations (listens for `skill_invocation` SocketIO events + MCP polling)
- **Query bar**: Type questions -> sends via MCP `agent_send` -> robot agent processes
- **MCP panel**: Lists all available skills from `tools/list`
- **`window.embedTool(panelId, url)`**: Console helper to manually embed any URL

## How to Run

```bash
# Option A: Full mission control as default view
dimos --replay --viewer rerun-web run unitree-go2-basic
# opens http://localhost:7779/ -> mission control

# Option B: Any viewer mode, access mission control directly
dimos --replay run unitree-go2-basic
# open http://localhost:7779/mission-control
```

Spy tools (lcmspy, agentspy, humancli) are auto-launched — no separate terminals needed.

## TODO — Next Steps

### Phase 2: Skill Event Stream
- [ ] Add a SocketIO or SSE endpoint to MCP server that streams skill invocations in real-time
- [ ] Dashboard subscribes and populates skills feed live (not polling)
- [ ] Show: timestamp, skill name, args, result/error, duration

### Phase 3: People Intelligence
- [ ] Person detection via YOLO (already exists in perception stack)
- [ ] VLM-based activity classification per person crop ("studying", "cooking", etc.)
- [ ] Person identification via clothing description (user provides descriptions)
- [ ] Log person sightings + activities to temporal memory entity graph
- [ ] Dashboard panel: person cards with status, last seen, activity timeline

### Phase 4: Query Engine
- [ ] Wire query bar to temporal memory `query()` skill
- [ ] Support questions like "what has Ruthwik been doing today?"
- [ ] Display answers in skills feed panel

### Phase 5: Room Intelligence
- [ ] 2D room layout overlay on command center map
- [ ] Annotate: desk assignments, room names, object locations
- [ ] Click room/desk to see who's there and what they're doing

## Key Files
- `dimos/web/templates/mission_control.html` — the dashboard (HTML + CSS + JS, single file)
- `dimos/web/templates/rerun_dashboard.html` — legacy 2-panel dashboard
- `dimos/web/websocket_vis/websocket_vis_module.py` — serves dashboards, launches spy tools, SocketIO server
- `dimos/agents/mcp/mcp_server.py` — MCP server (port 9990)
- `dimos/utils/cli/lcmspy/run_lcmspy.py` — LCM traffic monitor
- `dimos/utils/cli/agentspy/agentspy.py` — agent message monitor
- `dimos/utils/cli/human/humanclianim.py` — human CLI
- `dimos/perception/experimental/temporal_memory/` — entity graph + VLM analysis
- `dimos/perception/spatial_perception.py` — spatial memory (ChromaDB + CLIP)

## Architecture Notes

### How spy tools are auto-launched
`WebsocketVisModule.start()` calls `_launch_spy_tools()` which spawns:
```python
_SPY_TOOLS = {
    "lcmspy":   ("dimos.utils.cli.lcmspy.run_lcmspy", 8001),
    "agentspy": ("dimos.utils.cli.agentspy.agentspy",  8002),
    "humancli": ("dimos.utils.cli.human.humanclianim", 8003),
}
```
Each is launched as `subprocess.Popen(["python3", "-c", "from textual_serve.server import Server; Server(command=..., port=...).serve()"])`. The `/api/services` endpoint reports their status. Dashboard polls this and auto-embeds iframes.

### Why `/mission-control` exists
The `/` route redirects to `/command-center` unless `--viewer rerun-web`. The `/mission-control` route always serves the dashboard regardless of viewer mode — useful during development or when using native Rerun viewer.

### SocketIO
The dashboard loads `socket.io.min.js` from CDN and connects to the same server on port 7779. This gives:
- Real connection status (green/yellow/red dot)
- Future: real-time skill invocation streaming (Phase 2)
- Future: live robot state updates in the dashboard

## Known Issues
- `textual-serve` may not be installed — spy tools will fail silently (logged as warnings)
- CORS: cross-origin iframes (Rerun on 9090, Camera on 5555) may be blocked by browser depending on those servers' headers
- The `doclinks` pre-commit hook fails — use `SKIP=doclinks git commit`
