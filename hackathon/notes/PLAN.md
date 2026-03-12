# Hackathon Plan

## What We're Building
A surveillance platform: robot patrols a space, watches people, tracks activities, answers questions — all in a unified Mission Control dashboard.

## Phases

### Phase 1: Dashboard Shell — DONE
- Mission control HTML (3x3 grid), routing, SocketIO connection, spy tool auto-launch, clean shutdown.

### Phase 2: In-Process Monitors + Claude Chat — DONE
- AgentMessageMonitor + GraphLCMSpy streaming in-process via SocketIO (replaced textual-serve iframes).
- Native Agent Feed, LCM Stats, MCP Skills panels.
- Claude Chat panel with MCP tool-use.
- Light control skill (Sonoff smart plug).

### Phase 3: Skill Event Stream — TODO
- [ ] SocketIO `skill_invocation` event from MCP server (real-time, not polling)
- [ ] Skills feed panel: timestamp, skill name, args, result/error, duration
- [ ] Dashboard subscribes and populates live

### Phase 4: People Intelligence — TODO (core hackathon feature)
- [ ] Wire YOLO person detection → dashboard
- [ ] VLM activity classification per person crop ("studying", "cooking", "on phone")
- [ ] Person ID via clothing/appearance descriptions (user provides)
- [ ] Log sightings + activities to EntityGraphDB (temporal memory)
- [ ] Dashboard: person cards — name, last seen, current activity, thumbnail

Existing code to wire up:
- `PersonTracker` module (YOLO → 3D pose)
- `TemporalMemory` module (VideoRAG entity rosters + rolling summaries)
- `EntityGraphDB` (SQLite graph — relations, distances, semantic edges)
- `ReID` embeddings (person re-identification)

### Phase 5: Query Engine — TODO
- [ ] Wire query bar → temporal memory `query()` skill
- [ ] "What has Ruthwik been doing today?" → entity graph + summaries
- [ ] Answers display in dashboard

### Phase 6: Room Intelligence — STRETCH
- [ ] 2D room layout overlay on command center map
- [ ] Desk assignments, room names, object locations
- [ ] Click room/desk → who's there, what they're doing

## How to Run

```bash
# Full mission control
dimos --replay --viewer rerun-web run unitree-go2-basic
# → http://localhost:7779/

# Direct access
dimos --replay run unitree-go2-basic
# → http://localhost:7779/mission-control
```

## Architecture Notes

### Dashboard backend
`WebsocketVisModule` (Starlette + SocketIO on port 7779):
- Serves dashboard HTML
- Runs AgentMessageMonitor + GraphLCMSpy in-process
- Streams `agent_message`, `lcm_stats` via SocketIO
- `/api/chat` proxies to Claude with MCP tools
- Launches `dtop` via textual-serve (port 8001)

### Spy tools (current)
Only `dtop` runs as textual-serve subprocess now. LCM and agent monitoring are native SocketIO.

```python
_SPY_TOOLS = {
    "dtop": ("dimos.utils.cli.dtop", 8001),
}
```

### Known Issues
- `textual-serve` may not be installed — dtop fails silently
- CORS on cross-origin iframes (Rerun 9090)
- `doclinks` pre-commit hook fails — use `SKIP=doclinks git commit`
