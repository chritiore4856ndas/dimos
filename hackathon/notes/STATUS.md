# Status — What's Done, What's Next

## Phase 1: Dashboard Shell — DONE

- [x] `mission_control.html` — dark 3-col grid layout
- [x] Routes: `/`, `/mission-control`, `/legacy`, `/command-center`, `/health`, `/api/services`
- [x] SocketIO connection status (green/yellow/red dot in header)
- [x] Auto-launch `dtop` via textual-serve
- [x] Clean shutdown of spy tool subprocesses
- [x] Rerun overlay (shows hint if Rerun not running)

## Phase 2: In-Process Monitors + Claude Chat — DONE

- [x] `AgentMessageMonitor` running in-process, streaming `agent_message` via SocketIO
- [x] `GraphLCMSpy` running in-process, streaming `lcm_stats` every 1s via SocketIO
- [x] Native LCM Stats panel (table with topic name, freq, bandwidth, total)
- [x] MCP Skills panel (polls `localhost:9990/mcp` every 15s, lists all tools)
- [x] Claude Chat panel (`/api/chat` → Claude opus with MCP tool-use, brief tool usage summary)
- [x] Light control skill (`light_skill.py` — Sonoff S31 smart plug via ESPHome)

## Phase 3: Skill Event Stream — DONE

- [x] Real-time `skill_invocation` SocketIO event from both LCM agent path and `/api/chat` HTTP path
- [x] Event payload: `{id, timestamp, name, args, status, result, duration_ms}`
- [x] Duration tracking: matches AIMessage tool_calls to ToolMessage responses by tool_call_id
- [x] Skills Feed panel: live log with timestamp, skill name, args, duration, status badge (RUN/OK/ERR)
- [x] Status updates in-place (running → success/error with duration)
- [x] Orphan tool messages handled gracefully (no crash if start event missed)
- [x] Claude Chat cleaned up: shows "Used: skill_name" instead of dumping raw results/base64
- [x] Tests: 40 tests in `hackathon/tests/test_skills_feed.py`

### Layout changes (Phase 3):
- Removed Command Center (2D map)
- Rerun 3D moved to full left column (rows 1-2)
- LCM Stats + dtop split side-by-side in bottom-left (row 3)
- Skills Feed (col2, row1), Claude (col2, row2), MCP Skills (col2, row3)
- Empty right column (col3, rows 1-3) — reserved for Phase 4

## Phase 4: People Intelligence — NOT STARTED

This is the core surveillance feature.

- [ ] Wire YOLO person detection to dashboard (perception stack already exists)
- [ ] VLM activity classification per person crop ("studying", "cooking", "on phone")
- [ ] Person identification via clothing/appearance descriptions
- [ ] Log sightings + activities to temporal memory entity graph (EntityGraphDB exists)
- [ ] Dashboard panel: person cards in empty col3 — name/label, last seen time, current activity, thumbnail

### Existing code to leverage:
- `dimos/perception/detection/person_tracker.py` — PersonTracker module (YOLO → 3D pose)
- `dimos/perception/experimental/temporal_memory/temporal_memory.py` — VideoRAG-style entity memory
- `dimos/perception/experimental/temporal_memory/entity_graph_db.py` — SQLite graph DB (relations, distances, semantic)
- `dimos/perception/spatial_perception.py` — ChromaDB + CLIP spatial memory
- `dimos/perception/detection/reid/` — Re-identification embeddings

## Phase 5: Query Engine — NOT STARTED

- [ ] Wire query bar to temporal memory `query()` skill
- [ ] "What has Ruthwik been doing today?" → searches entity graph + rolling summaries
- [ ] Display answers in dashboard

## Phase 6: Room Intelligence — NOT STARTED

- [ ] 2D room layout overlay on command center map
- [ ] Annotate desk assignments, room names, object locations
- [ ] Click room/desk → see who's there and what they're doing
