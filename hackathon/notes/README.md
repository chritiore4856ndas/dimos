# Hackathon: Surveillance Dashboard

Robot-powered surveillance platform built on DimOS. A Go2 quadruped patrols a space,
detects and identifies people, classifies activities via VLM, tracks everything over time,
and presents it all in a unified Mission Control dashboard.

## Docs in this folder

| File | What |
|------|------|
| [STATUS.md](STATUS.md) | What's done, what's in progress, what's next |
| [SPEC.md](SPEC.md) | Dashboard architecture, routes, SocketIO events, layout |
| [PLAN.md](PLAN.md) | Phased implementation plan with TODOs |
| [FILES.md](FILES.md) | Key files map — where everything lives in the codebase |

## How to run

```bash
# Full mission control (opens browser automatically)
dimos --replay --viewer rerun-web run unitree-go2-basic
# → http://localhost:7779/

# Any viewer mode, access dashboard directly
dimos --replay run unitree-go2-basic
# → http://localhost:7779/mission-control
```

## The vision

A single browser page where you can see:
- Live 3D view (Rerun) + 2D map (Command Center)
- Real-time agent reasoning (agent feed + Claude chat)
- LCM message traffic stats
- MCP skills available to the agent
- **People cards** — who's in the space, what they're doing, when last seen
- **Activity timeline** — query "what has X been doing today?"
- **Room overlay** — desk assignments, room names, who's where
