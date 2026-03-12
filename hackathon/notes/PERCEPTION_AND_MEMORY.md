# Perception, Memory & Agent — How It All Works

## Overview

DimOS has a modular perception → memory → agent pipeline. Camera frames flow through detection modules, get stored in memory systems, and the agent queries those memories via skills (RPC calls exposed as LLM tools).

```
Camera (100fps)
  ↓
Detection2DModule (10Hz, YOLO11n)
  ├─ bounding boxes, track_ids, crops
  ├─→ PersonTracker → 3D world pose
  └─→ ReidModule → long-term person IDs

Camera (100fps)
  ├─→ SpatialMemory (every 1s / 0.01m moved)
  │     ChromaDB + CLIP embeddings + robot pose
  │
  └─→ TemporalMemory (every stride_s seconds)
        VLM analysis → entity roster → EntityGraphDB (SQLite)

Agent
  └─ discovers all module @skill methods via RPC
     └─ calls TemporalMemory.query(), SpatialMemory.query_by_text(), etc.
```

---

## 1. Detection Pipeline

### Detection2DModule
**File**: `dimos/perception/detection/module2D.py`

- **Input**: `color_image: In[Image]` — camera feed
- **Output**: `detections: Out[Detection2DArray]` + `annotations` + `detected_image_0/1/2` (cropped)
- **Model**: YOLO11n (`models_yolo/yolo11n.pt`) via Ultralytics
- **Tracking**: ByteTrack — assigns short-term `track_id` per person across frames
- **Rate**: Max 10Hz (configurable), confidence ≥ 0.5, IoU 0.6

Each detection is a `Detection2DBBox`:
```
bbox: (x1, y1, x2, y2)    # pixel coords
track_id: int               # YOLO's short-term tracker ID (resets on occlusion)
class_id: int               # 0 = person
confidence: float           # 0-1
name: str                   # "person"
image: Image                # full frame reference
cropped_image(padding=20)   # crops bbox region from frame
```

### Detection2DModule (Pose variant)
**File**: `dimos/perception/detection/detectors/person/yolo.py`

- Uses `yolo11n-pose.pt` — same as above but also outputs 17 COCO keypoints per person
- Returns `Detection2DPerson` with `keypoints: [17, 2]` array

### Detection3DModule
**File**: `dimos/perception/detection/module3D.py`

- Extends 2D module, also takes `pointcloud: In[PointCloud2]`
- For each 2D bbox, crops the pointcloud → 3D object pose in world frame
- Outputs `Detection3DPC` with centroid, bounding box dimensions, world-frame pose

### PersonTracker
**File**: `dimos/perception/detection/person_tracker.py`

- **Input**: `detections: In[Detection2DArray]` + `color_image: In[Image]`
- **Output**: `target: Out[PoseStamped]` — single person's 3D world position
- Picks the **largest** detection, unprojects to 3D using camera intrinsics + assumed depth (2m)
- Transforms camera frame → world frame via TF tree
- Used by person-follow skill

---

## 2. Re-Identification (ReID)

### The Problem
YOLO's `track_id` resets when a person leaves the frame and returns. ReID assigns a **persistent long-term ID** across appearances.

### EmbeddingIDSystem
**File**: `dimos/perception/detection/reid/embedding_id_system.py`

**How it works**:
1. For each detection, crop the person image
2. Run through **TorchReIDModel** (OSNet) → 512-dim embedding
3. Store embedding in a per-track buffer (up to 500 per track)
4. When a track accumulates ≥ 10 embeddings, compare against all other tracks
5. If cosine similarity ≥ 0.63 (threshold), **fuse** — same person, assign same `long_term_id`
6. **Negative constraints**: tracks visible in the same frame can never be fused

**Comparison modes**: `top_k_mean` (default) — averages top 30 pairwise similarities between track embedding sets.

### ReidModule
**File**: `dimos/perception/detection/reid/module.py`

- DimOS Module wrapper around EmbeddingIDSystem
- **Input**: `detections: In[Detection2DArray]` + `image: In[Image]`
- **Output**: `annotations: Out[ImageAnnotations]` — text labels with long-term IDs
- Publishes cyan text overlay: "PERSON: {long_term_id}"

### Embedding Models Available
| Model | File | Dims | Use Case |
|-------|------|------|----------|
| TorchReIDModel (OSNet) | `dimos/models/embedding/treid.py` | 512 | Person ReID (vision-only) |
| CLIPModel | `dimos/models/embedding/clip.py` | 512 | Vision-language matching |
| MobileCLIPModel | `dimos/models/embedding/mobileclip.py` | 512 | Lightweight CLIP for edge |

---

## 3. Spatial Memory

### SpatialMemory
**File**: `dimos/perception/spatial_perception.py`

**What it does**: Builds a semantic 3D map — associates CLIP embeddings of camera frames with the robot's pose at capture time.

**Input**: `color_image: In[Image]` — camera feed

**Processing** (`_process_frame`):
1. Get robot pose from TF tree (`world → base_link`)
2. Skip if moved < 0.01m or < 1s elapsed
3. Compute CLIP embedding of full frame
4. Store in ChromaDB: `{embedding, pos_x, pos_y, pos_z, rot_x, rot_y, rot_z, timestamp}`
5. Store image in VisualMemory (base64 JPEG)
6. Every 100 frames: serialize VisualMemory to disk

### What Gets Saved

| File | Location | Content |
|------|----------|---------|
| `chromadb_data/` | `assets/output/memory/spatial_memory/` | ChromaDB persistent store (CLIP embeddings + pose metadata) |
| `visual_memory.pkl` | `assets/output/memory/spatial_memory/` | Pickled images (base64 JPEGs) |

### Query Skills (exposed to agent)

| Skill | What it does |
|-------|--------------|
| `query_by_text(text)` | CLIP text→image search. "kitchen" → frames that look like a kitchen |
| `query_by_image(image)` | Find similar frames by visual similarity |
| `query_by_location(x, y, radius)` | Find frames captured near (x,y) |
| `tag_location(name)` | Save current robot pose as a named waypoint |
| `query_tagged_location(name)` | Look up a saved waypoint by name |
| `add_named_location(name, pos, rot, desc)` | Manually add a location |

---

## 4. Temporal Memory

### TemporalMemory
**File**: `dimos/perception/experimental/temporal_memory/temporal_memory.py`

**What it does**: VLM-powered video understanding. Periodically analyzes windows of frames, identifies entities (people, objects), tracks interactions over time, maintains a rolling natural-language summary.

**Input**: `color_image: In[Image]` — camera feed

### Processing Cycle

Every `stride_s` seconds (configurable):

1. **Extract**: Grab frames from buffer covering the last `window_s` seconds
2. **Stale check**: Skip if scene hasn't changed (hash-based motion detection)
3. **Keyframe selection**: Pick diverse frames via `adaptive_keyframes()` (CLIP-based)
4. **VLM analysis**: Send frames + context prompt → structured JSON response:
   ```json
   {
     "entities_present": ["E1", "E3"],
     "new_entities": [{"entity_id": "E5", "type": "person", "descriptor": "man in blue shirt"}],
     "relations": [{"type": "holds", "subject": "E1", "object": "E2", "confidence": 0.9}],
     "on_screen_text": ["EXIT"]
   }
   ```
5. **Update state**: Add new entities to roster, update presence, append to chunk buffer
6. **Distance estimation** (background thread): VLM estimates spatial distances between entity pairs
7. **Rolling summary**: When enough chunks accumulate, VLM generates a condensed summary (≤120 words)

### Configuration (`TemporalMemoryConfig`)
```
fps: float              # frame capture rate
window_s: float         # analysis window duration
stride_s: float         # time between analyses
summary_interval_s      # how often to update rolling summary
max_frames_per_window   # max keyframes sent to VLM
max_tokens: 900         # VLM response limit
temperature: 0.2        # VLM temperature
persistent_memory: bool # keep DB across restarts
enable_distance_estimation: bool
output_dir: str         # default: "assets/temporal_memory"
```

### Internal State (thread-safe)
```python
{
    "entity_roster": [                    # all known entities
        {"entity_id": "E1", "type": "person", "descriptor": "woman in red jacket"},
        {"entity_id": "E2", "type": "object", "descriptor": "coffee mug"}
    ],
    "rolling_summary": "A woman entered...",  # condensed recent activity
    "chunk_buffer": [...],                     # recent window results pending summary
    "last_present": ["E1", "E3"],              # entities in last window
    "next_summary_at_s": 120.0                 # when to next summarize
}
```

### What Gets Saved

| File | Content |
|------|---------|
| `evidence.jsonl` | One JSON line per window analysis (full VLM response + metadata) |
| `state.json` | Current state snapshot (entity roster, rolling summary) |
| `entities.json` | Entity roster with descriptions |
| `frames_index.jsonl` | Frame timestamps and indices |
| `entity_graph.db` | SQLite knowledge graph (see below) |

All saved to `assets/temporal_memory/` by default.

### EntityGraphDB
**File**: `dimos/perception/experimental/temporal_memory/entity_graph_db.py`

SQLite database with **3 interconnected graphs** sharing entity nodes:

#### Entities Table
```sql
entity_id    TEXT PRIMARY KEY  -- "E1", "E2", etc.
entity_type  TEXT              -- "person" | "object" | "screen" | "text" | "location"
descriptor   TEXT              -- "woman in red jacket"
first_seen_ts REAL             -- timestamp when first appeared
last_seen_ts  REAL             -- timestamp when last seen
metadata     TEXT              -- JSON blob
```

#### Graph 1: Relations (interactions)
```sql
subject_id  → entity_id
object_id   → entity_id
relation_type: "holds" | "looks_at" | "speaks_to" | "walks_past" | "uses" | "gesture"
confidence: REAL
timestamp_s: REAL
evidence: TEXT               -- VLM reasoning
```

#### Graph 2: Distances (spatial)
```sql
entity_a_id ↔ entity_b_id
distance_meters: REAL
category: "near" (<1m) | "medium" (1-3m) | "far" (>3m)
confidence: REAL
method: "vlm" | "depth" | "bbox"
```

#### Graph 3: Semantic (conceptual)
```sql
entity_a_id ↔ entity_b_id
relation_type: "goes_with" | "opposite_of" | "part_of" | "used_for"
observation_count: INT       -- increases on repeated observations
confidence: REAL
```

#### Key Query Methods
| Method | Returns |
|--------|---------|
| `get_entity_neighborhood(id, max_hops)` | BFS: entity + all neighbors across all graphs |
| `get_nearby_entities(id, max_distance)` | Spatially close entities |
| `get_relations_for_entity(id, type, time_window)` | Interaction history |
| `get_semantic_relations(id, type)` | Conceptual connections |

### Query Skill (the agent's main interface)

```python
@skill
def query(self, question: str) -> str:
```

When the agent calls `query("What has the person in blue been doing?")`:

1. Extract time window from question (e.g., "last hour" → 3600s)
2. Read current state: entity roster, rolling summary, recent windows
3. Build graph context:
   - Entity timestamps (visibility duration)
   - Recent relations (interactions)
   - Nearby entities (distances)
   - Semantic relations
4. Send to VLM: latest frame + full context prompt + question
5. Return: natural language answer

---

## 5. How the Agent Accesses Everything

### Skill Discovery
**File**: `dimos/core/module.py` (lines 376-387)

Every DimOS Module can expose methods as agent-callable tools using the `@skill` decorator (`dimos/agents/annotation.py`). At startup:

1. Agent calls `get_skills()` RPC on every deployed module
2. Each `@skill` method is wrapped as a LangChain `StructuredTool`
3. Tools are passed to the LLM (Claude/GPT) as available function calls

### Agent (Direct RPC)
**File**: `dimos/agents/agent.py`

- Discovers skills from all modules via `_get_tools_from_modules()`
- Wraps each as `StructuredTool` via `_skill_to_tool()`
- When LLM decides to call a tool, it invokes the skill via RPC
- Results (including images via `agent_encode()`) are added to conversation history

### MCP Agent (HTTP JSON-RPC)
**File**: `dimos/agents/mcp/mcp_client.py` + `mcp_server.py`

- **McpServer** collects all module skills, serves them at `http://localhost:9990/mcp`
- **McpClient** (the agent) calls `tools/list` to discover, `tools/call` to invoke
- Same result — just over HTTP instead of direct RPC
- This is what the Mission Control dashboard's Claude Chat uses via `/api/chat`

### VLMAgent
**File**: `dimos/agents/vlm_agent.py`

- Subscribes to `color_image`, keeps `_latest_image`
- `query(text)` → sends latest frame + text to VLM → returns answer
- Used for on-demand "what do you see?" queries
- **Not** used by the main Agent directly — TemporalMemory has its own VLM

---

## 6. Blueprint Wiring (How Modules Connect)

Blueprints define which modules run together. `autoconnect()` wires matching In/Out port names.

### Basic
```
unitree_go2_basic = autoconnect(Go2, WebsocketVis, ...)
  → camera frames, navigation, web dashboard
```

### Spatial
```
unitree_go2_spatial = autoconnect(unitree_go2, SpatialMemory)
  → adds CLIP-based spatial memory
  → color_image → SpatialMemory.color_image
```

### Agentic
```
unitree_go2_agentic = autoconnect(unitree_go2_spatial, Agent, skills...)
  → adds LLM agent with tool access to SpatialMemory + navigation skills
```

### Temporal Memory
```
unitree_go2_temporal_memory = autoconnect(unitree_go2_agentic, TemporalMemory)
  → adds VLM-powered temporal entity tracking
  → color_image → TemporalMemory.color_image
  → agent can call TemporalMemory.query()
```

### MCP (what we run for hackathon)
```
unitree_go2_agentic_mcp = autoconnect(unitree_go2_spatial, McpServer, McpClient, LightSkill, ...)
  → skills exposed via HTTP MCP server (port 9990)
  → McpClient agent calls tools via JSON-RPC
  → Mission Control dashboard talks to this
```

---

## 7. Key File Reference

### Perception
| File | What |
|------|------|
| `dimos/perception/detection/module2D.py` | 2D detection module (YOLO) |
| `dimos/perception/detection/module3D.py` | 3D detection (2D + pointcloud) |
| `dimos/perception/detection/person_tracker.py` | Single-person 3D pose tracker |
| `dimos/perception/detection/detectors/yolo.py` | YOLO11n detector wrapper |
| `dimos/perception/detection/detectors/person/yolo.py` | YOLO11n-pose (keypoints) |
| `dimos/perception/detection/type/detection2d/bbox.py` | Detection2DBBox type |
| `dimos/perception/detection/type/detection2d/person.py` | Detection2DPerson (+ keypoints) |

### ReID
| File | What |
|------|------|
| `dimos/perception/detection/reid/embedding_id_system.py` | Core ReID logic (embedding matching) |
| `dimos/perception/detection/reid/module.py` | ReidModule (DimOS module wrapper) |
| `dimos/perception/detection/reid/type.py` | IDSystem protocol |

### Memory
| File | What |
|------|------|
| `dimos/perception/spatial_perception.py` | SpatialMemory (ChromaDB + CLIP) |
| `dimos/perception/experimental/temporal_memory/temporal_memory.py` | TemporalMemory (VLM analysis) |
| `dimos/perception/experimental/temporal_memory/entity_graph_db.py` | EntityGraphDB (SQLite graphs) |
| `dimos/perception/experimental/temporal_memory/temporal_utils/prompts.py` | VLM prompt templates |
| `dimos/perception/experimental/temporal_memory/temporal_utils/state.py` | State management |
| `dimos/perception/experimental/temporal_memory/temporal_utils/graph_utils.py` | Graph context builder |

### Embedding Models
| File | What |
|------|------|
| `dimos/models/embedding/treid.py` | TorchReIDModel (OSNet, 512-dim, person ReID) |
| `dimos/models/embedding/clip.py` | CLIPModel (vision-language, 512-dim) |
| `dimos/models/embedding/mobileclip.py` | MobileCLIPModel (lightweight CLIP) |

### Agent
| File | What |
|------|------|
| `dimos/agents/agent.py` | Main Agent (LangChain, direct RPC) |
| `dimos/agents/vlm_agent.py` | VLMAgent (camera + text → answer) |
| `dimos/agents/mcp/mcp_server.py` | MCP server (port 9990, JSON-RPC) |
| `dimos/agents/mcp/mcp_client.py` | MCP client agent |
| `dimos/agents/annotation.py` | `@skill` decorator |
| `dimos/agents/skills/navigation.py` | Navigation skills (navigate_with_text, tag_location) |
| `dimos/agents/skills/light_skill.py` | Hackathon light control skill |

### Blueprints
| File | What |
|------|------|
| `dimos/robot/unitree/go2/blueprints/basic/unitree_go2_basic.py` | Basic (camera + nav + web) |
| `dimos/robot/unitree/go2/blueprints/smart/unitree_go2_spatial.py` | + SpatialMemory |
| `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic.py` | + Agent |
| `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_temporal_memory.py` | + TemporalMemory |
| `dimos/robot/unitree/go2/blueprints/agentic/unitree_go2_agentic_mcp.py` | + MCP (hackathon) |
