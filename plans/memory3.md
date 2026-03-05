# Memory2 Implementation Plan
## Context

Check `questions.md`

## File Structure

```
dimos/memory2/
    __init__.py              # public exports (re-exports from API + default backend)
    types.py                 # ObservationRow, Lineage, StreamInfo
    store.py                 # Store ABC (Resource lifecycle)
    session.py               # Session ABC (stream factory)
    stream.py                # StreamBase, BlobStream, EmbeddingStream, TextStream ABCs
    query.py                 # Query ABC (filter/search/rank/limit → fetch/fetch_set)
    observation_set.py       # ObservationSet ABC

    impl/
        sqlite/
            __init__.py      # exports SqliteStore
            store.py         # SqliteStore (connection, WAL, extension loading)
            session.py       # SqliteSession (stream factory, _streams registry)
            stream.py        # SqliteBlobStream, SqliteEmbeddingStream, SqliteTextStream
            query.py         # SqliteQuery (SQL generation, execution)
            observation_set.py # SqliteObservationSet (predicate/ref-table backing)
            _sql.py          # SQL helpers, identifier validation, schema DDL

    test_memory2.py          # tests (against SqliteStore)
```

## API Layer (`dimos/memory2/`)

### `types.py` — Data classes

```python
@dataclass
class ObservationRow:
    id: int
    ts: float | None = None
    pose: PoseStamped | None = None
    tags: dict[str, Any] = field(default_factory=dict)
    _data: Any = field(default=None, repr=False)
    _load: Callable[[], Any] | None = field(default=None, repr=False)

    @property
    def data(self) -> Any:
        """Lazy payload access. Pre-populated for appended events, fetched on demand for query results."""
        if self._data is None and self._load is not None:
            self._data = self._load()
        return self._data

@dataclass
class Lineage:
    parent_stream: str | None = None   # from _streams registry (stream-level)
    parent_id: int | None = None       # per-row: which row in parent stream

@dataclass
class StreamInfo:
    name: str
    payload_type: type
    parent_stream: str | None     # lineage: all rows derive from this stream
    count: int
```

Poses use DimOS's existing `PoseLike` type alias (`Pose | PoseStamped | Point | PointStamped`). No custom Pose type.

### `store.py` — Store ABC

```python
class Store(Resource, ABC):
    def session(self) -> Session: ...
    def close(self) -> None: ...
    def start(self) -> None: pass
    def stop(self) -> None: self.close()
```

### `session.py` — Session ABC

```python
PoseProvider = Callable[[], PoseLike | None]

class Session(ABC):
    def stream(self, name: str, payload_type: type, *,
               pose_provider: PoseProvider | None = None) -> BlobStream: ...
    def embedding_stream(self, name: str, *,
                         model: EmbeddingModel) -> EmbeddingStream: ...
    def text_stream(self, name: str, payload_type: type, *,
                    tokenizer: str = "unicode61",
                    pose_provider: PoseProvider | None = None) -> TextStream: ...
    def list_streams(self) -> list[StreamInfo]: ...
    def close(self) -> None: ...
    def __enter__ / __exit__
```

### `stream.py` — Stream hierarchy ABCs

```python
class StreamBase(ABC, Generic[T]):
    """Abstract base. No text/vector indexes."""
    pose_provider: PoseProvider | None = None  # auto-fills pose on append if set

    # Write
    def append(self, payload: T, *,
               ts: float | None = None,      # defaults to time.time()
               pose: PoseLike | None = None,  # explicit pose overrides provider
               tags: dict[str, Any] | None = None,
               parent_id: int | None = None,
               ) -> ObservationRow: ...        # returned row has .data pre-populated

    # Reactive
    @property
    def appended(self) -> Observable[ObservationRow]: ...  # .data pre-populated

    # Read
    def query(self) -> Query[T]: ...
    def load(self, row: ObservationRow) -> T: ...
    def load_many(self, rows: list[ObservationRow], *, batch_size=32) -> list[T]: ...
    def count(self) -> int: ...

class BlobStream(StreamBase[T]):
    """Stream for arbitrary serializable payloads. No special indexes."""

class EmbeddingStream(StreamBase[T]):
    """Stream with vector index. No payload table — the vector IS the data."""
    model: EmbeddingModel

    def attach(self, parent: StreamBase) -> Self:
        """Sets lineage parent + subscribes to parent.appended to auto-embed."""
        # parent.appended.pipe(
        #     ops.map(lambda row: self._embed_and_store(row)),
        # ).subscribe()
        ...

    def vector(self, row: ObservationRow) -> list[float] | None: ...

class TextStream(StreamBase[T]):
    """Stream with FTS index."""
```

### `query.py` — Query ABC

```python
class Query(ABC, Generic[T]):
    # Hard filters
    def time_range(self, t1: float, t2: float) -> Query[T]: ...
    def before(self, t: float) -> Query[T]: ...
    def after(self, t: float) -> Query[T]: ...

    def filter_tags(self, **tags: Any) -> Query[T]: ...
    def at(self, t: float, *, tolerance: float = 1.0) -> Query[T]: ...

    # Candidate generation (raise TypeError if stream lacks the required index)
    def search_text(self, text: str, *, candidate_k: int | None = None) -> Query[T]: ...
    def search_embedding(self, vector: list[float], *, candidate_k: int) -> Query[T]: ...

    def order_by(self, field: str, *, desc: bool = False) -> Query[T]: ...
    def limit(self, k: int) -> Query[T]: ...

    # Terminals
    def fetch(self) -> list[ObservationRow]: ...
    def fetch_set(self) -> ObservationSet[T]: ...
    def count(self) -> int: ...
    def one(self) -> ObservationRow: ...
    def last(self) -> ObservationRow: ...
```

TODO: terminals that generate spatial or temporal summaries (maybe as numpy arrays).

### `observation_set.py` — ObservationSet ABC

```python
class ObservationSet(ABC, Generic[T]):
    # Re-query
    def query(self) -> Query[T]: ...

    # Read
    def load(self, row: ObservationRow) -> T: ...
    def load_many(self, rows, *, batch_size=32) -> list[T]: ...
    def rows(self, *, limit=None) -> list[ObservationRow]: ...
    def one(self) -> ObservationRow: ...
    def fetch_page(self, *, limit=128, offset=0) -> list[ObservationRow]: ...
    def count(self) -> int: ...
    def lineage(self) -> Lineage: ...

    # Cross-stream
    def project_to(self, stream: StreamBase) -> ObservationSet: ...

    # Cleanup
    def close(self) -> None: ...
    def __enter__(self) -> Self: ...
    def __exit__(self, *exc) -> None: ...
    def __del__(self) -> None: ...  # best-effort fallback
```

---

## SQLite Implementation (`dimos/memory2/impl/sqlite/`)

### `store.py` — SqliteStore

- Stores file path, creates parent dirs on connect
- `_connect()`: `sqlite3.connect()`, WAL mode, loads sqlite-vec (optional), loads FTS5
- Tracks sessions via `WeakSet` for cleanup
- `:memory:` uses `file::memory:?cache=shared` URI
- Thread safety: each session = one connection, no `check_same_thread=False`

### `session.py` — SqliteSession

- Holds one `sqlite3.Connection`
- `stream()` / `embedding_stream()` / `text_stream()`: creates tables if needed, caches stream instances
- Registers stream metadata in a `_streams` registry table:

```sql
CREATE TABLE _streams (
    rowid INTEGER PRIMARY KEY,
    name TEXT UNIQUE NOT NULL,
    type TEXT NOT NULL,           -- 'blob', 'embedding', 'text'
    payload_type TEXT NOT NULL,
    parent_stream_id INTEGER      -- FK to _streams.rowid (lineage)
);
```

### `stream.py` — SqliteBlobStream, SqliteEmbeddingStream, SqliteTextStream

`append()` inserts a metadata row (SQLite auto-assigns `rowid`), serializes payload into `_payload`, and inserts an R*Tree entry if pose is provided. `EmbeddingStream.append()` inserts into `_vec` only (no `_payload`). `TextStream.append()` inserts into both `_payload` (as TEXT) and `_fts`. Returns `ObservationRow` with `id`, `ts`, `pose`, `tags` populated.

### `query.py` — SqliteQuery

- Accumulates filter predicates, search ops, rank spec, ordering, limit
- `at(t, tolerance)` → sugar for `filter_time(t - tol, t + tol)` + `ORDER BY ABS(ts - t) LIMIT 1`
- `order_by(field, desc)` → appends `ORDER BY` clause; valid fields: `ts`
- `fetch()`: generates SQL, executes, returns rows
- `fetch_set()`: creates ObservationSet (predicate-backed or ref-table-backed)
- `search_embedding` → sqlite-vec `MATCH`, writes top-k to temp table → ref-table-backed
- `search_text` → FTS5 `MATCH`
- `filter_near` → R*Tree range query
- `rank` → computes composite score from available score columns

### `observation_set.py` — SqliteObservationSet

Internal backing:

```python
@dataclass
class PredicateBacking:
    """Lazy: expressible as SQL WHERE over source stream."""
    source_name: str
    query_repr: str  # serialized query filters for replay

@dataclass
class RefTableBacking:
    """Materialized: temp table of refs + scores."""
    table_name: str  # SQLite temp table
    source_streams: list[str]
    ordered: bool = False
```

- `.query()` on predicate-backed → adds more predicates
- `.query()` on ref-table-backed → filters within that temp table
- `project_to()` → joins backing refs via lineage parent_rowid to target stream
- `close()` drops the temp table for ref-table-backed sets; no-op for predicate-backed
- Supports context manager (`with`) for deterministic cleanup; `__del__` as fallback
- SQLite connection close is the final safety net for any leaked temp tables

### `_sql.py` — SQL helpers

```python
def validate_identifier(name: str) -> str: ...  # regex check, length limit
```

Pose extraction: `_extract_pose(p: PoseLike) -> tuple[float, ...]` pulls `(x, y, z, qx, qy, qz, qw)`. `_reconstruct_pose(row) -> Pose` rebuilds from stored floats.

Payload serialization: `lcm_encode(payload)` / `lcm_decode(blob, payload_type)`. Non-LCM types rejected at `append()` with `TypeError`.

---

### Schema (per stream)

**`{name}_meta`** — metadata for all stream types:
```sql
CREATE TABLE {name}_meta (
    rowid INTEGER PRIMARY KEY,
    ts REAL,
    pose_x REAL, pose_y REAL, pose_z REAL,
    pose_qx REAL, pose_qy REAL, pose_qz REAL, pose_qw REAL,
    tags TEXT,                     -- JSON
    parent_rowid INTEGER           -- lineage: rowid in parent stream
);
CREATE INDEX idx_{name}_meta_ts ON {name}_meta(ts);
```

**`{name}_payload`** — BlobStream and TextStream only (not EmbeddingStream):
```sql
CREATE TABLE {name}_payload (
    rowid INTEGER PRIMARY KEY,    -- matches _meta.rowid
    data BLOB NOT NULL            -- TextStream stores TEXT here instead of BLOB
);
```

**`{name}_rtree`** — all stream types (rows with pose only):
```sql
CREATE VIRTUAL TABLE {name}_rtree USING rtree(
    rowid,
    min_t, max_t,                 -- both set to ts
    min_x, max_x, min_y, max_y, min_z, max_z  -- both set to pose_xyz
);
```

**`{name}_fts`** — TextStream only:
```sql
CREATE VIRTUAL TABLE {name}_fts USING fts5(content);
```

**`{name}_vec`** — EmbeddingStream only:
```sql
CREATE VIRTUAL TABLE {name}_vec USING vec0(embedding float[{dim}]);
```

All virtual table rowids match `_meta.rowid` directly.

---

## Phase 3: Later (not in first PR)

- `derive()` with Transform protocol
- `CompositeBacking` (union/intersection/difference)
- `Correlator` / `s.correlate()`
- `retention` enforcement / cleanup
- Full introspection (stats, spatial_bounds)

## Design Decisions

### API-level

- **Poses**: all pose params accept `PoseLike` (`Pose | PoseStamped | Point | PointStamped`). No custom pose type.
- **Row identity**: `id` is auto-assigned integer per stream. Unique within a stream. Impl layer maps to SQLite `rowid`.
- **Unlocalized observations**: rows without pose excluded from `filter_near()` by default. `include_unlocalized=True` to include them.
- **Stream hierarchy**: `StreamBase` (ABC) → `BlobStream`, `EmbeddingStream`, `TextStream`. Indexing is determined by stream type, not config.
- **Lineage**: parent stream defined at stream level (in `_streams` registry). Per-row `parent_id` links to specific row in parent.

### SQLite-specific

- **Separate payload table**: `_payload` separate from `_meta` so queries never page in multi-MB blobs.
- **EmbeddingStream has no payload table**: the vector in `_vec` IS the data.
- **R*Tree for spatio-temporal**: time-only queries use B-tree index on `_meta.ts` (faster for 1D). Spatial/spatio-temporal queries use R*Tree.
- **Payload serialization**: `lcm_encode()` / `lcm_decode()`. Non-LCM types rejected at `append()` with `TypeError`.
- **ObservationSet cleanup**: ref-table-backed sets use SQLite temp tables. Cleaned via context manager, `__del__` fallback, or connection close.
