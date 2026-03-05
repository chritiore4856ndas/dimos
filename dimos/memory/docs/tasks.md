# Memory2 — Remaining Tasks

Gap analysis between `plans/memory/` specs and `dimos/memory/` implementation.

## P0 — Security / Correctness

### 1. Stream name validation

Stream names are interpolated directly into SQL via f-strings. No validation exists — arbitrary input is a SQL injection vector.

**Spec** (`sqlite.md`): `^[a-zA-Z_][a-zA-Z0-9_]{0,63}$`, reject with `ValueError`.

**Where**: Add a `_validate_stream_name(name)` check at the top of `SqliteSession.stream()`, `.text_stream()`, `.embedding_stream()`.

### 2. `_clone()` type annotation vs runtime

`Stream._clone()` (`stream.py:94-108`) is annotated `-> Stream[T]`, but at runtime it uses `self.__class__.__new__(self.__class__)` which correctly preserves the subclass. So `EmbeddingStream.after(t)` returns an `EmbeddingStream` at runtime — no bug.

The annotation is wrong for mypy though. Consider `-> Self` (from `typing_extensions`) if we want strict typing. Low priority — runtime works.

## P1 — Core API Gaps

### 3. Wire `parent_stream` into `_streams` registry

`_register_stream()` (`sqlite.py:847-861`) never writes the `parent_stream` column. The column exists in the schema but is always NULL.

**Where**: `materialize_transform()` (`sqlite.py:770-799`) knows both `source_table` and `name`. Pass `parent_stream=source_table` to `_register_stream()`, and update `_register_stream` to accept and INSERT it.

This is a prerequisite for `.join()` and stream-level lineage discovery.

### 4. ~~Implement `.project_to()` — cross-stream lineage~~ ✅

Implemented. `project_to(target)` adds a `LineageFilter` to the target stream (same `_with_filter` mechanism as `.after()`, `.near()`, etc.). The filter compiles to a SQL subquery walking the `parent_id` chain. Multi-hop lineage is resolved via `_streams.parent_stream` registry. Result is a fully chainable `Stream`.

### 4b. Implement `.join()` — cross-stream lineage returning pairs

`api.md` specifies:
```python
for det, img in detections.after(t).join(images):
    print(f"Detected {det.data} in image at {img.pose}")
```

Unlike `project_to()` which returns a `Stream`, `join()` yields `tuple[Observation, Observation]` pairs. This is a terminal operation (not chainable) since the return type is pairs, not observations.

**Depends on**: ~~Task 3~~ Done — `parent_stream` is now written by `materialize_transform()` and read by `resolve_lineage_chain()`.

### 5. Filtered `.appended` — predicate-filtered reactive subscriptions

`api.md` specifies:
```python
images.near(kitchen_pose, 5.0).appended.subscribe(...)
```

Current impl (`stream.py:276-278`) returns the raw Subject regardless of filters.

**Fix** (from `sqlite.md`): When `self._query.filters` is non-empty, pipe the root subject through `ops.filter()` that evaluates each predicate in Python:

```python
@property
def appended(self):
    backend = self._require_backend()
    obs = backend.appended_subject
    if not self._query.filters:
        return obs
    return obs.pipe(ops.filter(lambda o: self._matches_filters(o)))
```

Each filter type needs a `matches(obs) -> bool` method for Python-side evaluation:
- `AfterFilter`: `obs.ts > self.t`
- `NearFilter`: Euclidean distance check
- `TagsFilter`: dict subset check
- etc.

### 6. Incremental backfill

`sqlite.md` specifies that re-running a stored transform resumes from the last processed item:

```python
max_parent = conn.execute(
    f"SELECT MAX(parent_id) FROM {target_name}"
).fetchone()[0]

if max_parent is not None:
    source = source.after_id(max_parent)  # internal: WHERE id > ?
```

**Where**: `materialize_transform()` (`sqlite.py:791-793`). Before calling `transformer.process()`, check if target already has rows and filter source accordingly.

**Needs**: An internal `_after_id(row_id)` filter (not exposed in public API) that adds `WHERE id > ?`.

## P2 — Robustness

### 7. Separate connections per session

`SqliteStore.session()` (`sqlite.py:886-887`) shares `self._conn` across all sessions. The spec says each session should own its own connection.

**Fix**: `session()` should call `sqlite3.connect(self._path)` + WAL pragma + extension loading each time, not reuse `self._conn`. Store keeps the path, sessions get independent connections.

This is required for multi-threaded use (e.g., one session writing in a background thread, another querying in the main thread).

### 8. `_CollectorStream` doesn't set pose on observations

`_CollectorStream.append()` (`stream.py:401-419`) accepts `pose` but doesn't store it on the `Observation`:

```python
obs = Observation(id=self._next_id, ts=ts, tags=tags or {}, parent_id=parent_id, _data=payload)
# pose is silently dropped
```

**Fix**: Add `pose=pose` to the Observation constructor call.

## P3 — Future (not blocking)

### 9. Query objects — composable 4D regions + soft scoring

`query_objects.md` proposes `Criterion` types (`TimeRange`, `Sphere`, `TimeProximity`, `SpatialProximity`, `EmbeddingSimilarity`) with `&`/`|`/`~` composition and weighted `Score()`.

Explicitly Phase 2. Current flat filter API covers all simple cases. Implement when real usage demands soft scoring or region composition.

### 10. `questions.md` hard cases

Unresolved query patterns from the product requirements:
- Negation queries ("when did I NOT see the cat")
- Temporal regularity ("what time does the mailman come")
- Cross-agent memory diff
- Conditional pose integration
- Event-anchored multi-stream slicing

These require extensions beyond the current Stream API — likely built on top of the composable query layer (task 9).
