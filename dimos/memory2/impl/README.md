# impl — Backend implementations

Storage backends for memory. Each backend implements the `Backend` protocol to provide observation storage with query support. All backends support live mode via a pluggable `LiveChannel`.

## Existing backends

| Backend         | File        | Status   | Storage                             |
|-----------------|-------------|----------|-------------------------------------|
| `ListBackend`   | `memory.py` | Complete | In-memory lists, brute-force search |
| `SqliteBackend` | `sqlite.py` | Stub     | SQLite (WAL, FTS5, vec0)            |

## Writing a new backend

### 1. Implement the Backend protocol

```python
from dimos.memory2.backend import Backend, BackendConfig, LiveChannel
from dimos.memory2.filter import StreamQuery
from dimos.memory2.livechannel.subject import SubjectChannel
from dimos.memory2.type import Observation
from dimos.protocol.service.spec import Configurable

class MyBackend(Configurable[BackendConfig], Generic[T]):
    default_config: type[BackendConfig] = BackendConfig

    def __init__(self, name: str, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._name = name
        self._channel: LiveChannel[T] = self.config.live_channel or SubjectChannel()

    @property
    def name(self) -> str:
        return self._name

    @property
    def live_channel(self) -> LiveChannel[T]:
        return self._channel

    def append(self, obs: Observation[T]) -> Observation[T]:
        """Assign an id and store. Return the stored observation."""
        obs.id = self._next_id
        self._next_id += 1
        # ... persist obs ...
        self._channel.notify(obs)
        return obs

    def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]:
        """Yield observations matching the query."""
        # The backend is responsible for applying ALL query fields:
        #   query.filters      — list of Filter objects (each has .matches(obs))
        #   query.order_field   — sort field name (e.g. "ts")
        #   query.order_desc    — sort direction
        #   query.limit_val     — max results
        #   query.offset_val    — skip first N
        #   query.search_vec    — Embedding for vector search
        #   query.search_k      — top-k for vector search
        #   query.search_text   — substring text search
        #   query.live_buffer   — if set, switch to live mode
        ...

    def count(self, query: StreamQuery) -> int:
        """Count matching observations."""
        ...
```

`Backend` is a `@runtime_checkable` Protocol — no base class needed, just implement the methods.

### 2. Live mode via LiveChannel

Every backend exposes a `live_channel` property. The default `SubjectChannel` handles same-process fan-out. Inject a custom `LiveChannel` (Redis pub/sub, Postgres LISTEN/NOTIFY, etc.) via `BackendConfig.live_channel` for cross-process use.

The `iterate()` method should check `query.live_buffer`:
- If `None`: return a snapshot iterator
- If set: subscribe via `self._channel.subscribe(buf)` before backfill, then yield a live tail that deduplicates by `obs.id`

See `ListBackend._iterate_live()` for the reference implementation.

### 3. Add Store and Session

```python
from dimos.memory2.store import Session, Store

class MySession(Session):
    def _create_backend(
        self, name: str, payload_type: type | None = None, **config: Any
    ) -> Backend:
        return MyBackend(name, **config)

class MyStore(Store):
    def session(self, **kwargs: Any) -> MySession:
        return MySession(**kwargs)
```

### 4. Add to the grid test

In `test_impl.py`, add your store to the fixture so all standard tests run against it:

```python
@pytest.fixture(params=["memory", "sqlite", "mybackend"])
def store(request, tmp_path):
    if request.param == "mybackend":
        return MyStore(...)
    ...
```

Use `pytest.mark.xfail` for features not yet implemented — the grid test covers: append, fetch, iterate, count, first/last, exists, all filters, ordering, limit/offset, embeddings, text search.

### Query contract

The backend must handle the full `StreamQuery`. The Stream layer does NOT apply filters to backend results — it trusts the backend to do so.

`StreamQuery.apply(iterator)` provides a complete Python-side execution path — filters, text search, vector search, ordering, offset/limit — all as in-memory operations. Backends can use it in three ways:

**Full delegation** — simplest, good enough for in-memory backends:
```python
def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]:
    return query.apply(iter(self._data))
```

**Partial push-down** — handle some operations natively, delegate the rest:
```python
def iterate(self, query: StreamQuery) -> Iterator[Observation[T]]:
    # Handle filters and ordering in SQL
    rows = self._sql_query(query.filters, query.order_field, query.order_desc)
    # Delegate remaining operations (vector search, text search, offset/limit) to Python
    remaining = StreamQuery(
        search_vec=query.search_vec, search_k=query.search_k,
        search_text=query.search_text,
        offset_val=query.offset_val, limit_val=query.limit_val,
    )
    return remaining.apply(iter(rows))
```

**Full push-down** — translate everything to native queries (SQL WHERE, FTS5 MATCH, vec0 knn) without calling `apply()` at all.

For filters, each `Filter` object has a `.matches(obs) -> bool` method that backends can use directly if they don't have a native equivalent.
