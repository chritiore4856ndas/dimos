# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from __future__ import annotations

import json
from typing import TYPE_CHECKING

from dimos.memory2.backend import VectorStore

if TYPE_CHECKING:
    import sqlite3

    from dimos.models.embedding.base import Embedding


class SqliteVectorStore(VectorStore):
    """Vector store backed by sqlite-vec's vec0 virtual tables.

    Creates one virtual table per stream: ``"{stream}_vec"``.
    Dimensionality is determined lazily on the first ``put()``.

    Does NOT own the connection — lifecycle managed externally.
    """

    def __init__(self, conn: sqlite3.Connection) -> None:
        self._conn = conn
        self._tables: dict[str, int] = {}  # stream -> dimensionality

    def _ensure_table(self, stream: str, dim: int) -> None:
        if stream in self._tables:
            return
        self._conn.execute(
            f'CREATE VIRTUAL TABLE IF NOT EXISTS "{stream}_vec" '
            f"USING vec0(embedding float[{dim}] distance_metric=cosine)"
        )
        self._tables[stream] = dim

    # ── Resource lifecycle ────────────────────────────────────────

    def start(self) -> None:
        pass

    def stop(self) -> None:
        pass

    # ── VectorStore interface ────────────────────────────────────

    def put(self, stream: str, key: int, embedding: Embedding) -> None:
        vec = embedding.to_numpy().tolist()
        self._ensure_table(stream, len(vec))
        self._conn.execute(
            f'INSERT OR REPLACE INTO "{stream}_vec" (rowid, embedding) VALUES (?, ?)',
            (key, json.dumps(vec)),
        )

    def search(self, stream: str, query: Embedding, k: int) -> list[tuple[int, float]]:
        if stream not in self._tables:
            return []
        vec = query.to_numpy().tolist()
        rows = self._conn.execute(
            f'SELECT rowid, distance FROM "{stream}_vec" WHERE embedding MATCH ? AND k = ?',
            (json.dumps(vec), k),
        ).fetchall()
        # vec0 cosine distance = 1 - cosine_similarity
        return [(int(row[0]), max(0.0, 1.0 - row[1])) for row in rows]

    def delete(self, stream: str, key: int) -> None:
        if stream not in self._tables:
            return
        self._conn.execute(f'DELETE FROM "{stream}_vec" WHERE rowid = ?', (key,))
