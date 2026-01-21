# Copyright 2025-2026 Dimensional Inc.
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
"""Unified timestamped sensor storage and replay."""

from abc import ABC, abstractmethod
from collections.abc import Iterator
from typing import Generic, TypeVar

from reactivex.observable import Observable

T = TypeVar("T")


class SensorStore(Generic[T], ABC):
    """Unified storage + replay for timestamped sensor data.

    Implement 4 abstract methods for your backend (in-memory, pickle, sqlite, etc.).
    All iteration, streaming, and seek logic comes free from the base class.
    """

    # === Abstract - implement for your backend ===

    @abstractmethod
    def _save(self, timestamp: float, data: T) -> None:
        """Save data at timestamp."""
        ...

    @abstractmethod
    def _load(self, timestamp: float) -> T | None:
        """Load data at exact timestamp. Returns None if not found."""
        ...

    @abstractmethod
    def _iter_items(
        self, start: float | None = None, end: float | None = None
    ) -> Iterator[tuple[float, T]]:
        """Lazy iteration of (timestamp, data) in range."""
        ...

    @abstractmethod
    def _find_closest_timestamp(
        self, timestamp: float, tolerance: float | None = None
    ) -> float | None:
        """Find closest timestamp. Backend can optimize (binary search, db index, etc.)."""
        ...

    def save(self, data: T, timestamp: float | None = None) -> None:
        """Save data. Uses data.ts if available, otherwise timestamp arg, otherwise now."""
        import time

        if timestamp is None:
            if hasattr(data, "ts"):
                timestamp = data.ts  # type: ignore[union-attr]
            else:
                timestamp = time.time()
        self._save(timestamp, data)

    def load(self, timestamp: float) -> T | None:
        """Load data at exact timestamp."""
        return self._load(timestamp)

    def find_closest(
        self,
        timestamp: float | None = None,
        seek: float | None = None,
        tolerance: float | None = None,
    ) -> T | None:
        """Find data closest to timestamp (absolute) or seek (relative to start)."""
        ...

    def first_timestamp(self) -> float | None:
        """Get the first timestamp in the store."""
        ...

    def iterate(self, loop: bool = False) -> Iterator[tuple[float, T]]:
        """Iterate over (timestamp, data) pairs."""
        ...

    def iterate_ts(
        self,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Iterator[tuple[float, T]]:
        """Iterate with seek/duration options."""
        ...

    def iterate_realtime(self, speed: float = 1.0, **kwargs) -> Iterator[T]:
        """Iterate data, sleeping to match original timing."""
        ...

    def stream(
        self,
        speed: float = 1.0,
        seek: float | None = None,
        duration: float | None = None,
        from_timestamp: float | None = None,
        loop: bool = False,
    ) -> Observable[T]:
        """Stream data as Observable with timing control."""
        ...


class InMemoryStore(SensorStore[T]):
    """In-memory storage using dict. Good for live use."""

    def __init__(self) -> None:
        self._data: dict[float, T] = {}
        self._sorted_timestamps: list[float] | None = None

    def _save(self, timestamp: float, data: T) -> None:
        self._data[timestamp] = data
        self._sorted_timestamps = None  # Invalidate cache

    def _load(self, timestamp: float) -> T | None:
        return self._data.get(timestamp)

    def _iter_items(
        self, start: float | None = None, end: float | None = None
    ) -> Iterator[tuple[float, T]]:
        for ts in self._get_sorted_timestamps():
            if start is not None and ts < start:
                continue
            if end is not None and ts >= end:
                break
            yield (ts, self._data[ts])

    def _find_closest_timestamp(
        self, timestamp: float, tolerance: float | None = None
    ) -> float | None:
        import bisect

        timestamps = self._get_sorted_timestamps()
        if not timestamps:
            return None

        pos = bisect.bisect_left(timestamps, timestamp)

        candidates = []
        if pos > 0:
            candidates.append(timestamps[pos - 1])
        if pos < len(timestamps):
            candidates.append(timestamps[pos])

        if not candidates:
            return None

        closest = min(candidates, key=lambda ts: abs(ts - timestamp))

        if tolerance is not None and abs(closest - timestamp) > tolerance:
            return None

        return closest

    def _get_sorted_timestamps(self) -> list[float]:
        if self._sorted_timestamps is None:
            self._sorted_timestamps = sorted(self._data.keys())
        return self._sorted_timestamps
