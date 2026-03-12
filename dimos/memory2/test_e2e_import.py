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

"""E2E test: import legacy pickle replays into memory SqliteStore."""

from __future__ import annotations

import bisect
from typing import TYPE_CHECKING, Any

import pytest

from dimos.memory2.impl.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data_dir
from dimos.utils.testing import TimedSensorReplay

if TYPE_CHECKING:
    from collections.abc import Generator

    from dimos.memory2.impl.sqlite import SqliteSession

DB_PATH = get_data_dir("go2_bigoffice_v2.db")


class PoseIndex:
    """Preloaded odom data with O(log n) closest-timestamp lookup."""

    def __init__(self, replay: TimedSensorReplay) -> None:  # type: ignore[type-arg]
        self._timestamps: list[float] = []
        self._data: list[Any] = []
        for ts, data in replay.iterate_ts():
            self._timestamps.append(ts)
            self._data.append(data)

    def find_closest(self, ts: float) -> Any | None:
        if not self._timestamps:
            return None
        idx = bisect.bisect_left(self._timestamps, ts)
        # Compare the two candidates around the insertion point
        if idx == 0:
            return self._data[0]
        if idx >= len(self._timestamps):
            return self._data[-1]
        if ts - self._timestamps[idx - 1] <= self._timestamps[idx] - ts:
            return self._data[idx - 1]
        return self._data[idx]


@pytest.fixture(scope="module")
def store() -> Generator[SqliteStore, None, None]:
    s = SqliteStore(path=str(DB_PATH))
    yield s


@pytest.fixture(scope="module")
def session(store: SqliteStore) -> Generator[SqliteSession, None, None]:
    with store.session() as session:
        yield session


@pytest.fixture(scope="module")
def video_replay() -> TimedSensorReplay:  # type: ignore[type-arg]
    return TimedSensorReplay("unitree_go2_bigoffice/video")


@pytest.fixture(scope="module")
def odom_index() -> PoseIndex:
    return PoseIndex(TimedSensorReplay("unitree_go2_bigoffice/odom"))


@pytest.fixture(scope="module")
def lidar_replay() -> TimedSensorReplay:  # type: ignore[type-arg]
    return TimedSensorReplay("unitree_go2_bigoffice/lidar")


@pytest.mark.tool
class TestImportReplay:
    """Import legacy pickle replay data into a memory SqliteStore."""

    def test_import_video(
        self,
        session: SqliteSession,
        video_replay: TimedSensorReplay,  # type: ignore[type-arg]
        odom_index: PoseIndex,
    ) -> None:
        video = session.stream("color_image", Image)

        count = 0
        for ts, frame in video_replay.iterate_ts():
            pose = odom_index.find_closest(ts)
            print(frame)
            video.append(frame, ts=ts, pose=pose)
            count += 1

        assert count > 0
        assert video.count() == count
        print(f"Imported {count} video frames")

    def test_import_lidar(
        self,
        session: SqliteSession,
        lidar_replay: TimedSensorReplay,  # type: ignore[type-arg]
        odom_index: PoseIndex,
    ) -> None:
        lidar = session.stream("lidar", PointCloud2)

        count = 0
        for ts, frame in lidar_replay.iterate_ts():
            pose = odom_index.find_closest(ts)
            print(frame)
            lidar.append(frame, ts=ts, pose=pose)
            count += 1

        assert count > 0
        assert lidar.count() == count
        print(f"Imported {count} lidar frames")

    def test_query_imported_data(self, session: SqliteSession) -> None:
        video = session.stream("color_image", Image)
        lidar = session.stream("lidar", PointCloud2)

        assert video.exists()
        assert lidar.exists()

        first_frame = video.first()
        last_frame = video.last()
        assert first_frame.ts < last_frame.ts

        mid_ts = (first_frame.ts + last_frame.ts) / 2
        subset = video.time_range(first_frame.ts, mid_ts).fetch()
        assert 0 < len(subset) < video.count()

        streams = session.list_streams()
        assert "color_image" in streams
        assert "lidar" in streams
