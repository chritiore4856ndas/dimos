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

"""E2E query tests against pre-built go2_bigoffice_v2.db.

Read-only — no writes, just verifies query paths against real robot data.
"""

from __future__ import annotations

from typing import TYPE_CHECKING

import pytest

from dimos.memory2.impl.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.data import get_data

if TYPE_CHECKING:
    from collections.abc import Generator

    from dimos.memory2.impl.sqlite import SqliteSession


@pytest.fixture(scope="module")
def session() -> Generator[SqliteSession, None, None]:
    db_path = get_data("go2_bigoffice_v2.db")
    store = SqliteStore(path=str(db_path))
    with store.session() as s:
        yield s


@pytest.mark.tool
class TestE2EQuery:
    """Query operations against real robot replay data."""

    def test_list_streams(self, session: SqliteSession) -> None:
        streams = session.list_streams()
        print(streams)

        assert "color_image" in streams
        assert "lidar" in streams

    def test_video_count(self, session: SqliteSession) -> None:
        video = session.stream("color_image", Image)
        assert video.count() > 1000

    def test_lidar_count(self, session: SqliteSession) -> None:
        lidar = session.stream("lidar", PointCloud2)
        assert lidar.count() > 1000

    def test_first_last_timestamps(self, session: SqliteSession) -> None:
        video = session.stream("color_image", Image)
        first = video.first()
        last = video.last()
        assert first.ts < last.ts
        duration = last.ts - first.ts
        assert duration > 10.0  # at least 10s of data

    def test_time_range_filter(self, session: SqliteSession) -> None:
        video = session.stream("color_image", Image)
        first = video.first()

        # Grab first 5 seconds
        window = video.time_range(first.ts, first.ts + 5.0).fetch()
        assert len(window) > 0
        assert len(window) < video.count()
        assert all(first.ts <= obs.ts <= first.ts + 5.0 for obs in window)

    def test_limit_offset_pagination(self, session: SqliteSession) -> None:
        video = session.stream("color_image", Image)
        page1 = video.limit(10).fetch()
        page2 = video.offset(10).limit(10).fetch()

        assert len(page1) == 10
        assert len(page2) == 10
        assert page1[-1].ts < page2[0].ts  # no overlap

    def test_order_by_desc(self, session: SqliteSession) -> None:
        video = session.stream("color_image", Image)
        last_10 = video.order_by("ts", desc=True).limit(10).fetch()

        assert len(last_10) == 10
        assert all(last_10[i].ts >= last_10[i + 1].ts for i in range(9))

    def test_lazy_data_loads_correctly(self, session: SqliteSession) -> None:
        """Verify lazy blob loading returns valid Image data."""
        from dimos.memory2.type import _Unloaded

        video = session.stream("color_image", Image)
        obs = next(iter(video.limit(1)))

        # Should start lazy
        assert isinstance(obs._data, _Unloaded)

        # Trigger load
        frame = obs.data
        assert isinstance(frame, Image)
        assert frame.width > 0
        assert frame.height > 0

    def test_iterate_window_decodes_all(self, session: SqliteSession) -> None:
        """Iterate a time window and verify every frame decodes."""
        video = session.stream("color_image", Image)
        first_ts = video.first().ts

        window = video.time_range(first_ts, first_ts + 2.0)
        count = 0
        for obs in window:
            frame = obs.data
            assert isinstance(frame, Image)
            count += 1
        assert count > 0

    def test_lidar_data_loads(self, session: SqliteSession) -> None:
        """Verify lidar blobs decode to PointCloud2."""
        lidar = session.stream("lidar", PointCloud2)
        frame = lidar.first().data
        assert isinstance(frame, PointCloud2)

    def test_poses_present(self, session: SqliteSession) -> None:
        """Verify poses were stored during import."""
        video = session.stream("color_image", Image)
        obs = video.first()
        assert obs.pose is not None

    def test_cross_stream_time_alignment(self, session: SqliteSession) -> None:
        """Video and lidar should overlap in time."""
        video = session.stream("color_image", Image)
        lidar = session.stream("lidar", PointCloud2)

        v_first, v_last = video.first().ts, video.last().ts
        l_first, l_last = lidar.first().ts, lidar.last().ts

        # Overlap: max of starts < min of ends
        overlap_start = max(v_first, l_first)
        overlap_end = min(v_last, l_last)
        assert overlap_start < overlap_end, "Video and lidar should overlap in time"
