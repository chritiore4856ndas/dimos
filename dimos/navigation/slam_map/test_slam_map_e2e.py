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

"""E2E tests: SlamMap with replay data.

Verifies that the SlamMap module produces:
  1. A non-empty corrected_global_map (occupied voxels)
  2. Non-degenerate corrected_odom messages

Uses GO2Connection in replay mode with the default dataset, which provides
real lidar (PointCloud2) and odom (PoseStamped) — same types as production.

Marked ``slow`` — skipped by default in ``pytest``.
"""

from __future__ import annotations

import math
from pathlib import Path
import time

import pytest

from dimos.core.blueprints import autoconnect
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.slam_map.module import SlamMap
from dimos.robot.unitree.go2.connection import GO2Connection

_REPLAY_DIR = Path(__file__).resolve().parents[3] / "data" / "unitree_go2_bigoffice"


def _replay_data_available() -> bool:
    """Check if replay dataset with lidar + odom exists."""
    return (_REPLAY_DIR / "lidar").is_dir() and (_REPLAY_DIR / "odom").is_dir()


@pytest.mark.slow
@pytest.mark.skipif(
    not _replay_data_available(),
    reason="Replay dataset not available",
)
class TestSlamMapE2E:
    """E2E tests for SlamMap with replay data."""

    @staticmethod
    def _build_blueprint() -> autoconnect:
        return (
            autoconnect(
                GO2Connection.blueprint(),
                SlamMap.blueprint(
                    resolution=0.3,
                    max_range=10.0,
                    publish_rate=2.0,
                ),
            )
            .global_config(
                replay=True,
                replay_dir=str(_REPLAY_DIR),
                n_workers=3,
                robot_model="unitree_go2",
            )
            .remappings(
                [
                    (GO2Connection, "odom", "go2_odom"),
                    (SlamMap, "raw_scan", "lidar"),
                    (SlamMap, "raw_odom", "go2_odom"),
                ]
            )
        )

    def test_produces_corrected_map_and_odom(self) -> None:
        """SlamMap produces corrected_global_map and corrected_odom from replay data."""
        received_maps: list[PointCloud2] = []
        received_odoms: list[PoseStamped] = []
        map_stream = None
        odom_stream = None

        coordinator: ModuleCoordinator = self._build_blueprint().build()

        try:
            slam_map_instance = coordinator.get_instance(SlamMap)
            assert slam_map_instance is not None

            map_stream = slam_map_instance.corrected_global_map  # type: ignore[union-attr]
            odom_stream = slam_map_instance.corrected_odom  # type: ignore[union-attr]

            map_stream.subscribe(lambda msg: received_maps.append(msg))
            odom_stream.subscribe(lambda msg: received_odoms.append(msg))

            deadline = time.time() + 30.0
            while time.time() < deadline:
                time.sleep(0.5)
                has_map = len(received_maps) > 0
                has_odom = len(received_odoms) >= 5
                if has_map and has_odom:
                    break

            # -- corrected_odom assertions --
            assert len(received_odoms) >= 5, (
                f"Expected >=5 corrected_odom messages, got {len(received_odoms)}"
            )
            for odom in received_odoms:
                assert not math.isnan(odom.x), "NaN in odom x"
                assert not math.isnan(odom.y), "NaN in odom y"
                assert not math.isnan(odom.z), "NaN in odom z"
                assert abs(odom.x) < 1000, "Odom x out of bounds"
                assert abs(odom.y) < 1000, "Odom y out of bounds"

            # -- corrected_global_map assertions --
            assert len(received_maps) > 0, "No corrected_global_map messages received"
        finally:
            # Stop the LCM transports backing the subscriptions to kill
            # their _lcm_loop threads before the conftest thread-leak check.
            for stream in (map_stream, odom_stream):
                if stream is None:
                    continue
                transport = getattr(stream, "_transport", None)
                if transport is not None and hasattr(transport, "stop"):
                    transport.stop()

            coordinator.stop()
