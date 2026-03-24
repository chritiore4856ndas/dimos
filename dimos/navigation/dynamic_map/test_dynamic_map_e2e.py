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

"""E2E test: DynamicMap with Unity simulator.

Verifies that the DynamicMap module produces a non-empty global_map
when fed lidar scans from the Unity sim's kinematic model.

Requires:
  - Compiled C++ binary (cmake)
  - Unity sim assets (LFS)
  - X11 display (headless mode doesn't produce lidar)

Marked ``slow`` — skipped by default in ``pytest``.
"""

from __future__ import annotations

import time

import pytest

from dimos.core.blueprints import autoconnect
from dimos.core.module_coordinator import ModuleCoordinator
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.dynamic_map.module import DynamicMap
from dimos.simulation.unity.module import UnityBridgeModule


@pytest.mark.slow
@pytest.mark.skipif(
    not UnityBridgeModule._resolve_binary.__func__(None),  # type: ignore[arg-type]
    reason="Unity binary not available (LFS asset missing)",
)
def test_dynamic_map_with_unity_sim() -> None:
    """DynamicMap produces occupied voxels from Unity sim lidar data."""
    received_maps: list[PointCloud2] = []

    blueprint = (
        autoconnect(
            UnityBridgeModule.blueprint(
                unity_binary="",
                headless=True,
                sim_rate=50.0,
            ),
            DynamicMap.blueprint(
                resolution=0.3,
                max_range=10.0,
                publish_rate=2.0,
            ),
        )
        .global_config(simulation=True, n_workers=4)
        .remappings(
            [
                (UnityBridgeModule, "pointcloud", "registered_scan"),
                (UnityBridgeModule, "odometry", "raw_odom"),
            ]
        )
    )

    coordinator: ModuleCoordinator = blueprint.build()
    try:
        # Let the sim run for a few seconds to accumulate scans
        dynamic_map_instance = coordinator.get_instance(DynamicMap)
        assert dynamic_map_instance is not None

        # Subscribe to global_map output
        dynamic_map_instance.global_map.subscribe(  # type: ignore[union-attr]
            lambda msg: received_maps.append(msg)
        )

        # Wait for at least one non-empty map
        deadline = time.time() + 15.0
        while time.time() < deadline:
            time.sleep(0.5)
            if any(len(m.data) > 0 for m in received_maps):
                break

        assert len(received_maps) > 0, "No global_map messages received"
        assert any(len(m.data) > 0 for m in received_maps), (
            "All global_map messages were empty — no occupied voxels detected"
        )
    finally:
        coordinator.stop()
