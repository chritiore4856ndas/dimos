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

"""Tests for SlamMap NativeModule Python wrapper.

These tests cover the Python side only (config, stream declarations, CLI arg
generation, path resolution).  They do NOT require the C++ binary to be
compiled — the subprocess is never launched.
"""

from __future__ import annotations

from collections.abc import Generator
from pathlib import Path
import pickle
from typing import get_origin, get_type_hints

import pytest

from dimos.core.stream import In, Out
from dimos.navigation.slam_map.module import SlamMap, SlamMapConfig

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _make_module() -> SlamMap:
    """Instantiate SlamMap with default config."""
    m = SlamMap()
    return m


@pytest.fixture
def module() -> Generator[SlamMap, None, None]:
    m = _make_module()
    try:
        yield m
    finally:
        m.stop()


# ---------------------------------------------------------------------------
# Config defaults
# ---------------------------------------------------------------------------


class TestSlamMapConfig:
    def test_config_defaults(self) -> None:
        cfg = SlamMapConfig()
        assert cfg.resolution == 0.15
        assert cfg.max_range == 15.0
        assert cfg.occ_threshold == 0.5
        assert cfg.publish_rate == 0.5
        assert cfg.icp_max_corr_dist == 1.0
        assert cfg.icp_max_iterations == 20
        assert cfg.icp_voxel_downsample == 0.3
        assert cfg.icp_degeneracy_threshold == 0.01
        assert cfg.sc_dist_threshold == 0.2
        assert cfg.sc_exclude_recent == 30
        assert cfg.loop_icp_max_dist == 0.5
        assert cfg.odom_noise_trans == 0.1
        assert cfg.odom_noise_rot == 0.05
        assert cfg.loop_noise_trans == 0.2
        assert cfg.loop_noise_rot == 0.1
        assert cfg.smooth_frames == 10
        assert cfg.scan_buffer_size == 100

    def test_config_override(self) -> None:
        cfg = SlamMapConfig(resolution=0.25, max_range=20.0, sc_dist_threshold=0.3)
        assert cfg.resolution == 0.25
        assert cfg.max_range == 20.0
        assert cfg.sc_dist_threshold == 0.3

    def test_to_cli_args_includes_custom_fields(self) -> None:
        cfg = SlamMapConfig(resolution=0.25, icp_max_iterations=30)
        args = cfg.to_cli_args()
        # Should include our custom fields as --key value pairs
        assert "--resolution" in args
        idx = args.index("--resolution")
        assert args[idx + 1] == "0.25"
        assert "--icp_max_iterations" in args
        idx = args.index("--icp_max_iterations")
        assert args[idx + 1] == "30"

    def test_to_cli_args_excludes_base_fields(self) -> None:
        cfg = SlamMapConfig()
        args = cfg.to_cli_args()
        # Base NativeModuleConfig fields should NOT appear
        assert "--executable" not in args
        assert "--build_command" not in args
        assert "--cwd" not in args
        assert "--extra_args" not in args
        assert "--shutdown_timeout" not in args


# ---------------------------------------------------------------------------
# Port declarations
# ---------------------------------------------------------------------------


class TestSlamMapPorts:
    def test_has_expected_inputs(self) -> None:
        hints = get_type_hints(SlamMap)
        assert "raw_scan" in hints
        assert "raw_odom" in hints
        assert get_origin(hints["raw_scan"]) is In
        assert get_origin(hints["raw_odom"]) is In

    def test_has_expected_outputs(self) -> None:
        hints = get_type_hints(SlamMap)
        assert "corrected_global_map" in hints
        assert "corrected_odom" in hints
        assert get_origin(hints["corrected_global_map"]) is Out
        assert get_origin(hints["corrected_odom"]) is Out

    def test_input_types(self) -> None:
        hints = get_type_hints(SlamMap)
        # In[PointCloud2] and In[PoseStamped]
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        assert hints["raw_scan"].__args__[0] is PointCloud2
        assert hints["raw_odom"].__args__[0] is PoseStamped

    def test_output_types(self) -> None:
        hints = get_type_hints(SlamMap)
        from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        assert hints["corrected_global_map"].__args__[0] is PointCloud2
        assert hints["corrected_odom"].__args__[0] is PoseStamped


# ---------------------------------------------------------------------------
# Serialisation
# ---------------------------------------------------------------------------


class TestSlamMapPickle:
    def test_config_pickle_roundtrip(self) -> None:
        cfg = SlamMapConfig(resolution=0.3, sc_dist_threshold=0.15)
        restored = pickle.loads(pickle.dumps(cfg))
        assert restored.resolution == 0.3
        assert restored.sc_dist_threshold == 0.15


# ---------------------------------------------------------------------------
# Path resolution
# ---------------------------------------------------------------------------


class TestSlamMapPaths:
    def test_executable_resolved_relative_to_module(self, module: SlamMap) -> None:
        """Executable path should be resolved relative to module.py's directory."""
        exe = Path(module.config.executable)
        assert exe.is_absolute()
        # Should end with result/bin/slam_map_node
        assert exe.name == "slam_map_node"

    def test_cwd_resolved_relative_to_module(self, module: SlamMap) -> None:
        """cwd should be resolved to the native/ subdirectory."""
        assert module.config.cwd is not None
        cwd = Path(module.config.cwd)
        assert cwd.is_absolute()
        assert cwd.name == "native"


# ---------------------------------------------------------------------------
# Blueprint factory
# ---------------------------------------------------------------------------


class TestSlamMapBlueprint:
    def test_blueprint_callable(self) -> None:
        from dimos.navigation.slam_map.module import slam_map

        bp = slam_map()
        assert bp is not None
