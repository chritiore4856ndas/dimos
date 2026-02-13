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

"""Python NativeModule wrapper for the FAST-LIO2 + Livox Mid-360 binary.

Binds Livox SDK2 directly into FAST-LIO-NON-ROS for real-time LiDAR SLAM.
Outputs registered (world-frame) point clouds and odometry with covariance.

Usage::

    from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
    from dimos.core.blueprints import autoconnect

    autoconnect(
        FastLio2.blueprint(host_ip="192.168.1.5"),
        SomeConsumer.blueprint(),
    ).build().loop()
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from dimos.core import Out  # noqa: TC001
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.msgs.nav_msgs.Odometry import Odometry  # noqa: TC001
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2  # noqa: TC001

_DEFAULT_EXECUTABLE = str(Path(__file__).parent / "cpp" / "result" / "bin" / "fastlio2_native")
_CONFIG_DIR = Path(__file__).parent / "config"


@dataclass(kw_only=True)
class FastLio2Config(NativeModuleConfig):
    """Config for the FAST-LIO2 + Livox Mid-360 native module."""

    executable: str = _DEFAULT_EXECUTABLE

    # Livox SDK hardware config
    host_ip: str = "192.168.1.5"
    lidar_ip: str = "192.168.1.155"
    frequency: float = 10.0

    # Frame IDs for output messages
    frame_id: str = "map"
    child_frame_id: str = "body"

    # FAST-LIO internal processing rates
    msr_freq: float = 50.0
    main_freq: float = 5000.0

    # Output publish rates (Hz)
    pointcloud_freq: float = 10.0
    odom_freq: float = 30.0

    # Point cloud filtering
    voxel_size: float = 0.1
    sor_mean_k: int = 50
    sor_stddev: float = 1.0

    # Global voxel map (disabled when map_freq <= 0)
    map_freq: float = 0.0
    map_voxel_size: float = 0.1
    map_max_range: float = 100.0

    # FAST-LIO YAML config (relative to config/ dir, or absolute path)
    # C++ binary reads YAML directly via yaml-cpp
    config: str = "mid360.yaml"

    # SDK port configuration (for multi-sensor setups)
    cmd_data_port: int = 56100
    push_msg_port: int = 56200
    point_data_port: int = 56300
    imu_data_port: int = 56400
    log_data_port: int = 56500
    host_cmd_data_port: int = 56101
    host_push_msg_port: int = 56201
    host_point_data_port: int = 56301
    host_imu_data_port: int = 56401
    host_log_data_port: int = 56501

    # Resolved in __post_init__, passed as --config_path to the binary
    config_path: str | None = None

    # config is not a CLI arg (config_path is)
    cli_exclude: frozenset[str] = frozenset({"config"})

    def __post_init__(self) -> None:
        if self.config_path is None:
            path = Path(self.config)
            if not path.is_absolute():
                path = _CONFIG_DIR / path
            self.config_path = str(path.resolve())


class FastLio2(NativeModule):
    """FAST-LIO2 SLAM module with integrated Livox Mid-360 driver.

    Ports:
        lidar (Out[PointCloud2]): World-frame registered point cloud.
        odometry (Out[Odometry]): Pose with covariance at LiDAR scan rate.
        global_map (Out[PointCloud2]): Global voxel map (optional, enable via map_freq > 0).
    """

    default_config: type[FastLio2Config] = FastLio2Config  # type: ignore[assignment]
    lidar: Out[PointCloud2]
    odometry: Out[Odometry]
    global_map: Out[PointCloud2]


fastlio2_module = FastLio2.blueprint

__all__ = [
    "FastLio2",
    "FastLio2Config",
    "fastlio2_module",
]
