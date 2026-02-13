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

"""Python NativeModule wrapper for the C++ Livox Mid-360 driver.

Declares the same ports as LivoxLidarModule (pointcloud, imu) but delegates
all real work to the ``mid360_native`` C++ binary, which talks directly to
the Livox SDK2 C API and publishes on LCM.

Usage::

    from dimos.hardware.sensors.lidar.livox.module import Mid360Module
    from dimos.core.blueprints import autoconnect

    autoconnect(
        Mid360Module.blueprint(host_ip="192.168.1.5"),
        SomeConsumer.blueprint(),
    ).build().loop()
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

from dimos.core import Out  # noqa: TC001
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.msgs.sensor_msgs.Imu import Imu  # noqa: TC001
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2  # noqa: TC001
from dimos.spec import perception

_DEFAULT_EXECUTABLE = str(Path(__file__).parent / "cpp" / "result" / "bin" / "mid360_native")


@dataclass(kw_only=True)
class Mid360Config(NativeModuleConfig):
    """Config for the C++ Mid-360 native module."""

    executable: str = _DEFAULT_EXECUTABLE
    host_ip: str = "192.168.1.5"
    lidar_ip: str = "192.168.1.155"
    frequency: float = 10.0
    enable_imu: bool = True
    frame_id: str = "lidar_link"
    imu_frame_id: str = "imu_link"

    # SDK port configuration (match defaults in LivoxMid360Config)
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


class Mid360Module(NativeModule, perception.Lidar, perception.IMU):
    """Livox Mid-360 LiDAR module backed by a native C++ binary.

    Ports:
        pointcloud (Out[PointCloud2]): Point cloud frames at configured frequency.
        imu (Out[Imu]): IMU data at ~200 Hz (if enabled).
    """

    config: Mid360Config
    default_config = Mid360Config

    pointcloud: Out[PointCloud2]
    imu: Out[Imu]


mid360_module = Mid360Module.blueprint

__all__ = [
    "Mid360Config",
    "Mid360Module",
    "mid360_module",
]
