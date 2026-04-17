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

from pathlib import Path

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

_RUST_DIR = Path(__file__).parent / "costmapper_rust"


class RustCostMapperConfig(NativeModuleConfig):
    executable: str = str(_RUST_DIR / "target" / "release" / "costmapper")
    build_command: str = "cargo build --release"
    cwd: str = str(_RUST_DIR)
    stdin_config: bool = True
    resolution: float = 0.05
    can_pass_under: float = 0.6
    can_climb: float = 0.15
    ignore_noise: float = 0.05
    smoothing: float = 1.0


class RustCostMapper(NativeModule):
    config: RustCostMapperConfig
    global_map: In[PointCloud2]
    global_costmap: Out[OccupancyGrid]
