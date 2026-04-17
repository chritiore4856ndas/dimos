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

import csv
from dataclasses import asdict
import os
import time

from pydantic import Field
from reactivex import operators as ops

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.pointclouds.occupancy import (
    OCCUPANCY_ALGOS,
    HeightCostConfig,
    OccupancyConfig,
)
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

_TIMING_CSV = os.path.expanduser("~/costmapper_timing_python.csv")


def _write_timing_row(compute_ms: float, total_ms: float, n_points: int) -> None:
    write_header = not os.path.exists(_TIMING_CSV)
    with open(_TIMING_CSV, "a", newline="") as f:
        writer = csv.writer(f)
        if write_header:
            writer.writerow(["timestamp_s", "compute_ms", "total_ms", "n_points"])
        writer.writerow([time.time(), f"{compute_ms:.3f}", f"{total_ms:.3f}", n_points])


class Config(ModuleConfig):
    algo: str = "height_cost"
    config: OccupancyConfig = Field(default_factory=HeightCostConfig)


class CostMapper(Module):
    config: Config
    global_map: In[PointCloud2]
    global_costmap: Out[OccupancyGrid]

    @rpc
    def start(self) -> None:
        super().start()

        def _publish_costmap(
            grid: OccupancyGrid, compute_ms: float, total_start: float, n_points: int
        ) -> None:
            self.global_costmap.publish(grid)
            total_ms = (time.perf_counter() - total_start) * 1000
            _write_timing_row(compute_ms, total_ms, n_points)

        def _calculate_and_time(
            msg: PointCloud2,
        ) -> tuple[OccupancyGrid, float, float, int]:
            total_start = time.perf_counter()
            n_points = len(msg)
            compute_start = time.perf_counter()
            grid = self._calculate_costmap(msg)
            compute_ms = (time.perf_counter() - compute_start) * 1000
            return grid, compute_ms, total_start, n_points

        self.register_disposable(
            self.global_map.observable()  # type: ignore[no-untyped-call]
            .pipe(ops.map(_calculate_and_time))
            .subscribe(lambda result: _publish_costmap(result[0], result[1], result[2], result[3]))
        )

    @rpc
    def stop(self) -> None:
        super().stop()

    # @timed()  # TODO: fix thread leak in timed decorator
    def _calculate_costmap(self, msg: PointCloud2) -> OccupancyGrid:
        fn = OCCUPANCY_ALGOS[self.config.algo]
        return fn(msg, **asdict(self.config.config))
