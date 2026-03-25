#!/usr/bin/env python3
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

"""unitree-go2-slam-nav blueprint.

Extends ``unitree_go2_basic`` with ``SlamMap`` — a combined SLAM + loop
closure + dynamic obstacle mapping module.  Dynamic obstacles fade from the
map via OctoMap ray-casting, while ICP + Scan Context + GTSAM correct
accumulated drift via loop closure.

Data flow::

    GO2Connection ─┬─ lidar ──▶ SlamMap ──corrected_global_map──▶ CostMapper
                   └─ go2_odom ──▶ SlamMap ──corrected_odom──▶ Planner / etc.

GO2Connection's ``odom`` is renamed to ``go2_odom`` so SlamMap can subscribe
to it without colliding with its own ``corrected_odom`` output (remapped to
``odom`` for downstream consumers).

Usage::

    dimos run unitree-go2-slam-nav --robot-ip 192.168.123.161
    dimos --replay run unitree-go2-slam-nav
"""

from dimos.core.blueprints import autoconnect
from dimos.mapping.costmapper import CostMapper
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    WavefrontFrontierExplorer,
)
from dimos.navigation.patrolling.module import PatrollingModule
from dimos.navigation.replanning_a_star.module import ReplanningAStarPlanner
from dimos.navigation.slam_map.module import SlamMap
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.connection import GO2Connection

unitree_go2_slam_nav = (
    autoconnect(
        unitree_go2_basic,
        SlamMap.blueprint(),
        CostMapper.blueprint(),
        ReplanningAStarPlanner.blueprint(),
        WavefrontFrontierExplorer.blueprint(),
        PatrollingModule.blueprint(),
    )
    .remappings(
        [
            # Rename GO2Connection's odom to go2_odom to avoid collision
            # with SlamMap's corrected_odom output (remapped to odom below).
            (GO2Connection, "odom", "go2_odom"),
            # GO2Connection.lidar → SlamMap.raw_scan
            (SlamMap, "raw_scan", "lidar"),
            # GO2Connection.go2_odom → SlamMap.raw_odom
            (SlamMap, "raw_odom", "go2_odom"),
            # SlamMap.corrected_global_map → CostMapper.global_map (auto-wired)
            (SlamMap, "corrected_global_map", "global_map"),
            # SlamMap.corrected_odom → downstream odom consumers
            (SlamMap, "corrected_odom", "odom"),
        ]
    )
    .global_config(n_workers=7, robot_model="unitree_go2")
)

__all__ = ["unitree_go2_slam_nav"]
