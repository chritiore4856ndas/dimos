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

"""Go2 SmartNav blueprint: PGO + CostMapper + ReplanningAStarPlanner.

Uses PGO for loop-closure-corrected odometry and global map from the Go2's
world-frame lidar + drifted odom. OdomAdapter bridges PoseStamped <-> Odometry
between GO2Connection and PGO.

Data flow:
    GO2Connection.lidar (remapped → registered_scan) → PGO
    GO2Connection.odom (remapped → raw_odom) → OdomAdapter → PGO.odometry
    PGO.corrected_odometry → OdomAdapter → odom → ReplanningAStarPlanner
    PGO.global_map → CostMapper → ReplanningAStarPlanner
    ReplanningAStarPlanner.cmd_vel → GO2Connection
"""

from dimos.core.blueprints import autoconnect
from dimos.mapping.costmapper import cost_mapper
from dimos.navigation.frontier_exploration.wavefront_frontier_goal_selector import (
    wavefront_frontier_explorer,
)
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.navigation.smartnav.modules.odom_adapter.odom_adapter import odom_adapter
from dimos.navigation.smartnav.modules.pgo.pgo import PGO
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic
from dimos.robot.unitree.go2.connection import GO2Connection

unitree_go2_smartnav = autoconnect(
    unitree_go2_basic,
    PGO.blueprint(),
    odom_adapter(),
    cost_mapper(),
    replanning_a_star_planner(),
    wavefront_frontier_explorer(),
).global_config(n_workers=8, robot_model="unitree_go2").remappings([
    (GO2Connection, "lidar", "registered_scan"),
    (GO2Connection, "odom", "raw_odom"),
])

__all__ = ["unitree_go2_smartnav"]
