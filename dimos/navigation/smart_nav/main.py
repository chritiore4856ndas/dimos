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

"""SmartNav composable navigation stack.

`smart_nav(**kwargs)` returns an autoconnected Blueprint containing the core
SmartNav modules (terrain analysis, local planner, path follower, FAR planner,
PGO, click-to-goal, cmd-vel mux), with optional TARE exploration and
GlobalMapUpdater accumulator.

`smart_nav_rerun_config(user_config)` returns a Rerun config dict with the
SmartNav defaults filled in via setdefault — pass it to `RerunBridgeModule`
or `vis_module` separately.

Defaults match the onboard (real hardware) configuration. Override any
module's config via per-module kwarg dicts (e.g.
`terrain_analysis={"obstacle_height_threshold": 0.1}`).
"""

from __future__ import annotations

from typing import Any

from dimos.core.blueprints import Blueprint, autoconnect
from dimos.navigation.cmd_vel_mux import CmdVelMux
from dimos.navigation.smart_nav.blueprints._rerun_helpers import (
    global_map_override,
    goal_path_override,
    path_override,
    sensor_scan_override,
    static_floor,
    static_robot,
    terrain_map_ext_override,
    terrain_map_override,
    waypoint_override,
)
from dimos.navigation.smart_nav.modules.click_to_goal.click_to_goal import ClickToGoal
from dimos.navigation.smart_nav.modules.far_planner.far_planner import FarPlanner
from dimos.navigation.smart_nav.modules.global_map_updater.global_map_updater import (
    GlobalMapUpdater,
)
from dimos.navigation.smart_nav.modules.local_planner.local_planner import LocalPlanner
from dimos.navigation.smart_nav.modules.path_follower.path_follower import PathFollower
from dimos.navigation.smart_nav.modules.pgo.pgo import PGO
from dimos.navigation.smart_nav.modules.tare_planner.tare_planner import TarePlanner
from dimos.navigation.smart_nav.modules.terrain_analysis.terrain_analysis import TerrainAnalysis
from dimos.navigation.smart_nav.modules.terrain_map_ext.terrain_map_ext import TerrainMapExt
from dimos.protocol.pubsub.impl.lcmpubsub import LCM


def _default_rerun_blueprint() -> Any:
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Spatial3DView(origin="world", name="3D"),
    )


def smart_nav_rerun_config(
    user_config: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Return a rerun config dict with SmartNav defaults filled in via setdefault.

    The caller's entries win — this just ensures missing keys (blueprint,
    pubsubs, min_interval_sec, visual_override entries, static entries) are
    populated with the SmartNav defaults.
    """
    resolved = dict(user_config or {})
    resolved.setdefault("blueprint", _default_rerun_blueprint)
    resolved.setdefault("pubsubs", [LCM()])
    resolved.setdefault("min_interval_sec", 0.25)
    resolved.setdefault("visual_override", {})
    resolved.setdefault("static", {})
    visual_override = dict(resolved["visual_override"])
    visual_override.setdefault("world/sensor_scan", sensor_scan_override)
    visual_override.setdefault("world/terrain_map", terrain_map_override)
    visual_override.setdefault("world/terrain_map_ext", terrain_map_ext_override)
    visual_override.setdefault("world/global_map", global_map_override)
    visual_override.setdefault("world/path", path_override)
    visual_override.setdefault("world/way_point", waypoint_override)
    visual_override.setdefault("world/goal_path", goal_path_override)
    resolved["visual_override"] = visual_override
    static_entries = dict(resolved["static"])
    static_entries.setdefault("world/floor", static_floor)
    static_entries.setdefault("world/tf/robot", static_robot)
    resolved["static"] = static_entries
    return resolved


def smart_nav(
    *,
    use_tare: bool = False,
    use_global_map_updater: bool = False,
    use_terrain_map_ext: bool = True,
    vehicle_height: float | None = None,
    terrain_analysis: dict[str, Any] | None = None,
    terrain_map_ext: dict[str, Any] | None = None,
    local_planner: dict[str, Any] | None = None,
    path_follower: dict[str, Any] | None = None,
    far_planner: dict[str, Any] | None = None,
    pgo: dict[str, Any] | None = None,
    click_to_goal: dict[str, Any] | None = None,
    cmd_vel_mux: dict[str, Any] | None = None,
    tare_planner: dict[str, Any] | None = None,
    global_map_updater: dict[str, Any] | None = None,
) -> Blueprint:
    """Compose a SmartNav autoconnect Blueprint with the given options.

    Args:
        use_tare: Add the TARE frontier-based exploration planner. Auto-remaps
            ClickToGoal's `way_point` output so TARE has exclusive control of
            LocalPlanner's waypoint input.
        use_global_map_updater: Add the bounded-memory voxel accumulator
            (GlobalMapUpdater) on top of registered_scan.
        use_terrain_map_ext: Add TerrainMapExt — the persistent extended terrain
            accumulator used for visualization and wider-range planning.
        vehicle_height: Ignore terrain points above this height (m). Threaded
            into TerrainAnalysis's `vehicle_height` config. Defaults to 1.2m.
        terrain_analysis, terrain_map_ext, local_planner, path_follower,
        far_planner, pgo, click_to_goal, cmd_vel_mux, tare_planner,
        global_map_updater: Per-module config override dicts. Merged on top
        of the SmartNav defaults.

    Returns:
        An autoconnected Blueprint with the selected modules wired together.
    """
    modules: list[Blueprint] = [
        TerrainAnalysis.blueprint(
            **{
                # Input filtering
                "scan_voxel_size": 0.15,
                # Voxel grid
                "terrain_voxel_size": 1.0,
                "terrain_voxel_half_width": 10,
                # Obstacle/ground classification
                "obstacle_height_threshold": 0.2,
                "ground_height_threshold": 0.1,
                "min_relative_z": -1.5,
                "max_relative_z": 1.5,
                "use_sorting": True,
                "quantile_z": 0.25,
                # Decay and clearing
                "decay_time": 2.0,
                "no_decay_distance": 1.5,
                "clearing_distance": 8.0,
                "clear_dynamic_obstacles": True,
                "no_data_obstacle": False,
                "no_data_block_skip_count": 0,
                "min_block_point_count": 10,
                # Voxel culling
                "voxel_point_update_threshold": 30,
                "voxel_time_update_threshold": 2.0,
                # Dynamic obstacle filtering
                "min_dynamic_obstacle_distance": 0.14,
                "abs_dynamic_obstacle_relative_z_threshold": 0.2,
                "min_dynamic_obstacle_vfov": -55.0,
                "max_dynamic_obstacle_vfov": 10.0,
                "min_dynamic_obstacle_point_count": 1,
                "min_out_of_fov_point_count": 20,
                # Ground lift limits
                "consider_drop": False,
                "limit_ground_lift": False,
                "max_ground_lift": 0.15,
                "distance_ratio_z": 0.2,
                "vehicle_height": 1.2 if vehicle_height is None else vehicle_height,
                **(terrain_analysis or {}),
            }
        ),
        LocalPlanner.blueprint(
            **{
                "autonomy_mode": True,
                "use_terrain_analysis": True,
                "max_speed": 1.0,
                "autonomy_speed": 1.0,
                "obstacle_height_threshold": 0.2,
                "max_relative_z": 1.5,
                "min_relative_z": -1.5,
                **(local_planner or {}),
            }
        ),
        PathFollower.blueprint(
            **{
                "autonomy_mode": True,
                "max_speed": 1.0,
                "autonomy_speed": 1.0,
                "max_acceleration": 2.0,
                "slow_down_distance_threshold": 0.2,
                "omni_dir_goal_threshold": 0.5,
                **(path_follower or {}),
            }
        ),
        FarPlanner.blueprint(**{"sensor_range": 15.0, **(far_planner or {})}),
        PGO.blueprint(**(pgo or {})),
        ClickToGoal.blueprint(**(click_to_goal or {})),
        CmdVelMux.blueprint(**(cmd_vel_mux or {})),
    ]
    if use_terrain_map_ext:
        modules.append(
            TerrainMapExt.blueprint(
                **{
                    "voxel_size": 0.4,
                    "decay_time": 8.0,
                    "publish_rate": 2.0,
                    "max_range": 40.0,
                    **(terrain_map_ext or {}),
                }
            )
        )
    if use_tare:
        modules.append(TarePlanner.blueprint(**(tare_planner or {})))
    if use_global_map_updater:
        modules.append(GlobalMapUpdater.blueprint(**(global_map_updater or {})))

    remappings = [
        # PathFollower cmd_vel → CmdVelMux nav input (avoid collision with mux output)
        (PathFollower, "cmd_vel", "nav_cmd_vel"),
        # Global-scale planners use PGO-corrected odometry (per CMU ICRA 2022):
        # loop-closure adjustments go to high-level planners; local modules
        # care only about the local environment and work in the odom frame.
        (FarPlanner, "odometry", "corrected_odometry"),
        (ClickToGoal, "odometry", "corrected_odometry"),
        (TerrainAnalysis, "odometry", "corrected_odometry"),
    ]
    if use_tare:
        # TARE drives way_point; disconnect ClickToGoal's output to avoid conflict.
        remappings.append((ClickToGoal, "way_point", "_click_way_point_unused"))

    return autoconnect(*modules).remappings(remappings)
