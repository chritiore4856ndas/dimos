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

"""SlamMap — NativeModule for SLAM + loop closure + dynamic obstacle mapping.

Combines four algorithms in a single C++ subprocess:

1. **ICP odometry** (simplified KISS-ICP) — scan-to-local-map matching for
   refined relative pose estimates.
2. **Scan Context** (Kim & Kim, IROS 2018) — geometry-based loop closure
   detection with geometric ICP verification.
3. **GTSAM iSAM2** (Kaess et al., IJRR 2012) — incremental pose graph
   optimisation that corrects accumulated drift on loop closure.
4. **OctoMap** (Hornung et al., 2013) — probabilistic 3-D occupancy grid
   with ray-casting for dynamic obstacle removal.

The key insight is that SLAM corrects poses *first*, then OctoMap uses
corrected poses for mapping.  Dynamic obstacles fade via ray-casting in a
drift-free frame.

Build the C++ binary before running::

    sudo apt install liboctomap-dev libgtsam-dev libeigen3-dev
    cd dimos/navigation/slam_map/native
    cmake -B build -DCMAKE_BUILD_TYPE=Release
    cmake --build build -j$(nproc)
"""

from __future__ import annotations

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class SlamMapConfig(NativeModuleConfig):
    """Configuration for the SlamMap native module.

    All fields below (except those inherited from ``NativeModuleConfig``) are
    forwarded to the C++ subprocess as ``--field_name value`` CLI arguments
    via :meth:`NativeModuleConfig.to_cli_args`.
    """

    cwd: str | None = "native"
    executable: str = "result/bin/slam_map_node"
    build_command: str | None = "nix build .#slam_map_node"

    # -- OctoMap parameters --------------------------------------------------
    #: Voxel resolution in metres.
    resolution: float = 0.15
    #: Rays beyond this distance (metres) are truncated.
    max_range: float = 15.0
    #: Log-odds threshold above which a voxel is "occupied".
    occ_threshold: float = 0.5
    #: Map publication frequency in Hz.
    publish_rate: float = 0.5

    # -- ICP odometry --------------------------------------------------------
    #: Max correspondence distance for ICP matching (metres).
    icp_max_corr_dist: float = 1.0
    #: Max ICP iterations per scan.
    icp_max_iterations: int = 20
    #: Voxel downsample resolution for ICP input (metres).
    icp_voxel_downsample: float = 0.3
    #: Min eigenvalue ratio for degeneracy detection (0–1).
    icp_degeneracy_threshold: float = 0.01

    # -- Loop closure (Scan Context) -----------------------------------------
    #: Cosine distance threshold for Scan Context candidates.
    sc_dist_threshold: float = 0.2
    #: Skip this many recent scans when searching for loop closures.
    sc_exclude_recent: int = 30
    #: Max mean-squared residual for loop closure geometric verification.
    loop_icp_max_dist: float = 0.5

    # -- Pose graph noise model ----------------------------------------------
    #: Odometry factor translation noise std-dev (metres).
    odom_noise_trans: float = 0.1
    #: Odometry factor rotation noise std-dev (radians).
    odom_noise_rot: float = 0.05
    #: Loop closure factor translation noise std-dev (metres).
    loop_noise_trans: float = 0.2
    #: Loop closure factor rotation noise std-dev (radians).
    loop_noise_rot: float = 0.1

    # -- Post-loop-closure smoothing -----------------------------------------
    #: Number of frames over which to smooth a loop closure correction.
    smooth_frames: int = 10
    #: Number of recent scans to buffer for post-loop-closure re-integration.
    scan_buffer_size: int = 100


class SlamMap(NativeModule):
    """SLAM + loop closure + dynamic obstacle mapping via a C++ subprocess.

    Pipeline per scan:
    1. ICP scan-to-local-map matching → refined relative pose
    2. Degeneracy check → fall back to raw odom for degenerate axes
    3. GTSAM pose graph → add odometry factor
    4. Scan Context → loop closure candidate detection
    5. If candidate: geometric verification via ICP → loop closure factor
    6. iSAM2 optimise → corrected poses; re-integrate OctoMap if needed
    7. OctoMap insert at corrected pose → dynamic obstacle removal

    Ports
    -----
    raw_scan : In[PointCloud2]
        Sensor-frame lidar point cloud (from GO2Connection.lidar via remapping).
    raw_odom : In[PoseStamped]
        Raw robot pose (from GO2Connection.odom via remapping).
    corrected_global_map : Out[PointCloud2]
        Drift-corrected occupied voxels above ``occ_threshold``.
    corrected_odom : Out[PoseStamped]
        Loop-closure-corrected pose estimate.
    """

    default_config: type[SlamMapConfig] = SlamMapConfig  # type: ignore[assignment]

    raw_scan: In[PointCloud2]
    raw_odom: In[PoseStamped]

    corrected_global_map: Out[PointCloud2]
    corrected_odom: Out[PoseStamped]


# Blueprint factory — used by autoconnect() and dimos run
slam_map = SlamMap.blueprint
