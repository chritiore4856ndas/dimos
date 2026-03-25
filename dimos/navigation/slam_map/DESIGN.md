# SlamMap — Design Notes

## Problem

Previous navigation stacks handled *either* dynamic obstacle removal *or* loop
closure / SLAM, but not both simultaneously.  Five attempts failed because:

- **OctoMap-only** (DynamicMap v1): dynamic removal works, but no loop closure
  → drift accumulates → map doubles back on itself.
- **SLAM-only** (FAST-LIO2, Cartographer, etc.): loop closure corrects drift,
  but dynamic objects are baked permanently into a static map.
- **Naïve combination**: two separate processes with poor synchronisation →
  corrected pose arrives after the scan is already inserted at the wrong pose.

**Root cause**: SLAM and dynamic mapping are fundamentally different concerns
that conflict when naïvely combined.  SLAM assumes a static world (dynamic
objects corrupt the map).  Dynamic mapping assumes good poses (drift corrupts
the map).  The solution is a carefully ordered, *single-process* pipeline where
SLAM corrects poses first, then OctoMap uses those corrected poses.

---

## Architecture: Single NativeModule, Four Internal Components

```
raw_scan ──┐
           ├──► [ICP Odometry] ──► [Pose Graph (GTSAM)] ──► corrected_odom
raw_odom ──┘         │                    │
                     │   [Scan Context]───┘  (loop closure edges)
                     │                    │
                     └──► [OctoMap Dynamic Mapper] ──► corrected_global_map
                          (uses corrected poses)
```

All four components live in **one C++ subprocess** (`slam_map_node`).  This
guarantees atomic consistency between corrected pose and OctoMap insertion, and
avoids inter-process race conditions.

### Component summary

| Component | Class | Responsibility |
|-----------|-------|----------------|
| ICP Odometry | `IcpOdometry` | Scan-to-local-map matching, degeneracy detection |
| Scan Context | `ScanContext` | Loop closure candidate detection |
| Pose Graph | `PoseGraph` | GTSAM iSAM2 incremental optimisation |
| Dynamic Mapper | `DynamicMapper` | OctoMap with scan buffer for re-integration |

---

## Algorithm Pipeline (per incoming scan)

Each step runs synchronously in the LCM scan callback.  At 10 Hz lidar the
entire pipeline takes ~15 ms (ICP ~8 ms, Scan Context ~3 ms, GTSAM ~2 ms,
OctoMap insert ~2 ms).

1. **Parse** PointCloud2 → `vector<PointXYZI>` via `smartnav::parse_pointcloud2()`.

2. **ICP alignment**: `IcpOdometry::align(scan)` matches the current scan
   against a voxel-hash-map local map (last ~20 scans).  Returns relative
   transform `T_rel` from previous scan frame to current.

3. **Degeneracy detection**: Analyse eigenvalues of the ICP Hessian
   (`J^T J`).  If the smallest eigenvalue ratio is below
   `icp_degeneracy_threshold`, the corresponding axis is *degenerate*
   (e.g. translation along a corridor).  For degenerate axes, substitute the
   raw odometry delta.

4. **Pose graph — odometry factor**: `PoseGraph::addOdometryEdge(T_rel)` adds
   a `BetweenFactor<Pose3>` with the configured odometry noise model.  Returns
   the new node ID.

5. **Scan Context — loop detection**: `ScanContext::addScan(scan)` computes
   and stores the egocentric ring-sector descriptor.
   `ScanContext::detectLoop()` queries the KD-tree of ring-keys, skipping the
   last `sc_exclude_recent` frames, and returns the best candidate (if its
   cosine distance is below `sc_dist_threshold`).

6. **Geometric verification**: If a candidate is found, run ICP between the
   current scan and the candidate scan (from the scan buffer).  Only accept the
   loop closure if the mean-squared residual is below `loop_icp_max_dist`.

7. **Pose graph — loop closure factor**: If verified,
   `PoseGraph::addLoopClosure(from, to, T_loop)` adds a
   `BetweenFactor<Pose3>` with the loop closure noise model.

8. **iSAM2 optimise**: `PoseGraph::optimize()`.  If a loop closure was added,
   all historical poses may change.  In that case, call
   `DynamicMapper::reintegrate(corrected_poses)` to clear the OctoMap and
   re-insert the last N scans from the circular buffer at their corrected poses.

9. **OctoMap insert**: `DynamicMapper::insertScan(scan, corrected_pose)`.
   OctoMap's `insertPointCloud()` performs 3-D DDA ray-casting — intermediate
   voxels decremented (free-space), endpoint incremented (occupied).  Dynamic
   obstacles fade as rays pass through their former positions.

10. **Publish corrected_odom**: The latest optimised pose, with
    post-loop-closure smoothing applied (see below).

11. **Timer thread** (separate): periodically calls
    `DynamicMapper::getOccupiedVoxels()` and publishes as
    `corrected_global_map` (PointCloud2).

---

## ICP Odometry (Simplified KISS-ICP)

Based on Vizzo et al., "KISS-ICP: In Defense of Point-to-Point ICP", RAL 2023.

### Local map

A **spatial hash map** (voxel grid) stores downsampled points from the last
~20 scans.  Each voxel stores up to one representative point.  No PCL
dependency — uses `std::unordered_map<VoxelKey, Eigen::Vector3d>`.

### Adaptive threshold

After each alignment, the model error (mean distance between corresponding
points) updates an exponential moving average.  The correspondence distance
threshold is set to `3 × EMA` (clamped to a configurable range).  This makes
ICP robust to varying environments without manual tuning.

### Degeneracy detection

After convergence, compute the 6×6 Hessian `H = J^T J` from the final
correspondences.  Eigendecompose `H`.  If the ratio
`λ_min / λ_max < icp_degeneracy_threshold`, the corresponding axis is
degenerate:

- Translation along a long corridor (x or y)
- Rotation in a symmetric room (yaw)

For degenerate axes, substitute the raw odometry delta.  This prevents ICP
from drifting along unconstrained directions.

**Reference**: Zhang et al., "Degeneracy-Aware Factors with Applications to
Underwater SLAM", IROS 2015.

---

## Loop Closure Detection (Scan Context)

Based on Kim & Kim, "Scan Context: Egocentric Spatial Descriptor for Place
Recognition within 3D Point Cloud Map", IROS 2018.

### Descriptor

A 2-D matrix (`num_rings × num_sectors`), where each cell stores the max
height of points falling into that (ring, sector) bin.  Rings are concentric
annuli centred on the sensor; sectors are angular slices.

### Retrieval

1. Compute the **ring key** (column-wise mean of the descriptor) — a compact
   1-D signature.
2. Query a **nanoflann KD-tree** of all stored ring keys to find the
   top-K nearest neighbours (fast, <1 ms).
3. For each candidate, compute the **aligned cosine distance**: try all
   column shifts (sector rotations) and take the minimum.  This handles
   arbitrary yaw differences between visits.
4. Accept the candidate with the smallest distance if it's below
   `sc_dist_threshold` and the candidate index is at least
   `sc_exclude_recent` frames in the past.

### Why Scan Context

- Geometry-based — no camera or visual features needed.
- Viewpoint-invariant via sector rotation alignment.
- Fast: <30 ms including retrieval and distance computation.
- Works well for ground robots (2.5-D descriptor is a good match for
  approximately-planar motion).

---

## Pose Graph Optimisation (GTSAM iSAM2)

Uses the GTSAM library's **iSAM2** incremental solver (Kaess et al., "iSAM2:
Incremental Smoothing and Mapping Using the Bayes Tree", IJRR 2012).

### Factor types

- **PriorFactor<Pose3>**: anchors the first node at the origin.
- **BetweenFactor<Pose3>**: odometry edges (every scan) and loop closure
  edges (when verified).

### Noise models

Diagonal noise models (`noiseModel::Diagonal::Sigmas`):

| Factor | Translation σ | Rotation σ |
|--------|---------------|------------|
| Odometry | `odom_noise_trans` (0.1 m) | `odom_noise_rot` (0.05 rad) |
| Loop closure | `loop_noise_trans` (0.2 m) | `loop_noise_rot` (0.1 rad) |

Loop closure noise is intentionally looser — we trust geometric verification
but don't let a single loop closure override hundreds of odometry measurements.

### Incremental update

On each scan, new factors and values are added via `isam2.update()`.  GTSAM
incrementally relinearises only affected variables, keeping per-scan cost to
~2 ms.  After a loop closure, the affected subgraph may be larger — typically
~10 ms.

---

## Dynamic Obstacle Mapping (OctoMap)

Identical algorithm to the original `DynamicMap` module (see
`../dynamic_map/DESIGN.md`), with one critical difference:
**scans are inserted at GTSAM-corrected poses, not raw odometry.**

### Scan buffer for re-integration

A fixed-size circular buffer (`std::deque<BufferedScan>`) stores the last
`scan_buffer_size` scans (sensor-frame points + node ID).  After a loop
closure triggers pose graph optimisation:

1. Clear the entire OctoMap.
2. Re-insert each buffered scan at its corrected pose from GTSAM.
3. Resume normal operation.

This is O(N × scan_size) where N ≤ `scan_buffer_size` (default 100).  At
15 K points/scan and 100 scans, this takes ~200 ms — acceptable since loop
closures are infrequent.

### Probabilistic model

Same as DynamicMap:

| Event | ΔL |
|-------|----|
| Ray passes *through* voxel | `log_odds_miss` ≈ −0.40 |
| Ray *terminates* in voxel | `log_odds_hit` ≈ +0.85 |

Clamped to `[0.1192, 0.9707]` probability (≈ `[-2.0, +3.5]` log-odds).

---

## Post-Loop-Closure Smoothing

A sudden pose correction after loop closure can confuse the downstream
planner (the robot appears to "teleport").  To prevent this:

1. Compute the correction delta `ΔT = T_new_corrected × T_old_corrected⁻¹`.
2. Divide into `smooth_frames` increments (default 10).
3. Over the next 10 scans, linearly interpolate from the old pose to the
   corrected pose (SLERP for rotation, LERP for translation).

The OctoMap uses the *fully corrected* pose immediately (map accuracy is
paramount), but the *published* `corrected_odom` transitions smoothly.

---

## Edge Cases and Failure Modes

| Edge case | Detection | Mitigation |
|-----------|-----------|------------|
| **ICP degeneracy** (corridor, open field) | Hessian eigenvalue ratio < threshold | Substitute raw odom delta for degenerate axes |
| **Loop closure false positive** | ICP verification residual > `loop_icp_max_dist` | Reject the candidate |
| **Large pose jump** after loop closure | Correction magnitude measured | Smooth over `smooth_frames` frames |
| **OctoMap inconsistency** after correction | Loop closure triggers re-integration | Clear tree, re-insert from scan buffer |
| **Scan buffer overflow** | Circular buffer, fixed size | Oldest scans evicted; very old map regions retain their last-known state |
| **ICP failure** (too few points, bad initial guess) | ICP convergence check (max iterations reached with high residual) | Fall back entirely to raw odom for this scan |

---

## Threading Model

```
Thread 1 (main):  LCM event loop → scan/odom callbacks
                  ICP + Scan Context + GTSAM + OctoMap insert
                  (~15 ms per scan at 10 Hz — well within budget)

Thread 2 (timer): sleep(1/publish_rate) → DynamicMapper::getOccupiedVoxels()
                  → publish corrected_global_map PointCloud2
```

A single `std::mutex` protects the shared state (OctoMap tree, sensor origin,
poses).  The scan callback acquires the lock for the full pipeline; the publish
thread acquires it briefly to iterate occupied voxels.

---

## Stream Interface

| Stream | Direction | Type | Notes |
|--------|-----------|------|-------|
| `raw_scan` | In | `PointCloud2` | Sensor-frame lidar (remapped from `lidar`) |
| `raw_odom` | In | `PoseStamped` | Raw robot odometry (remapped from `go2_odom`) |
| `corrected_global_map` | Out | `PointCloud2` | Drift-corrected occupied voxels |
| `corrected_odom` | Out | `PoseStamped` | Loop-closure-corrected pose |

Output port names are intentionally different from `DynamicMap` (`global_map`,
`odom`) to prevent accidental substitution without updating remappings.

---

## Build & Dependencies

```bash
# System packages
sudo apt install liboctomap-dev libgtsam-dev libeigen3-dev

# Build
cd dimos/navigation/slam_map/native
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

The Python module's `build_command` runs this automatically on first start if
the binary is missing.

### CMake dependencies

| Dependency | Source | Purpose |
|------------|--------|---------|
| dimos-lcm | FetchContent (GitHub) | C++ LCM message definitions |
| nanoflann | FetchContent (GitHub) | Header-only KD-tree for Scan Context |
| LCM | pkg-config | Transport layer |
| OctoMap | find_package | 3-D probabilistic mapping |
| Eigen3 | find_package | Linear algebra (ICP, pose math) |
| GTSAM | find_package | Factor graph optimisation |

**No PCL dependency.**  ICP uses hand-rolled voxel hash map + Eigen SVD.

---

## References

1. I. Vizzo, T. Guadagnino, B. Mersch, L. Wiesmann, J. Behley, and
   C. Stachniss, "KISS-ICP: In Defense of Point-to-Point ICP — Simple,
   Accurate, and Robust Registration If Done in the Right Way", *IEEE
   Robotics and Automation Letters* 8(2), pp. 1029–1036, 2023.
   https://github.com/PRBonn/kiss-icp

2. G. Kim and A. Kim, "Scan Context: Egocentric Spatial Descriptor for Place
   Recognition within 3D Point Cloud Map", *IEEE/RSJ Int. Conf. on
   Intelligent Robots and Systems (IROS)*, pp. 4802–4809, 2018.
   https://github.com/irapkaist/scancontext

3. M. Kaess, H. Johannsson, R. Roberts, V. Ila, J. J. Leonard, and
   F. Dellaert, "iSAM2: Incremental Smoothing and Mapping Using the Bayes
   Tree", *Int. Journal of Robotics Research* 31(2), pp. 216–235, 2012.
   https://gtsam.org

4. A. Hornung, K. M. Wurm, M. Bennewitz, C. Stachniss, and W. Burgard,
   "OctoMap: An efficient probabilistic 3D mapping framework based on
   octrees", *Autonomous Robots* 34(3), pp. 189–206, 2013.
   https://octomap.github.io/

5. J. Amanatides and A. Woo, "A fast voxel traversal algorithm for ray
   tracing", *Eurographics* 87, pp. 3–10, 1987.

6. J. Zhang, M. Kaess, and S. Singh, "On Degeneracy of Optimization-based
   State Estimation Problems", *IEEE Int. Conf. on Robotics and Automation
   (ICRA)*, pp. 809–816, 2016.

7. G. Kim, S. Choi, and A. Kim, "FAST_LIO_SLAM: FAST-LIO2 + Scan Context
   loop closure", 2022. https://github.com/gisbi-kim/FAST_LIO_SLAM
