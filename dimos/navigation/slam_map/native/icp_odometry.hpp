// Copyright 2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * ICP Odometry — simplified KISS-ICP scan-to-local-map matching.
 *
 * Maintains a voxel hash map of recent points and aligns each incoming
 * scan against it using point-to-point ICP with an adaptive correspondence
 * threshold.  Includes Hessian-based degeneracy detection.
 *
 * Reference: Vizzo et al., "KISS-ICP: In Defense of Point-to-Point ICP",
 *            IEEE Robotics and Automation Letters, 2023.
 */

#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace slam_map {

struct IcpConfig {
    double max_corr_dist          = 1.0;    // metres — initial / max
    int    max_iterations         = 20;
    double convergence_threshold  = 1e-4;   // mean-squared change
    double adaptive_threshold_init = 1.0;
    double min_eigenvalue_ratio   = 0.01;   // degeneracy threshold
    double voxel_downsample       = 0.3;    // pre-filter resolution (m)
    int    local_map_max_scans    = 20;     // sliding window size
};

class IcpOdometry {
public:
    explicit IcpOdometry(const IcpConfig& cfg);

    /// Align a new scan against the local map.
    ///
    /// @param scan_points   Sensor-frame 3-D points (Nx3).
    /// @param odom_hint     Raw odometry delta since last scan (initial guess).
    /// @param degenerate_axes  Output bitmask — bit i set if axis i is
    ///                         degenerate (0=tx,1=ty,2=tz,3=rx,4=ry,5=rz).
    /// @return  Refined relative transform (previous → current, world frame).
    Eigen::Matrix4d align(const std::vector<Eigen::Vector3d>& scan_points,
                          const Eigen::Matrix4d& odom_hint,
                          uint8_t& degenerate_axes);

    /// Current cumulative global pose (product of all relative transforms).
    const Eigen::Matrix4d& globalPose() const { return global_pose_; }

    /// Has at least one scan been processed?
    bool hasData() const { return scan_count_ > 0; }

private:
    // --- Voxel hash map for the local map ---
    struct VoxelKey {
        int64_t x, y, z;
        bool operator==(const VoxelKey& o) const {
            return x == o.x && y == o.y && z == o.z;
        }
    };

    struct VoxelKeyHash {
        size_t operator()(const VoxelKey& k) const {
            // Combine three ints into one hash
            size_t h = std::hash<int64_t>{}(k.x);
            h ^= std::hash<int64_t>{}(k.y) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
            h ^= std::hash<int64_t>{}(k.z) + 0x9e3779b97f4a7c15ULL + (h << 6) + (h >> 2);
            return h;
        }
    };

    using VoxelMap = std::unordered_map<VoxelKey, Eigen::Vector3d, VoxelKeyHash>;

    // Downsample points into a voxel grid, keeping one point per voxel.
    std::vector<Eigen::Vector3d> downsample(
        const std::vector<Eigen::Vector3d>& pts, double voxel_size) const;

    // Find nearest-neighbour correspondences in the local map.
    // Returns pairs of (source_idx, target_point).
    std::vector<std::pair<int, Eigen::Vector3d>> findCorrespondences(
        const std::vector<Eigen::Vector3d>& source_transformed,
        double max_dist) const;

    // Core ICP loop: returns transform, computes Hessian eigenvalues.
    Eigen::Matrix4d runIcp(
        const std::vector<Eigen::Vector3d>& source,
        const Eigen::Matrix4d& initial_guess,
        Eigen::Matrix<double, 6, 1>& eigenvalues);

    // Solve for the rigid transform minimising point-to-point distance.
    // Returns (rotation, translation).
    static std::pair<Eigen::Matrix3d, Eigen::Vector3d> solvePointToPoint(
        const std::vector<Eigen::Vector3d>& src,
        const std::vector<Eigen::Vector3d>& tgt);

    // Add a downsampled scan (in world frame) to the local map.
    void addToLocalMap(const std::vector<Eigen::Vector3d>& world_points);

    // Trim old scans from the local map to maintain sliding window.
    void trimLocalMap();

    IcpConfig   cfg_;
    VoxelMap    local_map_;
    double      adaptive_threshold_;
    int         scan_count_ = 0;

    Eigen::Matrix4d global_pose_ = Eigen::Matrix4d::Identity();

    // For sliding window: track which voxels were added per scan
    struct ScanRecord {
        std::vector<VoxelKey> voxel_keys;
    };
    std::vector<ScanRecord> scan_records_;
};

}  // namespace slam_map
