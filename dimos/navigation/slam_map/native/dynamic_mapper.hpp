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
 * DynamicMapper — OctoMap with circular scan buffer for re-integration.
 *
 * On every scan, inserts into OctoMap at the given (SLAM-corrected) pose.
 * After a loop closure, clears the tree and re-inserts from the scan buffer
 * with updated poses.
 *
 * Reference: Hornung et al., "OctoMap: An efficient probabilistic 3D mapping
 *            framework based on octrees", Autonomous Robots, 2013.
 */

#pragma once

#include <cstddef>
#include <deque>
#include <memory>
#include <vector>

#include <Eigen/Core>
#include <octomap/OcTree.h>

#include "common/point_cloud_utils.hpp"

namespace slam_map {

struct DynamicMapperConfig {
    double resolution       = 0.15;
    double max_range        = 15.0;
    double occ_threshold    = 0.5;   // log-odds
    double clamping_min     = 0.1192;  // probability
    double clamping_max     = 0.9707;
    size_t scan_buffer_size = 100;
};

class DynamicMapper {
public:
    explicit DynamicMapper(const DynamicMapperConfig& cfg);

    // Movable (caller must ensure no concurrent access during move)
    DynamicMapper(DynamicMapper&&) noexcept = default;
    DynamicMapper& operator=(DynamicMapper&&) noexcept = default;

    /// Insert a scan (sensor-frame points) at the given global pose.
    /// Caller must hold external lock if accessed from multiple threads.
    void insertScan(const std::vector<Eigen::Vector3d>& scan_sensor_frame,
                    const Eigen::Matrix4d& global_pose,
                    size_t node_id);

    /// Re-integrate all buffered scans with corrected poses.
    /// Clears the tree, then re-inserts each buffered scan.
    void reintegrate(const std::vector<Eigen::Matrix4d>& corrected_poses);

    /// Get occupied voxels as XYZI points.
    /// Caller must hold external lock if accessed from multiple threads.
    std::vector<smartnav::PointXYZI> getOccupiedVoxels() const;

private:
    struct BufferedScan {
        size_t node_id;
        std::vector<Eigen::Vector3d> points;  // sensor-frame
    };

    void doInsert(const std::vector<Eigen::Vector3d>& sensor_points,
                  const Eigen::Matrix4d& pose);

    DynamicMapperConfig cfg_;
    std::unique_ptr<octomap::OcTree> tree_;
    std::deque<BufferedScan> scan_buffer_;
};

}  // namespace slam_map
