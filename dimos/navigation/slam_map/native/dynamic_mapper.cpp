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

#include "dynamic_mapper.hpp"

#include <cstdio>

namespace slam_map {

// ============================================================================
// Construction
// ============================================================================

DynamicMapper::DynamicMapper(const DynamicMapperConfig& cfg)
    : cfg_(cfg),
      tree_(std::make_unique<octomap::OcTree>(cfg.resolution))
{
    tree_->setClampingThresMin(cfg_.clamping_min);
    tree_->setClampingThresMax(cfg_.clamping_max);
}

// ============================================================================
// Public: insertScan
// ============================================================================

void DynamicMapper::insertScan(
    const std::vector<Eigen::Vector3d>& scan_sensor_frame,
    const Eigen::Matrix4d& global_pose,
    size_t node_id)
{
    // Buffer the scan for potential re-integration
    scan_buffer_.push_back({node_id, scan_sensor_frame});
    while (scan_buffer_.size() > cfg_.scan_buffer_size) {
        scan_buffer_.pop_front();
    }

    doInsert(scan_sensor_frame, global_pose);
}

// ============================================================================
// Public: reintegrate
// ============================================================================

void DynamicMapper::reintegrate(const std::vector<Eigen::Matrix4d>& corrected_poses)
{
    printf("[slam_map] Reintegrating %zu scans after loop closure\n",
           scan_buffer_.size());

    // Clear the tree completely
    tree_->clear();

    // Re-insert each buffered scan at its corrected pose
    for (const auto& buf : scan_buffer_) {
        if (buf.node_id < corrected_poses.size()) {
            doInsert(buf.points, corrected_poses[buf.node_id]);
        }
    }
}

// ============================================================================
// Public: getOccupiedVoxels
// ============================================================================

std::vector<smartnav::PointXYZI> DynamicMapper::getOccupiedVoxels() const
{
    std::vector<smartnav::PointXYZI> occupied;
    occupied.reserve(4096);

    for (auto it = tree_->begin_leafs(); it != tree_->end_leafs(); ++it) {
        if (it->getLogOdds() > cfg_.occ_threshold) {
            smartnav::PointXYZI p;
            p.x = static_cast<float>(it.getX());
            p.y = static_cast<float>(it.getY());
            p.z = static_cast<float>(it.getZ());
            p.intensity = it->getLogOdds();
            occupied.push_back(p);
        }
    }

    return occupied;
}

// ============================================================================
// Private: doInsert
// ============================================================================

void DynamicMapper::doInsert(
    const std::vector<Eigen::Vector3d>& sensor_points,
    const Eigen::Matrix4d& pose)
{
    // Extract sensor origin from pose
    octomap::point3d origin(
        static_cast<float>(pose(0, 3)),
        static_cast<float>(pose(1, 3)),
        static_cast<float>(pose(2, 3)));

    // Transform points to world frame and build OctoMap point cloud
    octomap::Pointcloud octo_cloud;
    octo_cloud.reserve(sensor_points.size());

    for (const auto& p : sensor_points) {
        Eigen::Vector4d pw = pose * Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0);
        octo_cloud.push_back(
            static_cast<float>(pw.x()),
            static_cast<float>(pw.y()),
            static_cast<float>(pw.z()));
    }

    tree_->insertPointCloud(octo_cloud, origin,
                            cfg_.max_range,
                            /*lazy_eval=*/false,
                            /*discretize=*/true);
}

}  // namespace slam_map
