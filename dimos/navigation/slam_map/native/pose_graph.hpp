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
 * PoseGraph — GTSAM iSAM2 incremental pose graph optimisation.
 *
 * Thin wrapper around GTSAM that manages odometry and loop closure factors.
 *
 * Reference: Kaess et al., "iSAM2: Incremental Smoothing and Mapping Using
 *            the Bayes Tree", IJRR 2012.
 */

#pragma once

#include <vector>

#include <Eigen/Core>

// Forward-declare GTSAM types to avoid header pollution
namespace gtsam {
class ISAM2;
class Values;
class NonlinearFactorGraph;
}  // namespace gtsam

namespace slam_map {

struct PoseGraphConfig {
    double odom_noise_trans = 0.1;   // metres std-dev
    double odom_noise_rot   = 0.05;  // radians std-dev
    double loop_noise_trans  = 0.2;
    double loop_noise_rot    = 0.1;
};

class PoseGraph {
public:
    explicit PoseGraph(const PoseGraphConfig& cfg);
    ~PoseGraph();

    // Non-copyable, but movable (owns GTSAM heap objects via PIMPL)
    PoseGraph(const PoseGraph&) = delete;
    PoseGraph& operator=(const PoseGraph&) = delete;
    PoseGraph(PoseGraph&& other) noexcept;
    PoseGraph& operator=(PoseGraph&& other) noexcept;

    /// Add an odometry edge.  Returns the new node ID.
    size_t addOdometryEdge(const Eigen::Matrix4d& relative_transform);

    /// Add a loop closure edge between two existing nodes.
    void addLoopClosure(size_t node_from, size_t node_to,
                        const Eigen::Matrix4d& relative_transform);

    /// Run iSAM2 incremental update.
    /// Returns true if any pose changed by more than 0.01 m or 0.01 rad.
    bool optimize();

    /// Get optimised global pose for a node.
    Eigen::Matrix4d getPose(size_t node_id) const;

    /// Get the latest node's optimised global pose.
    Eigen::Matrix4d getLatestPose() const;

    /// Get all optimised poses (for scan re-integration after loop closure).
    std::vector<Eigen::Matrix4d> getAllPoses() const;

    size_t size() const { return next_id_; }

private:
    struct Impl;
    Impl* impl_;             // PIMPL to hide GTSAM headers
    PoseGraphConfig cfg_;
    size_t next_id_ = 0;
};

}  // namespace slam_map
