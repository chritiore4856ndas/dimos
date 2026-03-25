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

#include "pose_graph.hpp"

#include <cmath>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/Marginals.h>

namespace slam_map {

using gtsam::Symbol;
using gtsam::Pose3;
using gtsam::Rot3;
using gtsam::Point3;

static Symbol key(size_t id) { return Symbol('x', id); }

// Convert Eigen 4x4 to GTSAM Pose3
static Pose3 toPose3(const Eigen::Matrix4d& T) {
    return Pose3(T);
}

// Convert GTSAM Pose3 to Eigen 4x4
static Eigen::Matrix4d toMatrix4d(const Pose3& p) {
    return p.matrix();
}

// ============================================================================
// PIMPL implementation
// ============================================================================

struct PoseGraph::Impl {
    gtsam::ISAM2 isam2;
    gtsam::NonlinearFactorGraph new_factors;
    gtsam::Values new_values;
    gtsam::Values current_estimate;

    Impl() {
        gtsam::ISAM2Params params;
        params.relinearizeThreshold = 0.1;
        params.relinearizeSkip = 1;
        isam2 = gtsam::ISAM2(params);
    }
};

// ============================================================================
// Construction / Destruction
// ============================================================================

PoseGraph::PoseGraph(const PoseGraphConfig& cfg)
    : impl_(new Impl()), cfg_(cfg) {}

PoseGraph::~PoseGraph() {
    delete impl_;
}

PoseGraph::PoseGraph(PoseGraph&& other) noexcept
    : impl_(other.impl_), cfg_(other.cfg_), next_id_(other.next_id_)
{
    other.impl_ = nullptr;
}

PoseGraph& PoseGraph::operator=(PoseGraph&& other) noexcept {
    if (this != &other) {
        delete impl_;
        impl_ = other.impl_;
        cfg_ = other.cfg_;
        next_id_ = other.next_id_;
        other.impl_ = nullptr;
    }
    return *this;
}

// ============================================================================
// Public: addOdometryEdge
// ============================================================================

size_t PoseGraph::addOdometryEdge(const Eigen::Matrix4d& relative_transform)
{
    size_t new_id = next_id_;

    if (next_id_ == 0) {
        // First node: add prior at origin
        auto prior_noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-6, 1e-6, 1e-6).finished());
        impl_->new_factors.addPrior(key(0), Pose3::Identity(), prior_noise);
        impl_->new_values.insert(key(0), Pose3::Identity());
    } else {
        // Odometry factor from previous to new
        auto noise = gtsam::noiseModel::Diagonal::Sigmas(
            (gtsam::Vector(6) << cfg_.odom_noise_rot, cfg_.odom_noise_rot,
             cfg_.odom_noise_rot, cfg_.odom_noise_trans, cfg_.odom_noise_trans,
             cfg_.odom_noise_trans).finished());

        Pose3 odom = toPose3(relative_transform);
        impl_->new_factors.emplace_shared<gtsam::BetweenFactor<Pose3>>(
            key(next_id_ - 1), key(next_id_), odom, noise);

        // Initial estimate for new node
        Pose3 prev_pose = (impl_->current_estimate.exists(key(next_id_ - 1)))
            ? impl_->current_estimate.at<Pose3>(key(next_id_ - 1))
            : Pose3::Identity();
        impl_->new_values.insert(key(next_id_), prev_pose * odom);
    }

    next_id_++;
    return new_id;
}

// ============================================================================
// Public: addLoopClosure
// ============================================================================

void PoseGraph::addLoopClosure(size_t node_from, size_t node_to,
                               const Eigen::Matrix4d& relative_transform)
{
    auto noise = gtsam::noiseModel::Diagonal::Sigmas(
        (gtsam::Vector(6) << cfg_.loop_noise_rot, cfg_.loop_noise_rot,
         cfg_.loop_noise_rot, cfg_.loop_noise_trans, cfg_.loop_noise_trans,
         cfg_.loop_noise_trans).finished());

    impl_->new_factors.emplace_shared<gtsam::BetweenFactor<Pose3>>(
        key(node_from), key(node_to), toPose3(relative_transform), noise);
}

// ============================================================================
// Public: optimize
// ============================================================================

bool PoseGraph::optimize()
{
    if (impl_->new_factors.empty() && impl_->new_values.empty()) {
        return false;
    }

    impl_->isam2.update(impl_->new_factors, impl_->new_values);
    impl_->new_factors.resize(0);
    impl_->new_values.clear();

    gtsam::Values new_estimate = impl_->isam2.calculateEstimate();

    // Check if any pose changed significantly
    bool changed = false;
    if (!impl_->current_estimate.empty()) {
        for (const auto& kv : new_estimate) {
            if (impl_->current_estimate.exists(kv.key)) {
                Pose3 old_pose = impl_->current_estimate.at<Pose3>(kv.key);
                Pose3 new_pose = new_estimate.at<Pose3>(kv.key);
                Pose3 delta = old_pose.between(new_pose);
                double t_change = delta.translation().norm();
                double r_change = delta.rotation().rpy().norm();
                if (t_change > 0.01 || r_change > 0.01) {
                    changed = true;
                    break;
                }
            }
        }
    }

    impl_->current_estimate = new_estimate;
    return changed;
}

// ============================================================================
// Public: pose accessors
// ============================================================================

Eigen::Matrix4d PoseGraph::getPose(size_t node_id) const
{
    return toMatrix4d(impl_->current_estimate.at<Pose3>(key(node_id)));
}

Eigen::Matrix4d PoseGraph::getLatestPose() const
{
    if (next_id_ == 0) return Eigen::Matrix4d::Identity();
    return getPose(next_id_ - 1);
}

std::vector<Eigen::Matrix4d> PoseGraph::getAllPoses() const
{
    std::vector<Eigen::Matrix4d> poses;
    poses.reserve(next_id_);
    for (size_t i = 0; i < next_id_; ++i) {
        if (impl_->current_estimate.exists(key(i))) {
            poses.push_back(toMatrix4d(impl_->current_estimate.at<Pose3>(key(i))));
        } else {
            poses.push_back(Eigen::Matrix4d::Identity());
        }
    }
    return poses;
}

}  // namespace slam_map
