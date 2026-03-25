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

#include "icp_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

#include <Eigen/Dense>
#include <Eigen/SVD>

namespace slam_map {

// ============================================================================
// Construction
// ============================================================================

IcpOdometry::IcpOdometry(const IcpConfig& cfg)
    : cfg_(cfg),
      adaptive_threshold_(cfg.adaptive_threshold_init) {}

// ============================================================================
// Public: align
// ============================================================================

Eigen::Matrix4d IcpOdometry::align(
    const std::vector<Eigen::Vector3d>& scan_points,
    const Eigen::Matrix4d& odom_hint,
    uint8_t& degenerate_axes)
{
    degenerate_axes = 0;

    // Downsample the incoming scan
    auto source = downsample(scan_points, cfg_.voxel_downsample);
    if (source.empty()) {
        // No points — fall back to odometry
        global_pose_ = global_pose_ * odom_hint;
        return odom_hint;
    }

    // First scan: just add to local map, no alignment possible
    if (!hasData()) {
        // Transform source to world frame (identity for first scan)
        addToLocalMap(source);
        scan_count_++;
        return Eigen::Matrix4d::Identity();
    }

    // Run ICP against local map
    Eigen::Matrix<double, 6, 1> eigenvalues;
    Eigen::Matrix4d T_rel = runIcp(source, odom_hint, eigenvalues);

    // Degeneracy detection: check eigenvalue ratios
    double max_eig = eigenvalues.maxCoeff();
    if (max_eig > 0) {
        for (int i = 0; i < 6; ++i) {
            if (eigenvalues(i) / max_eig < cfg_.min_eigenvalue_ratio) {
                degenerate_axes |= (1 << i);
            }
        }
    }

    // For degenerate axes, substitute odom_hint components
    if (degenerate_axes != 0) {
        // Extract translation and rotation from both transforms
        Eigen::Vector3d t_icp = T_rel.block<3, 1>(0, 3);
        Eigen::Vector3d t_odom = odom_hint.block<3, 1>(0, 3);

        for (int i = 0; i < 3; ++i) {
            if (degenerate_axes & (1 << i)) {
                t_icp(i) = t_odom(i);
            }
        }
        T_rel.block<3, 1>(0, 3) = t_icp;

        // For rotational degeneracy, keep ICP rotation but this is a
        // simplification — full implementation would decompose into RPY
        // and substitute degenerate axes.  For now, if any rotation axis
        // is degenerate, we blend rotation with odom_hint.
        if (degenerate_axes & 0x38) {  // bits 3,4,5
            Eigen::Matrix3d R_icp = T_rel.block<3, 3>(0, 0);
            Eigen::Matrix3d R_odom = odom_hint.block<3, 3>(0, 0);
            // Simple blend: 50% ICP, 50% odom for degenerate rotation
            Eigen::Quaterniond q_icp(R_icp);
            Eigen::Quaterniond q_odom(R_odom);
            Eigen::Quaterniond q_blend = q_icp.slerp(0.5, q_odom);
            T_rel.block<3, 3>(0, 0) = q_blend.toRotationMatrix();
        }
    }

    // Update global pose
    global_pose_ = global_pose_ * T_rel;

    // Transform source to world frame and add to local map
    std::vector<Eigen::Vector3d> world_points;
    world_points.reserve(source.size());
    for (const auto& p : source) {
        Eigen::Vector4d pw = global_pose_ * Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0);
        world_points.emplace_back(pw.head<3>());
    }
    addToLocalMap(world_points);
    trimLocalMap();
    scan_count_++;

    return T_rel;
}

// ============================================================================
// Private: downsample
// ============================================================================

std::vector<Eigen::Vector3d> IcpOdometry::downsample(
    const std::vector<Eigen::Vector3d>& pts, double voxel_size) const
{
    if (voxel_size <= 0) return pts;

    double inv = 1.0 / voxel_size;
    std::unordered_map<VoxelKey, Eigen::Vector3d, VoxelKeyHash> grid;
    grid.reserve(pts.size() / 2);

    for (const auto& p : pts) {
        VoxelKey k{
            static_cast<int64_t>(std::floor(p.x() * inv)),
            static_cast<int64_t>(std::floor(p.y() * inv)),
            static_cast<int64_t>(std::floor(p.z() * inv)),
        };
        // Keep the first point in each voxel (simple, fast)
        grid.emplace(k, p);
    }

    std::vector<Eigen::Vector3d> result;
    result.reserve(grid.size());
    for (auto& [_, pt] : grid) {
        result.push_back(pt);
    }
    return result;
}

// ============================================================================
// Private: findCorrespondences
// ============================================================================

std::vector<std::pair<int, Eigen::Vector3d>> IcpOdometry::findCorrespondences(
    const std::vector<Eigen::Vector3d>& source_transformed,
    double max_dist) const
{
    // Brute-force nearest neighbour in the voxel map.
    // For each source point, look up its voxel and neighbouring voxels.
    double inv = 1.0 / cfg_.voxel_downsample;
    double max_dist_sq = max_dist * max_dist;

    std::vector<std::pair<int, Eigen::Vector3d>> correspondences;
    correspondences.reserve(source_transformed.size());

    for (int i = 0; i < static_cast<int>(source_transformed.size()); ++i) {
        const auto& p = source_transformed[i];
        int64_t cx = static_cast<int64_t>(std::floor(p.x() * inv));
        int64_t cy = static_cast<int64_t>(std::floor(p.y() * inv));
        int64_t cz = static_cast<int64_t>(std::floor(p.z() * inv));

        double best_dist_sq = max_dist_sq;
        Eigen::Vector3d best_point;
        bool found = false;

        // Search 3x3x3 neighbourhood
        for (int64_t dx = -1; dx <= 1; ++dx) {
            for (int64_t dy = -1; dy <= 1; ++dy) {
                for (int64_t dz = -1; dz <= 1; ++dz) {
                    VoxelKey k{cx + dx, cy + dy, cz + dz};
                    auto it = local_map_.find(k);
                    if (it == local_map_.end()) continue;

                    double d2 = (p - it->second).squaredNorm();
                    if (d2 < best_dist_sq) {
                        best_dist_sq = d2;
                        best_point = it->second;
                        found = true;
                    }
                }
            }
        }

        if (found) {
            correspondences.emplace_back(i, best_point);
        }
    }

    return correspondences;
}

// ============================================================================
// Private: runIcp
// ============================================================================

Eigen::Matrix4d IcpOdometry::runIcp(
    const std::vector<Eigen::Vector3d>& source,
    const Eigen::Matrix4d& initial_guess,
    Eigen::Matrix<double, 6, 1>& eigenvalues)
{
    Eigen::Matrix4d T = initial_guess;
    double prev_error = std::numeric_limits<double>::max();

    eigenvalues.setZero();

    for (int iter = 0; iter < cfg_.max_iterations; ++iter) {
        // Transform source points by current T, then by global_pose_ to world
        std::vector<Eigen::Vector3d> transformed;
        transformed.reserve(source.size());
        for (const auto& p : source) {
            Eigen::Vector4d pw = global_pose_ * T *
                                 Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0);
            transformed.push_back(pw.head<3>());
        }

        // Find correspondences
        auto corr = findCorrespondences(transformed, adaptive_threshold_);
        if (corr.size() < 10) break;

        // Collect matched pairs
        std::vector<Eigen::Vector3d> src_matched, tgt_matched;
        src_matched.reserve(corr.size());
        tgt_matched.reserve(corr.size());
        for (const auto& [idx, tgt_pt] : corr) {
            // Source in sensor frame, transformed by current T then global_pose_
            src_matched.push_back(transformed[idx]);
            tgt_matched.push_back(tgt_pt);
        }

        // Solve for delta transform
        auto [R_delta, t_delta] = solvePointToPoint(src_matched, tgt_matched);

        // Apply delta as a world-frame correction
        Eigen::Matrix4d T_delta = Eigen::Matrix4d::Identity();
        T_delta.block<3, 3>(0, 0) = R_delta;
        T_delta.block<3, 1>(0, 3) = t_delta;

        // Convert world-frame correction to relative frame
        // T_new = global_pose_^{-1} * T_delta * global_pose_ * T
        Eigen::Matrix4d gp_inv = global_pose_.inverse();
        T = gp_inv * T_delta * global_pose_ * T;

        // Check convergence
        double mean_error = 0;
        for (size_t i = 0; i < src_matched.size(); ++i) {
            mean_error += (src_matched[i] - tgt_matched[i]).squaredNorm();
        }
        mean_error /= static_cast<double>(src_matched.size());

        if (std::abs(prev_error - mean_error) < cfg_.convergence_threshold) {
            break;
        }
        prev_error = mean_error;
    }

    // Compute Hessian eigenvalues from final correspondences for degeneracy
    {
        std::vector<Eigen::Vector3d> transformed;
        transformed.reserve(source.size());
        for (const auto& p : source) {
            Eigen::Vector4d pw = global_pose_ * T *
                                 Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0);
            transformed.push_back(pw.head<3>());
        }

        auto corr = findCorrespondences(transformed, adaptive_threshold_);
        if (corr.size() >= 6) {
            // Build the 6×6 approximation of the Hessian (J^T J)
            Eigen::Matrix<double, 6, 6> H = Eigen::Matrix<double, 6, 6>::Zero();

            for (const auto& [idx, tgt_pt] : corr) {
                Eigen::Vector3d p = transformed[idx];
                Eigen::Vector3d e = p - tgt_pt;
                double en = e.norm();
                if (en < 1e-10) continue;
                Eigen::Vector3d n = e / en;  // error direction

                // Jacobian of point w.r.t. [tx, ty, tz, rx, ry, rz]
                Eigen::Matrix<double, 3, 6> J;
                J.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
                // Cross-product matrix for rotation
                J(0, 3) = 0;       J(0, 4) = p.z();  J(0, 5) = -p.y();
                J(1, 3) = -p.z();  J(1, 4) = 0;       J(1, 5) = p.x();
                J(2, 3) = p.y();   J(2, 4) = -p.x();  J(2, 5) = 0;

                Eigen::Matrix<double, 1, 6> Jn = n.transpose() * J;
                H += Jn.transpose() * Jn;
            }

            Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> solver(H);
            eigenvalues = solver.eigenvalues();
        }
    }

    // Update adaptive threshold (EMA of mean correspondence distance)
    {
        std::vector<Eigen::Vector3d> transformed;
        transformed.reserve(source.size());
        for (const auto& p : source) {
            Eigen::Vector4d pw = global_pose_ * T *
                                 Eigen::Vector4d(p.x(), p.y(), p.z(), 1.0);
            transformed.push_back(pw.head<3>());
        }
        auto corr = findCorrespondences(transformed, cfg_.max_corr_dist);
        if (!corr.empty()) {
            double mean_dist = 0;
            for (const auto& [idx, tgt_pt] : corr) {
                mean_dist += (transformed[idx] - tgt_pt).norm();
            }
            mean_dist /= static_cast<double>(corr.size());
            // EMA update
            constexpr double alpha = 0.1;
            adaptive_threshold_ = alpha * (3.0 * mean_dist) +
                                  (1.0 - alpha) * adaptive_threshold_;
            adaptive_threshold_ = std::clamp(adaptive_threshold_,
                                             0.3, cfg_.max_corr_dist);
        }
    }

    return T;
}

// ============================================================================
// Private: solvePointToPoint (SVD-based rigid body alignment)
// ============================================================================

std::pair<Eigen::Matrix3d, Eigen::Vector3d> IcpOdometry::solvePointToPoint(
    const std::vector<Eigen::Vector3d>& src,
    const std::vector<Eigen::Vector3d>& tgt)
{
    assert(src.size() == tgt.size());

    // Centroids
    Eigen::Vector3d cs = Eigen::Vector3d::Zero();
    Eigen::Vector3d ct = Eigen::Vector3d::Zero();
    for (size_t i = 0; i < src.size(); ++i) {
        cs += src[i];
        ct += tgt[i];
    }
    cs /= static_cast<double>(src.size());
    ct /= static_cast<double>(tgt.size());

    // Cross-covariance matrix
    Eigen::Matrix3d W = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < src.size(); ++i) {
        W += (src[i] - cs) * (tgt[i] - ct).transpose();
    }

    // SVD
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(W, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix3d U = svd.matrixU();
    Eigen::Matrix3d V = svd.matrixV();

    // Ensure proper rotation (det = +1)
    Eigen::Matrix3d R = V * U.transpose();
    if (R.determinant() < 0) {
        V.col(2) *= -1;
        R = V * U.transpose();
    }

    Eigen::Vector3d t = ct - R * cs;
    return {R, t};
}

// ============================================================================
// Private: local map management
// ============================================================================

void IcpOdometry::addToLocalMap(const std::vector<Eigen::Vector3d>& world_points)
{
    double inv = 1.0 / cfg_.voxel_downsample;
    ScanRecord record;

    for (const auto& p : world_points) {
        VoxelKey k{
            static_cast<int64_t>(std::floor(p.x() * inv)),
            static_cast<int64_t>(std::floor(p.y() * inv)),
            static_cast<int64_t>(std::floor(p.z() * inv)),
        };
        auto [it, inserted] = local_map_.emplace(k, p);
        if (inserted) {
            record.voxel_keys.push_back(k);
        }
    }

    scan_records_.push_back(std::move(record));
}

void IcpOdometry::trimLocalMap()
{
    while (static_cast<int>(scan_records_.size()) > cfg_.local_map_max_scans) {
        auto& oldest = scan_records_.front();
        for (const auto& k : oldest.voxel_keys) {
            local_map_.erase(k);
        }
        scan_records_.erase(scan_records_.begin());
    }
}

}  // namespace slam_map
