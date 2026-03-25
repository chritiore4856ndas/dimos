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

#include "scan_context.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include <nanoflann.hpp>

namespace slam_map {

// ============================================================================
// nanoflann adaptor for ring-key vectors
// ============================================================================

struct RingKeyCloud {
    const std::vector<Eigen::VectorXd>* keys;

    size_t kdtree_get_point_count() const { return keys->size(); }

    double kdtree_get_pt(size_t idx, size_t dim) const {
        return (*keys)[idx](static_cast<Eigen::Index>(dim));
    }

    template <class BBox>
    bool kdtree_get_bbox(BBox& /*bb*/) const { return false; }
};

// ============================================================================
// Construction
// ============================================================================

ScanContext::ScanContext(const ScanContextConfig& cfg) : cfg_(cfg) {}

// ============================================================================
// Public: addScan
// ============================================================================

size_t ScanContext::addScan(const std::vector<Eigen::Vector3d>& scan_sensor_frame)
{
    auto desc = computeDescriptor(scan_sensor_frame);
    auto rk = computeRingKey(desc);

    size_t idx = descriptors_.size();
    descriptors_.push_back(std::move(desc));
    ring_keys_.push_back(std::move(rk));
    return idx;
}

// ============================================================================
// Public: detectLoop
// ============================================================================

std::pair<int, double> ScanContext::detectLoop() const
{
    if (descriptors_.size() < static_cast<size_t>(cfg_.exclude_recent + 1)) {
        return {-1, 1.0};
    }

    size_t query_idx = descriptors_.size() - 1;
    const auto& query_desc = descriptors_[query_idx];
    const auto& query_rk = ring_keys_[query_idx];

    // Build KD-tree over candidate ring keys (exclude recent)
    size_t n_candidates = descriptors_.size() - cfg_.exclude_recent - 1;
    if (n_candidates == 0) return {-1, 1.0};

    // Subset of ring keys for the tree
    std::vector<Eigen::VectorXd> candidate_keys(
        ring_keys_.begin(),
        ring_keys_.begin() + static_cast<long>(n_candidates));

    RingKeyCloud cloud{&candidate_keys};

    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, RingKeyCloud>,
        RingKeyCloud, -1, size_t>;

    int dim = cfg_.num_rings;
    KDTree tree(dim, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();

    // Query for top-K nearest ring keys
    size_t k = std::min(static_cast<size_t>(cfg_.candidates_per_query),
                        n_candidates);
    std::vector<size_t> indices(k);
    std::vector<double> dists(k);

    nanoflann::KNNResultSet<double> result_set(k);
    result_set.init(indices.data(), dists.data());
    tree.findNeighbors(result_set, query_rk.data());

    // Check candidates with full aligned distance
    int best_idx = -1;
    double best_dist = cfg_.dist_threshold;

    for (size_t i = 0; i < k; ++i) {
        size_t cand_idx = indices[i];
        double d = alignedDistance(query_desc, descriptors_[cand_idx]);
        if (d < best_dist) {
            best_dist = d;
            best_idx = static_cast<int>(cand_idx);
        }
    }

    return {best_idx, best_dist};
}

// ============================================================================
// Private: computeDescriptor
// ============================================================================

ScanContext::Descriptor ScanContext::computeDescriptor(
    const std::vector<Eigen::Vector3d>& pts) const
{
    Descriptor desc = Descriptor::Zero(cfg_.num_rings, cfg_.num_sectors);

    double ring_step = cfg_.max_range / cfg_.num_rings;
    double sector_step = 2.0 * M_PI / cfg_.num_sectors;

    for (const auto& p : pts) {
        double range = std::sqrt(p.x() * p.x() + p.y() * p.y());
        if (range > cfg_.max_range || range < 1e-3) continue;

        double angle = std::atan2(p.y(), p.x()) + M_PI;  // [0, 2π]

        int ring = static_cast<int>(range / ring_step);
        int sector = static_cast<int>(angle / sector_step);

        ring = std::clamp(ring, 0, cfg_.num_rings - 1);
        sector = std::clamp(sector, 0, cfg_.num_sectors - 1);

        // Max height encoding
        if (p.z() > desc(ring, sector)) {
            desc(ring, sector) = p.z();
        }
    }

    return desc;
}

// ============================================================================
// Private: computeRingKey
// ============================================================================

Eigen::VectorXd ScanContext::computeRingKey(const Descriptor& desc) const
{
    // Column-wise mean (one value per ring)
    Eigen::VectorXd rk(cfg_.num_rings);
    for (int r = 0; r < cfg_.num_rings; ++r) {
        rk(r) = desc.row(r).mean();
    }
    return rk;
}

// ============================================================================
// Private: alignedDistance
// ============================================================================

double ScanContext::alignedDistance(const Descriptor& a, const Descriptor& b) const
{
    // Try all sector rotations (column shifts) and return minimum distance
    double best = std::numeric_limits<double>::max();

    for (int shift = 0; shift < cfg_.num_sectors; ++shift) {
        // Column-shifted version of b
        Eigen::MatrixXd b_shifted(cfg_.num_rings, cfg_.num_sectors);
        for (int s = 0; s < cfg_.num_sectors; ++s) {
            b_shifted.col(s) = b.col((s + shift) % cfg_.num_sectors);
        }

        // Cosine distance between flattened descriptors
        Eigen::VectorXd va = Eigen::Map<const Eigen::VectorXd>(
            a.data(), a.size());
        Eigen::VectorXd vb = Eigen::Map<const Eigen::VectorXd>(
            b_shifted.data(), b_shifted.size());

        double d = cosineDistance(va, vb);
        if (d < best) best = d;
    }

    return best;
}

// ============================================================================
// Private: cosineDistance
// ============================================================================

double ScanContext::cosineDistance(const Eigen::VectorXd& a,
                                  const Eigen::VectorXd& b)
{
    double dot = a.dot(b);
    double na = a.norm();
    double nb = b.norm();
    if (na < 1e-10 || nb < 1e-10) return 1.0;
    return 1.0 - dot / (na * nb);
}

}  // namespace slam_map
