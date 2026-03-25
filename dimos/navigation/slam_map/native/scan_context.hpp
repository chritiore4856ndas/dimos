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
 * Scan Context — geometry-based place recognition for loop closure detection.
 *
 * Reference: Kim & Kim, "Scan Context: Egocentric Spatial Descriptor for
 *            Place Recognition within 3D Point Cloud Map", IROS 2018.
 */

#pragma once

#include <utility>
#include <vector>

#include <Eigen/Core>

namespace slam_map {

struct ScanContextConfig {
    int    num_rings        = 20;     // radial bins
    int    num_sectors      = 60;     // angular bins
    double max_range        = 20.0;   // metres
    double dist_threshold   = 0.2;    // cosine distance threshold
    int    exclude_recent   = 30;     // skip last N frames
    int    candidates_per_query = 5;  // top-K from KD-tree
};

class ScanContext {
public:
    using Descriptor = Eigen::MatrixXd;  // num_rings x num_sectors

    explicit ScanContext(const ScanContextConfig& cfg);

    /// Add a new scan and store its descriptor.  Returns its index.
    size_t addScan(const std::vector<Eigen::Vector3d>& scan_sensor_frame);

    /// Detect a loop closure for the most recently added scan.
    /// Returns (candidate_index, cosine_distance).
    /// candidate_index == -1 if no candidate found.
    std::pair<int, double> detectLoop() const;

    size_t size() const { return descriptors_.size(); }

private:
    // Compute the ring-sector height descriptor.
    Descriptor computeDescriptor(
        const std::vector<Eigen::Vector3d>& pts) const;

    // Ring key: column-wise mean of descriptor (1-D summary).
    Eigen::VectorXd computeRingKey(const Descriptor& desc) const;

    // Aligned cosine distance: try all sector rotations, return minimum.
    double alignedDistance(const Descriptor& a, const Descriptor& b) const;

    // Cosine distance between two vectors.
    static double cosineDistance(const Eigen::VectorXd& a, const Eigen::VectorXd& b);

    ScanContextConfig cfg_;
    std::vector<Descriptor> descriptors_;
    std::vector<Eigen::VectorXd> ring_keys_;
};

}  // namespace slam_map
