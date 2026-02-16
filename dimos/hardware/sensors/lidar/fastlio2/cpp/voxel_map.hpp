// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Efficient global voxel map using a hash map.
// Supports O(1) insert/update, distance-based pruning, and
// convex hull-based clearing for scan integration.

#ifndef VOXEL_MAP_HPP_
#define VOXEL_MAP_HPP_

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/convex_hull.h>

struct VoxelKey {
    int32_t x, y, z;
    bool operator==(const VoxelKey& o) const { return x == o.x && y == o.y && z == o.z; }
};

struct VoxelKeyHash {
    size_t operator()(const VoxelKey& k) const {
        // Fast spatial hash — large primes reduce collisions for grid coords
        size_t h = static_cast<size_t>(k.x) * 73856093u;
        h ^= static_cast<size_t>(k.y) * 19349669u;
        h ^= static_cast<size_t>(k.z) * 83492791u;
        return h;
    }
};

struct Voxel {
    float x, y, z;       // running centroid
    float intensity;
    uint32_t count;       // points merged into this voxel
};

class VoxelMap {
public:
    explicit VoxelMap(float voxel_size, float max_range = 100.0f)
        : voxel_size_(voxel_size), max_range_(max_range) {
        map_.reserve(500000);
    }

    /// Insert a point cloud into the map, merging into existing voxels.
    template <typename PointT>
    void insert(const typename pcl::PointCloud<PointT>::Ptr& cloud) {
        if (!cloud) return;
        float inv = 1.0f / voxel_size_;
        for (const auto& pt : cloud->points) {
            VoxelKey key{
                static_cast<int32_t>(std::floor(pt.x * inv)),
                static_cast<int32_t>(std::floor(pt.y * inv)),
                static_cast<int32_t>(std::floor(pt.z * inv))};

            auto it = map_.find(key);
            if (it != map_.end()) {
                // Running average update
                auto& v = it->second;
                float n = static_cast<float>(v.count);
                float n1 = n + 1.0f;
                v.x = (v.x * n + pt.x) / n1;
                v.y = (v.y * n + pt.y) / n1;
                v.z = (v.z * n + pt.z) / n1;
                v.intensity = (v.intensity * n + pt.intensity) / n1;
                v.count++;
            } else {
                map_.emplace(key, Voxel{pt.x, pt.y, pt.z, pt.intensity, 1});
            }
        }
    }

    /// Clear all voxels inside the convex hull of the given cloud, then insert it.
    /// Computes the 3D convex hull, extracts outward-facing facet planes, and
    /// deletes any existing voxel whose centroid lies inside all half-planes.
    template <typename PointT>
    void hull_clear_and_insert(const typename pcl::PointCloud<PointT>::Ptr& cloud) {
        if (!cloud || cloud->size() < 4) {
            // Need at least 4 non-coplanar points for a 3D hull
            insert<PointT>(cloud);
            return;
        }

        // Compute 3D convex hull
        pcl::ConvexHull<PointT> hull;
        hull.setInputCloud(cloud);
        hull.setDimension(3);

        typename pcl::PointCloud<PointT>::Ptr hull_vertices(new pcl::PointCloud<PointT>());
        std::vector<pcl::Vertices> hull_polygons;
        hull.reconstruct(*hull_vertices, hull_polygons);

        if (hull_polygons.empty() || hull_vertices->empty()) {
            insert<PointT>(cloud);
            return;
        }

        // Compute hull centroid for orienting normals outward
        float cx = 0, cy = 0, cz = 0;
        for (const auto& pt : hull_vertices->points) {
            cx += pt.x; cy += pt.y; cz += pt.z;
        }
        float inv_n = 1.0f / static_cast<float>(hull_vertices->size());
        cx *= inv_n; cy *= inv_n; cz *= inv_n;

        // Extract facet planes: each polygon → outward normal + offset
        struct Plane {
            float nx, ny, nz, d;  // normal (outward) and dot(normal, vertex)
        };

        std::vector<Plane> planes;
        planes.reserve(hull_polygons.size());

        for (const auto& polygon : hull_polygons) {
            if (polygon.vertices.size() < 3) continue;

            const auto& p0 = hull_vertices->points[polygon.vertices[0]];
            const auto& p1 = hull_vertices->points[polygon.vertices[1]];
            const auto& p2 = hull_vertices->points[polygon.vertices[2]];

            // Two edges from p0
            float e1x = p1.x - p0.x, e1y = p1.y - p0.y, e1z = p1.z - p0.z;
            float e2x = p2.x - p0.x, e2y = p2.y - p0.y, e2z = p2.z - p0.z;

            // Cross product
            float nx = e1y * e2z - e1z * e2y;
            float ny = e1z * e2x - e1x * e2z;
            float nz = e1x * e2y - e1y * e2x;

            // Normalize
            float len = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (len < 1e-10f) continue;
            nx /= len;
            ny /= len;
            nz /= len;

            // Ensure normal points outward (away from centroid)
            float to_centroid_x = cx - p0.x;
            float to_centroid_y = cy - p0.y;
            float to_centroid_z = cz - p0.z;
            if (nx * to_centroid_x + ny * to_centroid_y + nz * to_centroid_z > 0) {
                nx = -nx; ny = -ny; nz = -nz;
            }

            planes.push_back({nx, ny, nz, nx * p0.x + ny * p0.y + nz * p0.z});
        }

        if (planes.empty()) {
            insert<PointT>(cloud);
            return;
        }

        // Delete voxels whose centroids are inside the hull.
        // Inside = dot(normal, centroid) <= d for ALL facets.
        size_t cleared = 0;
        for (auto it = map_.begin(); it != map_.end();) {
            const auto& v = it->second;
            bool inside = true;
            for (const auto& pl : planes) {
                if (pl.nx * v.x + pl.ny * v.y + pl.nz * v.z > pl.d) {
                    inside = false;
                    break;
                }
            }
            if (inside) {
                it = map_.erase(it);
                ++cleared;
            } else {
                ++it;
            }
        }

        if (cleared > 0) {
            printf("[voxel_map] hull_clear: %zu facets, cleared %zu voxels, map size %zu\n",
                   planes.size(), cleared, map_.size());
        }

        insert<PointT>(cloud);
    }

    /// Remove voxels farther than max_range from the given position.
    void prune(float px, float py, float pz) {
        float r2 = max_range_ * max_range_;
        for (auto it = map_.begin(); it != map_.end();) {
            float dx = it->second.x - px;
            float dy = it->second.y - py;
            float dz = it->second.z - pz;
            if (dx * dx + dy * dy + dz * dz > r2)
                it = map_.erase(it);
            else
                ++it;
        }
    }

    /// Export all voxel centroids as a point cloud.
    template <typename PointT>
    typename pcl::PointCloud<PointT>::Ptr to_cloud() const {
        typename pcl::PointCloud<PointT>::Ptr cloud(
            new pcl::PointCloud<PointT>(map_.size(), 1));
        size_t i = 0;
        for (const auto& [key, v] : map_) {
            auto& pt = cloud->points[i++];
            pt.x = v.x;
            pt.y = v.y;
            pt.z = v.z;
            pt.intensity = v.intensity;
        }
        return cloud;
    }

    size_t size() const { return map_.size(); }
    void clear() { map_.clear(); }
    void set_max_range(float r) { max_range_ = r; }

private:
    std::unordered_map<VoxelKey, Voxel, VoxelKeyHash> map_;
    float voxel_size_;
    float max_range_;
};

#endif
