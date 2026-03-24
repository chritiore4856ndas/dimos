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
 * dynamic_map_node — OctoMap-based 3-D occupancy grid via LCM.
 *
 * Subscribes to:
 *   --registered_scan   sensor_msgs.PointCloud2   World-frame lidar scan
 *   --raw_odom          geometry_msgs.PoseStamped  Sensor origin pose
 *
 * Publishes to:
 *   --global_map        sensor_msgs.PointCloud2   Occupied voxels (log-odds > threshold)
 *   --odom              geometry_msgs.PoseStamped  Pass-through odometry
 *
 * Parameters (--key value):
 *   --resolution        float   Voxel resolution in metres        (default 0.15)
 *   --max_range         float   Max ray length in metres          (default 15.0)
 *   --occ_threshold     float   Log-odds publish threshold        (default 0.5)
 *   --publish_rate      float   Map publish frequency in Hz       (default 0.5)
 *
 * The node uses OctoMap's `insertPointCloud()` which performs full 3-D DDA
 * ray-casting and Bayesian log-odds updates internally.  Dynamic obstacle
 * removal emerges from rays passing through formerly-occupied voxels.
 *
 * References:
 *   A. Hornung et al., "OctoMap: An efficient probabilistic 3D mapping
 *   framework based on octrees", Autonomous Robots 34(3), 2013.
 */

#include <chrono>
#include <csignal>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

#include <lcm/lcm-cpp.hpp>

#include <octomap/octomap.h>
#include <octomap/OcTree.h>

#include "common/dimos_native_module.hpp"
#include "common/point_cloud_utils.hpp"

#include "sensor_msgs/PointCloud2.hpp"
#include "geometry_msgs/PoseStamped.hpp"

// ---------------------------------------------------------------------------
// Global shutdown flag (set by SIGTERM / SIGINT)
// ---------------------------------------------------------------------------
static volatile sig_atomic_t g_shutdown = 0;
static void on_signal(int /*sig*/) { g_shutdown = 1; }

// ---------------------------------------------------------------------------
// Message handler
// ---------------------------------------------------------------------------
struct DynamicMapHandler {
    lcm::LCM*         lcm;
    octomap::OcTree*  tree;

    std::string       global_map_topic;
    std::string       odom_topic;

    float             max_range;
    float             occ_threshold;

    std::mutex        tree_mutex;  // protects tree + odom state
    bool              has_odom = false;
    octomap::point3d  sensor_origin{0.0f, 0.0f, 0.0f};

    // --- odometry callback ---
    void onOdom(const lcm::ReceiveBuffer* /*rbuf*/,
                const std::string& /*channel*/,
                const geometry_msgs::PoseStamped* msg) {
        std::lock_guard<std::mutex> lk(tree_mutex);
        sensor_origin = octomap::point3d(
            static_cast<float>(msg->pose.position.x),
            static_cast<float>(msg->pose.position.y),
            static_cast<float>(msg->pose.position.z));
        has_odom = true;

        // Pass odom through unchanged
        lcm->publish(odom_topic, msg);
    }

    // --- lidar scan callback ---
    void onScan(const lcm::ReceiveBuffer* /*rbuf*/,
                const std::string& /*channel*/,
                const sensor_msgs::PointCloud2* msg) {
        std::lock_guard<std::mutex> lk(tree_mutex);
        if (!has_odom) return;

        // Parse scan into OctoMap point cloud
        auto raw_pts = smartnav::parse_pointcloud2(*msg);
        if (raw_pts.empty()) return;

        octomap::Pointcloud octo_cloud;
        octo_cloud.reserve(raw_pts.size());
        for (const auto& p : raw_pts) {
            octo_cloud.push_back(p.x, p.y, p.z);
        }

        // OctoMap performs full 3-D DDA ray-casting internally:
        //   - Each ray from sensor_origin to endpoint
        //   - Intermediate voxels decremented (log_odds_miss)
        //   - Endpoint voxel incremented (log_odds_hit)
        tree->insertPointCloud(octo_cloud, sensor_origin,
                               static_cast<double>(max_range),
                               /*lazy_eval=*/false,
                               /*discretize=*/true);
    }

    // --- periodic map publish ---
    void publishMap() {
        std::lock_guard<std::mutex> lk(tree_mutex);
        std::vector<smartnav::PointXYZI> occupied;
        occupied.reserve(4096);

        for (auto it = tree->begin_leafs(); it != tree->end_leafs(); ++it) {
            if (it->getLogOdds() > occ_threshold) {
                smartnav::PointXYZI p;
                p.x = static_cast<float>(it.getX());
                p.y = static_cast<float>(it.getY());
                p.z = static_cast<float>(it.getZ());
                p.intensity = it->getLogOdds();
                occupied.push_back(p);
            }
        }

        double ts = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        sensor_msgs::PointCloud2 cloud = smartnav::build_pointcloud2(occupied, "world", ts);
        lcm->publish(global_map_topic, &cloud);
    }
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
    dimos::NativeModule mod(argc, argv);

    std::signal(SIGTERM, on_signal);
    std::signal(SIGINT,  on_signal);

    // --- topic names ---
    const std::string registered_scan_topic = mod.topic("registered_scan");
    const std::string raw_odom_topic         = mod.topic("raw_odom");
    const std::string global_map_topic       = mod.topic("global_map");
    const std::string odom_topic             = mod.topic("odom");

    // --- parameters ---
    const float resolution    = mod.arg_float("resolution",    0.15f);
    const float max_range     = mod.arg_float("max_range",     15.0f);
    const float occ_threshold = mod.arg_float("occ_threshold", 0.5f);
    const float publish_rate  = mod.arg_float("publish_rate",  0.5f);

    printf("[dynamic_map] resolution=%.3f max_range=%.1f occ_threshold=%.2f publish_rate=%.2fHz\n",
           resolution, max_range, occ_threshold, publish_rate);
    printf("[dynamic_map] scan='%s'  odom='%s'  map='%s'\n",
           registered_scan_topic.c_str(), raw_odom_topic.c_str(), global_map_topic.c_str());

    // --- LCM ---
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[dynamic_map] ERROR: Failed to initialise LCM\n");
        return 1;
    }

    // --- OctoMap ---
    octomap::OcTree tree(static_cast<double>(resolution));
    // Tighter clamping than default to allow dynamic removal.
    // OctoMap API takes probability (0-1); these correspond to log-odds ≈ -2.0 / +3.5.
    tree.setClampingThresMin(0.1192);
    tree.setClampingThresMax(0.9707);

    // --- handler ---
    DynamicMapHandler handler;
    handler.lcm             = &lcm;
    handler.tree            = &tree;
    handler.global_map_topic = global_map_topic;
    handler.odom_topic       = odom_topic;
    handler.max_range        = max_range;
    handler.occ_threshold    = occ_threshold;

    lcm.subscribe(raw_odom_topic,         &DynamicMapHandler::onOdom, &handler);
    lcm.subscribe(registered_scan_topic,  &DynamicMapHandler::onScan, &handler);

    printf("[dynamic_map] Started.\n");

    // --- publish timer thread ---
    const long publish_us = static_cast<long>(1'000'000.0f / publish_rate);
    std::thread pub_thread([&]() {
        while (!g_shutdown) {
            std::this_thread::sleep_for(std::chrono::microseconds(publish_us));
            handler.publishMap();
        }
    });

    // --- LCM spin ---
    while (!g_shutdown) {
        lcm.handleTimeout(10 /*ms*/);
    }

    pub_thread.join();
    printf("[dynamic_map] Shutdown.\n");
    return 0;
}
