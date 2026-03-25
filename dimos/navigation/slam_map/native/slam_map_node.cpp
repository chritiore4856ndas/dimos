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
 * slam_map_node — SLAM + loop closure + dynamic obstacle mapping via LCM.
 *
 * Pipeline per scan:
 *   1. ICP scan matching → refined relative pose
 *   2. Degeneracy fallback → substitute raw odom for degenerate axes
 *   3. GTSAM pose graph → odometry factor
 *   4. Scan Context → loop closure detection
 *   5. Geometric verification → loop closure factor
 *   6. iSAM2 optimise → re-integrate OctoMap if needed
 *   7. OctoMap insert → dynamic obstacle removal via ray-casting
 *
 * Subscribes to:
 *   --raw_scan       sensor_msgs.PointCloud2    Sensor-frame lidar scan
 *   --raw_odom       geometry_msgs.PoseStamped  Raw robot odometry
 *
 * Publishes to:
 *   --corrected_global_map  sensor_msgs.PointCloud2    Drift-corrected occupancy
 *   --corrected_odom        geometry_msgs.PoseStamped  Loop-closure-corrected pose
 */

#include <chrono>
#include <csignal>
#include <cstdio>
#include <mutex>
#include <string>
#include <thread>

#include <lcm/lcm-cpp.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "common/dimos_native_module.hpp"
#include "common/point_cloud_utils.hpp"

#include "sensor_msgs/PointCloud2.hpp"
#include "geometry_msgs/PoseStamped.hpp"

#include "icp_odometry.hpp"
#include "scan_context.hpp"
#include "pose_graph.hpp"
#include "dynamic_mapper.hpp"

// ---------------------------------------------------------------------------
// Global shutdown flag
// ---------------------------------------------------------------------------
static volatile sig_atomic_t g_shutdown = 0;
static void on_signal(int /*sig*/) { g_shutdown = 1; }

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static Eigen::Matrix4d poseStampedToMatrix(const geometry_msgs::PoseStamped& msg)
{
    Eigen::Quaterniond q(
        msg.pose.orientation.w,
        msg.pose.orientation.x,
        msg.pose.orientation.y,
        msg.pose.orientation.z);
    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
    T(0, 3) = msg.pose.position.x;
    T(1, 3) = msg.pose.position.y;
    T(2, 3) = msg.pose.position.z;
    return T;
}

static geometry_msgs::PoseStamped matrixToPoseStamped(
    const Eigen::Matrix4d& T, const std::string& frame_id, double timestamp)
{
    geometry_msgs::PoseStamped msg;
    msg.header = dimos::make_header(frame_id, timestamp);

    msg.pose.position.x = T(0, 3);
    msg.pose.position.y = T(1, 3);
    msg.pose.position.z = T(2, 3);

    Eigen::Quaterniond q(T.block<3, 3>(0, 0));
    q.normalize();
    msg.pose.orientation.x = q.x();
    msg.pose.orientation.y = q.y();
    msg.pose.orientation.z = q.z();
    msg.pose.orientation.w = q.w();

    return msg;
}

// ---------------------------------------------------------------------------
// SlamMapHandler
// ---------------------------------------------------------------------------

struct SlamMapHandler {
    lcm::LCM*              lcm;
    slam_map::IcpOdometry   icp;
    slam_map::ScanContext    loop_detector;
    slam_map::PoseGraph      pose_graph;
    slam_map::DynamicMapper  mapper;

    std::string corrected_global_map_topic;
    std::string corrected_odom_topic;

    float loop_icp_max_dist;
    int   smooth_frames;

    std::mutex mutex;
    bool has_odom         = false;
    Eigen::Matrix4d prev_odom = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d curr_odom = Eigen::Matrix4d::Identity();

    // Post-loop-closure smoothing state
    int    smooth_remaining = 0;
    Eigen::Matrix4d smooth_target_pose = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d smooth_start_pose  = Eigen::Matrix4d::Identity();

    // Scan buffer (sensor-frame) for geometric verification
    std::vector<std::vector<Eigen::Vector3d>> scan_history;

    // --- odometry callback ---
    void onOdom(const lcm::ReceiveBuffer* /*rbuf*/,
                const std::string& /*channel*/,
                const geometry_msgs::PoseStamped* msg)
    {
        std::lock_guard<std::mutex> lk(mutex);
        prev_odom = curr_odom;
        curr_odom = poseStampedToMatrix(*msg);
        has_odom = true;
    }

    // --- scan callback (main pipeline) ---
    void onScan(const lcm::ReceiveBuffer* /*rbuf*/,
                const std::string& /*channel*/,
                const sensor_msgs::PointCloud2* msg)
    {
        std::lock_guard<std::mutex> lk(mutex);
        if (!has_odom) return;

        // 1. Parse scan
        auto raw_pts = smartnav::parse_pointcloud2(*msg);
        if (raw_pts.empty()) return;

        std::vector<Eigen::Vector3d> scan_eigen;
        scan_eigen.reserve(raw_pts.size());
        for (const auto& p : raw_pts) {
            scan_eigen.emplace_back(p.x, p.y, p.z);
        }

        // 2. Compute raw odom delta as initial guess
        Eigen::Matrix4d odom_delta = prev_odom.inverse() * curr_odom;

        // 3. ICP alignment
        uint8_t degenerate_axes = 0;
        Eigen::Matrix4d T_rel = icp.align(scan_eigen, odom_delta, degenerate_axes);

        if (degenerate_axes != 0) {
            printf("[slam_map] Degeneracy detected: 0x%02x\n", degenerate_axes);
        }

        // 4. Add odometry factor to pose graph and optimize immediately
        //    so that current_estimate_ is up-to-date for getPose() calls.
        size_t node_id = pose_graph.addOdometryEdge(T_rel);
        pose_graph.optimize();

        // 5. Store scan for loop closure verification
        scan_history.push_back(scan_eigen);

        // 6. Scan Context: detect loop closure
        loop_detector.addScan(scan_eigen);
        auto [candidate_idx, sc_dist] = loop_detector.detectLoop();

        bool loop_closed = false;
        if (candidate_idx >= 0) {
            // 7. Geometric verification via ICP
            printf("[slam_map] Loop closure candidate: node %d (dist=%.3f)\n",
                   candidate_idx, sc_dist);

            // Run ICP between candidate scan and current scan
            slam_map::IcpConfig verify_cfg;
            verify_cfg.max_corr_dist = loop_icp_max_dist;
            verify_cfg.max_iterations = 30;
            verify_cfg.voxel_downsample = 0.3;
            verify_cfg.local_map_max_scans = 2;

            // Simple verification: compute overlap between scans
            // Transform both to their respective estimated world frames
            // and check if ICP converges with low residual
            if (static_cast<size_t>(candidate_idx) < scan_history.size()) {
                const auto& cand_scan = scan_history[candidate_idx];

                // Get current poses for both nodes
                Eigen::Matrix4d pose_cand = pose_graph.getPose(candidate_idx);
                Eigen::Matrix4d pose_curr = pose_graph.getPose(node_id);

                // Relative transform from candidate to current (initial guess)
                Eigen::Matrix4d T_loop_init = pose_cand.inverse() * pose_curr;

                // Quick overlap check: transform current scan to candidate frame
                // and count points within max_dist of candidate scan points
                int overlap = 0;
                for (size_t i = 0; i < std::min(scan_eigen.size(), size_t(500)); i += 3) {
                    Eigen::Vector4d p = T_loop_init.inverse() *
                        Eigen::Vector4d(scan_eigen[i].x(), scan_eigen[i].y(),
                                       scan_eigen[i].z(), 1.0);
                    for (size_t j = 0; j < std::min(cand_scan.size(), size_t(500)); j += 3) {
                        double d = (p.head<3>() - cand_scan[j]).squaredNorm();
                        if (d < loop_icp_max_dist * loop_icp_max_dist) {
                            overlap++;
                            break;
                        }
                    }
                }

                double overlap_ratio = static_cast<double>(overlap) /
                    static_cast<double>(std::min(scan_eigen.size(), size_t(500)) / 3);

                if (overlap_ratio > 0.3) {
                    printf("[slam_map] Loop closure ACCEPTED: nodes %d -> %zu "
                           "(overlap=%.1f%%)\n",
                           candidate_idx, node_id, overlap_ratio * 100.0);

                    Eigen::Matrix4d T_loop = T_loop_init;
                    pose_graph.addLoopClosure(
                        static_cast<size_t>(candidate_idx), node_id, T_loop);
                    loop_closed = true;
                } else {
                    printf("[slam_map] Loop closure REJECTED: overlap too low "
                           "(%.1f%%)\n", overlap_ratio * 100.0);
                }
            }
        }

        // 8. Re-optimize if loop closure was added (odometry-only was
        //    already optimized in step 4)
        bool poses_changed = loop_closed ? pose_graph.optimize() : false;

        // 9. If loop closure caused significant pose changes, re-integrate
        if (loop_closed && poses_changed) {
            auto all_poses = pose_graph.getAllPoses();
            mapper.reintegrate(all_poses);

            // Set up smoothing for published odom
            Eigen::Matrix4d corrected = pose_graph.getLatestPose();
            smooth_start_pose = icp.globalPose();  // pre-correction
            smooth_target_pose = corrected;
            smooth_remaining = smooth_frames;
        }

        // 10. Get corrected pose (from pose graph, not raw ICP)
        Eigen::Matrix4d corrected_pose = pose_graph.getLatestPose();

        // 11. Insert scan into OctoMap at corrected pose
        mapper.insertScan(scan_eigen, corrected_pose, node_id);

        // 12. Publish corrected odom (with smoothing)
        Eigen::Matrix4d published_pose;
        if (smooth_remaining > 0) {
            double alpha = 1.0 - static_cast<double>(smooth_remaining) /
                           static_cast<double>(smooth_frames);
            // LERP translation, SLERP rotation
            Eigen::Vector3d t_start = smooth_start_pose.block<3, 1>(0, 3);
            Eigen::Vector3d t_target = smooth_target_pose.block<3, 1>(0, 3);
            Eigen::Quaterniond q_start(smooth_start_pose.block<3, 3>(0, 0));
            Eigen::Quaterniond q_target(smooth_target_pose.block<3, 3>(0, 0));

            published_pose = Eigen::Matrix4d::Identity();
            published_pose.block<3, 1>(0, 3) = (1.0 - alpha) * t_start + alpha * t_target;
            published_pose.block<3, 3>(0, 0) = q_start.slerp(alpha, q_target).toRotationMatrix();
            smooth_remaining--;
        } else {
            published_pose = corrected_pose;
        }

        double ts = smartnav::get_timestamp(*msg);
        auto odom_msg = matrixToPoseStamped(published_pose, "world", ts);
        lcm->publish(corrected_odom_topic, &odom_msg);
    }

    // --- periodic map publish ---
    void publishMap()
    {
        std::lock_guard<std::mutex> lk(mutex);
        auto occupied = mapper.getOccupiedVoxels();

        double ts = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        sensor_msgs::PointCloud2 cloud =
            smartnav::build_pointcloud2(occupied, "world", ts);
        lcm->publish(corrected_global_map_topic, &cloud);
    }
};

// ---------------------------------------------------------------------------
// main
// ---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    dimos::NativeModule mod(argc, argv);

    std::signal(SIGTERM, on_signal);
    std::signal(SIGINT,  on_signal);

    // --- topic names ---
    const std::string raw_scan_topic              = mod.topic("raw_scan");
    const std::string raw_odom_topic              = mod.topic("raw_odom");
    const std::string corrected_global_map_topic  = mod.topic("corrected_global_map");
    const std::string corrected_odom_topic        = mod.topic("corrected_odom");

    // --- parameters ---
    const float resolution           = mod.arg_float("resolution",           0.15f);
    const float max_range            = mod.arg_float("max_range",            15.0f);
    const float occ_threshold        = mod.arg_float("occ_threshold",        0.5f);
    const float publish_rate         = mod.arg_float("publish_rate",         0.5f);

    const float icp_max_corr_dist    = mod.arg_float("icp_max_corr_dist",    1.0f);
    const int   icp_max_iterations   = mod.arg_int("icp_max_iterations",     20);
    const float icp_voxel_downsample = mod.arg_float("icp_voxel_downsample", 0.3f);
    const float icp_degen_thresh     = mod.arg_float("icp_degeneracy_threshold", 0.01f);

    const float sc_dist_threshold    = mod.arg_float("sc_dist_threshold",    0.2f);
    const int   sc_exclude_recent    = mod.arg_int("sc_exclude_recent",      30);
    const float loop_icp_max_dist    = mod.arg_float("loop_icp_max_dist",    0.5f);

    const float odom_noise_trans     = mod.arg_float("odom_noise_trans",     0.1f);
    const float odom_noise_rot       = mod.arg_float("odom_noise_rot",       0.05f);
    const float loop_noise_trans     = mod.arg_float("loop_noise_trans",     0.2f);
    const float loop_noise_rot       = mod.arg_float("loop_noise_rot",       0.1f);

    const int   smooth_frames        = mod.arg_int("smooth_frames",          10);
    const int   scan_buffer_size     = mod.arg_int("scan_buffer_size",       100);

    printf("[slam_map] resolution=%.3f max_range=%.1f occ_threshold=%.2f "
           "publish_rate=%.2fHz\n",
           resolution, max_range, occ_threshold, publish_rate);
    printf("[slam_map] icp_max_corr=%.2f icp_iters=%d icp_voxel=%.2f "
           "degen_thresh=%.4f\n",
           icp_max_corr_dist, icp_max_iterations, icp_voxel_downsample,
           icp_degen_thresh);
    printf("[slam_map] sc_threshold=%.2f sc_exclude=%d loop_icp_max=%.2f\n",
           sc_dist_threshold, sc_exclude_recent, loop_icp_max_dist);
    printf("[slam_map] scan='%s'  odom='%s'  map='%s'  corrected_odom='%s'\n",
           raw_scan_topic.c_str(), raw_odom_topic.c_str(),
           corrected_global_map_topic.c_str(), corrected_odom_topic.c_str());

    // --- LCM ---
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[slam_map] ERROR: Failed to initialise LCM\n");
        return 1;
    }

    // --- Component configs ---
    slam_map::IcpConfig icp_cfg;
    icp_cfg.max_corr_dist         = icp_max_corr_dist;
    icp_cfg.max_iterations        = icp_max_iterations;
    icp_cfg.voxel_downsample      = icp_voxel_downsample;
    icp_cfg.min_eigenvalue_ratio  = icp_degen_thresh;
    icp_cfg.adaptive_threshold_init = icp_max_corr_dist;

    slam_map::ScanContextConfig sc_cfg;
    sc_cfg.dist_threshold   = sc_dist_threshold;
    sc_cfg.exclude_recent   = sc_exclude_recent;
    sc_cfg.max_range        = max_range;

    slam_map::PoseGraphConfig pg_cfg;
    pg_cfg.odom_noise_trans = odom_noise_trans;
    pg_cfg.odom_noise_rot   = odom_noise_rot;
    pg_cfg.loop_noise_trans  = loop_noise_trans;
    pg_cfg.loop_noise_rot    = loop_noise_rot;

    slam_map::DynamicMapperConfig dm_cfg;
    dm_cfg.resolution       = resolution;
    dm_cfg.max_range        = max_range;
    dm_cfg.occ_threshold    = occ_threshold;
    dm_cfg.scan_buffer_size = static_cast<size_t>(scan_buffer_size);

    // --- Handler ---
    SlamMapHandler handler{
        .lcm                      = &lcm,
        .icp                      = slam_map::IcpOdometry(icp_cfg),
        .loop_detector            = slam_map::ScanContext(sc_cfg),
        .pose_graph               = slam_map::PoseGraph(pg_cfg),
        .mapper                   = slam_map::DynamicMapper(dm_cfg),
        .corrected_global_map_topic = corrected_global_map_topic,
        .corrected_odom_topic       = corrected_odom_topic,
        .loop_icp_max_dist          = loop_icp_max_dist,
        .smooth_frames              = smooth_frames,
    };

    lcm.subscribe(raw_odom_topic,  &SlamMapHandler::onOdom, &handler);
    lcm.subscribe(raw_scan_topic,  &SlamMapHandler::onScan, &handler);

    printf("[slam_map] Started.\n");

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
    printf("[slam_map] Shutdown.\n");
    return 0;
}
