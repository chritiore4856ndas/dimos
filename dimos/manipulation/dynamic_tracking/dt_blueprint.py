# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

# Directory where this file is located
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))

"""
Blueprint for ArUco marker tracking with RealSense camera.

This module provides declarative blueprints for combining the ArucoTracker
with a RealSense camera, optionally with XArm6 manipulation.

Usage:
    # ArUco tracking only:
    from dimos.manipulation.dynamic_tracking.blueprint import aruco_tracker_realsense

    coordinator = aruco_tracker_realsense.build()
    coordinator.start_all_modules()

    # ArUco tracking + XArm6 manipulation:
    from dimos.manipulation.dynamic_tracking.blueprint import aruco_tracker_realsense_xarm6

    coordinator = aruco_tracker_realsense_xarm6.build()
    coordinator.start_all_modules()

    # Or customize:
    from dimos.manipulation.dynamic_tracking import aruco_tracker
    from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
    from dimos.core.blueprints import autoconnect

    my_tracker = autoconnect(
        RealSenseCamera.blueprint(width=848, height=480, fps=30),
        aruco_tracker(marker_size=0.05, save_images=True),
    )
"""

import cv2

from dimos.core.blueprints import autoconnect
from dimos.core.transport import LCMTransport
from dimos.hardware.sensors.camera.realsense.camera import RealSenseCamera
from dimos.manipulation.dynamic_tracking.aruco_tracker import aruco_tracker
# from dimos.manipulation.manipulation_blueprints import xarm6_manipulation
from dimos.msgs.geometry_msgs import Quaternion, Transform, Vector3
from dimos.msgs.sensor_msgs import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.robot.foxglove_bridge import foxglove_bridge

# =============================================================================
# ArUco Tracker with RealSense Camera
# =============================================================================
# Combines:
#   - RealSenseCamera: RGB-D camera with hardware interface
#   - ArucoTracker: Detects ArUco markers and computes transforms
#
# Data flow:
#   RealSenseCamera.color_image ──► ArucoTracker.color_image (marker detection)
#   RealSenseCamera.camera_info ──► ArucoTracker.camera_info (intrinsics)
# =============================================================================

aruco_tracker_realsense = autoconnect(
    RealSenseCamera.blueprint(
        width=848,
        height=480,
        fps=15,
        camera_name="camera",
        base_frame_id="ee_link",
        base_transform=Transform(
            translation=Vector3(0.067052239, -0.0311387575, 0.021611456),
            rotation=Quaternion(0.0015569323, -0.0044709112, 0.7138064706, 0.7003270047),
        ),
        enable_depth=True,
        align_depth_to_color=False,
    ),
    aruco_tracker(
        marker_size=0.027,  # 27mm markers (default)
        aruco_dict=cv2.aruco.DICT_4X4_50,
        camera_frame_id="camera_color_optical_frame",
        target_marker_id=0,  # Only track marker ID 0 (set to None to track all)
        save_images=True,
        output_dir=os.path.join(_THIS_DIR, "aruco_output"),
        processing_rate=1,
        max_loops=30,
        move_robot_to_aruco=False,
    ),
    foxglove_bridge(),
).transports(
    {
        # Camera color image for ArUco detection
        ("color_image", Image): LCMTransport("/camera/color", Image),
        # Camera info for pose estimation
        ("camera_info", CameraInfo): LCMTransport("/camera/color_info", CameraInfo),
    }
).global_config(viewer_backend="foxglove")


__all__ = ["aruco_tracker_realsense", "aruco_tracker_realsense_xarm6"]
