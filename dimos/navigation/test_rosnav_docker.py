# Copyright 2025-2026 Dimensional Inc.
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

"""Tests for rosnav_docker goal pose wiring and conversion."""

import sys
import types
from unittest.mock import MagicMock, patch

import pytest

from dimos.msgs.geometry_msgs import PoseStamped, Quaternion, Vector3


# ---------------------------------------------------------------------------
# ROS stub helpers — rclpy and ROS message types are not available on the host.
# We mock them so we can import rosnav_docker and test conversion logic.
# ---------------------------------------------------------------------------


class _StubROSTime:
    def __init__(self) -> None:
        self.sec = 0
        self.nanosec = 0


class _StubROSHeader:
    def __init__(self) -> None:
        self.stamp = _StubROSTime()
        self.frame_id = ""


class _StubROSPoint:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class _StubROSQuaternion:
    def __init__(self) -> None:
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.w = 1.0


class _StubROSPose:
    def __init__(self) -> None:
        self.position = _StubROSPoint()
        self.orientation = _StubROSQuaternion()


class _StubROSPoseStamped:
    def __init__(self) -> None:
        self.header = _StubROSHeader()
        self.pose = _StubROSPose()


def _install_ros_stubs() -> dict[str, types.ModuleType]:
    """Install fake rclpy / ROS message modules so rosnav_docker can import."""
    stubs: dict[str, types.ModuleType] = {}

    # rclpy
    rclpy = types.ModuleType("rclpy")
    rclpy.ok = lambda: True  # type: ignore[attr-defined]
    rclpy.init = lambda *a, **kw: None  # type: ignore[attr-defined]
    rclpy.spin_once = lambda *a, **kw: None  # type: ignore[attr-defined]
    stubs["rclpy"] = rclpy

    rclpy_node = types.ModuleType("rclpy.node")
    mock_node_cls = MagicMock()
    mock_node_cls.return_value = MagicMock()
    rclpy_node.Node = mock_node_cls  # type: ignore[attr-defined]
    stubs["rclpy.node"] = rclpy_node

    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = MagicMock()  # type: ignore[attr-defined]
    rclpy_qos.ReliabilityPolicy = MagicMock()  # type: ignore[attr-defined]
    stubs["rclpy.qos"] = rclpy_qos

    # geometry_msgs
    geom = types.ModuleType("geometry_msgs")
    geom_msg = types.ModuleType("geometry_msgs.msg")
    geom_msg.PoseStamped = _StubROSPoseStamped  # type: ignore[attr-defined]
    geom_msg.PointStamped = MagicMock()  # type: ignore[attr-defined]
    geom_msg.TwistStamped = MagicMock()  # type: ignore[attr-defined]
    geom.msg = geom_msg  # type: ignore[attr-defined]
    stubs["geometry_msgs"] = geom
    stubs["geometry_msgs.msg"] = geom_msg

    # nav_msgs
    nav = types.ModuleType("nav_msgs")
    nav_msg = types.ModuleType("nav_msgs.msg")
    nav_msg.Path = MagicMock()  # type: ignore[attr-defined]
    nav.msg = nav_msg  # type: ignore[attr-defined]
    stubs["nav_msgs"] = nav
    stubs["nav_msgs.msg"] = nav_msg

    # sensor_msgs
    sensor = types.ModuleType("sensor_msgs")
    sensor_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msg.CompressedImage = MagicMock()  # type: ignore[attr-defined]
    sensor_msg.Joy = MagicMock()  # type: ignore[attr-defined]
    sensor_msg.PointCloud2 = MagicMock()  # type: ignore[attr-defined]
    sensor.msg = sensor_msg  # type: ignore[attr-defined]
    stubs["sensor_msgs"] = sensor
    stubs["sensor_msgs.msg"] = sensor_msg

    # std_msgs
    std = types.ModuleType("std_msgs")
    std_msg = types.ModuleType("std_msgs.msg")
    std_msg.Bool = MagicMock()  # type: ignore[attr-defined]
    std_msg.Int8 = MagicMock()  # type: ignore[attr-defined]
    std.msg = std_msg  # type: ignore[attr-defined]
    stubs["std_msgs"] = std
    stubs["std_msgs.msg"] = std_msg

    # tf2_msgs
    tf2 = types.ModuleType("tf2_msgs")
    tf2_msg = types.ModuleType("tf2_msgs.msg")
    tf2_msg.TFMessage = MagicMock()  # type: ignore[attr-defined]
    tf2.msg = tf2_msg  # type: ignore[attr-defined]
    stubs["tf2_msgs"] = tf2
    stubs["tf2_msgs.msg"] = tf2_msg

    for name, mod in stubs.items():
        sys.modules[name] = mod

    return stubs


# Install stubs before importing rosnav_docker so the try/except picks them up.
_ros_stubs = _install_ros_stubs()

from dimos.navigation.rosnav_docker import _pose_to_ros  # noqa: E402


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------


class TestPoseToRosConversion:
    """Verify DimOS PoseStamped → ROS PoseStamped conversion."""

    def test_position_fields(self) -> None:
        pose = PoseStamped(
            ts=100.5,
            frame_id="map",
            position=Vector3(1.0, 2.0, 3.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        ros_pose = _pose_to_ros(pose)

        assert ros_pose.pose.position.x == 1.0
        assert ros_pose.pose.position.y == 2.0
        assert ros_pose.pose.position.z == 3.0

    def test_orientation_fields(self) -> None:
        pose = PoseStamped(
            ts=100.5,
            frame_id="base_link",
            position=Vector3(0.0, 0.0, 0.0),
            orientation=Quaternion(0.1, 0.2, 0.3, 0.9),
        )
        ros_pose = _pose_to_ros(pose)

        assert ros_pose.pose.orientation.x == pytest.approx(0.1)
        assert ros_pose.pose.orientation.y == pytest.approx(0.2)
        assert ros_pose.pose.orientation.z == pytest.approx(0.3)
        assert ros_pose.pose.orientation.w == pytest.approx(0.9)

    def test_timestamp_split(self) -> None:
        pose = PoseStamped(
            ts=123.456789,
            frame_id="map",
            position=Vector3(0.0, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        ros_pose = _pose_to_ros(pose)

        assert ros_pose.header.stamp.sec == 123
        assert ros_pose.header.stamp.nanosec == pytest.approx(456789000, abs=1)

    def test_frame_id(self) -> None:
        pose = PoseStamped(
            ts=1.0,
            frame_id="odom",
            position=Vector3(0.0, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        ros_pose = _pose_to_ros(pose)

        assert ros_pose.header.frame_id == "odom"

    def test_zero_timestamp(self) -> None:
        pose = PoseStamped(
            ts=0.0,
            frame_id="map",
            position=Vector3(0.0, 0.0, 0.0),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        # ts=0 triggers PoseStamped.__init__ to set ts=time.time(),
        # so we just verify the conversion doesn't crash and produces valid sec/nanosec
        ros_pose = _pose_to_ros(pose)
        assert ros_pose.header.stamp.sec >= 0
        assert ros_pose.header.stamp.nanosec >= 0

    def test_negative_position(self) -> None:
        pose = PoseStamped(
            ts=1.0,
            frame_id="map",
            position=Vector3(-5.5, -3.2, -0.1),
            orientation=Quaternion(0.0, 0.0, 0.0, 1.0),
        )
        ros_pose = _pose_to_ros(pose)

        assert ros_pose.pose.position.x == pytest.approx(-5.5)
        assert ros_pose.pose.position.y == pytest.approx(-3.2)
        assert ros_pose.pose.position.z == pytest.approx(-0.1)


class TestGoalRequestTransportWiring:
    """Verify the blueprint transports map goal_request to /goal_req."""

    def test_goal_request_mapped_to_goal_req_channel(self) -> None:
        """The primitive blueprint must map goal_request → /goal_req LCM channel."""
        from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_primitive_no_cam import (
            unitree_g1_primitive_no_cam,
        )

        transport_map = unitree_g1_primitive_no_cam.transport_map

        # goal_req (rosnav input) should be on /goal_req
        assert (
            "goal_req",
            PoseStamped,
        ) in transport_map, "goal_req transport not configured"
        assert transport_map[("goal_req", PoseStamped)].topic.topic == "/goal_req"

        # goal_request (websocket_vis output) should also be on /goal_req
        assert (
            "goal_request",
            PoseStamped,
        ) in transport_map, "goal_request transport not configured"
        assert transport_map[("goal_request", PoseStamped)].topic.topic == "/goal_req"

    def test_both_ports_share_same_lcm_channel(self) -> None:
        """goal_request and goal_req must use the same LCM topic."""
        from dimos.robot.unitree.g1.blueprints.primitive.unitree_g1_primitive_no_cam import (
            unitree_g1_primitive_no_cam,
        )

        transport_map = unitree_g1_primitive_no_cam.transport_map
        goal_req_topic = transport_map[("goal_req", PoseStamped)].topic.topic
        goal_request_topic = transport_map[("goal_request", PoseStamped)].topic.topic

        assert goal_req_topic == goal_request_topic
