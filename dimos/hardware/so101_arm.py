# Copyright 2025 Dimensional Inc.
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

import select
import sys
import termios
import threading
import time
import tty

from dimos_lcm.geometry_msgs import Pose, Twist, Vector3
import kinpy as kp
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import (
    FeetechMotorsBus,
    OperatingMode,
)
import numpy as np
import pytest
from reactivex.disposable import Disposable
from scipy.spatial.transform import Rotation as R

import dimos.core as core
from dimos.core import In, Module, rpc
import dimos.protocol.service.lcmservice as lcmservice
from dimos.utils.logging_config import setup_logger
from dimos.utils.transform_utils import euler_to_quaternion, quaternion_to_euler

logger = setup_logger(__file__)


class SO101Arm:
    def __init__(self, arm_name: str = "arm") -> None:
        pass

    def enable(self) -> None:
        pass

    def gotoZero(self) -> None:
        pass

    def gotoObserve(self) -> None:
        pass

    def softStop(self) -> None:
        pass

    def cmd_ee_pose_values(self, x, y, z, r, p, y_, line_mode: bool = False) -> None:
        """Command end-effector to target pose in space (position + Euler angles)"""
        pass

    def cmd_ee_pose(self, pose: Pose, line_mode: bool = False) -> None:
        """Command end-effector to target pose using Pose message"""
        pass

    def get_ee_pose(self):
        """Return the current end-effector pose as Pose message with position in meters and quaternion orientation"""
        pass

    def cmd_gripper_ctrl(self, position, effort: float = 0.25) -> None:
        """Command end-effector gripper"""
        pass

    def enable_gripper(self) -> None:
        """Enable the gripper using the initialization sequence"""
        pass

    def release_gripper(self) -> None:
        """Release gripper by opening to 100mm (10cm)"""
        pass

    def get_gripper_feedback(self) -> tuple[float, float]:
        """
        Get current gripper feedback.

        Returns:
            Tuple of (angle_degrees, effort) where:
                - angle_degrees: Current gripper angle in degrees
                - effort: Current gripper effort (0.0 to 1.0 range)
        """
        pass

    def close_gripper(self, commanded_effort: float = 0.5) -> None:
        """
        Close the gripper.

        Args:
            commanded_effort: Effort to use when closing gripper (default 0.25 N/m)
        """
        pass

    def gripper_object_detected(self, commanded_effort: float = 0.25) -> bool:
        """
        Check if an object is detected in the gripper based on effort feedback.

        Args:
            commanded_effort: The effort that was used when closing gripper (default 0.25 N/m)

        Returns:
            True if object is detected in gripper, False otherwise
        """
        pass

    def resetArm(self) -> None:
        pass

    def init_vel_controller(self) -> None:
        pass

    def cmd_vel(self, x_dot, y_dot, z_dot, R_dot, P_dot, Y_dot) -> None:
        pass

    def cmd_vel_ee(self, x_dot, y_dot, z_dot, RX_dot, PY_dot, YZ_dot) -> None:
        pass

    def disable(self) -> None:
        pass


class VelocityController(Module):
    cmd_vel: In[Twist] = None

    def __init__(self, arm, period: float = 0.01, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.arm = arm
        self.period = period
        self.latest_cmd = None
        self.last_cmd_time = None
        self._thread = None

    @rpc
    def start(self) -> None:
        pass

    @rpc
    def stop(self) -> None:
        pass

    def handle_cmd_vel(self, cmd_vel: Twist) -> None:
        pass


@pytest.mark.tool
def run_velocity_controller() -> None:
    pass


if __name__ == "__main__":
    arm = SO101Arm()

    def get_key(timeout: float = 0.1):
        """Non-blocking key reader for arrow keys."""
        pass

    def teleop_linear_vel(arm) -> None:
        pass

    run_velocity_controller()
