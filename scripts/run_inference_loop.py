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

from __future__ import annotations

import argparse
import threading
import time
from typing import Any

import numpy as np
from openpi_client import websocket_client_policy

from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs import Image, JointCommand


class LatestValue:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._value: Any | None = None
        self._ts: float | None = None

    def update(self, value: Any) -> None:
        with self._lock:
            self._value = value
            self._ts = time.time()
            self._event.set()

    def wait(self, timeout: float | None = None) -> bool:
        return self._event.wait(timeout=timeout)

    def get(self) -> tuple[Any | None, float | None]:
        with self._lock:
            return self._value, self._ts


def _as_actions_array(actions: Any) -> np.ndarray:
    actions_array = np.asarray(actions)
    if actions_array.ndim == 1:
        actions_array = actions_array[None, :]
    return actions_array


def _build_observation(
    *,
    image: Image,
    joint_position: list[float],
    gripper_position: float = 0.0,
    prompt: str,
    image_size: int | None,
) -> dict[str, Any]:
    if image_size is not None:
        image = image.resize(image_size, image_size)
    rgb = image.to_rgb().to_opencv()
    return {
        "observation/exterior_image_1_left": rgb,
        "observation/wrist_image_left": rgb,
        "observation/joint_position": joint_position,
        "observation/gripper_position": gripper_position,
        "prompt": prompt,
    }


def _wait_for_target(
    arm: Any,
    target: list[float],
    *,
    tolerance: float,
    stable_time: float,
    timeout: float,
    poll_dt: float,
) -> bool:
    start = time.time()
    stable_since: float | None = None
    while time.time() - start < timeout:
        code, angles = arm.get_servo_angle()
        if code != 0:
            raise RuntimeError(f"xArm get_servo_angle failed with code {code}")
        max_err = max(abs(angles[i] - target[i]) for i in range(len(target)))
        if max_err <= tolerance:
            if stable_since is None:
                stable_since = time.time()
            if time.time() - stable_since >= stable_time:
                return True
        else:
            stable_since = None
        time.sleep(poll_dt)
    return False


def _wait_for_image(
    image_buffer: LatestValue,
    last_ts: float,
    *,
    timeout: float,
    poll_dt: float,
) -> tuple[Image | None, float]:
    start = time.time()
    while time.time() - start < timeout:
        img, ts = image_buffer.get()
        if isinstance(img, Image) and ts is not None and ts > last_ts:
            return img, ts
        time.sleep(poll_dt)
    img, _ = image_buffer.get()
    return img if isinstance(img, Image) else None, last_ts


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Run a VLA policy loop using LCM camera images and xArm SDK state."
    )
    parser.add_argument("--policy-host", default="localhost")
    parser.add_argument("--policy-port", type=int, default=8000)
    parser.add_argument("--camera-topic", default="/camera/color")
    parser.add_argument("--prompt", default="move the arm slightly to the left")
    parser.add_argument("--image-size", type=int, default=224)
    parser.add_argument("--action-topic", default="/xarm/joint_position_command")
    parser.add_argument("--action-rate-hz", type=float, default=10.0)
    parser.add_argument("--chunk-timeout", type=float, default=10.0)
    parser.add_argument("--joint-tolerance", type=float, default=0.03)
    parser.add_argument("--stable-time", type=float, default=0.2)
    parser.add_argument("--num-joints", type=int, default=None)
    parser.add_argument("--obs-timeout", type=float, default=5.0)
    parser.add_argument("--xarm-ip", default="192.168.2.235")
    parser.add_argument("--xarm-dof", type=int, default=6)
    args = parser.parse_args()

    if args.action_rate_hz <= 0:
        raise ValueError("action-rate-hz must be > 0")

    if args.xarm_ip is None:
        raise ValueError("Provide --xarm-ip to read joint and gripper state from the xArm SDK.")

    try:
        from xarm.wrapper import XArmAPI
    except ImportError as exc:
        raise ImportError(
            "xArm SDK not installed. Install xArm-Python-SDK to read joint/gripper state."
        ) from exc

    arm = XArmAPI(args.xarm_ip, is_radian=False)

    print("Connecting to policy server...")
    policy = websocket_client_policy.WebsocketClientPolicy(
        host=args.policy_host,
        port=args.policy_port,
    )
    print("Server metadata:", policy.get_server_metadata())

    image_buffer = LatestValue()

    image_sub = LCMTransport(args.camera_topic, Image)
    image_sub.subscribe(lambda msg: image_buffer.update(msg))

    action_pub = LCMTransport(args.action_topic, JointCommand)

    print(f"Listening for camera images on {args.camera_topic}")
    if not image_buffer.wait(timeout=args.obs_timeout):
        raise TimeoutError(f"No images received on {args.camera_topic}")

    inferred_num_joints = args.num_joints or args.xarm_dof

    if inferred_num_joints <= 0:
        raise RuntimeError("Unable to determine num_joints; provide --num-joints")

    action_dt = 1.0 / args.action_rate_hz

    print("Starting inference loop...")
    last_image_ts = 0.0
    try:
        while True:
            image, image_ts = _wait_for_image(
                image_buffer,
                last_image_ts,
                timeout=args.obs_timeout,
                poll_dt=min(0.05, action_dt),
            )
            if image_ts is not None:
                last_image_ts = image_ts
            if image is None:
                print("Skipping inference: no image available.")
                time.sleep(0.1)
                continue

            code, angles = arm.get_servo_angle()
            print(f"xArm get_servo_angle: {code}, {angles}")
            if code != 0:
                raise RuntimeError(f"xArm get_servo_angle failed with code {code}")
            joint_position = list(angles[:inferred_num_joints])

            # code, gripper_position = arm.get_gripper_position()
            # if code != 0:
            #     raise RuntimeError(f"xArm get_gripper_position failed with code {code}")

            # obs = _build_observation(
            #     image=image,
            #     joint_position=joint_position,
            #     gripper_position=0.0,
            #     prompt=args.prompt,
            #     image_size=args.image_size,
            # )
            obs = {
                "observation/exterior_image_1_left": np.random.randint(
                    256, size=(224, 224, 3), dtype=np.uint8
                ),
                "observation/wrist_image_left": np.random.randint(
                    256, size=(224, 224, 3), dtype=np.uint8
                ),
                "observation/joint_position": joint_position,  # Your xArm joint positions
                "observation/gripper_position": 0.0,  # Your gripper position
                "prompt": "move the arm slightly to the left",  # Your task description
            }
            # print(f"Observation: {obs}")
            result = policy.infer(obs)
            actions = _as_actions_array(result["actions"])
            if actions.shape[1] < inferred_num_joints:
                raise ValueError(
                    f"Policy returned {actions.shape[1]} dims, need {inferred_num_joints} joints."
                )

            for step in actions:
                target = step[:inferred_num_joints].tolist()
                action_pub.broadcast(None, JointCommand(positions=target))
                time.sleep(action_dt)

            final_target = actions[-1][:inferred_num_joints].tolist()
            completed = _wait_for_target(
                arm,
                final_target,
                tolerance=args.joint_tolerance,
                stable_time=args.stable_time,
                timeout=args.chunk_timeout,
                poll_dt=min(0.02, action_dt),
            )
            if not completed:
                print("Warning: action chunk timeout waiting for final target.")
    except KeyboardInterrupt:
        print("Stopping inference loop.")
    finally:
        try:
            arm.disconnect()
        except Exception:
            pass


if __name__ == "__main__":
    main()
