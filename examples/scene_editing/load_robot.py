#!/usr/bin/env python3
"""Swap the robot embodiment."""

from dimos.robot.sim.scene_client import SceneClient

with SceneClient() as scene:
    # Use a preset
    cfg = scene.set_embodiment("drone")
    print(f"Switched to drone: {cfg}")

    # Or override params
    cfg = scene.set_embodiment("unitree-go2", walk_speed=4.0)
    print(f"Go2 with speed 4: {cfg}")
