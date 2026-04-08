#!/usr/bin/env python3
"""Add an object to the running scene."""

from dimos.robot.sim.scene_client import SceneClient

with SceneClient() as scene:
    scene.add_object(
        "box",
        size=(1, 0.5, 1),
        color=0x8B4513,
        position=(3, 0.25, 2),
        name="crate",
    )
    print("Object added")
