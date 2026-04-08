#!/usr/bin/env python3
"""Remove a named object from the scene."""

from dimos.robot.sim.scene_client import SceneClient

with SceneClient() as scene:
    removed = scene.remove_object("crate")
    print(f"Removed: {removed}")
