#!/usr/bin/env python3
"""Load a full GLB scene with physics colliders."""

from dimos.robot.sim.scene_client import SceneClient

COLLISION_WORLD = "/proxy?url=https://threejs.org/examples/models/gltf/collision-world.glb"

with SceneClient() as scene:
    result = scene.load_map(
        url=COLLISION_WORLD,
        name="environment",
        collider="trimesh",
    )
    print(f"Scene loaded: {result['name']} (scale: {result.get('scaleFactor', 1.0)})")
