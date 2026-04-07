"""Point cloud filtering utilities for smart_nav modules."""

from __future__ import annotations

import numpy as np

from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


def filter_points_near_position(
    point_cloud: PointCloud2,
    position: np.ndarray,
    radius: float,
) -> PointCloud2:
    """Remove all points within ``radius`` of ``position`` from the cloud.

    Args:
        point_cloud: The input point cloud.
        position: Robot position as a (3,) array [x, y, z].
        radius: Exclusion radius in metres. Points strictly within this
            distance of ``position`` are removed.

    Returns:
        A new PointCloud2 with the nearby points removed.
    """
    if radius <= 0.0:
        raise ValueError(f"radius must be positive, got {radius}")

    points, _ = point_cloud.as_numpy()
    if len(points) == 0:
        return PointCloud2.from_numpy(
            points=np.zeros((0, 3), dtype=np.float32),
            frame_id=point_cloud.frame_id,
            timestamp=point_cloud.ts,
        )

    position = np.asarray(position, dtype=np.float64).ravel()[:3]
    distances_squared = np.sum((points - position) ** 2, axis=1)
    keep_mask = distances_squared >= radius * radius

    filtered_points = points[keep_mask]
    return PointCloud2.from_numpy(
        points=filtered_points,
        frame_id=point_cloud.frame_id,
        timestamp=point_cloud.ts,
    )
