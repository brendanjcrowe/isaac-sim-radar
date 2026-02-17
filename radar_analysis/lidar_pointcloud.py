"""Load and process LiDAR point clouds from ROS2 bags or PCD files."""

import numpy as np

try:
    import open3d as o3d
except ImportError:
    o3d = None


def load_pcd(path: str):
    """Load a point cloud from a PCD or PLY file."""
    if o3d is None:
        raise ImportError("open3d is required")
    return o3d.io.read_point_cloud(path)


def pointcloud2_to_numpy(msg) -> np.ndarray:
    """Convert a ROS2 PointCloud2 message to a numpy (N, 3) array.

    Assumes the message has x, y, z as the first three float32 fields.
    """
    data = np.frombuffer(msg.data, dtype=np.uint8)
    points = []
    for i in range(msg.width * msg.height):
        offset = i * msg.point_step
        x, y, z = np.frombuffer(
            data[offset : offset + 12], dtype=np.float32
        )
        points.append([x, y, z])
    return np.array(points, dtype=np.float32)


def downsample(pcd, voxel_size: float = 0.1):
    """Voxel-downsample a point cloud."""
    if o3d is None:
        raise ImportError("open3d is required")
    return pcd.voxel_down_sample(voxel_size)
