"""Compare radar and LiDAR point clouds.

Metrics:
- Point-to-point distance (nearest neighbor)
- Coverage ratio (radar points with LiDAR match within threshold)
- Detection density comparison
"""

import numpy as np

try:
    import open3d as o3d
except ImportError:
    o3d = None


def compute_distances(source_points: np.ndarray, target_points: np.ndarray) -> np.ndarray:
    """Compute nearest-neighbor distances from source to target.

    Args:
        source_points: (N, 3) array
        target_points: (M, 3) array

    Returns:
        (N,) array of distances to nearest target point
    """
    if o3d is None:
        # Fallback: brute force (slow for large clouds)
        from scipy.spatial import KDTree
        tree = KDTree(target_points)
        distances, _ = tree.query(source_points)
        return distances

    target_pcd = o3d.geometry.PointCloud()
    target_pcd.points = o3d.utility.Vector3dVector(target_points)
    tree = o3d.geometry.KDTreeFlann(target_pcd)

    distances = np.zeros(len(source_points))
    for i, pt in enumerate(source_points):
        _, _, d2 = tree.search_knn_vector_3d(pt, 1)
        distances[i] = np.sqrt(d2[0])

    return distances


def coverage_ratio(radar_points: np.ndarray, lidar_points: np.ndarray,
                   threshold_m: float = 0.5) -> float:
    """Fraction of radar detections that have a LiDAR match within threshold."""
    if len(radar_points) == 0:
        return 0.0
    distances = compute_distances(radar_points, lidar_points)
    return float(np.mean(distances < threshold_m))


def compare_summary(radar_points: np.ndarray, lidar_points: np.ndarray,
                    threshold_m: float = 0.5) -> dict:
    """Compute a summary comparison between radar and LiDAR point clouds."""
    radar_to_lidar = compute_distances(radar_points, lidar_points)

    return {
        "radar_count": len(radar_points),
        "lidar_count": len(lidar_points),
        "mean_distance_m": float(np.mean(radar_to_lidar)),
        "median_distance_m": float(np.median(radar_to_lidar)),
        "max_distance_m": float(np.max(radar_to_lidar)),
        "coverage_ratio": float(np.mean(radar_to_lidar < threshold_m)),
        "threshold_m": threshold_m,
    }
