"""Compare radar and LiDAR point clouds.

Metrics:
- Point-to-point distance (nearest neighbor)
- Coverage ratio (radar points with LiDAR match within threshold)
- Detection density comparison
"""

import numpy as np

try:
    import open3d as o3d
except Exception:
    o3d = None


def compute_distances(source_points: np.ndarray, target_points: np.ndarray,
                      max_target_pts: int = 500_000) -> np.ndarray:
    """Compute nearest-neighbor distances from source to target.

    Args:
        source_points: (N, 3) array
        target_points: (M, 3) array
        max_target_pts: subsample target if larger (avoids OOM on 85M+ lidar points)

    Returns:
        (N,) array of distances to nearest target point
    """
    # Subsample target if too large for KDTree
    if len(target_points) > max_target_pts:
        idx = np.random.default_rng(42).choice(
            len(target_points), max_target_pts, replace=False
        )
        target_points = target_points[idx]

    # Try open3d first, then scipy, then pure-numpy chunked brute force
    if o3d is not None:
        target_pcd = o3d.geometry.PointCloud()
        target_pcd.points = o3d.utility.Vector3dVector(target_points)
        tree = o3d.geometry.KDTreeFlann(target_pcd)
        distances = np.zeros(len(source_points))
        for i, pt in enumerate(source_points):
            _, _, d2 = tree.search_knn_vector_3d(pt, 1)
            distances[i] = np.sqrt(d2[0])
        return distances

    try:
        from scipy.spatial import KDTree
        tree = KDTree(target_points)
        distances, _ = tree.query(source_points)
        return distances
    except (ImportError, ValueError):
        pass

    # Pure-numpy fallback: chunked brute-force
    distances = np.full(len(source_points), np.inf, dtype=np.float64)
    chunk = 5000
    for i in range(0, len(source_points), chunk):
        src = source_points[i:i + chunk]  # (C, 3)
        # Compute distances to all target points in chunks to limit memory
        for j in range(0, len(target_points), 50000):
            tgt = target_points[j:j + 50000]  # (T, 3)
            d = np.linalg.norm(src[:, None, :] - tgt[None, :, :], axis=2)  # (C, T)
            min_d = d.min(axis=1)
            distances[i:i + chunk] = np.minimum(distances[i:i + chunk], min_d)
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
