"""Convert captured radar data to Open3D point clouds for analysis."""

import numpy as np

try:
    import open3d as o3d
except ImportError:
    o3d = None
    print("Warning: open3d not installed. Install with: pip install open3d")


def detections_to_pointcloud(x, y, z, color_by=None, colormap="viridis"):
    """Convert radar detection arrays to an Open3D point cloud.

    Args:
        x, y, z: Detection position arrays (N,)
        color_by: Optional array (N,) to color points by (e.g., RCS, velocity)
        colormap: Matplotlib colormap name for coloring
    """
    if o3d is None:
        raise ImportError("open3d is required")

    points = np.column_stack([x, y, z])
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    if color_by is not None and len(color_by) > 0:
        import matplotlib.pyplot as plt
        cmap = plt.get_cmap(colormap)
        vmin, vmax = color_by.min(), color_by.max()
        if vmax > vmin:
            normalized = (color_by - vmin) / (vmax - vmin)
        else:
            normalized = np.zeros_like(color_by)
        colors = cmap(normalized)[:, :3]  # RGB only
        pcd.colors = o3d.utility.Vector3dVector(colors)

    return pcd


def visualize_radar(x, y, z, rcs=None):
    """Quick visualization of radar detections colored by RCS."""
    if o3d is None:
        raise ImportError("open3d is required")

    pcd = detections_to_pointcloud(x, y, z, color_by=rcs, colormap="hot")
    o3d.visualization.draw_geometries(
        [pcd],
        window_name="Radar Point Cloud",
        width=1024,
        height=768,
    )
