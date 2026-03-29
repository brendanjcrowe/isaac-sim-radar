"""Build a 2D radar occupancy grid from accumulated radar detections.

Can be used as a library (accumulate() per frame, then save_map()) or
run standalone to process a rosbag2 file.

Radar detections are rotated from the sensor frame into the odom/map
frame using synchronised odometry before being binned into a 2D grid.

Output files produced by save_map():
  {out_dir}/radar_map.png   — colour heat map (matplotlib)
  {out_dir}/radar_map.pgm   — ROS-compatible greyscale occupancy image
  {out_dir}/radar_map.yaml  — ROS map metadata (resolution, origin)
"""

import argparse
import math
import os
import struct
from typing import List, Optional, Tuple

import numpy as np


class RadarOccupancyGrid:
    """2D accumulation grid for radar detections.

    Coordinates are in the odom/map frame (metres).  Each cell
    increments by 1 when a detection falls within it.
    """

    def __init__(
        self,
        resolution: float = 0.1,   # m/cell
        x_min: float = -10.0,
        x_max: float = 100.0,
        y_min: float = -30.0,
        y_max: float = 30.0,
        rcs_threshold: float = -20.0,  # dBsm — ignore weak ghost returns
    ):
        self.resolution = resolution
        self.x_min = x_min
        self.x_max = x_max
        self.y_min = y_min
        self.y_max = y_max
        self.rcs_threshold = rcs_threshold

        nx = int(math.ceil((x_max - x_min) / resolution))
        ny = int(math.ceil((y_max - y_min) / resolution))
        self._grid = np.zeros((ny, nx), dtype=np.int32)

    @property
    def grid(self) -> np.ndarray:
        return self._grid

    def _to_cell(self, x: float, y: float) -> Optional[Tuple[int, int]]:
        col = int((x - self.x_min) / self.resolution)
        row = int((y - self.y_min) / self.resolution)
        ny, nx = self._grid.shape
        if 0 <= col < nx and 0 <= row < ny:
            return row, col
        return None

    def accumulate(
        self,
        xs: np.ndarray,
        ys: np.ndarray,
        rcs: Optional[np.ndarray] = None,
        robot_x: float = 0.0,
        robot_y: float = 0.0,
        robot_yaw: float = 0.0,
    ) -> int:
        """Add one frame of radar detections (sensor frame) to the grid.

        Transforms each detection from the sensor frame into the odom
        frame using (robot_x, robot_y, robot_yaw) from odometry.

        Returns the number of detections actually accumulated.
        """
        cos_y = math.cos(robot_yaw)
        sin_y = math.sin(robot_yaw)
        count = 0
        for i in range(len(xs)):
            if rcs is not None and rcs[i] < self.rcs_threshold:
                continue
            x_map = robot_x + cos_y * float(xs[i]) - sin_y * float(ys[i])
            y_map = robot_y + sin_y * float(xs[i]) + cos_y * float(ys[i])
            cell = self._to_cell(x_map, y_map)
            if cell is not None:
                self._grid[cell] += 1
                count += 1
        return count

    def save_map(self, out_dir: str, prefix: str = "radar_map") -> Tuple[str, str, str]:
        """Write PNG heat map + ROS-format PGM + YAML to out_dir."""
        os.makedirs(out_dir, exist_ok=True)
        png_path  = os.path.join(out_dir, f"{prefix}.png")
        pgm_path  = os.path.join(out_dir, f"{prefix}.pgm")
        yaml_path = os.path.join(out_dir, f"{prefix}.yaml")

        g = self._grid.astype(float)
        g_max = g.max() if g.max() > 0 else 1.0

        # ── PNG heat map ──────────────────────────────────────────────────────
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(figsize=(12, 8))
        im = ax.imshow(
            g / g_max,
            origin="lower",
            extent=[self.x_min, self.x_max, self.y_min, self.y_max],
            cmap="hot",
            vmin=0,
            vmax=1,
        )
        plt.colorbar(im, ax=ax, label="Normalised detection count")
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("Radar Occupancy Map")
        ax.set_aspect("equal")
        fig.tight_layout()
        fig.savefig(png_path, dpi=150)
        plt.close(fig)

        # ── ROS PGM (greyscale occupancy) ─────────────────────────────────────
        # Convention: 0 = occupied (black), 254 = free (white), 205 = unknown
        pgm = np.full_like(self._grid, 205, dtype=np.uint8)  # unknown
        nonzero = self._grid > 0
        if nonzero.any():
            hit_norm = np.clip(self._grid[nonzero] / g_max, 0.0, 1.0)
            pgm[nonzero] = (254 * (1.0 - hit_norm)).astype(np.uint8)

        ny, nx = pgm.shape
        with open(pgm_path, "wb") as f:
            f.write(f"P5\n{nx} {ny}\n255\n".encode())
            f.write(pgm.tobytes())

        # ── ROS map YAML ──────────────────────────────────────────────────────
        with open(yaml_path, "w") as f:
            f.write(
                f"image: {os.path.basename(pgm_path)}\n"
                f"resolution: {self.resolution}\n"
                f"origin: [{self.x_min:.3f}, {self.y_min:.3f}, 0.0]\n"
                f"negate: 0\n"
                f"occupied_thresh: 0.65\n"
                f"free_thresh: 0.196\n"
            )

        return png_path, pgm_path, yaml_path


# ── Bag-level helper ──────────────────────────────────────────────────────────

def _unpack_field(data: bytes, n: int, point_step: int, fields_map: dict, name: str) -> Optional[np.ndarray]:
    """Extract a float32 field from a PointCloud2 data buffer."""
    if name not in fields_map:
        return None
    off = fields_map[name].offset
    return np.array(
        [struct.unpack_from("f", data, i * point_step + off)[0] for i in range(n)],
        dtype=np.float32,
    )


def build_from_bag(
    bag_path: str,
    out_dir: str,
    radar_topic: str = "/radar/point_cloud",
    odom_topic:  str = "/odom",
    **grid_kwargs,
) -> RadarOccupancyGrid:
    """Read a rosbag2, accumulate radar detections, and save the map.

    Requires: rosbags>=0.9
    """
    from rosbags.rosbag2 import Reader
    from rosbags.typesys import get_typestore, Stores
    typestore = get_typestore(Stores.ROS2_HUMBLE)
    deserialize_cdr = typestore.deserialize_cdr

    grid = RadarOccupancyGrid(**grid_kwargs)

    # ── Pass 1: buffer odom ───────────────────────────────────────────────────
    odom_rows: List[Tuple[int, float, float, float]] = []
    with Reader(bag_path) as reader:
        odom_conns = [c for c in reader.connections if c.topic == odom_topic]
        for conn, stamp, rawdata in reader.messages(connections=odom_conns):
            msg = deserialize_cdr(rawdata, conn.msgtype)
            pos = msg.pose.pose.position
            ori = msg.pose.pose.orientation
            siny = 2.0 * (ori.w * ori.z + ori.x * ori.y)
            cosy = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
            yaw  = math.atan2(siny, cosy)
            odom_rows.append((stamp, pos.x, pos.y, yaw))

    odom_arr = np.array(odom_rows, dtype=np.float64) if odom_rows else np.zeros((0, 4))

    # ── Pass 2: accumulate radar detections ───────────────────────────────────
    with Reader(bag_path) as reader:
        radar_conns = [c for c in reader.connections if c.topic == radar_topic]
        for conn, stamp, rawdata in reader.messages(connections=radar_conns):
            msg = deserialize_cdr(rawdata, conn.msgtype)
            n   = msg.width * msg.height
            fmap = {f.name: f for f in msg.fields}
            data = bytes(msg.data)
            ps   = msg.point_step

            xs  = _unpack_field(data, n, ps, fmap, "x")
            ys  = _unpack_field(data, n, ps, fmap, "y")
            rcs = _unpack_field(data, n, ps, fmap, "rcs")

            if xs is None:
                xs = np.zeros(n, dtype=np.float32)
            if ys is None:
                ys = np.zeros(n, dtype=np.float32)

            # Nearest odom by timestamp
            rx, ry, ryaw = 0.0, 0.0, 0.0
            if len(odom_arr):
                idx = int(np.argmin(np.abs(odom_arr[:, 0] - stamp)))
                _, rx, ry, ryaw = odom_arr[idx]

            grid.accumulate(xs, ys, rcs=rcs, robot_x=rx, robot_y=ry, robot_yaw=ryaw)

    grid.save_map(out_dir)
    return grid


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description="Build radar occupancy map from a rosbag2.")
    ap.add_argument("bag",            help="Path to rosbag2 directory")
    ap.add_argument("--out",          default="analysis_output", help="Output directory")
    ap.add_argument("--radar-topic",  default="/radar/point_cloud")
    ap.add_argument("--odom-topic",   default="/odom")
    ap.add_argument("--resolution",   type=float, default=0.1)
    ap.add_argument("--x-min",        type=float, default=-10.0)
    ap.add_argument("--x-max",        type=float, default=100.0)
    ap.add_argument("--y-min",        type=float, default=-30.0)
    ap.add_argument("--y-max",        type=float, default=30.0)
    ap.add_argument("--rcs-threshold",type=float, default=-20.0)
    args = ap.parse_args()

    grid = build_from_bag(
        args.bag, args.out,
        radar_topic=args.radar_topic,
        odom_topic=args.odom_topic,
        resolution=args.resolution,
        x_min=args.x_min, x_max=args.x_max,
        y_min=args.y_min, y_max=args.y_max,
        rcs_threshold=args.rcs_threshold,
    )
    total = int(grid.grid.sum())
    cells = int((grid.grid > 0).sum())
    print(f"Total accumulated detections : {total}")
    print(f"Occupied cells               : {cells}")
    print(f"Map written to               : {args.out}/")
