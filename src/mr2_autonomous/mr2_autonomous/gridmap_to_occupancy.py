#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from grid_map_msgs.msg import GridMap
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose

def quat_to_rot(x, y, z, w):
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1 - 2*(yy+zz), 2*(xy - wz),   2*(xz + wy)],
        [2*(xy + wz),   1 - 2*(xx+zz), 2*(yz - wx)],
        [2*(xz - wy),   2*(yz + wx),   1 - 2*(xx+yy)],
    ], dtype=np.float64)

class GridMapToOcc(Node):
    def __init__(self):
        super().__init__("gridmap_to_occupancy")
        # Params
        self.declare_parameter("input_topic", "/traversability_gridmap")
        self.declare_parameter("output_topic", "/traversability_occupancy")
        self.declare_parameter("layer", "traversability")
        self.declare_parameter("min_value", 0.0)  # grid value -> occ scaling range
        self.declare_parameter("max_value", 1.0)  # e.g., traversability in [0..1]
        self.declare_parameter("invert", True)    # if True: occ = 100 - scaled
        self.declare_parameter("unknown_value", -1) # OccupancyGrid unknown

        self.input_topic  = self.get_parameter("input_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.layer        = self.get_parameter("layer").value
        self.min_v        = float(self.get_parameter("min_value").value)
        self.max_v        = float(self.get_parameter("max_value").value)
        self.invert       = bool(self.get_parameter("invert").value)
        self.occ_unknown  = int(self.get_parameter("unknown_value").value)

        self.sub = self.create_subscription(GridMap, self.input_topic, self.cb, 1)
        self.pub = self.create_publisher(OccupancyGrid, self.output_topic, 1)
        self.get_logger().info(
            f"GridMap({self.layer}) → OccupancyGrid on {self.output_topic} | "
            f"range=[{self.min_v},{self.max_v}], invert={self.invert}"
        )

    def cb(self, gm):
        # 1) locate the requested layer
        try:
            idx = gm.layers.index(self.layer)
        except ValueError:
            self.get_logger().warn(f"Layer '{self.layer}' not found in {gm.layers}")
            return
        arr = gm.data[idx]

        # 2) get grid size (rows, cols) from the layout; fall back to info if needed
        if len(arr.layout.dim) >= 2:
            dim0, dim1 = arr.layout.dim[0], arr.layout.dim[1]
            rows, cols = int(dim0.size), int(dim1.size)
        else:
            # fallback from info (length/resolution)
            rows = int(round(gm.info.length_y / gm.info.resolution))
            cols = int(round(gm.info.length_x / gm.info.resolution))

        # 3) restore 2D numpy array
        data_f32 = np.array(arr.data, dtype=np.float32, copy=False)
        if data_f32.size != rows * cols:
            self.get_logger().warn(
                f"Data size mismatch: got {data_f32.size}, expected {rows*cols}; "
                "attempting best-effort reshape."
            )
            # best effort: infer cols from size
            cols = data_f32.size // max(1, rows)
        grid = data_f32.reshape((rows, cols), order="C")  # same order as publisher
        # Undo the producer's RViz-oriented hard flip (both axes)
        grid = grid[::-1, ::-1]

        # 4) scale to 0..100, handle NaNs, optional invert
        vmin, vmax = (self.min_v, self.max_v)
        if np.isclose(vmax, vmin):
            scale = 1.0
        else:
            scale = 100.0 / (vmax - vmin)

        occ = (grid - vmin) * scale
        # clip; NaNs become unknown later
        occ = np.clip(occ, 0.0, 100.0, out=occ)
        if self.invert:
            occ = 100.0 - occ

        # unknown where original is NaN
        unknown_mask = ~np.isfinite(grid)
        occ_i8 = occ.astype(np.int16)  # work in wider type
        occ_i8[unknown_mask] = self.occ_unknown  # -1 by default
        # final cast: OccupancyGrid uses int8 [-1, 0..100]
        occ_i8 = np.where(occ_i8 < -1, -1, occ_i8)
        occ_i8 = occ_i8.astype(np.int8)

        # 5) fill OccupancyGrid.info
        info = gm.info
        res = float(info.resolution)
        width, height = cols, rows

        # GridMap pose is CENTER of the grid in map frame, with orientation.
        # OccupancyGrid origin is CELL (0,0) corner (bottom-left of the grid),
        # with the SAME orientation. Compute origin = center - R*[Lx/2, Ly/2, 0].
        cx, cy = float(info.pose.position.x), float(info.pose.position.y)
        q = info.pose.orientation
        R = quat_to_rot(q.x, q.y, q.z, q.w)
        half = R @ np.array([info.length_x/2.0, info.length_y/2.0, 0.0], dtype=np.float64)
        origin_x = cx - half[0]
        origin_y = cy - half[1]

        occ_msg = OccupancyGrid()
        occ_msg.header = gm.header
        occ_msg.info.resolution = res
        occ_msg.info.width = width
        occ_msg.info.height = height
        occ_msg.info.origin = Pose()
        occ_msg.info.origin.position.x = origin_x
        occ_msg.info.origin.position.y = origin_y
        occ_msg.info.origin.position.z = float(info.pose.position.z)
        occ_msg.info.origin.orientation = info.pose.orientation  # preserve grid rotation

        # 6) row-major flatten; matches OccupancyGrid expectation
        occ_msg.data = occ_i8.flatten(order="C").tolist()

        self.pub.publish(occ_msg)

def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GridMapToOcc())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
