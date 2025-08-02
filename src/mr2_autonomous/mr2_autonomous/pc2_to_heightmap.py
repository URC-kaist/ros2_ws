#!/usr/bin/env python3
"""
RealSense PointCloud2 → grid_map_msgs/GridMap (Elevation Layer)
----------------------------------------------------------------
- Mean Z per cell (map frame)
- Vectorized transforms (fast)
- Timing logs for each stage
- **Axes are hard-flipped (both X & Y) to match RViz view** — no flip/invert params left
- ROI crop (camera Z) + voxel downsample for speed

Params
------
cloud_topic : PointCloud2 topic (default /camera/depth/color/points)
base_frame  : base frame (default base_link)
map_frame   : map frame (default map)
x_forward_m : forward length (m)
y_width_m   : lateral width (m)
resolution  : cell size (m)
layer_name  : grid map layer name (default elevation)
voxel_size_m: voxel size for downsampling (0 = off)
roi_z_max_m : drop points farther than this Z in camera frame (0 = off)

Requires: ros-${ROS_DISTRO}-grid-map-msgs and python3-open3d (Open3D).
"""

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

from grid_map_msgs.msg import GridMap, GridMapInfo
from std_msgs.msg import Float32MultiArray, MultiArrayDimension
from geometry_msgs.msg import Pose, TransformStamped

import tf2_ros
import open3d as o3d


class PC2ToGridMapNode(Node):
    def __init__(self):
        super().__init__("pc2_to_gridmap_node")

        # ── Parameters ──────────────────────────────────────────────────────
        self.declare_parameter("cloud_topic", "/rgbd_camera/points")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "base_link")
        self.declare_parameter("x_forward_m", 5.0)
        self.declare_parameter("y_width_m", 3.0)
        self.declare_parameter("resolution", 0.05)
        self.declare_parameter("layer_name", "elevation")
        # Speed params
        self.declare_parameter("voxel_size_m", 0.0)
        self.declare_parameter("roi_z_max_m", 0.0)

        p = self.get_parameter
        self.cloud_topic = p("cloud_topic").value
        self.base_frame = p("base_frame").value
        self.map_frame = p("map_frame").value
        self.x_forward = p("x_forward_m").value
        self.y_width = p("y_width_m").value
        self.resolution = p("resolution").value
        self.layer_name = p("layer_name").value
        self.voxel_size = p("voxel_size_m").value
        self.roi_z_max = p("roi_z_max_m").value

        # Grid geometry
        self.grid_cols = int(math.ceil(self.x_forward / self.resolution))
        self.grid_rows = int(math.ceil(self.y_width / self.resolution))
        self.y_min = -self.y_width / 2.0

        # TF
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Pub/Sub
        self.gridmap_pub = self.create_publisher(GridMap, "height_gridmap", 1)
        self.create_subscription(PointCloud2, self.cloud_topic, self.cloud_cb, 5)

        self.get_logger().info(
            "Publishing GridMap on 'height_gridmap' (layer='%s')." % self.layer_name
        )

    # ────────────────────────────────────────────────────────────────────────
    def cloud_cb(self, cloud_msg: PointCloud2):
        t0 = time.perf_counter()
        stamp = Time.from_msg(cloud_msg.header.stamp)

        # TF lookups
        try:
            tf_cam_map = self.tf_buffer.lookup_transform(
                self.map_frame, cloud_msg.header.frame_id, stamp, Duration(seconds=0.1)
            )
            tf_cam_base = self.tf_buffer.lookup_transform(
                self.base_frame, cloud_msg.header.frame_id, stamp, Duration(seconds=0.1)
            )
            tf_base_map = self.tf_buffer.lookup_transform(
                self.map_frame, self.base_frame, stamp, Duration(seconds=0.1)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed: {e}")
            return
        t_tf = time.perf_counter()

        T_cam_map = transform_to_matrix(tf_cam_map.transform)
        T_cam_base = transform_to_matrix(tf_cam_base.transform)
        T_base_map = transform_to_matrix(tf_base_map.transform)

        # Read cloud
        t_read0 = time.perf_counter()
        # ---- FAST zero-copy read using numpy.frombuffer ----
        fields = {f.name: f for f in cloud_msg.fields}
        dt = np.dtype(
            {
                "names": ["x", "y", "z"],
                "formats": ["<f4", "<f4", "<f4"],
                "offsets": [fields["x"].offset, fields["y"].offset, fields["z"].offset],
                "itemsize": cloud_msg.point_step,
            }
        )
        raw = np.frombuffer(
            cloud_msg.data, dtype=dt, count=cloud_msg.width * cloud_msg.height
        )
        pts = np.stack((raw["x"], raw["y"], raw["z"]), axis=1).astype(np.float32)
        pts = pts[::4]
        pts_total = pts.shape[0]
        if pts_total == 0:
            return

        # ROI crop (camera frame z)
        if self.roi_z_max > 0.0:
            pts = pts[pts[:, 2] < self.roi_z_max]
        t_roi = time.perf_counter()

        # Voxel downsample (Open3D)
        t_vox_start = time.perf_counter()
        if self.voxel_size > 0.0 and pts.shape[0] > 0:
            v = float(self.voxel_size)
            if v <= 1e-4:
                self.get_logger().warn(
                    f"voxel_size_m={v} too small, skipping voxel filter"
                )
            else:
                try:
                    pcl = o3d.geometry.PointCloud()
                    pcl.points = o3d.utility.Vector3dVector(pts.astype(np.float64))
                    pcl = pcl.voxel_down_sample(voxel_size=v)
                    pts = np.asarray(pcl.points, dtype=np.float32)
                except RuntimeError as e:
                    self.get_logger().warn(
                        f"Open3D voxel_down_sample failed: {e}; skipping voxel filter"
                    )
        t_vox = time.perf_counter()
        t_read = t_vox

        # Vectorized transform
        R_cam_map, t_cam_map = T_cam_map[:3, :3], T_cam_map[:3, 3]
        R_cam_base, t_cam_base = T_cam_base[:3, :3], T_cam_base[:3, 3]
        pts_map = pts @ R_cam_map.T + t_cam_map
        pts_base = pts @ R_cam_base.T + t_cam_base

        # Remove NaNs/Infs
        finite = np.isfinite(pts_map).all(axis=1) & np.isfinite(pts_base).all(axis=1)
        pts_map, pts_base = pts_map[finite], pts_base[finite]
        if pts_map.shape[0] == 0:
            return
        t_trans = time.perf_counter()

        # Front + bounds filter
        x = pts_base[:, 0]
        y = pts_base[:, 1]
        mask_front = x > 0.0
        if not np.any(mask_front):
            return
        x = x[mask_front]
        y = y[mask_front]
        z_map = pts_map[mask_front, 2]

        mask_inside = (
            (x >= 0.0)
            & (x < self.x_forward)
            & (y >= self.y_min)
            & (y < (self.y_min + self.y_width))
        )
        if not np.any(mask_inside):
            return
        x, y, z_map = x[mask_inside], y[mask_inside], z_map[mask_inside]
        pts_used = z_map.size

        ix = (x / self.resolution).astype(np.int32)
        iy = ((y - self.y_min) / self.resolution).astype(np.int32)

        # Bounds check to keep bincount size fixed
        mask_idx = (ix >= 0) & (ix < self.grid_cols) & (iy >= 0) & (iy < self.grid_rows)
        ix, iy, z_map = ix[mask_idx], iy[mask_idx], z_map[mask_idx]
        pts_used = z_map.size

        Ncells = self.grid_rows * self.grid_cols
        flat_idx = iy.astype(np.int64) * self.grid_cols + ix.astype(np.int64)

        sum_bins = np.bincount(flat_idx, weights=z_map, minlength=Ncells).astype(
            np.float32
        )
        cnt_bins = np.bincount(flat_idx, minlength=Ncells).astype(np.float32)
        if sum_bins.size > Ncells:
            sum_bins = sum_bins[:Ncells]
            cnt_bins = cnt_bins[:Ncells]

        height_flat = np.full(Ncells, np.nan, dtype=np.float32)
        valid = cnt_bins > 0
        height_flat[valid] = sum_bins[valid] / cnt_bins[valid]
        heightmap = height_flat.reshape(self.grid_rows, self.grid_cols)

        # Hard flip both axes
        heightmap = heightmap[::-1, ::-1]

        cells_filled = int(np.count_nonzero(valid))
        t_bin = time.perf_counter()

        self.publish_gridmap(heightmap, cloud_msg.header.stamp, tf_base_map, T_base_map)
        t_pub = time.perf_counter()

        # Timings
        roi_ms = (t_roi - t_read0) * 1000.0
        vox_ms = (t_vox - t_roi) * 1000.0
        tf_ms = (t_tf - t0) * 1000.0
        read_ms = (t_read - t_tf) * 1000.0
        trans_ms = (t_trans - t_read) * 1000.0
        bin_ms = (t_bin - t_trans) * 1000.0
        pub_ms = (t_pub - t_bin) * 1000.0
        tot_ms = (t_pub - t0) * 1000.0
        self.get_logger().debug(
            f"pts:{pts_total} -> used:{pts_used} cells:{cells_filled} | "
            f"tf:{tf_ms:.1f} read:{read_ms:.1f} roi:{roi_ms:.1f} vox:{vox_ms:.1f} trans:{trans_ms:.1f} bin:{bin_ms:.1f} pub:{pub_ms:.1f} total:{tot_ms:.1f} ms"
        )

    # ────────────────────────────────────────────────────────────────────────
    def publish_gridmap(
        self,
        heightmap: np.ndarray,
        stamp,
        tf_base_map: TransformStamped,
        T_base_map: np.ndarray,
    ):
        rows, cols = heightmap.shape

        info = GridMapInfo()
        info.resolution = self.resolution
        info.length_x = self.x_forward
        info.length_y = self.y_width

        # center of grid in base -> map
        center_base = np.array(
            [self.x_forward / 2.0, self.y_width / 2.0 + self.y_min, 0.0, 1.0],
            dtype=np.float32,
        )
        center_xyz = (T_base_map @ center_base)[:3]

        pose = Pose()
        pose.position.x = float(center_xyz[0])
        pose.position.y = float(center_xyz[1])
        pose.position.z = float(center_xyz[2])
        pose.orientation = tf_base_map.transform.rotation
        info.pose = pose

        arr = Float32MultiArray()
        arr.layout.dim = [
            MultiArrayDimension(label="column_index", size=rows, stride=rows * cols),
            MultiArrayDimension(label="row_index", size=cols, stride=cols),
        ]
        arr.data = heightmap.astype(np.float32).flatten(order="C").tolist()

        msg = GridMap()
        msg.header.stamp = stamp
        msg.header.frame_id = self.map_frame
        msg.info = info
        msg.layers = [self.layer_name]
        msg.basic_layers = [self.layer_name]
        msg.data = [arr]
        msg.outer_start_index = 0
        msg.inner_start_index = 0

        self.gridmap_pub.publish(msg)


# ── Helpers ────────────────────────────────────────────────────────────────


def transform_to_matrix(t):
    x = t.translation.x
    y = t.translation.y
    z = t.translation.z
    qx = t.rotation.x
    qy = t.rotation.y
    qz = t.rotation.z
    qw = t.rotation.w
    R = quat_to_rot(qx, qy, qz, qw)
    T = np.eye(4, dtype=np.float32)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    return T


def quat_to_rot(x, y, z, w):
    xx = x * x
    yy = y * y
    zz = z * z
    xy = x * y
    xz = x * z
    yz = y * z
    wx = w * x
    wy = w * y
    wz = w * z
    return np.array(
        [
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)],
        ],
        dtype=np.float32,
    )


def main(args=None):
    rclpy.init(args=args)
    node = PC2ToGridMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
