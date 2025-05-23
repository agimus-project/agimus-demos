from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
import numpy as np


def read_points_xyz_rgb_separate_fields(point_cloud_msg):
    data = point_cloud2.read_points(point_cloud_msg, field_names=("x", "y", "z", "rgb"))
    # Reinterpret the rgb single field into separate fields.
    dtype = np.dtype(
        dict(
            names=["x", "y", "z", "r", "g", "b"],
            formats=[
                "<f4",
                "<f4",
                "<f4",
                "<B",
                "<B",
                "<B",
            ],
            offsets=[0, 4, 8, 16, 17, 18],
            itemsize=data.dtype.itemsize,
        )
    )
    return data.view(dtype)


def read_points_xyz_rgb(msg: PointCloud2, frame_prefix: str = ""):
    xyz_rgb = read_points_xyz_rgb_separate_fields(msg)
    xyz = np.lib.recfunctions.drop_fields(xyz_rgb, ["r", "g", "b"])
    rgb = np.lib.recfunctions.drop_fields(xyz_rgb, ["x", "y", "z"])

    rgb_f = np.lib.recfunctions.structured_to_unstructured(rgb) / 255
    rgba_f = np.concatenate((rgb_f, np.ones((rgb_f.shape[0], 1))), axis=1)

    return (
        np.lib.recfunctions.structured_to_unstructured(xyz),
        rgba_f,
        frame_prefix + msg.header.frame_id,
    )
