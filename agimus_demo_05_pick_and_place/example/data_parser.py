# coding: utf-8
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import JointState, PointCloud2
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
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


def read_points_xyz_rgb(data, frame_prefix: str = ""):
    msg: PointCloud2 = data["pointcloud"]
    xyz_rgb = read_points_xyz_rgb_separate_fields(msg)
    xyz = np.lib.recfunctions.drop_fields(xyz_rgb, ["r", "g", "b"])
    rgb = np.lib.recfunctions.drop_fields(xyz_rgb, ["x", "y", "z"])

    rgb_f = np.lib.recfunctions.structured_to_unstructured(rgb) / 255
    rgba_f = np.concatenate((rgb_f, np.ones((rgb_f.shape[0], 1))), axis=1)

    return (
        np.lib.recfunctions.structured_to_unstructured(xyz),
        rgba_f,
        frame_prefix + msg.header.frame_id
    )


def get_robot_config(data, hpp_robot, robot_name: str):
    joint_msg: JointState = data["joints_data"]
    q = []
    for name in hpp_robot.jointNames:
        name: str
        if not name.startswith(robot_name):
            continue
        try:
            i = joint_msg.name.index(name.split("/", 1)[1])
        except ValueError:
            continue
        q.append(joint_msg.position[i])
    return q


def get_object_pose(data, obj_class_id, frame_prefix: str = ""):
    detection_array: Detection2DArray = data["happypose_data"]
    for detection in detection_array.detections:
        detection: Detection2D
        result: ObjectHypothesisWithPose = detection.results[0]
        if result.hypothesis.class_id == obj_class_id:
            frame = frame_prefix + detection.header.frame_id
            T = result.pose.pose.position
            Q = result.pose.pose.orientation
            pose = [T.x, T.y, T.z, Q.x, Q.y, Q.z, Q.w]
            return pose, frame


if __name__ == "__main__":
    import pickle
    import matplotlib.pyplot as plt

    with open("datas/recorded_scene01b.pkl", "rb") as f:
        data = pickle.load(f)

    pc = data["pointcloud"]
    pc_data2 = read_points_xyz_rgb_separate_fields(pc)

    fig = plt.figure()
    ax = fig.add_subplot(projection="3d")
    ax.scatter(
        pc_data2["x"],
        pc_data2["y"],
        pc_data2["z"],
        c=np.array(pc_data2[["r", "g", "b"]].tolist()) / 255,
    )
    plt.show()
