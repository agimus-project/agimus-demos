import numpy as np
import xml.etree.ElementTree as ET
import typing as T
import pinocchio as pin
from geometry_msgs.msg import Pose
import numpy.typing as npt
import os

XYZQuatType: T.TypeAlias = tuple[float, float, float, float, float, float, float]


def hack_for_ros2_support_in_hpp():
    """Hack to support ROS2 in HPP."""
    if "ROS_PACKAGE_PATH" not in os.environ and "AMENT_PREFIX_PATH" in os.environ:
        os.environ["ROS_PACKAGE_PATH"] = ":".join(
            v + "/share" for v in os.environ["AMENT_PREFIX_PATH"].split(":")
        )


def concatenatePaths(paths, c_robot=None):
    if len(paths) == 0:
        return None
    p = paths[0].asVector()
    for q in paths[1:]:
        if c_robot is None:
            np.testing.assert_allclose(p.end(), q.initial())
        else:
            diff = c_robot.difference(p.end(), q.initial())
            assert all(np.abs(diff) < 1e-8)
        p.appendPath(q)
    return p


def split_path(path, c_robot=None):
    path = path.flatten()
    grasp_path_idxs = [0]
    placing_path_idxs = []
    freefly_path_idxs = []

    for idx in range(1, path.numberPaths()):
        if path_move_object(path.pathAtRank(idx)):
            placing_path_idxs.append(idx)
        else:
            if len(placing_path_idxs) == 0:
                grasp_path_idxs.append(idx)
            else:
                freefly_path_idxs.append(idx)
    grasp_path = concatenatePaths(
        [path.pathAtRank(idx) for idx in grasp_path_idxs], c_robot
    )
    placing_path = concatenatePaths(
        [path.pathAtRank(idx) for idx in placing_path_idxs], c_robot
    )
    freefly_path = concatenatePaths(
        [path.pathAtRank(idx) for idx in freefly_path_idxs], c_robot
    )
    return grasp_path, placing_path, freefly_path


def path_move_object(path):
    object_init_pose = np.array(path.initial()[9:12])
    object_end_pose = np.array(path.end()[9:12])
    eps = 1e-4
    if np.linalg.norm(object_end_pose - object_init_pose) < eps:
        return False
    else:
        return True


class BaseObject(object):
    def __init__(
        self,
        urdf_path: str,
        srdf_path: str,
        name: str,
        rootJointType: str = "freeflyer",
    ):
        self.urdfFilename = urdf_path
        self.srdfFilename = srdf_path
        self.name = name
        self.rootJointType = rootJointType


def get_obj_goal_handles(prefix: str, srdf_path: str) -> (list[str], list[str]):
    """Returns the object and goal handles from the srdf file."""
    tree = ET.parse(srdf_path)
    root = tree.getroot()
    all_handles = [handle.attrib["name"] for handle in root.findall("handle")]
    goal_handles = [prefix + handle for handle in all_handles if "goal" in handle]
    object_handles = [prefix + handle for handle in all_handles if "goal" not in handle]
    return object_handles, goal_handles


def normalize_quaternion(pose: XYZQuatType) -> XYZQuatType:
    pose[3:] = pose[3:] / np.linalg.norm(pose[3:])
    return pose


def posemsg2mat(pose: Pose) -> npt.NDArray:
    """Convert a ROS2 Pose message to a 4x4 numpy array."""
    return pin.XYZQUATToSE3(
        np.array(
            [
                pose.position.x,
                pose.position.y,
                pose.position.z,
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
        )
    ).homogeneous


def graspnet_to_handle(world_to_cam: pin.SE3, cam_to_grasp: pin.SE3) -> list[float]:
    # convert graspnet frame to franka hand frame
    grasp_to_ee = pin.SE3(pin.rpy.rpyToMatrix(0, 0, -np.pi / 2), np.zeros(3))
    world_to_ee = world_to_cam * cam_to_grasp * grasp_to_ee
    # from franka hand to grasp location, and align x with axis going through the fingers
    ee_to_grasp = pin.SE3(
        pin.rpy.rpyToMatrix(0, -np.pi / 2, 0), np.array([0, 0, 0.103])
    )
    handle_in_world = world_to_ee * ee_to_grasp
    return pin.SE3ToXYZQUAT(handle_in_world).tolist()


def multiply_poses(pose1: XYZQuatType, pose2: XYZQuatType) -> XYZQuatType:
    p1 = pin.XYZQUATToSE3(pose1)
    p2 = pin.XYZQUATToSE3(pose2)
    return normalize_quaternion(pin.SE3ToXYZQUAT(p1 * p2).tolist())


def inverse_pose(pose: XYZQuatType) -> XYZQuatType:
    p = pin.XYZQUATToSE3(pose)
    return normalize_quaternion(pin.SE3ToXYZQUAT(p.inverse()).tolist())


def config_dist(q1: XYZQuatType, q2: XYZQuatType) -> float:
    """Computes distance between two configurations.
    Initial implementation is just Euclidean distance between the two
    """
    return np.linalg.norm(np.array(q1) - np.array(q2))
