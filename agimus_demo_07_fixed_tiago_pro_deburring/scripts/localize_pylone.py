#!/usr/bin/env python3
"""
Pylone localization via manual pointing.

Switches the right arm to gravity compensation, asks the operator to point
the gripper tip at 3 known holes of the pylone, then estimates the pylone
pose using FK + SVD registration.

Run this script BEFORE launching the LFC bringup, while arm_right_controller
is still active.

Usage:
    python3 localize_pylone.py
    python3 localize_pylone.py --holes hole_right_00 hole_right_25 hole_right_49
    python3 localize_pylone.py --output /tmp/pylone_pose.yaml
"""

import argparse
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import pinocchio as pin
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import yaml

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
PYLONE_SRDF = os.path.join(_SCRIPTS_DIR, "..", "hpp", "pylone.srdf")

EE_LINK = "gripper_right_fingertip_left_link"
GRAVITY_COMP_CONTROLLER = "arm_right_gravity_compensation_controller_torque"
ARM_POSITION_CONTROLLER = "arm_right_controller"

DEFAULT_HOLES = ["hole_right_00", "hole_right_25", "hole_right_49"]


def parse_pylone_holes(srdf_path: str) -> dict:
    tree = ET.parse(srdf_path)
    holes = {}
    for handle in tree.getroot().findall("handle"):
        name = handle.get("name")
        pos_elem = handle.find("position")
        if pos_elem is not None:
            xyz = list(map(float, pos_elem.get("xyz").split()))
            holes[name] = np.array(xyz)
    return holes


def switch_controllers(deactivate: list, activate: list) -> None:
    cmd = ["ros2", "control", "switch_controllers"]
    if deactivate:
        cmd += ["--deactivate"] + deactivate
    if activate:
        cmd += ["--activate"] + activate
    subprocess.run(cmd, check=True)


def get_urdf(node: Node, timeout: float = 10.0) -> str:
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    result = [None]
    sub = node.create_subscription(
        String, "/robot_description",
        lambda msg: result.__setitem__(0, msg.data), qos
    )
    t0 = time.time()
    while result[0] is None and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if result[0] is None:
        raise RuntimeError(f"Timed out after {timeout}s waiting for /robot_description")
    return result[0]


def get_joint_state(node: Node, timeout: float = 5.0):
    result = [None]
    sub = node.create_subscription(
        JointState, "/joint_states",
        lambda msg: result.__setitem__(0, msg), 10
    )
    t0 = time.time()
    while result[0] is None and time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return result[0]


def fk_ee(urdf_str: str, joint_state: JointState) -> np.ndarray:
    model = pin.buildModelFromXML(urdf_str)
    data = model.createData()
    q = pin.neutral(model)
    for name, pos in zip(joint_state.name, joint_state.position):
        for jname in [name, f"tiago_pro/{name}"]:
            if model.existJointName(jname):
                idx = model.getJointId(jname)
                q[model.joints[idx].idx_q] = pos
                break
    pin.forwardKinematics(model, data, q)
    pin.updateFramePlacements(model, data)
    frame_id = model.getFrameId(EE_LINK)
    if frame_id == model.nframes:
        raise RuntimeError(f"Frame '{EE_LINK}' not found in model")
    return data.oMf[frame_id].translation.copy()


def estimate_pose_svd(p_base: np.ndarray, p_pylone: np.ndarray):
    c_base = p_base.mean(axis=0)
    c_pylone = p_pylone.mean(axis=0)
    H = (p_base - c_base).T @ (p_pylone - c_pylone)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.linalg.det(U @ Vt)])
    R = U @ D @ Vt
    t = c_base - R @ c_pylone
    return R, t


def main():
    parser = argparse.ArgumentParser(description="Localize pylone via manual pointing.")
    parser.add_argument(
        "--holes", nargs=3, default=DEFAULT_HOLES, metavar="HOLE",
        help=f"3 hole names from pylone SRDF (default: {DEFAULT_HOLES})"
    )
    parser.add_argument(
        "--output", default=None,
        help="Path to write result YAML (default: print only)"
    )
    args = parser.parse_args()

    all_holes = parse_pylone_holes(PYLONE_SRDF)
    for h in args.holes:
        if h not in all_holes:
            print(f"ERROR: hole '{h}' not found in {PYLONE_SRDF}")
            print(f"Available holes: {sorted(all_holes.keys())}")
            sys.exit(1)
    p_pylone = np.array([all_holes[h] for h in args.holes])

    rclpy.init()
    node = rclpy.create_node("pylone_localization")

    print("Fetching robot URDF …")
    urdf_str = get_urdf(node)

    print(f"\nSwitching to gravity compensation …")
    switch_controllers(
        deactivate=[ARM_POSITION_CONTROLLER],
        activate=[GRAVITY_COMP_CONTROLLER],
    )
    print("Arm is in gravity compensation mode. You can move it freely.\n")

    p_base = np.zeros((3, 3))
    try:
        for i, hole_name in enumerate(args.holes):
            xyz = all_holes[hole_name]
            print(f"[{i+1}/3] Point gripper at hole '{hole_name}'  (pylone frame: {np.round(xyz, 3).tolist()})")
            input("      → Press Enter when ready …")
            js = get_joint_state(node)
            if js is None:
                raise RuntimeError("Could not read /joint_states")
            p_base[i] = fk_ee(urdf_str, js)
            print(f"      Measured position: {np.round(p_base[i], 4).tolist()}\n")
    finally:
        print("Switching back to position control …")
        switch_controllers(
            deactivate=[GRAVITY_COMP_CONTROLLER],
            activate=[ARM_POSITION_CONTROLLER],
        )

    R, t = estimate_pose_svd(p_base, p_pylone)

    residuals = [np.linalg.norm(R @ p_pylone[i] + t - p_base[i]) for i in range(3)]
    quat = pin.Quaternion(R)

    print("\n=== Pylone pose in base_link frame ===")
    print(f"position (x, y, z):  {np.round(t, 4).tolist()}")
    print(f"quaternion (x,y,z,w): {[round(v, 6) for v in [quat.x, quat.y, quat.z, quat.w]]}")
    print(f"residuals (m):        {[round(r, 5) for r in residuals]}")
    print(f"mean residual (m):    {np.mean(residuals):.5f}")

    result = {
        "scene": {
            "pylone_x": round(float(t[0]), 4),
            "pylone_y": round(float(t[1]), 4),
            "pylone_z": round(float(t[2]), 4),
        }
    }
    print("\nYAML snippet for hpp_orchestrator_params.yaml:")
    print(yaml.dump(result, default_flow_style=False))

    if args.output:
        with open(args.output, "w") as f:
            yaml.dump(result, f)
        print(f"Result written to {args.output}")

    t_list = [round(float(v), 4) for v in t]
    print("To visualize in Viser, run in the HPP orchestrator IPython shell:")
    print(f"    o.init_viewer()  # if not already open")
    print(f"    o.update_pylone_pose({t_list})")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
