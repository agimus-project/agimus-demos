#!/usr/bin/env python3
"""
Visualize Tiago Pro + pylone collision models in the same Viser scene.

Loads both the robot URDF (with_sc:=true) and environment.urdf.xacro
(with_sc:=true), merges them via pin.appendModel, and displays:
  - real visual geometry (opaque meshes) for robot and pylone
  - _sc capsule collision links in transparent green for both

If ROS 2 is running, subscribes to /robot_description (live calibrated URDF)
and /joint_states (live pose). Falls back to xacro + static pose.

Usage:
    python3 scripts/display_full_collision.py
"""

import math
import os
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import yaml
import xacro
from ament_index_python.packages import get_package_share_directory

import pinocchio as pin

# --- Paths ---
TIAGO_DESC = Path(get_package_share_directory("tiago_pro_description"))
ROBOT_XACRO = TIAGO_DESC / "robots" / "tiago_pro.urdf.xacro"
_PKG_DIR = Path(__file__).resolve().parent.parent  # source dir, no rebuild needed
ENV_XACRO = _PKG_DIR / "urdf" / "environment.urdf.xacro"
PACKAGE_DIRS = [str(TIAGO_DESC.parent.parent), str(_PKG_DIR.parent)]

ROBOT_MAPPINGS = {
    "end_effector_right": "pal-atc",
    "end_effector_left": "pal-pro-gripper",
    "wrist_model_right": "spherical-wrist",
    "with_sc": "true",
    "camera_model": "realsense-d435",
}

_STATIC_JOINTS = [
    ("arm_right_2_joint", 30.0),
    ("arm_right_4_joint", 30.0),
    ("arm_right_6_joint", 30.0),
]
_DISPLAY_PERIOD = 0.05


# --- Helpers ---


def _load_pylone_pose():
    with open(_PKG_DIR / "config" / "pylone_pose.yaml") as f:
        pp = yaml.safe_load(f)
    q = pp.get("pylone_quat", [0.0, 0.0, 0.0, 1.0])
    qx, qy, qz, qw = q
    roll = math.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (qw * qy - qz * qx))))
    yaw = math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy * qy + qz * qz))
    return {
        "pylone_x": str(pp["pylone_x"]),
        "pylone_y": str(pp["pylone_y"]),
        "pylone_z": str(pp["pylone_z"]),
        "pylone_roll": str(roll),
        "pylone_pitch": str(pitch),
        "pylone_yaw": str(yaw),
    }


def _build_models(urdf_string):
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="w") as f:
        f.write(urdf_string)
        tmp = f.name
    try:
        return pin.buildModelsFromUrdf(tmp, package_dirs=PACKAGE_DIRS)
    finally:
        os.unlink(tmp)


def _filter_sc(geom_model):
    sc = pin.GeometryModel()
    for g in geom_model.geometryObjects:
        if "_sc" in g.name:
            sc.addGeometryObject(g)
    return sc


def _merge(env_model, robot_model, env_geom, robot_geom):
    """Append robot model to env model at env's universe frame."""
    universe_id = env_model.getFrameId("universe")
    return pin.appendModel(
        env_model,
        robot_model,
        env_geom,
        robot_geom,
        universe_id,
        pin.SE3.Identity(),
    )


def _static_q(model):
    q = pin.neutral(model)
    for joint_name, angle_deg in _STATIC_JOINTS:
        for prefix in ("tiago_pro/", ""):
            jid = model.getJointId(prefix + joint_name)
            if jid < model.njoints:
                q[model.joints[jid].idx_q] = np.deg2rad(angle_deg)
                break
    return q


class _PinWrapper:
    def __init__(self, model, collision_model, visual_model):
        self._m, self._cm, self._vm = model, collision_model, visual_model

    def model(self):
        return self._m

    def geomModel(self):
        return self._cm

    def visualModel(self):
        return self._vm


def _reload_viewer(viz_ref, model, sc_model, visual_model, q=None):
    from pyhpp_viser import Viewer

    wrapper = _PinWrapper(model, sc_model, visual_model)
    viz = Viewer(wrapper)
    open_browser = viz_ref[0] is None
    viz.initViewer(open=open_browser, loadModel=False)
    viz.loadViewerModel(collision_color=(0.0, 0.8, 0.0, 0.35))
    viz.displayVisuals(True)
    viz.displayCollisions(True)
    viz.display(q if q is not None else _static_q(model))
    viz_ref[0] = viz


def _try_ros_mode(viz_ref, sc_xml, env_model, env_sc, env_visual):
    """Live mode: rebuild combined model on /robot_description, update pose on /joint_states."""
    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import JointState
        from std_msgs.msg import String
    except ImportError:
        print("rclpy not available — static display only.")
        return
    try:
        rclpy.init()
    except Exception as e:
        print(f"rclpy.init() failed ({e}) — static display only.")
        return

    state = {"model": None, "q": None, "last_disp": 0.0}

    class _Listener(Node):
        def __init__(self):
            super().__init__("display_full_collision")
            qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(String, "/robot_description", self._urdf_cb, qos)
            self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        def _urdf_cb(self, msg):
            # RSP already publishes with with_sc:=true — inject only if _sc links are missing.
            if "_link_sc" in msg.data:
                robot_urdf = msg.data
            else:
                robot_urdf = msg.data.replace("</robot>", sc_xml + "\n</robot>")
            try:
                robot_model, robot_collision, robot_visual = _build_models(robot_urdf)
            except Exception as e:
                self.get_logger().error(f"Failed to parse /robot_description: {e}")
                return
            robot_sc = _filter_sc(robot_collision)
            combined_model, combined_visual = _merge(
                env_model, robot_model, env_visual, robot_visual
            )
            _, combined_sc = _merge(env_model, robot_model, env_sc, robot_sc)
            self.get_logger().info("Reloading viewer from /robot_description.")
            _reload_viewer(
                viz_ref, combined_model, combined_sc, combined_visual, state["q"]
            )
            state["model"] = combined_model

        def _js_cb(self, msg):
            model = state["model"]
            if model is None or viz_ref[0] is None:
                return
            now = time.monotonic()
            if now - state["last_disp"] < _DISPLAY_PERIOD:
                return
            state["last_disp"] = now
            js_map = dict(zip(msg.name, msg.position))
            q = _static_q(model)
            for jid in range(1, model.njoints):
                jname = model.names[jid]
                val = js_map.get(jname) or js_map.get(jname.replace("tiago_pro/", ""))
                if val is not None and model.joints[jid].nq == 1:
                    q[model.joints[jid].idx_q] = val
            state["q"] = q
            viz_ref[0].display(q)

    node = _Listener()
    print("ROS mode active — subscribed to /robot_description and /joint_states.")
    threading.Thread(target=lambda: rclpy.spin(node), daemon=True).start()


def main():
    # --- Robot ---
    print("Processing Tiago Pro xacro …")
    robot_urdf = xacro.process_file(str(ROBOT_XACRO), mappings=ROBOT_MAPPINGS).toxml()
    tree = ET.fromstring(robot_urdf)
    # Match only names ending with "_sc" (links) or containing "_link_sc" (joints)
    # to avoid false positives like "head_screen_link".
    sc_xml = "\n".join(
        ET.tostring(e, encoding="unicode")
        for e in tree
        if e.get("name", "").endswith("_sc") or "_link_sc" in e.get("name", "")
    )
    robot_model, robot_collision, robot_visual = _build_models(robot_urdf)
    robot_sc = _filter_sc(robot_collision)

    # --- Environment ---
    print("Loading pylone pose …")
    pose_args = _load_pylone_pose()
    print(
        f"  x={pose_args['pylone_x']}, y={pose_args['pylone_y']}, z={pose_args['pylone_z']}"
    )
    print("Processing environment.urdf.xacro …")
    env_urdf = xacro.process_file(
        str(ENV_XACRO), mappings={"with_sc": "true", **pose_args}
    ).toxml()
    env_model, env_collision, env_visual = _build_models(env_urdf)
    env_sc = _filter_sc(env_collision)

    # --- Merge ---
    print("Merging models …")
    combined_model, combined_visual = _merge(
        env_model, robot_model, env_visual, robot_visual
    )
    _, combined_sc = _merge(env_model, robot_model, env_sc, robot_sc)

    print(f"\nSC geometries ({combined_sc.ngeoms} total):")
    for g in combined_sc.geometryObjects:
        print(f"  {g.name}  ({type(g.geometry).__name__})")

    # --- Display ---
    print("\nStarting Viser …")
    try:
        from pyhpp_viser import Viewer  # noqa
    except ImportError as e:
        print(f"pyhpp_viser not available: {e}")
        sys.exit(1)

    viz_ref = [None]
    _reload_viewer(viz_ref, combined_model, combined_sc, combined_visual)

    _try_ros_mode(viz_ref, sc_xml, env_model, env_sc, env_visual)

    print("\nViewer ready — press Ctrl-C to exit.")
    try:
        input()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
