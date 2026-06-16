#!/usr/bin/env python3
"""
Visualize the Tiago Pro collision model in Viser.

Loads the full robot URDF with with_sc:=true and displays the real visual
geometry (opaque meshes) alongside the simplified capsule collision links
(_sc links on arm links 3, 5, 7) so you can tune capsule placement.

If ROS 2 is running, subscribes to /robot_description (calibrated URDF, _sc
links injected automatically) and /joint_states (live robot pose). Falls back
to xacro loading and a fixed display pose when ROS is not available.

Usage (inside the container, with ROS 2 sourced):
    python3 scripts/display_collision_model.py

Then open the Viser URL printed in the terminal.
"""

import os
import sys
import tempfile
import threading
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory

import pinocchio as pin

TIAGO_DESC = Path(get_package_share_directory("tiago_pro_description"))
XACRO_PATH = TIAGO_DESC / "robots" / "tiago_pro.urdf.xacro"
PACKAGE_DIRS = [str(TIAGO_DESC.parent.parent)]

# Same xacro args as bringup.launch.py + enable simplified capsule links
XACRO_MAPPINGS = {
    "end_effector_right": "pal-atc",
    "end_effector_left":  "pal-pro-gripper",
    "wrist_model_right":  "spherical-wrist",
    "with_sc":            "true",
}

# Display joint angles for the fallback static pose
_STATIC_JOINTS = [
    ("arm_right_2_joint", 30.0),
    ("arm_right_4_joint", 30.0),
    ("arm_right_6_joint", 30.0),
]

# Throttle live display updates
_DISPLAY_PERIOD = 0.05  # s  (20 Hz max)


class _PinRobotWrapper:
    """Minimal wrapper to make a Pinocchio model look like an HPP Device."""
    def __init__(self, model, collision_model, visual_model):
        self._m  = model
        self._cm = collision_model
        self._vm = visual_model
    def model(self):       return self._m
    def geomModel(self):   return self._cm
    def visualModel(self): return self._vm


def _build_models(urdf_string):
    with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="w") as f:
        f.write(urdf_string)
        tmp = f.name
    try:
        return pin.buildModelsFromUrdf(tmp, package_dirs=PACKAGE_DIRS)
    finally:
        os.unlink(tmp)


def _filter_sc(collision_model):
    sc = pin.GeometryModel()
    for g in collision_model.geometryObjects:
        if "_sc" in g.name:
            sc.addGeometryObject(g)
    return sc


def _static_q(model):
    """Build a configuration with the static display joint angles."""
    q = pin.neutral(model)
    for joint_name, angle_deg in _STATIC_JOINTS:
        for prefix in ("tiago_pro/", ""):
            try:
                jid = model.getJointId(prefix + joint_name)
                if jid < model.njoints:
                    q[model.joints[jid].idx_q] = np.deg2rad(angle_deg)
                    break
            except Exception:
                pass
    return q


def _reload_viewer(viz_ref, model, sc_collision_model, visual_model, q=None):
    """(Re)initialize the Viser scene with a new model. Thread-safe."""
    from pyhpp_viser import Viewer

    wrapper = _PinRobotWrapper(model, sc_collision_model, visual_model)
    viz = Viewer(wrapper)
    open_browser = viz_ref[0] is None
    viz.initViewer(open=open_browser, loadModel=False)
    viz.loadViewerModel(collision_color=(0.0, 0.8, 0.0, 0.35))
    viz.displayVisuals(True)
    viz.displayCollisions(True)
    viz.display(q if q is not None else _static_q(model))
    viz_ref[0] = viz


def _try_ros_mode(viz_ref, sc_xml, initial_model):
    """Subscribe to /robot_description and /joint_states if ROS 2 is available.

    Runs in a background daemon thread — no-op if ROS is not available.
    """
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

    state = {
        "model":    initial_model,
        "q":        None,
        "last_disp": 0.0,
    }

    class _Listener(Node):
        def __init__(self):
            super().__init__("display_collision_model")
            qos_latched = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(String, "/robot_description", self._urdf_cb, qos_latched)
            self.create_subscription(JointState, "/joint_states", self._js_cb, 10)

        def _urdf_cb(self, msg):
            # Inject _sc links into the calibrated URDF from the robot
            urdf = msg.data.replace("</robot>", sc_xml + "\n</robot>")
            try:
                model, collision_model, visual_model = _build_models(urdf)
            except Exception as e:
                self.get_logger().error(f"Failed to build model from /robot_description: {e}")
                return
            sc_collision_model = _filter_sc(collision_model)
            self.get_logger().info(
                f"Reloading viewer from /robot_description "
                f"({sc_collision_model.ngeoms} SC geoms)."
            )
            _reload_viewer(viz_ref, model, sc_collision_model, visual_model, state["q"])
            state["model"] = model

        def _js_cb(self, msg):
            model = state["model"]
            if model is None or viz_ref[0] is None:
                return
            now = time.monotonic()
            if now - state["last_disp"] < _DISPLAY_PERIOD:
                return
            state["last_disp"] = now

            js_map = dict(zip(msg.name, msg.position))
            q = pin.neutral(model)
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
    print("Processing Tiago Pro xacro …")
    robot_urdf = xacro.process_file(str(XACRO_PATH), mappings=XACRO_MAPPINGS).toxml()

    # Extract _sc link/joint XML so it can be injected into /robot_description later
    tree = ET.fromstring(robot_urdf)
    sc_xml = "\n".join(
        ET.tostring(elem, encoding="unicode")
        for elem in tree
        if "_sc" in elem.get("name", "")
    )

    print("Building Pinocchio model …")
    model, collision_model, visual_model = _build_models(robot_urdf)

    sc_collision_model = _filter_sc(collision_model)
    print(f"\nSC capsule geometries ({sc_collision_model.ngeoms} shapes):")
    for g in sc_collision_model.geometryObjects:
        print(f"  {g.name}  ({type(g.geometry).__name__})")

    print("\nStarting Viser …")
    try:
        from pyhpp_viser import Viewer  # noqa: F401 — check import before _reload_viewer
    except ImportError as e:
        print(f"pyhpp_viser not available: {e}")
        sys.exit(1)

    viz_ref = [None]
    _reload_viewer(viz_ref, model, sc_collision_model, visual_model)

    _try_ros_mode(viz_ref, sc_xml, model)

    print("\nViewer ready — press Ctrl-C to exit.")
    try:
        input()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
