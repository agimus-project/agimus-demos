#!/usr/bin/env python3
"""
Visualize the pylone collision model in Viser.

Loads environment.urdf.xacro with with_sc:=true and displays the real pylone
visual geometry (opaque) alongside the simplified capsule collision links
(pylone_sc_link) so you can verify capsule placement.

The pylone pose is read from config/pylone_pose.yaml (same as bringup).

Usage (inside the container, with ROS 2 sourced):
    python3 scripts/display_pylone_collision.py

Then open the Viser URL printed in the terminal.
"""

import math
import os
import sys
import tempfile
from pathlib import Path

import yaml
import xacro

import pinocchio as pin

# Use __file__ so the script works from source without colcon rebuild
_PKG_DIR = Path(__file__).resolve().parent.parent
ENV_XACRO_PATH = _PKG_DIR / "urdf" / "environment.urdf.xacro"
PACKAGE_DIRS = [str(_PKG_DIR.parent)]


def _load_pylone_pose():
    pose_file = _PKG_DIR / "config" / "pylone_pose.yaml"
    with open(pose_file) as f:
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


def _filter_sc(collision_model):
    sc = pin.GeometryModel()
    for g in collision_model.geometryObjects:
        if "_sc" in g.name:
            sc.addGeometryObject(g)
    return sc


class _PinWrapper:
    def __init__(self, model, collision_model, visual_model):
        self._m, self._cm, self._vm = model, collision_model, visual_model

    def model(self):
        return self._m

    def geomModel(self):
        return self._cm

    def visualModel(self):
        return self._vm


def main():
    print("Loading pylone pose from config/pylone_pose.yaml …")
    pose_args = _load_pylone_pose()
    print(
        f"  position: x={pose_args['pylone_x']}, y={pose_args['pylone_y']}, z={pose_args['pylone_z']}"
    )

    print("Processing environment.urdf.xacro …")
    env_urdf = xacro.process_file(
        str(ENV_XACRO_PATH),
        mappings={"with_sc": "true", **pose_args},
    ).toxml()

    print("Building Pinocchio model …")
    model, collision_model, visual_model = _build_models(env_urdf)

    sc_model = _filter_sc(collision_model)
    print(f"\nPylone SC geometries ({sc_model.ngeoms} shapes):")
    for g in sc_model.geometryObjects:
        print(f"  {g.name}  ({type(g.geometry).__name__})")

    print("\nStarting Viser …")
    try:
        from pyhpp_viser import Viewer
    except ImportError as e:
        print(f"pyhpp_viser not available: {e}")
        sys.exit(1)

    wrapper = _PinWrapper(model, sc_model, visual_model)
    viz = Viewer(wrapper)
    viz.initViewer(open=True, loadModel=False)
    viz.loadViewerModel(collision_color=(0.0, 0.8, 0.0, 0.35))
    viz.displayVisuals(True)
    viz.displayCollisions(True)
    viz.display(pin.neutral(model))

    print("\nViewer ready — press Ctrl-C to exit.")
    try:
        input()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
