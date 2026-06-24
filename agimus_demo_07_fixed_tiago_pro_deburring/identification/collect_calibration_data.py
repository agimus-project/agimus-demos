#!/usr/bin/env python3
"""
Calibration data collector for Tiago Pro — automatic version.

For each optimal configuration (from generate_optimal_configs.py):
  1. Sends a FollowJointTrajectory goal to arm_right_controller and
     torso_controller sequentially (torso first, then arm).
  2. Waits for the robot to settle.
  3. Records the (q, mocap_EE_pose) pair automatically.
  4. Advances to the next configuration.

Viser shows the live robot state and the target EE frame simultaneously.

Prerequisites:
  - arm_right_controller and torso_controller active (NOT agimus_controller)
  - mocap_ee_publisher.py running (publishes /mocap_ee_pose)

Usage:
    python3 scripts/collect_calibration_data.py
    python3 scripts/collect_calibration_data.py \\
        --configs /home/gepetto/ros2_ws/src/figaroh_tiagoPro/data/optimal_configs.yaml \\
        --output  /home/gepetto/ros2_ws/src/figaroh_tiagoPro/data/calibration_samples.csv \\
        --duration 5.0
"""

import argparse
import csv
import os
import sys
import threading
import time
from pathlib import Path

import numpy as np
import yaml

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

import pinocchio as pin

_IDENTIFICATION = Path(__file__).parent
_CONFIGS_DEFAULT = _IDENTIFICATION / "optimal_configs.yaml"
_OUTPUT_DEFAULT  = _IDENTIFICATION / "calibration_samples.csv"

ARM_JOINTS = [
    "arm_right_1_joint",
    "arm_right_2_joint",
    "arm_right_3_joint",
    "arm_right_4_joint",
    "arm_right_5_joint",
    "arm_right_6_joint",
    "arm_right_7_joint",
]
TORSO_JOINTS  = ["torso_lift_joint"]
ACTIVE_JOINTS = TORSO_JOINTS + ARM_JOINTS

_ARM_ACTION   = "/arm_right_controller/follow_joint_trajectory"
_TORSO_ACTION = "/torso_controller/follow_joint_trajectory"

_SETTLE_S        = 5.0
_FRESHNESS_S     = 0.5
_STILL_THRESHOLD = 0.005  # rad/s


# ── Utility ───────────────────────────────────────────────────────────────────

def _q_from_js(model, js_msg):
    js_map = dict(zip(js_msg.name, js_msg.position))
    q = pin.neutral(model)
    for jid in range(1, model.njoints):
        jname = model.names[jid]
        val = js_map.get(jname) or js_map.get(f"tiago_pro/{jname}")
        if val is not None and model.joints[jid].nq == 1:
            q[model.joints[jid].idx_q] = val
    return q


def _q_from_active(model, active_vals):
    q = pin.neutral(model)
    for jname, val in zip(ACTIVE_JOINTS, active_vals):
        for candidate in (jname, f"tiago_pro/{jname}"):
            try:
                jid = model.getJointId(candidate)
                if jid < model.njoints and model.joints[jid].nq == 1:
                    q[model.joints[jid].idx_q] = val
                    break
            except Exception:
                pass
    return q


def _get_active_vals(model, q):
    vals = []
    for jname in ACTIVE_JOINTS:
        found = False
        for candidate in (jname, f"tiago_pro/{jname}"):
            try:
                jid = model.getJointId(candidate)
                if jid < model.njoints and model.joints[jid].nq == 1:
                    vals.append(float(q[model.joints[jid].idx_q]))
                    found = True
                    break
            except Exception:
                pass
        if not found:
            vals.append(0.0)
    return vals


def _fk_ee(model, q, ee_id):
    data = model.createData()
    pin.framesForwardKinematics(model, data, q)
    return data.oMf[ee_id].copy()


def _print_error_table(target_vals, current_vals):
    print(f"\n  {'Joint':<30} {'Target':>9} {'Current':>9} {'Error':>9}")
    print("  " + "─" * 62)
    for jname, tgt, cur in zip(ACTIVE_JOINTS, target_vals, current_vals):
        err = np.degrees(cur - tgt)
        flag = "✓" if abs(err) < 1.0 else ("~" if abs(err) < 3.0 else "✗")
        print(f"  {jname:<30} {np.degrees(tgt):>8.2f}° {np.degrees(cur):>8.2f}°  {err:>+7.2f}°  {flag}")


def _make_trajectory(joint_names, positions, duration_sec):
    traj = JointTrajectory()
    traj.joint_names = joint_names
    pt = JointTrajectoryPoint()
    pt.positions  = [float(p) for p in positions]
    pt.velocities = [0.0] * len(positions)
    secs  = int(duration_sec)
    nsecs = int((duration_sec - secs) * 1e9)
    pt.time_from_start = Duration(sec=secs, nanosec=nsecs)
    traj.points = [pt]
    return traj


# ── Viser display ─────────────────────────────────────────────────────────────

class ViserDisplay:

    def __init__(self, model, visual_model, collision_model):
        from pyhpp_viser import Viewer

        class _Wrap:
            def __init__(self, m, cm, vm):
                self._m, self._cm, self._vm = m, cm, vm
            def model(self):       return self._m
            def geomModel(self):   return self._cm
            def visualModel(self): return self._vm

        self._model = model
        self.viz = Viewer(_Wrap(model, collision_model, visual_model))
        self.viz.initViewer(open=True, loadModel=False)
        self.viz.loadViewerModel()
        self.viz.displayVisuals(True)
        self.viz.displayCollisions(False)
        self._server = self.viz.viewer

        self._current_frame = self._server.scene.add_frame(
            "calib/current_ee", show_axes=True, axes_length=0.10, axes_radius=0.004,
        )
        self._target_frame = self._server.scene.add_frame(
            "calib/target_ee", show_axes=True, axes_length=0.14, axes_radius=0.007,
        )
        self._last_upd = 0.0

        ee_name = "gripper_right_tool_holder"
        self._ee_id = model.getFrameId(ee_name)
        if self._ee_id >= len(model.frames):
            self._ee_id = model.getFrameId(f"tiago_pro/{ee_name}")

    def _se3_to_viser(self, T):
        return T.translation, pin.Quaternion(T.rotation).coeffs()[[3, 0, 1, 2]]

    def update_current(self, q):
        now = time.monotonic()
        if now - self._last_upd < 0.05:
            return
        self._last_upd = now
        self.viz.display(q)
        pos, wxyz = self._se3_to_viser(_fk_ee(self._model, q, self._ee_id))
        self._current_frame.position = pos
        self._current_frame.wxyz     = wxyz

    def update_target(self, q_target):
        pos, wxyz = self._se3_to_viser(_fk_ee(self._model, q_target, self._ee_id))
        self._target_frame.position = pos
        self._target_frame.wxyz     = wxyz


# ── ROS 2 node ────────────────────────────────────────────────────────────────

class CalibrationCollector(Node):

    def __init__(self, output_path, viser, model, duration_sec):
        super().__init__("calibration_collector")
        self._output_path = output_path
        self._viser       = viser
        self._model       = model
        self._duration    = duration_sec
        self._samples     = []
        self._last_target = None
        self._last_ee_pos = None   # guard against frozen mocap

        self._lock     = threading.Lock()
        self._js_msg   : JointState | None  = None
        self._pose_msg : PoseStamped | None = None

        self.create_subscription(JointState,  "/joint_states",  self._js_cb,   10)
        self.create_subscription(PoseStamped, "/mocap_ee_pose", self._pose_cb, 10)

        self._arm_client   = ActionClient(self, FollowJointTrajectory, _ARM_ACTION)
        self._torso_client = ActionClient(self, FollowJointTrajectory, _TORSO_ACTION)

    def _js_cb(self, msg):
        with self._lock:
            self._js_msg = msg
        if self._viser and self._model:
            self._viser.update_current(_q_from_js(self._model, msg))

    def _pose_cb(self, msg):
        with self._lock:
            self._pose_msg = msg

    def _is_fresh(self, stamp):
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now - (stamp.sec + stamp.nanosec * 1e-9)) < _FRESHNESS_S

    def _is_still(self, js):
        js_map = dict(zip(js.name, js.velocity))
        for jname in ACTIVE_JOINTS:
            vel = js_map.get(jname, js_map.get(f"tiago_pro/{jname}", 0.0))
            if abs(vel) > _STILL_THRESHOLD:
                return False
        return True

    def _wait_for_servers(self, timeout=10.0):
        self.get_logger().info("Waiting for action servers ...")
        if not self._arm_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f"Action server not available: {_ARM_ACTION}")
        if not self._torso_client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f"Action server not available: {_TORSO_ACTION}")
        self.get_logger().info("Action servers ready.")

    def _send_goal(self, client, joint_names, positions, duration_sec):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = _make_trajectory(joint_names, positions, duration_sec)
        future = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done():
            self.get_logger().warn("Goal send timed out — is the controller running?")
            return False
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn("Goal rejected!")
            return False
        result_future = handle.get_result_async()
        timeout = duration_sec + 15.0
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
        if not result_future.done():
            self.get_logger().warn(f"Goal execution timed out after {timeout:.0f}s — skipping.")
            return False
        return result_future.result().status == GoalStatus.STATUS_SUCCEEDED

    def move_to(self, target_vals):
        torso_vals = [target_vals[ACTIVE_JOINTS.index(j)] for j in TORSO_JOINTS]
        arm_vals   = [target_vals[ACTIVE_JOINTS.index(j)] for j in ARM_JOINTS]

        self.get_logger().info("Moving torso ...")
        ok_t = self._send_goal(self._torso_client, TORSO_JOINTS, torso_vals, self._duration)
        self.get_logger().info("Moving arm ...")
        ok_a = self._send_goal(self._arm_client,   ARM_JOINTS,   arm_vals,   self._duration)
        return ok_t and ok_a

    def wait_and_record(self):
        self.get_logger().info(f"Settling ({_SETTLE_S:.1f}s) ...")
        time.sleep(_SETTLE_S)

        with self._lock:
            js   = self._js_msg
            pose = self._pose_msg

        if js is None or pose is None:
            self.get_logger().warn("Missing messages — skipping sample.")
            return False
        if not self._is_fresh(js.header.stamp):
            self.get_logger().warn("/joint_states is stale — skipping.")
            return False
        if not self._is_fresh(pose.header.stamp):
            self.get_logger().warn("/mocap_ee_pose is stale — skipping.")
            return False
        if not self._is_still(js):
            self.get_logger().warn("Robot still moving — waiting 1s more ...")
            time.sleep(1.0)
            with self._lock:
                js = self._js_msg
            if not self._is_still(js):
                self.get_logger().warn("Still moving — skipping sample.")
                return False

        ee_pos = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
        if self._last_ee_pos is not None and np.linalg.norm(ee_pos - self._last_ee_pos) < 1e-4:
            self.get_logger().error(
                "EE position unchanged from previous sample — mocap may be frozen! Skipping."
            )
            return False
        self._last_ee_pos = ee_pos

        js_map = dict(zip(js.name, js.position))
        sample = {
            "x1": float(ee_pos[0]),
            "y1": float(ee_pos[1]),
            "z1": float(ee_pos[2]),
        }
        for jname in ACTIVE_JOINTS:
            val = js_map.get(jname) or js_map.get(f"tiago_pro/{jname}")
            if val is None:
                self.get_logger().warn(f"Joint '{jname}' missing — skipping.")
                return False
            sample[jname] = val

        self._samples.append(sample)
        x, y, z = sample["x1"], sample["y1"], sample["z1"]
        self.get_logger().info(
            f"[+] sample #{len(self._samples)}  EE=({x:.4f}, {y:.4f}, {z:.4f})"
        )

        if self._model and self._last_target is not None:
            q_cur = _q_from_js(self._model, js)
            _print_error_table(self._last_target, _get_active_vals(self._model, q_cur))

        return True

    def save(self):
        if not self._samples:
            print("No samples to save.")
            return
        os.makedirs(os.path.dirname(os.path.abspath(self._output_path)), exist_ok=True)
        fieldnames = ["x1", "y1", "z1"] + ACTIVE_JOINTS
        with open(self._output_path, "w", newline="") as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(self._samples)
        print(f"\nSaved {len(self._samples)} samples → {self._output_path}")


# ── Main ─────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Automatic calibration data collection for Tiago Pro."
    )
    parser.add_argument("--configs",  default=str(_CONFIGS_DEFAULT))
    parser.add_argument("--output",   default=str(_OUTPUT_DEFAULT))
    parser.add_argument("--urdf",     default=None,
                        help="Path to robot URDF for Viser display (optional)")
    parser.add_argument("--duration", type=float, default=5.0,
                        help="Trajectory duration per config in seconds (default: 5)")
    parser.add_argument("--no-viser", action="store_true")
    parsed, ros_args = parser.parse_known_args()

    with open(parsed.configs) as f:
        cfg_data = yaml.safe_load(f)
    all_configs = cfg_data["calibration_joint_configurations"]
    print(f"Loaded {len(all_configs)} optimal configurations.")

    model, visual_model, collision_model, viser = None, None, None, None

    if not parsed.no_viser:
        try:
            if parsed.urdf:
                model, collision_model, visual_model = pin.buildModelsFromUrdf(parsed.urdf)
            else:
                from ament_index_python.packages import get_package_share_directory
                import xacro, tempfile
                tiago_desc = Path(get_package_share_directory("tiago_pro_description"))
                urdf_str = xacro.process_file(
                    str(tiago_desc / "robots" / "tiago_pro.urdf.xacro"),
                    mappings={
                        "end_effector_right": "pal-atc",
                        "end_effector_left":  "pal-pro-gripper",
                        "wrist_model_right":  "spherical-wrist",
                    }
                ).toxml()
                with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="w") as f_:
                    f_.write(urdf_str); tmp = f_.name
                model, collision_model, visual_model = pin.buildModelsFromUrdf(
                    tmp, package_dirs=[str(tiago_desc.parent.parent)]
                )
                os.unlink(tmp)
            viser = ViserDisplay(model, visual_model, collision_model)
            print("Viser started.")
        except Exception as e:
            print(f"Viser init failed ({e}) — continuing without display.")
            model, viser = None, None

    rclpy.init(args=ros_args)
    node = CalibrationCollector(parsed.output, viser, model, parsed.duration)
    node._wait_for_servers()

    spin_done = threading.Event()
    def _spin():
        while not spin_done.is_set():
            rclpy.spin_once(node, timeout_sec=0.02)
    threading.Thread(target=_spin, daemon=True).start()

    time.sleep(1.0)

    print("\nStarting automated collection. Ctrl+C to abort.\n")

    try:
        idx = 0
        while idx < len(all_configs):
            target_vals = all_configs[idx]

            print(f"\n{'='*60}")
            print(f"Configuration {idx+1}/{len(all_configs)}")
            print(f"{'='*60}")
            for jname, val in zip(ACTIVE_JOINTS, target_vals):
                print(f"  {jname:<30}  {np.degrees(val):>+8.2f}°")

            if viser and model:
                viser.update_target(_q_from_active(model, target_vals))
            node._last_target = target_vals

            ok = node.move_to(target_vals)
            if not ok:
                print("  [warn] motion failed — skipping.")
                idx += 1
                continue

            recorded = node.wait_and_record()
            if recorded:
                idx += 1
            else:
                print("  [warn] recording failed — skipping.")
                idx += 1

    except KeyboardInterrupt:
        pass

    spin_done.set()
    node.save()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
