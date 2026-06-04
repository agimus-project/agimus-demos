#!/usr/bin/env python3
"""
Mocap vs FK calibration analysis.

Records pairs (T_FK, T_mocap) at multiple robot configurations and analyses
the consistency of  δT = T_FK⁻¹ · T_mocap  to determine whether the error
is a fixed marker offset (constant δT) or a FK / base-frame error (variable δT).

Usage (IPython):
    %run scripts/calibrate_mocap_fk.py
    c = Calibrator()
    c.connect_mocap()

    # Move robot to config 1, wait for it to settle, then:
    c.record()
    # Move robot to config 2 ...
    c.record()
    # (repeat for ~10–15 well-spread configurations)

    c.analyze()           # prints diagnosis + T_correction candidate
    c.save("results/run1.npz")

Offline re-analysis:
    c = Calibrator.load("results/run1.npz")
    c.analyze()
"""

import os
import sys
import glob
import time
import tempfile

import numpy as np
import pinocchio as pin

# ── Make ROS 2 packages importable from HPP environment ──────────────────────
for _p in sorted(
    glob.glob("/home/gepetto/ros2_ws/install/*/lib/python3*/site-packages")
    + glob.glob("/home/gepetto/agimus_deps_ws/install/*/lib/python3*/site-packages")
):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

# ── Constants ─────────────────────────────────────────────────────────────────
_SCRIPTS_DIR   = os.path.dirname(os.path.abspath(__file__))
_PKG_DIR       = os.path.join(_SCRIPTS_DIR, "..")
_QUALISYS_IP   = "140.93.1.100"
_MOCAP_BODIES  = {"pylone": 0, "tiago_endEffector": 2, "tiago_base": 1}
_MOCAP_BASE_IDX = 2   # tiago_base = reference frame
_EE_IDX         = 1   # tiago_endEffector local index
_EE_FRAME_KW   = "gripper_right_tool_holder"


# ── Helpers ───────────────────────────────────────────────────────────────────

def _rotation_mean(rotations: np.ndarray) -> np.ndarray:
    """Chordal L2 mean of a set of (3×3) rotation matrices (SVD method)."""
    R_sum = rotations.sum(axis=0)
    U, _, Vt = np.linalg.svd(R_sum)
    R_mean = U @ Vt
    if np.linalg.det(R_mean) < 0:   # fix reflection
        U[:, -1] *= -1
        R_mean = U @ Vt
    return R_mean


def _se3_to_mat4(T: pin.SE3) -> np.ndarray:
    mat = np.eye(4)
    mat[:3, :3] = T.rotation
    mat[:3,  3] = T.translation
    return mat


def _mat4_to_se3(mat: np.ndarray) -> pin.SE3:
    return pin.SE3(mat[:3, :3], mat[:3, 3])


# ── Main class ────────────────────────────────────────────────────────────────

class Calibrator:
    """
    Interactive mocap-vs-FK calibration tool.

    Attributes
    ----------
    measurements : list of (T_fk, T_mocap, T_delta)
        All recorded SE3 triples (pinocchio SE3 objects).
    T_correction : pin.SE3
        Mean δT = T_FK⁻¹ · T_mocap (available after analyze()).
    """

    def __init__(self):
        if not rclpy.ok():
            rclpy.init()
        self._node = rclpy.create_node("mocap_fk_calibrator")
        self.measurements: list = []
        self._qc = None
        self.T_correction: pin.SE3 | None = None

        print("Loading robot model from /robot_description …")
        self._load_model()
        ee_name = self._model.frames[self._ee_frame_id].name
        print(f"  EE frame : '{ee_name}'")
        print(f"  nq       : {self._model.nq}")
        print("\nCalibrator ready.")
        print("  connect_mocap()  →  move robot  →  record()  (repeat)")
        print("  analyze()  →  save('results/run.npz')\n")

    # ── Model loading ─────────────────────────────────────────────────────────

    def _load_model(self, timeout: float = 10.0) -> None:
        """Fetch URDF from /robot_description and build pinocchio model."""
        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        urdf_str = [None]
        sub = self._node.create_subscription(
            String, "/robot_description",
            lambda m: urdf_str.__setitem__(0, m.data), qos,
        )
        t0 = time.time()
        while urdf_str[0] is None and time.time() - t0 < timeout:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        self._node.destroy_subscription(sub)

        if urdf_str[0] is None:
            raise RuntimeError(
                f"Timeout after {timeout}s waiting for /robot_description. "
                "Is the robot running?"
            )

        with tempfile.NamedTemporaryFile(suffix=".urdf", delete=False, mode="w") as f:
            f.write(urdf_str[0])
            tmp_path = f.name

        self._model = pin.buildModelFromUrdf(tmp_path)
        os.unlink(tmp_path)
        self._data = self._model.createData()

        # Find EE frame (search for keyword, tolerant to tiago_pro/ prefix)
        self._ee_frame_id = None
        for i, frame in enumerate(self._model.frames):
            if _EE_FRAME_KW in frame.name:
                self._ee_frame_id = i
                break
        if self._ee_frame_id is None:
            raise RuntimeError(
                f"Frame containing '{_EE_FRAME_KW}' not found in model. "
                f"Available frames: {[f.name for f in self._model.frames]}"
            )

    # ── Mocap ─────────────────────────────────────────────────────────────────

    def connect_mocap(self, ip: str = _QUALISYS_IP) -> None:
        """Start the Qualisys mocap subprocess."""
        sys.path.insert(0, _SCRIPTS_DIR)
        from qualisys import QualisysClient  # noqa: PLC0415
        self._qc = QualisysClient(ip=ip, bodies=_MOCAP_BODIES)
        time.sleep(1.0)
        print(f"Mocap connected to {ip}.")

    def disconnect_mocap(self) -> None:
        if self._qc is not None:
            self._qc.stop()
            self._qc = None
            print("Mocap disconnected.")

    def _mocap_se3_bulk(self) -> tuple:
        """Return (positions, quats) as two bulk snapshots to reduce race conditions."""
        return self._qc.getPositions(), self._qc.getOrientationQuats()

    def _mocap_se3(self, idx: int) -> pin.SE3:
        pos  = self._qc.getPositions()[idx]
        quat = self._qc.getOrientationQuats()[idx]
        return pin.XYZQUATToSE3(np.concatenate([pos, quat]))

    # ── FK ────────────────────────────────────────────────────────────────────

    def _read_joint_states(self, timeout: float = 5.0) -> dict:
        js = [None]
        sub = self._node.create_subscription(
            JointState, "/joint_states",
            lambda m: js.__setitem__(0, m), 10,
        )
        t0 = time.time()
        while js[0] is None and time.time() - t0 < timeout:
            rclpy.spin_once(self._node, timeout_sec=0.1)
        self._node.destroy_subscription(sub)
        if js[0] is None:
            raise RuntimeError("Timeout reading /joint_states.")
        return dict(zip(js[0].name, js[0].position))

    def _fk_ee(self, js_map: dict) -> pin.SE3:
        """Forward kinematics at EE frame from a joint-state dict."""
        q = pin.neutral(self._model)
        for jid in range(1, self._model.njoints):
            jname = self._model.names[jid]
            # Try exact name, then stripped of "tiago_pro/" prefix
            val = js_map.get(jname) or js_map.get(jname.replace("tiago_pro/", ""))
            if val is not None:
                nq = self._model.joints[jid].nq
                if nq == 1:   # revolute / prismatic
                    q[self._model.joints[jid].idx_q] = val
        pin.forwardKinematics(self._model, self._data, q)
        pin.updateFramePlacements(self._model, self._data)
        return self._data.oMf[self._ee_frame_id].copy()

    # ── Recording ─────────────────────────────────────────────────────────────

    def record(self) -> None:
        """
        Capture one (T_FK, T_mocap, δT) measurement at the current configuration.

        Both poses are expressed relative to the robot base frame:
          - T_FK   : pinocchio FK of gripper_right_tool_holder
          - T_mocap: tiago_endEffector relative to tiago_base (mocap)
          - δT     : T_FK⁻¹ · T_mocap
        """
        if self._qc is None:
            print("No mocap — call connect_mocap() first.")
            return

        js_map  = self._read_joint_states()
        T_fk    = self._fk_ee(js_map)

        positions, quats = self._mocap_se3_bulk()
        T_base  = pin.XYZQUATToSE3(np.concatenate([positions[_MOCAP_BASE_IDX], quats[_MOCAP_BASE_IDX]]))
        T_mocap = T_base.inverse() * pin.XYZQUATToSE3(np.concatenate([positions[_EE_IDX], quats[_EE_IDX]]))

        T_delta = T_fk.inverse() * T_mocap
        self.measurements.append((T_fk.copy(), T_mocap.copy(), T_delta.copy()))

        n          = len(self.measurements)
        t_err_mm   = np.linalg.norm(T_delta.translation) * 1e3
        r_err_deg  = np.degrees(np.linalg.norm(pin.log3(T_delta.rotation)))
        print(f"  [{n:2d}]  |δt| = {t_err_mm:6.1f} mm   |δR| = {r_err_deg:.2f}°")

    # ── Analysis ──────────────────────────────────────────────────────────────

    def analyze(self) -> tuple[float, float]:
        """
        Analyse the dispersion of δT across all recorded configurations.

        Prints:
          - Mean δT (translation + rotation) → T_correction candidate
          - Config-to-config standard deviation → diagnosis

        Returns
        -------
        (t_std_mm, r_std_deg) : translation and rotation dispersion.
        """
        n = len(self.measurements)
        if n < 2:
            print("Need at least 2 measurements to analyse.")
            return None

        deltas = [m[2] for m in self.measurements]

        # ── Translation ────────────────────────────────────────────────────
        translations = np.array([d.translation for d in deltas])
        t_mean       = translations.mean(axis=0)
        t_std_mm     = (np.linalg.norm(translations - t_mean, axis=1) * 1e3).std()
        t_mean_mm    = np.linalg.norm(t_mean) * 1e3

        # ── Rotation ───────────────────────────────────────────────────────
        rotations  = np.array([d.rotation for d in deltas])
        R_mean     = _rotation_mean(rotations)
        r_errs_deg = np.array([
            np.degrees(np.linalg.norm(pin.log3(R_mean.T @ R)))
            for R in rotations
        ])
        r_std_deg  = r_errs_deg.std()
        r_mean_deg = np.degrees(np.linalg.norm(pin.log3(R_mean)))
        rpy_mean   = np.degrees(pin.rpy.matrixToRpy(R_mean))
        q_mean     = pin.Quaternion(R_mean)

        # Store T_correction for later use
        self.T_correction = pin.SE3(R_mean, t_mean)

        # ── Report ─────────────────────────────────────────────────────────
        print(f"\n{'='*64}")
        print(f"  Mocap vs FK calibration — {n} configurations")
        print(f"{'='*64}")

        print(f"\n  Mean δT  =  T_FK⁻¹ · T_mocap  (marker offset candidate)")
        print(f"    Translation  : {np.round(t_mean * 1e3, 2)} mm   |t| = {t_mean_mm:.1f} mm")
        print(f"    Rotation     : |δR| = {r_mean_deg:.2f}°")
        print(f"    RPY  [°]     : {np.round(rpy_mean, 2)}")
        print(f"    Quaternion   : xyzw = "
              f"{np.round([q_mean.x, q_mean.y, q_mean.z, q_mean.w], 4)}")

        print(f"\n  Config-to-config dispersion")
        print(f"    Translation σ : {t_std_mm:.2f} mm")
        print(f"    Rotation σ    : {r_std_deg:.3f}°")

        print(f"\n  Per-measurement errors:")
        print(f"    {'#':>3}  {'|δt| mm':>9}  {'|δR| °':>8}  {'σ_t mm':>8}  {'σ_R °':>7}")
        for i, d in enumerate(deltas):
            t_mm  = np.linalg.norm(d.translation) * 1e3
            r_deg = np.degrees(np.linalg.norm(pin.log3(d.rotation)))
            dt_mm = np.linalg.norm((d.translation - t_mean)) * 1e3
            dr_deg = r_errs_deg[i]
            print(f"    {i+1:>3}  {t_mm:>9.1f}  {r_deg:>8.2f}  {dt_mm:>8.2f}  {dr_deg:>7.3f}")

        print(f"\n  Diagnosis")
        if t_std_mm < 3.0 and r_std_deg < 1.0:
            print(f"    ✓  δT is CONSTANT across configurations.")
            print(f"       → Error is a fixed EE marker offset.")
            print(f"       → FK is reliable; use T_correction = c.T_correction once.")
        elif t_std_mm < 10.0 and r_std_deg < 3.0:
            print(f"    ~  δT is MOSTLY CONSTANT (small variation).")
            print(f"       → Marker offset dominates, small FK residuals present.")
            print(f"       → T_correction valid; accuracy limited to ~{t_std_mm:.0f} mm.")
        else:
            print(f"    !  δT VARIES SIGNIFICANTLY across configurations.")
            print(f"       → FK errors or base-frame misalignment are significant.")
            print(f"       → T_correction is only locally valid.")
        print(f"{'='*64}\n")

        return t_std_mm, r_std_deg

    # ── Persistence ───────────────────────────────────────────────────────────

    def save(self, path: str) -> None:
        """Save all measurements to a .npz file."""
        os.makedirs(os.path.dirname(os.path.abspath(path)) or ".", exist_ok=True)
        np.savez(
            path,
            T_fk    = np.array([_se3_to_mat4(m[0]) for m in self.measurements]),
            T_mocap = np.array([_se3_to_mat4(m[1]) for m in self.measurements]),
            T_delta = np.array([_se3_to_mat4(m[2]) for m in self.measurements]),
        )
        print(f"Saved {len(self.measurements)} measurements → {path}")

    @classmethod
    def load(cls, path: str) -> "Calibrator":
        """Load saved measurements for offline analysis (no ROS / mocap needed)."""
        data = np.load(path)
        obj = object.__new__(cls)
        obj._qc = None
        obj.T_correction = None
        obj.measurements = [
            (_mat4_to_se3(fk), _mat4_to_se3(mc), _mat4_to_se3(dt))
            for fk, mc, dt in zip(data["T_fk"], data["T_mocap"], data["T_delta"])
        ]
        print(f"Loaded {len(obj.measurements)} measurements from {path}")
        return obj
