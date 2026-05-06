from math import sqrt
from pathlib import Path

import numpy as np
import pinocchio as pin
import yaml

from pyhpp.manipulation import Device, Graph, Problem
from pyhpp.manipulation import urdf
from pyhpp.constraints import ComparisonType, ComparisonTypes, LockedJoint

from agimus_demo_10_tiago_pro_bar_manip.planner.path_planner import PathPlanner


class BaseObject:
    """Wrapper to describe a URDF/SRDF object for HPP."""

    def __init__(self, root_joint_type: str, urdf_path: str, srdf_path: str, name: str):
        self.rootJointType = root_joint_type
        self.urdfFilename = urdf_path
        self.srdfFilename = srdf_path
        self.name = name


class HPPPathGenerator:
    def __init__(
        self,
        urdf_str: str,
        srdf_str: str,
        robot_model: pin.Model,
        handle_config: Path,
        handle_object: BaseObject,
        plate_object: BaseObject,
        table_object: BaseObject,
        ocp_dt: float,
        robot_name: str,
        logger,
    ) -> None:
        self._ocp_dt = ocp_dt
        self._robot_name = robot_name
        self._logger = logger
        self._robot_model = robot_model

        self._handle_object = handle_object
        self._plate_object = plate_object
        self._table_object = table_object

        with open(handle_config, "r") as f:
            self._handle_config_file = yaml.safe_load(f)
        self._handles_config: dict[str, list] = {}
        for gname, config in self._handle_config_file.get("grippers", {}).items():
            self._handles_config[gname] = config.get("handles", [])

        # == HPP Device ========================================================
        robot = Device(f"{robot_name}-manip")
        urdf.loadModelFromString(
            robot,
            0,
            robot_name,
            "planar",
            urdf_str,
            srdf_str,
            pin.SE3.Identity(),
        )
        for obj in (handle_object, plate_object, table_object):
            urdf.loadModel(
                robot,
                0,
                obj.name,
                obj.rootJointType,
                obj.urdfFilename,
                obj.srdfFilename,
                pin.SE3.Identity(),
            )

        self.robot = robot
        self._pin_model = robot.model()

        # == Joint bounds ======================================================
        self.robot.setJointBounds(
            f"{handle_object.name}/root_joint",
            [-5.0, 5.0, -5.0, 5.0, 0.0, 2.0, -1, 1, -1, 1, -1, 1, -1, 1],
        )
        self.robot.setJointBounds(
            f"{plate_object.name}/root_joint",
            [-5.0, 5.0, -5.0, 5.0, 0.0, 2.0, -1, 1, -1, 1, -1, 1, -1, 1],
        )
        self.robot.setJointBounds(
            f"{robot_name}/root_joint",
            [-5.0, 5.0, -5.0, 5.0, -1, 1, -1, 1],
        )

        def _set_vel_bounds(joint_name, bounds):
            jid = self._pin_model.getJointId(joint_name)
            idx_v = self._pin_model.joints[jid].idx_v
            nv = self._pin_model.joints[jid].nv
            assert len(bounds) == nv, (
                f"{joint_name}: expected {nv} bounds, got {len(bounds)}"
            )
            for i, v in enumerate(bounds):
                self._pin_model.velocityLimit[idx_v + i] = v

        # Base (PlanarJoint) — vx, vy, ω
        _set_vel_bounds(f"{robot_name}/root_joint", [0.5, 0.5, 1.0])

        # Objects freeflyer — vx, vy, vz, ωx, ωy, ωz
        for obj in (handle_object, plate_object):
            _set_vel_bounds(f"{obj.name}/root_joint", [1.0, 1.0, 1.0, 2.0, 2.0, 2.0])

        # == Problem & Graph ===================================================
        # ORDER IS CRITICAL — see comments in _generate_constraint_graph
        self._problem = Problem(self.robot)
        self.graph = Graph("robot", self.robot, self._problem)
        self._problem.constraintGraph(self.graph)

        self.graph.errorThreshold(1e-2)
        self.graph.maxIterations(500)

        # == Locked joints =====================================================
        self._locked_grippers = self._create_locked_grippers(robot_name)
        self._locked_head = self._create_locked_head(robot_name)
        self._locked_wheels = self._create_locked_wheels(robot_name)
        self._locked_plate = self._create_locked_plate()
        self._locked_base_mobility = self._create_locked_base_mobility(robot_name)
        self._locked_arms, self._locked_torso = self._create_locked_arms_and_torso(
            robot_name
        )
        self._locked_handle = self._locked_handle()

        # == Grippers & handles ================================================
        c = sqrt(2) / 2
        pose_gripper = pin.XYZQUATToSE3(
            np.array([0, 0, 0.19, 0, -c, 0, c], dtype=float)
        )
        pose_hd_left = pin.XYZQUATToSE3(
            np.array([0, 0.01, -0.25, 0, 0, -c, c], dtype=float)
        )
        pose_hd_right = pin.XYZQUATToSE3(
            np.array([0, 0.01, 0.25, 0, 0, -c, c], dtype=float)
        )
        self.define_gripper(
            self.robot,
            f"{robot_name}/arm_left_7_link",
            f"{robot_name}/left",
            pose_gripper,
        )
        self.define_gripper(
            self.robot,
            f"{robot_name}/arm_right_7_link",
            f"{robot_name}/right",
            pose_gripper,
        )
        self.define_handle(
            self.robot,
            f"{handle_object.name}/bar_base_link",
            f"{handle_object.name}/left",
            pose_hd_left,
        )
        self.define_handle(
            self.robot,
            f"{handle_object.name}/bar_base_link",
            f"{handle_object.name}/right",
            pose_hd_right,
        )

        # == Build constraint graph (ends with graph.initialize()) =============
        self._generate_constraint_graph()

        self._logger.info("Constraint graph built and initialized.")

        self.init_viewer()
        # PathPlanner MUST come after graph.initialize()
        self._path_planner = PathPlanner(self._problem, self.graph, self.robot)

    # == Locked joint helpers ==================================================

    def _make_cts(self, n: int) -> ComparisonTypes:
        cts = ComparisonTypes()
        cts[:] = [ComparisonType.EqualToZero] * n
        return cts

    def _lock(self, joint_name: str, value: list) -> LockedJoint:
        val = np.array(value, dtype=float)
        return LockedJoint(self.robot, joint_name, val, self._make_cts(len(val)))

    def _create_locked_grippers(self, robot_name):
        locked = []
        values = {
            f"{robot_name}/gripper_left_finger_joint": 0.05,
            f"{robot_name}/gripper_left_inner_finger_left_joint": -0.05,
            f"{robot_name}/gripper_left_fingertip_left_joint": 0.05,
            f"{robot_name}/gripper_left_finger_right_joint": 0.0,
            f"{robot_name}/gripper_left_inner_finger_right_joint": -0.05,
            f"{robot_name}/gripper_left_fingertip_right_joint": 0.05,
            f"{robot_name}/gripper_left_outer_finger_left_joint": -0.05,
            f"{robot_name}/gripper_left_outer_finger_right_joint": -0.05,
            f"{robot_name}/gripper_right_finger_joint": 0.05,
            f"{robot_name}/gripper_right_inner_finger_left_joint": -0.05,
            f"{robot_name}/gripper_right_fingertip_left_joint": 0.05,
            f"{robot_name}/gripper_right_finger_right_joint": 0.0,
            f"{robot_name}/gripper_right_inner_finger_right_joint": -0.05,
            f"{robot_name}/gripper_right_fingertip_right_joint": 0.05,
            f"{robot_name}/gripper_right_outer_finger_left_joint": -0.05,
            f"{robot_name}/gripper_right_outer_finger_right_joint": -0.05,
        }
        for joint, value in values.items():
            lj = self._lock(joint, [value])
            self._problem.setConstantRightHandSide(lj, False)
            locked.append(lj)
        return locked

    def _create_locked_head(self, robot_name):
        locked = []
        for joint, value in {
            f"{robot_name}/head_1_joint": 0.0,
            f"{robot_name}/head_2_joint": 0.0,
        }.items():
            lj = self._lock(joint, [value])
            self._problem.setConstantRightHandSide(lj, True)
            locked.append(lj)
        return locked

    def _create_locked_wheels(self, robot_name: str) -> list:
        locked = []
        for joint in [
            f"{robot_name}/wheel_front_left_joint",
            f"{robot_name}/wheel_front_right_joint",
            f"{robot_name}/wheel_rear_left_joint",
            f"{robot_name}/wheel_rear_right_joint",
        ]:
            lj = self._lock(joint, [1.0, 0.0])
            self._problem.setConstantRightHandSide(lj, True)
            locked.append(lj)
        return locked

    def _create_locked_plate(self):
        lj = self._lock(
            f"{self._plate_object.name}/root_joint",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        self._problem.setConstantRightHandSide(lj, False)
        return [lj]

    def _create_locked_base_mobility(self, robot_name):
        lj = self._lock(f"{robot_name}/root_joint", [0.0, 0.0, 1.0, 0.0])
        self._problem.setConstantRightHandSide(lj, False)
        return [lj]

    def _create_locked_arms_and_torso(self, robot_name: str) -> tuple[list, list]:
        locked_arms, seen = [], set()
        for j in self.robot.model().names:
            if not j.startswith(f"{robot_name}/"):
                continue
            if (
                j.startswith(f"{robot_name}/head")
                or j.startswith(f"{robot_name}/wheel")
                or "gripper" in j
                or j == f"{robot_name}/torso_lift_joint"
                or j == f"{robot_name}/root_joint"
            ):
                continue
            if j in seen:
                continue
            seen.add(j)
            lj = self._lock(j, [0.0])
            self._problem.setConstantRightHandSide(lj, False)
            locked_arms.append(lj)
        lj_torso = self._lock(f"{robot_name}/torso_lift_joint", [0.0])
        self._problem.setConstantRightHandSide(lj_torso, False)
        return locked_arms, [lj_torso]

    def _locked_handle(self):
        lj = self._lock(
            f"{self._handle_object.name}/root_joint",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        self._problem.setConstantRightHandSide(lj, False)
        return lj

    def _locked_handle_orientation(self):
        lj = self._lock(
            f"{self._handle_object.name}/root_joint",
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        )
        self._problem.setConstantRightHandSide(lj, False)
        return lj

    # == Constraint graph (manual — no factory) ================================

    def _generate_constraint_graph(self) -> None:
        """
        Build the constraint graph manually, replicating exactly what
        ConstraintGraphFactory.generate() would produce
        -------------------
        Structure generated
        -------------------
        States (+ waypoint states):
            free       : place on manifold,  place/complement on foliation
            grasp      : grasp on manifold,  grasp/complement on foliation
            _pregrasp  : place + pregrasp    (waypoint)
            _intersec  : place + grasp       (waypoint)
            _preplace  : preplace + grasp    (waypoint)

        Loop transitions:
            "Loop | f"   : free->free,   foliation=place/complement
            "Loop | 0-0" : grasp->grasp, foliation=grasp/complement

        Waypoint transitions (3 waypoints -> 4 sub-transitions each):
            "g > h | f"   forward:  free->pregrasp->intersec->preplace->grasp
            "g < h | 0-0" backward: grasp->preplace->intersec->pregrasp->free

        Sub-transition foliations:
            _01, _12 -> place/complement  (_12 also gets grasp/complement)
            _23, _34 -> grasp/complement

        Exposed constraint references (for RHS control from path_planner):
            self._place_c       : place constraint (bar on surface)
            self._place_comp_c  : place/complement — fix its RHS from TF to
                                  prevent HPP from projecting the bar pose
            self._grasp_c       : grasp constraint
            self._grasp_comp_c  : grasp/complement
        """
        obj_name = self._handle_object.name
        table_name = self._table_object.name
        plate_name = self._plate_object.name

        gripper_names = list(self._handles_config.keys())
        primary_gripper = gripper_names[0]
        primary_handle = self._handles_config[primary_gripper][0]["name"]

        prefix_fwd = f"{primary_gripper} > {primary_handle} | f"
        prefix_bwd = f"{primary_gripper} < {primary_handle} | 0-0"
        grasp_name = f"{primary_gripper} grasps {primary_handle}"

        # == 1. Grasp constraints ==============================================
        # createGraspConstraint -> (grasp, grasp/complement, grasp/hold)
        grasp_c, grasp_comp_c, grasp_hold_c = self.graph.createGraspConstraint(
            grasp_name, primary_gripper, primary_handle
        )
        pregrasp_c = self.graph.createPreGraspConstraint(
            f"{primary_gripper} pregrasps {primary_handle}",
            primary_gripper,
            primary_handle,
        )

        # == 2. Placement constraints ==========================================
        # createPlacementConstraint -> (place, place/complement, place/hold)
        # place/complement RHS is what HPP modifies when projecting onto the
        # placement manifold. Expose it so callers can fix it from TF poses.
        place_c, place_comp_c, place_hold_c = self.graph.createPlacementConstraint(
            f"place_{obj_name}",
            [f"{obj_name}/bottom"],
            [
                f"{table_name}/reinforcment_bar_support",
                f"{plate_name}/bottom",
            ],
        )
        preplace_c = self.graph.createPrePlacementConstraint(
            f"preplace_{obj_name}",
            [f"{obj_name}/bottom"],
            [
                f"{table_name}/reinforcment_bar_support",
                f"{plate_name}/top",
            ],
            0.1,  # width (approach distance)
        )

        # Keep references for external RHS manipulation in path_planner
        self._place_c = place_c
        self._place_comp_c = place_comp_c
        self._grasp_c = grasp_c
        self._grasp_comp_c = grasp_comp_c

        # == 3. States =========================================================
        s_free = self.graph.createState("free", False, 0)
        s_grasp = self.graph.createState(grasp_name, False, 1)
        s_pregrasp = self.graph.createState(prefix_fwd + "_pregrasp", True, 0)
        s_intersec = self.graph.createState(prefix_fwd + "_intersec", True, 0)
        s_preplace = self.graph.createState(prefix_fwd + "_preplace", True, 0)

        # Manifold constraints on states
        # self.graph.addNumericalConstraintsToState(s_free,     [self._locked_handle])
        self.graph.addNumericalConstraintsToState(s_grasp, [grasp_c])
        self.graph.addNumericalConstraintsToState(s_pregrasp, [pregrasp_c])
        self.graph.addNumericalConstraintsToState(s_intersec, [grasp_c])
        self.graph.addNumericalConstraintsToState(s_preplace, [preplace_c, grasp_c])

        # == 4. Loop transitions ===============================================
        loop_f = self.graph.createTransition(s_free, s_free, "Loop | f", 0, s_free)
        loop_g = self.graph.createTransition(s_grasp, s_grasp, "Loop | 0-0", 0, s_grasp)
        self.graph.addNumericalConstraintsToTransition(loop_f, [self._locked_handle])
        self.graph.addNumericalConstraintsToTransition(loop_g, [grasp_comp_c])

        # == 5. Waypoint transitions ===========================================
        # 3 waypoints -> nbWaypoints=3, automaticBuilder=False (we wire manually)
        fwd_edge = self.graph.createWaypointTransition(
            s_free, s_grasp, prefix_fwd, 3, 1, s_free, False
        )
        bwd_edge = self.graph.createWaypointTransition(
            s_grasp, s_free, prefix_bwd, 3, 1, s_free, False
        )

        # == 6. Sub-transitions ================================================
        # Forward path: free(0) -> pregrasp(1) -> intersec(2) -> preplace(3) -> grasp(4)
        # Backward:     grasp(4) -> preplace(3) -> intersec(2) -> pregrasp(1) -> free(0)
        # Sub-transition names: fwd _01…_34, bwd _43…_10
        fwd_states = [s_free, s_pregrasp, s_intersec, s_preplace, s_grasp]

        fwd_subs = []
        bwd_subs = []
        for i in range(4):
            nf = f"{prefix_fwd}_{i}{i + 1}"
            nb = f"{prefix_bwd}_{i + 1}{i}"
            # All sub-transitions initially use the containing state of
            # their source — corrected below per factory logic.
            tf = self.graph.createTransition(
                fwd_states[i], fwd_states[i + 1], nf, -1, fwd_states[i]
            )
            tb = self.graph.createTransition(
                fwd_states[i + 1], fwd_states[i], nb, -1, fwd_states[i]
            )
            fwd_subs.append(tf)
            bwd_subs.append(tb)
            self.graph.setWaypoint(fwd_edge, i, tf, fwd_states[i + 1])
            self.graph.setWaypoint(bwd_edge, 3 - i, tb, fwd_states[i])

        # Mark short edges (constrained direct paths, not explored by RRT)
        # Factory marks all but the first forward and last backward as short.
        for i in range(1, 4):
            self.graph.setShort(fwd_subs[i], True)
        for i in range(0, 3):
            self.graph.setShort(bwd_subs[i], True)

        # Containing states:
        # M = 1 (pregrasp) + 1 = 2  ->  first 2 sub-trans belong to free,
        #                               last 2 belong to grasp.
        for tf, tb in [(fwd_subs[0], bwd_subs[0]), (fwd_subs[1], bwd_subs[1])]:
            self.graph.setContainingNode(tf, s_free)
            self.graph.setContainingNode(tb, s_free)
        for tf, tb in [(fwd_subs[2], bwd_subs[2]), (fwd_subs[3], bwd_subs[3])]:
            self.graph.setContainingNode(tf, s_grasp)
            self.graph.setContainingNode(tb, s_grasp)

        # Foliations on sub-transitions (parametric complements):
        #   _01        -> place/complement
        #   _12        -> place/complement + grasp/complement
        #   _23, _34   -> grasp/complement
        self.graph.addNumericalConstraintsToTransition(
            fwd_subs[0], [self._locked_handle]
        )
        self.graph.addNumericalConstraintsToTransition(bwd_subs[0], [place_comp_c])

        self.graph.addNumericalConstraintsToTransition(
            fwd_subs[1], [self._locked_handle, grasp_comp_c]
        )
        self.graph.addNumericalConstraintsToTransition(
            bwd_subs[1], [place_comp_c, grasp_comp_c]
        )

        self.graph.addNumericalConstraintsToTransition(fwd_subs[2], [grasp_comp_c])
        self.graph.addNumericalConstraintsToTransition(bwd_subs[2], [grasp_comp_c])

        self.graph.addNumericalConstraintsToTransition(fwd_subs[3], [grasp_comp_c])
        self.graph.addNumericalConstraintsToTransition(bwd_subs[3], [grasp_comp_c])

        # == 7. Bimanual secondary gripper =========================
        if len(gripper_names) == 2:
            secondary_gripper = gripper_names[1]
            secondary_handle = self._handles_config[secondary_gripper][0]["name"]
            sec_grasp_list = self.graph.createGraspConstraint(
                f"{secondary_gripper} grasps {secondary_handle}",
                secondary_gripper,
                secondary_handle,
            )
            sec_grasp_obj = sec_grasp_list[0]
            sec_pregrasp_obj = self.graph.createPreGraspConstraint(
                f"{secondary_gripper} pregrasps {secondary_handle}",
                secondary_gripper,
                secondary_handle,
            )
            for node_name, c_obj in [
                (prefix_fwd + "_pregrasp", sec_pregrasp_obj),
                (prefix_fwd + "_intersec", sec_grasp_obj),
                (prefix_fwd + "_preplace", sec_grasp_obj),
                (grasp_name, sec_grasp_obj),
            ]:
                self.graph.addNumericalConstraintsToState(
                    self.graph.getState(node_name), [c_obj]
                )

        # == 8. Global locked joints ===========================================
        self.graph.addNumericalConstraintsToGraph(
            self._locked_grippers + self._locked_head + self._locked_wheels
        )

        # == 9. Custom planning transitions ====================================
        state_unconstrained = self.graph.createState("unconstrained", False, 0)
        self.graph.createTransition(
            state_unconstrained, s_free, "project-on-free", 1, state_unconstrained
        )

        t_transit_free = self.graph.createTransition(
            s_free, s_free, "transit_free", 1, s_free
        )
        self.graph.addNumericalConstraintsToTransition(
            t_transit_free,
            self._locked_arms + [self._locked_handle],
        )

        t_transit_grasp = self.graph.createTransition(
            s_grasp, s_grasp, "transit_grasp", 1, s_grasp
        )
        self.graph.addNumericalConstraintsToTransition(
            t_transit_grasp, self._locked_arms
        )

        # locked_base_mobility on grasp/approach sub-transitions and loops
        for edge_name in [
            f"{prefix_fwd}_01",
            f"{prefix_fwd}_12",
            f"{prefix_fwd}_23",
            f"{prefix_bwd}_32",
            f"{prefix_bwd}_21",
            f"{prefix_bwd}_10",
            "Loop | f",
            "Loop | 0-0",
        ]:
            self.graph.addNumericalConstraintsToTransition(
                self.graph.getTransition(edge_name), self._locked_base_mobility
            )

        self.graph.setWeight(self.graph.getTransition("Loop | f"), 1)
        self.graph.setWeight(self.graph.getTransition("Loop | 0-0"), 1)

        # locked_plate on every transition
        for e_name in self.graph.getTransitionNames():
            self.graph.addNumericalConstraintsToTransition(
                self.graph.getTransition(e_name), self._locked_plate
            )

        self.graph.initialize()

    # == Public helpers ========================================================

    def define_gripper(
        self,
        robot,
        frame_name: str,
        gripper_name: str,
        pose: pin.SE3,
        clearance: float = 0.02,
    ):
        robot.addGripper(frame_name, gripper_name, pose, clearance)

    def define_handle(
        self,
        robot,
        link_name: str,
        handle_name: str,
        pose: pin.SE3,
        clearance: float = 0.05,
    ):
        robot.addHandle(link_name, handle_name, pose, clearance, 6 * [True])

    def sample_trajectory(self, path_obj) -> list | None:
        if path_obj is None:
            return None
        tr = path_obj.timeRange()
        t_min, t_max = tr.first, tr.second
        times = np.arange(t_min, t_max, self._ocp_dt)
        return [np.asarray(path_obj.eval(t)[0], dtype=np.float64) for t in times]

    def plan_grasp(self, gripper: str, handle: str, q_init: list):
        q = np.asarray(q_init, dtype=np.float64)
        self.view(q)
        result = self._path_planner.planPathtoBarHandling(
            gripper, handle, q, self._logger, self._viewer
        )
        if result is not None:
            self._logger.info("Path to bar grasping planned successfully.")
            if hasattr(self, "_viewer"):
                self._viewer.loadPath(result)
        else:
            self._logger.warn("Path to bar grasping planning failed.")
            return None, None
        traj = self.sample_trajectory(result)
        last_q = list(traj[-1]) if traj else None
        return traj, last_q

    def plan_place(
        self, gripper: str, handle: str, q_init: list, target_bar_pose: list
    ):
        q = np.asarray(q_init, dtype=np.float64)
        self.view(q)
        result = self._path_planner.planPathtoBarPlacement(
            gripper, handle, q, target_bar_pose, self._logger, self._viewer
        )
        if result is not None:
            self._logger.info("Path to bar placement planned successfully.")
            if hasattr(self, "_viewer"):
                self._viewer.loadPath(result)
        else:
            self._logger.warn("Path to bar placement planning failed.")
            return None, None
        traj = self.sample_trajectory(result)
        last_q = list(traj[-1]) if traj else None
        return traj, last_q

    # == Visualisation =========================================================

    def init_viewer(self, open: bool = False):
        from pyhpp_viser import Viewer

        self._viewer = Viewer(self.robot)
        self._viewer.initViewer(open=open, loadModel=True)
        self._viewer.setProblem(self._problem)
        self._viewer.setGraph(self.graph)
        print("Viser viewer ready.  Use o.view(q) or o.play(path).")

    def view(self, q=None):
        if not hasattr(self, "_viewer"):
            self.init_viewer()
        self._viewer(q if q is not None else pin.neutral(self._pin_model))

    def play(self, path, n=100, dt=0.05):
        import time as _time

        if not hasattr(self, "_viewer"):
            self.init_viewer()
        try:
            self._viewer.loadPath(path)
        except Exception:
            pass
        t0 = path.timeRange().first
        tf = path.timeRange().second
        for i in range(n):
            t = t0 + i * (tf - t0) / (n - 1)
            self._viewer(path.eval(t)[0])
            _time.sleep(dt)
