import numpy as np

from pyhpp.core import (
    RandomShortcut,
    SplineGradientBased_bezier3,
    SimpleTimeParameterization,
    ProgressiveProjector,
)
from pyhpp.manipulation import TransitionPlanner
from pyhpp.core.path import Vector as PathVector

_DIST_MIN = 0.40  # m  — closest approach to handle
_DIST_MAX = 0.90  # m  — furthest approach
_ANGLE_SPREAD = 0.35  # rad ≈ 20°  — lateral angular jitter around the ideal axis
_TORSO_MIN = 0.00  # m
_TORSO_MAX = 0.25  # m
_ARM_NOISE = 0.10  # rad — Gaussian std for arm joint perturbation
_MAX_ATTEMPTS = 200  # hard cap; was 10000 — most successes happen in < 20


def concatenatePaths(paths, logger=None):
    if not paths:
        return None
    pv = PathVector.create(paths[0].outputSize(), paths[0].outputDerivativeSize())
    for p in paths:
        pv.appendPath(p)
    return pv


class PathPlanner:
    def __init__(self, problem, cg, robot):
        self.problem = problem
        self.graph = cg
        self.robot = robot
        self._set_transition_planner()
        # graph MUST be initialized before this constructor is called
        # (enforced by HPPPathGenerator which calls graph.initialize()
        #  inside _generate_constraint_graph() before creating PathPlanner).

    def _get_idx_q(self, joint_name):
        """Get idx_q for a joint via Pinocchio model."""
        model = self.robot.model()
        if not model.existJointName(joint_name):
            raise ValueError(f"Joint '{joint_name}' not found in model.")
        jid = model.getJointId(joint_name)
        return model.joints[jid].idx_q

    def _set_transition_planner(self):
        """Initialize TransitionPlanner with appropriate settings for our problem."""
        self.transitionPlanner = TransitionPlanner(self.problem)

        inner_problem = self.transitionPlanner.innerProblem()
        projector = ProgressiveProjector(
            inner_problem.distance(),
            inner_problem.steeringMethod(),
            0.05,
        )
        self.transitionPlanner.pathProjector(projector)

        self.transitionPlanner.maxIterations(1000)

        self.transitionPlanner.clearPathOptimizers()
        self.transitionPlanner.addPathOptimizer(RandomShortcut(self.problem))
        stp = SimpleTimeParameterization(self.problem)
        stp.order = 2
        stp.maxAcceleration = 0.2
        self.transitionPlanner.addPathOptimizer(stp)

        self.transitionPlanner.setTransition(self.graph.getTransition("Loop | f"))

    def checkConfigurationValid(self, q):
        """Check if configuration q is valid (collision-free and satisfies constraints)."""
        res, _msg = self.problem.isConfigValid(q)
        return res

    def compute_base_pose_from_handle(
        self, handle_pose, current_base_pose, distance=0.4
    ):
        """Compute a base pose in front of the handle, facing it."""
        hx, hy = handle_pose[0], handle_pose[1]
        rx, ry = current_base_pose[0], current_base_pose[1]
        vx, vy = rx - hx, ry - hy
        norm = np.hypot(vx, vy)
        if norm < 1e-3:
            vx, vy, norm = -1.0, 0.0, 1.0
        vx /= norm
        vy /= norm
        x, y = hx + vx * distance, hy + vy * distance
        theta = np.arctan2(-vy, -vx)
        return x, y, np.cos(theta), np.sin(theta)

    def generateGraspingConfigurations(
        self, gripper, handle, q_init, testCollision=True, logger=None, viewer=None
    ):
        ctx = f"[grasp {gripper} → {handle}]"
        prefix = f"{gripper} > {handle} | f"

        handle_idx = self._get_idx_q("reinforcement_bar/root_joint")
        base_idx = self._get_idx_q("tiago_pro/root_joint")
        torso_idx = self._get_idx_q("tiago_pro/torso_lift_joint")
        left_arm_idx = self._get_idx_q("tiago_pro/arm_left_1_joint")
        right_arm_idx = self._get_idx_q("tiago_pro/arm_right_1_joint")

        handle_pose = q_init[handle_idx : handle_idx + 7]
        base_pose = q_init[base_idx : base_idx + 4]
        hx, hy = handle_pose[0], handle_pose[1]

        q_base_nom = self.compute_base_pose_from_handle(handle_pose, base_pose)
        theta_nom = np.arctan2(q_base_nom[1] - hy, q_base_nom[0] - hx)

        def _seed_pg_g(q_ref, attempt):
            """_01 and _12 : bar locked → noise only arms and torso"""
            q_s = q_ref.copy()
            if attempt > 0:
                q_s[torso_idx] = float(np.random.uniform(_TORSO_MIN, _TORSO_MAX))
                q_s[left_arm_idx : left_arm_idx + 7] += np.random.normal(
                    0, _ARM_NOISE, 7
                )
                q_s[right_arm_idx : right_arm_idx + 7] += np.random.normal(
                    0, _ARM_NOISE, 7
                )
            return q_s

        def _seed_pp(q_ref, attempt):
            """_23 : bar free → Noise pose bar + arms"""
            q_s = q_ref.copy()
            if attempt > 0:
                q_s[torso_idx] = float(np.random.uniform(_TORSO_MIN, _TORSO_MAX))
                q_s[handle_idx + 3] += 0.05
                q_s[left_arm_idx : left_arm_idx + 7] += np.random.normal(0, 0.03, 7)
                q_s[right_arm_idx : right_arm_idx + 7] += np.random.normal(0, 0.03, 7)
                res, q_s, _ = self.graph.applyStateConstraints(
                    self.graph.getState(prefix + "_preplace"), q_s
                )
            return q_s

        MAX_BASE_ATTEMPTS = 50
        MAX_LOCAL_RETRIES = 10

        for attempt_base in range(MAX_BASE_ATTEMPTS):
            # == APPROACH CONFIGURATION ==
            q = q_init.copy()
            if attempt_base == 0:
                q[base_idx : base_idx + 4] = q_base_nom
            else:
                theta = theta_nom + np.random.uniform(-_ANGLE_SPREAD, _ANGLE_SPREAD)
                dist = np.random.uniform(_DIST_MIN, _DIST_MAX)
                q[base_idx] = hx + dist * np.cos(theta)
                q[base_idx + 1] = hy + dist * np.sin(theta)
                q[base_idx + 2] = np.cos(theta + np.pi)
                q[base_idx + 3] = np.sin(theta + np.pi)
                q[torso_idx] = float(np.random.uniform(_TORSO_MIN, _TORSO_MAX))

            res, qap, _ = self.graph.generateTargetConfig(
                self.graph.getTransition("transit_free"), q_init, q
            )
            if not res:
                continue
            if testCollision and not self.checkConfigurationValid(qap):
                continue
            if viewer:
                viewer(qap)

            # == PREGRASP CONFIGURATION ==
            for attempt_pg in range(MAX_LOCAL_RETRIES):
                res, qpg, _ = self.graph.generateTargetConfig(
                    self.graph.getTransition(prefix + "_01"),
                    qap,
                    _seed_pg_g(qap, attempt_pg),
                )
                if not res:
                    continue
                if testCollision and not self.checkConfigurationValid(qpg):
                    continue
                if viewer:
                    viewer(qpg)

                # == GRASP CONFIG ==
                for attempt_g in range(MAX_LOCAL_RETRIES):
                    res, qg, _ = self.graph.generateTargetConfig(
                        self.graph.getTransition(prefix + "_12"),
                        qpg,
                        _seed_pg_g(qpg, attempt_g),
                    )
                    if not res:
                        continue
                    if testCollision and not self.checkConfigurationValid(qg):
                        continue
                    if viewer:
                        viewer(qg)

                    # ++ PREPLACE CONFIG ==
                    for attempt_pp in range(MAX_LOCAL_RETRIES):
                        q_seed = _seed_pp(qg, attempt_pp)
                        viewer(q_seed)
                        res, qpp, _ = self.graph.generateTargetConfig(
                            self.graph.getTransition(prefix + "_23"), qg, q_seed
                        )
                        if not res:
                            continue
                        if testCollision and not self.checkConfigurationValid(qpp):
                            continue
                        if viewer:
                            viewer(qpp)

                        self._log(
                            logger,
                            "INFO",
                            f"{ctx} found — base={attempt_base + 1}/{MAX_BASE_ATTEMPTS} "
                            f"pg={attempt_pg + 1} g={attempt_g + 1} pp={attempt_pp + 1}",
                        )
                        return qap, qpg, qg, qpp

        self._log(
            logger, "WARN", f"{ctx} failed after {MAX_BASE_ATTEMPTS} base attempts"
        )
        return None, None, None, None

    def generatePlacementConfigurations(
        self,
        gripper,
        handle,
        q_init,
        target_bar_pose,
        testCollision=True,
        logger=None,
    ):
        """Generate placing configurations (approach, preplacement, release, post-realese) for a given gripper and handle.

        Args:
            gripper (str): The name of the gripper (e.g., "tiago_pro/left").
            handle (str): The name of the handle (e.g., "reinforcement_bar/left").
            q_init (list): The initial configuration of the robot.
            target_bar_pose (list): The desired final configuration of the bar
            testCollision (bool): Whether to test for collision when generating configurations.
            logger: Optional logger for logging messages.
        Returns:
            tuple: A tuple containing the pregrasp configuration, grasp configuration, and preplace configuration. If no valid configurations are found, returns (None, None, None).
        """
        prefix = f"{gripper} < {handle} | 0-0"

        handle_idx = self._get_idx_q("reinforcement_bar/root_joint")
        base_idx = self._get_idx_q("tiago_pro/root_joint")

        q_init = self._asq(q_init)
        q_goal = q_init.copy()
        q_goal[handle_idx : handle_idx + 7] = target_bar_pose

        base_pose = q_init[base_idx : base_idx + 4]
        q_base_place = self.compute_base_pose_from_handle(target_bar_pose, base_pose)

        qap, qpp, qp, qrel = None, None, None, None

        # == APPROACH CONFIGURATION ==
        # Generate a configuration that satisfies the "transit_grasp" constraints, with the base positioned in front of the target placement pose.
        q = q_init.copy()
        q[base_idx : base_idx + 4] = q_base_place
        _, q, _ = self.graph.applyStateConstraints(
            self.graph.getState(f"{gripper} grasps {handle}"), self._asq(q)
        )
        transition = self.graph.getTransition("transit_grasp")
        res, qap, err = self.graph.generateTargetConfig(transition, q_init, q)
        if not res:
            self._log(logger, "WARN", f"qap generation failed — {err}")
            return None, None, None, None
        if testCollision:
            pv = transition.pathValidation()
            res, msg = pv.validateConfiguration(qap)
            if not res:
                self._log(logger, "WARN", f"qap fail — {msg}")
                return None, None, None, None
        # == PREPLACE CONFIGURATION ==
        transition = self.graph.getTransition(prefix + "_32")
        res, qpp, err = self.graph.generateTargetConfig(transition, qap, qap)
        if not res:
            self._log(logger, "WARN", f"qpp generation failed — {err}")
            return qap, None, None, None
        if testCollision:
            pv = transition.pathValidation()
            res, msg = pv.validateConfiguration(qpp)
            if not res:
                self._log(logger, "WARN", f"qpp fail — {msg}")
                return qap, None, None, None
        # == PLACE CONFIGURATION ==
        transition = self.graph.getTransition(prefix + "_21")
        res, qp, err = self.graph.generateTargetConfig(transition, qpp, qpp)
        if not res:
            self._log(logger, "WARN", f"qp generation failed — {err}")
            return qap, qpp, None, None
        if testCollision:
            pv = transition.pathValidation()
            res, msg = pv.validateConfiguration(qp)
            if not res:
                self._log(logger, "WARN", f"qp fail — {msg}")
                return qap, qpp, None, None
        # == RELEASE CONFIGURATION ==
        transition = self.graph.getTransition(prefix + "_10")
        res, qrel, err = self.graph.generateTargetConfig(transition, qp, qp)
        if not res:
            self._log(logger, "WARN", f"qrel generation failed — {err}")
            return qap, qpp, qp, None
        if testCollision:
            pv = transition.pathValidation()
            res, msg = pv.validateConfiguration(qrel)
            if not res:
                self._log(logger, "WARN", f"qrel fail — {msg}")
                return qap, qpp, qp, None
        return qap, qpp, qp, qrel

    def _log(self, logger, level, msg):
        """
        Log a message with the given level using the provided logger, or print to console if logger is None.
        !!! Raise an exception if there are an error
        """
        formatted_msg = f"[PathPlanner] {msg}"
        # Ros logging
        if level == "INFO":
            if logger is not None:
                logger.info(formatted_msg)
        elif level == "WARN":
            if logger is not None:
                logger.warn(formatted_msg)
            raise RuntimeError(msg)
        elif level == "ERROR":
            if logger is not None:
                logger.error(formatted_msg)
            raise Exception(msg)

    def _asq(self, q) -> np.ndarray:
        """Convert a configuration to a numpy array of type float64."""
        return np.asarray(q, dtype=np.float64)

    def _goals_matrix(self, q: np.ndarray) -> np.ndarray:
        """Convert a single configuration q into a 2D numpy array with shape (1, config_size) in Fortran order."""
        q_goal = np.zeros((1, self.robot.configSize()), order="F")
        q_goal[0, :] = q
        return q_goal

    def optimizePath(self, path, logger=None):
        """Optimize a path using shortcutting and spline optimization, with logging."""
        shortcut = RandomShortcut(self.problem)
        spline_opt = SplineGradientBased_bezier3(self.problem)
        try:
            for i in range(3):
                p_new = shortcut.optimize(path)
                tr_before = path.timeRange()
                tr_after = p_new.timeRange()
                dt = (tr_before.second - tr_before.first) - (
                    tr_after.second - tr_after.first
                )
                path = p_new
                self._log(
                    logger,
                    "INFO",
                    f"  path shortcut pass {i + 1}/3: {tr_after.second - tr_after.first:.2f} s  (−{dt:.2f} s)",
                )
                if dt < 1e-3:
                    break
        except Exception:
            self._log(logger, "ERROR", "  path shortcut failed: {e}")
        try:
            path = spline_opt.optimize(path)
            tr = path.timeRange()
            self._log(logger, "INFO", f"  path spline: {tr.second - tr.first:.2f} s")
        except Exception as e:
            self._log(logger, "ERROR", f"  path spline optimisation failed: {e}")
        return path

    def planPathtoBarHandling(self, gripper, handle, q_init, logger, v):
        """
        Plan a path from the initial configuration to a grasping configuration for the specified gripper and handle.
        Args:
            gripper (str): The name of the gripper (e.g., "tiago_pro/left").
            handle (str): The name of the handle (e.g., "reinforcement_bar/left").
            q_init (list): The initial configuration of the robot.
            logger: Optional logger for logging messages.
        Returns:
            PathVector: A path vector representing the planned path, or None if planning failed.
        """
        res, q_init, err = self.graph.applyStateConstraints(
            self.graph.getState("free"), q_init
        )
        if not res:
            self._log(
                logger,
                "WARN",
                f"applyStateConstraints('free') failed — using raw q_init, HPP error: {err}",
            )

        qap, qpg, qg, qpp = self.generateGraspingConfigurations(
            gripper,
            handle,
            self._asq(q_init),
            testCollision=True,
            logger=None,
            viewer=v,
        )
        if qap is None or qpg is None or qg is None or qpp is None:
            self._log(logger, "WARN", "Failed to generate grasping configurations")
            return None
        self._log(logger, "INFO", "Grasping configurations generated")

        prefix = f"{gripper} > {handle} | f"

        # ==================================================================
        # Segment 0: free → approach (direct, transit_free)
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition("transit_free"))
        res, p0, msg = self.transitionPlanner.directPath(q_init, qap, True)
        if not res:
            self._log(logger, "WARN", f"Segment 0 FAILED — {msg}")
            return None

        # ==================================================================
        # Segment 1: grasp approach -> preplacement (RRT, prefix + "_01")
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition(prefix + "_01"))
        p1 = self.transitionPlanner.planPath(qap, self._goals_matrix(qpg), True)
        if not p1:
            self._log(logger, "WARN", "Segment 1 FAILED")
            return None
        # p1 = self.transitionPlanner.optimizePath(p1)
        # p1 = self.optimizePath(p1, logger=logger)

        # ==================================================================
        # Segment 2: pregrasp → grasp (constrained direct, prefix + "_12")
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition(prefix + "_12"))
        res, p2, msg = self.transitionPlanner.directPath(qpg, qg, True)
        if not res:
            self._log(logger, "WARN", f"Segment 2 FAILED — {msg}")
            return None
        p2_vector = PathVector.create(p2.outputSize(), p2.outputDerivativeSize())
        p2_vector.appendPath(p2)
        # p2 = self.transitionPlanner.timeParameterization(p2_vector)
        # p2 = self.optimizePath(p2_vector, logger=logger)

        # ==================================================================
        # Segment 3: grasp → preplace (constrained direct, prefix + "_23")
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition(prefix + "_23"))
        res, p3, msg = self.transitionPlanner.directPath(qg, qpp, True)
        if not res:
            self._log(logger, "WARN", f"Segment 3 FAILED — {msg}")
            return None
        p3_vector = PathVector.create(p3.outputSize(), p3.outputDerivativeSize())
        p3_vector.appendPath(p3)
        # p3 = self.transitionPlanner.timeParameterization(p3_vector)
        # p3 = self.optimizePath(p3_vector, logger=logger)

        self._log(logger, "INFO", "Grasping path planned successfully.")
        return concatenatePaths([p0, p1, p2, p3], logger=logger)

    def planPathtoBarPlacement(self, gripper, handle, q_init, target_bar_pose, logger):
        self._set_transition_planner()
        """
        Plan a path from the grasping configuration to the placement configuration for the specified gripper and handle.
        Args:
            gripper (str): The name of the gripper (e.g., "tiago_pro/left").
            handle (str): The name of the handle (e.g., "reinforcement_bar/left").
            q_init (list): The initial configuration of the robot (after grasping).
            target_bar_pose (list): The desired final configuration of the bar.
            logger: Optional logger for logging messages.
        Returns:
            PathVector: A path vector representing the planned path, or None if planning failed.
        """

        res, q_init, _ = self.graph.applyStateConstraints(
            self.graph.getState(f"{gripper} grasps {handle}"), q_init
        )
        if not res:
            self._log(logger, "WARN", "applyStateConstraints for grasp state failed")

        qap, qpp, qp, qrel = self.generatePlacementConfigurations(
            gripper, handle, q_init, target_bar_pose, testCollision=True, logger=logger
        )
        if qap is None or qpp is None or qp is None or qrel is None:
            self._log(logger, "ERROR", "Failed to generate placement configurations")
            return None
        self._log(logger, "INFO", "Placement configurations generated")

        prefix = f"{gripper} < {handle} | 0-0"
        # ==================================================================
        # Segment 0: grasp approach (direct, transit_grasp)
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition("transit_grasp"))
        res, p0, msg = self.transitionPlanner.directPath(q_init, qap, True)
        if not res:
            self._log(logger, "ERROR", f"Segment 0 FAILED — {msg}")
            return None

        # ==================================================================
        # Segment 1: approach → preplacement (RRT, prefix + "_32")
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition(prefix + "_32"))
        p1 = self.transitionPlanner.planPath(qap, self._goals_matrix(qpp), True)
        if not p1:
            self._log(logger, "ERROR", "Segment 1 FAILED")
            return None

        # p1 = self.transitionPlanner.optimizePath(p1)
        # p1 = self.optimizePath(p1, logger=logger)

        # ==================================================================
        # Segment 2: preplacement → placement (constrained direct, prefix + "_21")
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition(prefix + "_21"))
        res, p2, msg = self.transitionPlanner.directPath(qpp, qp, True)
        if not res:
            self._log(logger, "ERROR", f"Segment 2 FAILED — {msg}")
            return None
        # p2 = self.transitionPlanner.timeParameterization(p2)
        # p2 = self.optimizePath(p2, logger=logger)

        # ==================================================================
        # Segment 3: placement → release (constrained direct, prefix + "_10")
        # ==================================================================
        self.transitionPlanner.setTransition(self.graph.getTransition(prefix + "_10"))
        res, p3, msg = self.transitionPlanner.directPath(qp, qrel, True)
        if not res:
            self._log(logger, "ERROR", f"Segment 3 FAILED — {msg}")
            return None
        # p3 = self.transitionPlanner.timeParameterization(p3)
        # p3 = self.optimizePath(p3, logger=logger)

        self._log(logger, "INFO", "Placement path planned successfully.")
        return concatenatePaths([p0, p1, p2, p3])
