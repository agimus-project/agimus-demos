
from math import pi
import os

from hpp.corbaserver import loadServerPlugin, wrap_delete as wd
from hpp.corbaserver.manipulation import Client, ProblemSolver, Robot
from hpp.gepetto.manipulation import ViewerFactory
import numpy as np
import pinocchio as pin

PLOT = True
if PLOT:
    import matplotlib.pyplot as plt
    plt.ion()

from agimus_demo_03_mpc_dummy_traj_tiago_pro.spline import SplineBezier

class WrapDeleteHPP:
    """! Wrapper to the wrap_delete method
    Automatically deletes the corresponding servant object on the server when
    the Python object is deleted.
    """
    cmp = None
    crobot = None
    cproblem = None
    csm = None
    cs = None
    cp = None


class HPPResults:
    ee_path = None
    joint_path = None

class HPPLeftArmSquareMotion:
    """Plan trajectory on a square motion for Tiago pro robot using HPP.
    """

    def __init__(self,
            robot_urdf_string: str = "",
            robot_srdf_string: str = "",
        ):
        """Initialize HPP client, robot, problem solver, viewer factory.
        Create locked joints, constraint set, config projector, steering method.
        Create spline path for end-effector moving in square motion.
        """
        self.ee_link_name = "tiago/arm_left_6_joint"
        self.base_link_name = "tiago/base_link"

        self.hack_for_ros2_support_in_hpp()
        self.load_robot_in_hpp(robot_urdf_string, robot_srdf_string)

    @property
    def ps(self):
        return self.problem_solver

    @property
    def vf(self):
        return self.viewer_factory

    def load_robot_in_hpp(self,
            robot_urdf_string: str,
            robot_srdf_string: str,
        ):
        """Load robot in HPP with given URDF and SRDF strings.
        Create locked joints, constraint set, config projector, steering method.
        Create spline path for end-effector moving in square motion.
        """
        Robot.urdfSrdfString = robot_urdf_string
        Robot.srdfSrdfString = robot_srdf_string
        loadServerPlugin("corbaserver", "manipulation-corba.so")
        Client().problem.resetProblem()
        self.robot = Robot("tiago-manip", "tiago", rootJointType="anchor")
        self.problem_solver = ProblemSolver(self.robot)
        self.viewer_factory = ViewerFactory(self.problem_solver)
        
        # Lock all joint except left arm in neutral positions
        locked_joints = list(
            filter(lambda j: not j.startswith("tiago/arm_left"), self.robot.jointNames)
        )
        self.locked_joint_names = list()
        q0 = self.robot.getCurrentConfig()
        for j in locked_joints:
            r = self.robot.rankInConfiguration[j]
            size = self.robot.getJointConfigSize(j)
            name = f"locked_{j}"
            self.locked_joint_names.append(name)
            self.ps.createLockedJoint(name, j, q0[r : r + size])
            self.ps.setConstantRightHandSide(name, True)

    def set_current_config(self, q):
        self.robot.setCurrentConfig(q)

    def set_pi_4_left_arm(self):
        """Fold left arm to move away from singularity.
        """
        q0 = self.robot.getCurrentConfig()
        r = self.robot.rankInConfiguration["tiago/arm_left_2_joint"]
        q0[r] = pi / 4
        q0[r + 1] = pi / 4
        q0[r + 2] = -pi / 2
        q0[r + 3] = -pi / 4
        q0[r + 4] = pi / 2
        return q0

    def plan_cartesian_trajectory(self, q0: list):
        # Return a HPPResults object containing the planned paths.        
        hpp_results = HPPResults()

        # Create an EndEffectorTrajectory steering method
        self.wdhpp = WrapDeleteHPP()
        self.wdhpp.cmp = wd(self.ps.client.basic.problem.getProblem())
        self.wdhpp.crobot = wd(self.wdhpp.cmp.robot())
        self.wdhpp.cproblem = wd(self.ps.client.basic.problem.createProblem(self.wdhpp.crobot))
        self.wdhpp.csm = wd(
            self.ps.client.basic.problem.createSteeringMethod(
                "EndEffectorTrajectory", self.wdhpp.cproblem
            )
        )
        self.wdhpp.cs = wd(
            self.ps.client.basic.problem.createConstraintSet(self.wdhpp.crobot, "sm-constraints")
        )
        self.wdhpp.cp = wd(
            self.ps.client.basic.problem.createConfigProjector(
                self.wdhpp.crobot, "cp", 1e-4, 40
            )
        )
        self.wdhpp.cs.addConstraint(self.wdhpp.cp)
        self.wdhpp.cproblem.setConstraints(self.wdhpp.cs)
        self.wdhpp.cproblem.setSteeringMethod(self.wdhpp.csm)
        # Add locked joints to the problem
        for lj in self.locked_joint_names:
            tc = wd(self.ps.client.basic.problem.getConstraint(lj))
            self.wdhpp.cp.add(tc, 0)

        # Create constraint and set it to config projector
        self.wdhpp.crobot.setCurrentConfig(q0) # add first way point
        self.first_way_point_in_world_se3 = pin.XYZQUATToSE3(self.robot.getJointPosition(self.ee_link_name))
        self.base_in_world_se3 = pin.XYZQUATToSE3(self.robot.getJointPosition(self.base_link_name))
        self.first_way_point_in_base_se3 = (
            self.base_in_world_se3.inverse() *
            self.first_way_point_in_world_se3
        )
        self.first_way_point_in_base = pin.SE3ToXYZQUAT(self.first_way_point_in_base_se3).tolist()
        self.base_in_first_way_point_se3 = self.first_way_point_in_base_se3.inverse()
        self.base_in_first_way_point = pin.SE3ToXYZQUAT(self.base_in_first_way_point_se3).tolist()
        self.ps.client.basic.problem.createTransformationR3xSO3Constraint(
            "ee-pose",
            self.base_link_name,
            self.ee_link_name,
            self.first_way_point_in_base,
            [0, 0, 0, 0, 0, 0, 1],
            6 * [True],
        )
        self.ps.setConstantRightHandSide("ee-pose", False)
        tc = wd(self.ps.client.basic.problem.getConstraint("ee-pose"))
        self.wdhpp.cp.add(tc, 0)
        self.wdhpp.csm.trajectoryConstraint(tc)
        res, q1 = self.wdhpp.cp.apply(q0)
        assert res

        self.ee_spline = SplineBezier(self.ps) # Order 5 Bezier spline.
        r = 0.15 # square size
        # Offset for the first way point, expecting "offer" pose on AGIMUS-TIAGo
        ipos = [
            self.first_way_point_in_base[0]+0.1,
            self.first_way_point_in_base[1],
            self.first_way_point_in_base[2],
        ]
        irot = self.first_way_point_in_base[3:]
        rhs0 = [0, 0, 0, 0, 0, 0, 1]
        rhs1 = pin.SE3ToXYZQUAT(self.base_in_first_way_point_se3 * pin.XYZQUATToSE3([ipos[0], ipos[1], ipos[2]] + irot)).tolist()
        rhs2 = pin.SE3ToXYZQUAT(self.base_in_first_way_point_se3 * pin.XYZQUATToSE3([ipos[0], ipos[1] + r, ipos[2]] + irot)).tolist()
        rhs3 = pin.SE3ToXYZQUAT(self.base_in_first_way_point_se3 * pin.XYZQUATToSE3([ipos[0] - r, ipos[1] + r, ipos[2]] + irot)).tolist()
        rhs4 = pin.SE3ToXYZQUAT(self.base_in_first_way_point_se3 * pin.XYZQUATToSE3([ipos[0] - r, ipos[1], ipos[2]] + irot)).tolist()
        derivative = [[0, 0]] * 6
        p1 = wd(self.ee_spline.createSplinePath(rhs0, rhs1, 1.0, [1, 2], derivative, [1, 2], derivative))
        p2 = wd(self.ee_spline.createSplinePath(rhs1, rhs2, 1.0, [1, 2], derivative, [1, 2], derivative))
        p3 = wd(self.ee_spline.createSplinePath(rhs2, rhs3, 1.0, [1, 2], derivative, [1, 2], derivative))
        p4 = wd(self.ee_spline.createSplinePath(rhs3, rhs4, 1.0, [1, 2], derivative, [1, 2], derivative))
        p5 = wd(self.ee_spline.createSplinePath(rhs4, rhs1, 1.0, [1, 2], derivative, [1, 2], derivative))
        
        
        hpp_results.ee_path = p1.asVector()
        hpp_results.ee_path.appendPath(p2)
        hpp_results.ee_path.appendPath(p3)
        hpp_results.ee_path.appendPath(p4)
        hpp_results.ee_path.appendPath(p5)


    
    def plot_hpp_results(self, hpp_results: HPPResults):
        if not PLOT:
            return
        p = p1.asVector()
        p.appendPath(p2)
        p.appendPath(p3)
        p.appendPath(p4)
        p.appendPath(p5)

        # Plot spline path
        spline_path = np.array([])
        for t in np.arange(0, p.length(), 0.05):
            pos, res = p.call(t)
            assert(res)
            pos = first_way_point_in_world_se3 * pin.XYZQUATToSE3(pos)
            pos = pin.SE3ToXYZQUAT(pos)
            spline_path = np.vstack((spline_path, pos)) if spline_path.size else pos

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot(spline_path[:, 0], spline_path[:, 1], spline_path[:, 2], label='Spline Path')
        # Draw the axis at the first way point
        origin = first_way_point_in_world[:3]
        ax.quiver(origin[0], origin[1], origin[2],
                1, 0, 0,
                color='r', length=0.1, normalize=True, label='X axis')
        ax.quiver(origin[0], origin[1], origin[2],
                0, 1, 0,
                color='g', length=0.1, normalize=True, label='Y axis')
        ax.quiver(origin[0], origin[1], origin[2],
                0, 0, 1,
                color='b', length=0.1, normalize=True, label='Z axis')
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('End-Effector Spline Path')
        ax.legend()
        # Set equal aspect ratio for 3D plot
        ax.set_box_aspect([1,1,1])
        # Optional: set equal scaling for all axes
        x_limits = ax.get_xlim3d()
        y_limits = ax.get_ylim3d()
        z_limits = ax.get_zlim3d()
        x_range = abs(x_limits[1] - x_limits[0])
        y_range = abs(y_limits[1] - y_limits[0])
        z_range = abs(z_limits[1] - z_limits[0])
        max_range = max([x_range, y_range, z_range])
        x_middle = np.mean(x_limits)
        y_middle = np.mean(y_limits)
        z_middle = np.mean(z_limits)
        ax.set_xlim3d([x_middle - max_range/2, x_middle + max_range/2])
        ax.set_ylim3d([y_middle - max_range/2, y_middle + max_range/2])
        ax.set_zlim3d([z_middle - max_range/2, z_middle + max_range/2])
        plt.show()

        # Set this path as the time-varying right hand side of the constraint
        csm.trajectory(p, True)

        # Create goal configuration with final right hand side
        rhs = cp.getRightHandSide()
        rhs[:7] = p.end()
        cp.setRightHandSide(rhs)
        res, q2 = cp.apply(q1)
        # assert(res)
        # Call steering method
        traj = wd(csm.call(q1, q1))
        if traj:
            self.ps.client.basic.problem.addPath(traj.asVector())

        sq1, res = traj.call(0.0)
        sq2, res = traj.call(1.0)
        sq3, res = traj.call(2.0)
        sq4, res = traj.call(3.0)
        sq5, res = traj.call(4.0) # == 0

        # Create spline path for velocity computation in joint space
        spline2 = SplineBezierRobot(ps)
        derivative = robot.getNumberDof() * [[0.0, 0.0]]
        s1 = wd(spline2.createSplinePath(sq1, sq2, 1.0, [1, 2], derivative, [1, 2], derivative))
        s2 = wd(spline2.createSplinePath(sq2, sq3, 1.0, [1, 2], derivative, [1, 2], derivative))
        s3 = wd(spline2.createSplinePath(sq3, sq4, 1.0, [1, 2], derivative, [1, 2], derivative))
        s4 = wd(spline2.createSplinePath(sq4, sq5, 1.0, [1, 2], derivative, [1, 2], derivative))
        s = s1.asVector()
        s.appendPath(s2)
        s.appendPath(s3)
        s.appendPath(s4)

        v = vf.createViewer()

        class Data:
            q = None
            v = None
            ee = None
            v_ee = None

        class DataPlot:
            q = np.array([])
            v = np.array([])
            ee = np.array([])
            v_ee = np.array([])

        p_discr = list()
        data_plot = DataPlot()
        duration = p.length()
        t0 = 0.0
        dt = 0.05
        for t in np.arange(0, duration, dt):
            d = Data()
            ee, res = p.call(t)
            assert(res)
            d.ee = ee
            d.v_ee = p.derivative(t, 1)
            q, res = traj.call(t0 + t)
            assert(res)
            d.q = q
            # finite difference for velocity
            # if t == 0.0:
            #     d.v = robot.getNumberDof() * [0.0]
            # else:
            #     d.v = ((np.array(d.q) - np.array(p_discr[-1].q)) / dt).tolist()
            # use spline derivative for velocity
            d.v = s.derivative(t, 1)
            data_plot.q = np.vstack((data_plot.q, np.array(d.q))) if data_plot.q.size else np.array(d.q)
            data_plot.v = np.vstack((data_plot.v, np.array(d.v))) if data_plot.v.size else np.array(d.v)
            data_plot.ee = np.vstack((data_plot.ee, np.array(d.ee))) if data_plot.ee.size else np.array(d.ee)
            data_plot.v_ee = np.vstack((data_plot.v_ee, np.array(d.v_ee))) if data_plot.v_ee.size else np.array(d.v_ee)
            p_discr.append(d)
            v(d.q)
            time.sleep(dt)

        # use matplotlib to plot data
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(4, 1, figsize=(10, 15))
        # Joint positions: plot each joint with a consistent color
        axs[0].set_title('Joint positions')
        joint_colors = []
        for i in range(data_plot.q.shape[1]):
            line, = axs[0].plot(data_plot.q[:, i], label=f'Joint {i}')
            joint_colors.append(line.get_color())
        axs[0].set_ylabel('Position (rad)')
        axs[0].legend()

        # Joint velocities: use the same color for each joint as above
        axs[1].set_title('Joint velocities')
        for i in range(data_plot.v.shape[1]):
            axs[1].plot(data_plot.v[:, i], label=f'Joint {i}', color=joint_colors[i])
        axs[1].set_ylabel('Velocity (rad/s)')
        axs[1].legend()

        # End-effector positions: plot each dimension with a consistent color
        axs[2].set_title('End-effector positions')
        ee_colors = []
        for i in range(data_plot.ee.shape[1]):
            line, = axs[2].plot(data_plot.ee[:, i], label=f'EE dim {i}')
            ee_colors.append(line.get_color())
        axs[2].set_ylabel('Position (m and quat)')
        axs[2].legend()

        # End-effector velocities: use the same color for each dimension as above
        axs[3].set_title('End-effector velocities')
        for i in range(data_plot.v_ee.shape[1]):
            axs[3].plot(data_plot.v_ee[:, i], label=f'EE dim {i}', color=ee_colors[i])
        axs[3].set_ylabel('Velocity (m/s and quat/s)')
        axs[3].legend()

        plt.tight_layout()
        plt.show()

    def hack_for_ros2_support_in_hpp(self):
        if "ROS_PACKAGE_PATH" not in os.environ and "AMENT_PREFIX_PATH" in os.environ:
            os.environ["ROS_PACKAGE_PATH"] = ":".join(
                v + "/share" for v in os.environ["AMENT_PREFIX_PATH"].split(":")
            )
