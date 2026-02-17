"""HPP plan used on AGIMUS-TIAGo. Plan square motion of the end-effector."""

from agimus_controller.trajectory import TrajectoryPoint
import example_robot_data
from hpp.corbaserver import loadServerPlugin, wrap_delete as wd
from hpp.corbaserver.manipulation import Client, ProblemSolver, Robot
from hpp.gepetto.manipulation import ViewerFactory
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin

from spline import SplineBezier, SplineBezierRobot

# Enable interactive mode at the beginning
plt.ion()


def plan_trajectory(_, dt):
    tiago_pro = example_robot_data.ROBOTS["tiago_pro"]()
    Robot.urdfFilename = tiago_pro.df_path
    Robot.srdfFilename = ""
    Robot.packageDirs = tiago_pro.model_path + "/" + tiago_pro.path

    loadServerPlugin("corbaserver", "manipulation-corba.so")
    Client().problem.resetProblem()
    robot = Robot("tiago-manip", "tiago", rootJointType="anchor")
    ps = ProblemSolver(robot)
    vf = ViewerFactory(ps)
    v = vf.createViewer()

    q0 = robot.getCurrentConfig()
    v(q0)

    # Offer pose
    r = robot.rankInConfiguration["tiago/torso_lift_joint"]
    q0[r] = 0.14
    r = robot.rankInConfiguration["tiago/arm_left_1_joint"]
    # left pose
    q0[r] = -0.25843
    q0[r + 1] = -0.57522
    q0[r + 2] = 0.50314
    q0[r + 3] = -2.0337
    q0[r + 4] = 0.0
    q0[r + 5] = 1.0543
    q0[r + 6] = 1.5708
    # right pose
    r = robot.rankInConfiguration["tiago/arm_right_1_joint"]
    q0[r] = 0.25843
    q0[r + 1] = -0.57522
    q0[r + 2] = -0.50314
    q0[r + 3] = -2.0337
    q0[r + 4] = 0.0
    q0[r + 5] = 1.0543
    q0[r + 6] = -1.5708
    v(q0)

    # Lock all joint except right arm in neutral positions
    lockedJoints = list(
        filter(lambda j: not j.startswith("tiago/arm_right"), robot.jointNames)
    )

    lockedJointNames = list()
    for j in lockedJoints:
        r = robot.rankInConfiguration[j]
        size = robot.getJointConfigSize(j)
        name = f"locked_{j}"
        lockedJointNames.append(name)
        ps.createLockedJoint(name, j, q0[r : r + size])
        ps.setConstantRightHandSide(name, True)

    # Create an EndEffectorTrajectory steering method
    cmp = wd(ps.client.basic.problem.getProblem())
    crobot = wd(cmp.robot())
    cproblem = wd(ps.client.basic.problem.createProblem(crobot))
    csm = wd(
        ps.client.basic.problem.createSteeringMethod("EndEffectorTrajectory", cproblem)
    )
    cs = wd(ps.client.basic.problem.createConstraintSet(crobot, "sm-constraints"))
    cp = wd(ps.client.basic.problem.createConfigProjector(crobot, "cp", 1e-4, 40))
    cs.addConstraint(cp)
    cproblem.setConstraints(cs)
    cproblem.setSteeringMethod(csm)

    # Add locked joints to the problem
    for lj in lockedJointNames:
        tc = wd(ps.client.basic.problem.getConstraint(lj))
        cp.add(tc, 0)

    # Create constraint and set it to config projector
    robot.setCurrentConfig(q0)  # add first way point
    first_way_point_in_world = robot.getJointPosition("tiago/arm_right_7_joint")
    first_way_point_in_world_se3 = pin.XYZQUATToSE3(first_way_point_in_world)
    base_in_world = robot.getJointPosition("tiago/base_link")
    base_in_world_se3 = pin.XYZQUATToSE3(base_in_world)
    first_way_point_in_base_se3 = (
        base_in_world_se3.inverse() * first_way_point_in_world_se3
    )
    first_way_point_in_base = pin.SE3ToXYZQUAT(first_way_point_in_base_se3).tolist()
    base_in_first_way_point_se3 = first_way_point_in_base_se3.inverse()

    ps.client.basic.problem.createTransformationR3xSO3Constraint(
        "ee-pose",
        # "", # in world
        "tiago/base_link",  # in base
        "tiago/arm_right_7_joint",
        pin.SE3ToXYZQUAT(first_way_point_in_base_se3).tolist(),
        [0, 0, 0, 0, 0, 0, 1],
        6 * [True],
    )
    ps.setConstantRightHandSide("ee-pose", False)
    tc = wd(ps.client.basic.problem.getConstraint("ee-pose"))
    cp.add(tc, 0)
    csm.trajectoryConstraint(tc)
    res, q1 = cp.apply(q0)
    assert res

    # Create a spline path for the end-effector
    spline = SplineBezier(ps)
    r = 0.15
    ipos = [
        first_way_point_in_base[0] + 0.1,
        first_way_point_in_base[1],
        first_way_point_in_base[2],
    ]
    irot = first_way_point_in_base[3:]
    rhs0 = [0, 0, 0, 0, 0, 0, 1]
    rhs1 = pin.SE3ToXYZQUAT(
        base_in_first_way_point_se3
        * pin.XYZQUATToSE3([ipos[0], ipos[1], ipos[2]] + irot)
    ).tolist()
    rhs2 = pin.SE3ToXYZQUAT(
        base_in_first_way_point_se3
        * pin.XYZQUATToSE3([ipos[0], ipos[1] - r, ipos[2]] + irot)
    ).tolist()
    rhs3 = pin.SE3ToXYZQUAT(
        base_in_first_way_point_se3
        * pin.XYZQUATToSE3([ipos[0] - r, ipos[1] - r, ipos[2]] + irot)
    ).tolist()
    rhs4 = pin.SE3ToXYZQUAT(
        base_in_first_way_point_se3
        * pin.XYZQUATToSE3([ipos[0] - r, ipos[1], ipos[2]] + irot)
    ).tolist()
    derivative = [[0, 0]] * 6
    p1 = wd(
        spline.createSplinePath(rhs0, rhs1, 1.0, [1, 2], derivative, [1, 2], derivative)
    )
    p2 = wd(
        spline.createSplinePath(rhs1, rhs2, 1.0, [1, 2], derivative, [1, 2], derivative)
    )
    p3 = wd(
        spline.createSplinePath(rhs2, rhs3, 1.0, [1, 2], derivative, [1, 2], derivative)
    )
    p4 = wd(
        spline.createSplinePath(rhs3, rhs4, 1.0, [1, 2], derivative, [1, 2], derivative)
    )
    p5 = wd(
        spline.createSplinePath(rhs4, rhs1, 1.0, [1, 2], derivative, [1, 2], derivative)
    )
    p = p1.asVector()
    p.appendPath(p2)
    p.appendPath(p3)
    p.appendPath(p4)
    p.appendPath(p5)

    # Set this path as the time-varying right hand side of the constraint
    csm.trajectory(p, True)

    # Create goal configuration with final right hand side
    rhs = cp.getRightHandSide()
    rhs[:7] = rhs1[:7]
    cp.setRightHandSide(rhs)
    res, q2 = cp.apply(q1)
    assert res

    # Call steering method
    traj = wd(csm.call(q0, q2))
    if traj:
        ps.client.basic.problem.addPath(traj.asVector())

    sq1, res = traj.call(0.0)
    sq2, res = traj.call(1.0)
    sq3, res = traj.call(2.0)
    sq4, res = traj.call(3.0)
    sq5, res = traj.call(4.0)
    sq6, res = traj.call(5.0)

    # Create spline path for velocity computation in joint space
    spline2 = SplineBezierRobot(ps)
    derivative = robot.getNumberDof() * [[0.0, 0.0]]
    s1 = wd(
        spline2.createSplinePath(sq1, sq2, 1.0, [1, 2], derivative, [1, 2], derivative)
    )
    s2 = wd(
        spline2.createSplinePath(sq2, sq3, 1.0, [1, 2], derivative, [1, 2], derivative)
    )
    s3 = wd(
        spline2.createSplinePath(sq3, sq4, 1.0, [1, 2], derivative, [1, 2], derivative)
    )
    s4 = wd(
        spline2.createSplinePath(sq4, sq5, 1.0, [1, 2], derivative, [1, 2], derivative)
    )
    s5 = wd(
        spline2.createSplinePath(sq5, sq6, 1.0, [1, 2], derivative, [1, 2], derivative)
    )
    s = s1.asVector()
    s.appendPath(s2)
    s.appendPath(s3)
    s.appendPath(s4)
    s.appendPath(s5)

    def discretize_path(dt, p, s, robot, first_way_point_in_base_se3):
        trajectory = []
        duration = p.length()
        pin_model = pin.buildModelFromUrdf(tiago_pro.df_path)
        pin_data = pin_model.createData()
        t0 = 0.0
        for t in np.arange(0, duration, dt):
            p_ee, res = p.call(t)
            assert res
            ee_pose = first_way_point_in_base_se3 * pin.XYZQUATToSE3(p_ee)
            q, res = traj.call(t0 + t)
            assert res
            v = s.derivative(t, 1)
            a = s.derivative(t, 2)
            q = np.array(q)
            v = np.array(v)
            a = np.array(a)
            tau = pin.rnea(pin_model, pin_data, q, v, a).copy()
            rq = robot.rankInConfiguration["tiago/arm_right_1_joint"]
            rv = robot.rankInVelocity["tiago/arm_right_1_joint"]
            trajectory.append(
                TrajectoryPoint(
                    robot_configuration=q[rq : rq + 7],
                    robot_velocity=v[rv : rv + 7],
                    robot_acceleration=a[rv : rv + 7],
                    robot_effort=tau[rv : rv + 7],
                    end_effector_poses={"arm_right_7_joint": ee_pose.copy()},
                )
            )
        return trajectory

    p_reach = p1.asVector()
    p = p2.asVector()
    p.appendPath(p3)
    p.appendPath(p4)
    p.appendPath(p5)

    s_reach = s1.asVector()
    s = s2.asVector()
    s.appendPath(s3)
    s.appendPath(s4)
    s.appendPath(s5)

    reach_traj = discretize_path(
        dt, p_reach, s_reach, robot, first_way_point_in_base_se3
    )
    traj = discretize_path(dt, p, s, robot, first_way_point_in_base_se3)
    return reach_traj, traj
