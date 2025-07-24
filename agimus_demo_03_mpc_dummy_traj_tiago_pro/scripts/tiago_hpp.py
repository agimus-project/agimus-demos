from math import pi
from hpp.corbaserver import loadServerPlugin, wrap_delete as wd
from hpp.corbaserver.manipulation import Client, ProblemSolver, Robot
from hpp.gepetto.manipulation import ViewerFactory
from spline import SplineBezier

Robot.urdfFilename = (
    "package://example-robot-data/robots/tiago_pro_description/robots/tiago_pro.urdf"
)
Robot.srdfFilename = ""

loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()
robot = Robot("tiago-manip", "tiago", rootJointType="anchor")
ps = ProblemSolver(robot)
vf = ViewerFactory(ps)

q0 = robot.getCurrentConfig()

# Lock all joint except left arm in neutral positions
lockedJoints = list(
    filter(lambda j: not j.startswith("tiago/arm_left"), robot.jointNames)
)

lockedJointNames = list()
for j in lockedJoints:
    r = robot.rankInConfiguration[j]
    size = robot.getJointConfigSize(j)
    name = f"locked_{j}"
    lockedJointNames.append(name)
    ps.createLockedJoint(name, j, q0[r : r + size])
    ps.setConstantRightHandSide(name, True)

# Fold left arm to move away from singularity
r = robot.rankInConfiguration["tiago/arm_left_2_joint"]
q0[r] = pi / 4
q0[r + 1] = pi / 4
q0[r + 2] = -pi / 2
q0[r + 3] = -pi / 4
q0[r + 4] = pi / 2

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
ps.client.basic.problem.createTransformationR3xSO3Constraint(
    "ee-pose",
    "",
    "tiago/arm_left_6_joint",
    [
        0.5196730441033661,
        0.034650161767261184,
        0.9812741725898027,
        -0.6206963609757624,
        0.674163364588632,
        -0.13794947533411075,
        -0.3757788279999625,
    ],
    #    [0.7067601889929528, 0.28711291398934, 0.829846210341564,
    #     0.5742710231206255, -0.4542156045558627, 0.45517856289111874, 0.5066689772093018],
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
rhs0 = 6 * [0] + [1]
rhs1 = [0, 0, 0.2, 0, 0, 0, 1]
rhs2 = [0, 0.2, 0.2, 0, 0, 0, 1]
rhs3 = [0, 0.2, 0, 0, 0, 0, 1]
derivative = [[0], [0], [0], [0], [0], [0]]
p1 = wd(spline.createSplinePath(rhs0, rhs1, 1.0, [1], derivative, [1], derivative))
p2 = wd(spline.createSplinePath(rhs1, rhs2, 1.0, [1], derivative, [1], derivative))
p3 = wd(spline.createSplinePath(rhs2, rhs3, 1.0, [1], derivative, [1], derivative))
p4 = wd(spline.createSplinePath(rhs3, rhs0, 1.0, [1], derivative, [1], derivative))
p = p1.asVector()
p.appendPath(p2)
p.appendPath(p3)
p.appendPath(p4)
# Set this path as the time-varying right hand side of the constraint
csm.trajectory(p, True)

# Create goal configuration with final right hand side
rhs = cp.getRightHandSide()
rhs[:7] = p.end()
cp.setRightHandSide(rhs)
res, q2 = cp.apply(q1)
# assert(res)
# Call steering method
p1 = wd(csm.call(q1, q1))
if p1:
    ps.client.basic.problem.addPath(p1.asVector())
