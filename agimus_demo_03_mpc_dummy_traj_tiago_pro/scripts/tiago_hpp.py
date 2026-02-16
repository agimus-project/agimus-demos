from hpp.corbaserver import loadServerPlugin, wrap_delete as wd
from hpp.corbaserver.manipulation import Client, ProblemSolver, Robot
from hpp.gepetto.manipulation import ViewerFactory
import matplotlib.pyplot as plt
import numpy as np
import pinocchio as pin
import time

from spline import SplineBezier, SplineBezierRobot

# Enable interactive mode at the beginning
plt.ion()


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


Robot.urdfFilename = (
    "package://example-robot-data/robots/tiago_pro_description/robots/tiago_pro.urdf"
)
Robot.srdfFilename = ""

loadServerPlugin("corbaserver", "manipulation-corba.so")
Client().problem.resetProblem()
robot = Robot("tiago-manip", "tiago", rootJointType="anchor")
ps = ProblemSolver(robot)
vf = ViewerFactory(ps)
v = vf.createViewer()

q0 = robot.getCurrentConfig()

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
first_way_point_in_base_se3 = base_in_world_se3.inverse() * first_way_point_in_world_se3
first_way_point_in_base = pin.SE3ToXYZQUAT(first_way_point_in_base_se3).tolist()
base_in_first_way_point_se3 = first_way_point_in_base_se3.inverse()
base_in_first_way_point = pin.SE3ToXYZQUAT(base_in_first_way_point_se3).tolist()
first_way_point_R_base = pin.SE3(first_way_point_in_base_se3.rotation, np.zeros(3))
base_R_first_way_point = pin.SE3ToXYZQUAT(first_way_point_R_base.inverse()).tolist()

ps.client.basic.problem.createTransformationR3xSO3Constraint(
    "ee-pose",
    # "", # in world
    "tiago/base_link",  # in base
    "tiago/arm_right_7_joint",
    first_way_point_in_base,
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
print(f"[ipos irot]: {ipos + irot}")
rhs0 = [0, 0, 0, 0, 0, 0, 1]
rhs1 = pin.SE3ToXYZQUAT(
    base_in_first_way_point_se3 * pin.XYZQUATToSE3([ipos[0], ipos[1], ipos[2]] + irot)
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
print("Way points in base frame:")
print("rhs0:", rhs0)
print("rhs1:", rhs1)
print("rhs2:", rhs2)
print("rhs3:", rhs3)
print("rhs4:", rhs4)
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


def plot_spline_path(p):
    # Plot spline path
    spline_path = np.array([])
    for t in np.arange(0, p.length(), 0.05):
        pos, res = p.call(t)
        assert res
        pos = pin.SE3ToXYZQUAT(first_way_point_in_base_se3 * pin.XYZQUATToSE3(pos))
        spline_path = np.vstack((spline_path, pos)) if spline_path.size else pos

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    ax.plot(
        spline_path[:, 0], spline_path[:, 1], spline_path[:, 2], label="Spline Path"
    )

    # Draw the axis at the first way point
    def plot_frame(axes, se3, length=0.1):
        t = se3.translation
        R = se3.rotation
        axes.quiver(
            t[0],
            t[1],
            t[2],
            R[0, 0],
            R[1, 0],
            R[2, 0],
            color="r",
            length=length,
            normalize=True,
        )
        axes.quiver(
            t[0],
            t[1],
            t[2],
            R[0, 1],
            R[1, 1],
            R[2, 1],
            color="g",
            length=length,
            normalize=True,
        )
        axes.quiver(
            t[0],
            t[1],
            t[2],
            R[0, 2],
            R[1, 2],
            R[2, 2],
            color="b",
            length=length,
            normalize=True,
        )

    plot_frame(ax, first_way_point_in_base_se3, length=0.1)
    plot_frame(ax, first_way_point_in_base_se3 * pin.XYZQUATToSE3(rhs1), length=0.1)
    plot_frame(ax, base_in_world_se3, length=0.1)
    plot_frame(ax, pin.SE3(np.eye(3), np.zeros(3)), length=0.1)
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_title("End-Effector Spline Path")
    ax.legend()
    # Set equal aspect ratio for 3D plot
    ax.set_box_aspect([1, 1, 1])
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
    ax.set_xlim3d([x_middle - max_range / 2, x_middle + max_range / 2])
    ax.set_ylim3d([y_middle - max_range / 2, y_middle + max_range / 2])
    ax.set_zlim3d([z_middle - max_range / 2, z_middle + max_range / 2])
    plt.show()


plot_spline_path(p)

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
s1 = wd(spline2.createSplinePath(sq1, sq2, 1.0, [1, 2], derivative, [1, 2], derivative))
s2 = wd(spline2.createSplinePath(sq2, sq3, 1.0, [1, 2], derivative, [1, 2], derivative))
s3 = wd(spline2.createSplinePath(sq3, sq4, 1.0, [1, 2], derivative, [1, 2], derivative))
s4 = wd(spline2.createSplinePath(sq4, sq5, 1.0, [1, 2], derivative, [1, 2], derivative))
s5 = wd(spline2.createSplinePath(sq5, sq6, 1.0, [1, 2], derivative, [1, 2], derivative))
s = s1.asVector()
s.appendPath(s2)
s.appendPath(s3)
s.appendPath(s4)
s.appendPath(s5)


p_discr = list()
data_plot = DataPlot()
duration = p.length()
t0 = 0.0
dt = 0.05
for t in np.arange(0, duration, dt):
    d = Data()
    ee, res = p.call(t)
    assert res
    d.ee = ee
    d.v_ee = p.derivative(t, 1)
    q, res = traj.call(t0 + t)
    assert res
    d.q = q
    # finite difference for velocity
    # if t == 0.0:
    #     d.v = robot.getNumberDof() * [0.0]
    # else:
    #     d.v = ((np.array(d.q) - np.array(p_discr[-1].q)) / dt).tolist()
    # use spline derivative for velocity
    d.v = s.derivative(t, 1)
    data_plot.q = (
        np.vstack((data_plot.q, np.array(d.q))) if data_plot.q.size else np.array(d.q)
    )
    data_plot.v = (
        np.vstack((data_plot.v, np.array(d.v))) if data_plot.v.size else np.array(d.v)
    )
    data_plot.ee = (
        np.vstack((data_plot.ee, np.array(d.ee)))
        if data_plot.ee.size
        else np.array(d.ee)
    )
    data_plot.v_ee = (
        np.vstack((data_plot.v_ee, np.array(d.v_ee)))
        if data_plot.v_ee.size
        else np.array(d.v_ee)
    )
    p_discr.append(d)
    v(d.q)
    time.sleep(dt)

# use matplotlib to plot data
import matplotlib.pyplot as plt

fig, axs = plt.subplots(4, 1, figsize=(10, 15))
# Joint positions: plot each joint with a consistent color
axs[0].set_title("Joint positions")
joint_colors = []
for i in range(data_plot.q.shape[1]):
    (line,) = axs[0].plot(data_plot.q[:, i], label=f"Joint {i}")
    joint_colors.append(line.get_color())
axs[0].set_ylabel("Position (rad)")
axs[0].legend()

# Joint velocities: use the same color for each joint as above
axs[1].set_title("Joint velocities")
for i in range(data_plot.v.shape[1]):
    axs[1].plot(data_plot.v[:, i], label=f"Joint {i}", color=joint_colors[i])
axs[1].set_ylabel("Velocity (rad/s)")
axs[1].legend()

# End-effector positions: plot each dimension with a consistent color
axs[2].set_title("End-effector positions")
ee_colors = []
for i in range(data_plot.ee.shape[1]):
    (line,) = axs[2].plot(data_plot.ee[:, i], label=f"EE dim {i}")
    ee_colors.append(line.get_color())
axs[2].set_ylabel("Position (m and quat)")
axs[2].legend()

# End-effector velocities: use the same color for each dimension as above
axs[3].set_title("End-effector velocities")
for i in range(data_plot.v_ee.shape[1]):
    axs[3].plot(data_plot.v_ee[:, i], label=f"EE dim {i}", color=ee_colors[i])
axs[3].set_ylabel("Velocity (m/s and quat/s)")
axs[3].legend()

plt.tight_layout()
plt.show()
