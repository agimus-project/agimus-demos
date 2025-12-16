def displayGripper(v, name):
    """
    Display gripper in gepetto-gui
    Retrieve the joint and pose information of the gripper in the robot model
    and display a frame.
    """
    robot = v.robot
    joint, pose = robot.getGripperPositionInJoint(name)
    gname = "gripper__" + name.replace("/", "_")
    v.client.gui.addXYZaxis(gname, [0, 1, 0, 1], 0.005, 0.015)
    if joint != "universe":
        link = robot.getLinkNames(joint)[0]
        v.client.gui.addToGroup(gname, robot.name + "/" + link)
    else:
        v.client.gui.addToGroup(gname, robot.name)
    v.client.gui.applyConfiguration(gname, pose)


def displayHandle(v, name):
    """
    Display handle in gepetto-gui
    Retrieve the joint and pose information of the handle in the robot model
    and display a frame.
    """
    robot = v.robot
    joint, pose = robot.getHandlePositionInJoint(name)
    hname = "handle__" + name.replace("/", "_")
    v.client.gui.addXYZaxis(hname, [0, 1, 0, 1], 0.005, 0.015)
    if joint != "universe":
        link = robot.getLinkNames(joint)[0]
        v.client.gui.addToGroup(hname, robot.name + "/" + link)
    else:
        v.client.gui.addToGroup(hname, robot.name)
    v.client.gui.applyConfiguration(hname, pose)
