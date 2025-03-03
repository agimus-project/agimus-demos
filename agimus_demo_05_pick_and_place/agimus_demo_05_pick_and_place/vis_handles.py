import pinocchio as pin
from robomeshcat import Object, Scene, Robot
from pathlib import Path
import xml.etree.ElementTree as ET
from example_robot_data.robots_loader import PandaLoader as RobLoader


def get_obj_goal_handles(srdf_path: str) -> (list[str], list[str]):
    """Returns the object and goal handles from the srdf file."""
    tree = ET.parse(srdf_path)
    root = tree.getroot()
    all_handles = [handle for handle in root.findall("handle")]

    goal_handles = {
        handle.attrib["name"]: handle.find("position")
        for handle in all_handles
        if "goal" in handle.attrib["name"]
    }
    object_handles = {
        handle.attrib["name"]: handle.find("position")
        for handle in all_handles
        if "goal" not in handle.attrib["name"]
    }
    # object_handles = [handle for handle in all_handles if "goal" not in handle]
    return object_handles, goal_handles


hand_to_gripper = pin.XYZQUATToSE3(
    [0, 0, 0.103, 0, -0.7071067811865476, 0, 0.7071067811865476]
)


scene = Scene()
"Create the first robot and add it to the scene"
rob = Robot(
    urdf_path=RobLoader().df_path,
    mesh_folder_path=Path(RobLoader().model_path).parent.parent,
    name="visual",
)
scene.add_robot(rob)
rmodel = rob._model
rdata = rmodel.createData()
q = pin.randomConfiguration(rmodel)
q[:7] = [0.0, 0.0, -0.3, -1.9, 0.0, 1.9, 0.4]
pin.forwardKinematics(rmodel, rdata, q)
pin.updateFramePlacements(rmodel, rdata)
link_id = rmodel.getFrameId("panda_hand")
ee_pose = rdata.oMf[link_id]

ycbv = Object.create_mesh(
    path_to_mesh=Path(__file__).parent / "urdf/tless/obj_000021/obj_000021.obj",
    scale=1e-3,
    name="ycbv",
)
scene.add_object(ycbv)

object_handles, goal_handles = get_obj_goal_handles(
    str(Path(__file__).parent / "srdf/obj_21.srdf")
)

with scene.animation(fps=1):  # start the animation with the current scene
    rob[:] = q
    for goal_handle in goal_handles:
        pass
        # # Extract the 'xyz' and 'xyzw' attributes
        # xyz = position.get('xyz')
        # xyzw = position.get('xyzw')
    for handle_name, position in object_handles.items():
        print(handle_name)
        xyz = position.get("xyz").strip().split()
        xyzw = position.get("xyzw").strip().split()
        xyz_quat = [float(el) for el in xyz + xyzw]
        # xyz_quat[3:] = [xyz_quat[6]] + xyz_quat[3:6]  # TODO: do i need this?
        handle = pin.XYZQUATToSE3(xyz_quat).inverse()

        obj_pose = ee_pose * hand_to_gripper * handle
        ycbv.pose = obj_pose.homogeneous
        scene.render()

scene.render_image()
