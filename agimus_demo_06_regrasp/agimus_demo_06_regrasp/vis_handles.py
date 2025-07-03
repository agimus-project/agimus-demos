"""
Run this script outside of docker since `pip install robomeshcat` breaks the env
"""

import pinocchio as pin
from robomeshcat import Object, Scene, Robot
from pathlib import Path

# import xml.etree.ElementTree as ET
from example_robot_data.robots_loader import PandaLoader as RobLoader
import coal
from lxml import etree


def get_obj_goal_handles(srdf_path: str) -> (list[str], list[str]):
    """Returns the object and goal handles from the srdf file."""
    parser = etree.XMLParser(remove_blank_text=False)
    tree = etree.parse(srdf_path, parser)
    # tree = ET.parse(srdf_path)
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
    return object_handles, goal_handles, tree


def load_convex_mesh(file_name: str):
    loader = coal.MeshLoader()
    bvh = loader.load(file_name)
    bvh.buildConvexHull(True, "Qt")
    return bvh.convex


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
# coal_robot_model = pin.RobotWrapper.BuildFromURDF(RobLoader().df_path, Path(RobLoader().model_path).parent.parent, pin.JointModelFreeFlyer())
scene.add_robot(rob)
rmodel = rob._model
rdata = rmodel.createData()
q = pin.randomConfiguration(rmodel)
q[:9] = [0.0, 0.0, -0.3, -1.9, 0.0, 1.9, 0.4, 0.039, 0.039]
pin.forwardKinematics(rmodel, rdata, q)
pin.updateFramePlacements(rmodel, rdata)
link_id = rmodel.getFrameId("panda_hand")
ee_pose = rdata.oMf[link_id]
object_name = "obj_23"
ycbv = Object.create_mesh(
    path_to_mesh=Path(__file__).parent.parent / f"urdf/tless/{object_name}.obj",
    scale=1e-3,
    name="ycbv",
)
# coal_robot = coal.CollisionRobot(rmodel)
# coal_robot.updateGeometry(q)
object_mesh = load_convex_mesh(
    str(Path(__file__).parent.parent / f"urdf/tless/{object_name}.obj")
)
# object_mesh.scale(0.001)
for i, geom_obj in enumerate(rob._geom_model.geometryObjects):
    if "panda_hand" in geom_obj.name:
        # mesh_path = geom_obj.geometry.meshPath
        print(geom_obj.meshPath)
        hand_geom = load_convex_mesh(geom_obj.meshPath)


scene.add_object(ycbv)

object_handles, goal_handles, tree = get_obj_goal_handles(
    str(Path(__file__).parent.parent / f"srdf/tless/{object_name}.srdf")
)
handles_to_remove = []

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
        T_robot = coal.Transform3s()
        T_robot.setTranslation(ee_pose.translation)
        T_robot.setRotation(ee_pose.rotation)

        T_object = coal.Transform3s()
        T_object.setTranslation(obj_pose.translation)
        T_object.setRotation(obj_pose.rotation)

        # Define collision request and result
        col_req = coal.CollisionRequest()
        col_res = coal.CollisionResult()

        # Perform collision check
        print(type(rob._geom_model))
        print(type(object_mesh))
        coal.collide(hand_geom, T_robot, object_mesh, T_object, col_req, col_res)

        # Check and print collision result
        if col_res.isCollision():
            contact = col_res.getContact(0)
            print(f"{handle_name} Collision detected!")
            print(f"Penetration depth: {contact.penetration_depth}")
            print(
                f"Distance including security margin: {contact.penetration_depth + col_req.security_margin}"
            )
            print(f"Witness point on shape1: {contact.getNearestPoint1()}")
            print(f"Witness point on shape2: {contact.getNearestPoint2()}")
            print(f"Normal: {contact.normal}")
            handles_to_remove.append(handle_name)
        else:
            print("No collision detected.")

print(handles_to_remove)
scene.render_image()
# for handle in tree.findall("handle"):
#     if handle.attrib.get("name") in handles_to_remove:
#         tree.remove(handle)

# tree.write(str(Path(__file__).parent.parent / f"srdf/tless/{object_name}.srdf"), pretty_print=True, xml_declaration=True, encoding='utf-8')
