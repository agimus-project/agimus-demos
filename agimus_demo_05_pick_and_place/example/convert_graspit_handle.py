import csv
from lxml import etree
import pinocchio as pin


# 1. Create objects for graspit world
def compute_box_inertia(mass_g, width_mm, height_mm, depth_mm):
    """
    Compute inertia matrix for a solid rectangular box aligned with coordinate axes.
    All units are in g and mm (output will be in g·mm², as used by GraspIt).
    """
    # Principal moments of inertia
    Ixx = (1 / 12) * mass_g * (height_mm**2 + depth_mm**2)
    Iyy = (1 / 12) * mass_g * (width_mm**2 + depth_mm**2)
    Izz = (1 / 12) * mass_g * (width_mm**2 + height_mm**2)

    # Since the box is axis-aligned and symmetric, off-diagonal terms are zero
    inertia_matrix = [Ixx, 0.0, 0.0, 0.0, Iyy, 0.0, 0.0, 0.0, Izz]
    return inertia_matrix


# Measurements of objects
# obj_ids = [19, 20, 21, 23, 24, 25, 26, 28, 29, 30]
# masses  = [59.4, 63.7, 80.4, 101.6, 47.8, 97.2, 97.6, 90.9, 100.3, 42.7]
# widths  = [67, 98, 76, 135, 42, 77, 80, 87, 110, 79]
# heights = [53, 46, 45, 50, 70, 53, 60, 46, 57, 79]
# depths  = [45, 50, 57, 50, 42, 60, 50, 87, 75, 50]

# TODO:
obj_ids = ["22", "01", "02"]
masses = [71.8, 61.7, 107.3]
widths = [76, 60, 60]
heights = [58, 30, 40]
depths = [43, 30, 40]

# for obj_id, mass, width, height, depth in zip(obj_ids, masses, widths, heights, depths):
#     inertia = compute_box_inertia(mass, width, height, depth)
#     print("GraspIt! inertia_matrix XML format:")
#     inertia_str = "<inertia_matrix>" + " ".join(f"{val:.2f}" for val in inertia) + "</inertia_matrix>"
#     print(inertia_str)


#     obj_xml = f"""<?xml version="1.0" ?>
#     <root>
#         <material>glass</material>
#         <mass>{mass}</mass>
#         <cog>0 0 0</cog>
#         {inertia_str}
#         <geometryFile type="ply">obj_{obj_id}.ply</geometryFile>
#     </root>"""

#     save_path = f"/home/gepetto/graspit_install/graspit_data/models/objects/tless/obj_alone_{obj_id}.xml"
#     with open(save_path, "w") as f:
#         f.write(obj_xml)

# 2. Convert handles and write to corresponding files


# Function to create a new handle element
def create_handle(name, clearance, xyz, xyzw, link_name="base_link"):
    handle = etree.Element("handle", name=name, clearance=str(clearance))

    _ = etree.SubElement(handle, "position", xyz=xyz, xyzw=xyzw)
    _ = etree.SubElement(handle, "link", name=link_name)

    return handle


for obj_id, width, height, depth in zip(obj_ids, widths, heights, depths):
    srdf_file = f"/home/gepetto/ros2_ws/src/agimus-demos/agimus_demo_05_pick_and_place/srdf/tless/obj_{obj_id}.srdf"
    parser = etree.XMLParser(remove_blank_text=False)
    tree = etree.parse(srdf_file, parser)
    root = tree.getroot()

    with open(
        f"/home/gepetto/graspit_dir/graspit_script/output_tless_obj_{obj_id}.csv",
        newline="",
    ) as csvfile:
        reader = csv.reader(csvfile)
        for i, row in enumerate(reader):
            if i == 0:
                continue
            tx, ty, tz, qx, qy, qz, qw, ax, ay, az = row
            obj_to_hand = pin.XYZQUATToSE3(
                [
                    float(tx),
                    float(ty),
                    float(tz),
                    float(qx),
                    float(qy),
                    float(qz),
                    float(qw),
                ]
            )

            hand_to_gripper = pin.XYZQUATToSE3(
                [0.0, 0.0, 0.103, 0.0, -0.7071067811865476, 0.0, 0.7071067811865476]
            )
            handle = pin.SE3ToXYZQUAT(
                obj_to_hand.inverse() * hand_to_gripper
            )  # if graspit give hand_to_obj
            # handle = pin.SE3ToXYZQUAT(obj_to_hand * hand_to_gripper)  # if graspit give obj_to_hand

            clearance = max(width, height, depth) / 1000  # mm -> m
            handle_elem = create_handle(
                f"graspit{i}",
                clearance,
                f"{handle[0]} {handle[1]} {handle[2]}",
                f"{handle[3]} {handle[4]} {handle[5]} {handle[6]}",
            )

            #             f"{tx} {ty} {tz}",
            # f"{qx} {qy} {qz} {qw}")
            root.append(handle_elem)
            print("i")

    tree.write(srdf_file, pretty_print=True, xml_declaration=True, encoding="utf-8")
