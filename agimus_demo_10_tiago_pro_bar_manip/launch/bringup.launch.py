from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable

from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
    get_use_sim_time,
)
from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)
import numpy as np
import pinocchio as pin
from launch.actions import DeclareLaunchArgument


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    ref_frame = LaunchConfiguration("ref_frame").perform(context)

    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_10_tiago_pro_bar_manip"),
            "rviz",
            "config.rviz",
        ]
    )
    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )
    orchestrator = Node(
        package="agimus_demo_10_tiago_pro_bar_manip",
        executable="orchestrator_node",
        name="orchestrator",
        output="screen",
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": get_use_sim_time()}],
        arguments=["-d", rviz_config_path.perform(context)],
    )

    plate_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_10_tiago_pro_bar_manip"),
                        "urdf",
                        "plate.urdf",
                    ]
                ),
            ]
        ),
        value_type=str,
    )
    plate_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="plate_publisher",
        output="screen",
        parameters=[get_use_sim_time(), {"robot_description": plate_description}],
        remappings=[("robot_description", "plate_description")],
    )
    bar_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_10_tiago_pro_bar_manip"),
                        "urdf",
                        "reinforcement_bar.urdf",
                    ]
                ),
            ]
        ),
        value_type=str,
    )
    bar_reinforcement_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="bar_publisher",
        output="screen",
        parameters=[get_use_sim_time(), {"robot_description": bar_description}],
        remappings=[("robot_description", "bar_description")],
    )

    table_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_10_tiago_pro_bar_manip"),
                        "urdf",
                        "table.urdf",
                    ]
                ),
            ]
        ),
        value_type=str,
    )
    table_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="table_publisher",
        output="screen",
        parameters=[get_use_sim_time(), {"robot_description": table_description}],
        remappings=[("robot_description", "table_description")],
    )

    tf_odom = Node(
        package="agimus_demo_10_tiago_pro_bar_manip",
        executable="tf_base_publisher",
        name="tf_base_publisher",
        output="screen",
    )

    tf_node_plate = static_transform_publisher_node(
        frame_id=f"{ref_frame}",
        child_frame_id="plate_base_link",
        xyz=[2.3, 0, 0.66],
        rot_xyzw=["0", "0", "0", "1"],
    )
    quat_values = pin.Quaternion(pin.rpy.rpyToMatrix(np.array([np.pi / 2, -0.6, 0])))

    tf_node_bar = static_transform_publisher_node(
        frame_id=f"{ref_frame}",
        child_frame_id="bar_base_link",
        xyz=["1.6", "0", "0.67"],
        rot_xyzw=quat_values.coeffs().tolist(),  # [x, y, z, w]
    )

    quat_values = pin.Quaternion(pin.rpy.rpyToMatrix(np.array([0, 0, -np.pi / 2])))

    tf_goal_bar = static_transform_publisher_node(
        frame_id="plate_base_link",
        child_frame_id="bar_goal_pose",
        xyz=["0", "0.4", "0.01"],
        rot_xyzw=quat_values.coeffs().tolist(),  # [x, y, z, w]
    )

    quat_values = pin.Quaternion(pin.rpy.rpyToMatrix(np.array([0, 0, np.pi])))

    tf_node_table = static_transform_publisher_node(
        frame_id=f"{ref_frame}",
        child_frame_id="table_link",
        xyz=["2.9", "0", "0."],
        rot_xyzw=quat_values.coeffs().tolist(),
    )
    # TODO we should get the srdf use by hpp from a topic directly
    # robot_srdf_description =  ParameterValue(
    #     Command(
    #         [
    #             PathJoinSubstitution([FindExecutable(name="xacro")]),
    #             " ",
    #             PathJoinSubstitution(
    #                 [
    #                     FindPackageShare("tiago_pro_moveit_config"),
    #                     "config/srdf",
    #                     "tiago_pro.srdf.xacro",
    #                 ]
    #             ),
    #             " ",
    #             "end_effector_left:=pal-pro-gripper",
    #             " ",
    #             "end_effector_right:=pal-pro-gripper",
    #             " ",
    #         ]
    #     ),
    #     value_type=str,
    # )
    # robot_collision_publisher = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="robot_collision_publisher",
    #     output="screen",
    #     parameters=[get_use_sim_time(), {"robot_description": robot_srdf_description}],
    #     remappings=[("robot_description", "robot_description_collision")],
    # )

    return [
        rviz,
        wait_for_non_zero_joints_node,
        plate_publisher_node,
        table_publisher,
        bar_reinforcement_publisher,
        tf_odom,
        tf_node_plate,
        tf_node_bar,
        tf_node_table,
        orchestrator,
        tf_goal_bar,
    ]


def generate_launch_description():
    args = generate_default_tiago_pro_args()
    args.append(
        DeclareLaunchArgument(
            "ref_frame",
            default_value="base_link",
            description="Reference frame for the demo (world, map, odom...)",
        )
    )
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
