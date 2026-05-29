"""
Bringup launchfile of demo n°10 : TIAGoPro bar bi-manipulation

Usage:
  ros2 launch agimus_demo_10_tiago_pro_bar_manip bringup.launch.py use_gazebo:=true use_sim_time:=True
"""

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
    generate_include_launch,
    get_use_sim_time,
)
from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)
import numpy as np
import pinocchio as pin
from launch.actions import DeclareLaunchArgument

PKG_NAME = "agimus_demo_10_tiago_pro_bar_manip"


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    ref_frame = LaunchConfiguration("ref_frame").perform(context)
    # ==========================================================================
    # Tiago pro simulation
    # ==========================================================================
    tiago_robot_launch = generate_include_launch(
        "tiago_pro_common.launch.py",
        extra_launch_arguments={
            "tuck_arm": "False",
            "end_effector_right": "pal-pro-gripper",
            "end_effector_left": "pal-pro-gripper",
        },
    )

    # ==========================================================================
    # Orchestrator weights and executable
    # ! Launched by hand for now
    # ==========================================================================

    # orchestrator_hpp_config_path = PathJoinSubstitution(
    #     [
    #         FindPackageShare(PKG_NAME),
    #         "config",
    #         "planning",
    #         "orchestrator_hpp_config.yaml",
    #     ]
    # )

    # orchestrator = Node(
    #     package=PKG_NAME,
    #     executable="orchestrator_node",
    #     parameters=[
    #         {"orchestrator_hpp_config": orchestrator_hpp_config_path}
    #     ],
    #     name="orchestrator",
    #     output="screen",
    # )

    # Used to signal the publishing of valid joint values
    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )
    # ==========================================================================
    # Rviz config and executable
    # ==========================================================================

    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare(PKG_NAME),
            "rviz",
            "config.rviz",
        ]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": get_use_sim_time()}],
        arguments=["-d", rviz_config_path.perform(context)],
    )

    # ==========================================================================
    # HPP
    # ==========================================================================

    plate_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare(PKG_NAME),
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
                        FindPackageShare(PKG_NAME),
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
                        FindPackageShare(PKG_NAME),
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
        package=PKG_NAME,
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

    # ==========================================================================
    # Agimus-controller (MPC)
    # ==========================================================================
    # The agimus controller is waiting to receive an environment description
    empty_env_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare(PKG_NAME),
                        "urdf",
                        "environment.urdf",
                    ]
                ),
            ]
        ),
        value_type=str,
    )

    env_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="empty_env_publisher",
        output="screen",
        parameters=[get_use_sim_time(), {"robot_description": empty_env_description}],
        remappings=[("robot_description", "environment_description")],
    )

    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[
            get_use_sim_time(),
            PathJoinSubstitution(
                [
                    FindPackageShare(PKG_NAME),
                    "config",
                    "agimus_controller",
                    "agimus_controller_params.yaml",
                ]
            ),
        ],
        output="screen",
    )

    robot_srdf_publisher_node = Node(
        package="agimus_demos_common",
        executable="string_publisher",
        name="robot_srdf_description_publisher",
        output="screen",
        parameters=[
            {
                "topic_name": "robot_srdf_description",
                "string_value": ParameterValue(
                    Command(
                        [
                            PathJoinSubstitution([FindExecutable(name="xacro")]),
                            " ",
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("agimus_demos_common"),
                                    "config",
                                    "tiago_pro",
                                    "tiago_pro_dummy.srdf.xacro",
                                ]
                            ),
                        ]
                    ),
                    value_type=str,
                ),
            }
        ],
    )

    return [
        tiago_robot_launch,
        rviz,
        wait_for_non_zero_joints_node,
        plate_publisher_node,
        table_publisher,
        bar_reinforcement_publisher,
        tf_odom,
        tf_node_plate,
        tf_node_bar,
        tf_node_table,
        # orchestrator,
        tf_goal_bar,
        robot_srdf_publisher_node,
        agimus_controller_node,
        env_publisher,
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
