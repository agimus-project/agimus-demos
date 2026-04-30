"""
Bringup launch file for the TIAGo Pro deburring demo — fixed base (right arm only).

Usage:
    ros2 launch agimus_demo_07_fixed_tiago_pro_deburring bringup.launch.py use_gazebo:=true
    ros2 launch agimus_demo_07_fixed_tiago_pro_deburring bringup.launch.py use_gazebo:=true use_hpp_bridge:=true
"""

import os
import yaml

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
    generate_include_launch,
    get_use_sim_time,
)
from agimus_demos_common.mpc_debugger_node import mpc_debugger_node

PKG = "agimus_demo_07_fixed_tiago_pro_deburring"

_pylone_pose_path = os.path.join(
    os.path.dirname(__file__), "..", "config", "pylone_pose.yaml"
)
with open(_pylone_pose_path) as _f:
    _pp = yaml.safe_load(_f)

PYLONE_X = _pp["pylone_x"]
PYLONE_Y = _pp["pylone_y"]
PYLONE_Z = _pp["pylone_z"]


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:

    tiago_robot_launch = generate_include_launch(
        "tiago_pro_common.launch.py",
        extra_launch_arguments={
            "tuck_arm": "False",
            "end_effector_right": "pal-atc",
            "end_effector_left": "pal-pro-gripper",
        },
    )

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    environment_publisher_node = Node(
        package="agimus_demos_common",
        executable="string_publisher",
        name="environment_publisher",
        parameters=[
            {
                "topic_name": "environment_description",
                "string_value": "<robot name='empty'><link name='env'/></robot>",
            }
        ],
    )

    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[
            get_use_sim_time(),
            PathJoinSubstitution(
                [FindPackageShare(PKG), "config", "agimus_controller_params.yaml"]
            ),
        ],
        remappings=[
            ("robot_description_semantic", "robot_srdf_description"),
        ],
        output="screen",
    )

    spawn_pylone_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "pylone",
            "-string", Command([
                FindExecutable(name="xacro"), " ",
                PathJoinSubstitution([FindPackageShare(PKG), "urdf", "pylone.urdf.xacro"]),
            ]),
            "-x", str(PYLONE_X), "-y", str(PYLONE_Y), "-z", str(PYLONE_Z),
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
    )

    mpc_debugger = mpc_debugger_node(
        "gripper_right_tool_holder",
        parent_frame="base_link",
        cost_plot=True,
        node_kwargs=dict(
            condition=IfCondition(LaunchConfiguration("use_mpc_debugger")),
            remappings=[
                ("robot_description_semantic", "robot_srdf_description"),
            ],
        ),
    )

    hpp_bridge_node = ExecuteProcess(
        cmd=[
            "xterm", "-hold", "-T", "HPP orchestrator", "-e",
            "bash -c '"
            "source /opt/ros/humble/setup.bash && "
            "source /home/gepetto/agimus_deps_ws/install/setup.bash && "
            "source /home/gepetto/ros2_ws/install/setup.bash && "
            "source /home/gepetto/hpp_ws/install/setup.bash && "
            f"python3 /home/gepetto/ros2_ws/install/{PKG}/share/{PKG}/hpp/orchestrator_node.py"
            "'",
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_hpp_bridge")),
    )

    return [
        tiago_robot_launch,
        spawn_pylone_node,
        wait_for_non_zero_joints_node,
        environment_publisher_node,
        mpc_debugger,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    hpp_bridge_node,
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_mpc_debugger",
                default_value="false",
                choices=["true", "false"],
            ),
            DeclareLaunchArgument(
                "use_hpp_bridge",
                default_value="false",
                choices=["true", "false"],
                description="Launch the HPP orchestrator shell in xterm.",
            ),
        ]
        + generate_default_tiago_pro_args()
        + [OpaqueFunction(function=launch_setup)]
    )
