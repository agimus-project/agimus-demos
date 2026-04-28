"""
Bringup launch file for the TIAGo Pro deburring demo (simulation).

Usage:
    ros2 launch agimus_demo_07_tiago_pro_deburring bringup.launch.py use_gazebo:=true
    ros2 launch agimus_demo_07_tiago_pro_deburring bringup.launch.py use_gazebo:=true use_hpp_bridge:=true
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

PKG = "agimus_demo_07_tiago_pro_deburring"

_cfg_path = os.path.join(
    os.path.dirname(__file__), "..", "config", "hpp_orchestrator_params.yaml"
)
with open(_cfg_path) as _f:
    _cfg = yaml.safe_load(_f)

_s = _cfg["scene"]
TABLE_X = _s["table_x"]
TABLE_Y = _s["table_y"]
TABLE_Z = _s["table_z"]
PYLONE_X = TABLE_X
PYLONE_Y = TABLE_Y
PYLONE_Z = _s["pylone_z_offset"]


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:

    tiago_robot_launch = generate_include_launch(
        "tiago_pro_common.launch.py",
        extra_launch_arguments={"tuck_arm": "False"},
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

    base_cmd_bridge_node = Node(
        package=PKG,
        executable="base_cmd_bridge",
        name="base_cmd_bridge",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    spawn_table_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "table",
            "-file", PathJoinSubstitution([FindPackageShare(PKG), "urdf", "table.urdf"]),
            "-x", str(TABLE_X), "-y", str(TABLE_Y), "-z", str(TABLE_Z),
        ],
        output="screen",
    )

    spawn_pylone_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "pylone",
            "-file", PathJoinSubstitution([FindPackageShare(PKG), "urdf", "pylone.urdf"]),
            "-x", str(PYLONE_X), "-y", str(PYLONE_Y), "-z", str(PYLONE_Z),
        ],
        output="screen",
    )

    mpc_debugger = mpc_debugger_node(
        "gripper_right_fingertip_left_link",
        parent_frame="odom",
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
        spawn_table_node,
        spawn_pylone_node,
        wait_for_non_zero_joints_node,
        environment_publisher_node,
        mpc_debugger,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    base_cmd_bridge_node,
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
