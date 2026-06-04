"""
Bringup launch file for the TIAGo Pro deburring demo — fixed base (right arm only).

Usage:
    ros2 launch agimus_demo_07_fixed_tiago_pro_deburring bringup.launch.py use_gazebo:=true
    ros2 launch agimus_demo_07_fixed_tiago_pro_deburring bringup.launch.py use_gazebo:=true use_hpp_bridge:=true
"""

import os
import yaml
import math

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
    PythonExpression,
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

# Convert quaternion [qx, qy, qz, qw] → roll/pitch/yaw for Gazebo spawn
def _quat_to_rpy(q):
    qx, qy, qz, qw = q
    roll  = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = math.asin( 2*(qw*qy - qz*qx))
    yaw   = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return roll, pitch, yaw
_quat = _pp.get("pylone_quat", [0.0, 0.0, 0.0, 1.0])
PYLONE_ROLL, PYLONE_PITCH, PYLONE_YAW = _quat_to_rpy(_quat)


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

    use_mocap = LaunchConfiguration("use_mocap")

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
            # When mocap is enabled, consume corrected trajectory instead of raw one
            ("mpc_input", PythonExpression([
                "'mpc_input_corrected' if '", use_mocap, "' == 'true' else 'mpc_input'"
            ])),
        ],
        output="screen",
    )

    _scripts_dir = PathJoinSubstitution([FindPackageShare(PKG), "scripts"])

    mocap_ee_publisher_node = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([_scripts_dir, "mocap_ee_publisher.py"]),
        ],
        output="screen",
        condition=IfCondition(use_mocap),
    )

    mocap_mpc_corrector_node = ExecuteProcess(
        cmd=[
            "python3",
            PathJoinSubstitution([_scripts_dir, "mocap_mpc_corrector.py"]),
        ],
        output="screen",
        condition=IfCondition(use_mocap),
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
            "-x", str(PYLONE_X),
            "-y", str(PYLONE_Y),
            "-z", str(PYLONE_Z),
            "-R", str(PYLONE_ROLL),
            "-P", str(PYLONE_PITCH),
            "-Y", str(PYLONE_YAW),
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
        mocap_ee_publisher_node,
        mocap_mpc_corrector_node,
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
                "use_mocap",
                default_value="false",
                choices=["true", "false"],
                description="Enable Qualisys mocap: publishes mocap_ee TF and corrects MPC targets.",
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
