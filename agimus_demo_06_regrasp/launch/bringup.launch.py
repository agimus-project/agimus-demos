from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_launch,
    get_use_sim_time,
    parse_config,
)

from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)
import os

def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    franka_robot_launch = generate_include_launch("franka_common_lfc.launch.py")
    arm_id_str = LaunchConfiguration("arm_id").perform(context)

    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_06_regrasp"),
            "config",
            "agimus_controller_params.yaml",
        ]
    )
    # Parsing franka_controllers_params with arm_id replacement
    replacements = {
        'arm_id': arm_id_str,
    }
    agimus_controller_yaml_file = parse_config(path=agimus_controller_yaml.perform(context), replacements=replacements)
    print(f"Using agimus_controller file: {agimus_controller_yaml_file}")
    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )
    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[get_use_sim_time(), agimus_controller_yaml_file],
        output="screen",
        remappings=[("robot_description", "robot_description_with_collision")],
    )

    environment_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_06_regrasp"),
                        "urdf",
                        "environment.urdf.xacro",
                    ]
                ),
            ]
        ),
        value_type=str,
    )
    environment_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="environment_publisher",
        output="screen",
        remappings=[("robot_description", "environment_description")],
        parameters=[{"robot_description": environment_description}],
    )
    tf_node = static_transform_publisher_node(
        frame_id="robot_attachment_link",
        child_frame_id="world",
    )

    regrasp_node = ExecuteProcess(
        cmd=[
            "xterm",
            "-hold",
            "-e",
            'bash -c "source /opt/ros/humble/setup.bash && '
            f'ros2 run agimus_demo_06_regrasp regrasp_node --ros-args -p arm_id:={arm_id_str}"',
        ],
        output="screen",
    )
    # Cleanup temporary file on shutdown
    cleanup_action = RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event, context: (
                os.remove(agimus_controller_yaml_file),
            )
        )
    )
    return [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        environment_publisher_node,
        tf_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    regrasp_node,
                ],
            )
        ),
        cleanup_action,
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
