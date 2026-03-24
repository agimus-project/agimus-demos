from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_launch,
    get_use_sim_time,
    parse_config,
    safe_remove,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    franka_robot_launch = generate_include_launch("franka_common_lfc.launch.py")
    arm_id = LaunchConfiguration("arm_id")
    pd_plus_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_02_simple_pd_plus"),
            "config",
            "pd_plus_controller_params.yaml",
        ]
    )

    # Parsing agimus_franka_controllers_params with arm_id replacement
    replacements = {"arm_id": arm_id.perform(context)}
    pd_plus_controller_params = parse_config(
        path=pd_plus_controller_params.perform(context), replacements=replacements
    )
    # Cleanup temporary file on shutdown
    cleanup_action = RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event, context: (safe_remove(pd_plus_controller_params),)
        )
    )
    print(f"Temporary pd_plus_controller_params file: {pd_plus_controller_params}")

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    pd_plus_controller_node = Node(
        package="linear_feedback_controller",
        executable="pd_plus_controller",
        parameters=[get_use_sim_time(), pd_plus_controller_params],
        output="screen",
    )

    return [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[pd_plus_controller_node],
            )
        ),
        cleanup_action,
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
