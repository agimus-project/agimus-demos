from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.event_handlers import OnProcessExit

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_franka_launch,
    get_use_sim_time,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    franka_robot_launch = generate_include_franka_launch("franka_common_lfc.launch.py")
    package_name = "agimus_demo_03_mpc_dummy_traj"

    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "agimus_controller_params.yaml",
        ]
    )

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[get_use_sim_time(), agimus_controller_yaml],
        output="screen",
    )

    simple_trajectory_publisher_node = Node(
        package="agimus_controller_ros",
        executable="simple_trajectory_publisher",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    xacro_path = (
        Path(get_package_share_directory(package_name)) / "urdf" / "obstacles.xacro"
    )
    environment_publisher_node = Node(
        package="agimus_demos_common",
        executable="environment_publisher",
        name="environment_publisher",
        output="screen",
        parameters=[{"environment_path": str(xacro_path)}],
    )

    return [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    simple_trajectory_publisher_node,
                    environment_publisher_node,
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
