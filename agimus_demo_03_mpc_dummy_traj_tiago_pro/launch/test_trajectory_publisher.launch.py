from launch import LaunchContext, LaunchDescription
from launch.actions import (
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
    get_use_sim_time,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    #
    # Robot
    #
    tiago_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("tiago_pro_description"),
                        "launch",
                        "show.launch.py",
                    ]
                )
            ]
        )
    )

    #
    # Parameters
    #
    trajectory_weights_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
            "config",
            "trajectory_weights_params.yaml",
        ]
    )
    environment_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
                        "urdf",
                        "obstacles.xacro",
                    ]
                ),
                # Convert dict to list of parameters
            ]
        ),
        value_type=str,
    )

    #
    # Nodes
    #
    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )
    hpp_trajectory_publisher_node = Node(
        package="agimus_controller_ros",
        executable="simple_trajectory_publisher",
        name="simple_trajectory_publisher",
        parameters=[get_use_sim_time(), trajectory_weights_yaml],
        output="screen",
    )
    environment_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="environment_publisher",
        output="screen",
        remappings=[("robot_description", "environment_description")],
        parameters=[{"robot_description": environment_description}],
    )

    environment_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
                        "urdf",
                        "obstacles.xacro",
                    ]
                ),
                # Convert dict to list of parameters
            ]
        ),
        value_type=str,
    )

    return [
        tiago_robot_launch,
        wait_for_non_zero_joints_node,
        environment_publisher_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[simple_trajectory_publisher_node],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_tiago_pro_args() + [OpaqueFunction(function=launch_setup)]
    )
