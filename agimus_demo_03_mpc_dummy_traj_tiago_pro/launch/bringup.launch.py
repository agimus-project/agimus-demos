from launch import LaunchContext, LaunchDescription
from launch.actions import (
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
    generate_include_launch,
    get_use_sim_time,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    #
    # Robot
    #
    tiago_robot_launch = generate_include_launch("tiago_pro_common.launch.py")

    #
    # Parameters
    #
    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
            "config",
            "agimus_controller_params.yaml",
        ]
    )
    ocp_definition_file = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
            "config",
            "ocp_definition_file.yaml",
        ]
    )
    extra_params = {
        "ocp": {"definition_yaml_file": ocp_definition_file.perform(context)}
    }
    trajectory_weights_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
            "config",
            "trajectory_weigths_params.yaml",
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
    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[
            get_use_sim_time(),
            agimus_controller_yaml,
            extra_params,
        ],
        output="screen",
        remappings=[("robot_description", "robot_description_with_collision")],
    )
    simple_cartesian_trajectory_publisher_node = Node(
        package="agimus_controller_ros",
        executable="simple_cartesian_trajectory_publisher",
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
                on_exit=[agimus_controller_node],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=agimus_controller_node,
                on_start=TimerAction(
                    period=2.0,
                    actions=[simple_cartesian_trajectory_publisher_node],
                ),
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_tiago_pro_args() + [OpaqueFunction(function=launch_setup)]
    )
