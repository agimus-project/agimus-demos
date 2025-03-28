from launch import LaunchContext, LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from agimus_demos_common.launch_utils import (
    generate_default_tiago_args,
    generate_include_tiago_launch,
    get_use_sim_time,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    tiago_robot_launch = generate_include_tiago_launch("tiago_common_lfc.launch.py")

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
    trajectory_weights_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
            "config",
            "trajectory_weigths_params.yaml",
        ]
    )

    simple_trajectory_publisher_node = Node(
        package="agimus_controller_ros",
        executable="simple_trajectory_publisher",
        parameters=[get_use_sim_time(), trajectory_weights_yaml],
        output="screen",
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
    environment_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="environment_publisher",
        output="screen",
        remappings=[("robot_description", "environment_description")],
        parameters=[{"robot_description": environment_description}],
    )

    def on_tuck_arm_exit_callback(event, context):
        if "tuck_arm.py" in event.process_name:
            if event.returncode == 0:
                return [
                    LogInfo(msg="Starting the MPC controller."),
                    agimus_controller_node,
                    environment_publisher_node,
                    RegisterEventHandler(
                        event_handler=OnProcessStart(
                            target_action=agimus_controller_node,
                            on_start=TimerAction(
                                period=2.0,
                                actions=[simple_trajectory_publisher_node],
                            ),
                        )
                    ),
                ]
            else:
                return [
                    LogInfo(
                        msg="Problem during the initialization, "
                        "PD+ controller not started."
                    )
                ]

    return [
        tiago_robot_launch,
        RegisterEventHandler(OnProcessExit(on_exit=on_tuck_arm_exit_callback)),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_tiago_args() + [OpaqueFunction(function=launch_setup)]
    )
