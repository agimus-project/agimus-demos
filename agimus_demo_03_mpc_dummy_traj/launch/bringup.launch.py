from launch import LaunchContext, LaunchDescription
from launch.actions import (
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
    DeclareLaunchArgument,
)
from launch.event_handlers import OnProcessExit, OnProcessStart, OnShutdown
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
    safe_remove,
)
from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    franka_robot_launch = generate_include_launch("franka_common_lfc.launch.py")
    ocp_choice_arg = LaunchConfiguration("ocp")
    use_collision_detection = (
        context.perform_substitution(ocp_choice_arg).lower()
        == "custom_with_collision_avoidance"
    )

    arm_id_str = LaunchConfiguration("arm_id").perform(context)

    # Parsing agimus_franka_controllers_params with arm_id replacement
    replacements = {"arm_id": arm_id_str}

    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj"),
            "config",
            "agimus_controller_params.yaml",
        ]
    )
    ocp_definition_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj"),
            "config",
            "ocp_definition_file.yaml",
        ]
    )

    ocp_definition_yaml_file = parse_config(
        path=ocp_definition_yaml.perform(context), replacements=replacements
    )
    agimus_controller_yaml_file = parse_config(
        path=agimus_controller_yaml.perform(context), replacements=replacements
    )

    print(
        f"Temporary mpc dummy file: ocp:{ocp_definition_yaml_file}, agimus controller:{agimus_controller_yaml_file}"
    )

    if use_collision_detection:
        extra_params = {"ocp": {"definition_yaml_file": ocp_definition_yaml_file}}
    else:
        extra_params = {}

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
            agimus_controller_yaml_file,
            extra_params,
        ],
        output="screen",
        remappings=[("robot_description", "robot_description_with_collision")],
    )

    trajectory_weights_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj"),
            "config",
            "trajectory_weigths_params.yaml",
        ]
    )

    trajectory_weights_yaml_file = parse_config(
        path=trajectory_weights_yaml.perform(context), replacements=replacements
    )

    simple_trajectory_publisher_node = Node(
        package="agimus_controller_ros",
        executable="simple_trajectory_publisher",
        parameters=[get_use_sim_time(), trajectory_weights_yaml_file],
        arguments=[
            "-T",
            "4",
            "-A",
            "0.2",
            arm_id_str + "_joint3",
            arm_id_str + "_joint5",
        ],
        output="screen",
    )
    environment_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_03_mpc_dummy_traj"),
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
    tf_node = static_transform_publisher_node(
        frame_id=arm_id_str + "_link0",
        child_frame_id="obstacle1",
    )

    # Cleanup temporary file on shutdown
    cleanup_action = RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event, context: (
                safe_remove(ocp_definition_yaml_file),
                safe_remove(agimus_controller_yaml_file),
                safe_remove(trajectory_weights_yaml_file),
            )
        )
    )
    return [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        tf_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    environment_publisher_node,
                ],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=agimus_controller_node,
                on_start=TimerAction(
                    period=2.0,
                    actions=[simple_trajectory_publisher_node],
                ),
            )
        ),
        cleanup_action,
    ]


def generate_launch_description():
    ocp_choice = DeclareLaunchArgument(
        "ocp",
        default_value="custom_with_collision_avoidance",
        description="Select the ocp to use. Either the default one or the one from this package that does collision avoidance.",
        choices=["default_ocp", "custom_with_collision_avoidance"],
    )
    return LaunchDescription(
        [
            ocp_choice,
        ]
        + generate_default_franka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
