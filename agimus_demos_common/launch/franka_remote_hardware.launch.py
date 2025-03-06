import jinja2
from pathlib import Path
from tempfile import NamedTemporaryFile

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


from launch.substitutions import EnvironmentVariable


def evaluate_compose_template(
    context: LaunchContext, template_path_join_substitution: PathJoinSubstitution
) -> Path:
    arm_id = LaunchConfiguration("arm_id")
    robot_ip = LaunchConfiguration("robot_ip")
    rmw_implementation = EnvironmentVariable("RMW_IMPLEMENTATION")
    ros_domain_id = EnvironmentVariable("ROS_DOMAIN_ID")

    # Load jinja template for compose.yaml file
    template_path = Path(context.perform_substitution(template_path_join_substitution))
    environment = jinja2.Environment()
    template = environment.get_template(template_path)

    # Render the compose.yaml file with proper values
    content = template.render(
        {
            "arm_id": context.perform_substitution(arm_id),
            "robot_ip": context.perform_substitution(robot_ip),
            "rmw_implementation": context.perform_substitution(rmw_implementation),
            "ros_domain_id": context.perform_substitution(ros_domain_id),
        }
    )

    # Save new, rendered compose.yaml file as a temporary file on this machine
    param_file_path = Path()
    with NamedTemporaryFile(mode="w", prefix="compose_realtime_", delete=False) as f:
        f.write(content)
        param_file_path = Path(f.name)
    return param_file_path


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    aux_computer_ip = LaunchConfiguration("aux_computer_ip")
    aux_computer_user = LaunchConfiguration("aux_computer_user")

    aux_computer_ip = context.perform_substitution(aux_computer_ip)
    aux_computer_user = context.perform_substitution(aux_computer_user)

    linear_feedback_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demos_common"),
            "config",
            "linear_feedback_controller_params.yaml",
        ]
    )

    compose_rt_computer_template = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demos_common"),
            "config",
            "compose.rt_computer.yaml.jinja",
        ]
    )

    # Evaluate jinja template of the compose.yaml file
    compose_rt_computer_path = evaluate_compose_template(
        context, compose_rt_computer_template
    )

    remote = f"{aux_computer_user}@{aux_computer_ip}"
    # Register list of commands to perform on bringup
    bringup_remote_commands = [
        # Send parameters of LFC to RT computer
        [
            "scp",
            linear_feedback_controller_params,
            f"{remote}:/tmp/linear_feedback_controller_params.yaml",
        ],
        # Send rendered compose.yaml file to RT computer
        [
            "scp",
            compose_rt_computer_path,
            f"{remote}:/tmp/compose.yaml",
        ],
        # Start docker on the RT computer
        [
            "ssh",
            remote,
            "'bash -s 'docker compose up -f /tmp/compose.yaml'",
        ],
    ]
    bringup_processes = [
        ExecuteProcess(
            cmd=cmd,
            output="screen",
        )
        for cmd in bringup_remote_commands
    ]
    # Chain commands to execute them in order
    bringup_events = [
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=bringup_processes[i],
                on_exit=[bringup_processes[i + 1]],
            )
        )
        for i in range(1, len(bringup_processes) - 1)
    ]

    # Register commands to execute on the shutdown of the system
    shutdown_command = [
        # Stop docker container on the RT computer
        "ssh",
        remote,
        "'bash -s 'docker compose down -f /tmp/compose.yaml'",
    ]
    shutdown_process = ExecuteProcess(
        cmd=shutdown_command,
        output="screen",
    )
    shutdown_event = RegisterEventHandler(OnShutdown(on_shutdown=shutdown_process))

    return [
        bringup_processes[0],
        *bringup_events,
        shutdown_event,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            description="Hostname or IP address of the robot.",
        ),
        DeclareLaunchArgument(
            "aux_computer_ip",
            description="Hostname or IP address of the auxiliary computer "
            + "with real-time kernel. If not empty launch file is configured "
            + "to spawn docker container on that machine. If empty, controllers "
            + "are spawned locally on the computer executing launch file.",
        ),
        DeclareLaunchArgument(
            "aux_computer_user",
            description="Username used to execute commands on auxiliary computer over ssh. "
            + "Required if `aux_computer_ip` is not empty.",
        ),
        DeclareLaunchArgument(
            "arm_id",
            default_value="fer",
            description="ID of the type of arm used. Supported values: fer, fr3, fp3",
            choices=["fer", "fr3", "fp3"],
        ),
        DeclareLaunchArgument(
            "franka_controllers_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "franka_controllers.yaml",
                ]
            ),
            description="Path to the yaml file used to define controller parameters.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
