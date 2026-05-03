from pathlib import Path
from tempfile import NamedTemporaryFile

import jinja2
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.event_handlers import (
    OnProcessExit,
    OnProcessIO,
    OnShutdown,
)
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    COMPOSE_REMOTE_PATH,
    EXTERNAL_CONTROLLERS_PARAMS_REMOTE_PATH,
    FRANKA_PARAMS_REMOTE_PATH,
)


def evaluate_compose_template(
    context: LaunchContext, template_path_join_substitution: PathJoinSubstitution
) -> Path:
    arm_id = LaunchConfiguration("arm_id")
    robot_ip = LaunchConfiguration("robot_ip")
    external_controllers_names = LaunchConfiguration("external_controllers_names")
    rmw_implementation = EnvironmentVariable("RMW_IMPLEMENTATION")
    ros_domain_id = EnvironmentVariable("ROS_DOMAIN_ID", default_value="0")

    # Load jinja template for compose.yaml file
    template_path = Path(context.perform_substitution(template_path_join_substitution))
    environment = jinja2.Environment(
        loader=jinja2.FileSystemLoader(searchpath=template_path.parents[0])
    )
    template = environment.get_template(template_path.name)

    # Render the compose.yaml file with proper values
    content = template.render(
        {
            "arm_id": context.perform_substitution(arm_id),
            "franka_params_remote_path": FRANKA_PARAMS_REMOTE_PATH.as_posix(),
            "external_controllers_params_remote_path": EXTERNAL_CONTROLLERS_PARAMS_REMOTE_PATH.as_posix(),
            # Add quotes around the list to make it look as "['val_0', 'val_1]" so it is recognized as ROS param
            "external_controllers_names": f'"{context.perform_substitution(external_controllers_names)}"',
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
    external_controllers_params = LaunchConfiguration("external_controllers_params")
    agimus_franka_controllers_params = LaunchConfiguration(
        "agimus_franka_controllers_params"
    )

    aux_computer_ip_str = context.perform_substitution(aux_computer_ip)
    aux_computer_user_str = context.perform_substitution(aux_computer_user)
    external_controllers_params_str = context.perform_substitution(
        external_controllers_params
    )

    compose_rt_computer_template = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demos_common"),
            "templates",
            "compose.yaml.jinja",
        ]
    )

    # Evaluate jinja template of the compose.yaml file
    compose_rt_computer_path = evaluate_compose_template(
        context, compose_rt_computer_template
    )

    remote = f"{aux_computer_user_str}@{aux_computer_ip_str}"
    # Register list of commands to perform on bringup
    bringup_remote_commands = [
        # Check if host is available
        {
            "name": "aux-ping",
            "cmd": ["ping", "-c1", aux_computer_ip_str],
            "on_exit": None,
            "skip_step": False,
        },
        # Check if ssh keys were exchanged
        {
            "name": "aux-ssh-check-keys",
            "cmd": ["ssh", "-o", "BatchMode=yes", remote, "'exit'"],
            "on_exit": None,
            "skip_step": False,
        },
        # Send parameters of Franka controller to RT computer
        {
            "name": "aux-scp-franka-params",
            "cmd": [
                "scp",
                agimus_franka_controllers_params,
                f"{remote}:{FRANKA_PARAMS_REMOTE_PATH.as_posix()}",
            ],
            "on_exit": None,
            "skip_step": False,
        },
        # Send parameters of external controllers to RT computer
        {
            "name": "aux-scp-external-controller-params",
            "cmd": [
                "scp",
                external_controllers_params,
                f"{remote}:{EXTERNAL_CONTROLLERS_PARAMS_REMOTE_PATH.as_posix()}",
            ],
            "on_exit": None,
            # If no file is passed, skip this step
            "skip_step": external_controllers_params_str == "",
        },
        # Send rendered compose.yaml file to RT computer
        {
            "name": "aux-scp-compose",
            "cmd": [
                "scp",
                compose_rt_computer_path.absolute().as_posix(),
                f"{remote}:{COMPOSE_REMOTE_PATH.as_posix()}",
            ],
            "on_exit": None,
            "skip_step": False,
        },
        # Start docker on the RT computer
        {
            "name": "aux-compose-up",
            "cmd": [
                "ssh",
                remote,
                "-t",
                f'"/bin/bash -c \\"docker compose -f {COMPOSE_REMOTE_PATH.as_posix()} up --force-recreate\\""',
            ],
            # If docker container is stopped (eg. hardware error or e-stop)
            # Propagate this stop to the rest of the system on the main computer
            "on_exit": None,  # Shutdown(),
            "skip_step": False,
        },
    ]
    bringup_processes = [
        ExecuteProcess(
            name=cmd["name"],
            cmd=cmd["cmd"],
            output="both",
            shell=True,
            emulate_tty=True,
            log_cmd=True,
            on_exit=cmd["on_exit"],
        )
        for cmd in bringup_remote_commands
        if not cmd["skip_step"]
    ]
    # Chain commands to execute them in order
    bringup_events = [
        [
            # Chain next command after previous finished correctly
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=bringup_processes[i],
                    on_exit=[bringup_processes[i + 1]],
                )
            ),
            # In case stderr is present stop launch (there is no way to capture return code)
            RegisterEventHandler(
                event_handler=OnProcessIO(
                    target_action=bringup_processes[i],
                    on_stderr=[Shutdown()],
                )
            ),
        ]
        for i in range(len(bringup_processes) - 1)
    ]

    # Register commands to execute on the shutdown of the system
    shutdown_command = {
        # Stop docker container on the RT computer
        "name": "aux-compose-down",
        "cmd": [
            "ssh",
            remote,
            "-t",
            f'"/bin/bash -c \\"docker compose -f {COMPOSE_REMOTE_PATH.as_posix()} '
            + 'down --remove-orphans\\""',
        ],
    }

    shutdown_process = ExecuteProcess(
        name=shutdown_command["name"],
        cmd=shutdown_command["cmd"],
        output="both",
        shell=True,
        emulate_tty=True,
        log_cmd=True,
    )
    shutdown_event = RegisterEventHandler(OnShutdown(on_shutdown=[shutdown_process]))

    return [
        bringup_processes[0],
        # Flatten list of lists
        *[be for bes in bringup_events for be in bes],
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
            "external_controllers_params",
            default_value="",
            description="Path to the yaml file use to define external controllers parameters.",
        ),
        DeclareLaunchArgument(
            "external_controllers_names",
            default_value="['']",
            description="Names of the external controllers to spawn.",
        ),
        DeclareLaunchArgument(
            "agimus_franka_controllers_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "franka",
                    "franka_controllers.yaml",
                ]
            ),
            description="Path to the yaml file used to define controller parameters.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
