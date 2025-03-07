from pathlib import Path

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# Constants used to synchronize paths expected on remote between launch files
LFC_PARAMS_REMOTE_PATH: Path = Path("/tmp/linear_feedback_controller_params.yaml")
COMPOSE_REMOTE_PATH: Path = Path("/tmp/compose.yaml")


def generate_default_franka_args() -> list[DeclareLaunchArgument]:
    """Generates list of default arguments expected to be public interface of
        launch files used by Agimus Demos for Franka robots.

    Returns:
        list[DeclareLaunchArgument]: List of DeclareLaunchArgument objects with
            arguments expected by `franka_common.launch.py` and `franka_common_lfc.launch.py`
    """
    return [
        DeclareLaunchArgument(
            "robot_ip",
            default_value="",
            description="Hostname or IP address of the robot. "
            + "If not empty launch file is configured for real robot. "
            + "If empty `use_gazebo` is expected to be set to `true`.",
        ),
        DeclareLaunchArgument(
            "aux_computer_ip",
            default_value="",
            description="Hostname or IP address of the auxiliary computer "
            + "with real-time kernel. If not empty launch file is configured "
            + "to spawn docker container on that machine. If empty, controllers "
            + "are spawned locally on the computer executing launch file.",
        ),
        DeclareLaunchArgument(
            "aux_computer_user",
            default_value="",
            description="Username used to execute commands on auxiliary computer over ssh. "
            + "Required if `aux_computer_ip` is not empty.",
        ),
        DeclareLaunchArgument(
            "on_aux_computer",
            default_value="false",
            description="Whether launch file is executed on auxiliary computer. "
            + "If set to `true`, `robot_ip` can not be empty and only minimal "
            + "set of nodes to control the robot is launched on this machine.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "arm_id",
            default_value="fer",
            description="ID of the type of arm used. Supported values: fer, fr3, fp3",
            choices=["fer", "fr3", "fp3"],
        ),
        DeclareLaunchArgument(
            "use_gazebo",
            default_value="false",
            description="Configures launch file for Gazebo simulation. "
            + "If set to `true` launch file is configured for simulated robot. "
            + "If set to `false` argument `robot_ip` is expected not to be empty.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Visualize the robot in RViz",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "gz_verbose",
            default_value="false",
            description="Whether to set verbosity level of Gazebo to 3.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Whether to launch Gazebo in headless mode "
            + "(no GUI is launched, only physics server).",
            choices=["true", "false"],
        ),
    ]


def generate_include_franka_launch(launch_file_name: str) -> IncludeLaunchDescription:
    """Generates IncludeLaunchDescription object of default launch files
        for Agimus Demos for Franka robots. Automatically obtains values of launch arguments required
        by the launch file. Assumes argument are declared with function `generate_default_franka_args()`.

    Args:
        launch_file_name (str): Name of the python launch file to included
            from directory `agimus_demos_common/launch`.

    Returns:
        IncludeLaunchDescription: Include launch description with all default parameters passed to it.
    """
    public_launch_files = ["franka_common.launch.py", "franka_common_lfc.launch.py"]
    if launch_file_name not in public_launch_files:
        raise RuntimeError(
            f"Incorrect launch file name! '{launch_file_name}' is not part of public API "
            + f"launch files of Agimus Demos! Allowed options are {public_launch_files}!"
        )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        launch_file_name,
                    ]
                )
            ]
        ),
        launch_arguments={
            "arm_id": LaunchConfiguration("arm_id"),
            "aux_computer_ip": LaunchConfiguration("aux_computer_ip"),
            "aux_computer_user": LaunchConfiguration("aux_computer_user"),
            "on_aux_computer": LaunchConfiguration("on_aux_computer"),
            "robot_ip": LaunchConfiguration("robot_ip"),
            "use_gazebo": LaunchConfiguration("use_gazebo"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "gz_verbose": LaunchConfiguration("gz_verbose"),
            "gz_headless": LaunchConfiguration("gz_headless"),
        }.items(),
    )


def get_use_sim_time() -> dict[str, LaunchConfiguration]:
    """Helper function creating action setting param `use_sim_time`.

    Returns:
        dict[str, LaunchConfiguration]: Dict mapping value of
        `use_gazebo` launch argument to `use_sim_time` param.
    """
    return {"use_sim_time": LaunchConfiguration("use_gazebo")}
