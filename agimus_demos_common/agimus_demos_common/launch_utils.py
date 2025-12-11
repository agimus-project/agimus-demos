from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import EnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os
import re
import yaml
import tempfile

# Constants used to synchronize paths expected on remote between launch files
COMPOSE_REMOTE_PATH: Path = Path("/tmp/compose.yaml")
EXTERNAL_CONTROLLERS_PARAMS_REMOTE_PATH: Path = Path(
    "/tmp/external_controllers_params.yaml"
)
FRANKA_PARAMS_REMOTE_PATH: Path = Path("/tmp/franka_controllers.yaml")


def safe_remove(path):
    """Remove the file if it exists; do nothing if it is already gone."""
    if path and os.path.exists(path):
        try:
            os.remove(path)
        except OSError:
            pass


def parse_config(path=None, data=None, replacements=None, output_path=None):
    """
    Load a YAML file, replace all occurrences of ${VAR}
    in keys and values, and save the result if requested.
    args:
        path (str): Path to the YAML file to load.
        data (str): YAML content as a string.
        replacements (dict): Dictionary of variable replacements.
        output_path (str): If provided, save the modified YAML to this path.
    returns:
        str: Path to the modified YAML temporary file or output_path.
    """

    pattern = re.compile(r"\${(\w+)}")

    def replace_in_string(value):
        """Replace all ${...} variables in a string."""
        matches = pattern.findall(value)
        for m in matches:
            r = None
            if replacements:
                r = replacements.get(m)
            if r is None:
                r = os.environ.get(m)
            if r is None:
                raise ValueError(f"Variable '{m}' not found.")
            value = value.replace(f"${{{m}}}", r)
        return value

    def replace_recursive(obj):
        """Apply replacement throughout the YAML."""
        if isinstance(obj, dict):
            new_dict = {}
            for key, val in obj.items():
                # replacement in keys
                new_key = replace_in_string(key) if isinstance(key, str) else key
                new_dict[new_key] = replace_recursive(val)
            return new_dict

        elif isinstance(obj, list):
            return [replace_recursive(item) for item in obj]

        elif isinstance(obj, str):
            return replace_in_string(obj)

        return obj

    if path:
        with open(path) as f:
            config = yaml.safe_load(f)
    elif data:
        config = yaml.safe_load(data)
    else:
        raise ValueError("Provide path or data")

    # --- Apply replacements ---
    config = replace_recursive(config)

    # --- Saving ---
    if output_path:
        with open(output_path, "w") as f:
            yaml.dump(config, f, allow_unicode=True, default_flow_style=False)
        return output_path
    else:
        with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as tmp:
            yaml.dump(config, tmp)
            tmp_path = tmp.name

    return tmp_path


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
            description="ID of the type of arm used. Supported values: fer, fr3, fp3.",
            choices=["fer", "fr3", "fp3"],
        ),
        DeclareLaunchArgument(
            "disable_collision_safety",
            default_value="false",
            description="Whether to disable safety limits for franka robot.",
            choices=["true", "false"],
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
            "use_plotjuggler",
            default_value="false",
            description="Visualize time series with PLotJuggler",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "use_ft_sensor",
            default_value="false",
            description="Enable or disable use of force-torque sensor",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "ft_sensor_ip",
            default_value="",
            description="Hostname or IP address of the force-torque sensor.",
        ),
        DeclareLaunchArgument(
            "ee_id",
            default_value="franka_hand",
            description="Name of the end effector used.",
        ),
        DeclareLaunchArgument(
            "use_camera",
            default_value="true",
            description="Enable or disable RealSense d435i in the URDF.",
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


def generate_default_tiago_pro_args() -> list[DeclareLaunchArgument]:
    """Generates list of default arguments expected to be public interface of
        launch files used by Agimus Demos for Tiago-Pro robot.

    Returns:
        list[DeclareLaunchArgument]: List of DeclareLaunchArgument objects with
            arguments expected by `tiago_common.launch.py` and `tiago_common_lfc.launch.py`
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
            "on_robot",
            default_value="false",
            description="Are we running on the robot?",
            choices=["true", "false"],
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
            "rviz_config_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "rviz",
                    "tiago_pro",
                    "preview.rviz",
                ]
            ),
            description="Path to RViz configuration file",
        ),
        DeclareLaunchArgument(
            "world_name",
            default_value="empty",
            description="Gazebo world name from the pal_gazebo_worlds package.",
            choices=["empty", "pal_office"],
        ),
        DeclareLaunchArgument(
            "base_type",
            description="Base type.",
            choices=["omni_base"],
            default_value="omni_base",
        ),
        DeclareLaunchArgument(
            "arm_type_right",
            description="Arm type right.",
            choices=["tiago-pro", "no-arm"],
            default_value="tiago-pro",
        ),
        DeclareLaunchArgument(
            "arm_type_left",
            description="Arm type left.",
            choices=["tiago-pro", "no-arm"],
            default_value="tiago-pro",
        ),
        DeclareLaunchArgument(
            "end_effector_right",
            description="End effector model right arm.",
            choices=["pal-pro-gripper", "custom", "no-end-effector"],
            default_value="no-end-effector",
        ),
        DeclareLaunchArgument(
            "end_effector_left",
            description="End effector model left arm.",
            choices=["pal-pro-gripper", "custom", "no-end-effector"],
            default_value="no-end-effector",
        ),
        DeclareLaunchArgument(
            "ft_sensor_right",
            description="FT sensor model right arm.",
            choices=["ati", "rokubi", "no-ft-sensor"],
            default_value="no-ft-sensor",
        ),
        DeclareLaunchArgument(
            "ft_sensor_left",
            description="FT sensor model left arm.",
            choices=["ati", "rokubi", "no-ft-sensor"],
            default_value="no-ft-sensor",
        ),
        DeclareLaunchArgument(
            "tool_changer_right",
            description="The arm has a tool changer.",
            choices=["False", "True"],
            default_value="True",
        ),
        DeclareLaunchArgument(
            "tool_changer_left",
            description="The arm has a tool changer.",
            choices=["False", "True"],
            default_value="True",
        ),
        DeclareLaunchArgument(
            "wrist_model_right",
            description="Wrist model right.",
            choices=["spherical-wrist", "straight-wrist"],
            default_value="spherical-wrist",
        ),
        DeclareLaunchArgument(
            "wrist_model_left",
            description="Wrist model left.",
            choices=["spherical-wrist", "straight-wrist"],
            default_value="spherical-wrist",
        ),
        DeclareLaunchArgument(
            "camera_model",
            description="Head camera model.",
            choices=["realsense-d435", "realsense-d455"],
            default_value="realsense-d435",
        ),
        DeclareLaunchArgument(
            "laser_model",
            description="Base laser model.",
            choices=["no-laser", "sick-571"],
            default_value="no-laser",
        ),
        DeclareLaunchArgument(
            "navigation",
            description="Specify if launching Navigation2.",
            choices=["True", "False"],
            default_value="False",
        ),
        DeclareLaunchArgument(
            "advanced_navigation",
            description="Specify if launching Advanced Navigation.",
            choices=["True", "False"],
            default_value="False",
        ),
        DeclareLaunchArgument(
            "slam",
            description="Whether or not you are using SLAM",
            default_value="False",
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "docking",
            description="Specify if launching Docking.",
            choices=["True", "False"],
            default_value="False",
        ),
        DeclareLaunchArgument(
            "moveit",
            description="Specify if launching MoveIt 2.",
            choices=["True", "False"],
            default_value="True",
        ),
        DeclareLaunchArgument(
            "world_name",
            description="Specify world name, will be converted to full path.",
            default_value="empty",
        ),
        DeclareLaunchArgument(
            "tuck_arm",
            description="Launches tuck arm node.",
            choices=["True", "False"],
            default_value="True",
        ),
        DeclareLaunchArgument(
            "is_public_sim",
            description="Enable public simulation.",
            choices=["True", "False"],
            default_value="False",
        ),
    ]


def generate_include_launch(
    launch_file_name: str, extra_launch_arguments={}
) -> IncludeLaunchDescription:
    """Generates IncludeLaunchDescription object of default launch files
        for Agimus Demos for robots. Automatically obtains values of launch arguments required
        by the launch file. Assumes argument are declared with function `generate_default_franka_args()`.

    Args:
        launch_file_name (str): Name of the python launch file to included
            from directory `agimus_demos_common/launch`.
        extra_launch_arguments (dict): List of extra arguments.

    Returns:
        IncludeLaunchDescription: Include launch description with all default parameters passed to it.
    """
    public_launch_files = [
        "franka_common.launch.py",
        "franka_common_lfc.launch.py",
        "tiago_pro_common.launch.py",
    ]
    if launch_file_name not in public_launch_files:
        raise RuntimeError(
            f"Incorrect launch file name! '{launch_file_name}' is not part of public API "
            + f"launch files of Agimus Demos! Allowed options are {public_launch_files}!"
        )
    if "tiago_pro" in launch_file_name:
        robot_name = "tiago_pro"
        generate_default_args = generate_default_tiago_pro_args
    elif "franka" in launch_file_name:
        robot_name = "franka"
        generate_default_args = generate_default_franka_args
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        robot_name,
                        launch_file_name,
                    ]
                )
            ]
        ),
        launch_arguments={
            **{
                arg.name: LaunchConfiguration(arg.name)
                for arg in generate_default_args()
                if arg.name not in extra_launch_arguments.keys()
            },
            **extra_launch_arguments,
        }.items(),
    )


def get_use_sim_time() -> dict[str, LaunchConfiguration]:
    """Helper function creating action setting param `use_sim_time`.

    Returns:
        dict[str, LaunchConfiguration]: Dict mapping value of
        `use_gazebo` launch argument to `use_sim_time` param.
    """
    return {"use_sim_time": LaunchConfiguration("use_gazebo")}


def ament_prefix_to_ros_package(context: LaunchContext) -> dict[str, str]:
    """Converts `AMENT_PREFIX_PATH` to match old `ROS_PACKAGE_PATH` as it is required by HPP

    Args:
        context (LaunchContext): launch context used to evaluate environment variables

    Returns:
        dict[str, str]: Dictionary with `ROS_PACKAGE_PATH`
    """
    ament_prefix = context.perform_substitution(
        EnvironmentVariable("AMENT_PREFIX_PATH")
    )
    return {"ROS_PACKAGE_PATH": ":".join(v + "/share" for v in ament_prefix.split(":"))}
