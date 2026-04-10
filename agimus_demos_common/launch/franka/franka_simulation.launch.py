from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,  # noqa: I001
)

from agimus_demos_common.launch_utils import get_use_sim_time


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    gz_verbose = LaunchConfiguration("gz_verbose")
    gz_headless = LaunchConfiguration("gz_headless")
    gz_world_path = LaunchConfiguration("gz_world_path")
    use_ft_sensor = LaunchConfiguration("use_ft_sensor")
    ee_id = LaunchConfiguration("ee_id")

    gz_verbose_bool = context.perform_substitution(gz_verbose).lower() == "true"
    gz_headless_bool = context.perform_substitution(gz_headless).lower() == "true"
    gz_world_path_str = context.perform_substitution(gz_world_path)
    use_ft_sensor_bool = context.perform_substitution(use_ft_sensor).lower() == "true"
    ee_id_str = context.perform_substitution(ee_id).lower()

    gz_gui_config_path_str = context.perform_substitution(
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demos_common"),
                "config",
                "gz_gui.config",
            ]
        )
    )

    gazebo_empty_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ]
            )
        ),
        launch_arguments={
            "gz_args": gz_world_path_str
            + " -r"
            + f" {'-s' if gz_headless_bool else ''}"
            + f" {'-v 3' if gz_verbose_bool else ''}"
            + f" --gui-config {gz_gui_config_path_str}"
        }.items(),
    )

    ros_gz_bridge_node = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {
                "expand_gz_topic_names": True,
                "use_sim_time": True,
                "config_file": PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "config",
                        "gz_bridge.yaml",
                    ]
                ),
            }
        ],
        output="screen",
    )

    robot_spawner_node = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "/robot_description"],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    relay_node = Node(
        package="topic_tools",
        executable="relay",
        arguments=["/joint_states", "/agimus_franka/joint_states"],
        parameters=[get_use_sim_time()],
        output="screen",
        # silent warn "Some, but not all, publishers on topic /joint_states offer 'transient local' durability.
        # Falling back to 'volatile' durability in orderto connect to all publishers"
        ros_arguments=["--log-level", "relay:=ERROR"],
    )

    controllers = ["joint_state_broadcaster"]
    if ee_id_str == "agimus_franka_hand":
        controllers.append("gripper_action_controller")

    if use_ft_sensor_bool:
        controllers.append("force_torque_sensor_broadcaster")

    spawn_default_controllers = generate_controllers_spawner_launch_description(
        controllers
    )

    return [
        gazebo_empty_world,
        ros_gz_bridge_node,
        robot_spawner_node,
        relay_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=robot_spawner_node,
                on_exit=[spawn_default_controllers],
            )
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "gz_verbose",
            default_value="false",
            description="Whether to set verbosity level of Gazebo to 3.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "gz_headless",
            default_value="false",
            description="Whether to launch Gazebo in headless mode (no GUI is launched, only physics server).",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "gz_world_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_franka_description"),
                    "worlds",
                    "empty.sdf",
                ]
            ),
            description="Path to Gazebo world SDF file.",
        ),
        DeclareLaunchArgument(
            "use_ft_sensor",
            default_value="false",
            description="Enable or disable use of force-torque sensor",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "ee_id",
            default_value="agimus_franka_hand",
            description="Name of the end effector used.",
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
