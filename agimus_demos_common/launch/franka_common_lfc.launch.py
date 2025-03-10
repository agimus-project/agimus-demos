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
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,  # noqa: I001
)

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    get_use_sim_time,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    franka_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        "franka_common.launch.py",
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
            "franka_controllers_params": LaunchConfiguration(
                "franka_controllers_params"
            ),
            "use_gazebo": LaunchConfiguration("use_gazebo"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "rviz_config_path": LaunchConfiguration("rviz_config_path"),
            "gz_verbose": LaunchConfiguration("gz_verbose"),
            "gz_headless": LaunchConfiguration("gz_headless"),
        }.items(),
    )

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        name="lfc_wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    spawn_linear_feedback_controller = generate_controllers_spawner_launch_description(
        ["linear_feedback_controller", "joint_state_estimator"]
    )

    return [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[spawn_linear_feedback_controller],
            ),
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "franka_controllers_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "franka_controllers.yaml",
                ]
            ),
            description="Path to the yaml file use to define controller parameters.",
        ),
        DeclareLaunchArgument(
            "rviz_config_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "rviz",
                    "franka_preview.rviz",
                ]
            ),
            description="Path to RViz configuration file",
        ),
    ]

    return LaunchDescription(
        declared_arguments
        + generate_default_franka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
