from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from controller_manager.launch_utils import (
    generate_load_controller_launch_description,  # noqa: I001
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    arm_id = LaunchConfiguration("arm_id")
    robot_ip = LaunchConfiguration("robot_ip")
    disable_collision_safety = LaunchConfiguration("disable_collision_safety")
    franka_controllers_params = LaunchConfiguration("franka_controllers_params")

    disable_collision_safety_bool = (
        context.perform_substitution(disable_collision_safety).lower() == "true"
    )

    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            franka_controllers_params,
            {"arm_id": arm_id, "load_gripper": "true"},
        ],
        remappings=[
            ("joint_states", "franka/joint_states"),
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )

    spawn_default_controller = generate_load_controller_launch_description(
        "joint_state_broadcaster",
        "force_torque_sensor_broadcaster",
        "net_ft_diagnostic_broadcaster",
    )

    franka_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("franka_gripper"),
                        "launch",
                        "gripper.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": "false",
        }.items(),
    )

    franka_safety_params_file = (
        "franka_collisions_unsafe.yaml"
        if disable_collision_safety_bool
        else "franka_collisions_default.yaml"
    )

    disable_franka_collisions_node = Node(
        package="agimus_demos_common",
        executable="disable_franka_collisions",
        name="disable_franka_collisions",
        output="screen",
        # If set to `true`, change values to custom ones. If `false` use default.
        # Robot remembers previous parameters, so we need to change values every time.
        parameters=(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "config",
                        franka_safety_params_file,
                    ]
                )
            ]
        ),
    )

    return [
        controller_manager_node,
        franka_gripper_launch,
        spawn_default_controller,
        disable_franka_collisions_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            description="Hostname or IP address of the robot.",
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
            "franka_controllers_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "franka",
                    "controllers.yaml",
                ]
            ),
            description="Path to the yaml file used to define controller parameters.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
