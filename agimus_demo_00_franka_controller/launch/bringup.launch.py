from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from controller_manager.launch_utils import (
    generate_load_controller_launch_description,  # noqa: I001
)

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_franka_launch,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    franka_robot_launch = generate_include_franka_launch("franka_common.launch.py")

    joint_impedance_example_controller_params = (
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demo_00_franka_controller"),
                "config",
                "joint_impedance_example_controller_params.yaml",
            ]
        ),
    )

    load_joint_impedance_example_controller = (
        generate_load_controller_launch_description(
            controller_name="joint_impedance_example_controller",
            controller_params_file=joint_impedance_example_controller_params,
            extra_spawner_args=["--controller-manager-timeout", "1000"],
        )
    )

    return [
        franka_robot_launch,
        load_joint_impedance_example_controller,
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
