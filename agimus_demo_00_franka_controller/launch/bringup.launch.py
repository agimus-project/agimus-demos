from launch import LaunchContext, LaunchDescription
from launch.actions import (
    OpaqueFunction,
)
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import (
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_franka_launch,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    joint_impedance_example_controller_params = (
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demo_00_franka_controller"),
                "config",
                "joint_impedance_example_controller_params.yaml",
            ]
        ),
    )

    joint_impedance_example_controller_names = ["joint_impedance_example_controller"]

    franka_robot_launch = generate_include_franka_launch(
        "franka_common.launch.py",
        extra_launch_arguments={
            "external_controllers_params": joint_impedance_example_controller_params,
            "external_controllers_names": str(joint_impedance_example_controller_names),
        },
    )

    return [franka_robot_launch]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
