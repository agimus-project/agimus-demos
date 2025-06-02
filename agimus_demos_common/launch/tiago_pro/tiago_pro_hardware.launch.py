from launch import LaunchContext, LaunchDescription
from launch.actions import (
    OpaqueFunction,
)
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,  # noqa: I001
)
from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    lfc_controllers_params = [
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demos_common"),
                "config",
                "tiago_pro",
                "joint_state_estimator_params.yaml",
            ]
        ),
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demos_common"),
                "config",
                "tiago_pro",
                "linear_feedback_controller_params.yaml",
            ]
        ),
    ]
    lfc_controllers = [
        "joint_state_estimator",
        "linear_feedback_controller",
    ]
    # Spawn external controllers, namely the lfc.
    spawn_lfc_controllers = generate_controllers_spawner_launch_description(
        controller_names=lfc_controllers,
        controller_params_files=lfc_controllers_params,
        extra_spawner_args=[
            "--inactive",
            "--controller-manager-timeout",
            "10000000",
        ],
    )

    activate_controllers = Node(
        package="agimus_demos_common",
        executable="switch_controllers_trigger_node",
        name="switch_controllers_trigger_node",
        output="screen",
        prefix="xterm -e",
    )
    return [spawn_lfc_controllers, activate_controllers]


def generate_launch_description():
    return LaunchDescription(
        generate_default_tiago_pro_args() + [OpaqueFunction(function=launch_setup)]
    )
