from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
)
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
    generate_include_launch,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    linear_feedback_controllers_names = [
        "linear_feedback_controller",
        "joint_state_estimator",
    ]
    tiago_pro_robot_launch = generate_include_launch(
        "tiago_pro_common.launch.py",
        extra_launch_arguments={
            "external_controllers_names": str(linear_feedback_controllers_names),
            "external_controllers_params": LaunchConfiguration(
                "linear_feedback_controller_params"
            ),
        },
    )
    return [tiago_pro_robot_launch]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "tiago_pro_controllers_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "tiago_pro",
                    "tiago_pro_controllers.yaml",
                ]
            ),
            description="Path to the yaml file use to define controller parameters.",
        ),
        DeclareLaunchArgument(
            "linear_feedback_controller_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "tiago_pro",
                    "linear_feedback_controller_params.yaml",
                ]
            ),
            description="Path to the yaml file use to define "
            + "Linear Feedback Controller's and Joint State Estimator's params.",
        ),
        DeclareLaunchArgument(
            "linear_feedback_controller_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "linear_feedback_controller_params.yaml",
                ]
            ),
            description="Path to the yaml file use to define "
            + "Linear Feedback Controller's and Joint State Estimator's params.",
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
    ]

    return LaunchDescription(
        declared_arguments
        + generate_default_tiago_pro_args()
        + [OpaqueFunction(function=launch_setup)]
    )
