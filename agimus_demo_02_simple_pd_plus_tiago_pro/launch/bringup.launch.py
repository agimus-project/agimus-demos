from launch import LaunchContext, LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    generate_default_tiago_args,
    generate_include_tiago_launch,
    get_use_sim_time,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    tiago_robot_launch = generate_include_tiago_launch("tiago_common_lfc.launch.py")

    pd_plus_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_02_simple_pd_plus_tiago_pro"),
            "config",
            "pd_plus_controller_params.yaml",
        ]
    )

    pd_plus_controller_node = Node(
        package="linear_feedback_controller",
        executable="pd_plus_controller",
        parameters=[get_use_sim_time(), pd_plus_controller_params],
        output="screen",
    )

    def on_tuck_arm_exit_callback(event, context):
        if "tuck_arm.py" in event.process_name:
            if event.returncode == 0:
                return [
                    LogInfo(msg="Starting the pd+ controller."),
                    pd_plus_controller_node,
                ]
            else:
                return [
                    LogInfo(
                        msg="Problem during the initialization, "
                        "PD+ controller not started."
                    )
                ]

    return [
        tiago_robot_launch,
        RegisterEventHandler(OnProcessExit(on_exit=on_tuck_arm_exit_callback)),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_tiago_args() + [OpaqueFunction(function=launch_setup)]
    )
