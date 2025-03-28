from launch import LaunchContext, LaunchDescription
from launch.actions import LogInfo, OpaqueFunction, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
    generate_include_launch,
    get_use_sim_time,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    tiago_robot_launch = generate_include_launch("tiago_pro_common_lfc.launch.py")

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

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    activate_lfc_controllers = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "switch_controllers",
            "--deactivate",
            "arm_left_controller",
            "--activate",
            "joint_state_estimator",
            "linear_feedback_controller",
        ],
        output="screen",
    )

    def on_tuck_arm_exit_callback(event, context: LaunchContext):
        if "tuck_arm.py" in event.process_name:
            if event.returncode == 0:
                return [
                    LogInfo(msg="Starting the pd+ controller."),
                    pd_plus_controller_node,
                    activate_lfc_controllers,
                ]
            else:
                return [
                    LogInfo(
                        msg="Problem during the initialization, "
                        "PD+ controller not started."
                    )
                ]

    use_gazebo = LaunchConfiguration("use_gazebo")
    use_gazebo_bool = context.perform_substitution(use_gazebo).lower() == "true"
    if not use_gazebo_bool:
        return [
            tiago_robot_launch,
            wait_for_non_zero_joints_node,
            RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=wait_for_non_zero_joints_node,
                    on_exit=[pd_plus_controller_node, activate_lfc_controllers],
                )
            ),
        ]

    return [
        tiago_robot_launch,
        RegisterEventHandler(OnProcessExit(on_exit=on_tuck_arm_exit_callback)),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_tiago_pro_args() + [OpaqueFunction(function=launch_setup)]
    )
