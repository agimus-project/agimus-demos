from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
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
    generate_load_controller_launch_description,  # noqa: I001
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

    linear_feedback_controller_params = LaunchConfiguration(
        "linear_feedback_controller_params"
    )

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        name="lfc_wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )
    load_linear_feedback_controller = generate_load_controller_launch_description(
        controller_name="linear_feedback_controller",
        controller_params_file=linear_feedback_controller_params,
        extra_spawner_args=["--inactive", "--controller-manager-timeout", "1000"],
    )
    load_joint_state_estimator = generate_load_controller_launch_description(
        controller_name="joint_state_estimator",
        controller_params_file=linear_feedback_controller_params,
        extra_spawner_args=["--inactive", "--controller-manager-timeout", "1000"],
    )
    activate_lfc_controllers = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/controller_manager/switch_controller",
            "controller_manager_msgs/srv/SwitchController",
            "{activate_controllers: ['joint_state_estimator', 'linear_feedback_controller'], stop_controllers: [], strictness: 2}",
        ],
        output="screen",
    )

    return [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[load_linear_feedback_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_linear_feedback_controller.entities[-1],
                on_exit=[load_joint_state_estimator],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_estimator.entities[-1],
                on_exit=[activate_lfc_controllers],
            )
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
