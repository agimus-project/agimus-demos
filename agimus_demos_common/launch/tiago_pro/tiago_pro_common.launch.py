from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,  # noqa: I001
)
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
    get_use_sim_time,
)

PASSTHROUGH_CONTROLLERS = [
    "arm_right_1_joint_inertia_shaping_controller",
    "arm_right_2_joint_inertia_shaping_controller",
    "arm_right_3_joint_inertia_shaping_controller",
    "arm_right_4_joint_inertia_shaping_controller",
    "arm_right_5_joint_inertia_shaping_controller",
    "arm_right_6_joint_inertia_shaping_controller",
    "arm_right_7_joint_inertia_shaping_controller",
]


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    # TODO make the demos launch more fluid by using this parameter.
    # robot_ip = LaunchConfiguration("robot_ip")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_path = LaunchConfiguration("rviz_config_path")

    use_gazebo_bool = context.perform_substitution(use_gazebo).lower() == "true"

    lfc_controllers = [
        "linear_feedback_controller",
        "joint_state_estimator",
    ]

    if use_gazebo_bool:
        lfc_pkg_share = get_package_share_directory(
            context.perform_substitution(LaunchConfiguration("lfc_pkg"))
        )
        lfc_yaml = context.perform_substitution(LaunchConfiguration("lfc_yaml"))
        jse_yaml = context.perform_substitution(LaunchConfiguration("jse_yaml"))
        pc_yaml = context.perform_substitution(LaunchConfiguration("pc_yaml"))
        lfc_controllers_params = [
            os.path.join(lfc_pkg_share, pc_yaml),
            os.path.join(lfc_pkg_share, jse_yaml),
            os.path.join(lfc_pkg_share, lfc_yaml),
        ]
        lfc_controllers = PASSTHROUGH_CONTROLLERS + lfc_controllers
    else:
        lfc_controllers_params = [
            "/tmp/joint_state_estimator_params.yaml",
            "/tmp/linear_feedback_controller_params.yaml",
        ]

    spawn_lfc_controllers = generate_controllers_spawner_launch_description(
        controller_names=lfc_controllers,
        controller_params_files=lfc_controllers_params,
        extra_spawner_args=[
            "--inactive",
            "--controller-manager-timeout",
            "10000000",
        ],
    )

    robot_srdf_description_substitution = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demos_common"),
            "config",
            "tiago_pro",
            "tiago_pro_dummy.srdf.xacro",
        ]
    )
    robot_srdf_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                robot_srdf_description_substitution,
            ]
        ),
        value_type=str,
    )
    robot_srdf_publisher_node = Node(
        package="agimus_demos_common",
        executable="string_publisher",
        name="robot_srdf_description_publisher",
        output="screen",
        parameters=[
            {
                "topic_name": "robot_srdf_description",
                "string_value": robot_srdf_description,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[get_use_sim_time()],
        arguments=["--display-config", rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    if use_gazebo_bool:
        gazebo_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [
                    PathJoinSubstitution(
                        [
                            FindPackageShare("tiago_pro_gazebo"),
                            "launch",
                            "tiago_pro_gazebo.launch.py",
                        ]
                    )
                ]
            ),
            launch_arguments={
                **{
                    arg.name: LaunchConfiguration(arg.name)
                    for arg in generate_default_tiago_pro_args()
                },
                "tuck_arm": "False",
                "is_public_sim": "True",
                "moveit": "False",
                "play_motion2": "False",
            }.items(),
        )

        # Activate all at once: ros2_control sets chained mode atomically
        # when preceding (LFC) and following (PassthroughControllers) are
        # activated in the same switch_controllers transaction.
        activate_controllers = ExecuteProcess(
            cmd=[
                "ros2",
                "control",
                "switch_controllers",
                "--deactivate",
                "arm_right_controller",
                "--activate",
            ]
            + PASSTHROUGH_CONTROLLERS
            + ["linear_feedback_controller", "joint_state_estimator"],
            output="screen",
        )

        return [
            gazebo_launch,
            spawn_lfc_controllers,
            robot_srdf_publisher_node,
            rviz_node,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=spawn_lfc_controllers.entities[2],
                    on_exit=[activate_controllers],
                )
            ),
        ]
    else:
        activate_controllers = Node(
            package="agimus_demos_common",
            executable="switch_controllers_trigger_node",
            name="switch_controllers_trigger_node",
            output="screen",
            prefix="xterm -e",
        )

        return [
            spawn_lfc_controllers,
            activate_controllers,
            robot_srdf_publisher_node,
            rviz_node,
        ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_tiago_pro_args() + [OpaqueFunction(function=launch_setup)]
    )
