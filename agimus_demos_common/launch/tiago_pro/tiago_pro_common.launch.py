from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,  # noqa: I001
)
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
    get_use_sim_time,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    robot_ip = LaunchConfiguration("robot_ip")
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_path = LaunchConfiguration("rviz_config_path")

    use_gazebo_bool = context.perform_substitution(use_gazebo).lower() == "true"
    if use_gazebo_bool:
        jse_file = "joint_state_estimator_simu_params.yaml"
        lfc_file = "linear_feedback_controller_simu_params.yaml"
    else:
        jse_file = "joint_state_estimator_params.yaml"
        lfc_file = "linear_feedback_controller_params.yaml"
    lfc_controllers_params = [
        f"/opt/agimus_demos/config/{jse_file}", # JSE params file
        f"/opt/agimus_demos/config/{lfc_file}", # LFC params file
    ]
    lfc_controllers = [
        "linear_feedback_controller",
        "joint_state_estimator",
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
