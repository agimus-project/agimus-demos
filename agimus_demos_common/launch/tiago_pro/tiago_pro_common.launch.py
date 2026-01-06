from launch import LaunchContext, LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    OpaqueFunction,
)
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

    tiago_pro_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        "tiago_pro",
                        "tiago_pro_hardware.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"robot_ip": robot_ip}.items(),
        condition=UnlessCondition(use_gazebo),
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
        tiago_pro_hardware_launch,
        robot_srdf_publisher_node,
        rviz_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_tiago_pro_args() + [OpaqueFunction(function=launch_setup)]
    )
