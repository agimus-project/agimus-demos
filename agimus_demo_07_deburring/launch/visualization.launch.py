from agimus_demos_common.launch_utils import (
    get_use_sim_time,
    ament_prefix_to_ros_package,
)
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, ExecuteProcess
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    use_rviz = LaunchConfiguration("use_rviz")
    use_gepetto_gui = LaunchConfiguration("use_gepetto_gui")

    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_07_deburring"),
            "rviz",
            "config.rviz",
        ]
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        parameters=[get_use_sim_time()],
        arguments=["--display-config", rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    hpp_corba_server = ExecuteProcess(
        cmd=["hppcorbaserver"],
        # Extra env variable for HPP to discover packages correctly
        additional_env=ament_prefix_to_ros_package(context),
        output="screen",
    )

    gepetto_gui = ExecuteProcess(
        cmd=["gepetto-gui"],
        # Extra env variable for HPP to discover packages correctly
        additional_env=ament_prefix_to_ros_package(context),
        condition=IfCondition(use_gepetto_gui),
        output="screen",
    )

    return [hpp_corba_server, rviz_node, gepetto_gui]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Whether to launch RViz.",
        ),
        DeclareLaunchArgument(
            "use_gepetto_gui",
            default_value="true",
            description="Whether to launch RViz.",
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
