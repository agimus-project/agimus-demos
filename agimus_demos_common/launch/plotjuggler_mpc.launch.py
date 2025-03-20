from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        arguments=[
            "--layout",
            PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "plotjuggler_mpc_debug.xml",
                ]
            ),
        ],
        output="screen",
    )

    return [plotjuggler_node]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
