from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_launch,
)
from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_07_deburring"),
            "rviz",
            "config.rviz",
        ]
    )

    pytroller_params = (
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demo_07_deburring"),
                "config",
                "pytroller_params.yaml",
            ]
        ),
    )

    agimus_pytroller_names = ["agimus_pytroller", "ft_calibration_filter"]

    franka_robot_launch = generate_include_launch(
        "franka_common.launch.py",
        extra_launch_arguments={
            "external_controllers_names": str(agimus_pytroller_names),
            "external_controllers_params": pytroller_params,
            "rviz_config_path": rviz_config_path,
            "use_ft_sensor": "true",
            "ee_id": "ati_mini45_with_camera",
        },
    )

    environment_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_07_deburring"),
                        "urdf",
                        "environment.urdf.xacro",
                    ]
                ),
                " ",
                "with_sc:=true",
            ]
        ),
        value_type=str,
    )
    environment_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="environment_publisher",
        output="screen",
        remappings=[("robot_description", "environment_description")],
        parameters=[{"robot_description": environment_description}],
    )
    tf_node = static_transform_publisher_node(
        frame_id="fer_link0",
        child_frame_id="env_ref_frame",
    )

    return [franka_robot_launch, environment_publisher_node, tf_node]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
