from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_franka_launch,
    get_use_sim_time,
)

from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    franka_robot_launch = generate_include_franka_launch("franka_common_lfc.launch.py")

    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_05_pick_and_place"),
            "config",
            "agimus_controller_params.yaml",
        ]
    )
    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )
    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[get_use_sim_time(), agimus_controller_yaml],
        output="screen",
        remappings=[("robot_description", "robot_description_with_collision")],
    )

    environment_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_05_pick_and_place"),
                        "urdf",
                        "environment.urdf.xacro",
                    ]
                ),
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
        frame_id="robot_attachment_link",
        child_frame_id="world",
    )

    pick_and_place_node = ExecuteProcess(
        cmd=[
            "xterm",
            "-hold",
            "-e",
            'bash -c "source /opt/ros/humble/setup.bash && '
            'ros2 run agimus_demo_05_pick_and_place pick_and_place_node"',
        ],
        output="screen",
    )

    return [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        environment_publisher_node,
        tf_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    pick_and_place_node,
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
