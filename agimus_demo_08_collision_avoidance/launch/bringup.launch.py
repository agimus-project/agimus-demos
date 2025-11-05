from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch.substitutions import Command, FindExecutable

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_launch,
    get_use_sim_time,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_08_collision_avoidance"),
            "rviz",
            "config.rviz",
        ]
    )

    franka_robot_launch = generate_include_launch(
        "franka_common_lfc.launch.py",
        extra_launch_arguments={"rviz_config_path": rviz_config_path},
    )

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    agimus_controller_params = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_08_collision_avoidance"),
            "config",
            "agimus_controller_params.yaml",
        ]
    )

    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[
            get_use_sim_time(),
            agimus_controller_params,
        ],
        output="screen",
        remappings=[("robot_description", "/robot_description_with_collision")],
    )

    environment_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_08_collision_avoidance"),
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
        parameters=[get_use_sim_time(), {"robot_description": environment_description}],
        remappings=[("robot_description", "environment_description")],
    )

    goal_publisher_params = (
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demo_08_collision_avoidance"),
                "config",
                "goal_publisher_params.yaml",
            ]
        ),
    )

    goal_publisher_node = Node(
        package="agimus_demo_08_collision_avoidance",
        executable="goal_publisher",
        name="goal_publisher_node",
        output="both",
        parameters=[
            get_use_sim_time(),
            ParameterFile(param_file=goal_publisher_params, allow_substs=True),
        ],
    )

    obstacle_pose_publisher_node = Node(
        package="agimus_demo_08_collision_avoidance",
        executable="obstacle_pose_publisher",
        name="obstacle_pose_publisher_node",
        output="both",
        parameters=[get_use_sim_time()],
    )

    return [
        franka_robot_launch,
        environment_publisher_node,
        wait_for_non_zero_joints_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    obstacle_pose_publisher_node,
                ],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessStart(
                target_action=agimus_controller_node,
                on_start=TimerAction(
                    period=5.0,
                    actions=[goal_publisher_node],
                ),
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
