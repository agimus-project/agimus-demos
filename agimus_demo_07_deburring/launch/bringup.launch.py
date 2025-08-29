from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    get_use_sim_time,
    ament_prefix_to_ros_package,
)
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node
from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution, Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    # pytroller_params = (
    #     PathJoinSubstitution(
    #         [
    #             FindPackageShare("agimus_demo_07_deburring"),
    #             "config",
    #             "pytroller_params.yaml",
    #         ]
    #     ),
    # )

    # agimus_pytroller_names = ["agimus_pytroller", "ft_calibration_filter"]

    # franka_robot_launch = generate_include_launch(
    #     "franka_common.launch.py",
    #     extra_launch_arguments={
    #         "external_controllers_names": str(agimus_pytroller_names),
    #         "external_controllers_params": pytroller_params,
    #         "use_ft_sensor": "true",
    #     },
    # )

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
        parameters=[get_use_sim_time(), {"robot_description": environment_description}],
    )

    hpp_corba_server = ExecuteProcess(
        cmd=["hppcorbaserver"],
        # Extra env variable for HPP to discover packages correctly
        additional_env=ament_prefix_to_ros_package(context),
        output="screen",
    )

    orchestrator_params = (
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demo_07_deburring"),
                "config",
                "orchestrator_params.yaml",
            ]
        ),
    )

    deburring_orchestrator_node = Node(
        prefix="xterm -hold -e",
        package="agimus_demo_07_deburring",
        executable="deburring_orchestrator_node",
        parameters=[get_use_sim_time(), orchestrator_params],
        # Extra env variable for HPP to discover packages correctly
        additional_env=ament_prefix_to_ros_package(context),
        output="screen",
    )

    # panda_to_pylone_tf = static_transform_publisher_node(
    #     frame_id="fer_link0",
    #     child_frame_id="pylone_link",
    #     xyz=(0.5, 0.0, 0.0),
    #     rot_rpy=(0.0, 0.0, 0.0),
    # )

    return [
        # franka_robot_launch,
        hpp_corba_server,
        deburring_orchestrator_node,
        # panda_to_pylone_tf,
        environment_publisher_node,
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
