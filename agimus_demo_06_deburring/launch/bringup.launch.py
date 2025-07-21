from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_launch,
)
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_06_deburring"),
            "rviz",
            "config.rviz",
        ]
    )

    pytroller_params = (
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demo_06_deburring"),
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
        },
    )

    # environment_description = ParameterValue(
    #     Command(
    #         [
    #             PathJoinSubstitution([FindExecutable(name="xacro")]),
    #             " ",
    #             PathJoinSubstitution(
    #                 [
    #                     FindPackageShare("agimus_demo_05_pick_and_place"),
    #                     "urdf",
    #                     "environment.urdf.xacro",
    #                 ]
    #             ),
    #         ]
    #     ),
    #     value_type=str,
    # )
    # environment_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     name="environment_publisher",
    #     output="screen",
    #     remappings=[("robot_description", "environment_description")],
    #     parameters=[{"robot_description": environment_description}],
    # )

    # pick_and_place_node = ExecuteProcess(
    #     cmd=[
    #         "xterm",
    #         "-hold",
    #         "-e",
    #         'bash -c "source /opt/ros/humble/setup.bash && '
    #         f'ros2 run agimus_demo_05_pick_and_place pick_and_place_node --ros-args --params-file {trajectory_weights_yaml}"',
    #     ],
    #     output="screen",
    # )
    # trajectory_publisher_node

    return [franka_robot_launch]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
