from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_launch,
    get_use_sim_time,
)
from launch.conditions import UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch_ros.substitutions import FindPackageShare

from launch import LaunchContext, LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)


def create_env_publisher(with_sc: bool) -> Node:
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
                f"with_sc:={str(with_sc).lower()}",
            ]
        ),
        value_type=str,
    )
    postfix = "_with_collision" if with_sc else "_without_collision"
    return Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="environment_publisher" + postfix,
        output="screen",
        remappings=[("robot_description", "environment_description" + postfix)],
        parameters=[{"robot_description": environment_description}],
    )


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    object_material = LaunchConfiguration("object_material")
    use_vision = LaunchConfiguration("use_vision")
    arm_id = LaunchConfiguration("arm_id")
    arm_id_str = arm_id.perform(context)

    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_07_deburring"),
            "rviz",
            "config.rviz",
        ]
    )

    plotjuggler_config_path = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_07_deburring"),
            "plotjuggler",
            "plotjuggler_view.xml",
        ]
    )

    pytroller_params = (
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demo_07_deburring"),
                "config",
                object_material,
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
            "plotjuggler_config_path": plotjuggler_config_path,
            "use_ft_sensor": "true",
            "use_camera": "true",
            "ee_id": "ati_mini45_with_compact_camera",
            "initial_joint_position": "'0.78 -0.78 0.0 -2.35 0.0 1.57 0.78 0.0'",
        },
    )

    environment_publisher_node_with_sc = create_env_publisher(True)
    environment_publisher_node_without_sc = create_env_publisher(False)

    deburring_path_planner_params = (
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demo_07_deburring"),
                "config",
                object_material,
                "deburring_path_planner_params.yaml",
            ]
        ),
    )

    deburring_path_planner = Node(
        package="agimus_demo_07_deburring",
        executable="deburring_path_planner",
        name="deburring_path_planner_node",
        output="both",
        parameters=[
            get_use_sim_time(),
            ParameterFile(param_file=deburring_path_planner_params, allow_substs=True),
        ],
    )

    dummy_pylone_detection_publisher = Node(
        package="agimus_demo_07_deburring",
        executable="dummy_pylone_detection_publisher",
        name="dummy_pylone_detection_publisher",
        output="both",
        parameters=[
            get_use_sim_time(),
            {
                "parent_frame_id": f"{arm_id_str}_link0",
                "child_frame_id": "pylone_link",
                "xyz": [0.45, -0.116, 0.739],
                "xyzw": [0.0, 0.0, 0.0, 1.0],
                # Execute a dummy linear sine along Y xis with rotation around Z
                # axis to see if the pose is being updated correctly in the system
                "test_sine_motion": True,
            },
        ],
        condition=UnlessCondition(use_vision),
    )

    return [
        franka_robot_launch,
        environment_publisher_node_with_sc,
        environment_publisher_node_without_sc,
        deburring_path_planner,
        dummy_pylone_detection_publisher,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_precomputed_trajectories",
            default_value="false",
            description="Whether to use paths that were previously precomputed.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "object_material",
            default_value="plastic",
            description="Which deburred material setup to use.",
            choices=["plastic", "metal"],
        ),
        DeclareLaunchArgument(
            "use_vision",
            default_value="false",
            description="Whether to expect Pylone detector to be running "
            "or to mock it.",
            choices=["true", "false"],
        ),
    ]

    return LaunchDescription(
        declared_arguments
        + generate_default_franka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
