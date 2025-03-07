from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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
    OrSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    get_use_sim_time,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    arm_id = LaunchConfiguration("arm_id")
    robot_ip = LaunchConfiguration("robot_ip")
    aux_computer_ip = LaunchConfiguration("aux_computer_ip")
    aux_computer_user = LaunchConfiguration("aux_computer_user")
    on_aux_computer = LaunchConfiguration("on_aux_computer")
    use_gazebo = LaunchConfiguration("use_gazebo")
    franka_controllers_params = LaunchConfiguration("franka_controllers_params")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_path = LaunchConfiguration("rviz_config_path")
    gz_verbose = LaunchConfiguration("gz_verbose")
    gz_headless = LaunchConfiguration("gz_headless")

    robot_ip_empty = context.perform_substitution(robot_ip) == ""
    aux_computer_ip_empty = context.perform_substitution(aux_computer_ip) == ""
    aux_computer_user_empty = context.perform_substitution(aux_computer_user) == ""
    use_gazebo_bool = context.perform_substitution(use_gazebo).lower() == "true"
    use_rviz_bool = context.perform_substitution(use_rviz).lower() == "true"
    on_aux_computer_bool = (
        context.perform_substitution(on_aux_computer).lower() == "true"
    )
    if robot_ip_empty and not use_gazebo_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Set `robot_ip` to configure hardware or "
            "`use_gazebo:=true` to configure simulation."
        )

    if not robot_ip_empty and use_gazebo_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`use_gazebo:=true` and non-empty `robot_ip`."
        )

    if robot_ip_empty and not aux_computer_ip_empty:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "non empty `aux_computer_ip` and empty `robot_ip`."
        )

    if not aux_computer_ip_empty and aux_computer_user_empty:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "non empty `aux_computer_ip` and empty `aux_computer_user`."
        )

    if robot_ip_empty and on_aux_computer_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer_bool:=true` and non-empty `robot_ip`."
        )

    if not aux_computer_ip_empty and on_aux_computer_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer_bool:=true` and non-empty `aux_computer_ip_empty`."
        )

    if on_aux_computer_bool and use_gazebo_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer_bool:=true` and `use_gazebo:=true`."
        )

    if on_aux_computer_bool and use_rviz_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer_bool:=true` and `use_rviz:=true`."
        )

    franka_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        "franka_hardware.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "arm_id": arm_id,
            "franka_controllers_params": franka_controllers_params,
        }.items(),
        condition=UnlessCondition(
            OrSubstitution(
                use_gazebo, PythonExpression(["'", aux_computer_ip, "' != ''"])
            )
        ),
    )
    # Auxiliary computer's docker does not have all the dependencies like franka_description
    # It is better to return with only minimal number of code evaluated to avoid errors
    # evaluating paths to packages that do not exist in the system.
    if on_aux_computer_bool:
        return [franka_hardware_launch]

    franka_remote_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        "franka_remote_hardware.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "arm_id": arm_id,
            "aux_computer_ip": aux_computer_ip,
            "aux_computer_user": aux_computer_user,
            "on_aux_computer": on_aux_computer,
            "franka_controllers_params": franka_controllers_params,
        }.items(),
        condition=UnlessCondition(use_gazebo),
    )

    franka_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        "franka_simulation.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "gz_verbose": gz_verbose,
            "gz_headless": gz_headless,
        }.items(),
        condition=IfCondition(use_gazebo),
    )

    arm_id_str = context.perform_substitution(arm_id)
    xacro_args = {
        "robot_ip": robot_ip,
        "arm_id": arm_id,
        "ros2_control": "true",
        "hand": "true",
        "use_fake_hardware": "false",
        "fake_sensor_commands": "false",
        "gazebo": use_gazebo,
        "ee_id": "franka_hand",
        "gazebo_effort": "true",
        "with_sc": "false",
        "franka_controllers_params": franka_controllers_params,
    }
    robot_description_file_substitution = PathJoinSubstitution(
        [
            FindPackageShare("franka_description"),
            "robots",
            arm_id_str,
            f"{arm_id_str}.urdf.xacro",
        ]
    )
    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                robot_description_file_substitution,
                # Convert dict to list of parameters
                *[arg for key, val in xacro_args.items() for arg in (f" {key}:=", val)],
            ]
        ),
        value_type=str,
    )

    xacro_collision_args = xacro_args.copy()
    xacro_collision_args["gazebo"] = "false"
    xacro_collision_args["with_sc"] = "true"

    robot_description_with_collision = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                robot_description_file_substitution,
                # Convert dict to list of parameters
                *[
                    arg
                    for key, val in xacro_collision_args.items()
                    for arg in (f" {key}:=", val)
                ],
            ]
        ),
        value_type=str,
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[get_use_sim_time(), {"robot_description": robot_description}],
        output="screen",
    )
    robot_collision_publisher_node = Node(
        package="agimus_demos_common",
        executable="string_publisher",
        name="robot_description_with_collision_publisher",
        output="screen",
        parameters=[
            get_use_sim_time(),
            {
                "topic_name": "robot_description_with_collision",
                "string_value": robot_description_with_collision,
            },
        ],
    )

    srdf_file_subtitution = PathJoinSubstitution(
        [
            FindPackageShare("franka_description"),
            "robots",
            arm_id_str,
            f"{arm_id_str}.srdf",
        ]
    )
    srdf_file = srdf_file_subtitution.perform(context)
    with open(srdf_file, "r") as f:
        robot_srdf_description = f.read()

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

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[
            get_use_sim_time(),
            {
                "source_list": [
                    "franka/joint_states",
                    f"{arm_id_str}_gripper/joint_states",
                ],
                "rate": 30,
            },
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
        franka_hardware_launch,
        franka_remote_hardware_launch,
        franka_simulation_launch,
        robot_state_publisher_node,
        robot_collision_publisher_node,
        robot_srdf_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "franka_controllers_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "franka_controllers.yaml",
                ]
            ),
            description="Path to the yaml file use to define controller parameters.",
        ),
        DeclareLaunchArgument(
            "rviz_config_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "rviz",
                    "franka_preview.rviz",
                ]
            ),
            description="Path to RViz configuration file",
        ),
    ]

    return LaunchDescription(
        declared_arguments
        + generate_default_franka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
