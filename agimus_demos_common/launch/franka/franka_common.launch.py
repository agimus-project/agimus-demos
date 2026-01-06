import ast
from copy import deepcopy

from launch import LaunchContext, LaunchDescription
from launch.actions import (
    ExecuteProcess,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    OrSubstitution,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,  # noqa: I001
)

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
    disable_collision_safety = LaunchConfiguration("disable_collision_safety")
    external_controllers_params = LaunchConfiguration("external_controllers_params")
    external_controllers_names = LaunchConfiguration("external_controllers_names")
    franka_controllers_params = LaunchConfiguration("franka_controllers_params")
    initial_joint_position = LaunchConfiguration("initial_joint_position")
    robot_safety_distance = LaunchConfiguration("robot_safety_distance")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_path = LaunchConfiguration("rviz_config_path")
    use_plotjuggler = LaunchConfiguration("use_plotjuggler")
    plotjuggler_config_path = LaunchConfiguration("plotjuggler_config_path")
    use_ft_sensor = LaunchConfiguration("use_ft_sensor")
    ft_sensor_ip = LaunchConfiguration("ft_sensor_ip")
    ee_id = LaunchConfiguration("ee_id")
    use_camera = LaunchConfiguration("use_camera")
    gz_verbose = LaunchConfiguration("gz_verbose")
    gz_headless = LaunchConfiguration("gz_headless")
    gz_world_path = LaunchConfiguration("gz_world_path")

    robot_ip_empty = context.perform_substitution(robot_ip) == ""
    aux_computer_ip_empty = context.perform_substitution(aux_computer_ip) == ""
    aux_computer_user_empty = context.perform_substitution(aux_computer_user) == ""
    use_gazebo_bool = context.perform_substitution(use_gazebo).lower() == "true"
    disable_collision_safety_bool = (
        context.perform_substitution(disable_collision_safety).lower() == "true"
    )
    use_rviz_bool = context.perform_substitution(use_rviz).lower() == "true"
    use_plotjuggler_bool = (
        context.perform_substitution(use_plotjuggler).lower() == "true"
    )
    use_ft_sensor_bool = context.perform_substitution(use_ft_sensor).lower() == "true"
    ft_sensor_ip_empty = context.perform_substitution(ft_sensor_ip) == ""
    ee_id_str = context.perform_substitution(ee_id)
    on_aux_computer_bool = (
        context.perform_substitution(on_aux_computer).lower() == "true"
    )
    external_controllers_params_str = context.perform_substitution(
        external_controllers_params
    )
    external_controllers_names_list = ast.literal_eval(
        context.perform_substitution(external_controllers_names)
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

    if robot_ip_empty and disable_collision_safety_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`disable_collision_safety:=true` and empty `robot_ip`. "
            "Disabling collision safety is only supported on the real robot."
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
            "`on_aux_computer:=true` and non-empty `robot_ip`."
        )

    if not aux_computer_ip_empty and on_aux_computer_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer:=true` and non-empty `aux_computer_ip_empty`."
        )

    if on_aux_computer_bool and use_gazebo_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer:=true` and `use_gazebo:=true`."
        )

    if on_aux_computer_bool and use_rviz_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer:=true` and `use_rviz:=true`."
        )

    if on_aux_computer_bool and use_plotjuggler_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with both "
            "`on_aux_computer:=true` and `use_rviz:=true`."
        )

    if use_ft_sensor_bool and ft_sensor_ip_empty and not robot_ip_empty:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with "
            "`use_ft_sensor:=true` empty `ft_sensor_ip` and non-empty `robot_ip`."
        )

    if not ft_sensor_ip_empty and use_gazebo_bool:
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with "
            "non empty `ft_sensor_ip` and `use_gazebo:=true`."
        )

    if not use_ft_sensor_bool and ee_id_str == "ati_mini45_with_camera":
        raise RuntimeError(
            "Incorrect launch configuration! Can not launch demo with "
            "`use_ft_sensor:=false` and `ee_id:=ati_mini45_with_camera`."
        )

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        name="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        remappings=[("joint_states", "franka/joint_states")],
        output="screen",
        condition=IfCondition(
            OrSubstitution(
                use_gazebo, PythonExpression(["'", aux_computer_ip, "' == ''"])
            )
        ),
    )

    spawn_external_controllers = generate_controllers_spawner_launch_description(
        deepcopy(external_controllers_names_list),
        controller_params_files=(
            [external_controllers_params_str]
            if external_controllers_params_str != ""
            else None
        ),
        extra_spawner_args=[
            "--inactive",
            "--controller-manager-timeout",
            "10000000",
        ],
    )

    print("external_controllers_names_list = ", external_controllers_names_list)
    activate_external_controllers = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "switch_controllers",
            "--activate",
        ]
        + deepcopy(external_controllers_names_list),
        output="screen",
    )

    spawn_external_controllers_on_exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=wait_for_non_zero_joints_node,
            on_exit=[spawn_external_controllers],
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "('",
                    use_gazebo,
                    "' == 'true' or '",
                    aux_computer_ip,
                    "' == '') and ",
                    external_controllers_names,
                    " != ['']",
                ]
            )
        ),
    )

    activate_external_controllers_on_exit_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_external_controllers.entities[2],
            on_exit=[activate_external_controllers],
        ),
        condition=IfCondition(
            PythonExpression(
                [
                    "('",
                    use_gazebo,
                    "' == 'true' or '",
                    aux_computer_ip,
                    "' == '') and ",
                    external_controllers_names,
                    " != ['']",
                ]
            )
        ),
    )

    franka_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        "franka",
                        "franka_hardware.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "arm_id": arm_id,
            "disable_collision_safety": disable_collision_safety,
            "use_ft_sensor": use_ft_sensor,
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
    # evaluating paths to packages that do not exist in the system. From this point on
    # checking if we are running on auxiliary computer is not required.
    if on_aux_computer_bool:
        return [
            franka_hardware_launch,
            wait_for_non_zero_joints_node,
            spawn_external_controllers_on_exit_event,
            activate_external_controllers_on_exit_event,
        ]

    franka_remote_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        "franka",
                        "franka_remote_hardware.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "aux_computer_ip": aux_computer_ip,
            "aux_computer_user": aux_computer_user,
            "arm_id": arm_id,
            "disable_collision_safety": disable_collision_safety,
            "use_ft_sensor": use_ft_sensor,
            "franka_controllers_params": franka_controllers_params,
        }.items(),
        condition=UnlessCondition(
            OrSubstitution(
                use_gazebo, PythonExpression(["'", aux_computer_ip, "' == ''"])
            )
        ),
    )

    franka_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        "franka",
                        "franka_simulation.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "gz_verbose": gz_verbose,
            "gz_headless": gz_headless,
            "gz_world_path": gz_world_path,
            "use_ft_sensor": use_ft_sensor,
            "ee_id": ee_id,
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
        "ee_id": ee_id,
        "use_camera": use_camera,
        "gazebo_effort": "true",
        "with_sc": "false",
        "franka_controllers_params": franka_controllers_params,
        "use_ft_sensor": use_ft_sensor,
        "ft_sensor_ip": ft_sensor_ip,
        "rdt_sampling_rate": str(int(7000 / 6)),
        "internal_filter_rate": "1",
        "use_hardware_biasing": "true",
        "initial_joint_position": initial_joint_position,
        "robot_safety_distance": robot_safety_distance,
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

    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        name="plotjuggler_node",
        arguments=[
            "-l",
            plotjuggler_config_path,
            "--nosplash",
            "--start_streamer",
            "ros2",
        ],
        output="screen",
        condition=IfCondition(use_plotjuggler),
    )

    return [
        franka_hardware_launch,
        franka_remote_hardware_launch,
        franka_simulation_launch,
        wait_for_non_zero_joints_node,
        spawn_external_controllers_on_exit_event,
        activate_external_controllers_on_exit_event,
        robot_state_publisher_node,
        robot_collision_publisher_node,
        robot_srdf_publisher_node,
        joint_state_publisher_node,
        rviz_node,
        plotjuggler_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "external_controllers_params",
            default_value="",
            description="Path to the yaml file use to define external controllers parameters.",
        ),
        DeclareLaunchArgument(
            "external_controllers_names",
            default_value="['']",
            description="List of names of the external controllers to spawn.",
        ),
        DeclareLaunchArgument(
            "franka_controllers_params",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "config",
                    "franka",
                    "controllers.yaml",
                ]
            ),
            description="Path to the yaml file use to define controller parameters.",
        ),
        DeclareLaunchArgument(
            "initial_joint_position",
            default_value="'0.0 -0.78 0.0 -2.35 0.0 1.57 0.78 0.0'",
            description="Initial configuration of the robot used in Gazebo.",
        ),
        DeclareLaunchArgument(
            "robot_safety_distance",
            default_value="0.03",
            description="Safety distance for capsules used by the robot.",
        ),
        DeclareLaunchArgument(
            "gz_world_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("franka_description"),
                    "worlds",
                    "empty.sdf",
                ]
            ),
            description="Path to Gazebo world SDF file.",
        ),
        DeclareLaunchArgument(
            "rviz_config_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "rviz",
                    "franka",
                    "preview.rviz",
                ]
            ),
            description="Path to RViz configuration file",
        ),
        DeclareLaunchArgument(
            "plotjuggler_config_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("agimus_demos_common"),
                    "plotjuggler",
                    "franka",
                    "plotjuggler_mpc_debug.xml",
                ]
            ),
            description="Path to PlotJuggler configuration file",
        ),
    ]

    return LaunchDescription(
        declared_arguments
        + generate_default_franka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
