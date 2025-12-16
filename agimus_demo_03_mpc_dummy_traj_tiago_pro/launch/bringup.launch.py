from launch import LaunchContext, LaunchDescription
from launch.actions import (
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition

from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import (
    PathJoinSubstitution,
    Command,
    FindExecutable,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
    generate_include_launch,
    get_use_sim_time,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    use_gazebo = LaunchConfiguration("use_gazebo")

    #
    # Robot
    #
    tiago_robot_launch = generate_include_launch(
        "tiago_pro_common.launch.py",
        extra_launch_arguments={"tuck_arm": "False"},
    )

    #
    # Parameters
    #
    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
            "config",
            "agimus_controller_params.yaml",
        ]
    )

    trajectory_weights_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
            "config",
            "trajectory_weights_params.yaml",
        ]
    )
    environment_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
                        "urdf",
                        "obstacles.xacro",
                    ]
                ),
                # Convert dict to list of parameters
            ]
        ),
        value_type=str,
    )
    plotjuggler_file = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
            "config",
            "plotjuggler_view.xml",
        ]
    )

    #
    # Nodes
    #
    tuck_arm = Node(
        package="agimus_demo_03_mpc_dummy_traj_tiago_pro",
        executable="tuck_arm.py",
        parameters=[get_use_sim_time()],
        output="screen",
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
        parameters=[
            get_use_sim_time(),
            agimus_controller_yaml,
        ],
        output="screen",
        # remappings=[("robot_description", "robot_description_with_collision")],
    )
    orchestrator_node = ExecuteProcess(
        cmd=[
            "xterm",
            "-hold",
            "-e",
            'bash -c "source /opt/ros/humble/setup.bash && '
            f'ros2 run agimus_demo_03_mpc_dummy_traj_tiago_pro orchestrator_node --ros-args -p use_sim_time:={use_gazebo_bool} --params-file {trajectory_weights_yaml}"',  #
        ],
        output="screen",
    )
    environment_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="environment_publisher",
        output="screen",
        remappings=[("robot_description", "environment_description")],
        parameters=[{"robot_description": environment_description}],
    )
    mpc_debugger_node = Node(
        package="agimus_controller_ros",
        executable="mpc_debugger_node",
        name="mpc_debugger_node",
        arguments=[
            "--frame=arm_right_7_joint",
            "--parent-frame=base_footprint",
            "--marker-size=0.05",
        ],
        # Disable ROS 2 arguments
        additional_env={"ROS_DISABLE_ARGUMENTS": "true"},
        output="screen",
    )
    plotjuggler_node = Node(
        package="plotjuggler",
        executable="plotjuggler",
        name="plotjuggler_node",
        arguments=[
            "-l",
            plotjuggler_file,  # Load the layout file
            "--nosplash",  # Skip the splash screen
            "--start_streamer",
            "ros2",  # Automatically start the ROS 2 streamer
        ],
        output="screen",
        condition=IfCondition(use_gazebo),
    )

    environment_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
                        "urdf",
                        "obstacles.xacro",
                    ]
                ),
                # Convert dict to list of parameters
            ]
        ),
        value_type=str,
    )

    return [
        tiago_robot_launch,
        wait_for_non_zero_joints_node,
        environment_publisher_node,
        tuck_arm,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    mpc_debugger_node,
                    orchestrator_node,
                    plotjuggler_node,
                ],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_tiago_pro_args() + [OpaqueFunction(function=launch_setup)]
    )
