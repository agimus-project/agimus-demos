from launch import LaunchContext, LaunchDescription
from launch.actions import (
    ExecuteProcess,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition

from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
    generate_include_launch,
    get_use_sim_time,
)
from agimus_demos_common.mpc_debugger_node import mpc_debugger_node


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
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
    use_gazebo = LaunchConfiguration("use_gazebo")
    use_gazebo_bool = context.perform_substitution(use_gazebo).lower() == "true"

    #
    # OCP
    #
    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
            "config",
            "agimus_controller_params.yaml",
        ]
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
        remappings=[
            ("robot_description_semantic", "robot_srdf_description"),
        ],
    )

    #
    # Orchestrator
    #
    trajectory_weights_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
            "config",
            "trajectory_weights_params.yaml",
        ]
    ).perform(context)
    orchestrator_node = ExecuteProcess(
        cmd=[
            "xterm",
            "-hold",
            "-e",
            'bash -c "source /opt/ros/humble/setup.bash && '
            f'ros2 run agimus_demo_03_mpc_dummy_traj_tiago_pro orchestrator_node.py --ros-args -p use_sim_time:={use_gazebo_bool} --params-file {trajectory_weights_yaml}"',  #
        ],
        output="screen",
    )

    # No collision avoidance case
    environment_publisher_node = Node(
        package="agimus_demos_common",
        executable="string_publisher",
        name="environment_publisher",
        parameters=[
            {
                "topic_name": "environment_description",
                "string_value": "<robot name='empty'><link name='env'/></robot>",
            }
        ],
    )

    #
    # Plotjuggler
    #
    plotjuggler_file = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_03_mpc_dummy_traj_tiago_pro"),
            "config",
            "plotjuggler_view.xml",
        ]
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

    #
    # Initialization
    #
    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )
    tuck_arm = Node(
        package="agimus_demo_03_mpc_dummy_traj_tiago_pro",
        executable="tuck_arm.py",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    #
    # Debugger node
    #
    mpc_debugger = mpc_debugger_node(
        "arm_right_7_joint",
        parent_frame="base_link",
        cost_plot=True,
        node_kwargs=dict(
            remappings=[
                # ("robot_description", "robot_description_with_collision"),
                ("robot_description_semantic", "robot_srdf_description"),
            ],
        ),
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
                    mpc_debugger,
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
