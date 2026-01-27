from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler, TimerAction, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterFile, ParameterValue
from launch.substitutions import Command, FindExecutable, LaunchConfiguration

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_launch,
    get_use_sim_time,
)
from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    arm_id_str = LaunchConfiguration("arm_id").perform(context)
    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_09_glue_spreading"),
            "rviz",
            "config.rviz",
        ]
    )

    franka_robot_launch = generate_include_launch(
        "franka_common_lfc.launch.py",
        extra_launch_arguments={"rviz_config_path": rviz_config_path,"arm_id": arm_id_str,"use_camera":"false", "initial_joint_position":"'-0.013754730661780176 0.08055123973596531 -0.0073070470163592634 -2.1355798783235875 -0.00918790986315326 2.2759845504437037 -2.3657036209762725 0. '"},
    )

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    # agimus_controller_params = PathJoinSubstitution(
    #     [
    #         FindPackageShare("agimus_demo_08_collision_avoidance"),
    #         "config",
    #         "agimus_controller_params.yaml",
    #     ]
    # )

    # agimus_controller_node = Node(
    #     package="agimus_controller_ros",
    #     executable="agimus_controller_node",
    #     parameters=[
    #         get_use_sim_time(),
    #         agimus_controller_params,
    #     ],
    #     output="screen",
    #     remappings=[("robot_description", "/robot_description_with_collision")],
    # )

    environment_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_09_glue_spreading"),
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

    plate_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_09_glue_spreading"),
                        "urdf",
                        "plate.urdf.xacro",
                    ]
                ),
            ]
        ),
        value_type=str,
    )
    plate_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="plate_publisher",
        output="screen",
        parameters=[get_use_sim_time(), {"robot_description": plate_description}],
        remappings=[("robot_description", "plate_description")],
    )

    tf_node = static_transform_publisher_node(
        frame_id=arm_id_str+"_link0",
        child_frame_id="robot_attachment_link",
    )
    # tf_node_plate = static_transform_publisher_node(
    #     frame_id="plate_base_link_happypose",
    #     child_frame_id="pannel_base_link",
    #     xyz=["0.0", "0.0", "0.0"],
    #     rot_xyzw=["-0.7071","0.0"," 0.0", "0.7071"],
    # )

    tf_node_plate_mpc = static_transform_publisher_node(
    frame_id=arm_id_str+"_link0",
    child_frame_id="pannel_base_link",
    xyz=["0.6", "0", "0.0"],
    rot_xyzw= ["0", "0", "0", "1"],
    )

    mpc_params = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_09_glue_spreading"),
            "config",
            "mpc.yaml",
        ]
    )

    mpc_node = Node(
        package = "agimus_demo_09_glue_spreading",
        executable = "aligator_MPC",
        name = "aligator_mpc",
        parameters = [{'config':mpc_params.perform(context)}]
    )


    return [
        franka_robot_launch,
        environment_publisher_node,
        plate_publisher_node,
        tf_node,
        tf_node_plate_mpc,
        # mpc_node,

        # wait_for_non_zero_joints_node,
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=wait_for_non_zero_joints_node,
        #         on_exit=[
        #             # agimus_controller_node,
        #             # obstacle_posstart_posee_publisher_node,
        #         ],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessStart(
        #         target_action=agimus_controller_node,
        #         on_start=TimerAction(
        #             period=5.0,
        #             actions=[goal_publisher_node],
        #         ),
        #     )
        # ),
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
