from launch import LaunchContext, LaunchDescription
from launch.conditions import IfCondition
from launch.actions import OpaqueFunction, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)
from agimus_demos_common.mpc_debugger_node import mpc_debugger_node

from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_launch,
    get_use_sim_time,
    parse_config,
    safe_remove,
)
import os

def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    use_mpc_debugger = LaunchConfiguration("use_mpc_debugger")
    tracked_object_name = LaunchConfiguration("tracked_object_name")
    arm_id_str = LaunchConfiguration("arm_id").perform(context)

    franka_robot_launch = generate_include_launch("franka_common_lfc.launch.py")

    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_04_visual_servoing"),
            "config",
            "agimus_controller_params.yaml",
        ]
    )

    ocp_definition_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_04_visual_servoing"),
            "config",
            "ocp_definition_file.yaml",
        ]
    )

    replacements = {"arm_id": arm_id_str}
    ocp_definition_yaml_file = parse_config(
        path=ocp_definition_yaml.perform(context), replacements=replacements
    )
    replacements_agimus_controller = {
        "arm_id": arm_id_str,
        "ocp_file": ocp_definition_yaml_file,
    }
    agimus_controller_yaml_file = parse_config(
        path=agimus_controller_yaml.perform(context),
        replacements=replacements_agimus_controller,
    )
    extra_params = {"ocp": {"definition_yaml_file": ocp_definition_yaml_file}}

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )
    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[get_use_sim_time(), agimus_controller_yaml_file, extra_params],
        output="screen",
        remappings=[("robot_description", "robot_description_with_collision")],
    )
    use_happypose = False
    vision_nodes: list[Node] = []
    if use_happypose:
        happypose_to_tf_node = Node(
            package="agimus_demos_common",
            executable="happypose_to_tf_node",
            parameters=[get_use_sim_time()],
            output="screen",
        )
        vision_nodes.append(happypose_to_tf_node)
    else:
        pass

    env_nodes: list[Node]
    if False:
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
        env_nodes = [environment_publisher_node]
    else:
        # With collision avoidance case.
        env_desc_file_sub = PathJoinSubstitution(
            [
                FindPackageShare("agimus_demo_04_visual_servoing"),
                "urdf",
                "big_box.urdf",
            ]
        )
        env_desc_file = env_desc_file_sub.perform(context)
        with open(env_desc_file, "r") as f:
            environment_description = f.read()
        environment_publisher_node = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="environment_publisher",
            output="screen",
            remappings=[("robot_description", "environment_description")],
            parameters=[{"robot_description": environment_description}],
        )
        tf_node = static_transform_publisher_node(
            frame_id=arm_id_str+"_link0",
            child_frame_id="big_box_root",
        )
        tf_node_2 = static_transform_publisher_node(
            frame_id=arm_id_str+"_link0",
            child_frame_id="big_box_root",
        )
        tf_node_3 = static_transform_publisher_node(
            frame_id=tracked_object_name,
            child_frame_id="current_object",
        )
        env_nodes = [environment_publisher_node, tf_node, tf_node_2, tf_node_3]

    trajectory_weights_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_04_visual_servoing"),
            "config",
            "trajectory_weights_params.yaml",
        ]
    )
    trajectory_weights_yaml_file = parse_config(path=trajectory_weights_yaml.perform(context), replacements=replacements)

    reference_publisher_node = Node(
        package="agimus_demo_04_visual_servoing",
        executable="reference_publisher",
        name="reference_publisher",
        output="screen",
        remappings=[("robot_description", "robot_description_with_collision")],
        parameters=[get_use_sim_time(), trajectory_weights_yaml_file],
    )

    mpc_debugger = mpc_debugger_node(
        arm_id_str+"_hand_tcp",
        parent_frame=arm_id_str+"_link0",
        cost_plot=True,
        node_kwargs=dict(
            remappings=[("robot_description", "robot_description_with_collision")],
            condition=IfCondition(use_mpc_debugger),
        ),
    )
    # Cleanup temporary file on shutdown
    cleanup_action = RegisterEventHandler(
        OnShutdown(
            on_shutdown=lambda event, context: (
                safe_remove(ocp_definition_yaml_file),
                safe_remove(agimus_controller_yaml_file),
                safe_remove(trajectory_weights_yaml_file),
            )
        )
    )

    return [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        *env_nodes,
        mpc_debugger,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    reference_publisher_node,
                ]
                + vision_nodes,
            )
        ),
        cleanup_action,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "use_mpc_debugger",
            default_value="false",
            description="Launches the mpc_debugger_node along.",
            choices=["true", "false"],
        ),
        DeclareLaunchArgument(
            "tracked_object_name",
            default_value="big_tag1",
            description="Name of the tracked object, as appearing in the tf2 tree.",
        ),
    ]
    return LaunchDescription(
        declared_arguments
        + generate_default_franka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
