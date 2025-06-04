from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from agimus_demos_common.mpc_debugger_node import mpc_debugger_node
from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)


from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_launch,
    get_use_sim_time,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    franka_robot_launch = generate_include_launch("franka_common_lfc.launch.py")

    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_04_visual_servoing"),
            "config",
            "agimus_controller_params.yaml",
        ]
    )
    ocp_definition_file = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_04_visual_servoing"),
            "config",
            "ocp_definition_file.yaml",
        ]
    )
    extra_params = {
        "ocp": {"definition_yaml_file": ocp_definition_file.perform(context)}
    }
    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )
    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[get_use_sim_time(), agimus_controller_yaml, extra_params],
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
            frame_id="world",
            child_frame_id="big_box_root",
        )
        env_nodes = [environment_publisher_node, tf_node]

    trajectory_weights_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_04_visual_servoing"),
            "config",
            "trajectory_weight_params.yaml",
        ]
    )

    reference_publisher_node = Node(
        package="agimus_demo_04_visual_servoing",
        executable="reference_publisher",
        name="reference_publisher",
        output="screen",
        remappings=[("robot_description", "robot_description_with_collision")],
        parameters=[get_use_sim_time(), trajectory_weights_yaml],
    )

    apriltag_tf_to_world_pose_pub = Node(
        package="agimus_demos_common",
        executable="apriltag_tf_to_world_pose",
        name="detection_pub_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    return [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        *env_nodes,
        apriltag_tf_to_world_pose_pub,
        mpc_debugger_node(
            "fer_hand_tcp",
            parent_frame="fer_link0",
            cost_plot=True,
            node_kwargs=dict(
                remappings=[("robot_description", "robot_description_with_collision")]
            ),
        ),
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
    ]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
