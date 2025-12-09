from pathlib import Path
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    OpaqueFunction,
    RegisterEventHandler,
    ExecuteProcess,
    DeclareLaunchArgument,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, FindExecutable
from launch_ros.parameter_descriptions import ParameterValue


from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_launch,
    get_use_sim_time,
    parse_config,
    safe_remove,
)
from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    franka_robot_launch = generate_include_launch("franka_common_lfc.launch.py")
    vision_type_arg = LaunchConfiguration("vision_type")
    vision_type = context.perform_substitution(vision_type_arg).lower()
    dataset_name_arg = LaunchConfiguration("dataset_name")
    dataset_name = context.perform_substitution(dataset_name_arg).lower()

    agimus_controller_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_05_pick_and_place"),
            "config",
            "agimus_controller_params.yaml",
        ]
    )
    ocp_definition_yaml = PathJoinSubstitution(
        [
                FindPackageShare("agimus_demo_05_pick_and_place"),
                "config",
                "ocp_definition_file.yaml"
        ]
    )
    replacements = {
        'arm_id': arm_id_str,
    }
    ocp_definition_yaml_file = parse_config(path=ocp_definition_yaml.perform(context), replacements=replacements)
    replacements_agimus_controller = {
        'arm_id': arm_id_str,
        'ocp_file': ocp_definition_yaml_file,
    }
    agimus_controller_yaml_file = parse_config(path=agimus_controller_yaml.perform(context), replacements=replacements_agimus_controller)

    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )
    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[get_use_sim_time(), agimus_controller_yaml_file],
        output="screen",
        remappings=[("robot_description", "robot_description_with_collision")],
    )
    happypose_to_tf_node = Node(
        package="agimus_demos_common",
        executable="happypose_to_tf_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )
    environment_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demo_05_pick_and_place"),
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
        parameters=[{"robot_description": environment_description}],
    )
    tf_node = static_transform_publisher_node(
        frame_id="robot_attachment_link",
        child_frame_id="world",
    )
    object_name = "tless-obj_000031"  # todo: fix this better later
    tf_node_2 = static_transform_publisher_node(
        frame_id=object_name,
        child_frame_id="current_object",
    )
    tf_node_support_link = static_transform_publisher_node(
        frame_id="support_link",
        child_frame_id="base",
        xyz=["0.563", "-0.166", "0.780"],
        rot_xyzw=["0.000", "0.000", "1.000", "0.000"],
    )
    env_nodes = [environment_publisher_node, tf_node, tf_node_2, tf_node_support_link]

    # add simulation of vision detection
    if vision_type in ["simulate_happypose", "simulate_apriltag_det"]:
        simulated_object_pose = [0.2, -0.1, 0.95, 0.0, 0.0, 0.707, 0.707]
        if vision_type == "simulate_apriltag_det":
            simulated_object_pose_as_str = [str(val) for val in simulated_object_pose]
            tf_node_object_detection = static_transform_publisher_node(
                frame_id="support_link",
                child_frame_id=object_name,
                xyz=simulated_object_pose_as_str[:3],
                rot_xyzw=simulated_object_pose_as_str[3:],
            )
            env_nodes.append(tf_node_object_detection)
        elif vision_type == "simulate_happypose":
            happypose_simulation_params = {
                "object_id": object_name,
                "base_name": "support_link",
                "camera_name": "camera_color_optical_frame",
                "object_pose_in_base_txyz": simulated_object_pose[:3],
                "object_pose_in_base_qxyzw": simulated_object_pose[3:],
            }
            happypose_simulation_node = Node(
                package="agimus_demos_common",
                executable="happypose_simulation",
                parameters=[get_use_sim_time(), happypose_simulation_params],
                output="screen",
            )

    trajectory_weights_yaml = PathJoinSubstitution(
        [
            FindPackageShare("agimus_demo_05_pick_and_place"),
            "config",
            "trajectory_weights_params.yaml",
        ]
    )
    trajectory_weights_yaml_file = parse_config(path=trajectory_weights_yaml.perform(context), replacements=replacements)

    use_gazebo = LaunchConfiguration("use_gazebo")
    use_gazebo_bool = context.perform_substitution(use_gazebo).lower() == "true"
    pick_and_place_node = ExecuteProcess(
        cmd=[
            "xterm",
            "-hold",
            "-e",
            'bash -c "source /opt/ros/humble/setup.bash && '
            f'ros2 run agimus_demo_05_pick_and_place pick_and_place_node --ros-args -p use_sim_time:={use_gazebo_bool} -p vision_type:={vision_type} -p dataset_name:={dataset_name} --params-file {trajectory_weights_yaml}"',  #
        ],
        output="screen",
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

    nodes_to_launch = [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        *env_nodes,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[
                    agimus_controller_node,
                    happypose_to_tf_node,
                    pick_and_place_node,
                ],
            )
        ),
        cleanup_action,
    ]
    if vision_type == "simulate_happypose":
        nodes_to_launch.append(happypose_simulation_node)
    elif vision_type in ["simulate_apriltag_det", "apriltag_det"]:
        apriltag_tf_to_world_pose_pub = Node(
            package="olt_ros2_pipeline",
            executable="apriltag_tf_to_world",
            name="detection_pub_node",
            parameters=[get_use_sim_time()],
            output="screen",
        )
        nodes_to_launch.append(apriltag_tf_to_world_pose_pub)
    return nodes_to_launch


def generate_launch_description():
    vision_type = DeclareLaunchArgument(
        "vision_type",
        default_value="apriltag_det",
        choices=[
            "simulate_happypose",
            "simulate_apriltag_det",
            "happypose",
            "apriltag_det",
        ],
        description="Type of vision used.",
    )
    dataset_name = DeclareLaunchArgument(
        "dataset_name",
        default_value="tless",
        choices=["tless", "ycbv"],
        description="Dataset used.",
    )
    return LaunchDescription(
        [vision_type]
        + [dataset_name]
        + generate_default_franka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
