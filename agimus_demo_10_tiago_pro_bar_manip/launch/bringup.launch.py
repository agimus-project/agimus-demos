"""
Bringup launchfile of demo n°10 : TIAGoPro bar bi-manipulation

Usage:
  ros2 launch agimus_demo_10_tiago_pro_bar_manip bringup.launch.py use_gazebo:=true use_sim_time:=True
"""

from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, FindExecutable

from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
    generate_include_launch,
    get_use_sim_time,
)
from agimus_demos_common.static_transform_publisher_node import (
    static_transform_publisher_node,
)
import numpy as np
import pinocchio as pin
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.actions import SetEnvironmentVariable
import os

PKG_NAME = "agimus_demo_10_tiago_pro_bar_manip"


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    ref_frame = LaunchConfiguration("ref_frame").perform(context)
    # plotjuggler_config = LaunchConfiguration("plotjuggler_config")

    # ==========================================================================
    # Tiago pro simulation
    # ==========================================================================
    tiago_robot_launch = generate_include_launch(
        "tiago_pro_common.launch.py",
        extra_launch_arguments={
            "tuck_arm": "False",
            "launch_lfc": "true",
            "lfc_pkg": "agimus_demos_common",
            "lfc_yaml": "config/tiago_pro/linear_feedback_controller_simu_params.yaml",
            "jse_yaml": "config/tiago_pro/joint_state_estimator_simu_params.yaml",
            "pc_yaml": "config/tiago_pro/dummy_controllers.yaml",
            "end_effector_right": "pal-pro-gripper",
            "end_effector_left": "pal-pro-gripper",
        },
    )

    # ==========================================================================
    # Orchestrator weights and executable
    # ! Launched by hand for now
    # ==========================================================================

    # orchestrator_hpp_config_path = PathJoinSubstitution(
    #     [
    #         FindPackageShare(PKG_NAME),
    #         "config",
    #         "planning",
    #         "orchestrator_hpp_config.yaml",
    #     ]
    # )

    # orchestrator = Node(
    #     package=PKG_NAME,
    #     executable="orchestrator_node",
    #     parameters=[
    #         {"orchestrator_hpp_config": orchestrator_hpp_config_path}
    #     ],
    #     name="orchestrator",
    #     output="screen",
    # )

    # Used to signal the publishing of valid joint values
    # wait_for_non_zero_joints_node = Node(
    #     package="agimus_demos_common",
    #     executable="wait_for_non_zero_joints_node",
    #     parameters=[get_use_sim_time()],
    #     output="screen",
    # )
    # ==========================================================================
    # Rviz config and executable
    # ==========================================================================

    rviz_config_path = PathJoinSubstitution(
        [
            FindPackageShare(PKG_NAME),
            "rviz",
            "config.rviz",
        ]
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        parameters=[{"use_sim_time": get_use_sim_time()}],
        arguments=["-d", rviz_config_path.perform(context)],
    )

    # ==========================================================================
    # HPP / environment description
    # ==========================================================================

    env_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare(PKG_NAME),
                        "urdf",
                        "environment.urdf.xacro",
                    ]
                ),
            ]
        ),
        value_type=str,
    )

    env_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="env_publisher",
        output="screen",
        parameters=[get_use_sim_time(), {"robot_description": env_description}],
        remappings=[("robot_description", "environment_description")],
    )

    bar_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare(PKG_NAME),
                        "urdf/standalone",
                        "reinforcement_bar.urdf.xacro",
                    ]
                ),
            ]
        ),
        value_type=str,
    )

    bar_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="bar_publisher",
        output="screen",
        parameters=[
            get_use_sim_time(),
            {"robot_description": bar_description},
        ],
        remappings=[("robot_description", "bar_description")],
    )

    spawn_environment = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "environment", "-topic", "environment_description"],
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
        output="screen",
    )

    spawn_bar = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            "reinforcement_bar",
            "-topic",
            "bar_description",
            "-x",
            "1.56",
            "-y",
            "0.0",
            "-z",
            "0.76",
            "-R",
            "1.5708",  # roll
            "-P",
            "0",  # pitch
            "-Y",
            "0",  # yaw
        ],
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
        output="screen",
    )

    environment_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/model/environment/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"],
        remappings=[("/model/environment/pose", "/tf")],
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
        output="screen",
    )

    bar_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/model/reinforcement_bar/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
        ],
        remappings=[("/model/reinforcement_bar/pose", "/tf")],
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
        output="screen",
    )
    bar_tf_bridge = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="bar_tf_bridge",
        arguments=[
            "0",
            "0",
            "0",
            "0",
            "0",
            "0",
            "reinforcement_bar/bar_base_link",  # Parent (Gazebo)
            "bar_base_link",  # Enfant (URDF)
        ],
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
        output="screen",
    )

    # # ==========================================================================
    # # Static TF fallback (real robot / no Gazebo): only used when use_gazebo
    # # is false, since in simulation these frames are now driven by the
    # # Gazebo ground-truth pose bridge above.
    # # ==========================================================================

    quat_values = pin.Quaternion(pin.rpy.rpyToMatrix(np.array([np.pi / 2, 0, 0])))

    tf_goal_bar = static_transform_publisher_node(
        frame_id="table_link",
        child_frame_id="bar_goal_pose",
        xyz=["-1.3", "0.", "0.7"],
        rot_xyzw=quat_values.coeffs().tolist(),  # [x, y, z, w]
    )

    quat_values = pin.Quaternion(pin.rpy.rpyToMatrix(np.array([0, 0, 0])))
    world_odom = static_transform_publisher_node(
        frame_id=f"{ref_frame}",
        child_frame_id="odom",
        xyz=["0", "0", "0."],
        rot_xyzw=quat_values.coeffs().tolist(),
    )

    tf_odom = Node(
        package=PKG_NAME,
        executable="tf_base_publisher",
        name="tf_base_publisher",
        output="screen",
        parameters=[{"parent_frame": "odom", "child_frame": "base_footprint"}],
    )

    # ==========================================================================
    # Agimus-controller (MPC)
    # ==========================================================================

    agimus_controller_node = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        parameters=[
            get_use_sim_time(),
            PathJoinSubstitution(
                [
                    FindPackageShare(PKG_NAME),
                    "config",
                    "agimus_controller",
                    "agimus_controller_params.yaml",
                ]
            ),
        ],
        output="screen",
    )

    robot_srdf_publisher_node = Node(
        package="agimus_demos_common",
        executable="string_publisher",
        name="robot_srdf_description_publisher",
        output="screen",
        parameters=[
            {
                "topic_name": "robot_srdf_description",
                "string_value": ParameterValue(
                    Command(
                        [
                            PathJoinSubstitution([FindExecutable(name="xacro")]),
                            " ",
                            PathJoinSubstitution(
                                [
                                    FindPackageShare("agimus_demos_common"),
                                    "config",
                                    "tiago_pro",
                                    "tiago_pro_dummy.srdf.xacro",
                                ]
                            ),
                        ]
                    ),
                    value_type=str,
                ),
            }
        ],
    )

    # plotjuggler = Node(
    #     package="plotjuggler",
    #     executable="plotjuggler",
    #     arguments=[
    #         "--layout",
    #         plotjuggler_config,
    #     ],
    #     output="screen",
    # )

    set_gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=os.pathsep.join(
            [
                os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
                os.path.join(get_package_share_directory(PKG_NAME), ".."),
            ]
        ),
    )
    world_to_gazebo_bridge = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_gazebo_bridge",
        arguments=["0", "0", "0", "0", "0", "0", "world", "empty"],
        condition=IfCondition(LaunchConfiguration("use_gazebo")),
        output="screen",
    )

    gz_bridge_odom = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tiago_gz_bridge_odom",
        parameters=[
            {
                "config_file": PathJoinSubstitution(
                    [FindPackageShare(PKG_NAME), "config", "bridge_gz.yaml"]
                ),
                "use_sim_time": True,
            }
        ],
        output="screen",
    )

    return [
        set_gz_resource_path,
        tiago_robot_launch,
        rviz,
        # wait_for_non_zero_joints_node,
        env_publisher,
        spawn_environment,
        bar_publisher,
        spawn_bar,
        environment_pose_bridge,
        bar_pose_bridge,
        tf_odom,
        world_odom,
        # orchestrator,
        tf_goal_bar,
        robot_srdf_publisher_node,
        agimus_controller_node,
        bar_tf_bridge,
        # plotjuggler,
        world_to_gazebo_bridge,
        gz_bridge_odom,
    ]


def generate_launch_description():
    args = generate_default_tiago_pro_args()
    args.append(
        DeclareLaunchArgument(
            "ref_frame",
            default_value="world",
            description="Reference frame for the demo (world, map, odom...)",
        )
    )
    args.append(
        DeclareLaunchArgument(
            "plotjuggler_config",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare(PKG_NAME),
                    "config",
                    "plotjuggler.xml",
                ]
            ),
            description="PlotJuggler layout/config file",
        )
    )
    return LaunchDescription(args + [OpaqueFunction(function=launch_setup)])
