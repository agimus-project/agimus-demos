from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from controller_manager.launch_utils import generate_load_controller_launch_description


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    robot_ip = LaunchConfiguration("robot_ip")
    arm_id = LaunchConfiguration("arm_id")
    load_gripper = LaunchConfiguration("load_gripper")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    use_rviz = LaunchConfiguration("use_rviz")
    package_name = "agimus_demo_02_simple_pd_plus"

    franka_controllers_params = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "controller_manager.yaml",
        ]
    )

    joint_state_estimator_params = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "joint_state_estimator.yaml",
        ]
    )
    load_joint_state_estimator = generate_load_controller_launch_description(
        controller_name="joint_state_estimator",
        controller_params_file=joint_state_estimator_params,
    )

    linear_feedback_controller_params = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "linear_feedback_controller.yaml",
        ]
    )
    load_linear_feedback_controller = generate_load_controller_launch_description(
        controller_name="linear_feedback_controller",
        controller_params_file=linear_feedback_controller_params,
    )

    franka_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("agimus_demos_common"),
                        "launch",
                        "franka.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "arm_id": arm_id,
            "load_gripper": load_gripper,
            "use_fake_hardware": use_fake_hardware,
            "fake_sensor_commands": fake_sensor_commands,
            "use_rviz": use_rviz,
            "franka_controllers_params": franka_controllers_params,
        }.items(),
    )

    # MPC
    mpc_yaml = PathJoinSubstitution(
        [
            FindPackageShare(package_name),
            "config",
            "mpc_params.yaml",
        ]
    )
    mpc_controller = Node(
        package="agimus_controller_ros",
        executable="agimus_controller_node",
        name="agimus_controller_node",
        output="screen",
        parameters=[mpc_yaml],
    )

    return [
        franka_bringup_launch,
        load_linear_feedback_controller,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_linear_feedback_controller.entities[-1],
                on_exit=[load_joint_state_estimator],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_estimator.entities[-1],
                on_exit=[mpc_controller],
            )
        ),
        Node(
            package="agimus_controller_ros",
            executable="mpc_input_dummy_publisher",
            name="mpc_input_dummy_publisher",
            output="screen",
        ),
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            description="Hostname or IP address of the robot.",
        ),
        DeclareLaunchArgument(
            "arm_id",
            default_value="fer",
            description="ID of the type of arm used. Supported values: fer, fr3, fp3",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Visualize the robot in Rviz",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Fake sensor commands. Only valid when '{}' is true".format(
                "use_fake_hardware"
            ),
        ),
        DeclareLaunchArgument(
            "load_gripper",
            default_value="true",
            description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded "
            "without an end-effector.",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
