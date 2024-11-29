from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.parameter_descriptions import ParameterValue

from controller_manager.launch_utils import generate_load_controller_launch_description


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:

    arm_id = LaunchConfiguration("arm_id")
    robot_ip = LaunchConfiguration("robot_ip")
    load_gripper = LaunchConfiguration("load_gripper")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    franka_controllers_params = LaunchConfiguration("franka_controllers_params")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config_path = LaunchConfiguration("rviz_config_path")

    arm_id_str = context.perform_substitution(arm_id)
    franka_controllers_params_str = context.perform_substitution(franka_controllers_params)

    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name="xacro")]),
                " ",
                PathJoinSubstitution(
                    [
                        FindPackageShare("franka_description"),
                        "robots",
                        arm_id_str,
                        f"{arm_id_str}.urdf.xacro",
                    ]
                ),
                " ros2_control:=",
                "true",
                " arm_id:=",
                arm_id,
                " robot_ip:=",
                robot_ip,
                " hand:=",
                load_gripper,
                " use_fake_hardware:=",
                use_fake_hardware,
                " fake_sensor_commands:=",
                fake_sensor_commands,
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            franka_controllers_params_str,
            {"robot_description": robot_description},
            {"arm_id": arm_id},
            {"load_gripper": load_gripper},
        ],
        remappings=[("joint_states", "franka/joint_states")],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },
        on_exit=Shutdown(),
    )
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        parameters=[
            {
                "source_list": [
                    "franka/joint_states",
                    f"{arm_id_str}_gripper/joint_states",
                ],
                "rate": 30,
            }
        ],
    )
    load_joint_state_broadcaster = generate_load_controller_launch_description(
        "joint_state_broadcaster"
    )
    load_franka_robot_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["franka_robot_state_broadcaster"],
        parameters=[{"arm_id": arm_id}],
        output="screen",
        condition=UnlessCondition(use_fake_hardware),
    )
    franka_gripper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("franka_gripper"),
                        "launch",
                        "gripper.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
        }.items(),
        condition=IfCondition(load_gripper),
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["--display-config", rviz_config_path],
        condition=IfCondition(use_rviz),
    )
    return [
        robot_state_publisher_node,
        controller_manager_node,
        joint_state_publisher_node,
        load_joint_state_broadcaster,
        load_franka_robot_state_broadcaster,
        franka_gripper_launch,
        rviz_node,
    ]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "robot_ip",
            description="Hostname or IP address of the robot.",
        ),
        DeclareLaunchArgument(
            "arm_id",
            description="ID of the type of arm used. Supported values: fer, fr3, fp3",
        ),
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware",
        ),
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description='Fake sensor commands. Only valid when "{}" is true'.format(
                "use_fake_hardware"
            ),
        ),
        DeclareLaunchArgument(
            "franka_controllers_params",
            description="Path to the yaml file use to define controller parameters.",
        ),
        DeclareLaunchArgument(
            "load_gripper",
            default_value="true",
            description="Use Franka Gripper as an end-effector, otherwise, the robot is loaded "
            "without an end-effector.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="false",
            description="Visualize the robot in RViz",
        ),
        DeclareLaunchArgument(
            "rviz_config_path",
            default_value=PathJoinSubstitution(
                [
                    FindPackageShare("franka_description"),
                    "rviz",
                    "visualize_franka.rviz",
                ]
            ),
            description="Path to RViz configuration file",
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
