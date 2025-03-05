from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    tiago_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("tiago_pro_gazebo"),
                        "launch",
                        "tiago_pro_gazebo.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={
            "world_name": LaunchConfiguration("world_name"),
            "base_type": LaunchConfiguration("base_type"),
            "arm_type_right": LaunchConfiguration("arm_type_right"),
            "arm_type_left": LaunchConfiguration("arm_type_left"),
            "end_effector_right": LaunchConfiguration("end_effector_right"),
            "end_effector_left": LaunchConfiguration("end_effector_left"),
            "ft_sensor_right": LaunchConfiguration("ft_sensor_right"),
            "ft_sensor_left": LaunchConfiguration("ft_sensor_left"),
            "tool_changer_right": LaunchConfiguration("tool_changer_right"),
            "tool_changer_left": LaunchConfiguration("tool_changer_left"),
            "wrist_model_right": LaunchConfiguration("wrist_model_right"),
            "wrist_model_left": LaunchConfiguration("wrist_model_left"),
            "camera_model": LaunchConfiguration("camera_model"),
            "laser_model": LaunchConfiguration("laser_model"),
            "navigation": LaunchConfiguration("navigation"),
            "advanced_navigation": LaunchConfiguration("advanced_navigation"),
            "slam": LaunchConfiguration("slam"),
            "docking": LaunchConfiguration("docking"),
            "moveit": LaunchConfiguration("moveit"),
            "tuck_arm": LaunchConfiguration("tuck_arm"),
            "is_public_sim": LaunchConfiguration("is_public_sim", default=True),
        }.items(),
    )

    return [tiago_simulation_launch]


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "world_name",
            default_value="empty",
            description="Gazebo world name from the pal_gazebo_worlds package.",
            choices=["empty", "pal_office"],
        ),
        DeclareLaunchArgument(
            "base_type",
            description="Base type.",
            choices=["omni_base"],
            default_value="omni_base",
        ),
        DeclareLaunchArgument(
            "arm_type_right",
            description="Arm type right.",
            choices=["tiago-pro", "no-arm"],
            default_value="tiago-pro",
        ),
        DeclareLaunchArgument(
            "arm_type_left",
            description="Arm type left.",
            choices=["tiago-pro", "no-arm"],
            default_value="tiago-pro",
        ),
        DeclareLaunchArgument(
            "end_effector_right",
            description="End effector model right arm.",
            choices=["pal-pro-gripper", "custom", "no-end-effector"],
            default_value="pal-pro-gripper",
        ),
        DeclareLaunchArgument(
            "end_effector_left",
            description="End effector model left arm.",
            choices=["pal-pro-gripper", "custom", "no-end-effector"],
            default_value="pal-pro-gripper",
        ),
        DeclareLaunchArgument(
            "ft_sensor_right",
            description="FT sensor model right arm.",
            choices=["ati", "rokubi", "no-ft-sensor"],
            default_value="no-ft-sensor",
        ),
        DeclareLaunchArgument(
            "ft_sensor_left",
            description="FT sensor model left arm.",
            choices=["ati", "rokubi", "no-ft-sensor"],
            default_value="no-ft-sensor",
        ),
        DeclareLaunchArgument(
            "tool_changer_right",
            description="The arm has a tool changer.",
            choices=["False", "True"],
            default_value="True",
        ),
        DeclareLaunchArgument(
            "tool_changer_left",
            description="The arm has a tool changer.",
            choices=["False", "True"],
            default_value="True",
        ),
        DeclareLaunchArgument(
            "wrist_model_right",
            description="Wrist model right.",
            choices=["spherical-wrist", "straight-wrist"],
            default_value="spherical-wrist",
        ),
        DeclareLaunchArgument(
            "wrist_model_left",
            description="Wrist model left.",
            choices=["spherical-wrist", "straight-wrist"],
            default_value="spherical-wrist",
        ),
        DeclareLaunchArgument(
            "camera_model",
            description="Head camera model.",
            choices=["realsense-d435", "realsense-d455"],
            default_value="realsense-d435",
        ),
        DeclareLaunchArgument(
            "laser_model",
            description="Base laser model.",
            choices=["no-laser", "sick-571"],
            default_value="sick-571",
        ),
        DeclareLaunchArgument(
            "navigation",
            description="Specify if launching Navigation2.",
            choices=["True", "False"],
            default_value="False",
        ),
        DeclareLaunchArgument(
            "advanced_navigation",
            description="Specify if launching Advanced Navigation.",
            choices=["True", "False"],
            default_value="False",
        ),
        DeclareLaunchArgument(
            "slam",
            description="Whether or not you are using SLAM",
            default_value="False",
            choices=["True", "False"],
        ),
        DeclareLaunchArgument(
            "docking",
            description="Specify if launching Docking.",
            choices=["True", "False"],
            default_value="False",
        ),
        DeclareLaunchArgument(
            "moveit",
            description="Specify if launching MoveIt 2.",
            choices=["True", "False"],
            default_value="True",
        ),
        DeclareLaunchArgument(
            "world_name",
            description="Specify world name, will be converted to full path.",
            default_value="empty",
        ),
        DeclareLaunchArgument(
            "tuck_arm",
            description="Launches tuck arm node.",
            choices=["True", "False"],
            default_value="True",
        ),
        DeclareLaunchArgument(
            "is_public_sim",
            description="Enable public simulation.",
            choices=["True", "False"],
            default_value="False",
        ),
    ]
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
