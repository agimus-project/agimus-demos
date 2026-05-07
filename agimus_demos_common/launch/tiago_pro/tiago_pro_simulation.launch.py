from launch import LaunchContext, LaunchDescription
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetLaunchConfiguration,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from controller_manager.launch_utils import (
    generate_controllers_spawner_launch_description,  # noqa: I001
)
from agimus_demos_common.launch_utils import (
    generate_default_tiago_pro_args,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    controllers_params = [
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demos_common"),
                "config",
                "tiago_pro",
                "dummy_controllers.yaml",
            ]
        ),
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demos_common"),
                "config",
                "tiago_pro",
                "linear_feedback_controller_simu_params.yaml",
            ]
        ),
        PathJoinSubstitution(
            [
                FindPackageShare("agimus_demos_common"),
                "config",
                "tiago_pro",
                "joint_state_estimator_simu_params.yaml",
            ]
        ),
    ]
    controllers = [
        "arm_right_1_joint_inertia_shaping_controller",
        "arm_right_2_joint_inertia_shaping_controller",
        "arm_right_3_joint_inertia_shaping_controller",
        "arm_right_4_joint_inertia_shaping_controller",
        "arm_right_5_joint_inertia_shaping_controller",
        "arm_right_6_joint_inertia_shaping_controller",
        "arm_right_7_joint_inertia_shaping_controller",
        "linear_feedback_controller",
        "joint_state_estimator",
    ]
    # Spawn external controllers, namely the lfc.
    spawn_controllers = generate_controllers_spawner_launch_description(
        controller_names=controllers,
        controller_params_files=controllers_params,
        extra_spawner_args=[
            "--inactive",
            "--controller-manager-timeout",
            "10000000",
        ],
    )

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
            **{
                arg.name: LaunchConfiguration(arg.name)
                for arg in generate_default_tiago_pro_args()
            },
            "tuck_arm": "False",
            "is_public_sim": "True",
            "moveit": "False",
            "play_motion2": "False",
        }.items(),
    )

    # Activate all at once: ros2_control sets chained mode atomically
    # when preceding (LFC) and following (PassthroughControllers) are
    # activated in the same switch_controllers transaction.
    activate_controllers = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "switch_controllers",
            "--deactivate",
            "arm_right_controller",
            "--activate",
            "arm_right_1_joint_inertia_shaping_controller",
            "arm_right_2_joint_inertia_shaping_controller",
            "arm_right_3_joint_inertia_shaping_controller",
            "arm_right_4_joint_inertia_shaping_controller",
            "arm_right_5_joint_inertia_shaping_controller",
            "arm_right_6_joint_inertia_shaping_controller",
            "arm_right_7_joint_inertia_shaping_controller",
            "linear_feedback_controller",
            "joint_state_estimator",
        ],
        output="screen",
    )

    return [
        spawn_controllers,
        tiago_simulation_launch,
        RegisterEventHandler(
            OnProcessExit(
                target_action=spawn_controllers.entities[2],
                on_exit=[activate_controllers],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [SetLaunchConfiguration("use_sim_time", "True")]
        + generate_default_tiago_pro_args()
        + [OpaqueFunction(function=launch_setup)]
    )
