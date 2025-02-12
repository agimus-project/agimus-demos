from launch import LaunchContext, LaunchDescription
from launch.actions import OpaqueFunction, ExecuteProcess
from launch.launch_description_entity import LaunchDescriptionEntity
from pathlib import Path

from agimus_demos_common.launch_utils import generate_default_franka_args


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    folder_idx = 0
    folder_path = Path(f"/tmp/rosbag_mpc_data_{folder_idx}")
    while folder_path.exists():
        folder_idx += 1
        folder_path = Path(f"/tmp/rosbag_mpc_data_{folder_idx}")
    rosbag_recorder = ExecuteProcess(
        cmd=[
            "ros2",
            "bag",
            "record",
            "-o",
            str(folder_path),
            "mpc_debug",
            "mpc_input",
            "ocp_solve_time",
            "ocp_x0",
        ],
        output="screen",
    )

    return [rosbag_recorder]


def generate_launch_description():
    return LaunchDescription(
        generate_default_franka_args() + [OpaqueFunction(function=launch_setup)]
    )
