# Copyright (c) 2024 PAL Robotics S.L. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

from launch_pal.arg_utils import LaunchArgumentsBase

from dataclasses import dataclass


@dataclass(frozen=True)
class LaunchArguments(LaunchArgumentsBase):
    pass


def declare_actions(launch_description: LaunchDescription, launch_args: LaunchArguments):

    controller_config = os.path.join(
            get_package_share_directory('agimus_demo_02_simple_pd_plus_tiago_pro'),
            'config', 'joint_state_estimator.yaml')

    joint_state_estimator_spawner_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "joint_state_estimator", "--param-file", controller_config, "--inactive"],
    )
    launch_description.add_action(joint_state_estimator_spawner_node)

    controller_config = os.path.join(
            get_package_share_directory('agimus_demo_02_simple_pd_plus_tiago_pro'),
            'config', 'linear_feedback_controller.yaml')

    linear_feedback_controller_node = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            "linear_feedback_controller", "--param-file", controller_config, "--inactive"],
    )
    launch_description.add_action(linear_feedback_controller_node)

    # PD plus
    pd_plus_controller_yaml = os.path.join(
        get_package_share_directory('agimus_demo_02_simple_pd_plus_tiago_pro'),
        "config",
        "pd_plus_controller.yaml"
    )
    pd_plus_controller = Node(
        package='linear_feedback_controller',
        executable='pd_plus_controller',
        output='screen',
        parameters=[pd_plus_controller_yaml]
    )
    launch_description.add_action(pd_plus_controller)

    return


def generate_launch_description():

    # Create the launch description
    ld = LaunchDescription()

    launch_arguments = LaunchArguments()

    launch_arguments.add_to_launch_description(ld)

    declare_actions(ld, launch_arguments)

    return ld
