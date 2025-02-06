# Agimus Demos Common

Package containing common launch and config files used across different demos.

This package intends to be used as a common source of basic launch files allowing to bring up both simulation and real robot in a unified manner.

> [!NOTE]
> All launch files specific to robot are expected to share common set of launch arguments. For Franka robots this common set of arguments should be generated with function `generate_default_franka_args()` found in [launch_utils.py](agimus_demos_common/launch_utils.py). Description of those arguments can be found below.

> [!WARNING]
> All ROS nodes launched within Agimus Demos are expected to configure themselves to expect parameter **use_sim_time**. There exist function `get_use_sim_time()` in utils that simplifies this process. Look at example below for usage.

## Launch files

**franka_common.launch.py** is meant to be included in demos where Franka Emika robots are used. Depending on configuration it launches Gazebo simulation or connects to the real robot. Aside from unifying include interface of Franka Emika robot between simulation and hardware it also provides options to launch RViz2 and configure verbosity of the simulation.

Launch arguments specific to this launch file:
- **franka_controllers_params**:

    Default: *agimus_demos_common/config/franka_controllers.yaml*

    Path to the yaml file use to define controller parameters.

- **rviz_config_path**:

    Default: *agimus_demos_common/rviz/franka_preview.rviz*

    Path to RViz configuration file

Launch arguments expected to be public interface of all demos using Franka robots:

- **robot_ip**:

    Default: *""*

    Hostname or IP address of the robot. If not empty launch file is configured for real robot. If empty **use_gazebo** is expected to be set to *true*.

- **arm_id**:

    Valid choices: [*fer*, *fr3*, *fp3*]

    Default: *fer*

    ID of the type of arm used. Supported values: fer, fr3, fp3.

- **use_gazebo**:

    Valid choices: [*true*, *false*]

    Default: *false*

    Configures launch file for Gazebo simulation. If set to *true* launch file is configured for simulated robot. If set to *false* argument **robot_ip** is expected not to be empty.

- **use_rviz**:

    Valid choices: [*true*, *false*]

    Default: *false*

    Visualize the robot in RViz.

- **gz_verbose**:

    Valid choices: [*true*, *false*]

    Default: *false*

    Whether to set verbosity level of Gazebo to 3.

- **gz_headless**:

    Valid choices: [*true*, *false*]

    Default: *false*

    Whether to launch Gazebo in headless mode (no GUI is launched, only physics server).

**franka_common_lfc.launch.py** extends **franka_common.launch.py** by launching Linear Feedback Controller and Joint State Estimator on top of it. This launch file in a sense is equivalent to *Demo 01 LFC Alone*.

Launch arguments are the same as in **franka_common.launch.py**, and are extended by:

- `linear_feedback_controller_params`: Path to the yaml file use to define Linear Feedback Controller\`s and Joint State Estimator\`s params.

    Default: *agimus_demos_common/config/linear_feedback_controller_params.yaml*


## Utils

This package provides utility functions to ease up creation of new launch files. Simple example launch file can look as follows:

```python
from agimus_demos_common.launch_utils import (
    generate_default_franka_args,
    generate_include_franka_launch,
    get_use_sim_time,
)


def launch_setup(
    context: LaunchContext, *args, **kwargs
) -> list[LaunchDescriptionEntity]:
    # Helper function that includes `franka_common_lfc.launch.py`.
    franka_robot_launch = generate_include_franka_launch("franka_common_lfc.launch.py")

    # Utility ROS node, delaying stat of other nodes until
    # robot's position was initialized in the simulation.
    # Has no impact on hardware.
    wait_for_non_zero_joints_node = Node(
        package="agimus_demos_common",
        executable="wait_for_non_zero_joints_node",
        # Set `use_sim_time` based on `use_gazebo` launch argument.
        parameters=[get_use_sim_time()],
        output="screen",
    )

    my_awesome_node = Node(
        package="my_awesome_package",
        executable="my_awesome_node",
        parameters=[get_use_sim_time()],
        output="screen",
    )

    return [
        franka_robot_launch,
        wait_for_non_zero_joints_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=wait_for_non_zero_joints_node,
                on_exit=[my_awesome_node],
            )
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        # Helper function creates launch arguments required by `franka_common.launch.py`.
        generate_default_franka_args()
        + [OpaqueFunction(function=launch_setup)]
    )
```

Function `generate_default_franka_args()` ensures all launch arguments used by `franka_common.launch.py` are exposed by launch file, while `generate_include_franka_launch()` includes that file and uses those declared parameters.

Function `generate_default_franka_args()` is directly used by `franka_common.launch.py`, so all arguments exposed by it are described in the documentation above.

Function `get_use_sim_time()` return dictionary with parameter **use_sim_time**, which value is set automatically whether simulation is launch or real robot is running.

## Nodes

### wait_for_non_zero_joints_node

Utility ROS node, mean to delay launch of other nodes until Gazebo simulation starts publishing non-zero joint states. It will subscribe to the topic and exit with exist code `0` when sum of absolute values of joint positions will be greater than set threshold. Otherwise if threshold is not exceeded and timeout si reached the node return exit code `1`, indicating error.

### Subscribers

- **/joint_states** [sensor_msgs/msg/JointState]

    Values of joint states of the robot.

### Parameters

- **timeout** [*double*]:

    Default: *10.0*

    Time to wait for joint positions to be above threshold.

- **timeout** [*double*]:

    Default: *1e-3*

    Threshold for sum of absolute values of joint positions used to determine success.
