AGIMUS demo 03 mpc dummy traj for TIAGo Pro
-------------------------------------------

The purpose of this demo is to run the [linear_feedback_controller](https://github.com/loco-3d/linear-feedback-controller) (LFC) with [agimus_controller](https://github.com/agimus-project/agimus_controller) (MPC) in simulation and on the real robot.
The LFC input is coming from agimus_controller which is following a trajectory published by the node `simple_trajectory_publisher`. The controller is more specifically a Whole Body Model Predictive Controller and the trajectory is a small sinus signal at the joint level.
Expected behavior: robot's joints are oscillating gently.

The parameters for the motion is split in two location [agimus_demo_03_mpc_dummy_traj_tiago_pro/config/agimus_controller.yaml](config/agimus_controller.yaml) and [agimus_demo_03_mpc_dummy_traj_tiago_pro/config/ocp_definition_file.yaml](config/ocp_definition_file.yaml) .

### Dependencies

This demo requires source built of dependencies found in:
- [control.repos](../control.repos)

### Simulation

> [!NOTE]
> Gazebo simulation of tiago-pro robots require very high frequency of the simulated environment, hence users might experience high CPU utilization or even errors in cases where older and less powerful computers are used.

To launch the demo run:

```bash
ros2 launch agimus_demo_03_mpc_dummy_traj_tiago_pro bringup.launch.py use_gazebo:=true use_rviz:=true
```

Expected result: after starting the demo, a Ignition Gazebo and a RViz 2
windows should be appearing with the Tiago-Pro robot initializing itself.
Then after the robot is done the demo starts and the robot's left arm
joints oscillate gently.

### Real robot

First turn on the robot and follow the initialization procedure.
Move the robot a safe position allowing for a full range of joint motion
while avoiding collisions with environment and not posing any threat to
safety of people around.

> [!CAUTION]
> Before starting the launch file make sure robot is in a safe position and has sufficient movement space for it's joints and is not likely to collide with anything. Ensure all spectators are in a safe distance from the machine, and **the operator can quickly reach the Emergency Button in case error occurs**!

> [!NOTE]
> Robot will start oscillating around it's starting point. When restarting the demo make sure robot was stopped with sufficient joint motion left, as during a re-run it might trigger joint limit safety!

### Run on the robot

#### Install the linear-feedback-controller

- Get an alum docker which is a mirror of the robot OS.
- Create a workspace with the following 2 packages:
  - https://github.com/loco-3d/linear-feedback-controller
  - https://github.com/loco-3d/linear-feedback-controller-msgs
- Use the pal deploy tooling to install this workspace onto the desired robot.
  See the official PAL documentation.

Further down the line this step will be useless as the linear-feedback-controller packages will be part of the debian packages available for install. Like `apt install pal-alum-linear-feedback-controller`.

#### Setup the environment

- In the docker install cyclonedds and mcap plugin for rosbags.
  ```bash
  sudo apt update
  sudo apt install ros-humble-cyclonedds
  sudo apt install ros-humble-rmw-cyclonedds-cpp
  sudo apt install ros-humble-rosbag2-storage-mcap
  ```
- Install the tiago-pro packages:
  ```bash
  cd ros2_ws/src
  git clone git@github.com/agimus-project/agimus-demos
  git clone git@github.com/agimus-project/agimus_controller
  # tmp branches are:
  # git clone git@github.com/MaximilienNaveau/agimus-demos -b feature/mnaveau/demo3-tiago-square
  # git clone git@github.com/MaximilienNaveau/agimus_controller -b bugfix/mnaveau/resource_retriver_and_bag_reading
  vcs-import agimus-demos/tiago_pro.repos
  ```
- Change the network interface in the
  `agimus-demos/agimus_demos_common/config/tiago_pro/cyclone_config.xml` with your own.
- copy the lfc + jse config in `/tmp` in the docker and on the robot
  ```bash
  cp agimus-demos/agimus_demos_common/config/tiago_pro/* /tmp/
  scp agimus-demos/agimus_demos_common/config/tiago_pro/* pal@tiago-pro:/tmp/
  ```
- Set the cyclone DDS config in bashrc in the docker.
  The file got copied in the previous step in
  `/tmp/cyclone_config.xml`.
  ```bash
  echo "" >> ~/.bashrc
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
  echo "export CYCLONEDDS_URI=/tmp/cyclone_config.xml" >> ~/.bashrc
  echo "export ROS_DOMAIN_ID=2 # for TIAGo-Pro" >> ~/.bashrc
  ```
- On the robot remove the laser from the urdf: `pal robot_info set laser_model no-laser`
- Lastly setup and build all
  ```bash
  cd ~/ros2_ws
  ./setup.sh
  ./build.sh
  ```

#### Set the behavior of the tracking.

To set the parameters of the trajectory publisher one need to edit
`agimus-demos/demo_03_mpc_dummy_traj_tiago_pro/config/trajectory_weights_params.yaml `

By default the cartesian control is setup.

- for a cartesian control:
  ```yaml
  simple_trajectory_publisher:
    ros__parameters:
      ee_frame_name: arm_right_7_joint
      trajectory_name: generic_trajectory
      w_q: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
      w_qdot: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
      w_qddot: [0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001]
      w_robot_effort: [0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
      w_pose: [60.0, 60.0, 60.0, 10.0, 10.0, 10.0]
      mask: [true, true, true, true, true, true]
  ```

- for a joint space control:
  ```yaml
  simple_trajectory_publisher:
    ros__parameters:
      ee_frame_name: arm_right_7_joint
      trajectory_name: generic_trajectory
      w_q: [10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0]
      w_qdot: [0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
      w_qddot: [0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001, 0.000001]
      w_robot_effort: [0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001]
      w_pose: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
      mask: [false, false, false, false, false, false]
  ```

#### Start the demo on the robot.

Restart the controller on the robot.
```bash
pal module_manager restart
```
Start the demo (with `ssh -X` in remote because terminal window will appear to start the different controllers.)
- Terminal 1
```bash
hppcorbaserver
```
- Terminal 2
```bash
gepetto-gui
```
- Terminal 3
```bash
ros2 launch agimus_demo_03_mpc_dummy_traj_tiago_pro bringup.launch.py use_gazebo:=false use_rviz:=true
```

This procedure spawns many windows:
- A rviz windows with the robot moving.
- A plot window showing the behavior of the solver inside the agimus_controller.
- A xterm that asks for pressing enter to start the controller.
- A xterm that display a ipython terminal.

Once the user is ready with an emergency stop at hand. The user can press enter
in the xterm that is asking for it and the robot will switch from position control
to torque control.

The degrees of freedom controlled (here the right arm by default (14/01/2026))
will perform an impedance control with low gains. The user can move the joints
and they acts like springs.

On the ipython terminal:
```
o.continuously_send_trajectory()
```
This will start the AGIMUS architecture. HPP will plan a square for the right hand,
the plan is send to the agimus_controller. In turn the MPC in the agimus_controller
will stream the control to the linear-feedback-controller running on the robot.

The result is the tiago robot drawing a square with it's right hand. One can
play with the redundancy if it's setup using the cartesian control in the `simple_trajectory_publisher`.
