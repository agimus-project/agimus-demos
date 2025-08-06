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

Launch the demo

```bash
ros2 launch agimus_demo_03_mpc_dummy_traj_tiago_pro bringup.launch.py robot_ip:=<robot-ip> use_rviz:=true
```
Expected result: after starting the demo, a RViz 2 window should be appearing with Tiago robot. The robot's joints oscillating gently.
