AGIMUS demo 02 simple pd plus
--------------------------------

The purpose of this demo is to run the [linear_feedback_controller](https://github.com/loco-3d/linear-feedback-controller) (LFC) in simulation and on the real robot.
The LFC input is coming from a python test node called `pd_plus_controller`. It fills the control object to design a PD+ controller tracking a small sinus signal at the joint level.
Expected behavior: robot's joints are oscillating gently.

The parameters for the motion is located in [agimus_demo_02_simple_pd_plus/config/pd_plus_controller.yaml](config/pd_plus_controller.yaml).

### Dependencies

This demo requires source built of dependencies found in:
- [franka.repos](../franka.repos)
- [control.repos](../control.repos)
- [agimus_dev.repos](../agimus_dev.repos)

### Robot model selection

The demo support two different Franka robot models, selectable via the `arm_id` parameter:

| arm_id | Models |
|--------|-------------------------------|
| fer | Franka Emika Panda |
| fr3 | Franka Research 3 |

If not specified, the parameter `arm_id` is set to fer.

### Simulation

> [!NOTE]
> Gazebo simulation of Franka robots require very high frequency of the simulated environment, hence users might experience high CPU utilization or even errors in cases where older and less powerful computers are used.

To launch the demo run:

```bash
ros2 launch agimus_demo_02_simple_pd_plus bringup.launch.py use_gazebo:=true use_rviz:=true
```
Expected result: after starting the demo, a Ignition Gazebo and a RViz 2 windows should be appearing with the Franka robot's joints oscillating gently.

### Real robot

First turn on the robot and unlock joint in the web-ui. Move the robot a safe position allowing for a full range of joint motion while avoiding collisions with environment and not posing any threat to safety of people around.

> [!CAUTION]
> Before starting the launch file make sure robot is in a safe position and has sufficient movement space for it's joints and is not likely to collide with anything. Ensure all spectators are in a safe distance from the machine, and **the operator can quickly reach the Emergency Button in case error occurs**!

> [!NOTE]
> Robot will start oscillating around it's starting point. When restarting the demo make sure robot was stopped with sufficient joint motion left, as during a re-run it might trigger joint limit safety!

> [!NOTE]
> The libfranka version differs depending on the robot model. Make sure you use the correct control container to operate the real robot.

Launch the demo

```bash
ros2 launch agimus_demo_02_simple_pd_plus bringup.launch.py robot_ip:=<robot-ip> use_rviz:=true
```
Expected result: after starting the demo, a RViz 2 window should be appearing with the Franka robot. The robot's joints oscillating gently.
