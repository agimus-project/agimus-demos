AGIMUS demo 03 mpc dummy traj
--------------------------------


> [!CAUTION]
> This demo is **CPU intensive** and strongly rely on short computation time. Even as simple motion as sinusid can become unstable if the optimal control problem does not have time to converge! Make sure you **run this demo on a modest computer** or unstable control applied to the robot can cause **injuries**!

The purpose of this demo is to use the [agimus_controller](https://github.com/agimus-project/agimus_controller) to send control messages to the [linear_feedback_controller](https://github.com/loco-3d/linear-feedback-controller) (LFC) in simulation and on the real robot.
The agimus_controller follows a trajectory that is a small sinus signal at the joint level, the trajectory is given by a dummy publisher node.
Expected behavior: robot's joints are oscillating gently.

### Dependencies

This demo requires source built of dependencies found in:
- [franka.repos](../franka.repos)
- [control.repos](../control.repos)
- [agimus_dev.repos](../agimus_dev.repos)

### Simulation

> [!NOTE]
> Gazebo simulation of Franka robots require very high frequency of the simulated environment, hence users might experience high CPU utilization or even errors in cases where older and less powerful computers are used.

To launch the demo run:

```bash
ros2 launch agimus_demo_03_mpc_dummy_traj bringup.launch.py use_gazebo:=true use_rviz:=true
```
Expected result: after starting the demo, a Ignition Gazebo and a RViz 2 windows should be appearing with the Franka robot's joints oscillating gently.

### Real robot

First turn on the robot and unlock joint in the web-ui. Move the robot a safe position allowing for a full range of joint motion while avoiding collisions with environment and not posing any threat to safety of people around.

> [!CAUTION]
> Before starting the launch file make sure robot is in a safe position and has sufficient movement space for it's joints and is not likely to collide with anything. Ensure all spectators are in a safe distance from the machine, and **the operator can quickly reach the Emergency Button in case error occurs**!

> [!NOTE]
> Robot will start oscillating around starting point. When restarting the demo make sure robot was stopped with sufficient joint motion left, as during a re-run it might trigger joint limit safety!

Launch the demo

```bash
ros2 launch agimus_demo_03_mpc_dummy_traj bringup.launch.py robot_ip:=<robot-ip> use_rviz:=true
```
Expected result: after starting the demo, a RViz 2 window should be appearing with the Franka robot. The robot's joints oscillating gently.
