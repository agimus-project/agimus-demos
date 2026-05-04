AGIMUS demo 02 simple pd plus for TIAGo Pro
-------------------------------------------

The purpose of this demo is to run the [linear_feedback_controller](https://github.com/loco-3d/linear-feedback-controller) (LFC) in simulation and on the real robot.
The LFC input is coming from a python test node called `pd_plus_controller`. It fills the control object to design a PD+ controller tracking a small sinus signal at the joint level.
Expected behavior: robot's joints are oscillating gently.

The parameters for the motion is located in [agimus_demo_02_simple_pd_plus_tiago_pro/config/pd_plus_controller.yaml](config/pd_plus_controller.yaml).

### Dependencies

This demo requires source built of dependencies found in:
- [control.repos](../control.repos)

### Simulation

> [!NOTE]
> Gazebo simulation of tiago-pro robots require very high frequency of the simulated environment, hence users might experience high CPU utilization or even errors in cases where older and less powerful computers are used.

Make sure `ROS_DOMAIN_ID` is set to avoid interference with other ROS 2 nodes on the network:
```bash
export ROS_DOMAIN_ID=<your_id>
```

To launch the demo run:

```bash
ros2 launch agimus_demo_02_simple_pd_plus_tiago_pro bringup.launch.py use_gazebo:=true use_rviz:=true
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

#### Install the linear-feedback-controller

- Get an alum docker which is a mirror of the robot OS.
- run sudo apt install pal-alum-linear-feedback-controller
- Use the pal deploy tooling to install this workspace onto the desired robot.
  See the official PAL documentation.

Further down the line this step will be useless as the linear-feedback-controller packages will be part of the debian packages available for install. Like `apt install pal-alum-linear-feedback-controller`.

#### Setup the environment

- Install the tiago-pro packages:
  ```bash
  cd ros2_ws/src
  git clone git@github.com/agimus-project/agimus-demos
  vcs-import agimus-demos/tiago_pro.repos
  ```
- Change the network interface in the
  `agimus-demos/agimus_demos_common/config/tiago_pro/cyclone_config.xml` with your own.
- Copy the lfc + jse config in `/tmp` in the docker and on the robot:
  ```bash
  cp agimus-demos/agimus_demos_common/config/tiago_pro/* /tmp/
  scp agimus-demos/agimus_demos_common/config/tiago_pro/* pal@tiago-pro:/tmp/
  ```
- Set the cyclone DDS config in bashrc in the docker.
  The file got copied in the previous step in `/tmp/cyclone_config.xml`.
  ```bash
  echo "" >> ~/.bashrc
  echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
  echo "export CYCLONEDDS_URI=/tmp/cyclone_config.xml" >> ~/.bashrc
  echo "export ROS_DOMAIN_ID=2 # for TIAGo-Pro" >> ~/.bashrc
  ```
- On the robot remove the laser from the urdf: `pal robot_info set laser_model no-laser`
- Lastly setup and build all:
  ```bash
  cd ~/ros2_ws
  ./setup.sh
  ./build.sh
  ```

#### Start the demo on the robot

Restart the controller on the robot:
```bash
pal module_manager restart
```

Start the demo (with `ssh -X` in remote because a terminal window will appear to start the controllers):
```bash
ros2 launch agimus_demo_02_simple_pd_plus_tiago_pro bringup.launch.py robot_ip:=<robot-ip> use_rviz:=true
```

This procedure spawns:
- A RViz window with the robot.
- A xterm that asks for pressing enter to switch to torque control.

Once the user is ready with an emergency stop at hand, press enter in the xterm.
The robot will switch from position control to torque control and its joints will oscillate gently.
