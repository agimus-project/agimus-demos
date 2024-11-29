AGIMUS demo 01 lfc alone
--------------------------------

The purpose of this demo is to run the [linear_feedback_controller](https://github.com/loco-3d/linear-feedback-controller) (LFC) in simulation and on the real robot.
The LFC input is comming from a python test node called `pd_plus_controller`. It fills the control object to design a PD+ controller tracking a small sinus signal at the joint level.
Expected behavior: robot's joints are oscillating gently.
The parameters for the motion is located in [agimus_demo_02_simple_pd_plus/config/pd_plus_controller.yaml](config/pd_plus_controller.yaml).

## Install dependencies and build.

```bash
vcs import src < src/agimus-demos/agimus_demo_02_simple_pd_plus/dependencies.repos
rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i --from-paths src --rosdistro $ROS_DISTRO --skip-keys libfranka
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install
source install/setup.bash
```

## Start the demo in simulation using the Panda robot.
```bash
cd workspace
reset && source install/setup.bash && ros2 launch agimus_demo_02_simple_pd_plus bringup.launch.py
```
