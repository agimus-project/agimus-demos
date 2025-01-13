AGIMUS demo 03 mpc dummy traj
--------------------------------

The purpose of this demo is to use the [agimus_controller](https://github.com/agimus-project/agimus_controller) to send control messages to the [linear_feedback_controller](https://github.com/loco-3d/linear-feedback-controller) (LFC) in simulation and on the real robot.
The agimus_controller follows a trajectory that is a small sinus signal at the joint level, the trajectory is given by a dummy publisher node.
Expected behavior: robot's joints are oscillating gently.

## Install dependencies and build.

```bash
vcs import src < src/agimus-demos/agimus_demo_03_mpc_dummy_traj/dependencies.repos
rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i --from-paths src --rosdistro $ROS_DISTRO --skip-keys libfranka
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install
source install/setup.bash
```

## Start the demo in simulation using the Panda robot.
```bash
cd workspace
reset && source install/setup.bash && ros2 launch agimus_demo_03_mpc_dummy_traj bringup.launch.py
```

## Start the demo on hardware using the Panda robot.
```bash
ros2 launch agimus_demo_03_mpc_dummy_traj bringup_hw.launch.py arm_id:=fer robot_ip:=<fci-ip>
```
