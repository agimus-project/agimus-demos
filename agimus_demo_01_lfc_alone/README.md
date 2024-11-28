AGIMUS demo 01 lfc alone
--------------------------------

The purpose of this demo is to run the [linear_feedback_controller](https://github.com/loco-3d/linear-feedback-controller) (lfc) in simulation and on the real robot. 
Expected behavior: robot not moving, lfc is implementing a default PD+ controller using the gains `agimus_demo_01_lfc_alone/linear_feedback_controller.yaml`.

## Install dependencies and build.

```bash
vcs import src < src/agimus-demos/agimus_demo_01_lfc_alone/dependencies.repos
colcon build --symlink-install
source install/setup.bash
```

## Start the demo in simulation using the Panda robot.
```bash
ros2 launch agimus_demo_01_lfc_alone bringup.launch.py
```