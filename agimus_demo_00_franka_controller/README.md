AGIMUS demo 00 franka controller
--------------------------------

The purpose of this demo is to simply check whether your installation is done correctly.
Expected result: after starting the demo, a gazebo and a rviz windows should be appearing with the panda in it.

![expected result](https://github.com/agimus-project/agimus-demos/blob/topic/mnaveau/demo00_readme/agimus_demo_00_franka_controller/doc/images/demo_00_result.png)

## Install dependencies and build.

```bash
cd ~/ros2_ws
vcs import src < src/agimus-demos/agimus_demo_00_franka_controller/dependencies.repos
colcon build --symlink-install
source install/setup.bash
```

## Start the demo in simulation using the Panda robot.
```bash
ros2 launch agimus_demo_00_franka_controller bringup_gz_simulation.launch.py
```
