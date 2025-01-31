AGIMUS demo 01 lfc alone
--------------------------------

The purpose of this demo is to run the [linear_feedback_controller](https://github.com/loco-3d/linear-feedback-controller) (LFC) in simulation and on the real robot.
Expected behavior: robot not moving, LFC is implementing a default PD+ controller using the gains [agimus_demo_01_lfc_alone/linear_feedback_controller.yaml](config/linear_feedback_controller.yaml).

## Install dependencies and build.

```bash
vcs import src < src/agimus-demos/agimus_demo_01_lfc_alone/dependencies.repos
rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i \
    --from-paths src \
    --rosdistro $ROS_DISTRO \
    --skip-keys libfranka \
    --skip-keys eigenpy \
    --skip-keys hpp-fcl \
    --skip-keys coal \
    --skip-keys pinocchio \
    --skip-keys crocoddyl \
    --skip-keys mim_solvers \
    --skip-keys colmpc \
    --skip-keys hpp-util \
    --skip-keys hpp-template \
    --skip-keys hpp-statistics \
    --skip-keys hpp-environments \
    --skip-keys hpp-pinocchio \
    --skip-keys hpp-constraints \
    --skip-keys proxsuite \
    --skip-keys hpp-core \
    --skip-keys hpp-corbaserver \
    --skip-keys hpp-manipulation \
    --skip-keys hpp-manipulation-corba \
    --skip-keys gepetto-viewer \
    --skip-keys gepetto-viewer \
    --skip-keys hpp-gepetto \
    --skip-keys hpp-plot \
    --skip-keys hpp-gui \
    --skip-keys hpp-bin-picking \
    --skip-keys rospy \
    --skip-keys rostest \
    --skip-keys catkin \
    --skip-keys mim_robots \
    --skip-keys jrl-cmakemodules
colcon build --symlink-install
source install/setup.bash
```

## Start the demo in simulation using the Panda robot.
```bash
cd ros2_ws
reset && source install/setup.bash && ros2 launch agimus_demo_01_lfc_alone bringup.launch.py
```


## Start the demo on hardware using the Panda robot.
```bash
ros2 launch agimus_demo_01_lfc_alone bringup_hw.launch.py arm_id:=fer robot_ip:=<fci-ip>
```
