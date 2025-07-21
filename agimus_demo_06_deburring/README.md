AGIMUS demo 05 pick and place
-----------------------------

The purpose of this demo is to use the AGIMUS architecture to unbox some objects.
Expected behaviors:
    - TODO

## Install dependencies and build.

```bash
vcs import src < src/agimus-demos/agimus_demo_05_pick_and_place/dependencies.repos
rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i --from-paths src --rosdistro $ROS_DISTRO --skip-keys libfranka
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install
source install/setup.bash
```

## Start the demo in simulation using the Panda robot.
```bash
cd workspace
reset && source install/setup.bash && ros2 launch agimus_demo_06_deburring bringup.launch.py use_gazebo:=true
```

## Start the demo on hardware using the Panda robot.
```bash
ros2 launch agimus_demo_05_pick_and_place bringup.launch.py robot_ip:=<fci-ip> ft_sensor_ip:=<ip of the FT sensor>
```
