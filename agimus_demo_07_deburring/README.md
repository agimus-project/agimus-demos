AGIMUS demo 07 deburring
-----------------------------

The purpose of this demo is to use the AGIMUS architecture to deburr user-selected holes in a pylones.
Expected behaviors:
    - TODO: will be added in next

## Install dependencies and build.

Follow the instructions from the main [README.md](../README.md) file.

## Start the demo in simulation using the Panda robot.
```bash
ros2 launch agimus_demo_07_deburring bringup.launch.py use_gazebo:=true
```

## Start the demo on hardware using the Panda robot.
```bash
ros2 launch agimus_demo_07_deburring bringup.launch.py robot_ip:=<fci-ip> ft_sensor_ip:=<ip of the FT sensor>
```
