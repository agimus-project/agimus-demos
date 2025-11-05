AGIMUS demo 07 deburring
-----------------------------

The purpose of this demo is to use the AGIMUS architecture to deburr user-selected holes in a pylone.

Expected behaviors:
- Launch Gepetto Viewer to see the model.
- Launch `bringup.launch.py` of the demo.
- Wait for a log *Waiting for goal message on topic '/target_handle'...* to start periodically show in the terminal.
- After publishing a message with a handle name corresponding to expected hole planner will attempt to find a corresponding path.
- If path is found robot will execute the motion till pregrasp and stop for a moment to change gains and smoothly change a task. (see `weights.schedule.interpolation_time` param for exact time of transition)
- The robot will insert the tool with a linear movement into the hole.
- Once insertion is done, robot will stop for a moment changing to force tracking task. In this phase it may move a bit.
- Next the robot will linearly increase force target, until reaching setpoint.
- Once force is set to a septoint value, deburring motion begins. Robot will perform a circular motion of the end effector, first increasing the radius, then maintaining it for a moment and then reducing it back to zero.
- Once the motion is done robot will lower the forces linearly and smoothly change gains back to linear TCP motion.
- Next it will move the end effector to pregrasp configuration and wait for new command.
- Log *Waiting for goal message on topic '/target_handle'...* indicated entire motion was executed correctly.

> [!CAUTION]
> Sending new goal when previous trajectory is executed will cause flushing of the trajectory buffer. This will stop the robot, and computation of the new trajectory will start from a slightly different configuration than current configuration of the robot. This may cause the robot to perform an abrupt motion when new trajectory starts being executed. In extreme cases this may cause the robot to become unstable!

## Install dependencies and build.

Follow the instructions from the main [README.md](../README.md) file.

## Start the demo in simulation using the Panda robot.
```bash
# Terminal 1
ros2 launch agimus_demo_07_deburring visualization.launch.py
# Terminal 2
ros2 launch agimus_demo_07_deburring bringup.launch.py use_gazebo:=true
```

## Start the demo on hardware using the Panda robot.
```bash
# Terminal 1
ros2 launch agimus_demo_07_deburring visualization.launch.py
# Terminal 2
ros2 launch agimus_demo_07_deburring bringup.launch.py \
    robot_ip:=<fci-ip> \
    ft_sensor_ip:=<ip of the FT sensor> \
    disable_collision_safety:=true \
    use_rviz:=true \
    use_plotjuggler:=true \
    use_precomputed_trajectories:=true
```

## Commanding the robot

In another terminal session run:
```bash
ros2 topic pub /target_handle std_msgs/msg/String "data: '<handle_name>'" --once
```
Replace `handle_name` with a name of the handle you want to reach. Available handles can be found in a folder [hpp_handle_configs.yaml](config/hpp_handle_configs.yaml). In case robot correctly finds a path, a red path corresponding to robot's TCP will be displayed in RViz 2. In case error occurred corresponding error message will be displayed.

Optionally there is a script with a default set of holes. To run it execute:
```bash
ros2 run agimus_demo_07_deburring sequencer_node
```
