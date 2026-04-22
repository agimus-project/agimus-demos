AGIMUS demo 09 glue spreading
--------------------------------
This demo runs a MPC on a Franka Research 3 arm with the end goal of spreading glue on a rectangular panel.


## Dependencies and installation:
Clone and install `aligator_mpc` in your /src folder:
```bash
cd ros2_ws/src
git clone git@github.com:LouiseMsn/aligator_mpc.git -b ros2_wrapper
cd aligator_mpc
pip install -e .
```
Build and install `agimus-demos`:
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Starting the demo
### In simulation:
```bash
ros2 launch agimus_demo_09_glue_spreading bringup.launch.py use_gazebo:=true use_rviz:=true arm_id:='fr3'
```

### On a real robot:
```bash
ros2 launch agimus_demo_09_glue_spreading bringup.launch.py robot_ip:=<YourRobotIP> use_rviz:=true arm_id:='fr3'
```
## Launching the demo:
Either use the Service call plugin of RQT to call the `/aligato_mpc/launch_mpc` service or run:
```bash
ros2 service call /aligator_mpc/launch_mpc std_srvs/srv/Trigger
```
