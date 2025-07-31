AGIMUS demo 06 regrasp
-----------------------------

The purpose of this demo is to use the AGIMUS architecture to regrasp an object.
In more detail, to pick an object, place it on a different side, pick the rotated object and place it to final pose.
The initial experiment is restricted to object 23 and rotation always in positive direction along the x-axis (hardcoded).
Expected behaviors:
    - The robot grasps an object
    - The robot places an object with 90 degrees rotation
    - The robot grasps an object again
    - The robot places an object with 90 degrees rotation again

Please carefully read notes after Terminal 3 for real robot experiment

## Install dependencies and build.

```bash
vcs import src < src/agimus-demos/agimus_demo_06_regrasp/dependencies.repos
rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i --from-paths src --rosdistro $ROS_DISTRO --skip-keys libfranka
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install
source install/setup.bash
```

## Start the demo in simulation using the Panda robot (three terminals needed).
Terminal 1
```bash
hppcorbaserver
```

Terminal 2
```bash
gepetto-gui
```
Before running the next command, update the params in `orchestrator.py`!
`use_simulation` should be `True`, and probably you want to use some default values for object poses, so `use_hardcoded_poses` should be `True`.

Terminal 3
```bash
source install/setup.bash
ros2 launch agimus_demo_06_regrasp bringup.launch.py use_gazebo:=true
```
After the xterm terminal is opened, type there `o.regrasp('obj_23')`.
You will be suggested to investigate the initial pose `q_init`.
For this go to Gepetto GUI, click `Reset connections`, then `Refresh`, then `Fetch configuration`, then `Zoom to fit` and inspect the initial configuration coming from happypose. Does it look reasonable? If yes - return to xterm and press Enter.
Now you will be offered to investigate projection to HPP contact surface. Click `Fetch configuration` in Gepetto GUI, new config should be quite close, but more horizontal. If it is too far away - try to update the `foam_block_contact` contact surface definition in `demo.srdf`.
Then repeat the same for goal configuration and it will start to look for a path.

Once the solution is found, go back to Gepetto GUI and open `Window` -> `Path PLayer`. Refresh and investigate the path.
If it looks reasonable, you can run it step-by-step on the robot by pressing Enter in xterm.
You'll see messages that ask you to confirm for each portion of the path.
This is done to have more control over the time given for closing/opening the gripper.

## Start the demo on hardware using the Panda robot (three terminals needed).
Terminal 1
```bash
hppcorbaserver
```

Terminal 2
```bash
gepetto-gui
```

Before running the next command, update the params in `orchestrator.py`!
`use_simulation` should be `False`, if you want to use happypose position, `use_hardcoded_poses` should be `False`.
`use_smoothing_at_waypoints` will prevent robot to stop at every intermediate waypoint along the path, so will make the path smoother.

Terminal 3
```bash
source install/setup.bash
(option 1) ros2 launch agimus_demo_06_regrasp bringup_hw.launch.py robot_ip:=<fci-ip>
(option 2) ros2 launch agimus_demo_06_regrasp bringup.launch.py robot_ip:=192.168.102.11 aux_computer_ip:=192.168.102.21 aux_computer_user:=ros
(record bag dor debugging) ros2 bag record /camera/color/image_raw /camera/color/camera_info /camera/depth/color/points /camera/depth/camera_info /camera2/color/image_raw /camera2/color/camera_info /happypose/detections /joint_states /mpc_debug /mpc_input /control
```

To generate video from saved camera images
```
python scripts/image_saver
ros2 bag play rosbag2_2025...

ffmpeg -framerate 30 -i output_images/frame_%05d.png -c:v libx264 -pix_fmt yuv420p dataset_video2_21_26_20_long.mp4
ffmpeg -ss 29 -i dataset_video2_21_26_20_long.mp4 -c copy dataset_video2_21_26_20.mp4
```
