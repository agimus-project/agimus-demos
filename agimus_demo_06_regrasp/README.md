AGIMUS demo 05 pick and place
-----------------------------

The purpose of this demo is to use the AGIMUS architecture to regrasp an object.
In more detail, to pick an object, place it on a different side, pick the rotated object and place it to final pose.
Currently the sequence is predetermined.
Expected behaviors:
    - The robot grasps an object
    - The robot places an object with 90 degrees rotation
    - The robot grasps an object again
    - The robot places an object with 90 degrees rotation again

The initial experiment is restricted to object 23 and rotation always in positive direction along the x-axis

## Install dependencies and build.

```bash
vcs import src < src/agimus-demos/agimus_demo_06_regrasp/dependencies.repos
rosdep update --rosdistro $ROS_DISTRO
rosdep install -y -i --from-paths src --rosdistro $ROS_DISTRO --skip-keys libfranka
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug --symlink-install
source install/setup.bash
```

## Start the demo in simulation using the Panda robot.
```bash
cd workspace
reset && source install/setup.bash
ros2 launch agimus_demo_06_regrasp bringup.launch.py use_gazebo:=true
```
After the xterm terminal is opened, type there `o.regrasp('obj_23')`.

## Start the demo on hardware using the Panda robot.
```bash
ros2 launch agimus_demo_06_regrasp bringup_hw.launch.py robot_ip:=<fci-ip>
ros2 launch agimus_demo_06_regrasp bringup.launch.py robot_ip:=192.168.102.11 aux_computer_ip:=192.168.102.21 aux_computer_user:=ros
ros2 bag record /camera/color/image_raw /camera/color/camera_info /camera/depth/color/points /camera/depth/camera_info /camera2/color/image_raw /camera2/color/camera_info /happypose/detections /joint_states /mpc_debug /mpc_input /control
```

```
python scripts/image_saver
ros2 bag play rosbag2_2025...

ffmpeg -framerate 30 -i output_images/frame_%05d.png -c:v libx264 -pix_fmt yuv420p dataset_video2_21_26_20_long.mp4
ffmpeg -ss 29 -i dataset_video2_21_26_20_long.mp4 -c copy dataset_video2_21_26_20.mp4
```
