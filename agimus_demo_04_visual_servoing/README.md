AGIMUS demo 04 visual servoing
-----------------------------

This demo shows the visual servoing capabilities of the AGIMUS architecture.
Expected behaviors (intention only):
    - The robot first sees an object and tries to keep the relative pose of object wrt camera constant.
    - The robot avoids obstacles.

---

## Install dependencies and build.
**This section may not be up to date.**

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
reset && source install/setup.bash && ros2 launch agimus_demo_04_visual_servoing bringup.launch.py use_gazebo:=true gz_headless:=true use_rviz:=true
```
Then, in another terminal, you can use `static_transform_publisher` to update the reference used by the visual servoing task.
You can do so by iteratively starting and killing the following command, with varying `--[xyz]` options.
```
ros2 run tf2_ros static_transform_publisher --frame-id fer_link0 --child-frame-id current_object
# Then kill and, e.g.
ros2 run tf2_ros static_transform_publisher --frame-id fer_link0 --child-frame-id current_object --x 0.1
```

In RViz, you can visualize frames:
- current_object: the frame wrt which the vision reference is sent.
- camera_color_optical_frame: the current pose of the frame
- camera_color_optical_frame_reference: the desired pose of camera_color_optical_frame

## Start the demo on hardware using the Panda robot.
**This section may not be up to date.**
```bash
ros2 launch agimus_demo_05_pick_and_place bringup_hw.launch.py arm_id:=fer robot_ip:=<fci-ip>
```
