AGIMUS demo 04 visual servoing
-----------------------------

This demo shows the visual servoing capabilities of the AGIMUS architecture.
Expected behaviors (intention only):
    - The robot first sees an object and tries to keep the relative pose of object wrt camera constant.
    - The robot avoids obstacles.

---

### Dependencies

This demo requires source built of dependencies found in:
- [franka.repos](../franka.repos)
- [control.repos](../control.repos)
- [agimus_dev.repos](../agimus_dev.repos)


install following packages
```bash
sudo apt install ros-humble-image-proc
sudo apt install ros-humble-apriltag-msgs
```
you'll also need dependencies related to apriltag that can be found in [vision.repos](../vision.repos)

### Apriltag setup
the apriltag used can be found  [here](https://triagechallenge.darpa.mil/docs/AprilTag_0-20_family36h11.pdf),
the apriltag side size used was 44 mm, but can be modified [here](https://github.com/TheoMF/olt_ros2_pipeline/blob/topic/tmartinez/apriltag/config/tags_36h11.yaml#L22)


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

for the vision side you'll have to launch
```bash
ros2 launch olt_ros2_pipeline vision_bringup.launch.py
```

for the control side you'll have to launch
```bash
ros2 launch agimus_demo_05_pick_and_place bringup.launch.py arm_id:=fer robot_ip:=<fci-ip> use_ft_sensor:=false
```
