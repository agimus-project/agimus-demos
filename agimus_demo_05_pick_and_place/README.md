AGIMUS demo 05 pick and place
-----------------------------

The purpose of this demo is to use the AGIMUS architecture to unbox some objects from one box to another with visual servoing and collision avoidance.
Expected behaviors:
- The robot goes over a box of object.
- The robot start approaching one object.
- The robot goes to grasp an object.
- The robot move the object above a desired location.
- The robot drops the object.

### Dependencies

This demo requires source built of dependencies found in:
- [franka.repos](../franka.repos)
- [control.repos](../control.repos)
- [agimus_dev.repos](../agimus_dev.repos)


for the vision you'll also need to install the package [olt_ros2_pipeline](https://github.com/agimus-project/olt_ros2_pipeline?tab=readme-ov-file#launch-apriltag) from source.

### Visual Servoing
The visual servoing can be done either using apriltags or happypose, the parameter vision_type allows to choose between them and whether it's in simulation or not. In experiment, you'll also have to launch the vision by using olt_ros2_pipeline package.

### Apriltag setup
the apriltag used can be found  [here](https://triagechallenge.darpa.mil/docs/AprilTag_0-20_family36h11.pdf),
the apriltag side size used was 44 mm, but can be modified [here](https://github.com/agimus-project/olt_ros2_pipeline/blob/topic/tmartinez/apriltag/config/tags_36h11.yaml#L22)
We used a maizena box with dimensions 0.11 * 0.048 * 0.166 (m³) for this demo when using apriltag.
In the code the object is introduced as the tless_obj-31 to add it more easily in the architecture.

### Available tless objects for happypose
Five tless objects can be used with happypose in this demo their tless numbers are 21, 22, 23, 25, 26.

## Start the demo in simulation using the Panda robot.
```bash
cd workspace
reset && source install/setup.bash && ros2 launch agimus_demo_05_pick_and_place bringup.launch.py vision_type:=simulate_apriltag_det use_gazebo:=true gz_headless:=true use_rviz:=true
```
After the xterm terminal is opened, type there `o.pick_and_place('obj_<tless object number>')`.

## Start the demo on hardware using the Panda robot.
To place the source and destination box as it is in hpp, you may have first to calibrate the position of the two boxes, for that you can use in the xterm terminal `o.calibrate`.
To start the demo :
```bash
ros2 launch agimus_demo_05_pick_and_place bringup_hw.launch.py robot_ip:=<fci-ip>
ros2 launch agimus_demo_05_pick_and_place bringup.launch.py robot_ip:=192.168.102.11 aux_computer_ip:=192.168.102.21 aux_computer_user:=ros
ros2 bag record /camera/color/image_raw /camera/color/camera_info /camera/depth/color/points /camera/depth/camera_info /camera2/color/image_raw /camera2/color/camera_info /happypose/detections /joint_states /mpc_debug /mpc_input /control
```
```
ros2 launch agimus_demo_05_pick_and_place bringup.launch.py arm_id:=fer vision_type:=apriltag_det robot_ip:=<fci-ip> use_ft_sensor:=false
```

GraspIt installation
```bash
sudo apt-get install libsoqt-dev
git clone https://github.com/jmirabel/graspit
git clone https://github.com/jmirabel/graspit_script
cd graspit
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/home/gepetto/graspit_install  -DGRASPIT_DATA_DIR=/home/gepetto/graspit_install/.graspit
make -j16
make install
cd ../../graspit_script/
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=/home/gepetto/graspit_install
make -j16
make install

env_dir=/home/gepetto/graspit_install
export LD_LIBRARY_PATH=${env_dir}/lib:${LD_LIBRARY_PATH}
export PATH=${env_dir}/bin:${PATH}
export CMAKE_PREFIX_PATH=${env_dir}:${CMAKE_PREFIX_PATH}
export GRASPIT=${env_dir}/graspit_data
export GRASPIT_PLUGIN_DIR=${env_dir}/lib/plugins
cd ..
mkdir $GRASPIT
cp -r data/* "$GRASPIT"
```

## tips
When hpp doesn't find a trajectory, looking at what the scene looks like for him in gepetto-gui is helpful, you can do that in the xterm terminal with
`v = o.hpp_client.vf.createViewer()`
`v = (o.hpp_client.q_init)`
you may find transformation or vision issues this way.
