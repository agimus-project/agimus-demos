AGIMUS demo 05 pick and place
-----------------------------

The purpose of this demo is to use the AGIMUS architecture to unbox some objects.
Expected behaviors:
    - The robot goes over a box of object.
    - The robot start approaching one object.
    - The robot goes to grasp an object.
    - The robot move the object above a desired location.
    - The robot drops the object.

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
reset && source install/setup.bash && ros2 launch agimus_demo_05_pick_and_place bringup.launch.py use_gazebo:=true
```
After the xterm terminal is opened, type there `o.pick_and_place('obj_26')`.

## Start the demo on hardware using the Panda robot.
```bash
ros2 launch agimus_demo_05_pick_and_place bringup_hw.launch.py robot_ip:=<fci-ip>
ros2 launch agimus_demo_05_pick_and_place bringup.launch.py robot_ip:=192.168.102.11 aux_computer_ip:=192.168.102.21 aux_computer_user:=ros
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
